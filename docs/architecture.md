# Architecture

`fil` separates generic ARM execution from STM32G4 devices and board-specific inputs. Firmware behavior is derived from the ELF, target configuration, architectural state, and modeled register accesses; the emulator does not dispatch on application symbols or source names.

## Execution path

```text
CLI
 |-- strict JSON config ---- MCU / board / network values
 |-- ELF32 ARM loader ------ segments, vectors, symbols, ABI attributes
 |
 +-- Board
     |-- MemoryBus --------- flash, SRAM, CCM SRAM, aliases, MMIO windows
     |-- CortexM4 ---------- Thumb fetch, decode, execute, CPU state
     |-- SystemControl ----- SysTick, NVIC, SCB, DWT, CoreDebug/FPU control
     |-- ExceptionController -- stacking, vectoring, EXC_RETURN
     |-- Stm32G4 ----------- routed register-level peripherals
     |-- EventLoop --------- deterministic simulated-time callbacks
     +-- TraceRecorder ----- stable ordered JSONL records

World
 |-- shared EventLoop and TraceRecorder
 |-- fixed-order Board dispatch
 +-- named VirtualCanBus instances connecting FDCAN nodes
```

The CLI is a thin adapter around library APIs. Lower layers return typed results and structured bus faults; they do not print output or choose process exit codes.

## Loading and memory

The ELF loader accepts ELF32 little-endian ARM executable images and keeps load addresses separate from runtime virtual addresses. Board startup validates a 128-byte-aligned vector base, an aligned initial MSP inside writable memory, a Thumb reset vector, and an executable reset target.

The STM32G474 map currently contains:

- executable flash at the configured flash address and a boot alias at `0x00000000`;
- SRAM and CCM SRAM from the MCU config;
- a read-only system-memory window at `0x1fff0000`;
- the STM32 peripheral aperture at `0x40000000..0x5fffffff`;
- the Cortex-M system window at `0xe0000000..0xe00fffff`.

The memory bus enforces mapped permissions and records access type and originating PC in faults. The peripheral aperture is subdivided by an MMIO router. Lenient mode returns zero for addresses outside routed blocks and aggregates those addresses; `--strict-mmio` turns the same accesses into faults.

## CPU and exceptions

`CortexM4` owns integer registers, xPSR and IT state, MSP/PSP and mask registers, and 32 single-precision register slots with D0-D15 memory-transfer aliases. Each step fetches one 16- or 32-bit Thumb instruction, decodes it through non-overlapping mask/value tables, executes it, and returns a stable diagnostic snapshot.

The board loop consumes synchronous SVC and EXC_RETURN markers from the CPU, then checks pending SysTick, PendSV, and enabled NVIC interrupts. `ExceptionController` stacks and restores basic or extended floating-point frames, selects MSP or PSP, vectors through VTOR, and supports nested active exceptions. Unsupported instruction encodings and invalid architectural state stop with diagnostics rather than silently continuing.

The exact implemented instruction subset is documented in [Thumb instruction coverage](thumb_instruction_coverage.md).

## Peripherals and external devices

`Stm32G4` owns stable register-device instances and routes their real STM32G474 base addresses. Modeled side effects cover startup clocks and flash state, GPIO, basic serial data paths, timers, ADC, DMA, watchdog timers, and three FDCAN controllers with shared message RAM. Device callbacks pend NVIC exceptions through `SystemControl`.

Board configuration supplies deterministic external inputs such as GPIO levels,
scripted USART RX, binary USART TX logs, ADC constants or simulated-time sine waves,
and SPI zero/echo devices. ADC conversion completion is an event-loop callback, and
the input provider is sampled at that deterministic completion timestamp.
`VirtualCanBus` validates classic CAN and CAN-FD frames and broadcasts synchronously
in attachment order. CLI CAN injections use the same shared event loop. The detailed
semantic and simplification boundary is in [STM32G4 peripheral coverage](stm32g4_peripheral_coverage.md).

## Time, scheduling, and traces

The interpreter currently charges one target cycle per executed instruction. A board converts cycles to nanoseconds using the RCC system-clock estimate, advances SysTick/DWT, then runs due events. Events with the same timestamp execute in insertion order and have a bounded same-time callback count. The decoded-instruction, address-region, sparse-NVIC, and allocation-free stepping optimizations are described in [Performance](performance.md).

`World` loads boards in network-config order and gives each board an independent virtual CPU timeline. Boards ready at the same timestamp start in configuration order; the shared event loop then advances to the earliest instruction-completion or external-event frontier. The instruction quantum is a same-time fairness cap, not serialized simulated time. All boards share one event loop and one trace recorder, so board dispatch, CAN delivery, and equal-time events remain reproducible without host threads or wall-clock time.

Every scheduled callback carries an inherited event owner: one board lane or the shared domain. Peripheral callbacks created while a board executes remain local through nested scheduling, while FDCAN delivery and externally injected work are explicitly shared. Event execution reports the affected owner mask, allowing local callbacks to invalidate only their lane's lookahead proof. A global timestamp/sequence heap preserves deterministic dispatch, while per-owner queues expose board-local and shared horizons without scanning unrelated callbacks. Global and owner queues reference the same live event, so a worker can commit one lane's callbacks exactly once while the global queue later skips those retired entries. Each owner also has a monotonic local clock; ordinary single-threaded execution advances all clocks together, while worker execution can advance one owner independently. MMIO devices classify accesses as board-local or shared through nested routers. In worker mode, shared accesses return a side-effect-free synchronization boundary before device dispatch; the CPU restores its pre-instruction state so the coordinator can restart that instruction exactly once in deterministic board order. FDCAN control blocks are shared, while message RAM and ordinary MCU peripherals remain board-local. `Board::runWorkerSlice()` advances one owner clock, services only that owner's callbacks, settles interrupts at every instruction boundary, and retains exact-state loop batching up to its local event or coordinator deadline. It returns before shared MMIO without charging an instruction or cycle. This provenance, queue ownership, restartable access boundary, and independently testable lane runner form the synchronization contract for worker threads without changing timestamp or insertion-order semantics.

Peripheral trace sources are qualified as `board.device` (for example,
`dashboard.FDCAN1`) so identical MCU instances remain unambiguous in a shared
world trace. Virtual CAN fabric records use `bus/node` sources. CAN records expose
the encoded data-length code as `dlc` and the decoded payload byte count as
`length`.

When an exact architectural state repeats through a side-effect-free idle loop, the
runner may batch identical iterations up to the next observable time, event,
interrupt, or budget boundary. Instruction tracing and explicit spin diagnosis
disable batching. This is a firmware-agnostic throughput optimization; it does not
recognize application symbols or task names. See [Performance](performance.md) for
the proof conditions, opt-outs, and benchmark method.

Trace records have a simulated timestamp and monotonic insertion sequence. Instruction tracing is optional because it is much larger than device and lifecycle tracing.

## Intentional fidelity boundary

The emulator is instruction-level, not pipeline- or bus-cycle-accurate. Peripheral clocks, electrical behavior, analog settling, CAN arbitration timing, debug transport, and many register corner cases are simplified. One instruction per cycle also means simulated time is suitable for deterministic regression tests, not performance prediction.

Required tests use synthetic firmware and direct unit fixtures. External PER firmware is an acceptance input, not a source of special cases: no application function name, task name, CAN identifier, or source path is recognized by the emulator.

The hermetic `startup_runtime.elf` integration fixture verifies flash-to-SRAM `.data`
copying, `.bss` zeroing, entry to `main`, and a deterministic `BKPT` stop. Separately,
the current external-firmware checks statically scan all seven configured ELFs, run
each board for 10 ms with zero top-level unknown MMIO addresses, run the configured
six-board CAN world, and run `g4_testing` into FreeRTOS scheduling through one
simulated second. These checks establish compatibility with those builds, not full
ARM or STM32 conformance.
