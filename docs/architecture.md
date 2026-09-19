# Architecture

`fil` separates generic ARM execution from STM32G4 devices and board-specific
inputs. Behavior comes from ELF contents, configuration, architectural state, and
modeled register accesses. The emulator does not dispatch on application symbols,
task names, or source paths.

## Components

```text
CLI
 |-- strict JSON config ---- MCU, board, and network values
 |-- ELF32 ARM loader ------ segments, vectors, symbols, ABI attributes
 |
 +-- Board
     |-- MemoryBus --------- flash, SRAM, CCM SRAM, aliases, MMIO windows
     |-- CortexM4 ---------- Thumb fetch, decode, execute, CPU state
     |-- SystemControl ----- SysTick, NVIC, SCB, DWT, CoreDebug/FPU control
     |-- ExceptionController -- stacking, vectoring, EXC_RETURN
     |-- Stm32G4 ----------- routed register-level peripherals
     |-- EventLoop --------- deterministic simulated-time callbacks
     +-- TraceRecorder ----- ordered JSONL records

World
 |-- shared EventLoop and TraceRecorder
 |-- fixed-order Board scheduling
 +-- named VirtualCanBus instances connecting FDCAN nodes
```

The CLI adapts command-line input to library APIs. Lower layers return typed results
and structured faults; they do not print output or choose process exit codes.

## Loading and memory

The ELF loader accepts ELF32 little-endian ARM executables and keeps physical load
addresses separate from runtime virtual addresses. Startup validates a 128-byte
aligned vector base, an aligned initial MSP in writable memory, a Thumb reset vector,
and an executable reset target.

The STM32G474 map contains:

- configured flash and a boot alias at `0x00000000`;
- SRAM and CCM SRAM from the MCU config;
- read-only system memory at `0x1fff0000`;
- the STM32 peripheral aperture at `0x40000000..0x5fffffff`; and
- the Cortex-M system window at `0xe0000000..0xe00fffff`.

`MemoryBus` enforces range and permission checks and includes access type and PC in
faults. MMIO accesses retain their original width and are routed once. Lenient mode
returns zero and aggregates addresses outside known peripheral blocks;
`--strict-mmio` turns those accesses into faults.

See [ELF loading](elf_loading.md) and [Memory system](memory_system.md).

## CPU and exceptions

`CortexM4` owns integer registers, xPSR/IT state, MSP/PSP, mask registers, and 32
single-precision FP slots. Each step fetches one 16- or 32-bit Thumb instruction,
decodes it through non-overlapping mask/value tables, executes it, and returns a
stable result.

At each instruction boundary, `Board` consumes SVC and EXC_RETURN markers and then
checks SysTick, PendSV, and enabled NVIC interrupts. `ExceptionController` stacks or
restores basic and extended FP frames, selects MSP or PSP, vectors through VTOR, and
tracks nested exceptions. Unsupported encodings and invalid state stop with
structured diagnostics.

See [Thumb instruction coverage](thumb_instruction_coverage.md).

## Peripherals and external devices

`Stm32G4` owns stable register-device instances at STM32G474 addresses. The model
covers startup clocks and flash state, GPIO, serial data paths, timers, ADC, DMA,
watchdogs, and three FDCAN controllers with message RAM. Peripheral callbacks drive
level-sensitive NVIC inputs through `SystemControl`; shared IRQ sources are ORed.
Exception return re-pends a still-asserted source, while resuming a preempted handler
preserves any separately latched pending interrupt.

Board configuration supplies deterministic GPIO levels, USART input/output, ADC
sources, and SPI devices. ADC inputs are sampled at simulated conversion-completion
time. `VirtualCanBus` validates classic CAN and CAN-FD frames and delivers them in
attachment order. CLI CAN injection uses the same event loop.

See [STM32G4 peripheral coverage](stm32g4_peripheral_coverage.md) for modeled
semantics and intentional gaps.

## Time and deterministic scheduling

Each instruction currently costs one target cycle. A board converts cycles to
nanoseconds from the RCC clock estimate, advances SysTick/DWT, and runs due events.
Same-time callbacks execute in insertion order and have a bounded callback count.

`World` gives every board an independent virtual CPU timeline. Boards ready at the
same timestamp start in configuration order, and the event loop advances to the
earliest instruction completion or callback. The instruction quantum limits
same-time fairness; it does not serialize board time. All boards share one trace
recorder and event loop, so dispatch, CAN delivery, and equal-time events are
reproducible without wall-clock input.

### Event ownership and worker slices

Every callback belongs to one board lane or to the shared domain. Ownership is
inherited by nested scheduling. Per-owner queues and clocks provide local horizons
without changing the global timestamp/sequence order.

This ownership supports conservative independent execution:

- board-local callbacks invalidate only their lane's lookahead;
- shared callbacks invalidate every lane;
- shared MMIO yields before device dispatch so the coordinator can restart the
  instruction once in deterministic order;
- `Board::runWorkerSlice()` services only one owner's callbacks and stops at shared
  synchronization; and
- a transaction captures CPU, system, RAM journal, event-clock, fractional-time,
  and loop-proof state for rollback.

Persistent lane workers may run reversible slices concurrently. An epoch commits
only when every board reaches the same MMIO-free boundary before the next event;
otherwise all checkpoints are restored and exact frontier scheduling resumes.
Tracing disables transactional epochs because speculative trace records cannot be
retracted. The path remains opt-in through `--transactional-slices`.

### Loop batching

When a side-effect-free polling loop returns to the exact same architectural and
memory state, the scheduler may account for repeated iterations up to the next
event, interrupt, deadline, budget, or scheduling frontier. Instruction tracing and
spin diagnosis disable batching. The proof and benchmark controls are documented in
[Performance](performance.md).

## Traces

Every trace record has a simulated timestamp and monotonic insertion sequence.
Peripheral sources use `board.device`; virtual CAN sources use `bus/node`.
Instruction tracing is optional because it is much larger than lifecycle and device
tracing. See the [JSONL trace contract](tracing.md).

## Fidelity boundary

The emulator is instruction-level, not pipeline- or bus-cycle-accurate. Peripheral
clocks, analog/electrical behavior, CAN arbitration timing, debug transport, and many
register corners are simplified. Simulated time is suitable for deterministic
regression tests, not hardware performance prediction.

Required tests use synthetic firmware and direct fixtures. External PER firmware is
an acceptance input, never a source of application-specific behavior. The optional
[hardware comparison](hardware_comparison.md) validates deterministic architectural
state through OpenOCD without treating physical timing as an emulator contract. See
[Testing](testing.md) for the validation matrix.
