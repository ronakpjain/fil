# fil

`fil` is a deterministic, instruction-level STM32G4/Cortex-M4F firmware emulator. It loads ELF32 ARM firmware, builds an STM32G474 memory map, runs one board or a fixed-order multi-board CAN network, and emits stable diagnostics and JSON-lines traces.

The model is deliberately scoped for firmware development and regression testing. It is not a cycle-accurate replacement for an STM32G4, and unsupported instructions fail explicitly. See [Thumb instruction coverage](docs/thumb_instruction_coverage.md) and [STM32G4 peripheral coverage](docs/stm32g4_peripheral_coverage.md) for the exact boundary.

## Build and test

Prerequisites are CMake 3.20 or newer, a C++20 compiler, and a CMake-supported build tool such as Ninja.

```bash
cmake -S . -B build -G Ninja -DFIL_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
./build/fil --help
```

The required tests use the committed synthetic Cortex-M4F ELF fixtures. In particular,
`tests/fixtures/elf/startup_runtime.elf` copies `.data` from its flash load address,
zeros a pre-dirtied `.bss`, calls `main`, writes a sentinel, and stops at `BKPT`.
The ARM GNU toolchain is needed only to regenerate the fixtures, not to build or test
`fil`.

Run the same suite with AddressSanitizer and UndefinedBehaviorSanitizer:

```bash
cmake -S . -B build-sanitize -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DFIL_BUILD_TESTS=ON \
  -DFIL_ENABLE_ASAN=ON \
  -DFIL_ENABLE_UBSAN=ON
cmake --build build-sanitize
ctest --test-dir build-sanitize --output-on-failure
```

## Run firmware

Run a single configured board:

```bash
./build/fil run configs/boards/g4_testing.json
./build/fil run configs/boards/g4_testing.json \
  --duration-ms 10 \
  --trace /tmp/g4_testing.jsonl \
  --trace-instr
```

Watch firmware execute at wall-clock speed with semantic events printed as they
happen:

```bash
./build/fil watch configs/boards/g4_testing.json --duration-ms 10000
./build/fil watch-network configs/networks/per_vehicle.json \
  --duration-ms 10000 \
  --inject-can vehicle@500:0x123:01020304 \
  --live-filter can_tx --live-filter can_rx
```

`watch` and `watch-network` enable both `--realtime` pacing and `--live` event
output. They show CAN traffic, exceptions, peripheral activity, resets, and other
semantic trace records with simulated timestamps. Add `--trace-instr` only when
every instruction is useful; it is intentionally off by default. `--refresh-ms N`
controls the wall-clock synchronization interval (10 ms by default). The same
features can be composed independently with ordinary `run` commands via
`--realtime` or `--live`. Repeat `--live-filter TYPE` to select event types in
noisy firmware while preserving the complete trace collected by `--trace`.

Useful single-board controls include `--max-instructions`, `--stop-address`, `--stop-at-symbol`, `--strict-mmio`, `--lenient-mmio`, `--detect-spin`, and `--no-loop-batching`.
Reaching a target `BKPT` is a CLI error unless `--allow-breakpoint` is supplied;
use the flag only when the breakpoint is an intended success boundary.

Run the configured six-board vehicle CAN network with a deterministic same-time
fairness quantum:

```bash
./build/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 100 \
  --quantum 1024 \
  --inject-can vehicle@5:0x123:01020304 \
  --inject-can vehicle@20:0x456:aabb \
  --trace /tmp/per_vehicle.jsonl
```

`--inject-can BUS[@TIME_MS]:ID:HEXDATA` may be repeated. Omitting `@TIME_MS`
schedules the frame at time zero; identifiers above `0x7ff` are treated as extended.
Interactive clients may add `--control-stdin` and write one
`BUS:ID:HEXDATA` command per line. Each valid command is injected at the next safe
shared simulation frontier, allowing a long-running `watch-network` process to be
controlled bidirectionally.

The same private control pipe accepts live 12-bit ADC overrides:

```text
adc BOARD ADCx CHANNEL VALUE
```

For example, `adc dashboard ADC1 3 2048` sets dashboard ADC1 channel 3 to
mid-scale at the next shared frontier. Boards and ADC instances must exist,
channels are 0 through 19, and values are 0 through 4095. Applied changes emit an
`adc_input` trace record.

GPIO inputs can be driven or released through the same pipe:

```text
gpio BOARD GPIOx PIN 0|1|release
```

For example, `gpio dashboard GPIOA 3 1` drives PA3 high, while
`gpio dashboard GPIOA 3 release` returns it to the firmware-controlled input.
Applied changes emit `gpio_input` records; firmware output changes emit
`gpio_output` records.

The board and network configs in `configs/` reference firmware under the sibling `PER` checkout. Those external ELFs must exist at the configured paths for these commands; they are optional compatibility inputs and are not embedded into emulator behavior.

## Current real-firmware acceptance

The current compatibility checks over the seven configured PER firmware ELFs report:

- a reproducible 55,068-instruction objdump/decoder scan with zero unsupported or
  address/width mismatches, plus a targeted mnemonic/operand audit of 2,268
  ADDW/scalar-VFP instances;
- successful 10 ms runs for all seven board configs with zero top-level unknown MMIO
  addresses;
- a successful run of the configured six-board CAN network; and
- `g4_testing` reaching FreeRTOS scheduling and continuing through the one-second
  simulated-time boundary.

With the configured `g4_testing.elf`, the one-second acceptance run completes at the simulated-time boundary with:

```text
stop: time-budget
instructions: 16000000
cycles: 16000000
time_ns: 1000000000
unknown_mmio_addresses: 0
```

The count follows the current one-cycle-per-instruction timing model at the 16 MHz reset clock. Zero unknown MMIO means the top-level peripheral router saw no access outside a routed block; it does not imply that every register in those blocks has hardware-complete semantics.

On the development host, a Clang PGO build trained on the strict-MMIO six-board
command ran the corrected one-second workload in 0.73 s, 0.73 s, and 0.74 s while
preserving the exact 96,000,000 logical instruction/cycle total and per-board
results—about 1.37x real time with clock-timed ADC sequences and per-rank DMA
enabled. The portable Release+IPO build is about 1.15x real time. On battery with
macOS Low Power Mode enabled, the latest separately measured medians are 1.57 s
portable and 1.33 s with freshly trained PGO. See
[Performance](docs/performance.md) for reproduction commands, the
unbatched comparison, correctness guards, and explanations of every speedup.

## Inspect inputs

Inspect an ELF without running it:

```bash
./build/fil inspect-elf path/to/firmware.elf
```

Decode a bounded instruction window at a target address:

```bash
./build/fil disasm-window path/to/firmware.elf \
  --addr 0x08001234 \
  --count 32
```

Validate and normalize MCU, board, or network configuration:

```bash
./build/fil inspect-config configs/mcus/stm32g474retx.json
./build/fil inspect-config configs/boards/g4_testing.json
./build/fil inspect-config configs/networks/per_vehicle.json
```

Configuration is strict: duplicate and unknown keys are rejected, and referenced paths resolve relative to the file containing them. See [Configuration](docs/configuration.md).

## Documentation

- [Architecture](docs/architecture.md)
- [JSONL trace contract](docs/tracing.md)
- [Performance](docs/performance.md)
- [Configuration](docs/configuration.md)
- [Thumb instruction coverage](docs/thumb_instruction_coverage.md)
- [STM32G4 peripheral coverage](docs/stm32g4_peripheral_coverage.md)
- [Implementation roadmap](docs/STM32G4_EMULATOR_IMPLEMENTATION_PLAN.md)

Generate the Doxygen site when Doxygen is installed:

```bash
cmake -S . -B build -G Ninja -DFIL_BUILD_DOCS=ON
cmake --build build --target docs
```
