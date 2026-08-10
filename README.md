# fil

`fil` is a deterministic, instruction-level STM32G4/Cortex-M4F firmware emulator.
It loads ELF32 ARM firmware, models an STM32G474 memory/peripheral map, runs one board
or a fixed-order multi-board CAN network, and emits stable diagnostics and JSONL
traces.

The model is intended for firmware development and regression tests. It is not
cycle-accurate, and unsupported instructions fail explicitly. See
[Thumb instruction coverage](docs/thumb_instruction_coverage.md) and
[STM32G4 peripheral coverage](docs/stm32g4_peripheral_coverage.md) for the exact
support boundary.

## Build and test

Requirements: CMake 3.20+, a C++20 compiler, and a CMake-supported build tool such
as Ninja. Test builds also require an installed GoogleTest CMake package.

```bash
cmake -S . -B build -G Ninja -DFIL_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
./build/fil --help
```

The required suite uses committed synthetic Cortex-M4F ELF fixtures. An ARM GNU
toolchain is needed only to regenerate those fixtures. See [Testing](docs/testing.md)
for sanitizer, coverage, and external-firmware checks.

## Run firmware

Run one configured board:

```bash
./build/fil run configs/boards/g4_testing.json \
  --duration-ms 10 \
  --strict-mmio \
  --trace /tmp/g4_testing.jsonl
```

Useful controls include `--max-instructions`, `--stop-address`,
`--stop-at-symbol`, `--strict-mmio`, `--lenient-mmio`, `--detect-spin`,
`--no-loop-batching`, and `--trace-instr`. A target `BKPT` is an error unless
`--allow-breakpoint` is supplied for an intentional breakpoint boundary.

Run the configured six-board vehicle network:

```bash
./build/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 100 \
  --quantum 1024 \
  --inject-can vehicle@5:0x123:01020304 \
  --inject-can vehicle@20:0x456:aabb \
  --trace /tmp/per_vehicle.jsonl
```

`--inject-can BUS[@TIME_MS]:ID:HEXDATA` may be repeated. Omitting `@TIME_MS`
schedules at time zero; identifiers above `0x7ff` are treated as extended.

The checked-in board configs reference ELF files in a sibling `PER` checkout. Those
files are optional acceptance inputs and are not embedded into emulator behavior.

## Performance

On the development host, the corrected one-second six-board workload has measured
medians of 0.87 s with portable Release+IPO and 0.73 s with workload-trained Clang
PGO. Both produce the same 96,000,000 logical instruction/cycle total and exact
per-board results. Host, compiler, power mode, and firmware affect these numbers.
See [Performance](docs/performance.md) for the benchmark method, controls, results,
and correctness boundaries of each optimization.

## Inspect inputs

Inspect an ELF or decode a bounded instruction window:

```bash
./build/fil inspect-elf path/to/firmware.elf
./build/fil disasm-window path/to/firmware.elf \
  --addr 0x08001234 \
  --count 32
```

Validate and normalize configuration:

```bash
./build/fil inspect-config configs/mcus/stm32g474retx.json
./build/fil inspect-config configs/boards/g4_testing.json
./build/fil inspect-config configs/networks/per_vehicle.json
```

Configuration rejects duplicate and unknown keys. Referenced paths resolve relative
to the file that contains them. See [Configuration](docs/configuration.md).

## Compare with STM32G4 hardware

With OpenOCD and an ST-Link attached, compare deterministic register and RAM state:

```bash
./build/fil compare-stlink \
  tests/fixtures/config/hardware_compare_board.json \
  --flash --memory 0x20000000:16
```

Flashing is opt-in; every comparison resets and controls the target. See
[Hardware comparison](docs/hardware_comparison.md) for safety, stop boundaries,
JSON artifacts, and comparison scope.

## Documentation

- [Architecture](docs/architecture.md)
- [Configuration](docs/configuration.md)
- [ELF loading](docs/elf_loading.md)
- [Memory system](docs/memory_system.md)
- [Hardware comparison](docs/hardware_comparison.md)
- [JSONL trace contract](docs/tracing.md)
- [Performance](docs/performance.md)
- [Testing](docs/testing.md)
- [Thumb instruction coverage](docs/thumb_instruction_coverage.md)
- [STM32G4 peripheral coverage](docs/stm32g4_peripheral_coverage.md)
- [Roadmap](docs/roadmap.md)

Generate the Doxygen site when Doxygen is installed:

```bash
cmake -S . -B build -G Ninja -DFIL_BUILD_DOCS=ON
cmake --build build --target docs
```
