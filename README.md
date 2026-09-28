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

Requirements: CMake 3.20+, a C++20 compiler, and a CMake-supported build tool.
The default build is optimized Release in `build/`; tests require an installed
GoogleTest CMake package.

```bash
make                 # configure and build
make test            # build, then run the hermetic CTest suite
make app-help        # show fil commands and options
```

Set `GENERATOR=Ninja` to choose Ninja explicitly, `BUILD_DIR=build-release` to use
another out-of-tree build, or `JOBS=8` to set build parallelism. `make debug`,
`make test-sanitize`, `make benchmark`, and `make clean` cover common workflows.
`make clean` removes only a root-level `build`/`build-*` directory whose CMake cache
belongs to this repo; it refuses source trees, symlinks, and other paths. `make clean-all`
removes the Makefile's designated out-of-tree build directories.
See `make help` for the full target list and [Testing](docs/testing.md) for
sanitizer, coverage, firmware-fixture, and external-firmware checks.

The required suite uses committed synthetic Cortex-M4F ELF fixtures. An ARM GNU
toolchain is needed only to regenerate those fixtures.

## Make shortcuts for fil

The Makefile exposes every CLI subcommand. For example:

```bash
make run RUN_CONFIG=configs/boards/g4_testing.json \
  RUN_ARGS='--duration-ms 10 --strict-mmio'
make run-network NETWORK_CONFIG=configs/networks/per_vehicle.json \
  NETWORK_ARGS='--duration-ms 1000 --quantum 1024'
make watch-network NETWORK_CONFIG=configs/networks/per_vehicle.json
make inspect-config CONFIG=configs/mcus/stm32g474retx.json
make inspect-elf ELF=tests/fixtures/elf/split_image.elf
make disasm-window ELF=tests/fixtures/elf/split_image.elf DISASM_ADDR=0x08000000
```

`make run` defaults to the committed synthetic hardware-comparison fixture; board
and network configs under `configs/` generally reference firmware in a sibling
PER checkout. The watch target is interactive and accepts control lines on stdin.
`make compare-stlink CONFIRM_STLINK=YES` explicitly opts into controlling/resetting
physical hardware; flashing remains opt-in via `COMPARE_ARGS=--flash`.
Use `make cli CLI_ARGS='--version'` for any other `fil` invocation. See
[Testing](docs/testing.md) for the external PER test setup.

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

Monitor a running network and inject CAN frames interactively:

```bash
./build/fil watch-network configs/networks/per_vehicle.json \
  --refresh-ms 10 \
  --live-filter can_tx \
  --live-filter can_rx \
  --control-stdin
```

Enter `BUS:ID:HEXDATA` lines on standard input, or enter `quit` or `exit` to stop.
The monitor prints selected events with simulated timestamps; `can_tx` is shown by
default, and repeatable `--live-filter TYPE` options replace that default. See
[Live network monitoring](docs/watch_network.md) for input syntax, output format,
options, and stopping behavior.

The checked-in board configs reference ELF files in a sibling `PER` checkout. Those
files are optional acceptance inputs and are not embedded into emulator behavior.

## Performance

Pacing follows the documented real-timing model: Cortex-M4 per-class pipeline
cycles plus STM32G4 flash wait states (`cycles >= instructions`, CPI reported).
On Apple M3 / Release+IPO, the six-board PER vehicle network (real firmware,
continuous ADC+DMA on every board) runs 1 s of simulated time in 2.57 s by
default, 1.24 s with `--adc-decimation 8`, and 0.93 s with `--adc-decimation
32` (1.08x realtime); single boards run well above realtime (dashboard: 19x
default, 48x decimated). The retired 1-CPI six-board figures (0.87 s Release,
0.73 s PGO for a 96M/96M total) are preserved as stale baselines in
docs/performance.md and are not comparable to real-timing runs. Host, compiler,
power mode, and firmware affect these numbers. See [Performance](docs/performance.md)
for the benchmark method, controls, results, and correctness boundaries of
each optimization, and [Real timing audit](docs/real_timing_audit.md) for the
silicon model.

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
- [Live network monitoring](docs/watch_network.md)
- [Performance](docs/performance.md)
- [Testing](docs/testing.md)
- [Thumb instruction coverage](docs/thumb_instruction_coverage.md)
- [STM32G4 peripheral coverage](docs/stm32g4_peripheral_coverage.md)
- [Roadmap](docs/roadmap.md)

Generate the Doxygen site when Doxygen is installed:

```bash
make docs
```

The equivalent direct CMake commands are `cmake -S . -B build -DFIL_BUILD_DOCS=ON`
then `cmake --build build --target docs`.
