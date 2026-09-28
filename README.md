# fil

`fil` runs STM32G4/Cortex-M4F ELF firmware on one board or a deterministic
multi-board CAN network. It models instruction execution and peripheral timing,
but is not cycle-accurate hardware.

## Build and run

Requires CMake 3.20+, a C++20 compiler, and GoogleTest for tests.

```bash
make
make test
./build/fil --help
./build/fil run configs/boards/g4_testing.json --duration-ms 100 --strict-mmio
./build/fil run-network configs/networks/per_vehicle.json --duration-ms 1000 --quantum 1024 --strict-mmio
```

The checked-in board/network configurations reference firmware in a sibling PER
checkout. `make benchmark` instead generates synthetic fixtures and requires no
PER firmware. Run `make help` for build, sanitizer, PGO, and CLI shortcuts.

## Model and interfaces

Each board loads an ELF image into a configured STM32G474 memory map, executes
Thumb instructions with Cortex-M4F exceptions, and advances clocked peripheral
events. A network schedules boards in deterministic order and connects their
FDCAN controllers. Unsupported instructions fail explicitly; not all hardware
and timing effects are modeled. See [Architecture](docs/architecture.md),
[instruction coverage](docs/thumb_instruction_coverage.md), and
[peripheral coverage](docs/stm32g4_peripheral_coverage.md) for the support boundary.

`run` executes one board; `run-network` executes a configured network;
`watch-network` monitors a live network and accepts CAN/ADC/GPIO input on stdin.
`--trace FILE` writes JSONL events; `--trace-instr` records individual instructions.
`inspect-config`, `inspect-elf`, and `disasm-window` inspect inputs.
`compare-stlink` requires an attached ST-Link and controls/resets the physical
target; flashing requires explicit `--flash`. See [Configuration](docs/configuration.md),
[Live monitoring](docs/watch_network.md), [Tracing](docs/tracing.md), and
[Hardware comparison](docs/hardware_comparison.md) for contracts and safety.

## Performance

The default run preserves modeled instruction, event, and peripheral behavior.
Decoded-instruction and memory caches, bounded scheduler bursts, event ownership,
and proven idle-loop batching reduce host overhead. Release builds use IPO when
supported; optional PGO trains on a representative workload.

`--adc-decimation N` (1–1024, default 1) is an **opt-in fidelity tradeoff** for
continuous ADC scans: only one scan in N produces samples, DMA, and interrupts.
It can accelerate ADC-heavy workloads but changes firmware-visible behavior.
Do not use it for fidelity comparisons.

See [Performance](docs/performance.md) for the mechanisms, correctness boundaries,
and reproducible benchmark procedure. Turn off `--trace-instr` for throughput
measurements; instruction tracing intentionally disables batching.
