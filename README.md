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

The checked-in board/network configurations reference external firmware images.
`make benchmark` instead generates synthetic fixtures and requires no external
firmware. Run `make help` for build, sanitizer, PGO, and CLI shortcuts.
See [Options and recommended defaults](docs/options.md) for the complete command
and build reference, including experimental modes and Make-wrapper differences.

## Model and interfaces

Each board loads an ELF image into a configured STM32G474 memory map, executes
Thumb instructions with Cortex-M4F exceptions, and advances clocked peripheral
events. A network schedules boards in deterministic order and connects their
FDCAN controllers. Unsupported instructions fail explicitly; not all hardware
and timing effects are modeled. See [Architecture](docs/architecture.md),
[instruction coverage](docs/thumb_instruction_coverage.md), and
[peripheral coverage](docs/stm32g4_peripheral_coverage.md) for the support boundary.

`run` executes one board; `run-network` executes a configured network as a
human-readable batch job; `watch-network` is the human-facing live monitor with
text traces and CAN/ADC/GPIO stdin controls. `serve-network` requires
`--transport stdio` and is the interprocess counterpart: stdin/stdout carry a
documented, versioned binary protocol and stdout contains no human-readable output. See
[Live monitoring](docs/watch_network.md) and [the stdio protocol](docs/serve_network.md).
`--trace FILE` writes JSONL events for `run` and `run-network`; `--trace-instr`
records individual instructions. `inspect-config`, `inspect-elf`, and
`disasm-window` inspect inputs.
`compare-stlink` requires an attached ST-Link and controls/resets the physical
target; flashing requires explicit `--flash`. See [Configuration](docs/configuration.md),
[Live monitoring](docs/watch_network.md), [the binary network protocol](docs/serve_network.md),
[Tracing](docs/tracing.md), and [Hardware comparison](docs/hardware_comparison.md)
for contracts and safety.

## Performance

Our recommended baseline is Release + supported IPO, the default JIT with
network RAM capsules, and ADC decimation 1. Plain `run-network` uses the
validated fast path without enabling flags. Use strict MMIO for validation;
the compatibility CLI default is lenient. Transactional slices and deferred
prefixes remain experimental opt-ins. `--no-jit` selects the interpreter and
automatically disables RAM capsules.

The default run preserves modeled instruction, event, and peripheral behavior.
Decoded-instruction and memory caches, bounded scheduler bursts, guarded RAM
capsules on network commands, and proven-loop batching on single-board `run`
reduce host overhead. Network loop batching is off by default. Release builds
use IPO when supported; optional PGO trains on a representative workload.

ADC decimation 1 is the full-fidelity default. `--adc-decimation N` (1–1024)
is an **opt-in fidelity tradeoff** for continuous ADC scans: only one scan in N
produces samples, DMA, and interrupts.
It can accelerate ADC-heavy workloads but changes firmware-visible behavior.
Do not use it for fidelity comparisons.

See [Performance](docs/performance.md) for the mechanisms, correctness boundaries,
and reproducible benchmark procedure. Turn off `--trace-instr` for throughput
measurements; instruction tracing intentionally disables batching.
