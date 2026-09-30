# Performance

## Measuring throughput

Throughput is simulated seconds divided by host wall seconds. Compare only runs that reach the simulated-time deadline, not the instruction limit.

Keep firmware, build, PGO profile, host power mode, tracing, ADC decimation, stdin conditions, and CLI options fixed. Alternate run order and compare median wall times. Include initialization and JIT compilation.

The timing model includes instruction costs, branch penalties, and flash fetch stalls; it is not cycle-accurate silicon timing.

## Execution optimizations

| Mechanism | Behavior |
| --- | --- |
| Decode and memory caches | Reduce decoding and mapping work while retaining permission checks and executable-write invalidation. |
| Event-aware scheduling | Bounds execution by events, interrupts, instruction budgets, and simulated-time deadlines. |
| Loop batching | Batches proven repeated loops; unproven paths execute normally. Disable with `--no-loop-batching`. `--detect-spin` requests loop detection; `--trace-instr` retains instruction-level execution. |
| Cached JIT handlers | `--jit` opts into specialized handlers and prepared blocks, with fallback for unsupported or unsafe operations. |
| Native LLVM JIT | Emits host machine code for a conservative integer subset when LLVM support is available. Memory/MMIO, system, and floating-point operations retain existing paths. See [Native LLVM JIT](llvm-jit.md). |

The interpreter is the default. JIT execution is experimental and does not guarantee a speedup. General network dispatch retains single-instruction boundaries; larger batches require scheduler boundary proofs.

## Build

```bash
make BUILD_DIR=build-release BUILD_TYPE=Release IPO=ON test
```

- IPO/LTO is enabled when supported. Debug and sanitizer builds are not throughput baselines.
- CMake attempts native LLVM discovery by default; LLVM 18 or newer and a 64-bit host are required.
- Without LLVM, `--jit` uses cached handlers only. Check configure output for backend availability.
- CMake overrides: `FIL_ENABLE_LLVM_JIT=OFF` disables native support; `LLVM_DIR` selects an installation; `FIL_LLVM_COMPONENT_LINKING=ON` selects component-library linking.

### Profile-guided optimization

```bash
make pgo
```

The target cleans old PGO outputs, builds, trains, merges profiles, rebuilds with the profile, and tests. Default training uses the six-board PER configuration for 1,000 ms with a 50 million instruction limit.

- Override training with `PGO_TRAIN_ARGS='run-network ...'`.
- Use matching `PGO_CXX` and `LLVM_PROFDATA` versions.
- Retrain after material code, firmware, or toolchain changes.
- Individual stages: `pgo-generate`, `pgo-train`, `pgo-merge`, `pgo-use`, `pgo-test`.

## Network comparison

The configuration requires firmware from the sibling PER checkout.

```bash
/usr/bin/time -p ./build-release/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 5000 --max-instructions 250000000 --quantum 1024 --strict-mmio
/usr/bin/time -p ./build-release/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 5000 --max-instructions 250000000 --quantum 1024 --strict-mmio --jit
```

Record wall time, stop reason, simulated time, instruction/cycle counts, and JIT diagnostics. Increase the instruction limit if necessary.

With `--jit`, `run` and `run-network` report per-board `native_jit` diagnostics. Positive `executions` and `instructions` confirm generated code ran, not that it was faster. Account for additional diagnostics when comparing summaries; do not discard guest-visible differences.

## ADC decimation

`--adc-decimation N` keeps one of every N continuous-mode scans (1–1024; default 1). Skipped scans omit normal register, DMA, interrupt, and sample-recording effects. Single-shot conversions are not skipped.

**Decimation changes firmware-visible behavior.** Keep N fixed when comparing execution optimizations.

## Validation

```bash
make BUILD_DIR=build-release BUILD_TYPE=Release test
make BUILD_DIR=build-release BUILD_TYPE=Release benchmark BENCH_REPS=3
```

Differential tests cover cached execution, MMIO restart, code invalidation, execution caps, timing, events, and scheduling. Native-specific tests cover emitted arithmetic/flags, branches, prefix limits, preserved state, fetch faults, and invalidation.

Tests provide regression evidence, not proof for every firmware. Synthetic smoke benchmarks do not establish six-board network throughput; wall-time measurements are not correctness tests.
