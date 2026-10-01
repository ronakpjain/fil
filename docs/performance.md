# Performance

## Measuring throughput

Throughput is simulated seconds divided by host wall seconds. Compare only runs that reach the simulated-time deadline, not the instruction limit.

Keep firmware, build, PGO profile, host power mode, tracing, ADC decimation, stdin conditions, and CLI options fixed. Alternate run order and compare median wall times. Include initialization and JIT compilation.

The timing model includes instruction costs, branch penalties, and flash fetch stalls; it is not cycle-accurate silicon timing.

## Recommended defaults

Use Release with supported IPO, interpreter execution, proven-loop batching, and
ADC decimation 1 as the conservative baseline. Leave JIT and speculative
schedulers off until matched real-firmware measurements justify them. Prefer
strict MMIO for validation, although compatibility-oriented CLI parsing defaults
to lenient MMIO. Keep instruction tracing and spin detection off for throughput;
keep wall pacing on for interactive monitoring. See [Options](options.md) for
all runtime/build flags, actual defaults, and recommended command presets.

## Execution optimizations

| Mechanism | Behavior |
| --- | --- |
| Decode and memory caches | Direct-mapped decoded instructions and cached backed-memory access reduce decoding/mapping work while retaining permission checks and executable-write generation invalidation. Cache index folding reduces collisions between distant firmware addresses. |
| CPU hot paths | Specialized integer handlers, prepared block metadata, and guarded backed-memory multi-word paths avoid repeated generic dispatch. Unsupported, faulting, or MMIO work retains ordinary execution. |
| Event-aware scheduling | Bounds execution by events, interrupts, instruction budgets, and simulated-time deadlines. |
| Loop batching | Batches proven repeated loops; unproven paths execute normally. Disable with `--no-loop-batching`. `--detect-spin` requests loop detection; `--trace-instr` retains instruction-level execution. |
| Cached JIT handlers | `--jit` opts into specialized handlers and prepared blocks, with fallback for unsupported or unsafe operations. |
| Native LLVM JIT | Emits host machine code for a conservative integer subset when LLVM support is available. Memory/MMIO, system, and floating-point operations retain existing paths. Lazy initialization, hotness gates, and a 32-kernel LRU with admission hysteresis bound retained kernels and reduce startup/churn costs; they do not guarantee profitability. See [Native LLVM JIT](llvm-jit.md). |
| Exact network fast path | With loop batching, tracing, spin detection, and speculative modes disabled, up to 64 lanes use compact single-instruction dispatch without loop-proof state. |
| Transactional lane slices | Opt-in reversible parallel epochs commit only admissible work; observation-sensitive work falls back. Requires multiple boards without tracing or spin detection for worker execution. |
| Deferred pure / RAM prefixes | Opt-in delayed evaluation and reversible private-RAM spans reduce eager dispatch where boundary proofs permit. These are experiments, not defaults or native RAM JIT kernels. |
| Build optimization | Release, supported IPO/LTO, and representative optional Clang PGO reduce host overhead without changing modeled ADC fidelity. |

The interpreter is the default. JIT execution is experimental and does not guarantee a speedup. Synchronized network bursts may use different per-lane instruction counts only when every lane reaches the same exact simulated-time frontier. Multi-instruction prefixes stop before memory/MMIO, and event, deadline, and SysTick boundaries gate admission; mismatched work falls back to exact single-instruction dispatch. General asynchronous network dispatch also remains single-instruction: another lane may schedule a shared event that was not visible when a block's horizon was checked.

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

The target cleans old PGO outputs, builds, trains, merges profiles, rebuilds with the profile, and tests. Default training uses the six-board PER configuration with JIT enabled and ADC decimation 1 for 1,000 ms with a 50 million instruction limit.

- Override training with `PGO_TRAIN_ARGS='run-network ...'`.
- Use matching `PGO_CXX` and `LLVM_PROFDATA` versions.
- Retrain after material code, firmware, or toolchain changes.
- Individual stages: `pgo-generate`, `pgo-train`, `pgo-merge`, `pgo-use`, `pgo-test`.

## Network comparison

The configuration requires firmware from the sibling PER checkout.

```bash
/usr/bin/time -p ./build-release/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 5000 --max-instructions 250000000 --quantum 1024 --strict-mmio --adc-decimation 1
/usr/bin/time -p ./build-release/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 5000 --max-instructions 250000000 --quantum 1024 --strict-mmio --jit --adc-decimation 1
```

Record wall time, stop reason, simulated time, instruction/cycle counts, and JIT diagnostics. Increase the instruction limit if necessary.

With `--jit`, `run` and `run-network` report per-board `native_jit` diagnostics. Positive `executions` and `instructions` confirm generated code ran, not that it was faster. Account for additional diagnostics when comparing summaries; do not discard guest-visible differences.

### Transactional scheduling

`run-network --transactional-slices` enables experimental reversible lane epochs.
Parallel worker execution is gated on multiple boards, disabled tracing, and
no spin detection. MMIO-free admissible epochs commit; unsafe work rolls back
and resumes exact dispatch. Event ownership and observation ordering remain
correctness constraints, not overhead that can simply be removed. The
`transactional_attempts`, `transactional_commits`, and
`transactional_instructions` summary counters show use, not speedup. Leave it
off by default; thread setup, snapshots, failed attempts, and synchronization
can outweigh useful work.

### Exact and deferred scheduling

`--no-loop-batching` selects a compact exact single-instruction network scheduler when instruction tracing, spin detection, and transactional/deferred execution are disabled. It removes loop-proof bookkeeping, not target instructions or peripheral effects. General lockstep bursts require a proven pure fixed-cost multi-instruction frontier; scalar timing predictions alone do not authorize a burst.

`run-network --jit --deferred-prefixes` is an experimental, default-off alternative. It cannot be combined with instruction tracing, spin detection, transactional slices, or RAM capsules. It leaves certified pure CPU prefixes unevaluated until their completion or an observation barrier. Scheduled events and synchronous CAN delivery materialize only the instructions the exact scheduler has already started. Event-phase and CPU-dispatch-phase barriers preserve different equal-timestamp ordering. Public mutable board/CPU APIs must not be used concurrently with a deferred run. The current pure-prefix coverage is limited; this option is not a claim of faster-than-realtime throughput. Current full-network validation still measures below realtime with ADC decimation 1; a faster-than-realtime result must be established on matching current firmware, not inferred from the available mechanisms.

`run-network --jit --no-loop-batching --ram-capsules` is a separate default-off experiment. It chains up to 64 cached fast-handler instructions across integer branches, permitting only reversible writes to non-executable private RAM. MMIO, FP/system operations, executable stores, and unsupported handlers end the span. Conservative cycle credit prevents crossing known event, SysTick, or deadline bounds; dynamic observation cuts restore RAM/integer state and replay only the instructions already started. CAN delivery and board-failure draining are observation barriers. CPU/RAM state must not be inspected or modified while a prefix is active except through scheduled callbacks, which materialize it first. Run options are snapshotted per invocation. This path uses cached JIT handlers, not LLVM-native RAM kernels, so native execution diagnostics can remain zero. It is still below realtime and is not promoted to the default scheduler.

A repeatable full-network benchmark includes startup, checks all six boards reach the deadline, and exits unsuccessfully when median throughput is not above realtime:

```bash
python3 tools/bench_full_network.py --binary build-pgo/fil --reps 3 \
  --no-loop-batching --check-trace
```

Use `--ram-capsules` instead of `--no-loop-batching` in the benchmark helper to measure that experiment; the helper supplies its required CLI scheduling flags automatically.

The optional trace check compares 1,000 ms against exact single-instruction execution with the same firmware and ADC decimation 1. Retrain PGO with the scheduling options being measured; an old profile is not a matched baseline.

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
