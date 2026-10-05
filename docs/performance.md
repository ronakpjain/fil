# Performance

## Measuring throughput

Throughput is simulated seconds divided by host wall seconds. Compare only runs that reach the simulated-time deadline, not the instruction limit.

Keep firmware, build, PGO profile, host power mode, tracing, ADC decimation, stdin conditions, and CLI options fixed. Alternate run order and compare median wall times. Include initialization and JIT compilation.

The timing model includes instruction costs, branch penalties, and flash fetch stalls; it is not cycle-accurate silicon timing.

Profile the real network before changing execution paths. On macOS, run the
network unpaced with tracing off and collect `sample PID 5 1 -file profile.txt`
while it is running; on Linux, use an available sampling profiler such as
`perf`. Sampling identifies hot paths but perturbs wall time, so collect
throughput measurements separately with the benchmark helper.

## Recommended defaults

Use Release with supported IPO, automatic JIT, eligible guarded network RAM
capsules, and ADC decimation 1 as the baseline. Plain `run-network` uses its
validated fast path without enabling flags. Network loop batching is off;
single-board `run` batches proven loops automatically. Diagnostic modes such as
instruction tracing and spin detection retain safe correctness fallbacks and
may disable optimized paths. Experimental scheduler implementations remain
internal APIs, not CLI modes. Prefer strict MMIO for validation, although
compatibility-oriented CLI parsing defaults to lenient MMIO. Keep instruction
tracing and spin detection off for throughput; keep wall pacing on for
interactive monitoring. See [CLI options](options.md) for runtime controls and defaults;
see the [README](../README.md) and `make help` for building.

## Execution optimizations

| Mechanism | Behavior |
| --- | --- |
| Decode and memory caches | Direct-mapped decoded instructions and cached backed-memory access reduce decoding/mapping work while retaining permission checks and executable-write generation invalidation. Cache index folding reduces collisions between distant firmware addresses. |
| CPU hot paths | Specialized integer handlers, prepared block metadata, and guarded backed-memory multi-word paths avoid repeated generic dispatch. Unsupported, faulting, or MMIO work retains ordinary execution. |
| Pending-exception selection cache | Reuses exact exception selection for unchanged CPU masks and system-control generation. Selection-affecting mutations invalidate the cache; the cached takable-pending check avoids rescanning priorities without suppressing eligible exceptions. |
| Native compilation pipeline | Uses LLVM's O0 module pipeline to limit optimization-pass overhead for small integer kernels. This is a compile-time/runtime tradeoff, not evidence of an end-to-end firmware speedup. |
| Event-aware scheduling | Bounds execution by events, interrupts, instruction budgets, and simulated-time deadlines. Event insertion prunes retired owner-queue heads so global-only dispatch does not retain every completed ADC callback until shutdown; live event ordering and conversion effects are unchanged. |
| Loop batching | Single-board `run` automatically batches proven repeated loops; network commands do not. Unproven paths execute normally; tracing retains instruction-level execution. |
| Cached JIT handlers | JIT is automatic. Specialized handlers and prepared blocks fall back for unsupported or unsafe operations. |
| Native LLVM JIT | Emits host machine code for a conservative integer subset when LLVM support is available. Memory/MMIO, system, and floating-point operations retain existing paths. Lazy initialization, hotness gates, and a 32-kernel LRU with admission hysteresis bound retained kernels and reduce startup/churn costs; they do not guarantee profitability. See [Native LLVM JIT](llvm-jit.md). |
| Validated network fast path | Plain `run-network` uses compact single-instruction dispatch without loop-proof state. Networks of up to eight boards use a fixed-width timestamp reduction, retaining all tied completions and deterministic board ordering. |
| Experimental scheduler implementations | Transactional lane slices and deferred prefixes remain internal APIs, not CLI modes. Guarded reversible private-RAM capsules are automatic on eligible network work. Observation barriers and rollback preserve modeled effects; capsules are not native RAM JIT kernels. |
| Build optimization | Release, supported IPO/LTO, and representative optional Clang PGO reduce host overhead without changing modeled ADC fidelity. |

JIT is automatic when available, but does not guarantee a speedup. Synchronized network bursts may use different per-lane instruction counts only when every lane reaches the same exact simulated-time frontier. Multi-instruction prefixes stop before memory/MMIO, and event, deadline, and SysTick boundaries gate admission; mismatched work falls back to exact single-instruction dispatch. General asynchronous network dispatch also remains single-instruction: another lane may schedule a shared event that was not visible when a block's horizon was checked.

## Build

```bash
make BUILD_DIR=build-release BUILD_TYPE=Release IPO=ON test
```

- IPO/LTO is enabled when supported. Debug and sanitizer builds are not throughput baselines.
- CMake attempts native LLVM discovery by default; LLVM 18 or newer and a 64-bit host are required.
- Without LLVM, JIT uses cached handlers only. Check configure output for native backend availability.
- CMake overrides: `FIL_ENABLE_LLVM_JIT=OFF` disables native support; `LLVM_DIR` selects an installation; `FIL_LLVM_COMPONENT_LINKING=ON` selects component-library linking.

### Profile-guided optimization

```bash
make pgo
```

The target cleans old PGO outputs, builds, trains, merges profiles, rebuilds with the profile, and tests. Default training uses the checked-in network configuration with JIT enabled and ADC decimation 1 for 1,000 ms with a 50 million instruction limit.

- Override training with `PGO_TRAIN_ARGS='run-network ...'`.
- Use matching `PGO_CXX` and `LLVM_PROFDATA` versions.
- Retrain after material code, firmware, or toolchain changes.
- Match the scheduling mode too. Plain `run-network` uses the validated fast
  path; leave it unadorned unless measuring an experimental mode. ADC
  decimation 1 retains every conversion.
- Individual stages: `pgo-generate`, `pgo-train`, `pgo-merge`, `pgo-use`, `pgo-test`.

## Network comparison

Build the target firmware before measuring its configured network. Firmware
compiler optimization is a workload change, not an emulator-only speedup;
compare FIL binaries on identical ELFs and clock settings.

ADC common `CCR.CKMODE` selects each group's synchronous clock division.
Correct clock modeling is distinct from ADC decimation: every conversion,
DMA transfer, and interrupt at the configured rate is retained. Results from
different firmware clocks or older ADC clock models are not matched baselines.

```bash
/usr/bin/time -p ./build-release/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 5000 --max-instructions 250000000 --quantum 1024 --strict-mmio --adc-decimation 1
```

Record wall time, stop reason, simulated time, instruction/cycle counts, and JIT diagnostics. Increase the instruction limit if necessary.

With JIT enabled (the default), `run` and `run-network` report per-board `native_jit` diagnostics. Positive `executions` and `instructions` confirm generated code ran, not that it was faster. Account for additional diagnostics when comparing summaries; do not discard guest-visible differences.

### Scheduling and guarded RAM capsules

Network commands use the compact exact single-instruction scheduler by default; network loop batching remains off. General lockstep bursts require a proven pure fixed-cost multi-instruction frontier; scalar timing predictions alone do not authorize a burst. Experimental transactional and deferred scheduler implementations remain internal APIs, not CLI modes.

Guarded RAM capsules are automatic on eligible network work. Diagnostic safety fallbacks preserve behavior when tracing, spin detection, or other observation-sensitive conditions make capsule execution unsafe. Only journaled, reversible writes to non-executable private RAM are admitted; MMIO, FP/system operations, executable stores, unsupported handlers, and unsafe event boundaries end a span. CAN delivery and board-failure draining are observation barriers. CPU/RAM state must not be inspected or modified while a prefix is active except through scheduled callbacks, which materialize it first. Reentrant or concurrent mutation of a running World is unsupported. Run options are snapshotted per invocation.

Audited ADC conversions can be owner-local observations: a capsule may span
another board's conversions while its own conversions still materialize it.
Ownership alone is not sufficient. Ordinary scheduled callbacks remain global,
and guards are checked immediately before callback execution. Certification
requires built-in ADC providers and sample wiring, unchanged interrupt hooks,
matching DMA bus/source-device identity, and the actual next halfword DMA
transfer fitting directly backed RAM. Public hook replacements, unsupported DMA
transfers, and active trace observers force global materialization. Retaining
trace history without an observer remains eligible. No ADC conversion, DMA
write, interrupt, or sample is omitted.

World installs a private mandatory callback barrier, separate from the public
observer slot; an observer cannot disable scheduler synchronization by replacing
itself. Events retain timestamp/insertion ordering and precede CPU completion at
equal times. Synchronous CAN delivery retains its all-board observation barrier.
Standalone Board admission retains its global horizon. Reentrant/concurrent
mutation of a running World is unsupported; if a run throws, discard that World
rather than inspecting or resuming potentially speculative state.

A bounded, immutable period certificate can avoid execution and replay for naturally recurring spans whose RAM bytes remain unchanged after every instruction, including idempotent stores. Compilation records each CPU/fetch endpoint and charged cycle cost. Reuse requires a matching CPU phase, historical execution/clock/FLASH generations, complete canonical RAM read-and-store footprints, usable journal history, and an unchanged restoration token. A different fetch predecessor adjusts only the first instruction's ART stall; later fetch endpoints follow the certificate. Unrelated external RAM changes may remain compatible; changed inputs, journal rewinds, reset, and transaction restoration reject reuse. Cached repeating spans use compressed periodic timing rather than filling one metadata entry per instruction. They admit up to 4096 logical instructions, bounded by event, SysTick, deadline, and representable cycle budgets; uncached speculative work retains the 64-instruction limit. Timing accessors and binary-search cuts reconstruct the selected period phase without modifying RAM and charge cycles once using the admission's original timing fraction. A boundary that cuts an admission retains its valid leading instructions instead of discarding the whole cached span. This does not omit ADC conversions, DMA, interrupts, or observation barriers. JIT diagnostics describe actual handler work, not the logical instructions represented by a cached period. The `memoized_prefixes`, `memoized_instructions`, and per-board `memoized_ram` diagnostics report certificate admissions; admitted instructions may be cut by a later barrier, so these counters are not committed instruction totals. This path uses cached handlers, not LLVM-native RAM kernels. Capsule eligibility remains guarded: audited local ADC observations may be crossed only when provider/wiring, interrupt hooks, DMA identity, and the next directly backed RAM transfer satisfy the checks. Custom callbacks and unsupported DMA remain global observation barriers. Concurrency or reentrant mutation of a running World is unsupported; after a run throws, discard that World rather than inspecting or resuming potentially speculative state.

The full-network benchmark helper measures the automatic execution path. A
repeatable benchmark includes startup, checks all six boards reach the deadline,
and exits unsuccessfully when median throughput is not above realtime:

```bash
tools/bench_full_network.sh --binary build-pgo/fil --reps 3
```

For sustained before/after comparisons, pass distinct baseline and candidate
builds with `--reference-binary path/to/reference/fil` and use an appropriate
longer duration. The helper alternates binary run order and checks matching
counters and final board PCs; it does not compare interpreter and experimental
CLI modes. Keep compiler flags and firmware identical and train PGO for each
source version. Clean-rebuild profile-use objects when replacing a profile in
place; mixed old/new profile summaries can break ThinLTO.

For longer matched comparisons, use the same helper with an explicit duration.
The instruction budget scales with duration; every board must still reach the
deadline. Add `--min-throughput 1.2` to require at least 1.2 simulated seconds per
host wall second, including initialization. The threshold uses the unrounded
median ratio rather than the displayed value.
Keep ADC decimation 1 for full-fidelity comparisons. Retrain PGO for each source
version; an old profile is not a matched baseline.

## ADC decimation

`--adc-decimation N` keeps one of every N continuous-mode scans (1–1024; default 1). Skipped scans omit normal register, DMA, interrupt, and sample-recording effects. Single-shot conversions are not skipped.

**Decimation changes firmware-visible behavior.** Keep N fixed when comparing execution optimizations.

## Validation

```bash
make BUILD_DIR=build-release BUILD_TYPE=Release test
make BUILD_DIR=build-release BUILD_TYPE=Release benchmark BENCH_REPS=3
```

Differential tests cover cached execution, MMIO restart, code invalidation, execution caps, timing, events, and scheduling, including quantum handling when completion timestamps saturate. Native-specific tests cover emitted arithmetic/flags, branches, prefix limits, preserved state, fetch faults, and invalidation.

RAM-prefix tests compare every interruption cut, rotated period phases, CPU/RAM/timer state, input changes, and historical code/clock/FLASH invalidation. Memory-proof tests cover every touched word in unaligned/wide reads and idempotent stores, alias canonicalization, disabled tracking, and journal sequence reuse after rollback. Declining reversible handlers must preserve their entire entry CPU state.

Tests provide regression evidence, not proof for every firmware. Synthetic smoke benchmarks do not establish six-board network throughput; wall-time measurements are not correctness tests.
