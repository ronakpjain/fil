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
| Event-aware scheduling | Bounds execution by events, interrupts, instruction budgets, and simulated-time deadlines. Event insertion prunes retired owner-queue heads so global-only dispatch does not retain every completed ADC callback until shutdown; live event ordering and conversion effects are unchanged. |
| Loop batching | Single-board `run` automatically batches proven repeated loops; network commands do not. Unproven paths execute normally; tracing retains instruction-level execution. |
| Cached JIT handlers | JIT is automatic. Specialized handlers and prepared blocks fall back for unsupported or unsafe operations. |
| Bespoke stencil JIT | Compiles straight-line integer superblocks (ALU, shifts, conditional branches, terminal calls) directly to host machine code: one stencil per guest op in a bump-allocated arena, with scheduler prefix limits and taken-branch exits reported back to dispatch. A bounded exact-key cache reuses immutable kernels after decoded-block eviction, avoiding repeated emission and executable-memory permission changes. Register transfers are limited to the block's referenced and modified registers, preserving values across partial-prefix exits. Unshifted core-register MOVs are native, including N/Z updates that preserve C/V. Blocks with memory suffixes can use a native pure-integer leading prefix; every memory operation remains in checked handlers. Serves plain, trusted, and budgeted reversible block paths with identical guest state, cycles, and event ordering; unsupported, memory/MMIO, system, and floating-point work retains existing paths. Disabled with `FIL_ENABLE_MICRO_JIT=OFF`. |
| Validated network fast path | Plain `run-network` uses compact single-instruction dispatch without loop-proof state. Networks of up to eight boards use a fixed-width timestamp reduction, retaining all tied completions and deterministic board ordering. |
| Guarded RAM capsules | Reversible private-RAM capsules are automatic on eligible network work. Observation barriers, bounded native prefixes, and rollback preserve modeled effects; validate changes against capsules-disabled execution. |
| Build optimization | Release, supported IPO/LTO, and representative optional Clang PGO reduce host overhead without changing modeled ADC fidelity. |

Native kernel admission is performance-based rather than a minimum-length
heuristic beyond the existing two-instruction compilation floor. Each new
immutable kernel is calibrated using budgeted reversible dispatch with and
without native execution on scratch integer CPU state, exercising all prefix
limits and NZCV combinations. This includes native planning, rollback snapshots,
and cycle bookkeeping rather than timing the emitted function alone. Five paired timing
batches alternate execution order; admission requires a 10% median saving and
wins in at least four batches. Uncertain or slower kernels stay on handlers.
Calibration never executes a memory suffix, fetch-stall callback, or simulated
time accounting. Verdicts are cached for the kernel's lifetime, including across
decoded-block eviction, so dispatch contains no host-clock sampling. Host timing
can change optimization choices but must not change guest-visible results.
The `micro_admitted`, `micro_rejected`, and `micro_admission_cost_ns` diagnostics
report distinct-kernel decisions and total calibration cost; benchmark timings
must include that startup cost.

JIT is automatic when available, but does not guarantee a speedup. Synchronized network bursts may use different per-lane instruction counts only when every lane reaches the same exact simulated-time frontier. Multi-instruction prefixes stop before memory/MMIO, and event, deadline, and SysTick boundaries gate admission; mismatched work falls back to exact single-instruction dispatch. General asynchronous network dispatch also remains single-instruction: another lane may schedule a shared event that was not visible when a block's horizon was checked.

## Build

```bash
make BUILD_DIR=build-release BUILD_TYPE=Release IPO=ON test
```

- IPO/LTO is enabled when supported. Debug and sanitizer builds are not throughput baselines.

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

With JIT enabled (the default), `run` and `run-network` report per-board `jit:` diagnostics. The line retains the `micro_*`, `block_*`, and `single_*` counters; generated-code counts do not imply a speedup. Do not discard guest-visible differences when comparing summaries.

### Scheduling and guarded RAM capsules

Network commands use the compact exact single-instruction scheduler by default; network loop batching remains off. General lockstep bursts require a proven pure fixed-cost multi-instruction frontier; scalar timing predictions alone do not authorize a burst.

Guarded RAM capsules are automatic on eligible network work. Diagnostic safety fallbacks preserve behavior when tracing, spin detection, or other observation-sensitive conditions make capsule execution unsafe. Only journaled, reversible writes to non-executable private RAM are admitted; MMIO, FP/system operations, executable stores, unsupported handlers, and unsafe event boundaries end a span. CAN delivery and board-failure draining are observation barriers. CPU/RAM state must not be inspected or modified while a prefix is active except through scheduled callbacks, which materialize it first. Reentrant or concurrent mutation of a running World is unsupported. Run options are snapshotted per invocation.

Certificate-hit admission retains entry read footprints, memory checkpoints,
generation validation, and owner-aware event horizons, but skips the speculative
rollback-state snapshot and new-transition hotness probe. Cached cuts restore
certified states directly; cache misses capture rollback state before execution.
Non-mutating certificate validation reuses the entry checkpoint rather than
sampling it again after restoring only CPU/fetch state.

Native kernels keep the scheduler prefix limit and failed-conditional mask in disjoint fields. Requests are clamped to the block length before packing, and each prefix check reads only the limit field. This corrects a regression in which a failed conditional branch could enlarge the limit, execute an unreported suffix, undercharge cycles, and eventually break interrupted replay. Dispatch restores CPU state if a kernel reports an impossible retirement count; it must never retain speculative changes from a declined kernel.

Validation combines emitter and reversible-execution prefix regressions with real-firmware comparisons against capsules-disabled execution, including trace equality and final counters/PCs beyond the former failure point. These comparisons cover the tested workloads and intervals, not all possible firmware. Use `WorldRunOptions::enable_ram_capsules` for API comparisons; CLI comparisons currently require changing the network-command defaults in `src/cli/cli.cpp`. Comparing two capsule-enabled runs alone does not establish equivalence to exact dispatch.

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

A bounded, immutable period certificate can avoid execution and replay for naturally recurring spans whose RAM bytes remain unchanged after every instruction, including idempotent stores. Compilation records each CPU/fetch endpoint and charged cycle cost. Reuse requires a matching CPU phase, historical execution/clock/FLASH generations, complete canonical RAM read-and-store footprints, usable journal history, and an unchanged restoration token. A different fetch predecessor adjusts only the first instruction's ART stall; later fetch endpoints follow the certificate. Unrelated external RAM changes may remain compatible; changed inputs, journal rewinds, reset, and transaction restoration reject reuse. Cached repeating spans use compressed periodic timing rather than filling one metadata entry per instruction. They admit up to 4096 logical instructions, bounded by event, SysTick, deadline, and representable cycle budgets; uncached speculative work retains the 64-instruction limit. Timing accessors and binary-search cuts reconstruct the selected period phase without modifying RAM and charge cycles once using the admission's original timing fraction. A boundary that cuts an admission retains its valid leading instructions instead of discarding the whole cached span. This does not omit ADC conversions, DMA, interrupts, or observation barriers. JIT diagnostics describe actual handler work, not the logical instructions represented by a cached period. The `memoized_prefixes`, `memoized_instructions`, and per-board `memoized_ram` diagnostics report certificate admissions; admitted instructions may be cut by a later barrier, so these counters are not committed instruction totals. This path uses cached handlers and does not compile RAM kernels. Capsule eligibility remains guarded: audited local ADC observations may be crossed only when provider/wiring, interrupt hooks, DMA identity, and the next directly backed RAM transfer satisfy the checks. Custom callbacks and unsupported DMA remain global observation barriers. Concurrency or reentrant mutation of a running World is unsupported; after a run throws, discard that World rather than inspecting or resuming potentially speculative state.

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

Differential tests cover cached execution, MMIO restart, code invalidation, execution caps, timing, events, and scheduling, including quantum handling when completion timestamps saturate.

RAM-prefix tests compare every interruption cut, rotated period phases, CPU/RAM/timer state, input changes, and historical code/clock/FLASH invalidation. Memory-proof tests cover every touched word in unaligned/wide reads and idempotent stores, alias canonicalization, disabled tracking, and journal sequence reuse after rollback. Declining reversible handlers must preserve their entire entry CPU state.

Tests provide regression evidence, not proof for every firmware. Synthetic smoke benchmarks do not establish six-board network throughput; wall-time measurements are not correctness tests.
