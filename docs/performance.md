# Performance

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

## Event-loop overhead

Relative scheduling and relative clock advancement avoid redundant recursive
mutex acquisitions. During a serial `World::run`, an exclusive event-loop lease
holds the recursive mutex once; same-thread operations on that loop skip
relocking. Nested callbacks retain deterministic ordering, and mutex-protected
operations on other threads wait until the run returns. Outside a lease, those
operations retain ordinary locking. Owner-context scopes remain serial-only;
this does not enable concurrent or reentrant World mutation. Leases are
non-movable, unwind on exceptions, must nest in LIFO order, and must not outlive
the loop implementation. This reduces host synchronization overhead without changing simulated timing.

## Execution optimizations

| Mechanism | Behavior |
| --- | --- |
| Decode and memory caches | Direct-mapped decoded instructions and cached backed-memory access reduce decoding/mapping work while retaining permission checks and executable-write generation invalidation. Cache index folding reduces collisions between distant firmware addresses. |
| CPU hot paths | Specialized integer handlers, prepared block metadata, and guarded backed-memory multi-word paths avoid repeated generic dispatch. Unconditional non-flag-setting ordinary-register moves, immediate add/subtract, and eligible word loads/stores use compact prepared dispatch in reversible blocks, retaining per-instruction fetch/budget accounting and guarded memory helpers. Final-instruction diagnostics are read once after retiring the selected prefix, rather than reloaded for every instruction. Unsupported, faulting, or MMIO work retains ordinary execution. Callback-free reversible spans snapshot stable flash fetch timing, avoiding per-instruction indirect timing calls while preserving sequential ART hits, aliases, and wait-state charges; custom CPU timing callbacks still take precedence. |
| Pending-exception selection cache | Reuses exact exception selection for unchanged CPU masks and system-control generation. Selection-affecting mutations invalidate the cache; the cached takable-pending check avoids rescanning priorities without suppressing eligible exceptions. |
| Event-aware scheduling | Bounds execution by events, interrupts, instruction budgets, and simulated-time deadlines. Event insertion prunes retired owner-queue heads so global-only dispatch does not retain every completed ADC callback until shutdown; live event ordering and conversion effects are unchanged. |
| Loop batching | Single-board `run` automatically batches proven repeated loops; network commands do not. Unproven paths execute normally; tracing retains instruction-level execution. |
| Cached JIT handlers | JIT is automatic. Specialized handlers and prepared blocks fall back for unsupported or unsafe operations. |
| Bespoke stencil JIT | Compiles straight-line integer superblocks (ALU, shifts, conditional branches, terminal calls) directly to host machine code: one stencil per guest op in a bump-allocated arena, with scheduler prefix limits and taken-branch exits reported back to dispatch. A bounded exact-key cache reuses immutable kernels after decoded-block eviction, avoiding repeated emission and executable-memory permission changes. Register transfers are limited to the block's referenced and modified registers, preserving values across partial-prefix exits. Unshifted core-register MOVs are native, including N/Z updates that preserve C/V. Blocks with memory suffixes can use a native pure-integer leading prefix; every memory operation remains in checked handlers. Serves plain, trusted, and budgeted reversible block paths with identical guest state, cycles, and event ordering; unsupported, memory/MMIO, system, and floating-point work retains existing paths. Disabled with `FIL_ENABLE_MICRO_JIT=OFF`. |
| Guarded native traces | Hot generation-tagged traces stitch supported integer/word-memory operations, direct calls/branches, profiled conditional edges, and narrow PUSH/POP across basic blocks. x86-64 emits stencils; AArch64 retains an allocation-free checked C++ fallback. Conditional mismatches exit before the branch retires; predicted POP targets are validated before register/SP commit. Whole-span stack stores retain journal and store-footprint tracking. Every represented instruction executes individually under exact instruction/cycle credit, and retirement metadata comes from the trace plan rather than a basic-block slot. Plans, profiles, compilation attempts and executable storage are bounded; unsupported forms, changed generations, IT, pending exceptions and custom fetch callbacks keep checked execution. Trace diagnostics count actual native work, including speculative certificate replay, not a speed guarantee. |
| Mixed native RAM prefixes | Eligible reversible integer/word-memory prefixes, including normalized literal loads and read-only SP-relative bases, use emitted x86-64 stencils with precise instruction and cycle limits. Checked helpers retain RAM permissions, access attribution, read footprints and write journaling; declined memory operations remain unretired for exact fallback. Custom timing callbacks keep the existing handler path. SP-relative accesses select architectural MSP/PSP. Repeated first-access native declines at an unchanged address use the checked path; changed addresses retry native execution. AArch64 retains its pure stencil backend and a checked mixed-execution fallback over allocation-free, non-owning operation spans. Native retirement/decline diagnostics report coverage, not a speed guarantee. |
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
report distinct-kernel decisions and total calibration cost.

JIT is automatic when available, but does not guarantee a speedup. Synchronized network bursts may use different per-lane instruction counts only when every lane reaches the same exact simulated-time frontier. Multi-instruction prefixes stop before memory/MMIO, and event, deadline, and SysTick boundaries gate admission; mismatched work falls back to exact single-instruction dispatch. General asynchronous network dispatch also remains single-instruction: another lane may schedule a shared event that was not visible when a block's horizon was checked.

## Build

```bash
make BUILD_DIR=build-release BUILD_TYPE=Release IPO=ON test
```

- IPO/LTO is enabled when supported. Debug and sanitizer builds retain extra diagnostic overhead.
- Makefile builds default to `NATIVE=ON`, selecting a supported host CPU flag
  (`-march=native`, or `-mcpu=native` where supported). Use `NATIVE=OFF` for
  portable binaries. Cross-compilation skips host tuning; unsupported compilers
  fall back to portable defaults. Direct CMake builds remain portable unless
  `-DFIL_ENABLE_NATIVE=ON` is supplied.

### Profile-guided optimization

```bash
make pgo
```

The target cleans old PGO outputs, builds, trains, merges profiles, rebuilds with the profile, and tests. Default training uses the checked-in network configuration with JIT enabled and ADC decimation 1 for 1,000 ms with a 50 million instruction limit.

- Override training with `PGO_TRAIN_ARGS='run-network ...'`.
- Use matching `PGO_CXX` and `LLVM_PROFDATA` versions.
- Retrain after material code, firmware, or toolchain changes.
- Match the scheduling mode too. Plain `run-network` uses the validated fast
  path; leave it unadorned to retain that mode. ADC
  decimation 1 retains every conversion.
- Individual stages: `pgo-generate`, `pgo-train`, `pgo-merge`, `pgo-use`, `pgo-test`.

### Host-tuned PGO build

Makefile builds target the host CPU by default, including both PGO profile
generation and profile use. PGO itself remains opt-in; plain `make` does not
train a profile. For example:

```bash
make pgo JOBS=6 CTEST_JOBS=4 \
  PGO_GENERATE_BUILD_DIR=build-runtime-pgo-generate \
  PGO_BUILD_DIR=build-runtime-native \
  NATIVE=ON \
  PGO_TRAIN_ARGS='run-network configs/networks/per_vehicle.json --duration-ms 3000 --max-instructions 3000000000 --quantum 1024 --strict-mmio --adc-decimation 1'
```

`make pgo` replaces the selected PGO build directories. Host-tuned binaries may
require CPU features unavailable on other machines; use `NATIVE=OFF` for
distribution. Neither PGO nor host tuning changes ADC fidelity or simulated
instruction timing.

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

A bounded, immutable period certificate can avoid execution and replay for naturally recurring spans whose RAM bytes remain unchanged after every instruction, including idempotent stores. Compilation records each CPU/fetch endpoint and charged cycle cost. Discovery may ignore only the incoming prior-instruction diagnostic address and fetch predecessor: it independently certifies an exact endpoint-anchored period, adjusts the future first-fetch stall, and leaves the current ordinary prefix unfurled. Cached phase matching remains exact. Reuse requires a matching CPU phase, historical execution/clock/FLASH generations, complete canonical RAM read-and-store footprints, usable journal history, and an unchanged restoration token. A different fetch predecessor adjusts only the first instruction's ART stall; later fetch endpoints follow the certificate. Unrelated journaled RAM changes may remain compatible regardless of CPU or external origin; changed inputs, journal rewinds, reset, and transaction restoration reject reuse. Certificate validation uses complete read-and-store footprints, including unread destinations of idempotent stores; ordinary transaction validation remains stricter. Cached repeating spans use compressed periodic timing rather than filling one metadata entry per instruction. They admit up to 4096 logical instructions, bounded by event, SysTick, deadline, and representable cycle budgets; uncached speculative work retains the 64-instruction limit. Timing accessors and binary-search cuts reconstruct the selected period phase without modifying RAM and charge cycles once using the admission's original timing fraction. A boundary that cuts an admission retains its valid leading instructions instead of discarding the whole cached span. This does not omit ADC conversions, DMA, interrupts, or observation barriers. JIT diagnostics describe actual handler work, not the logical instructions represented by a cached period. The `memoized_prefixes`, `memoized_instructions`, and per-board `memoized_ram` diagnostics report certificate admissions; admitted instructions may be cut by a later barrier, so these counters are not committed instruction totals. This path uses cached handlers and does not compile RAM kernels. Capsule eligibility remains guarded: audited local ADC observations may be crossed only when provider/wiring, interrupt hooks, DMA identity, and the next directly backed RAM transfer satisfy the checks. Custom callbacks and unsupported DMA remain global observation barriers. Concurrency or reentrant mutation of a running World is unsupported; after a run throws, discard that World rather than inspecting or resuming potentially speculative state.

## ADC decimation

`--adc-decimation N` keeps one of every N continuous-mode scans (1–1024; default 1). Skipped scans omit normal register, DMA, interrupt, and sample-recording effects. Single-shot conversions are not skipped.

**Decimation changes firmware-visible behavior.** Keep N at 1 for full fidelity.

## Validation

```bash
make BUILD_DIR=build-release BUILD_TYPE=Release test
```

Differential tests cover cached execution, MMIO restart, code invalidation, execution caps, timing, events, and scheduling, including quantum handling when completion timestamps saturate.

RAM-prefix tests compare every interruption cut, rotated period phases, CPU/RAM/timer state, input changes, and historical code/clock/FLASH invalidation. Memory-proof tests cover every touched word in unaligned/wide reads and idempotent stores, alias canonicalization, disabled tracking, and journal sequence reuse after rollback. Declining reversible handlers must preserve their entire entry CPU state.

Tests provide regression evidence, not proof for every firmware.
