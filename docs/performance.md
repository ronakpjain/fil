# Performance

## Recommended defaults

Use Release with supported IPO, automatic JIT, and ADC decimation 1 as the
baseline. Plain `run-network` uses independent board slices with a 1000 ns
cross-board synchronization window. This deliberately relaxes cross-board
ordering; it does not skip local peripheral work. `--cross-board-window-ns 0`
selects the exact scheduler with eligible guarded RAM capsules as a reference.
Network loop batching is off;
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
mutex acquisitions. During a serial `World::run` or a standalone `Board::run`, an exclusive
event-loop lease holds the recursive mutex once; same-thread operations on that loop skip
relocking. Nested callbacks retain deterministic ordering, and mutex-protected
operations on other threads wait until the run returns. Outside a lease, those
operations retain ordinary locking. Owner-context scopes remain serial-only;
this does not enable concurrent or reentrant World mutation. Leases are
non-movable, unwind on exceptions, must nest in LIFO order, and must not outlive
the loop implementation. This reduces host synchronization overhead without changing simulated timing.

Scheduled callbacks live in reusable slots rather than per-event heap nodes.
Each owner and global queue stores a `(timestamp, insertion sequence)` key with
the slot identity; a monotonically increasing event id validates the slot, so a
reused slot cannot resurrect a cancelled predecessor. Scheduling, cancellation,
and same-time reentry keep the reference order: timestamp first, then insertion
sequence. The slot vector only grows to the maximum number of simultaneously
pending events. Recurring device events (FDCAN arbitration/completion and ADC
conversion/skipped-scan landing) use allocation-free built-in callbacks: the
slot stores a function pointer and device context instead of constructing a
`std::function`, and a scheduled-handle trampoline retires the handle identity
before the device callback runs.

## Execution optimizations

| Mechanism | Behavior |
| --- | --- |
| Decode and memory caches | Direct-mapped decoded instructions and cached backed-memory access reduce decoding/mapping work while retaining permission checks and executable-write generation invalidation. Cache index folding reduces collisions between distant firmware addresses. |
| CPU hot paths | Specialized integer handlers, prepared block metadata, and guarded backed-memory multi-word paths avoid repeated generic dispatch. Unconditional non-flag-setting ordinary-register moves, immediate add/subtract, and eligible word loads/stores use compact prepared dispatch in reversible blocks, retaining per-instruction fetch/budget accounting and guarded memory helpers. Final-instruction diagnostics are read once after retiring the selected prefix, rather than reloaded for every instruction. Unsupported, faulting, or MMIO work retains ordinary execution. Callback-free reversible spans snapshot stable flash fetch timing, avoiding per-instruction indirect timing calls while preserving sequential ART hits, aliases, and wait-state charges; custom CPU timing callbacks still take precedence. |
| Pending-exception selection cache | Reuses exact exception selection for unchanged CPU masks and system-control generation. Selection-affecting mutations invalidate the cache; the cached takable-pending check avoids rescanning priorities without suppressing eligible exceptions. |
| Conservative lane horizons | Allocation-free effect summaries convert validated native spans and proved private memory facts into an entry-anchored local prefix plus the earliest possible shared action. Unknown bases, MMIO, branches that may leave the span, and unproved stack ranges end the prefix conservatively; a coordinator frontier is the minimum finite bound across lanes, pending global events, and external inputs. Summaries carry no snapshot, journal, or replay state. Prepared native traces accumulate conservative compile-time counts of local prefix operations, traces that reach a shared access, traces with unresolved control flow, and zero-prefix traces; these describe horizon opportunity, not throughput. |
| Forward instruction admission | A forward admission checks instruction, cycle, deadline, local-boundary, and shared-frontier credit before the first effect. Exact cost comes from prepared instruction metadata; when it is unavailable or exceeds credit the instruction stays on checked execution with no state change. A deferred-completion result keeps at most one issued instruction in flight so callbacks at an equal timestamp can run before its completion is published. |
| Event-aware scheduling | Bounds execution by events, interrupts, instruction budgets, and simulated-time deadlines. Event insertion prunes retired owner-queue heads so global-only dispatch does not retain every completed ADC callback until shutdown; live event ordering and conversion effects are unchanged. |
| Loop batching | Single-board `run` automatically batches proven repeated loops; network commands do not. Unproven paths execute normally; tracing retains instruction-level execution. |
| Cached JIT handlers | JIT is automatic. Specialized handlers and prepared blocks fall back for unsupported or unsafe operations. |
| Bespoke stencil JIT | Compiles straight-line integer superblocks (ALU, shifts, conditional branches, terminal calls) directly to host machine code: one stencil per guest op in a bump-allocated arena, with scheduler prefix limits and taken-branch exits reported back to dispatch. A bounded exact-key cache reuses immutable kernels after decoded-block eviction, avoiding repeated emission and executable-memory permission changes. Register transfers are limited to the block's referenced and modified registers, preserving values across partial-prefix exits. Unshifted core-register MOVs are native, including N/Z updates that preserve C/V. Blocks with memory suffixes can use a native pure-integer leading prefix; every memory operation remains in checked handlers. Serves plain, trusted, and budgeted reversible block paths with identical guest state, cycles, and event ordering; unsupported, memory/MMIO, system, and floating-point work retains existing paths. Disabled with `FIL_ENABLE_MICRO_JIT=OFF`. |
| Guarded native traces | Hot generation-tagged traces stitch supported integer/byte/word-memory operations, register compares, byte/halfword extends and bitfield extracts, direct calls/branches, profiled conditional edges, and narrow PUSH/POP across basic blocks. x86-64 emits stencils; AArch64 retains an allocation-free checked C++ fallback. The trace builder admits these extra integer operations explicitly, so enum placement does not silently exclude them from the supported set. Conditional mismatches exit before the branch retires; predicted POP targets are validated before register/SP commit. Whole-span stack stores retain journal and store-footprint tracking. Repeated PCs can be unrolled up to 64 operations in compact committed execution; the detailed reversible API remains capped at its 16-entry metadata capacity. Timing plans cache the full flash-timing snapshot and entry fetch sequencing, with prefix sums for aggregate retirement. Every represented instruction executes individually under exact instruction/cycle credit, and retirement metadata comes from the trace plan rather than a basic-block slot. x86-64 retains committed cycles in a callee-saved register and publishes PC, diagnostic address, NZCV and progress once at the selected exit; guest register, LR, SP and RAM effects remain immediate. This requires synchronous RAM-only helpers that do not inspect deferred CPU/progress metadata; new PC/flag-reading operands or observers require materialization first. Plans, profiles, compilation attempts and executable storage are bounded; unsupported forms, changed generations, IT, pending exceptions and custom fetch callbacks keep checked execution. Trace diagnostics count actual native work, including speculative certificate replay, not a speed guarantee. |
| Mixed native RAM prefixes | Eligible reversible integer/word-memory prefixes, including normalized literal loads and read-only SP-relative bases, use emitted x86-64 stencils with precise instruction and cycle limits. Checked helpers retain RAM permissions, access attribution, read footprints and write journaling; declined memory operations remain unretired for exact fallback. Custom timing callbacks keep the existing handler path. SP-relative accesses select architectural MSP/PSP. Repeated first-access native declines at an unchanged address use the checked path; changed addresses retry native execution. AArch64 retains its pure stencil backend and a checked mixed-execution fallback over allocation-free, non-owning operation spans. Native retirement/decline diagnostics report coverage, not a speed guarantee. |
| Validated network fast path | Plain `run-network` uses compact single-instruction dispatch without loop-proof state. Networks of up to eight boards use a fixed-width timestamp reduction, retaining all tied completions and deterministic board ordering. |
| Guarded RAM capsules | Reversible private-RAM capsules are automatic on eligible network work. Preliminary safety checks resolve the first journal entry of each touched address with a single generation-stamped address set rather than rescanning the window per mutation, so multi-mutation windows stay linear. Capturing the speculative pass directly into the published evaluation record avoids a second per-instruction metadata copy. Observation barriers, bounded native prefixes, and rollback preserve modeled effects; validate changes against capsules-disabled execution. |
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

Network commands default to independent owner timelines. Eligible unobserved
networks execute boards on persistent host workers with private event domains;
tracing and external event/trace observers retain serial execution. Each owner
uses validated RAM-period certificates for repeated idle work and checked
RAM-only native spans otherwise. Certificate misses retain journaled local
admission, but each admitted span commits in full: there is no cross-board
snapshot, interruption rollback or replay. Spans stop strictly before owner
events and SysTick boundaries; MMIO and unsupported instructions use checked
scalar execution. Owner callbacks preserve their local timestamps and instruction
completion ordering. Shared callbacks are dispatched separately at window
rendezvous, before releasing the next board slices. Pending interrupts are
settled before the next slice's foreground instruction.

This is intentionally approximate **only across boards**: communication effects
are visible within one configured window plus the latency of an instruction
already in flight. The window defaults to 1000 ns. CAN request timestamps are
captured from the issuing board clock before switching to the shared domain.
CAN operations are staged per lane with their issuing timestamps, then replayed
in timestamp/board/local-order with shared events interleaved while workers are
parked. Shared queue admission and its error/completion feedback occur at this
rendezvous, not concurrently in workers. Reset/reuse generations reject stale
TX completions. CAN queue feedback and same-window shared-state interactions
may differ from exact dispatch; do not compare final PCs or instruction counts
across timing models as if they were identical. Owner callbacks must not directly inspect or
mutate another board. Reentrant clock/event dispatch during owner-only runs is
rejected; scheduling and cancellation from callbacks remain supported.

`--cross-board-window-ns 0` retains the exact reference scheduler. In that
scheduler, general lockstep bursts require a proven pure fixed-cost
multi-instruction frontier; scalar timing predictions alone do not authorize
a burst. Network loop batching remains off.

Guarded RAM capsules are automatic on eligible exact-scheduler network work. Diagnostic safety fallbacks preserve behavior when tracing, spin detection, or other observation-sensitive conditions make capsule execution unsafe. Only journaled, reversible writes to non-executable private RAM are admitted; MMIO, FP/system operations, executable stores, unsupported handlers, and unsafe event boundaries end a span. CAN delivery and board-failure draining are observation barriers. CPU/RAM state must not be inspected or modified while a prefix is active except through scheduled callbacks, which materialize it first. Reentrant or concurrent mutation of a running World is unsupported. Run options are snapshotted per invocation.

Certificate-hit admission retains entry read footprints, memory checkpoints,
generation validation, and owner-aware event horizons, but skips the speculative
rollback-state snapshot and new-transition hotness probe. Cached cuts restore
certified states directly; cache misses capture rollback state before execution.
Non-mutating certificate validation reuses the entry checkpoint rather than
sampling it again after restoring only CPU/fetch state.

Native kernels keep the scheduler prefix limit and failed-conditional mask in disjoint fields. Requests are clamped to the block length before packing, and each prefix check reads only the limit field. This corrects a regression in which a failed conditional branch could enlarge the limit, execute an unreported suffix, undercharge cycles, and eventually break interrupted replay. Dispatch restores CPU state if a kernel reports an impossible retirement count; it must never retain speculative changes from a declined kernel.

Validation combines emitter and reversible-execution prefix regressions with real-firmware comparisons against capsules-disabled execution, including trace equality and final counters/PCs beyond the former failure point. These comparisons cover the tested workloads and intervals, not all possible firmware. Use `WorldRunOptions::cross_board_window_ns = 0` and
`WorldRunOptions::enable_ram_capsules` for exact API comparisons; select exact
CLI timing with `--cross-board-window-ns 0`. Comparing two capsule-enabled runs alone does not establish equivalence to exact dispatch.

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

A bounded, immutable period certificate can avoid execution and replay for naturally recurring spans whose RAM bytes remain unchanged after every instruction, including idempotent stores. Compilation records each CPU/fetch endpoint and charged cycle cost. Discovery may ignore only the incoming prior-instruction diagnostic address and fetch predecessor: it independently certifies an exact endpoint-anchored period, adjusts the future first-fetch stall, and leaves the current ordinary prefix unfurled. Cached phase matching remains exact. Reuse requires a matching CPU phase, historical execution/clock/FLASH generations, complete canonical RAM read-and-store footprints, usable journal history, and an unchanged restoration token. A different fetch predecessor adjusts only the first instruction's ART stall; later fetch endpoints follow the certificate. Unrelated journaled RAM changes may remain compatible regardless of CPU or external origin; changed inputs, journal rewinds, reset, and transaction restoration reject reuse. Certificate validation uses complete read-and-store footprints, including unread destinations of idempotent stores; ordinary transaction validation remains stricter. Cached repeating spans use compressed periodic timing rather than filling one metadata entry per instruction. They admit up to 4096 logical instructions, bounded by event, SysTick, deadline, and representable cycle budgets; uncached speculative work retains a fixed 128-instruction limit, which amortizes the per-admission scheduler and certificate work over longer spans. Timing accessors and binary-search cuts reconstruct the selected period phase without modifying RAM and charge cycles once using the admission's original timing fraction. A boundary that cuts an admission retains its valid leading instructions instead of discarding the whole cached span. This does not omit ADC conversions, DMA, interrupts, or observation barriers. JIT diagnostics describe actual handler work, not the logical instructions represented by a cached period. The `memoized_prefixes`, `memoized_instructions`, and per-board `memoized_ram` diagnostics report certificate admissions; admitted instructions may be cut by a later barrier, so these counters are not committed instruction totals. This path uses cached handlers and does not compile RAM kernels. Capsule eligibility remains guarded: audited local ADC observations may be crossed only when provider/wiring, interrupt hooks, DMA identity, and the next directly backed RAM transfer satisfy the checks. Custom callbacks and unsupported DMA remain global observation barriers. Concurrency or reentrant mutation of a running World is unsupported; after a run throws, discard that World rather than inspecting or resuming potentially speculative state.

## ADC decimation

`--adc-decimation N` keeps one of every N continuous-mode scans (1–1024; default 1). Skipped scans omit normal register, DMA, interrupt, and sample-recording effects. Single-shot conversions are not skipped.

**Decimation changes firmware-visible behavior.** Keep N at 1 for full fidelity.

## Measured six-board characteristics

Host: AMD Ryzen 9 4900HS (8 physical cores, 16 threads), PGO Release build,
`configs/networks/per_vehicle.json`, ADC decimation 1. One simulated second of
the six-board `per_vehicle` network currently costs roughly 2.4 s wall
(~0.4x real time); the exact-scheduler reference (`--cross-board-window-ns 0`)
costs roughly 7 s. Wall time is insensitive to the window for values at or above
100 us and grows sharply below that, so the default 1000 ns window should be
raised for throughput runs.

Wall time is bounded by two boards, not by scheduling or cross-board
coordination. Per-board solo wall times for one simulated second are
dashboard ~0.37 s, main_module ~0.32 s, torque_vector ~0.16 s, a_box ~0.63 s,
and front_driveline and rear_driveline ~2.28 s each. Six independent
single-board processes reproduce the same total and the same front_driveline
cost, so the bottleneck is that board's per-instruction execution cost rather
than parallelism.

Parallelism is already near its floor, so the remaining gap cannot be closed by
scheduling. The six lanes cost 2.4x more CPU than the same boards run
separately, and front_driveline alone costs 2.29 s of CPU for one simulated
second under every scheduler (exact, serial owner slices, and parallel lanes),
so no parallel arrangement can finish that board faster than ~2.3 s. The
critical path is the slowest board, not the coordination.

front_driveline's cost is intrinsic to its code, and this is confirmed against
the firmware rather than inferred. `fil disasm-window`
`~/PER/monorepo/firmware/output/front_driveline/front_driveline.elf --addr
0x8002ada` shows the hot region is FreeRTOS
`prvCheckTasksWaitingTermination`: a tight list/tick loop of the form
`ldr addr; ldr; subs #1; str; ... bl ...` that writes RAM every iteration and
calls out every few instructions. The emulator's own counters agree:
`trace_horizon_shared` is 1474 of 1474 and `trace_horizon_zero_prefix` is 1230
of 1474, so 83% of native traces have no provably private prefix, and only 4012
block / 315 native / 45 trace compilations exist, confirming a tiny code
footprint. The native block builder also deliberately stops its stencil prefix
at the first control-flow instruction, and the loop contains `bl` calls, so
each entry retires only ~6 instructions. It fires ~259k events per simulated
second (one per ~181 instructions), and ADC decimation demonstrates event
dispatch dominates: decimation 1 costs 2.27 s with 259k events while decimation
8 costs 0.42 s with 65k events at nearly the same instruction count. Because
the loop mutates RAM it can never satisfy a RAM-neutral period certificate, so
only ~17% of instructions are covered by certificate hits. Use `--no-parallel-lanes` to run the same workload on one thread as
a validation reference; it costs ~6.5 s wall but the same ~6.5 s of CPU as the
parallel lanes' useful work.

`fil run-network ...` prints per-board JIT diagnostics. Note that the printed
`block_executions` counter aggregates the native-trace, IT-scalar, trusted
reversible and prepared block paths; dividing it by `block_instructions` does
not measure block-path span length. Measured instead with temporary
break-reason counters, front_driveline's 10.4M block executions end as follows:
terminator or PC change 47.7% (dominated by `term_ret` returns at 27.6%), MMIO
or store decline 2.5%, generic-op stop 2.5%, cycle credit 0.1%. Native traces
are compiled at ~37 instructions but execute only ~6.8 because they exit on
memory declines (509,584 per simulated second) far more often than on branch
mismatches (about 70,000 after the saturating predictor below).

The dominant cost is event delivery, not instruction execution. Running the same
front_driveline firmware with ADC decimation 64 (8,108 events) takes 298 ms for
43.5M instructions, while decimation 1 (259,492 events) takes 2,317 ms for 47.0M
instructions: 251k extra events cost about 2.0 s while adding only 3.5M
instructions. Each event adds roughly 62 extra execution entries (block,
native-trace and single-step combined) for only about 14 extra instructions,
i.e. roughly 8 us of host time per event callback. Full ADC fidelity is a hard
requirement, so decimation is not an acceptable fix; the per-event path is what
must become cheaper. A useful bound: even making every span-admission path free
would remove only about 1.1 s of the current 2.3 s, which is still above the
one-second target, so both per-event cost and span admission matter.

Measured call counts for front_driveline over one simulated second:
`prepareReversibleRamPrefix` runs 3,108,580 times and reaches its full
speculative search 2,956,880 times; `stepMemoizedLocal` succeeds 1,689,030
times and fails 1,419,550 times after doing that work; `stepCommittedLocal`
succeeds only 10,687 times while failing 1,408,863 times; and the loop falls
through to single-stepping 1,408,855 times. Every cheap pre-check (instruction
limit, journal tracking, block boundary, clock, horizon) fires zero times, so
there is no cheap rejection available ahead of the expensive search.

The systemic limit is that every span mechanism stops at the first taken
control flow or MMIO access: the fallback block loop breaks on any PC
discontinuity, `prepareJitBlock` marks unconditional control flow as a
terminator, and native traces must decline when a memory helper refuses an MMIO
address because peripheral side effects cannot be journaled or replayed. Real
firmware idle code (FreeRTOS `prvIdleTask` calling
`prvCheckTasksWaitingTermination`) writes an MMIO register every few
instructions and calls out every few instructions, so spans of three or four
instructions are the expected steady state, not a defect.

Removing the RAM-period certificate path is far worse (roughly 13x wall time
and ~8x more span entries), and disabling certificate discovery entirely is
neutral, so discovery is not a bottleneck; keep the certificate path.
Measured-neutral changes that should not be retried: certificate-discovery
guards, in-place scheduled-event rescheduling, clock caching, admitting every
stencil kernel past its profitability gate, and certificate-cache slot changes.
A real improvement requires a mechanism that can continue past an MMIO access
while keeping instruction and cycle totals bit-identical, or a return-stitching
strategy for call-dense code; both need differential tests over exact totals
rather than the six-board aggregate. Native trace return stitching is narrowly
limited to the canonical 16-bit `BX LR` encoding. It may follow a previously
observed Thumb return target only when that target's execution-generation
profile matches; generated and portable execution re-check LR at runtime and
decline before retiring BX on target mismatch, a non-Thumb LR, or an
exception-return token. This does not admit arbitrary `BX Rm` or claim a
measured speedup. Per-instruction execution/fetch cycle admission remains
unchanged.

## Validation

```bash
make BUILD_DIR=build-release BUILD_TYPE=Release test
```

Differential tests cover cached execution, MMIO restart, code invalidation, execution caps, timing, events, and scheduling, including quantum handling when completion timestamps saturate.

RAM-prefix tests compare every interruption cut, rotated period phases, CPU/RAM/timer state, input changes, and historical code/clock/FLASH invalidation. Memory-proof tests cover every touched word in unaligned/wide reads and idempotent stores, alias canonicalization, disabled tracking, and journal sequence reuse after rollback. Declining reversible handlers must preserve their entire entry CPU state.

Tests provide regression evidence, not proof for every firmware.
