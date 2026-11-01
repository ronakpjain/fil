# Performance

## Recommended defaults

Use Release with supported IPO, automatic JIT, and ADC decimation 1 as the
baseline. Network commands use eligible parallel board lanes by default, with a
100000 ns (100 µs) cross-board synchronization window. This deliberately relaxes
cross-board ordering; it does not skip local peripheral work. `--cross-board-window-ns 0`
selects the exact scheduler with eligible guarded RAM capsules as a reference.
Network loop batching is off;
single-board `run` batches proven loops automatically. Diagnostic modes such as
instruction tracing and spin detection retain safe correctness fallbacks and
may disable optimized paths. Expectation-only `run-network` keeps parallel lanes:
lane-local records are captured per lane and merged deterministically on the
coordinator, so observers run single-threaded without serializing board execution.
A requested `--trace` artifact now also traces lanes this way; `--trace-instr`
still selects the serial instruction path. Experimental scheduler implementations
remain internal APIs, not CLI modes. Prefer strict MMIO for validation, although
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

Parallel owner executors spin at window rendezvous, then park using C++20
atomic wait/notify. The spin length is selected per pool: when the host has at
least two cores beyond the active executors, an idle executor spins for a full
window (32K pause instructions) before parking, because a bounded cross-board
window synchronizes every `cross_board_window_ns` and the park/wake latency is
paid on the critical path of each one while the parked executors would otherwise
be idle; without that slack the conservative park-first budget is kept so the
spin cannot compete with a working lane. Finishing a light lane does not leave
its worker continuously competing for host execution resources while the
ADC-heavy lanes finish. The coordinator also parks after its bounded spin.
Window publication, completion, exception propagation, and shutdown retain
release/acquire ordering.
Owner costs are measured each window and expensive jobs start first. The pool
limits active executors using the ceiling of total owner cost divided by the
largest owner cost, bounded by available execution slots; balanced workloads
retain full parallelism. Inactive workers still acknowledge every generation,
so joins and deterministic exception handling retain the same guarantees. This
changes host scheduling only, not guest events or simulated timing.

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
| Owner-local committed RAM spans | Owner slices retain validated period/transition certificate hits, but sample speculative discovery using bounded PC-local profitability hints. A discovery miss can execute ordinary RAM spans directly, chaining prepared blocks under one exact instruction/cycle/fetch budget that ends strictly before events, SysTick, and the slice deadline. A declined span no longer costs a scheduler round trip: the same budget retires one checked instruction and continues, advancing the board clock over everything already retired first so a time-dependent MMIO read never observes stale simulated time, and re-deriving the horizon and SysTick bound only after a peripheral access actually happened. Batching is refused whenever the memoized probe at that pc could still admit a certificate, so idle-loop discovery and its clock jumps keep engaging. MMIO, executable writes, unsupported work, and pending architectural boundaries still return to checked execution. Sampling affects optimization opportunities only; it never suppresses guest instructions or peripheral work. |
| Compact committed fallback | Compile-time detailed/compact specializations share checked execution, admission, and decline logic. The compact result avoids exporting per-instruction reversible metadata, while updating exact remaining cycle credit and fetch state in place. Detailed callers retain their output arrays; committed callers leave those arrays untouched. IT and custom fetch-timing callbacks keep checked semantics and callback order. |
| Bespoke stencil JIT | Compiles straight-line integer superblocks (ALU, shifts, conditional branches, terminal calls) directly to host machine code: one stencil per guest op in a bump-allocated arena, with scheduler prefix limits and taken-branch exits reported back to dispatch. A bounded exact-key cache reuses immutable kernels after decoded-block eviction, avoiding repeated emission and executable-memory permission changes. Register transfers are limited to the block's referenced and modified registers, preserving values across partial-prefix exits. Unshifted core-register MOVs are native, including N/Z updates that preserve C/V. Blocks with memory suffixes can use a native pure-integer leading prefix; every memory operation remains in checked handlers. Serves plain, trusted, and budgeted reversible block paths with identical guest state, cycles, and event ordering; unsupported, memory/MMIO, system, and floating-point work retains existing paths. Disabled with `FIL_ENABLE_MICRO_JIT=OFF`. |
| Guarded native traces | Hot generation-tagged traces stitch supported integer/byte/word-memory operations, register compares, byte/halfword extends and bitfield extracts, direct calls/branches, guarded profiled register BLX calls, profiled conditional and CBZ/CBNZ edges, and narrow PUSH/POP across basic blocks. Both x86-64 and AArch64 emit direct integer, guard, and memory stencils; memory and stack transfers call checked scalar helpers. The trace builder admits these extra integer operations explicitly, so enum placement does not silently exclude them from the supported set. Conditional mismatches exit before the branch retires. POP PC validates Thumb state and excludes exception-return tokens before register/SP commit. CPU traces opt into retiring a valid alternative return target with an immediate terminal side exit: the actual target is retained and no predicted successor executes. Low-level native callers retain strict predicted-target guards by default. Whole-span stack stores retain journal and store-footprint tracking. Repeated PCs can be unrolled up to 64 operations in compact committed execution; the detailed reversible API remains capped at its 16-entry metadata capacity. Timing plans cache the full flash-timing snapshot, with prefix sums normalized to the cheaper first-fetch mode. An entry-mode bias is reserved before admission and charged only when an instruction retires, so alternating sequential/non-sequential entry does not rebuild the tables; detailed output still includes the exact first-fetch charge. Every represented instruction executes individually under exact instruction/cycle credit, and retirement metadata comes from the trace plan rather than a basic-block slot. AArch64 shares its integer stencils with the pure backend and publishes CPU metadata, NZCV and progress after each committed instruction. x86-64 selects admitted-prefix versus checked-cycle kernels once at trace entry, avoiding per-instruction mode checks while retaining instruction-limit, branch and memory guards in both kernels. Their shared exit stubs publish the exact prefix's cycle charge, PC, diagnostic address, NZCV and progress; guest register, LR, SP and RAM effects remain immediate. This requires synchronous RAM-only helpers that do not inspect deferred CPU/progress metadata; new PC/flag-reading operands or observers require materialization first. The trace cache preserves all 4096 prepared-block indices instead of folding them into 256 slots, so alternating hot prefixes do not repeatedly evict each other before compilation. Entry storage is allocated lazily on the heap, so non-native runs do not allocate this storage and CPU stack footprint does not grow. Plans, profiles, compilation attempts and executable storage remain bounded; unsupported forms, changed generations, IT, pending exceptions and custom fetch callbacks keep checked execution. Trace diagnostics count actual native work, including speculative certificate replay, not a speed guarantee. |
| Mixed native RAM prefixes | Eligible reversible integer/word-memory prefixes, including normalized literal loads and read-only SP-relative bases, use emitted x86-64 and AArch64 stencils with precise instruction and cycle limits. Checked helpers retain RAM permissions, access attribution, read footprints and write journaling; declined memory operations remain unretired for exact fallback. Custom timing callbacks keep the existing handler path. SP-relative accesses select architectural MSP/PSP. Repeated first-access native declines at an unchanged address use the checked path; changed addresses retry native execution. AArch64 uses the same shared native emitter for mixed blocks and guarded traces, with per-instruction commit boundaries. Native retirement/decline diagnostics report coverage, not a speed guarantee. |
| Validated network fast path | Plain `run-network` uses compact single-instruction dispatch without loop-proof state. Networks of up to eight boards use a fixed-width timestamp reduction, retaining all tied completions and deterministic board ordering. |
| Guarded RAM capsules | Reversible private-RAM capsules are automatic on eligible network work. Preliminary safety checks resolve the first journal entry of each touched address with a single generation-stamped address set rather than rescanning the window per mutation, so multi-mutation windows stay linear. Capturing the speculative pass directly into the published evaluation record avoids a second per-instruction metadata copy. Observation barriers, bounded native prefixes, and rollback preserve modeled effects; validate changes against capsules-disabled execution. |
| Build optimization | Release, supported IPO/LTO, and representative optional Clang PGO reduce host overhead without changing modeled ADC fidelity. |

## Bounded-window attribution

The bounded cross-board window is a real semantic boundary, not only a host
scheduling choice. In the owner-lane path a board retires straight-line work in
committed spans and yields to the coordinator only where the span budget ends,
so the memoized idle-loop certificates are offered at fewer instruction
boundaries than in the exact lockstep scheduler used by
`cross_board_window_ns = 0`. Both paths reach the same simulated deadline and
consume the same cycles (measured within 0.003% on the six-board PER network at
ADC decimation 1), but the *reported* instruction count can differ because idle
fast-forwarding engages later. Treat exact counter equality as an invariant
between parallel lanes and the same window executed serially, which is still
asserted, rather than between the windowed path and the exact lockstep
reference. `tools/bench_full_network.sh --reference-binary` compares counters and
will therefore reject a cross-scheduler pair; give it two builds of the same
scheduler when checking a change.

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

A separate whole-period certificate handles loops that temporarily change RAM
but restore every touched byte at the period boundary. It certifies the complete
read-and-store footprint and requires the exact entry CPU and fetch phase;
interior phases cannot be restored from CPU snapshots alone. Whole periods are
folded without RAM changes, while an arbitrary partial cut replays its residual
instructions with checked RAM effects. At clocks up to 1 GHz, these admissions
use compressed periodic timing for up to 4096 instructions; higher clocks keep
the bounded 128-instruction representation with strictly increasing completion
timestamps. Both retain the same event, SysTick, deadline, clock,
executable-generation, flash-timing, and journal guards. CPU-only cycle
metadata remains separate from charged flash timing for interrupted replay.
Successful exact-entry lookups rebase the validated journal window only after
input compatibility is proved, so unrelated history need not remain available
since certificate construction. This mechanism recognizes execution behavior,
not firmware symbols or function names.

## ADC decimation

`--adc-decimation N` keeps one of every N continuous-mode scans (1–1024; default 1). Skipped scans omit normal register, DMA, interrupt, and sample-recording effects. Single-shot conversions are not skipped.

**Decimation changes firmware-visible behavior.** Keep N at 1 for full fidelity.

## Control-flow and MMIO boundaries

Prepared blocks end at control-flow terminators. Guarded native traces can
stitch supported branches, calls, and returns across blocks, but checked memory
helpers decline MMIO: peripheral side effects cannot be journaled or replayed
by a RAM-only executor. Unsupported or declined work retains checked execution.
Execution must honor event, SysTick, interrupt, and instruction-budget boundaries;
batching cannot move peripheral effects ahead of their simulated instruction time.

Native trace return stitching is limited to the canonical 16-bit `BX LR`
encoding. It may follow a previously observed Thumb return target only when its
execution-generation profile matches. Generated and portable execution re-check
LR at runtime and decline before retiring BX on target mismatch, a non-Thumb
LR, or an exception-return token. This does not admit arbitrary `BX Rm`.
Per-instruction execution and fetch-cycle admission remain unchanged.

## Validation

```bash
make BUILD_DIR=build-release BUILD_TYPE=Release test
```

Differential tests cover cached execution, MMIO restart, code invalidation, execution caps, timing, events, and scheduling, including quantum handling when completion timestamps saturate.

RAM-prefix tests compare every interruption cut, rotated period phases, CPU/RAM/timer state, input changes, and historical code/clock/FLASH invalidation. Memory-proof tests cover every touched word in unaligned/wide reads and idempotent stores, alias canonicalization, disabled tracking, and journal sequence reuse after rollback. Declining reversible handlers must preserve their entire entry CPU state.

Tests provide regression evidence, not proof for every firmware.
