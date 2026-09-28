# Performance

`fil` aims to run representative firmware at or above real time without changing
observable target behavior. Host timings are measured with `/usr/bin/time`; simulated
time comes from emulator output. Pacing follows the documented real-timing
model (Cortex-M4 per-class pipeline cycles plus STM32G4 flash wait states),
not one cycle per instruction; see [Real timing audit](real_timing_audit.md).
Under this model `cycles >= instructions` and CPI is reported alongside
throughput (`simulated seconds / wall seconds`).

## Reference results

### Real firmware (PER vehicle network, current)

The reference workload is 1 simulated second of the six-board
`configs/networks/per_vehicle.json` network running real PER firmware
(dashboard, main module, torque vector, a_box, front and rear driveline on
a shared 500 kbit/s CAN bus). It needs the sibling PER checkout referenced
by `configs/boards/*.json`; without it, use the synthetic smoke benchmark
below. Every board runs continuous ADC+DMA motor-control firmware. At
default settings no batchable idle loop exists (`loop_batches: 0`), so the
default row measures the interpreter, scheduler, and event paths, not loop
acceleration. Decimation has a second-order effect beyond removing ADC
work: with fewer interrupts, firmware idles long enough for exact-state
loop proofs to engage (N=8: 87,984 batches covering 16.7M instructions;
N=32: 37,332 batches covering 21.7M) — same proofs, same guarantees, more
idle to cover.

Measured 2026-09-28 on Apple M3 / 8 cores / 8 GB / macOS 27.2, Apple Clang
21.0.0, Release + IPO, AC power, median of 3:

| Case | Instructions | Cycles | CPI | Event callbacks | Median wall time | Throughput | Speedup vs default |
|---|---|---:|---:|---:|---:|---:|---:|
| network x6, default | 39,844,635 | 96,000,012 | 2.41 | 244,239 | 2.565 s | 0.39x | 1.00x |
| network x6, `--adc-decimation 8` | 38,443,217 | 96,000,004 | 2.50 | 56,998 | 1.243 s | 0.80x | 2.06x |
| network x6, `--adc-decimation 32` | 38,208,038 | 96,000,010 | 2.51 | 14,256 | 0.926 s | 1.08x | 2.77x |
| front_driveline x1, default | 7,054,642 | 16,000,003 | 2.27 | — | 0.299 s | 3.35x | 1.00x |
| front_driveline x1, `--adc-decimation 8` | 6,402,378 | 16,000,000 | 2.50 | — | 0.070 s | 14.2x | 4.24x |
| dashboard x1, default | 6,301,749 | 16,000,001 | 2.54 | — | 0.052 s | 19.2x | 1.00x |
| dashboard x1, `--adc-decimation 8` | 6,264,525 | 16,000,000 | 2.55 | — | 0.021 s | 48.4x | 2.52x |

All runs stop at the 1 s simulated-time deadline (`time_ns: 1000000187–1000000249;
the sub-microsecond overshoot is atomic event completion at the frontier)
with `cycles >= instructions` everywhere. CPI sits near 2.4–2.5 because
the firmware runs PLL clocks with flash wait states, not because of host
behavior. Single boards are far above realtime on their own (dashboard:
19x); the network runs 2.4x slower per instruction than the sum of its
boards (17.6M scheduler frontiers for 39.8M instructions, 244k event
callbacks at default), which is why the ADC event path dominates the
tradeoff: decimation removes conversions, DMA transfers, ISR entries, and
queue operations while the conversion schedule itself never drifts, so
`cycles` totals agree to within 8 of 96M across all three network runs.
At `--adc-decimation 32` the network crosses realtime (1.08x) with a 2.77x
speedup; per-board instruction streams are unchanged apart from the
decimated ISR slices (see the decimation contract under
[Diagnostics and peripheral work](#diagnostics-and-peripheral-work)).
Host, compiler, power mode, and firmware affect these numbers — compare
only runs made under the same conditions, and never against the retired
1-CPI figures below (equal instruction counts now advance more simulated
time).

### Synthetic smoke benchmark (no PER checkout)

`tools/bench_real_timing.py` generates hand-assembled Thumb fixtures (an
idle `4x NOP + B` flash loop at reset 16 MHz, and a variant that programs
`FLASH_ACR=0x304` then a 170 MHz HSE-PLL before spinning the same loop)
and runs 1 s single-board and synthetic six-board workloads, batching on
and off. It is a portability smoke test for environments without the PER
firmware checkout — not the reference workload. It asserts
`cycles >= instructions` and the 1 s deadline window on every case and
prints instructions / cycles / CPI / wall time / throughput.

### Pre-model baseline (one cycle per instruction, stale)

The historical reference workload is a one-second run of the six-board
`configs/networks/per_vehicle.json` network. Under the retired 1-CPI model
all modes stopped at exactly 1,000,000,000 ns with 96,000,000 logical
instructions and cycles (16,000,000 per board). Do not compare these figures
with real-timing-model runs: equal instruction counts now advance more
simulated time.

| Host mode | Build | Loop batching | Median wall time | Throughput |
|---|---|---:|---:|---:|
| AC power | Release + IPO | on | 0.87 s | 1.15x real time |
| AC power | Clang PGO | on | 0.73 s | 1.37x real time |
| AC power | Release + IPO | off | 3.33 s | 0.30x real time |
| Battery, macOS Low Power Mode | Release + IPO | on | 1.57 s | 0.64x real time |
| Battery, macOS Low Power Mode | Clang PGO | on | 1.33 s | 0.75x real time |

(1-CPI model; see above.)

The AC figures are medians of three consecutive runs; the battery figures are
medians from alternating five-run A/B tests. Power mode, compiler, firmware, and
host hardware materially affect the result, so compare only runs made under the
same conditions. On the AC baseline, exact-state batching is about 3.8x faster
than instruction-by-instruction network execution.

For the single-board `g4_testing` workload, optimization of the unbatched
interpreter reduced wall time from 5.29 s to 0.86–0.89 s for 16,000,000
instructions (about 6.1x). Batching is a separate acceleration for stable idle
loops.

The reference network uses the configured 16 MHz HSE, each board's selected FDCAN
controller, and clock-timed ADC sequences with one DMA item per rank. Older results
from incorrect clock/CAN topology or unobservable ADC-to-DMA work are not
comparable.

## Reproduce the benchmark

With the PER firmware checkout, use the six-board network directly (this is
the reference workload tabulated above). Use a fresh optimized build and the
same firmware, compiler, power mode, and command line for every comparison:

```bash
make BUILD_DIR=build-release BUILD_TYPE=Release IPO=ON test

for run in 1 2 3; do
  /usr/bin/time -p ./build-release/fil run-network \
    configs/networks/per_vehicle.json \
    --duration-ms 1000 \
    --max-instructions 50000000 \
    --quantum 1024 \
    --strict-mmio
 done
```

Append `--adc-decimation 8` (or `32`) to reproduce the decimated rows.
Record wall time and verify the emulator result: expect
`time_ns: 1000000187`–`1000000249` with `cycles >= instructions` and
per-board counts set by firmware CPI (loads 2, calls/branches 3-4, flash
stalls at high clocks). Throughput is `simulated seconds / wall seconds`.
Keep tracing off: instruction tracing writes one record per instruction and
intentionally disables batching.

The single-board controls are:

```bash
/usr/bin/time -p ./build-release/fil run configs/boards/front_driveline.json \
  --duration-ms 1000 \
  --max-instructions 50000000 \
  --strict-mmio
/usr/bin/time -p ./build-release/fil run configs/boards/dashboard.json \
  --duration-ms 1000 \
  --max-instructions 50000000 \
  --strict-mmio
```

Without the PER firmware checkout, the Makefile benchmark target builds
first and generates its own synthetic fixtures (no PER needed):

```bash
make BUILD_DIR=build-release BUILD_TYPE=Release benchmark BENCH_REPS=3
```

It asserts `cycles >= instructions` and the 1 s deadline window on every
case, and prints instructions / cycles / CPI / wall time / throughput.
Treat it as a smoke test: its idle-loop fixtures behave nothing like ADC+
DMA motor-control firmware, so do not compare its wall times with the
network rows above.

## How the optimizations work

Most mechanisms reduce host overhead while dispatching every target instruction.
Loop batching is the only mechanism that accounts for proven repeated instructions
without dispatching each one.

### CPU and memory hot path

- **Decoded-instruction cache.** A 16,384-entry direct-mapped cache stores the PC,
  raw encoding, width, and decoded instruction. Hits execute the cached object
  directly; IT-state adjustment makes a copy only when needed. Entries are tagged
  with the executable-memory generation, so loading or modifying executable bytes
  invalidates stale code. Each entry memoizes its static pipeline cost at decode
  time, so stepping and burst prediction pay one load instead of a ~90-case
  switch (divides resolve data-dependently via a flag).
- **Real-timing pacing.** The stepper charges per-class pipeline cycles plus a
  taken-branch refill on discontinuous PCs, and the board wrapper adds the
  simplified-ART flash stall from cached FLASH_ACR LATENCY/prefetch state.
  Clock and ACR values are cached with generation checks, and loop proofs pin
  both, so timing work is a few loads and compares per instruction.
- **Exact lockstep bursts.** Equal-cost boards step up to 64 rounds per gate;
  prediction covers direct branches exactly (targets evaluate from pre-state,
  fault paths refuse), and the gate compares clock/numerator pairs with one
  ns division per round. Proof work is skipped when neither batching nor spin
  detection can consume it, which also keeps no-batch bursts firing.
- **Outlined cold paths.** The ~90 KiB loop-observation reset and the 250-byte
  MMIO-restart snapshot are noinline/heap-boxed so the hot frames carry no
  stack probes.
- **Compact stepping.** `stepFast()` returns counters, instruction metadata, and a
  stop reason instead of copying the full register file and constructing diagnostic
  text after every successful instruction. Fault and stop paths still materialize
  the complete snapshot.
- **Specialized word loads.** The dominant `LDR` form has a dedicated execution arm
  rather than repeatedly selecting width, signedness, and load/store direction.
  Addressing, writeback, PC loads, and faults retain the common semantics.
- **Cheap interrupt rejection.** `SystemControl` maintains a summary of enabled
  pending work. A positive summary performs full exception selection; external
  IRQ selection scans only bits in `pending & enabled` instead of all 240 lines.
- **Compact memory results.** Successful `MemoryResult<T>` values are stored inline.
  A structured `BusFault` is allocated only on failure, avoiding a large fault
  object or generic variant work on normal accesses.
- **Region lookup cache.** A 256-entry cache avoids walking the sorted memory map.
  Every hit is checked for full containment, so mappings that share a cache slot
  safely fall back to the authoritative map.
- **Direct 32-bit backing access.** Common RAM/ROM reads and RAM writes bypass
  generic alias/MMIO dispatch after full range and permission checks. The write
  path still records reversible mutations, DMA provenance, side-effect generations,
  and executable-memory invalidation. Other cases use the generic implementation.
- **Branch layout hints.** C++20 likelihood attributes mark stable invariants such
  as successful execution and cache hits. Workload-dependent branches are left to
  PGO.

### Scheduler and event path

- **In-place accounting.** Successful instructions update aggregate counters
  directly; `BoardRunResult`, optional faults, strings, and register snapshots are
  built only at boundaries.
- **Split boundary settlement.** A compact predicate keeps exception/reset handling
  out of the ordinary instruction path. The non-inlined slow path retains complete
  exception and reset behavior.
- **Exact lockstep bursts.** Equal-time boards can execute up to 64 exact rounds
  before returning to the general scheduler. A burst stops for events, exceptions,
  budgets, failures, clock divergence, or a usable loop proof.
- **Owner-aware events.** Every callback belongs to one board or the shared domain.
  Per-owner horizons and clocks let a local event invalidate only its lane, while a
  shared callback remains a global boundary. A serial fast path avoids owner-map
  and thread-local work when concurrency is disabled.
- **Contiguous lane data.** The world builds a stable `Board*` view once per run and
  reuses planner arrays at each frontier, avoiding repeated ownership traversal and
  allocation.
- **Transactional workers.** The opt-in `--transactional-slices` mode checkpoints
  CPU, RAM, system, event-clock, and loop state before parallel lane epochs. Any
  MMIO or event escape rolls every lane back and resumes exact scheduling. Tracing
  disables this mode because speculative records are not reversible.

### Exact-state loop batching

Polling and RTOS idle loops often return to the same architectural state until the
next interrupt or device event. `fil` batches such a loop only after proving all of
the following:

1. execution reaches the same backward-flow boundary with identical integer, FP,
   mask, stack, xPSR, and IT state;
2. directly backed memory has the same values at the boundary;
3. the iteration performed no MMIO or other observable side effect; and
4. a conservative time horizon permits another complete iteration before an event,
   serviceable interrupt, run deadline, instruction budget, or world frontier.

A fixed 256-entry observation table stores candidate states. Generation and revision
numbers make slot replacement fail closed. Calls and standard returns are excluded
before proof work because a lower destination address is not necessarily a loop
backedge.

Memory proof uses an 8,192-entry reversible byte-mutation journal. Restored or
idempotent writes are allowed. If the journal cannot cover the candidate, batching
is rejected. A small two-hash read footprint allows unrelated external/DMA writes;
collisions can only reject acceleration, never admit an unsafe proof. Any MMIO access
advances a separate generation and invalidates the candidate.

The scheduler computes the earliest observable horizon across every board and the
shared event queue. It converts that nanosecond horizon to an iteration count with
checked integer arithmetic, then validates the proof once immediately before
applying the batch. Logical instruction/cycle totals and fractional clock time are
advanced exactly as repeated execution would be.

If any proof is missing or stale, execution continues one instruction at a time.
`--no-loop-batching` disables acceleration, `--trace-instr` requests exact
per-instruction records, and `--detect-spin` reports a stable loop instead of
skipping it.

### Diagnostics and peripheral work

- **Disabled-history mode.** Without `--trace`, the CLI disables trace, ADC sample,
  and DMA transfer retention. Target-visible MMIO, interrupts, conversion results,
  and CAN traffic remain active.
- **Lazy ADC conversion.** A continuous single-rank ADC with no interrupt, DMA,
  callback, trace, or history observer keeps only its next conversion deadline.
  On the next MMIO observation it materializes the latest due conversion at the
  exact timestamp and preserves phase. Observable, single-shot, or multi-rank
  conversions continue to schedule every event.
- **DMAMUX route cache.** ADC requests use a generation-tagged route table instead
  of scanning all selectors. Any selector write rebuilds the complete table.
- **ADC scan decimation (`--adc-decimation N`, opt-in fidelity tradeoff).**
  Continuous-mode ADCs keep 1 of every N scans: the conversion schedule
  still advances on time (no clock drift, kept scans land on their exact
  deadlines), but skipped scans perform no DR/ISR writes, raise no EOC/EOS,
  trigger no DMA or interrupt callbacks, and record no sample history or
  trace. Whole decimated gaps collapse into a single event whose span
  repeats the live scan period, so event-queue work also divides by N.
  Decisions are per scan, preserving multi-rank DMA alignment, and
  single-shot conversions are never skipped. Firmware observes sample data
  coarsened to one scan per N periods (DR holds the last kept value) and
  ADC timing rewrites during a gap take effect at the next kept scan at the
  latest. Factor 1 (default) preserves current behavior exactly. On the
  six-board vehicle network (continuous ADC+DMA on every board), N=8 reaches
  about 2.2x end-to-end (0.79x realtime) and N=32 about 3.0x (1.09x realtime,
  faster than realtime) on Apple M3 / Release+IPO, with instruction, cycle,
  and `time_ns` totals unchanged apart from the decimated ISR stream.

## Build optimization

Release builds enable IPO/LTO when supported. Debug and sanitizer builds remain
unoptimized and do not use IPO. Clang PGO can additionally optimize dispatch,
branch layout, and scheduler code for representative workloads.

Run the staged Makefile workflow with Clang/AppleClang and a representative
workload. The example network needs the external PER firmware checkout; replace the
training arguments with a local workload if needed:

```bash
make clean-pgo
make pgo-generate PGO_CXX=clang++
make pgo-train PGO_TRAIN_ARGS='run-network configs/networks/per_vehicle.json --duration-ms 1000 --max-instructions 50000000 --quantum 1024 --strict-mmio'
make pgo-merge
make pgo-test
```

The instrumented Release build (IPO off) and optimized profile-use Release build
(IPO on) use separate `build-pgo-generate/` and `build-pgo/` directories. Training
writes `fil-<pid>.profraw` files under the instrumented directory. The merged
profile defaults to `build-pgo-generate/fil.profdata`. The Makefile uses
`llvm-profdata` on `PATH`, falling back to `xcrun llvm-profdata` on macOS, and
checks that its reported LLVM major version matches `PGO_CXX`. Keep both tools from
the same LLVM release; override `PGO_CXX` and `LLVM_PROFDATA` together if needed.
`make pgo-use` builds with the merged profile, and `make pgo-test` also runs CTest.
Set `PGO_PROFILE` to move the merged profile.

Start with `make clean-pgo` to avoid mixing stale profiles. Profiles are tied to the
compiler, instrumented binary, and source; use the same `PGO_CXX` for both phases
and retrain after material code/toolchain changes. CMake rejects GCC PGO and
sanitizer/PGO combinations. See `make help` for all staged PGO targets.

## Correctness checks

Performance tests assert semantic equivalence rather than host timing. The suite
compares batched and exact runs across instruction budgets, phase-mismatched clocks,
mid-run callbacks, SysTick exception entry/return, full CPU state, counters, and
normalized traces. It also covers executable-write cache invalidation, memory-cache
collisions, expired mutation journals, MMIO rejection, call/return loop filters,
worker rollback, event ownership, and spin-detection precedence.

See [Testing](testing.md) for the Release, sanitizer, coverage, and external-firmware
validation commands.
