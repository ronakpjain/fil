# Performance

`fil` aims to run representative firmware at or above real time without changing
observable target behavior. Host timings are measured with `/usr/bin/time`; simulated
time comes from emulator output. Pacing follows the documented real-timing
model (Cortex-M4 per-class pipeline cycles plus STM32G4 flash wait states),
not one cycle per instruction; see [Real timing audit](real_timing_audit.md).
Under this model `cycles >= instructions` and CPI is reported alongside
throughput (`simulated seconds / wall seconds`).

## Reference results

### Real-timing model (current)

The runnable reference is `tools/bench_real_timing.py`, which needs no PER
checkout: it generates hand-assembled Thumb fixtures (an idle `4x NOP + B`
flash loop at reset 16 MHz, and a variant that programs `FLASH_ACR=0x304`
then a 170 MHz HSE-PLL before spinning the same loop) and runs 1 s
single-board and synthetic six-board workloads, batching on and off.
All runs stop at 1,000,000,000 ns (pll170 overshoots by 1 ns of atomic batch
completion); `cycles >= instructions` holds everywhere.

Measured 2026-09-25 on Apple M3 / 8 cores / 8 GB / macOS 27.0, Homebrew Clang
23.1.1, Release + IPO, AC power, median of 3 (`python3
tools/bench_real_timing.py ./build-release/fil --reps 3`):

| Case | Instructions | Cycles | CPI | Median wall time | Throughput |
|---|---|---:|---:|---:|---:|
| idle16 x1, batching | 10,000,000 | 16,000,000 | 1.60 | 0.003 s | ~343x |
| idle16 x1, no-batch | 10,000,000 | 16,000,000 | 1.60 | 0.181 s | 5.53x |
| idle16 x6, batching | 60,000,000 | 96,000,000 | 1.60 | 0.004 s | ~266x |
| idle16 x6, no-batch | 60,000,000 | 96,000,000 | 1.60 | 1.493 s | 0.67x |
| pll170 x1, batching | 70,833,277 | 169,999,856 | 2.40 | 0.002 s | ~440x |
| pll170 x1, no-batch | 70,833,277 | 169,999,856 | 2.40 | 1.135 s | 0.88x |

The idle loop costs 5 instructions / 8 cycles per iteration (taken `B` pays
the +2 pipeline refill); the pll170 loop costs 5 / 12 (plus 4 ART-miss
cycles on the taken branch at LATENCY=4). Against the pre-model baseline
measured on the same host and binary configuration (1.00 CPI: x6 no-batch
1.697 s / 0.59x, pll170 no-batch 2.858 s / 0.35x), realistic pacing executes
fewer instructions per simulated second, so fixed-simulation-time throughput
improved even though per-instruction host cost rose (memoized cycle LUT and
cached clock/ACR keep the added timing work to a few loads and compares).
The six-board no-batch interpreter path (0.67x) remains below real time and
is the standing optimization target; batching covers it by 250x or more on
loop-dominated firmware.

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

Without the PER firmware checkout, run the runnable reference (needs only the
just-built binary):

```bash
python3 tools/bench_real_timing.py ./build-release/fil --reps 3
```

It generates its own fixtures, asserts `cycles >= instructions` and the 1 s
deadline window on every case, and prints instructions / cycles / CPI /
wall time / throughput. With the PER checkout, use the six-board network below
and expect `cycles >= instructions` with per-board instruction counts that
vary with firmware CPI instead of the fixed stale 16M/board.

Use a fresh optimized build and the same firmware, compiler, power mode, and command
line for every comparison:

```bash
cmake -S . -B build-release -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DFIL_ENABLE_IPO=ON \
  -DFIL_BUILD_TESTS=ON
cmake --build build-release
ctest --test-dir build-release --output-on-failure

for run in 1 2 3; do
  /usr/bin/time -p ./build-release/fil run-network \
    configs/networks/per_vehicle.json \
    --duration-ms 1000 \
    --max-instructions 50000000 \
    --quantum 1024 \
    --strict-mmio
done
```

Record wall time and verify the emulator result. Under the retired 1-CPI
model this was exactly `instructions: 96000000 / cycles: 96000000 /
time_ns: 1000000000` at 16M/board; under the real-timing model expect
`time_ns: 1000000000` with `cycles >= instructions` and per-board counts set
by firmware CPI (loads 2, calls/branches 3-4, flash stalls at high clocks).

Throughput is `simulated seconds / wall seconds`. Keep tracing off: instruction
tracing writes one record per instruction and intentionally disables batching.
Use `--no-loop-batching` to measure the interpreter and scheduler without loop
acceleration.

The single-board control is:

```bash
/usr/bin/time -p ./build-release/fil run configs/boards/g4_testing.json \
  --duration-ms 1000 \
  --max-instructions 50000000 \
  --strict-mmio \
  --no-loop-batching
```

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

## Build optimization

Release builds enable IPO/LTO when supported. Debug and sanitizer builds remain
unoptimized and do not use IPO. Clang PGO can additionally optimize dispatch,
branch layout, and scheduler code for representative workloads.

```bash
cmake -S . -B build-pgo-generate -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DFIL_ENABLE_IPO=OFF \
  -DFIL_BUILD_TESTS=OFF \
  -DFIL_PGO_GENERATE=ON
cmake --build build-pgo-generate

LLVM_PROFILE_FILE=/tmp/fil-per.profraw \
  ./build-pgo-generate/fil run-network configs/networks/per_vehicle.json \
    --duration-ms 1000 --max-instructions 50000000 \
    --quantum 1024 --strict-mmio
xcrun llvm-profdata merge -output=/tmp/fil-per.profdata /tmp/fil-per.profraw

cmake -S . -B build-pgo -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DFIL_ENABLE_IPO=ON \
  -DFIL_BUILD_TESTS=ON \
  -DFIL_PGO_PROFILE=/tmp/fil-per.profdata
cmake --build build-pgo
ctest --test-dir build-pgo --output-on-failure
```

Use `llvm-profdata` directly on non-Apple systems. Profiles are tied to the compiler
and instrumented binary; retrain after material source or toolchain changes. PGO
generation/use modes are mutually exclusive and cannot be combined with sanitizers.

## Correctness checks

Performance tests assert semantic equivalence rather than host timing. The suite
compares batched and exact runs across instruction budgets, phase-mismatched clocks,
mid-run callbacks, SysTick exception entry/return, full CPU state, counters, and
normalized traces. It also covers executable-write cache invalidation, memory-cache
collisions, expired mutation journals, MMIO rejection, call/return loop filters,
worker rollback, event ownership, and spin-detection precedence.

See [Testing](testing.md) for the Release, sanitizer, coverage, and external-firmware
validation commands.
