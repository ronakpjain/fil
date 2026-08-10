# Performance

`fil` aims to run representative firmware at or above real time without changing
observable target behavior. Host timings are measured with `/usr/bin/time`; simulated
time comes from emulator output. The one-cycle-per-instruction model is deterministic,
but it is not an estimate of STM32 pipeline timing.

## Reference results

The reference workload is a one-second run of the six-board
`configs/networks/per_vehicle.json` network. All modes stop at exactly
1,000,000,000 ns with 96,000,000 logical instructions and cycles (16,000,000 per
board).

| Host mode | Build | Loop batching | Median wall time | Throughput |
|---|---|---:|---:|---:|
| AC power | Release + IPO | on | 0.87 s | 1.15x real time |
| AC power | Clang PGO | on | 0.73 s | 1.37x real time |
| AC power | Release + IPO | off | 3.33 s | 0.30x real time |
| Battery, macOS Low Power Mode | Release + IPO | on | 1.57 s | 0.64x real time |
| Battery, macOS Low Power Mode | Clang PGO | on | 1.33 s | 0.75x real time |

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

Record wall time and verify the exact emulator result:

```text
stop: time-budget
instructions: 96000000
cycles: 96000000
time_ns: 1000000000
board dashboard:       instructions=16000000
board main_module:     instructions=16000000
board torque_vector:   instructions=16000000
board a_box:           instructions=16000000
board front_driveline: instructions=16000000
board rear_driveline:  instructions=16000000
```

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
  invalidates stale code.
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
