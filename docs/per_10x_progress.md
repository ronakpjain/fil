# PER firmware 10× runtime goal

Status: in progress; the 10× target has not been achieved.

## Current user requirements

The user clarified that incremental percentage gains are insufficient. Improve
FIL's own JIT; do not introduce LLJIT. Remove all existing host execution
caches and do not add new caches. Make reviewable commits for large changes.
Use inline-capacity/stack storage for bounded compiler lists and bitsets for
boolean sets. These requirements supersede the earlier cache-tuning direction
described below. Generated machine code and the descriptors it references still
need lifetime ownership; that ownership must not implement reuse lookups,
hit/miss policy, eviction or memoized execution. Modeled firmware FLASH/ART
register behavior remains part of the guest hardware contract.

The register-allocation foundation is committed as `d9424c3`. The x86 emitter
keeps three selected ordinary guest registers in host registers across
helper-free integer regions, with spills on every precise prefix exit and
helper boundary. The differential test sweeps all credit and instruction cuts,
including a store between regions. All 592 tests passed before cache removal.
A matched 1000 ms PER pair preserved counters/PCs but showed no meaningful
speedup (1.549 s reference, 1.564 s candidate). This is compiler groundwork,
not achievement of the performance target.

A ceiling experiment disabled certificate discovery and write journaling in
committed owner slices. Three 1000 ms runs took 1.344, 1.380 and 1.407 s.
This removes a whole overhead layer yet remains far from the target. Its
counters differed from the capsule baseline, so it was rejected as a production
change and fully restored. The larger redesign must eliminate per-operation
dispatch/materialization/helper costs inside compiled loops, not just remove
discovery overhead.

Cache removal is staged. Commit `8b43c8e` removes MicroJit's pure,
native and trace compilation-result lookup/reuse caches (each compile now emits
a new section), the memory-region lookup cache, the MMIO-route cache and the
pending-exception selection cache and the micro-kernel admission verdict cache.
Bounded temporary compiler metadata uses
`std::array` and `std::bitset`; owned trace prefix records also use inline arrays.
The recent idempotent-certificate lookup tuning and its test were discarded
because the certificate machinery is to be removed.

All 591 tests pass after this stage (the discarded phase-tuning test accounts
for the count dropping from 592). A 1000 ms real-PER pair matched instructions,
cycles, callback counts and final PCs. Wall time regressed from 1.585 s to
2.001 s; this is an incomplete architectural migration, not a speedup. Results
are in `/tmp/fil-per-10x/cache-removal-per.txt`. Retaining removed caches to
recover that interim performance would violate the user's direction; finish
the direct-execution replacement instead.

Remaining: CPU instruction decode cache, prepared block and trace caches,
period/transition replay caches and board
timing caches. Replace their execution responsibilities with direct compiled
program sections, then complete long-loop execution and guarded memory lowering.
No claim that all caches are removed or that 10× has been reached is justified.

The workload is the six-board `configs/networks/per_vehicle.json` network,
loading firmware from `~/coding/PER/monorepo/firmware/output`. Preserve ADC
decimation 1, strict MMIO, quantum 1024 and the existing 100000 ns cross-board
window for matched comparisons. Do not increase the window or reduce peripheral
work to meet the target. Startup and JIT compilation count toward runtime.

## Starting point (2026-11-02)

Source revision: `ec6a6e38d1e2b2b97880114f55298b6488298edc`.
The starting `build-pgo/fil` executable was preserved at
`/tmp/fil-per-10x/baseline`, SHA-256
`b38fe9082c3878fd6224a04254e649bbf09f101cfaf578376c4f51f99aaca225`.
Host: AMD Ryzen 9 4900HS, x86-64, 16 logical CPUs.

An initial isolated 1000 ms run took 1.706 s and completed 268870014
instructions / 1017313085 cycles. Three 5000 ms runs each completed
1346846529 instructions / 5097313076 cycles, with wall times
8.612, 7.470 and 8.860 s. These longer timings overlapped profiling/build
activity and are preliminary; use isolated, alternating matched measurements
for acceptance rather than treating these numbers as a final baseline.

The benchmark now accepts an explicit speedup gate:

```bash
bash tools/bench_full_network.sh \
  --binary build-pgo/fil \
  --reference-binary /tmp/fil-per-10x/baseline \
  --duration-ms 5000 --reps 5 --min-speedup 10 --min-throughput 0.001
```

The small throughput threshold leaves the relative 10× gate in charge; it
does not change the workload. The benchmark checks final PCs and aggregate
counters between paired runs. These checks alone do not prove full behavioral
equivalence; runtime changes also require relevant differential tests of CPU,
RAM, events and peripheral effects.

## Profiling

Callgrind rejected an instruction emitted by the CPU-tuned baseline in
`tryLowerMicroInst`. A separate current-source build, `build-per-profile`,
uses `/usr/bin/c++`, Release, debug symbols, native tuning off, IPO off.
Its completed 100 ms full-network profile is in
`/tmp/fil-per-10x/profile.callgrind` (stdout/stderr alongside it).
Use this profile to choose the next runtime change; measure final performance
using matched optimized builds, without profiling or builds running alongside.

The fresh profile recorded 6.086 billion host instructions. Worker-loop code
accounts for about 22.5% (including atomic operations), detailed reversible
execution 6.0%, RAM-input compatibility validation 5.1%, backed multiword
writes 3.2%, and native trace dispatch 2.3%. Callgrind instruction counts are
not wall-time shares, especially for worker spinning. This run confirms the
next investigation should cover certificate/discovery overhead and memory
transfers as well as dispatch, rather than only emitted arithmetic.

The first runtime change bypasses the native-trace prefix binary search when
the full available trace fits its exact existing cycle credit. Partial and
zero-instruction cuts retain the original search. All 590 CTest tests pass.
An initial timing series overlapped tests and is unsuitable as speedup evidence;
an isolated alternating five-pair series is recorded at
`/tmp/fil-per-10x/admission-isolated.txt`.
That series measured reference median 1.771 s and candidate median 1.663 s
(1.065×). All five pairs matched instructions, cycles, event callback counts
and final PCs. Individual wall times varied, so this is preliminary evidence
of a small gain, not evidence of the requested 10× improvement. Longer matched
measurements and further runtime work remain required.

Benchmark validation so far: shell syntax, missing/empty/negative speedup
arguments, a passing permissive speedup threshold, and a failing 10000×
threshold against the same binary. Both real paired smoke runs produced
matching instruction/cycle counts and final PCs.

## CPU-phase lookup before RAM validation

The next change defers `ramInputsCompatibleSince` for an idempotent-period
certificate until an exact CPU phase matches. Cheap generation guards still
run first; every usable phase still validates against the original journal
checkpoint before replay. A CPU miss may retain an unchecked certificate,
but it never advances the checkpoint or replays it. The added regression
`BoardTest.CachedPeriodRevalidatesRamAfterCpuPhaseMiss` changes a RAM input
during an excursion and verifies rejection when the matching phase returns.
The existing temporary-phase-miss reuse test also passes.

The full existing 590-test suite passed after the runtime change; the new
regression passed separately. Five isolated alternating PER pairs (1000 ms)
matched counters and final PCs. Reference median was 1.653 s, combined
candidate median 1.604 s (1.031×); results are in
`/tmp/fil-per-10x/phase-benchmark.txt`. Timing variation remains larger than
the incremental effect, so this is not a claim of a reliable additional gain
over the first optimization, and the 10× goal remains unmet.

An experimental increase of the discovery probe table from 256 to 4096 entries
was rejected and reverted. It produced 268870271 instructions / 1017313087
cycles instead of 268870014 / 1017313085 at 1000 ms. Changing discovery cadence
can therefore expose certificate/timing differences; further work on that
path must investigate those differences rather than accept faster but unequal
runs. The retained table still has 256 entries.
