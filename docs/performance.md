# Performance

`fil` is intended to run useful firmware workloads at or above real time while
preserving deterministic target behavior. Performance measurements use simulated
time from the emulator output and host wall time from `/usr/bin/time`; they are not
estimates of STM32 instruction timing.

## Current result

Before loop batching was enabled, the `g4_testing` one-second workload on the
development host improved from 5.29 s to 0.86--0.89 s of host wall time. That is
about a 6.1x core-interpreter speedup and about 1.15x real-time throughput. The run
executes 16,000,000 target instructions and reaches exactly 1,000,000,000 ns of
simulated time. These numbers are specific to that host, compiler, firmware build,
and the one-cycle-per-instruction timing model.

The result above has idle-loop batching disabled, so it measures the optimized
instruction-by-instruction path. Exact-state idle-loop batching is an additional
optimization for firmware that spends substantial time polling in a stable idle
loop.

The corrected six-board PER workload currently runs below real time after making
continuous ADC sequences and their DMA transfers observable. Three consecutive
one-second runs on the development host took **1.82 s, 1.77 s, and 1.85 s**
(median 1.82 s, about **0.55x real time**). Disabling loop batching took 5.52 s on
the same build, so the causality-bounded batching path is about **3.0x faster** for
this workload. Both modes reported the same exact result:

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

Acceleration counters for this workload show approximately 68.3 million logical
instructions represented by 263,664 proven-loop batches, while about 27.7 million
instructions still enter the interpreter across 5.09 million scheduler rounds.
Roughly 244,000 event callbacks per simulated second repeatedly terminate global
lookahead. These counters identify interpreter dispatch and globally synchronized
event handling—not the 96 MHz aggregate target rate itself—as the remaining limit.

These measurements include the firmware-matching 16 MHz HSE, connect each board's
actual VCAN controller (FDCAN2 except A-box FDCAN1), and service one circular DMA
item for every clock-timed ADC rank. Earlier 1.52x and 1.01x results respectively
used an incorrect clock/CAN topology and then omitted continuous ADC-to-DMA work;
those results are not comparable.

These host-specific measurements used Release plus IPO, strict MMIO, no trace,
and the current external PER ELFs. Do not compare the sum of all boards' target
cycles directly with wall time: the boards have independent virtual CPU timelines
and advance concurrently in simulated time.

## Reproduce the benchmark

The checked-in board configuration expects the `g4_testing` ELF in the sibling
PER checkout described in the README. Use an optimized build explicitly when
comparing hosts:

```bash
cmake -S . -B build-release -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DFIL_ENABLE_IPO=ON \
  -DFIL_BUILD_TESTS=ON
cmake --build build-release

for run in 1 2 3; do
  /usr/bin/time -p ./build-release/fil run configs/boards/g4_testing.json \
    --duration-ms 1000 \
    --max-instructions 50000000 \
    --strict-mmio \
    --no-loop-batching
done
```

Record both the `real` value from `/usr/bin/time` and these emulator fields:

```text
stop: time-budget
instructions: 16000000
cycles: 16000000
time_ns: 1000000000
unknown_mmio_addresses: 0
```

`simulated seconds / wall seconds` is the real-time multiple. Keep instruction
tracing off for throughput measurements because writing one record per instruction
is intentionally expensive. Use the same compiler, build type, ELF, command line,
and loop-batching setting for before/after comparisons. `--no-loop-batching` is
what makes this command reproduce the pre-batching core-interpreter measurement.

For the six-board workload, use the same measurement wrapper around:

```bash
./build-release/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 1000 \
  --max-instructions 50000000 \
  --quantum 1024 \
  --strict-mmio
```

## Technique inventory

The speedups fall into three layers. This table is the complete inventory of
performance-specific mechanisms currently implemented; ordinary container
`reserve()` calls and similar startup-only housekeeping are intentionally omitted.

| Layer | Technique | Avoided work | Correctness boundary |
|---|---|---|---|
| CPU | Decoded instruction cache | Repeated fetch and decode | PC plus executable-memory generation |
| CPU | Copy-free cache hits | Per-instruction `DecodedInstruction` copies | Stable fixed-size cache entry; IT adjustment still copies |
| CPU | `stepFast()` | Full register snapshots and diagnostic strings on success | Full diagnostics materialized on stop/fault |
| Cortex-M | Pending-interrupt summary | Calling exception selection and scanning NVIC words after every instruction | Recompute the summary only when pending/enable state mutates; full priority selection still runs for every positive summary |
| Cortex-M | Sparse NVIC scan | Testing all 240 external IRQs for a pending candidate | Scan only `pending & enabled`; priority rules unchanged |
| Memory | Compact `MemoryResult` | Carrying a large inline `BusFault` on successful accesses | Fault allocation occurs only on failure |
| Memory | Region dispatch cache | Ordered mapping walk for each fetch/data/MMIO access | Full containment check before accepting a hit |
| Build | Release plus IPO/LTO defaults | Unoptimized hot path and translation-unit barriers | Debug and sanitizer builds remain unoptimized/non-IPO |
| Board/world | Exact-state loop batching | Re-executing proven identical idle iterations | CPU state, reversible RAM journal, MMIO generation, and causal horizon |
| World | Reused batching planner storage | Per-frontier heap allocation | Storage is sized once per run and cleared before reuse |
| World | In-place successful-step accounting | Constructing/copying `BoardRunResult`, register arrays, optional faults, and strings per interpreted instruction | Full result and diagnostic materialization remains on non-success boundaries |
| World | Event ownership provenance | Invalidating unrelated board-local lookahead after a callback | Shared callbacks still invalidate every lane; nested local callbacks inherit their board owner |
| World | Transactional lane workers | Serial MMIO-free instruction epochs across independent boards | CPU/RAM/system state is checkpointed; any MMIO/event escape rolls every lane back; opt-in while commit rate is tuned |
| Diagnostics | Disabled trace/history fast mode | Retaining and serializing unrequested records, ADC samples, and DMA request logs | Any requested observer reenables exact events/data retention |
| ADC | Lazy unobserved continuous conversion | One callback per conversion period | Only continuous conversions with no interrupt/callback/trace/history observer |
| DMAMUX | Generation-tagged request-route cache | Scanning all 16 selectors for every ADC conversion | Any selector write invalidates and rebuilds the complete route table |

The first seven techniques preserve one host dispatch per target instruction. Loop
batching and lazy ADC conversion are conservative event-elision techniques: they
skip host work only after proving that the omitted intermediate activity cannot be
observed. The following sections describe each guard in detail.

## Instruction hot-path techniques

The following optimizations do not skip target instructions.

### Self-modifying-code-safe decoded instruction cache

Previously, every step fetched instruction halfwords through `MemoryBus` and ran
the decoder even when a loop had executed the same PC thousands of times.
`CortexM4` now has an 8,192-entry direct-mapped cache indexed by Thumb PC. An entry
stores the PC, instruction width, raw encoding, and decoded instruction, so a hit
avoids both memory fetch and decode.

Every entry is tagged with `MemoryBus::executionGeneration()`. Adding a mapping,
loading executable bytes, or writing executable backing storage advances that
generation, conservatively invalidating all old entries. The PC and generation
must both match before a cache entry is used, so direct-map collisions and
self-modifying code fall back to fetch and decode rather than using stale code.
The self-modifying-code regression described below proves that an executable write
replaces a previously cached instruction on the next execution.

### Copy-free decoded-cache hits

Avoiding decode alone still left a potential per-step copy of the comparatively
large `DecodedInstruction`. On a cache hit, `stepFast()` now executes through a
pointer to the decoded value stored in the cache entry. It creates a temporary copy
only for the uncommon IT-block case that must suppress implicit flag updates.

The pointed-to entry belongs to the fixed-size CPU cache and remains alive for the
step. PC and execution-generation validation happens before taking the pointer,
which gives this optimization the same collision and executable-write guardrails
as the instruction cache itself. CPU instruction tests cover both narrow and wide
cached execution, while the executable-write regression exercises the invalidation
boundary.

### Lightweight `stepFast()` CPU path

The original public step result contained a full register snapshot, optional bus
fault, and diagnostic message, even when the instruction completed normally.
`CortexM4::stepFast()` instead returns a compact `FastStepResult` on the hot path.
Board and world loops accumulate its stop reason, address, width, instruction
count, and cycle count without copying the full register file or constructing a
diagnostic string after every instruction. A fault, invalid state, unsupported
instruction, breakpoint, or other stop obtains the full structured snapshot from
`lastDiagnostic()`. The public `step()` API remains available and constructs the
traditional full result when a caller requests one. Thus the optimization changes
result materialization, not execution semantics or failure detail. The CPU, board,
startup, exception, and scheduler fixture tests all pass through this fast path.

### Sparse NVIC pending-interrupt bit scanning

The earlier check considered every external IRQ after each target instruction,
although almost all instructions run with no serviceable interrupt. The NVIC check
now first tests whether PendSV, SysTick, or any enabled external-pending
word contains work. If not, it returns immediately. When external interrupts are
pending, it iterates only the set bits in `pending & enabled` with a bit scan,
instead of checking all 240 external IRQ numbers after every instruction. Priority,
masking, active-exception, and exception-number tie-break rules are unchanged.
The disabled-pending-IRQ regression described below is important here: pending is
not sufficient unless the matching enable bit is also set.

### Compact `MemoryResult` success representation

Previously, every successful fetch or data access carried storage large enough for
the complete structured `BusFault`. Successful `MemoryResult<T>` values now stay
inline in a small variant. The much larger structured `BusFault` is held behind a
`unique_ptr` and is allocated only
on the uncommon failure path. Copying remains supported; a copied fault is deep
copied. Normal instruction fetches and data accesses therefore avoid carrying a
large fault object through every successful return. Callers still receive the same
typed fault fields on failure; memory-bus tests exercise successful values and
unmapped, permission, overflow, and cross-region faults, and sanitizers cover the
failure-only ownership path.

### 256-entry memory-region dispatch cache

Region lookup originally walked the sorted mapping list for each instruction
fetch, stack access, data access, and MMIO dispatch. `MemoryBus` now keeps a
256-entry cache indexed by the high byte of the 32-bit target address. A hit is
accepted only if the cached region still contains the complete
lookup address; otherwise the bus performs the normal ordered-region search and
updates the slot. This cheaply covers the stable flash, SRAM, peripheral, and
system address windows used by firmware.

The containment recheck is the correctness guardrail when multiple mappings share
one high-byte slot. Cache misses still use the authoritative ordered map, and the
subsequent access retains all permission, whole-range, alias-depth, and MMIO-width
validation. Memory map tests cover overlapping and wrapping map rejection,
cross-region accesses, aliases, and routed MMIO.

### Release IPO/LTO defaults without changing Debug or sanitizers

An unspecified single-config CMake build used to risk benchmarking an unoptimized
interpreter. CMake now selects `Release` when no build type was specified.
`FIL_ENABLE_IPO` defaults to `ON`, and supported compilers use
interprocedural optimization for Release, RelWithDebInfo, and MinSizeRel targets.
This lets the compiler optimize across CPU, memory, and board-loop translation-unit
boundaries.

The default changes only a previously unspecified single-config build. An explicit
`Debug` remains Debug, multi-config generators retain their selected configuration,
and IPO is automatically omitted whenever ASan or UBSan is enabled. Specify the
build type in benchmark and diagnostic scripts rather than relying on an old CMake
cache. Normal Release and explicit Debug sanitizer builds are both part of the
validation workflow below.

### Cache-invalidation and disabled-pending-IRQ regressions

Two focused regressions protect the correctness assumptions behind the largest
hot-path changes:

- `invalidatesDecodedInstructionsAfterExecutableWrites` in
  `tests/unit/cpu_step_test.cpp` executes `MOVS r0, #1` from executable RAM, caches
  it, overwrites the same address with `MOVS r0, #2`, and proves the next execution
  observes `2`.
- `modelsNvicAndScb` in `tests/unit/cortexm_test.cpp` pends an external IRQ whose
  enable bit is clear and proves `nextPending()` ignores it. The same test checks
  enabled delivery, PRIMASK, BASEPRI, priority tie-breaking, and active-exception
  preemption.

These are semantic regressions, not timing assertions, so they run identically in
Release and sanitizer configurations and do not become flaky with host load.

## Exact-state idle-loop batching

FreeRTOS and bare-metal firmware often execute a short polling loop until the next
SysTick or external event. `fil` can recognize a repeat only when the full
architectural CPU state returns to the same loop boundary, directly backed memory
has no net change, and the candidate iteration has no MMIO or other observable
side effect. An idempotent SRAM store that writes the bytes already present may
therefore qualify; a value-changing write does not. This matters for real idle
loops that balance a `PUSH` with a matching return without changing the saved stack
slot. Recognition is based on emulated state and access activity, never on
application symbols, task names, source paths, or hard-coded PCs.

The motivating PER FreeRTOS idle path repeats a 15-instruction loop containing
that kind of idempotent `PUSH`. The detector discovers the pattern from state; the
length and location are not emulator constants.

Once an exact, side-effect-free iteration is established, the board may account
for several identical iterations as one batch. It preserves the logical target
instruction and cycle totals and advances virtual time to the same deterministic
boundary. A batch is capped before any boundary that could make another iteration
observably different, including:

- the next SysTick transition or serviceable pending exception;
- the next shared `EventLoop` callback, including an injected CAN frame;
- the run's simulated-time or instruction budget; and
- a world scheduling frontier needed to preserve equal-time, configuration-order
  dispatch.

In a multi-board world, each proven lane contributes conservative lookahead. The
scheduler chooses the earliest observable time across every lane, the shared event
queue, and the run deadline; each board may land on its own loop boundary as long
as it does not cross that common causal horizon. This avoids requiring different
clock phases and loop lengths to coincide exactly.

### Allocation-free loop proof and planning state

Loop discovery itself uses a fixed-size, direct-mapped observation table keyed by
Thumb boundary PC. Each observation stores the complete CPU state, logical totals,
and a memory side-effect checkpoint. Direct-map collisions merely replace an old
candidate and delay proof; they cannot create a false match because the boundary
PC and full state are rechecked.

Reversible-memory proof uses a fixed 8,192-entry mutation journal in `MemoryBus`.
A checkpoint is rejected if the candidate spans more writes than the journal can
retain. Otherwise the journal is walked backward to verify that every changed byte
has returned to its checkpoint value. MMIO has a separate monotonically increasing
generation, so even a read prevents a loop from being classified as unobservable.
These bounded structures avoid heap allocation in the per-instruction detector and
fail closed when their evidence is insufficient.

The multi-board scheduler likewise allocates its state and planned-iteration arrays
once at the start of `World::run()`, clears the iteration counts at each frontier,
and reuses the storage. This avoids millions of host heap allocations in long
network simulations without changing scheduling order.

If a proof condition is absent or changes, execution continues one instruction at
a time. `--no-loop-batching` explicitly disables the optimization and is the right
control for raw interpreter comparisons. Batching is also disabled when
`--trace-instr` is active because a complete per-instruction trace was requested.
Finally, explicit spin detection takes precedence: `--detect-spin` asks the
exact-state detector to stop and report a stable loop after its threshold instead
of accelerating through it. This makes spin diagnosis and idle acceleration
distinct modes.

These constraints keep wakeups deterministic: a SysTick interrupt or scheduled
device event is handled at the same virtual-time boundary with or without
batching. `acceleratedPhaseMismatchedLoopsMatchExactExecution` runs different loop
periods at 1.016 GHz (where several target cycles can share one integer nanosecond)
with a mid-run callback, then compares batching on/off stop reason, virtual time,
logical instruction/cycle totals, full CPU state, and normalized trace.
`acceleratedLoopsPreserveSysTickAndExceptionEntry` separately proves a real
accelerated path preserves periodic SysTick handler execution and exception
entry/return traces. Further regressions cover restored/idempotent RAM, MMIO
rejection, cancelled event horizons, and time-zero CAN delivery before the first
CPU instruction.

## Disabled diagnostics and sample retention

Library callers default to enabled tracing, ADC sample history, and DMA transfer
history because those interfaces are observable. The CLI changes all three only
when no `--trace` output was requested: `TraceRecorder` stops appending records,
peripheral trace sites stop building event payloads where guarded, and ADC/DMA
history vectors stop growing. This reduces allocation, retained memory, and eventual
JSON serialization while leaving target-visible peripheral state enabled.

This is deliberately controlled by observer presence rather than workload names.
Requesting trace output reenables trace collection and ADC/DMA history together.
`--trace-instr` additionally requests a record for every instruction and disables
loop batching, since a batch cannot manufacture the requested intermediate trace.
Disabling diagnostics is therefore a performance mode only when the caller has
explicitly declined those observations; it does not discard target-visible MMIO,
interrupts, CAN traffic, or conversion results.

## Lazy unobserved continuous ADC conversions

Continuous ADC mode can otherwise create a host callback at every conversion
deadline. In the six-board workload, those callbacks reached roughly 1.95 million
per simulated second and repeatedly interrupted an otherwise batchable firmware
idle loop.

An ADC now retains only its next conversion timestamp when all per-conversion
outputs are unobservable: EOC/EOS interrupts are disabled, no sample callback is
installed, trace collection is disabled, and sample history is disabled. Advancing
the shared event loop then requires no ADC callback. On the next ADC MMIO access,
the model computes the most recent due conversion in constant time, samples the
configured provider at that exact deadline, and synchronizes DR plus EOC/EOS. It
also preserves the following deadline, so enabling diagnostics later resumes
scheduled conversion at the original phase rather than restarting the ADC clock.

This optimization never applies to single-shot conversions or multi-rank
sequences. It also falls back to one exact event per conversion as soon as an
EOC/EOS interrupt, sample callback, enabled trace recorder, retained sample
history, or DMA request makes intermediate conversions observable. The PER ADCs
use circular DMA, so their conversions intentionally take the exact event path and
are not eligible for this speedup. Trace collection and ADC history both default
to enabled for library callers and tests; the CLI disables them only when no trace
output was requested. Focused tests cover lazy provider timestamps, MMIO
synchronization, deadline-phase preservation, interrupt-observed continuous mode,
single-shot mode, sequence order/timing, and trace enable/disable behavior.

## Validation and diagnostic builds

Because an unspecified single-config build now defaults to Release, request Debug
explicitly for sanitizer work:

```bash
cmake -S . -B build-sanitize -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DFIL_BUILD_TESTS=ON \
  -DFIL_ENABLE_ASAN=ON \
  -DFIL_ENABLE_UBSAN=ON
cmake --build build-sanitize
ctest --test-dir build-sanitize --output-on-failure
```

External PER acceptance tests are opt-in. Point CMake at the directory containing
the seven per-board firmware output directories:

```bash
cmake -S . -B build-per -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DFIL_BUILD_TESTS=ON \
  -DFIL_PER_FIRMWARE_DIR=/absolute/path/to/PER/Projects/firmware/output
cmake --build build-per
ctest --test-dir build-per -L per --output-on-failure
```

CMake adds inspect tests for each ELF it finds, 10 ms strict-MMIO board runs, the
one-second `g4_testing` run, and the six-board network smoke test when all required
network ELFs are present. The long board gate asserts an exact time-budget stop at
1,000,000,000 ns and zero unknown MMIO. The network gate asserts an exact 10 ms
time-budget stop for all six boards and verifies firmware-originated shared-bus
`can_tx` and `can_rx` records in a generated JSONL trace. When Python 3.9+ and
`arm-none-eabi-objdump` are available, `fil.per.audit_instructions` also writes a
per-ELF static decoder report to `build-per/per-instruction-audit.json`. Use
`ctest --test-dir build-per -L per -LE long` to omit the one-second test during a
quick pass.

Finally, a target `BKPT` is a CLI error unless the caller explicitly passes
`--allow-breakpoint`. Use that flag only when reaching a breakpoint is the intended
success boundary, such as a synthetic firmware fixture; it prevents accidental
firmware breakpoints from being reported as successful benchmark runs.
