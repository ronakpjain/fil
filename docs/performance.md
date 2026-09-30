# Performance

`fil` reports simulated cycles and nanoseconds separately from host wall time. Throughput is simulated seconds divided by wall seconds. The timing model charges Cortex-M4 instruction-class costs, taken-branch refill, and STM32G4 flash fetch stalls from `FLASH_ACR`; it converts cycles at the current modeled clock, retaining fractional time. It is an approximation, not cycle-accurate silicon. Compare runs with identical firmware, host/power mode, compiler, optimized build, PGO training, tracing, and CLI options. Check `cycles >= instructions` and the simulated-time deadline rather than assuming one cycle per instruction.

## Semantics-preserving host optimizations

- **CPU and memory:** Decoded instructions and their static cycle costs are cached; executable writes invalidate affected decode state. A containment-checked region cache and checked direct backing-store access avoid repeated mapping/MMIO dispatch without bypassing permissions, mutation tracking, or executable invalidation. Successful steps and memory accesses use compact results; fault diagnostics are built only on failure.
- **Interrupts and events:** Pending-enabled summaries and cached interrupt selection avoid repeated scans; changes to pending state, masks, priorities, and active exceptions invalidate selection. Board-owned event horizons avoid treating an unrelated board's callback as a global boundary. Unobserved single-rank continuous ADC conversions can be materialized lazily; observable conversions retain scheduled callbacks. DMAMUX routes are cached until selector writes.
- **Scheduler:** Equal-time boards can take bounded lockstep bursts, stopping at event, interrupt, budget, clock-change, or fault boundaries. Stable board pointers and reusable planner arrays avoid frontier allocations. Cold boundary and MMIO-restart paths stay out of the instruction hot path.
- **Proven loop batching:** At a repeated backward-flow boundary, the emulator compares architectural state, memory mutations, MMIO generations, and a read footprint. It batches only when another complete iteration fits before the earliest event, serviceable interrupt, deadline, or instruction budget. It advances logical instruction/cycle counts and fractional time as if each iteration executed. Stale proofs, uncertain memory writes, and observable side effects fall back to instruction-by-instruction execution. `--no-loop-batching` disables it; `--detect-spin` reports loops rather than batching; `--trace-instr` retains per-instruction records and disables batching.
- **Build:** Release builds use IPO/LTO when supported. Clang PGO can improve dispatch and branch layout after training on representative firmware; retrain after code or toolchain changes. Debug and sanitizer builds are not throughput baselines.
- **Experimental cached hot-path compilation (`--jit`, default off):** After approximately 50 probes at a cache slot, the CPU compiles a decoded straight-line prefix of up to 16 instructions, ending at control-flow or exception/system/FP boundaries. This is a cached/predecoded interpreter with specialized integer handlers, **not native machine-code generation**. NOP, B/BL, CBZ/CBNZ, MOV/MOVW/MOVT, immediate and register arithmetic/compare, common bitwise operations, shifts, and MUL use cached specialized handlers; other supported operations call the interpreter. Single-instruction specialization becomes available as soon as an instruction is decoded; block preparation is hotness-gated and negative-caches unsupported regions. CPU-level blocks preserve condition checks, data-dependent divide costs, branch refill, MMIO restart checkpoints, and executable-write invalidation between instructions. IT blocks, pending exceptions, and instruction tracing stay interpreted. Board-level batching requires a memory-free prefix whose conservative pipeline-plus-flash cost finishes strictly before the next scheduled event, SysTick interrupt, and deadline. Memory-containing blocks and stop-address/spin runs retain single-instruction boundaries. Standalone and worker runs also prefer lightweight single steps when proven loop batching is enabled, rather than paying for competing block preparation. Worker and board requests cap execution to the remaining instruction budget. General network dispatch executes one instruction through a lightweight cached handler: another board can dynamically schedule an event while a lane is in flight, so merely booking a block's final completion time is not safe. Synchronized lockstep lanes may batch only when **every** lane has a memory-free, fixed-cost block with the same exact completion time, before all event/interrupt/deadline boundaries. With no MMIO in any lane and no earlier lane completion, none can create a new observable event within another's block. Conditional/register-target branches and divide instructions are excluded from that fixed-cost proof. Single-instruction execution shares the interpreter's fetch, condition, restart, timing, and diagnostic front end, avoiding duplicate fallback checks and block-result construction. Omitting `--jit` selects the default interpreter path; `--jit` opts in for experiments, not a guaranteed speedup.

## Cached hot-path validation and measurements

The implementation passed **206 tests in both Release and Debug ASan/UBSan builds**, including differential checks for MMIO prefix restart and exactly-once resumption, executable-RAM self-modification, capped execution, exact-preview count/cycle agreement, lookahead fetch faults, randomized Thumb-16/Thumb-32 integer operations with MSP/PSP operands, scheduled and dynamically added events, deadlines, SysTick/exception timing, interior stop addresses, small world instruction budgets, and divergent lane clocks. A cold synchronized-loop test confirms real block compilation and fewer dispatches while preserving full CPU state, logical counters, and traces.

The six-board JIT-on/off traces for 100 ms were byte-identical at ADC decimation 1 (**83,313 lines**) and 32 (**7,836 lines**). These checks are regression evidence, not proof for all firmware.

Measurements used an Apple M3 host, Apple Clang 21.0.0, Release `-O3` with IPO/LTO and no PGO, the sibling PER firmware, 5,000 ms simulated duration, 250 million instructions per board, quantum 1,024, strict MMIO, default loop batching, and no tracing. Five pairs alternated execution order between interpreter and JIT:

| ADC decimation | Interpreter wall seconds | `--jit` wall seconds | Median interpreter / JIT | Median wall-time reduction |
| --- | --- | --- | --- | --- |
| 1 | 12.99, 13.57, 14.60, 13.59, 13.35 | 13.18, 13.39, 13.53, 13.26, 13.08 | 13.57 / 13.26 s | 2.3% |
| 32 | 4.80, 4.82, 4.82, 4.84, 4.79 | 4.74, 4.82, 4.78, 4.75, 4.75 | 4.82 / 4.75 s | 1.5% |

Both modes reached the time deadline and produced identical summary output within each decimation setting: 199,231,699 instructions / 480,000,007 cycles / 1,224,321 callbacks at N=1; 191,031,795 instructions / 480,000,010 cycles / 71,427 callbacks at N=32. The lightweight shared front end removes the large slowdown of the earlier correctness-only implementation. The remaining measured gain is **modest and subject to host variance**, not a promised speedup; keep `--jit` opt-in. Earlier faster measurements using unsafe multi-instruction network dispatch are not valid evidence of a semantics-preserving speedup. Rebenchmark after changes to implementation, firmware, compiler, or host. Decimation changes firmware-visible behavior; compare JIT on/off with the same N.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 8
ctest --test-dir build --output-on-failure
# Repeat at least five pairs, alternating order. Repeat with --adc-decimation 32.
/usr/bin/time -p ./build/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 5000 --max-instructions 250000000 --quantum 1024 --strict-mmio
/usr/bin/time -p ./build/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 5000 --max-instructions 250000000 --quantum 1024 --strict-mmio --jit
```

## Opt-in ADC scan decimation

`--adc-decimation N` on `run` or `run-network` keeps one of every N continuous-mode ADC scans (N=1–1024; default 1). Skipped scans do not update DR/ISR, emit EOC/EOS, trigger DMA/interrupts, or record samples. Their conversion deadlines remain phase-aligned, and skipped spans collapse into fewer queued events. Multi-rank decisions are per scan; single-shot conversions are never skipped. A timing-register change during a gap may take effect only at the next kept scan. **This changes firmware-visible results and interrupt cadence**; N=1 preserves normal behavior. Never attribute a decimated speedup to a semantics-preserving optimization.

## Reproduce and interpret

The six-board `per_vehicle.json` workload requires the sibling PER firmware checkout. Use identical firmware, tracing settings, host power mode, and build for both runs:

```bash
make BUILD_DIR=build-release BUILD_TYPE=Release IPO=ON test
/usr/bin/time -p ./build-release/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 1000 --max-instructions 50000000 --quantum 1024 --strict-mmio
/usr/bin/time -p ./build-release/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 1000 --max-instructions 50000000 --quantum 1024 --strict-mmio \
  --adc-decimation 8
```

Repeat and compare median wall times and simulator counters only if both runs stop at the simulated-time deadline; if the instruction limit stops either run first, raise it. Different instruction counts are expected with decimation because fewer DMA/ISR paths execute. No single host measurement guarantees real-time throughput on another machine. The no-PER smoke benchmark runs `make BUILD_DIR=build-release BUILD_TYPE=Release benchmark BENCH_REPS=3`; its synthetic idle loops are not representative of the ADC-heavy network.

Run the full PGO cycle with `make pgo`. By default it trains on the six-board `configs/networks/per_vehicle.json` workload for 1,000 ms (50 million instruction limit); this requires the PER firmware checkout referenced by the configs. Override the workload with `PGO_TRAIN_ARGS='run-network ...'` if needed. The target cleans prior PGO outputs, builds the instrumented binary, trains it, merges the profile, builds the optimized binary, and runs tests. Set `PGO_CXX` and `LLVM_PROFDATA` to matching Clang/LLVM versions; retrain after material changes. Individual stages remain available via `make pgo-generate`, `pgo-train`, `pgo-merge`, `pgo-use`, and `pgo-test`; `make help` lists overrides.

The test suite checks batched versus exact execution, cache invalidation, event boundaries, timing, and ADC decimation behavior. Run `make test` after performance changes; benchmark wall times are not correctness tests.
