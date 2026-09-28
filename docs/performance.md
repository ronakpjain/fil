# Performance

`fil` reports simulated cycles and nanoseconds separately from host wall time. Throughput is simulated seconds divided by wall seconds. The timing model charges Cortex-M4 instruction-class costs, taken-branch refill, and STM32G4 flash fetch stalls from `FLASH_ACR`; it converts cycles at the current modeled clock, retaining fractional time. It is an approximation, not cycle-accurate silicon. Compare runs with identical firmware, host/power mode, compiler, optimized build, PGO training, tracing, and CLI options. Check `cycles >= instructions` and the simulated-time deadline rather than assuming one cycle per instruction.

## Semantics-preserving host optimizations

- **CPU and memory:** Decoded instructions and their static cycle costs are cached; executable writes invalidate affected decode state. A containment-checked region cache and checked direct backing-store access avoid repeated mapping/MMIO dispatch without bypassing permissions, mutation tracking, or executable invalidation. Successful steps and memory accesses use compact results; fault diagnostics are built only on failure.
- **Interrupts and events:** Pending-enabled summaries and cached interrupt selection avoid repeated scans; changes to pending state, masks, priorities, and active exceptions invalidate selection. Board-owned event horizons avoid treating an unrelated board's callback as a global boundary. Unobserved single-rank continuous ADC conversions can be materialized lazily; observable conversions retain scheduled callbacks. DMAMUX routes are cached until selector writes.
- **Scheduler:** Equal-time boards can take bounded lockstep bursts, stopping at event, interrupt, budget, clock-change, or fault boundaries. Stable board pointers and reusable planner arrays avoid frontier allocations. Cold boundary and MMIO-restart paths stay out of the instruction hot path.
- **Proven loop batching:** At a repeated backward-flow boundary, the emulator compares architectural state, memory mutations, MMIO generations, and a read footprint. It batches only when another complete iteration fits before the earliest event, serviceable interrupt, deadline, or instruction budget. It advances logical instruction/cycle counts and fractional time as if each iteration executed. Stale proofs, uncertain memory writes, and observable side effects fall back to instruction-by-instruction execution. `--no-loop-batching` disables it; `--detect-spin` reports loops rather than batching; `--trace-instr` retains per-instruction records and disables batching.
- **Build:** Release builds use IPO/LTO when supported. Clang PGO can improve dispatch and branch layout after training on representative firmware; retrain after code or toolchain changes. Debug and sanitizer builds are not throughput baselines.

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
