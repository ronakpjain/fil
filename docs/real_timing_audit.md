# Real STM32G4 timing audit (vs fil 1-cycle-per-instruction model)

Date: 2026-09-25. Host: Apple M3 / macOS 27. Source: fil 0.1.0.
References: ARM DDI0439C (Cortex-M4 TRM / Technical Reference),
ST RM0440 (STM32G4 reference manual), ST DS12288 (STM32G474 datasheet).

## 1. What the emulator does today

- `CortexM4::stepFast()` (`src/cpu/cortex_m4.cpp:316`) sets
  `result.cycles = 1` for every successfully executed instruction
  (0 only for the MMIO-restart path).
- `Board::accountCycles()` (`src/sim/board.cpp:125`) converts cycles to
  simulated nanoseconds as `cycles * 1e9 / systemClockHz`, keeping a
  `time_fraction_` remainder. `systemClockHz` comes from the RCC model
  (reset 16 MHz HSI, HSE/PLL selectable, `src/stm32g4/rcc.cpp`).
- Consequence, confirmed by CLI: 25 instructions at 16 MHz report
  `cycles: 25, time_ns: 1562` (= 25 * 62.5 ns truncated with fraction).
- `FLASH` peripheral (`src/stm32g4/flash.cpp`) models lock keys, status,
  and page erase only. There is **no FLASH_ACR** (access control register),
  no LATENCY / PRFTEN / ICEN / DCEN, no wait states, no prefetch/ART model.
- Loop batching (`Board::observeLoopBoundary`) multiplies
  `cycles_per_iteration` exactly, so any per-instruction cycle change flows
  through batching unchanged provided the clock and flash config are stable
  over the batch (previously assumed; now enforced by the proof).
- Performance docs (`docs/performance.md`) report the one-second six-board
  workload as exactly 96,000,000 instructions = 96,000,000 cycles =
  1,000,000,000 ns (16M/board at 16 MHz). That equality is the 1-CPI
  assumption, not a measurement of silicon.

## 2. How a real STM32G4 behaves

### 2.1 Cortex-M4 pipeline (ARM DDI0439C, 3-stage fetch/decode/execute)

Single-cycle for most 16-bit ALU/logic/shift/move/compare operations.
Documented multi-cycle / data-dependent cases (simplified to the forms
fil decodes):

| Class | Examples | Real cycles | Notes |
|---|---|---|---|
| ALU / move / shift / compare / bitfield / reverse / extend | ADD/SUB/MOV/AND/ORR/EOR/LSL/LSR/ASR/ROR/CLZ/REV/BFC/UBFX/SXT/UXT | 1 | register-shifted operand still 1 on M4 |
| DSP extension | UADD8, SEL | 1 | |
| Multiply | MUL | 1 | single-cycle 32x32 multiplier |
| Multiply-accumulate | MLA, MLS | 2 | |
| Long multiply | UMULL, SMULL | 5 (TRM 5-7 range; model uses 5) | 64-bit result path |
| Divide | UDIV, SDIV | 2-12, early termination on divisor magnitude | model: `12 - clz(divisor)*10/32`, 0-divisor = 2 |
| Single load/store | LDR/STR/LDRB/STRB/LDRH/STRH/LDRSB/LDRSH | 2 | 1 execute + 1 memory |
| Double load/store | LDRD/STRD | 3 | |
| Multi load/store | LDM/STM/PUSH/POP | 1 + register count | +1 counted by popcount; PC-loaded adds branch refill below |
| Taken branch refill | B/BL/BLX/BX/CBZ-taken/CBNZ-taken/LDR-to-PC/MOV-to-PC/LDM-to-PC/POP-to-PC | +2 over base (+1 for CBZ/CBNZ) | pipeline refill; not-taken conditional = base only |
| Call | BL/BLX | 4 total (base 2 + refill 2) | 32-bit + link + refill |
| Special / barrier | MRS/MSR 2, SVC 2, DMB/DSB/ISB 2, NOP/WFI/WFE/SEV/CPS/IT 1 | | WFI wait-for-interrupt dwell is event time, not extra pipeline cycles |
| FP data | VADD/VSUB/VMUL/VNMUL/VNEG/VABS/VMOV/VCMP 1; VFMA/VFMS/VFNMS 3; VCVT 2; VDIV/VSQRT 14 | | FPv4-SP-D16 |
| FP memory | VLDR/VSTR 2; VLDM/VSTM 1+n; VMRS 2 | | |
| Condition failed | any with `conditionPasses == false` | 1 | no memory/branch effect; IT-advance still applies |

Key correction vs 1-CPI: loads cost 2x, calls/branches 3-4x,
divides up to 12x, single-precision divide/sqrt 14x, multi-register
transfers scale with list length.

### 2.2 STM32G4 flash: wait states + ART (RM0440 § FLASH_ACR, DS12288)

- `FLASH_ACR` offset `0x00`: `LATENCY[2:0]`, `PRFTEN(8)`, `ICEN(9)`,
  `DCEN(10)`, `ICRST(11)`, `DCRST(12)`. Reset `0x00000000`.
- HCLK wait-state requirement (VOS Range 1 / Boost, DS12288):
  0 WS to 30 MHz, 1 WS to 60, 2 WS to 90, 3 WS to 120, 4 WS to 170.
  Firmware must program `LATENCY` before raising HCLK; running 170 MHz
  on LATENCY=0 is an overclock, not a faster configuration.
- With prefetch (`PRFTEN`) and/or instruction cache (`ICEN`) enabled,
  sequential 64-bit fetch rows hit in the ART buffer/cache and pay ~0
  extra; taken branches / cache misses pay the full `LATENCY` stall.
  Data cache (`DCEN`) affects data reads, not the fetch path modeled here.
- SRAM/CCM-SRAM/system-memory/RAM execution: 0 fetch wait states at any
  supported HCLK. Boot alias `0x00000000` maps flash and inherits flash
  timing.
- fil gap: no ACR at all, so high-clock firmware (e.g. 170 MHz PLL) was
  timed as if flash were 0-WS SRAM: 1 CPI at 5.88 ns/cycle. Real 170 MHz
  flash execution with LATENCY=4 pays ~1 + 0-4 extra per fetch depending
  on sequential/branch behavior, i.e. effective CPI well above 1 for
  branchy code even before pipeline costs.

### 2.3 Clock tree (RM0440 § RCC, `src/stm32g4/rcc.cpp`)

- Sources: MSI (~4 MHz model estimate), HSI16 (16 MHz, reset default),
  HSE 4-48 MHz (board config 16 MHz), HSI48, PLL (M/N/R) to 170 MHz max.
- `CFGR.SWS` mirrors `SW`; AHB/APB prescalers exist on silicon but fil
  advances CPU/pipeline time on SYSCLK only. SysTick and timer/ADC input
  clocks derive from the same SYSCLK value in this model.
- Time per pipeline cycle is therefore `1e9 / SYSCLK_Hz` ns; a 16 MHz
  board advances 62.5 ns/cycle, a 170 MHz board ~5.88 ns/cycle.
  Cycle-count errors scale directly into simulated-time errors.

### 2.4 What this changes for pacing

1. **Pipeline cycles** vary 1-14 per instruction (table above).
2. **Fetch stall** adds 0-`LATENCY` per flash fetch: 0 for SRAM/ROM,
   0 for sequential flash hits with ART/prefetch/cache, else LATENCY.
3. **Branch penalty** (+2, +1 CBZ/CBNZ) applies on any discontinuous PC,
   covering B/BL/BX/LDR-to-PC/POP-to-PC uniformly without special cases.
4. At reset/16 MHz/LATENCY=0 the model still yields 1 cycle for NOP/ALU,
   so existing NOP-idle-loop tests are unaffected; LDR/LDM/DIV/FPU/branch
   counts change to hardware-like values.
5. Simulated time per instruction = total cycles (pipeline + fetch) at the
   *current* SYSCLK. 16M instructions at 16 MHz no longer imply 1.0 s;
   with realistic CPI ~1.2-1.6 plus flash stalls the same instruction
   count advances *more* simulated time, so a 1.0 s budget executes
   *fewer* instructions than 96M. The old `instructions == cycles`
   benchmark invariant is retired; `cycles >= instructions` with
   CPI reported.

## 3. Benchmark consequences

- Old gate: `instructions=96000000 / cycles=96000000 / time_ns=1000000000 /
  16M per-board` (PER network, tracing off, `--quantum 1024 --strict-mmio`).
- New gate: same command, but expect `time_ns=1000000000`,
  `cycles >= instructions`, per-board instructions *not* fixed at 16M,
  CPI (`cycles/instructions`) reported, and synthetic fixture benchmarks
  runnable without the sibling PER checkout (which is absent on this host).
- `tools/bench_real_timing.py` runs the fixture board + a synthetic
  six-board fixture network, prints instructions/cycles/CPI/time/wall/
  throughput, and asserts `cycles >= instructions` and exact 1 s time.
- Throughput is still `simulated seconds / wall seconds`. Fewer
  instructions per simulated second (higher CPI) *reduces* host work per
  simulated second; the added per-instruction cost is one LUT + popcount +
  two range compares, measured in §4.

## 4. Performance plan (measured, not assumed)

Hot-path cost audited 2026-09-25: per-instruction dispatch switch +
`MemoryBus` containment/journal/footprint + `accountCycles` 64-bit divide
+ `EventLoop::advanceBy` per instruction. Added timing work is bounded:
one `InstrKind`-indexed LUT, one `popcount` only for list ops, one
divisor CLZ only for UDIV/SDIV, one flash range check + one sequential
compare, one PC-discontinuity compare. Frequency division is cached
(`cachedFrequencyHz_` fast path; divide only on clock change path is
unavoidable for exact fractional ns, but the load/compare is one branch).

Optimization order: (1) cycle LUT + cached clock/WS (this change),
(2) measure fixture wall times before/after, (3) only then consider
deeper scheduler/event batching. No JIT/transactional default changes.

## 5. Measured outcome (2026-09-25, Apple M3, Release + IPO)

`tools/bench_real_timing.py` (`--reps 3`): idle16 x1 no-batch 0.181 s
(5.53x), x6 no-batch 1.493 s (0.67x), pll170 x1 no-batch 1.135 s (0.88x);
batching on is 250-440x on these loop-dominated fixtures. Same-host 1-CPI
baseline: x6 no-batch 1.697 s (0.59x), pll170 no-batch 2.858 s (0.35x).
Per-instruction host cost rose ~16.9 ns to ~18-22 ns (timing LUT, ART check,
burst prediction), but realistic CPI (1.60/2.40) executes fewer instructions
per simulated second, so fixed-sim-time throughput improved on every case.
Delivered optimizations: memoized per-decode cycle costs, exact direct-branch
burst prediction with numerator-compare gating (one div/round), proof-work
gating (also repairs no-batch bursts), outlined 90 KiB observation reset and
heap-boxed restart snapshot (no hot-frame stack probes). Full suite 151/151
passes, including 9 new `RealTimingTest` cases and 4 updated 1-CPI
assumptions (split-image 3 instr/5 cycles, scheduler-tick 84 instr/156
cycles, 100 ns-budget and frontier-timestamp updates).
