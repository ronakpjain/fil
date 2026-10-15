# Roadmap

`fil` already provides deterministic Cortex-M4F execution, an STM32G474 memory and
peripheral model, single-board runs, and fixed-order multi-board CAN simulation.
This roadmap lists deliberate gaps; it is not a release schedule.

## Near-term priorities

1. **Broaden instruction tests.** Add execution cases for implemented VFP, DSP, and
   addressing forms that currently have decode-only or acceptance-only evidence.
2. **Deepen peripheral behavior.** EXTI/SYSCFG routing and USART/SPI DMA requests
   are now modelled (see peripheral coverage); prioritize remaining register
   semantics exercised by real firmware, especially watchdog reset integration
   and timer PWM/capture-compare behavior.
3. **Improve fault fidelity.** Convert more decode and memory failures into modeled
   UsageFault, BusFault, and HardFault escalation while retaining clear diagnostics.
4. **Measure scheduler changes.** Benchmark scheduler changes across representative
   workloads and preserve exact traces and modeled instruction/cycle counts.
5. **Automate compatibility evidence.** Run the external firmware decoder audit and smoke
   matrix in an environment where the firmware artifacts and ARM tools are
   available.

## Micro-JIT framework optimization priorities

Optimize the profitability and overhead of existing kernels before expanding
instruction coverage. More eligible blocks can increase compilation and dispatch
cost without improving firmware throughput.

1. **Profitability-based admission.** Start with cached handlers and require
   demonstrated reuse before native compilation. Consider prefix length and
   expected dispatch savings, not eligibility alone.
2. **Cheaper kernel boundaries.** Eliminate unnecessary flag transfers and use
   exit-specific register-write masks. Preserve untouched registers, later-written
   destinations, and LR across every partial-prefix and taken-branch exit.
3. **Cheaper prefix checks.** Investigate flag-preserving instruction-limit checks
   or a full-block fast route, accounting for extra code size and compilation cost.
4. **Less surrounding bookkeeping.** Profile cycle-budget admission and result
   materialization; reuse prepared metadata where exact event and timing contracts
   allow it.
5. **Indexed compilation-cache lookup, if profiling warrants it.** Use a
   fingerprint to narrow candidates while retaining exact semantic comparison.
   Avoid optimizing lookup ahead of larger execution costs.

Validate each change independently on representative real firmware, including
startup and compilation, with fixed fidelity settings and alternating baseline
and candidate runs. Compare against native kernels disabled as well as the
current implementation. Report compilation counts separately from throughput;
retired-instruction coverage is not a measure of host-time benefit. Profile the
surrounding execution and scheduler paths when kernels cover only a small share
of the workload. Preserve capsule replay correctness through differential
validation before relying on sustained capsule-enabled throughput measurements.

## Candidate extensions

- Cortex-M event-latch behavior for `WFI`, `WFE`, and `SEV`;
- exclusive accesses and additional ARMv7E-M DSP/SIMD instructions;
- FPSCR modes, exception status, and more complete floating-point edge behavior;
- GPIO alternate functions and pull-resistor electrical behavior;
- richer timer capture/compare and PWM behavior (CCMR/CCER/CCR/BDTR time-based
  compare scheduling beyond EGR software triggers);
- USART/SPI transfer timing (baud-derived delays) beyond functional DMA handshakes;
- CAN arbitration, serialization delay, error state, and additional M_CAN queues;
- optional automatic MCU reset/restart after watchdog or AIRCR reset requests.

## Scope rules

- Add behavior from architectural state, MMIO, configuration, and firmware inputs;
  never recognize application symbols, task names, source paths, or fixed PCs.
- Prefer explicit unsupported diagnostics over permissive guesses.
- Keep same-time ordering and traces deterministic.
- Add focused unit tests before relying on an external firmware acceptance case.
- Treat instruction-level timing as a regression model, not hardware cycle accuracy.

The current support boundary is defined by
[Thumb instruction coverage](thumb_instruction_coverage.md) and
[STM32G4 peripheral coverage](stm32g4_peripheral_coverage.md).
