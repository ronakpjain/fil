# Roadmap

`fil` already provides deterministic Cortex-M4F execution, an STM32G474 memory and
peripheral model, single-board runs, and fixed-order multi-board CAN simulation.
This roadmap lists deliberate gaps; it is not a release schedule.

## Near-term priorities

1. **Broaden instruction tests.** Add execution cases for implemented VFP, DSP, and
   addressing forms that currently have decode-only or acceptance-only evidence.
2. **Deepen peripheral behavior.** Prioritize register semantics exercised by real
   firmware, especially EXTI/SYSCFG routing, USART/SPI DMA requests, and watchdog
   reset integration.
3. **Improve fault fidelity.** Convert more decode and memory failures into modeled
   UsageFault, BusFault, and HardFault escalation while retaining clear diagnostics.
4. **Measure scheduler changes.** Keep transactional worker epochs opt-in until they
   show a repeatable benefit across representative workloads and preserve exact
   traces.
5. **Automate compatibility evidence.** Run the external PER decoder audit and smoke
   matrix in an environment where the firmware artifacts and ARM tools are
   available.

## Candidate extensions

- Cortex-M event-latch behavior for `WFI`, `WFE`, and `SEV`;
- exclusive accesses and additional ARMv7E-M DSP/SIMD instructions;
- FPSCR modes, exception status, and more complete floating-point edge behavior;
- functional EXTI edge routing and GPIO alternate functions;
- richer timer capture/compare and PWM behavior;
- USART/SPI transfer timing and peripheral DMA handshakes;
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
