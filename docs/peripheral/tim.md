# TIM (timer peripherals)

TIM1–8, TIM15–17, and TIM20 share one deterministic up-counter model. With CEN set, CNT advances against simulated time using the configured clock, PSC, and ARR; update events can set UIF, drive UIE interrupts, invoke callbacks, and stop a one-pulse run. EGR.UG forces an update. RCC clock changes reschedule timer progress.

**Use:** configure PSC/ARR, enable CEN, and enable UIE if interrupt notification is needed. Read CNT for live simulated counter state; clear status as firmware would. This is useful for periodic update and basic scheduler firmware.

**Limits:** same generic behavior across timer types; capture/compare, PWM, repetition/dead time, encoder/slave modes, chaining, and most status bits are absent. PSC writes apply immediately/rephase (unlike buffered hardware behavior); complete APB timer clock tree and instance-specific widths are not modeled.

**RM0440:** §29.3.3; §29.6.1 `CR1` (including OPM); §29.6.4 `DIER.UIE`; §29.6.5 `SR.UIF`; §29.6.6 `EGR.UG`; §29.6.12–14 `CNT/PSC/ARR`. Related timer families: §30.5, §31.7, §32.4. IRQ mapping: §14.3 Table 100.

**Tests:** `PeripheralTest.TimerPrescalerClockChangeCntWriteAndStopRestartRephase`; `PeripheralTest.TimerForcedUpdateStatusAndOnePulseRestart`; `Stm32G4Test.SharedTimerIrqRemainsAssertedUntilAllSourcesClear`.

See the [coverage matrix](../stm32g4_peripheral_coverage.md#tim) for model fidelity and IRQ-sharing notes.
