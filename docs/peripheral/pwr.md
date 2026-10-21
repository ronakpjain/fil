# PWR (power control)

PWR is intentionally a small register model: it provides Range 1 normal-mode reset state, writable boost selection in `CR5.R1MODE`, register storage through CR5, and immediately complete voltage-scaling status. It does not drive supply or low-power behavior.

**Use:** firmware may configure/read the modeled voltage-range and boost fields. `VOSF` is always clear; do not use it to infer a simulated regulator transition.

**Limits:** no voltage domains, supply effects, stop/standby transitions, wakeup pins, backup-domain rules, or electrical operating-limit validation.

**RM0440:** §6.1.5; §6.4.1 `CR1.VOS`; §6.4.9 `SR2.VOSF`; §6.4.22 `CR5.R1MODE`.

**Tests:** `PeripheralTest.PwrResetVoltageRangeAndBoostFieldAreStable`; `PeripheralTest.ModelsRangeOneBoostControl`.

See the [coverage matrix](../stm32g4_peripheral_coverage.md#pwr) for exact modeled fields and limitations.
