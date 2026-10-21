# IWDG and WWDG

The independent watchdog model accepts IWDG key operations for unlock/start/reload, stores prescaler/reload fields, and schedules nominal timeout from a 32 kHz clock. WWDG models enable/reload and a nominal timeout derived from configured input clock and prescaler. Both can invoke a reset-request callback.

**Use:** configure/reload on a deterministic simulated-time schedule. An integrated timeout signals `reset-requested` and stops the board; it does not automatically reset and restart the MCU. IWDG timeout arithmetic is nominal, not an LSI tolerance model.

**Limits:** IWDG status/window synchronization is minimal; WWDG is a deadline rather than live down-counter. Window/early-refresh/EWI behavior is unsupported. Integrated watchdogs are reset-disabled by default, and WWDG's software-clearable enable behavior differs from hardware.

**RM0440:** IWDG §35.3.5, §35.3; §35.4.1 `KR`, §35.4.2 `PR`, §35.4.3 `RLR`, §35.4.4 `SR`. WWDG §36.3.2–36.3.4; §36.4; §36.5.1 `CR`, §36.5.2 `CFR`, §36.5.3 `SR`.

**Tests:** `PeripheralTest.IwdgLockedWritesAndExactReloadDeadline`; `PeripheralTest.IwdgPrescalerCodesProduceNominalTimeoutDeadlines`; `PeripheralTest.WwdgCounterAndCfrPrescalerSetTimeoutDeadline`; `PeripheralTest.CompletesDmaAndWatchdogSideEffects`.

See the [coverage matrix](../stm32g4_peripheral_coverage.md#iwdg-and-wwdg) for distinctions between tested model behavior and hardware semantics.
