# GPIO

GPIOA–GPIOG are modeled as digital register-backed ports. `MODER`, `ODR`, and `IDR` are represented; `BSRR`/`BRR` update output state, output levels mirror to input unless externally overridden, and external input overrides can be released. Timestamped output transitions and callbacks support board-level pin stimulus. Level changes are forwarded to EXTI edge detection through SYSCFG routing.

**Use:** configure a pin as output and write `BSRR`/`BRR` (or ODR); provide external levels for modeled input pins. Configure SYSCFG/EXTI separately to route input transitions to interrupts.

**Limits:** digital-only; pull resistors, open-drain/electrical levels, slew, contention, alternate-function routing, and clock gating are absent. Other GPIO fields generally retain writes without implementing their hardware semantics.

**RM0440:** §9.4.1 `MODER`; §9.4.5 `IDR`; §9.4.6 `ODR`; §9.4.7 `BSRR`; §9.4.11 `BRR`; register map §9.4.12.

**Tests:** `PeripheralTest.GpioBsrrPriorityBrrSubwordAndExternalRelease`; `PeripheralTest.GpioResetRestoresPortSpecificDebugPinModes`; `Stm32G4Test.RoutesIntegratedPeripherals`.

See the [coverage matrix](../stm32g4_peripheral_coverage.md#gpio) for storage-only fields and behavior caveats.
