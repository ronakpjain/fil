# SYSCFG and EXTI

SYSCFG `EXTICR1–4` selects a GPIO port for each EXTI line 0–15; reserved port selections are treated as unrouted. EXTI models rising/falling edges from GPIO level changes, software trigger, interrupt mask, and pending-bit write-one-to-clear. Lines 0–4 use dedicated IRQs; 5–9 and 10–15 are ORed onto shared IRQs.

**Use:** select the source port in EXTICR, configure the edge and `IMR1`, then clear latched pending state by writing the corresponding `PR1` bit. `SWIER1` can set pending in the model.

**Limits:** only lines 0–15; no event output (`EMR`), bank-2 lines, or other SYSCFG remap/compensation behavior. This models GPIO digital edges, not electrical input characteristics.

**RM0440:** §10.2.3–10.2.6 `EXTICR1–4`; §15.3.3–15.3.4; §15.4; §15.5.1 `IMR1`, §15.5.3 `RTSR1`, §15.5.4 `FTSR1`, §15.5.5 `SWIER1`, §15.5.6 `PR1`; IRQ routing §14.3 Table 100.

**Tests:** `Stm32G4Test.SyscfgRoutesEveryExtiLineAcrossAllPortsAndRejectsReserved`; `Stm32G4Test.ExtiDedicatedAndSharedInterruptOutputsAssertAndClear`; `Stm32G4Test.ExtiSoftwareTriggerW1SAndSelectivePendingClear`.

See the [coverage matrix](../stm32g4_peripheral_coverage.md#syscfg-and-exti) for additional integration tests and limits.
