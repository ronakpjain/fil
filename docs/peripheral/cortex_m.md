# Cortex-M system peripherals

The system window (`0xe0000000..0xe00fffff`) models SysTick, NVIC, SCB, CoreDebug/DWT, and selected FP state. SysTick has deterministic LOAD/VAL wrap, COUNTFLAG and pending behavior. The NVIC tracks enable/pending/active state, software triggers and 240 priority slots; this device exposes four implemented priority bits for its 102 device IRQs. Arbitration accounts for PRIMASK, BASEPRI, FAULTMASK, and active exceptions. SCB includes selected identification/control/status registers, keyed AIRCR reset request, vector alignment, and write-one-to-clear fault-status views. DWT CYCCNT advances when enabled. Exception entry/return includes basic and extended frames with tested lazy FP preservation.

**Use:** configure SysTick/NVIC and exception priorities for firmware scheduling and interrupt handling. Peripheral interrupt inputs are level-sensitive: an enabled uncleared source can re-pend on exception return. AIRCR reset is a request consumed by the board, not an automatic MCU restart.

**Limits:** register and exception semantics are primarily architecture-defined; RM0440 delegates them to PM0214, and this model is not independently reference-verified against that manual. Generic NVIC has 240 slots although STM32G4 implements 102 device IRQs. MPU, ITM/SWO, debug transport, breakpoint/watchpoint comparators, and automatic fault escalation are absent. DWT counting is enabled by its modeled control, not gated by DEMCR.TRCENA.

**References:** RM0440 §14.1 (IRQ count/priority width and PM0214 delegation); §14.2 (SysTick calibration); §14.3 Table 100 (device IRQ routing); §47.2 (debug architecture delegation). PM0214 is needed for full architectural semantics; no unverified PM0214 section citations are asserted here.

**Tests:** `CortexMTest.ModelsSysTick`; `CortexMTest.AllDeviceInterruptPrioritiesImplementExactlyFourBits`; `CortexMTest.BasepriArbitrationUsesOnlyImplementedPriorityBits`; `CortexMTest.RequiresAircrKeyAndAlignsVectorTableBase`; `ExceptionTest.StacksAndReturnsBasicFrame`; `ExceptionTest.StacksAndReturnsExtendedFloatingPointFrame`.

See the [coverage matrix](../stm32g4_peripheral_coverage.md#cortex-m-system-window) for per-register evidence and remaining risks.
