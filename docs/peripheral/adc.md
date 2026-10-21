# ADC

ADC1–5 implement regular conversion sequences of up to 16 ranks. Calibration/enable readiness is immediate; conversion duration derives from selected channel sampling time, configured resolution timing, and the selected ADC clock. DR/EOC/EOS, single/continuous runs, selected interrupts, and constant or simulated-time sine input providers are modeled. ADC common groups (ADC12 and ADC345) select synchronous HCLK /1, /2, /4 or an RCC asynchronous kernel clock divided using CCR.PRESC. ADC results can request DMA through DMAMUX.

**Use:** configure RCC's ADC kernel source and the common CCR clock mode/prescaler, select rank channels via SQR and sample times via SMPR, enable/start the ADC, then observe DR/status or route requests through DMAMUX/DMA. Common clock fields cannot change while a group member is enabled; synchronous /1 needs undivided AHB.

**Limits:** channels 0–19 only; no injected sequence, external trigger routing, analog watchdog, oversampling, calibration delay/data, or full alignment/quantization behavior. Timing uses integer rates and nanosecond deadlines. Electrical settling and datasheet limits are not modeled. Decimation/lazy scheduling are simulator contracts, not hardware features.

**RM0440:** §21.4.3, §21.4.8–16, §21.4.22–26; §21.6; §21.7.1–3 (`ISR/IER/CR`), §21.7.6–7 (`SMPR1/2`), §21.7.11–15 (`SQR1–4/DR`), §21.8.2 `CCR`; ADC DMA request mapping §14.3 Table 100.

**Tests:** `PeripheralTest.AdcRegularSequenceCoversAllSixteenRanks`; `PeripheralTest.AdcSamplingSelectorsAndResolutionTimingCoverSmprBoundary`; `AdcClockTest.ClockMuxStopAndResumePreservesRemainingConversionProgress`; `AdcClockTest.CcrClockFieldsCannotChangeWhileAnyGroupMemberIsEnabled`; `Stm32G4Test.Adc5RoutesItsDmaRequestAndDedicatedInterrupt`.

See the [coverage matrix](../stm32g4_peripheral_coverage.md#adc-regular-conversion-and-common-control) for detailed clock behavior, scheduling edge cases, and gaps.
