# RCC (reset and clock control)

The RCC model provides startup-oriented oscillator flags and clock inputs for other modeled devices. It starts from the modeled HSI reset state, reflects oscillator-ready bits immediately when enabled (except absent HSE), reflects `CFGR.SW` into `SWS`, estimates SYSCLK from PLL settings, notifies timers on clock changes, and implements selected `CICR` flag clearing. ADCs additionally use the RCC HPRE-derived HCLK, independently selected ADC12/ADC345 asynchronous kernel source, PLLP output/divider, and group bus gates/resets.

**Use:** enable/configure clocks before relying on timer or ADC timing. ADC common clock selection lives in the ADC common CCRs; RCC selects its asynchronous input. ADC synchronous /1 requires undivided AHB. The general SYSCLK estimate is permissive and is not a full clock tree.

**Limits:** readiness/startup is immediate; oscillator failure/security and electrical limits are absent. Non-ADC peripheral gates/resets and general AHB/APB clock trees are not modeled. The legacy SYSCLK estimator accepts some not-ready/reserved inputs and fallback rates; reserved PLL sources do not provide ADC PLLP clock.

**RM0440:** §7.4.1 `CR`; §7.4.3 `CFGR`; §7.4.4 `PLLCFGR`; §7.4.6–7.4.7 `CIFR/CICR`; §7.4.9 `AHB2RSTR`; §7.4.15 `AHB2ENR`; §7.4.26 `CCIPR`; §7.4.27–29 `BDCR/CSR/CRRCR`.

**Tests:** `PeripheralTest.RccReadyFlagsMirrorEnableAndHonorAbsentHse`; `PeripheralTest.RccSysclkPllCalculationIsExplicitlyPermissive`; `AdcClockTest.PllpSupportsLegacyAndProgrammableDividers`; `AdcClockTest.SynchronousModesHonorEveryAhbDividerAndIgnoreAsyncPrescaler`; `AdcClockTest.AsynchronousSysclkHonorsEveryPrescalerAndIgnoresHpre`.

For complete limits and additional tests see the [coverage matrix](../stm32g4_peripheral_coverage.md#rcc).
