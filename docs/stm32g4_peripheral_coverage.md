# STM32G4 peripheral coverage

The STM32G4 layer is a deterministic register-level model for startup, RTOS scheduling, and device-facing firmware tests. It preserves little-endian byte/halfword/word register accesses and implements selected side effects. It is not a complete STM32G474 electrical, timing, or register-accuracy model.

## Evidence and scope

The reference is **RM0440**, the STM32G4 reference manual, rather than an
analog/electrical device datasheet. Citations below identify a numbered section
and, where relevant, its register/field. They are stable across local Markdown
line wrapping; no external firmware checkout is needed to run the tests.

A named test is evidence only for the assertions it makes, not the entire cited
section. The matrices distinguish **conformant subset**, **simplified model
contract**, and **unsupported** behavior. Simulation-only hooks and optimization
invariants are not hardware-conformance evidence. Generic register storage does
not imply implementation of the feature associated with that register.

This audit covers peripheral and system-window side effects. Instruction-set
coverage is separate: see [Thumb instruction coverage](thumb_instruction_coverage.md).
RM0440 delegates most Cortex-M register and exception semantics to PM0214; those
semantics are not labeled RM0440-verified without an actual specification check.

Check that the matrix names real tests and numbered manual sections with:

```bash
bash tools/check_peripheral_coverage.sh
bash tools/check_peripheral_coverage.sh --manual-dir /path/to/STM32G4_RM0440_chapters
```

The checker validates reference integrity, not assertion completeness or hardware
accuracy. Run the tests as described in [Testing](testing.md).

## Routed map

| Device | Address or instances | Current level |
|---|---|---|
| RCC | `0x40021000` | Startup clock side effects |
| FLASH | `0x40022000` | Startup/control side effects |
| PWR | `0x40007000` | Permissive register storage |
| GPIO | GPIOA-G at `0x48000000 + port * 0x400` | Digital input/output behavior |
| USART | USART1 `0x40013800`, USART2 `0x40004400`, USART3 `0x40004800` | Scripted byte data path |
| TIM | TIM1-8, TIM15-17, TIM20 at STM32G474 bases | Basic up-counter/update behavior |
| ADC | ADC1/2 at `0x50000000/0x50000100`, ADC3/4/5 at `0x50000400/0x50000500/0x50000600` | Clock-timed regular sequences |
| ADC common | ADC12 at `0x50000300`, ADC345 at `0x50000700` | Shared CCR-backed ADC clock groups |
| CRC | `0x40023000` | Configurable MSB-first 32-bit CRC accumulator |
| SPI | SPI1 `0x40013000`, SPI2 `0x40003800`, SPI3 `0x40003c00` | Immediate byte/halfword transfers |
| DMA | DMA1 `0x40020000`, DMA2 `0x40020400` | Request-driven channel transfers |
| DMAMUX | `0x40020800` | Request selection and ADC routing |
| IWDG / WWDG | `0x40003000` / `0x40002c00` | Deterministic timeout scheduling |
| FDCAN | FDCAN1-3 at `0x40006400`, `0x40006800`, `0x40006c00` | Message-RAM TX/RX and interrupts |
| FDCAN message RAM | `0x4000a400`, three `0x350`-byte slices | CPU-visible shared storage |
| SYSCFG / EXTI | `0x40010000` / `0x40010400` | EXTI line routing and edge-triggered interrupts |

Within a modeled register block, registers without a special hook generally retain written values. An address outside all routed blocks is handled by the top-level MMIO policy: lenient mode returns zero and counts the address, while `--strict-mmio` faults.

### Shared MMIO model contracts

`PeripheralTest.MergesRegisterByteLanesAndRejectsOutOfBlockWrites` checks the
shared little-endian register backing, byte/halfword lane merging, reset, and
out-of-block rejection without mutation. Cross-register accesses supported by
that backing are **not** proof that a particular hardware register permits them.
`PeripheralTest.StoresRegistersAndUnknownMmio` checks lenient sparse storage and
strict unknown-MMIO failure; these are simulator policies, not RM0440 features.

## Implemented device behavior

| Device | Implemented and tested semantics | Intentional simplifications or gaps |
|---|---|---|
| RCC | HSI reset state; immediate oscillator-ready reflection; SW-to-SWS reflection; PLL-derived system-clock estimate; clock-change callback to timers; CICR clearing. The ADC clock tree models HPRE-derived HCLK, independent ADC12SEL/ADC345SEL selection of no clock, PLLP, or SYSCLK, PLLP source readiness/output enable and both divider encodings, and ADC group bus-clock gates/resets. A bus-clock gate stops synchronous ADC clocking but does not gate the independent asynchronous kernel. Representative HSE/PLL startup and ADC clock-tree configuration are unit covered. | Oscillator startup delays, failures beyond absent HSE, clock security, electrical duty cycle, voltage/frequency operating limits, and exact PLL programming constraints are not modeled. General CPU/timer timing still uses the existing SYSCLK estimate rather than a complete AHB/APB tree; non-ADC peripheral gates/resets remain unsupported. The legacy SYSCLK estimator retains a permissive fixed MSI estimate, but reserved PLL sources do not supply the ADC PLLP clock. |
| FLASH | Main/option key unlock and relock, locked-CR protection, ACR mask/readback and cache-reset pulse clearing, SR W1C, and synchronous selected-page erase through an attached backing/callback. | No timed erase/program sequence, busy interval, hardware programming checks, mass erase, option-byte reload, ECC, ART cache contents, or full bank/DBANK geometry. Direct writes to mapped flash are a simulation facility, not FLASH_CR.PG programming conformance. |
| PWR | Register storage through CR5 (`0x80`), Range 1 normal-mode reset (`CR1.VOS=01`, `CR5.R1MODE=1`), writable boost selection, and always-complete voltage-scaling status. Supports Range 1 boost startup without falling through strict MMIO. | No voltage domains, stop/standby transitions, wakeup pins, backup-domain rules, or supply effects. |
| GPIO | MODER/ODR/IDR storage, BSRR/BRR atomic updates, external input overrides, output mirroring, timestamped transitions and callbacks. Unit and integrated routing covered. | No pull resistors, open-drain/electrical levels, slew, contention, alternate-function routing, or clock gating. GPIO level changes drive EXTI edge detection through SYSCFG routing. |
| USART | Scripted/provider RX queue, RDR pop, TDR TX log/callback, binary board `tx_log` output, RXNE/IDLE/TC/TXE and TEACK/REACK flags, selected CR1 interrupts, CR3 DMAT/DMAR DMA requests with burst transfers, ICR/RQR clearing. Unit covered including DMA request generation. | TX is instantaneous; no baud timing, framing, parity, oversampling, FIFO depth, errors, CTS/RTS, or synchronous modes. |
| TIM | CR1 enable/one-pulse, PSC, ARR, CNT, EGR update, UIF/UIE, live counter from simulated time, update events/IRQs, RCC clock-change input. Exact-period update is unit covered. | One generic up-counter model is reused for all listed timers. Capture/compare, PWM, repetition/dead time, encoder/slave modes, complementary outputs, timer chaining, and most status bits are absent. |
| ADC | Immediate calibration/enable readiness; regular sequences of up to 16 SQR ranks; per-channel SMPR and resolution-derived conversion timing; DR/EOC/EOS; single and continuous modes; interrupts; and constant or simulated-time sine providers sampled at conversion completion. ADC1-5 conversion requests route through DMAMUX to DMA. Independent ADC12/ADC345 common groups select synchronous HCLK /1, /2, /4 or the RCC asynchronous kernel clock divided by CCR.PRESC /1, /2, /4, /6, /8, /10, /12, /16, /32, /64, /128, /256. Missing/disabled sources and reserved encodings stop conversions rather than substituting SYSCLK. Clock stops preserve pending progress, including lazy conversions and decimated gaps; cancellation and group reset discard it. CCR clock fields are protected while any member ADC is enabled. Synchronous /1 requires undivided AHB; forbidden divided-AHB configurations are treated as clockless. Clock-tree and conversion scheduling are unit covered. | Channels 0-19 only; no injected sequences, external trigger routing, alignment/oversampling effects, calibration data/startup delays, or analog watchdog behavior. Rates use integer Hz and deadlines use nanosecond ceilings; oscillator phase, electrical duty cycle, analog settling, and datasheet operating-limit enforcement are not modeled. Out-of-spec electrical configurations are not validated. Bus-clock gating does not currently block ADC MMIO access or model delayed bus-domain result/interrupt synchronization. |
| CRC | MSB-first 32-bit accumulation with configurable polynomial/INIT, INIT-write reload, CR reset, right-aligned byte/halfword/word DR input and IDR backing. | No REV_IN/REV_OUT or non-32-bit polynomial-size algorithms; register access/reserved-bit constraints are partial. |
| SPI | TDR/DR-style immediate 1- or 2-byte transfer, RXNE/TXE/BSY, interrupt and DMA-request callbacks with burst transfers, zero/echo or custom response, stable trace. Echo path and DMA request generation are unit covered. | No serial clock or chip-select timing, frame formats beyond access width, FIFO depth, CRC, underrun/overrun, or I2S. |
| DMA | DMA1/2 channel register layout, one item per peripheral request with burst draining for serial peripherals, 1/2/4-byte widths, directions, address increment, circular count reload, TC/TE flags and completion interrupts, channel-enable callbacks, and MemoryBus copies. Unit covered. | No arbitration or transfer bandwidth, half-transfer interrupts, bursts beyond serial draining, DMA request generators, or transfer timing. |
| DMAMUX | Per-channel 7-bit request selection, overrun-flag clearing, and ADC1-5 plus USART1-3 and SPI1-3 request routing to DMA1/2 channels with burst transfers. Unit covered through ADC/DMA acceptance behavior and serial DMA request tests. | Synchronization and request generators are absent. |
| IWDG | Key unlock/start/reload, prescaler/reload storage, 32 kHz-derived timeout and reset request callback. Timeout timing is unit covered. | Status/window details are minimal. An integrated timeout stops the board with `reset-requested`; it does not automatically reset and restart the emulated MCU. |
| WWDG | Enable/reload, configurable nominal input-clock/prescaler timeout and reset-request callback. | Deadline model rather than a live hardware down-counter; window/early-refresh/EWI rules and automatic MCU reset/restart are unsupported. |
| FDCAN | INIT/CCE/clock-stop transitions; fixed message-RAM layout; standard/extended range, dual-ID, and mask filters; nonmatching policy; three-element RX FIFO 0 with acknowledge/overwrite/lost state; three TX elements; classic CAN/CAN-FD payloads through 64 bytes; IR W1C, IE/ILS/ILE line gating, RX and TX-complete interrupts. Dedicated unit tests cover control, filters, TX/RX RAM, FIFO state, and both interrupt lines. | Only RX FIFO 0 storage is functional. No arbitration or wire time, ACK/retry, bus-off/error counters, remote frames, timestamp clock, TX event FIFO, RX FIFO 1, high-priority messages, protocol timing, or full M_CAN configuration rules. Detached TX completes locally. |
| SYSCFG / EXTI | SYSCFG EXTICR1-4 port routing (PA-PG, reserved values unrouted); EXTI IMR/RTSR/FTSR/SWIER/PR with rising/falling edge detection from GPIO level changes, W1C PR clearing, software triggers, and NVIC lines EXTI0-4 (IRQ6-10), EXTI9_5 (IRQ23), EXTI15_10 (IRQ40) with shared-line OR. Dashboard button routing and shared-line behavior are unit covered. | No SYSCFG remap/compensation behavior, EMR event generation, IMR2/EMR2/RTSR2/FTSR2 lines 32+, or strict per-register validation. |

## Reference-to-test matrices

### RCC

| Behavior | Reference | Named test evidence | Fidelity / limits |
|---|---|---|---|
| HSI/HSE/PLL enable/ready, reset selection and absent HSE | RM0440 §7.4.1 `CR`; RM0440 §7.4.3 `CFGR.SW/SWS` | `PeripheralTest.RccReadyFlagsMirrorEnableAndHonorAbsentHse`; `PeripheralTest.ModelsClockFlashAndGpioStartup` | Selected flags/reset fields tested. Readiness is immediate, not oscillator startup timing. |
| LSE/LSI/HSI48 enable/ready and disabling | RM0440 §7.4.27 `BDCR`; RM0440 §7.4.28 `CSR`; RM0440 §7.4.29 `CRRCR` | `PeripheralTest.RccLowSpeedAndRecoveryOscillatorsMirrorOnAndHardwareReady` | Immediate-ready model; no stabilization/failure delays. |
| SYSCLK source/PLL arithmetic and immediate SW/SWS reflection | RM0440 §7.4.3 `CFGR`; RM0440 §7.4.4 `PLLCFGR` | `PeripheralTest.RccSysclkPllCalculationIsExplicitlyPermissive`; `Stm32G4Test.AdcCommonClockTracksRccPllChanges` | **Divergent legacy estimator**: accepts not-ready/reserved sources and fixed MSI/fallback rates. Not proof of valid RCC switching or PLL operating limits. |
| CICR clears selected CIFR bits and resets | RM0440 §7.4.6 `CIFR`; RM0440 §7.4.7 `CICR` | `PeripheralTest.RccCicrClearsOnlyWrittenFlagsAndResetRestoresStartup` | Clear-hook contract only: test primes generic writable CIFR backing; hardware CIFR is read-only. Flag generation/CIER IRQ are unsupported. |
| HPRE and independent asynchronous ADC source selection | RM0440 §7.4.3 `HPRE`; RM0440 §7.4.26 `CCIPR.ADC12SEL/ADC345SEL` | `AdcClockTest.AsynchronousSysclkHonorsEveryPrescalerAndIgnoresHpre`; `AdcClockTest.SynchronousModesHonorEveryAhbDividerAndIgnoreAsyncPrescaler`; `AdcClockTest.IndependentGroupsSelectPllpOrSysclkWithoutChangingSysclk` | Conformant clock-selection subset; integer-Hz rates/nanosecond deadlines. Other APB/peripheral mux trees are unsupported. |
| PLLP source/output enable, programmable and legacy dividers, independence from PLLR | RM0440 §7.4.4 `PLLPEN/PLLP/PLLPDIV/PLLSRC/PLLM/PLLN` | `AdcClockTest.PllpSupportsLegacyAndProgrammableDividers`; `AdcClockTest.PllpFrequencyIsIndependentOfPllrDivider`; `AdcClockTest.DisabledPllpOutputAndMissingPllSourceDoNotFallBack`; `AdcClockTest.MissingAndReservedAsyncSourcesProduceNoConversions` | Valid arithmetic/source subset; no PLL startup/voltage/frequency constraint enforcement. Reserved fields are deterministically clockless. |
| Separate ADC bus gates and asynchronous kernels | RM0440 §7.4.15 `AHB2ENR.ADC12EN/ADC345EN`; RM0440 §21.4.3, Figure 83 | `AdcClockTest.SynchronousBusClockGatePausesAndResumesConversion`; `AdcClockTest.BothGroupsHonorBusGatesInEverySynchronousMode`; `AdcClockTest.AsynchronousKernelIsIndependentOfTheBusClockGate` | Clock-domain model; no bus-access blocking or bus-domain result synchronization. |
| ADC group reset and full-machine reset clock notification | RM0440 §7.4.9 `AHB2RSTR.ADC12RST/ADC345RST` | `AdcClockTest.RccGroupResetCancelsOnlyItsOwnConversionsAndCommonRegisters`; `AdcClockTest.FullMachineResetClearsLiveAdcClockInputs` | Reset/cancellation subset; non-ADC peripheral RCC resets are storage-only. |

### FLASH (category 3)

The configured STM32G474 uses the category-3 FLASH description in RM0440 chapter
3. Tests of an attached erase callback do not establish DBANK-dependent physical
geometry or electrical programming behavior.

| Behavior | Reference | Named test evidence | Fidelity / limits |
|---|---|---|---|
| ACR reset `0x00040601`, writable latency/cache mask and self-clearing cache-reset pulses | RM0440 §3.7.1 `FLASH_ACR` | `FlashPeripheralTest.AcrMasksWritesAndClearsCacheResetPulses`; `RealTimingTest.FlashResetEnablesCachesAndOneWaitState` | Conformant tested reset/field subset; no physical cache contents. |
| Latency/prefetch/cache readback and proof-generation invalidation | RM0440 §3.3.3; RM0440 §3.7.1 `LATENCY/PRFTEN/ICEN/DCEN` | `RealTimingTest.FlashAcrProgramsLatencyAndPrefetch`; `RealTimingTest.FlashStallWithoutArtHitsEveryFetch`; `BoardTest.IdempotentPeriodCacheRejectsChangedFlashClockAndCodeGeneration` | Fetch stalls and cache/proof generations are **model contracts**, not ART cycle-accuracy evidence. |
| Main/option key sequences, relocking and protection of locked CR/subword writes | RM0440 §3.5.5; RM0440 §3.7.3 `KEYR`; RM0440 §3.7.4 `OPTKEYR`; RM0440 §3.7.6 `CR.LOCK/OPTLOCK` | `FlashPeripheralTest.KeySequencesUnlockAndRelockControlAndOptionLock`; `FlashPeripheralTest.EraseFailureClearsBusyWithoutReportingEopAndLockedPartialCrWriteIsIgnored`; `FlashPeripheralTest.IgnoresEraseWhileLocked`; `PeripheralTest.ModelsClockFlashAndGpioStartup` | Valid unlock/relock subset. Wrong keys merely abandon the sequence; hardware lock-until-reset/bus-error behavior is not modeled. |
| Selected page/bank erase, neighboring-page preservation, EOP and SR W1C | RM0440 §3.5.6; RM0440 §3.7.5 `SR`; RM0440 §3.7.6 `CR.PER/PNB/BKER/STRT` | `FlashPeripheralTest.StatusW1cAndPageEraseMapsBankAndPage`; `FlashPeripheralTest.ErasesSelectedPageThroughMappedBacking`; `FlashPeripheralTest.ReprogramsErasedPages`; `FlashIntegrationTest.RoutesEraseThroughStm32G4` | Immediate erase callback/backing model; no visible busy interval, timed programming/mass erase, full bank geometry or ECC. |
| Failed callback clears busy without EOP | RM0440 §3.7.5 `SR.BSY/EOP` (hardware error flags unsupported) | `FlashPeripheralTest.EraseFailureClearsBusyWithoutReportingEopAndLockedPartialCrWriteIsIgnored` | Simulator failure contract, not hardware erase-error reporting. |

### PWR

| Behavior | Reference | Named test evidence | Fidelity / limits |
|---|---|---|---|
| Range-1 VOS reset/readback, R1MODE reset/boost selection, VOSF read | RM0440 §6.1.5; RM0440 §6.4.1 `CR1.VOS`; RM0440 §6.4.9 `SR2.VOSF`; RM0440 §6.4.22 `CR5.R1MODE` | `PeripheralTest.PwrResetVoltageRangeAndBoostFieldAreStable`; `PeripheralTest.ModelsRangeOneBoostControl`; `PeripheralTest.StoresRegistersAndUnknownMmio` | Tested register subset. VOSF is always clear; regulator transitions, voltage limits and low-power state machines are unsupported. |

### GPIO

| Behavior | Reference | Named test evidence | Fidelity / limits |
|---|---|---|---|
| Port A/B debug-pin MODER reset exceptions and C–G reset modes | RM0440 §9.4.1 `MODER` reset values; RM0440 §9.4.12 register map | `PeripheralTest.GpioResetRestoresPortSpecificDebugPinModes` | Conformant tested MODER reset values; not all other per-port reset fields. |
| BSRR set/reset, set-over-reset priority, BRR, subword commands and ODR | RM0440 §9.4.6 `ODR`; RM0440 §9.4.7 `BSRR`; RM0440 §9.4.11 `BRR` | `PeripheralTest.GpioBsrrPriorityBrrSubwordAndExternalRelease`; `PeripheralTest.ModelsClockFlashAndGpioStartup`; `Stm32G4Test.RoutesIntegratedPeripherals` | Tested digital update subset; no electrical output type/contention model. |
| MODER-controlled IDR mirror, external override/release and output callback | RM0440 §9.4.1 `MODER`; RM0440 §9.4.5 `IDR` | `PeripheralTest.GpioBsrrPriorityBrrSubwordAndExternalRelease`; `PeripheralTest.ModelsClockFlashAndGpioStartup` | Simulator pin/provider contract, not electrical input sampling. |
| OTYPER/OSPEEDR/PUPDR/LCKR/AFRL/AFRH | RM0440 §9.4.2; RM0440 §9.4.3; RM0440 §9.4.4; RM0440 §9.4.8; RM0440 §9.4.9; RM0440 §9.4.10 | Shared backing: `PeripheralTest.MergesRegisterByteLanesAndRejectsOutOfBlockWrites` | **Storage-only**; lock sequencing, alternate-function routing, pulls/slew and electrical semantics are unsupported. |

### CRC

| Behavior | Reference | Named test evidence | Fidelity / limits |
|---|---|---|---|
| MSB-first word CRC and configurable 32-bit polynomial | RM0440 §16.3.3; RM0440 §16.4.1 `DR`; RM0440 §16.4.5 `POL` | `CrcPeripheralTest.MatchesCrc32Mpeg2Reference`; `CrcPeripheralTest.HonorsPolynomialRegister`; `CrcIntegrationTest.RoutesThroughStm32G4` | Tested non-reversed 32-bit subset; odd-polynomial/write-access restrictions are incomplete. |
| Right-aligned byte/halfword input, INIT write reload and CR reset | RM0440 §16.3.3; RM0440 §16.4.1 `DR`; RM0440 §16.4.3 `CR.RESET`; RM0440 §16.4.4 `INIT` | `CrcPeripheralTest.SubwordInputsInitReloadAndIdrSemantics`; `CrcPeripheralTest.ResetsFromConfiguredInit` | Conformant tested input/init/reset subset. |
| Eight-bit IDR storage, independent of CR accumulation reset | RM0440 §16.4.2 `IDR` | `CrcPeripheralTest.SubwordInputsInitReloadAndIdrSemantics` | Tested field width/reset independence. |
| REV_IN/REV_OUT and non-32-bit POLYSIZE | RM0440 §16.4.3 `CR` | `CrcPeripheralTest.ResetsFromConfiguredInit` characterizes ignored REV_OUT | **Unsupported**, not conformance coverage. Stored control bits do not implement these algorithms. |

### ADC regular conversion and common control

| Behavior | Reference | Named test evidence | Fidelity / limits |
|---|---|---|---|
| CKMODE/PRESC selection, protection while members are enabled, synchronous /1 HPRE restriction | RM0440 §21.4.3; RM0440 §21.8.2 `CCR.CKMODE/PRESC` | `AdcClockTest.AsynchronousSysclkHonorsEveryPrescalerAndIgnoresHpre`; `AdcClockTest.SynchronousModesHonorEveryAhbDividerAndIgnoreAsyncPrescaler`; `AdcClockTest.CcrClockFieldsCannotChangeWhileAnyGroupMemberIsEnabled`; `AdcClockTest.SynchronousDivideByOneRejectsDividedAhbClock`; `Stm32G4Test.AdcCommonCkmodeClocksSeparateGroupsAndHonorsPartialCcrWrites` | Conformant tested register/clock subset. Electrical 50% duty-cycle and datasheet maximum rates are not enforced. |
| Clock change/start without clock, suspension, resume, cancellation and completion-callback stop | RM0440 §21.4.3 clock domain; RM0440 §21.4.15 starting conversions | `AdcClockTest.ClockMuxStopAndResumePreservesRemainingConversionProgress`; `AdcClockTest.RccAhbDividerChangeReschedulesSynchronousConversion`; `AdcClockTest.ConversionStartedWithoutAClockWaitsUntilSourceIsSelected`; `AdcClockTest.DisablingSuspendedConversionDoesNotResurrectItOnResume`; `AdcClockTest.StoppingClockInsideSampleCallbackSuspendsTheNextRank`; `AdcClockTest.StoppingClockOnFinalSingleShotSampleDoesNotRestartOnResume` | Deterministic scheduling model with nanosecond ceilings, not oscillator-phase or asynchronous synchronization accuracy. |
| Calibration completion, ADEN/ADRDY, ADDIS cancellation, selected ISR W1C | RM0440 §21.4.8 calibration; RM0440 §21.4.9 on/off; RM0440 §21.7.1 `ISR`; RM0440 §21.7.3 `CR` | `PeripheralTest.AdcControlFlagsAndStatusClearing`; `PeripheralTest.DrivesTimerAndAdcFromSimulatedTime` | Flags/control subset tested; calibration and readiness are instantaneous and CR ordering restrictions are incomplete. |
| All 16 SQR ranks, channel order, per-rank EOC and final EOS | RM0440 §21.4.11 channel selection; RM0440 §21.7.11 `SQR1`; RM0440 §21.7.12 `SQR2`; RM0440 §21.7.13 `SQR3`; RM0440 §21.7.14 `SQR4` | `PeripheralTest.AdcRegularSequenceCoversAllSixteenRanks`; `PeripheralTest.SequencesAdcChannelsWithRegisterDerivedTiming` | Regular-sequence subset; injected sequences/external triggers are unsupported. |
| All sampling selectors and RES conversion-cycle encodings, channels 9/10 SMPR boundary | RM0440 §21.4.12 sampling; RM0440 §21.4.16 timing; RM0440 §21.4.22 resolution; RM0440 §21.7.6 `SMPR1`; RM0440 §21.7.7 `SMPR2` | `PeripheralTest.AdcSamplingSelectorsAndResolutionTimingCoverSmprBoundary`; `PeripheralTest.SequencesAdcChannelsWithRegisterDerivedTiming` | Timing encodings tested, **not** reduced-resolution DR quantization. No analog settling/offset/oversampling/alignment model. |
| SMPPLUS adds one cycle to the shortest selector in both SMPR banks | RM0440 §21.7.6 `SMPR1.SMPPLUS` | `PeripheralTest.AdcSamplePlusAddsOneCycleToShortestSample` | Tests channels 0/9/10/19 with exact 16 MHz deadline. |
| DR read clears EOC but preserves EOS; ISR W1C and IER interrupt gates | RM0440 §21.4.23 EOC; RM0440 §21.4.24 EOS; RM0440 §21.6 interrupts; RM0440 §21.7.2 `IER`; RM0440 §21.7.15 `DR` | `PeripheralTest.AdcControlFlagsAndStatusClearing`; `PeripheralTest.AdcRegularSequenceCoversAllSixteenRanks`; `PeripheralTest.AdcInterruptLevelMatchesEnabledStatusBits`; `PeripheralTest.PreservesObservableAndSingleShotAdcEvents` | Tested regular-group flag subset; OVR and most other ADC interrupt sources are unsupported. |
| Single/continuous conversion and sampling a provider at completion | RM0440 §21.4.13 single conversion; RM0440 §21.4.14 continuous conversion; RM0440 §21.4.15 start | `PeripheralTest.PreservesObservableAndSingleShotAdcEvents`; `PeripheralTest.LazilySynchronizesUnobservedContinuousAdc` | Hardware-mode subset with simulation providers/history. Lazy unobservable catch-up materializes only latest state. |
| ADC-to-DMAMUX/DMA route and dedicated ADC5 IRQ | RM0440 §21.4.26 data management; RM0440 §14.3, Table 100 | `Stm32G4Test.Adc5RoutesItsDmaRequestAndDedicatedInterrupt`; `PeripheralTest.DmaOwnerLocalAdcTransferRejectsPublicObserversAndSubstitutions` | Routing/ownership tests do **not** prove CFGR.DMAEN/DMACFG, OVR or DMA transfer pacing. Those ADC hardware semantics remain incomplete. |
| Lazy deadlines/ownership and decimation gaps retain phase across clock stops | No hardware equivalent for lazy scheduling/decimation; clock context RM0440 §21.4.3 | `AdcClockTest.ClockStopPreservesLazyContinuousDeadline`; `AdcClockTest.ClockStopPreservesDecimationGapLanding`; `PeripheralTest.DecimatesContinuousAdcScansWithoutDriftingSchedule`; `PeripheralTest.DecimationSkipsMultiRankScansAsWholeScans`; `PeripheralTest.DecimationNeverSkipsSingleShotAdc`; `PeripheralTest.DecimationFactorZeroMeansOne`; `Stm32G4Test.CertifiesOnlyTrustedAdcEventsAndRechecksPendingHooks` | **Simulator contracts**, not hardware-conformance evidence. Decimation intentionally drops externally observable samples/flags/DMA/IRQs. |

### TIM

The same generic up-counter is used for advanced, general-purpose and basic
timer instances. The cited advanced-timer registers identify the tested fields,
not a claim that every instance-specific feature is implemented. Corresponding
timer families are described in RM0440 §30.5, RM0440 §31.7 and RM0440 §32.4.

| Behavior | Reference | Named test evidence | Fidelity / limits |
|---|---|---|---|
| CEN, live CNT, PSC/ARR period, input-clock changes, CNT write and stop/restart | RM0440 §29.3.3 time base; RM0440 §29.6.1 `CR1`; RM0440 §29.6.12 `CNT`; RM0440 §29.6.13 `PSC`; RM0440 §29.6.14 `ARR` | `PeripheralTest.TimerPrescalerClockChangeCntWriteAndStopRestartRephase`; `PeripheralTest.TimerForcedUpdateStatusAndOnePulseRestart`; `PeripheralTest.DrivesTimerAndAdcFromSimulatedTime` | **Simplified**: PSC writes take effect immediately/rephase, whereas hardware PSC is buffered. No complete AHB/APB timer-clock tree or instance-specific counter widths. |
| UG forced update, UIF/UIE, W0C, update callback, OPM stop | RM0440 §29.6.1 `CR1.OPM`; RM0440 §29.6.4 `DIER.UIE`; RM0440 §29.6.5 `SR.UIF`; RM0440 §29.6.6 `EGR.UG` | `PeripheralTest.TimerForcedUpdateStatusAndOnePulseRestart`; `PeripheralTest.ReportsPeripheralInterruptLevelsAndRependsOnEnable`; `Stm32G4Test.TimerInterruptRependsUntilSourceCleared` | Tested generic update subset; compare/PWM/capture, direction/center alignment, repetition/dead time, slave/encoder/chaining and other status bits are unsupported. |
| Shared update IRQ remains asserted until all contributors clear | RM0440 §14.3, Table 100 | `Stm32G4Test.SharedTimerIrqRemainsAssertedUntilAllSourcesClear`; `Stm32G4Test.ResetDoesNotRelatchStaleSharedInterruptSources`; `Stm32G4Test.DestructionDeassertsPeripheralInterruptLines` | Routed shared-source/teardown model tests. |

### IWDG and WWDG

| Behavior | Reference | Named test evidence | Fidelity / limits |
|---|---|---|---|
| IWDG key protection, start/reload, PR/RLR and exact nominal timeout | RM0440 §35.3.5 access protection; RM0440 §35.4.1 `KR`; RM0440 §35.4.2 `PR`; RM0440 §35.4.3 `RLR` | `PeripheralTest.IwdgLockedWritesAndExactReloadDeadline`; `PeripheralTest.IwdgPrescalerCodesProduceNominalTimeoutDeadlines`; `PeripheralTest.CompletesDmaAndWatchdogSideEffects` | Tested nominal 32 kHz calculation, including PR=7 /256 alias and refresh. No LSI tolerance/startup or low-power behavior. |
| IWDG status and timeout callback | RM0440 §35.4.4 `SR`; RM0440 §35.3 functional description | `PeripheralTest.IwdgPrescalerCodesProduceNominalTimeoutDeadlines`; `PeripheralTest.CompletesDmaAndWatchdogSideEffects` | Status is always zero; PVU/RVU synchronization and WINR window are unsupported. Timeout callback is not automatic MCU reset/restart. |
| WWDG CR reload/enable, CFR prescaler, refresh and reset-disabled callback setting | RM0440 §36.3.2 enable; RM0440 §36.3.3 down-counter; RM0440 §36.3.4 timeout; RM0440 §36.5.1 `CR`; RM0440 §36.5.2 `CFR` | `PeripheralTest.WwdgCounterAndCfrPrescalerSetTimeoutDeadline` | **Model contract**, not full WWDG conformance: no live down-counter/window/EWI, and model WDGA clearing cancels the timer although hardware WDGA cannot be cleared by software. |
| WWDG SR/EWI and integrated MCU reset | RM0440 §36.4 interrupts; RM0440 §36.5.3 `SR` | No conformance test claimed | **Unsupported/unverified**. Integrated watchdogs are constructed reset-disabled; standalone reset callbacks are tested, not a full MCU restart. |

### USART

| Behavior | Reference | Named test evidence | Fidelity / limits |
|---|---|---|---|
| UE/TE/RE drive TEACK/REACK | RM0440 §40.8.1 `CR1`; RM0440 §40.8.9 `ISR` | `PeripheralTest.UsartAcknowledgementsIdleRefillAndInterruptGates` | Tested acknowledge subset; no baud/clock-domain modeling. |
| IDLE after the modeled gap and IDLECF clear; RXFRQ drains the queue and RXNE | RM0440 §40.5.6 receiver; RM0440 §40.8.8 `RQR`; RM0440 §40.8.9 `ISR`; RM0440 §40.8.11 `ICR` | `PeripheralTest.UsartAcknowledgementsIdleRefillAndInterruptGates` | Idle gap is an emulator approximation, not baud/frame timing. |
| Each CR1 source independently gates its status onto the interrupt line, and masking preserves status | RM0440 §40.8.1 `CR1` RXNEIE/IDLEIE/TCIE/TXEIE | `PeripheralTest.UsartInterruptEnablesIndependentlyAssertAndDeassert` | Gate/level subset; TXE without data is asserted by the idle model. |
| RDR pop, TDR byte log/callback, immediate TC, RX queue and TX path | RM0440 §40.5.5 transmitter; RM0440 §40.5.6 receiver; RM0440 §40.8.12 `RDR`; RM0440 §40.8.13 `TDR` | `PeripheralTest.ModelsUsartAndSpiDataPaths`; `PeripheralTest.UsartTransmissionCompleteCanBeAcknowledged`; `Stm32G4Test.UsartInterruptStopsRependingAfterReceiveConsumed`; `Stm32G4Test.RoutesIntegratedPeripherals` | Instantaneous byte model; no framing/parity/FIFO/CTS/RTS/synchronous modes. |
| CR3 DMAT/DMAR request generation | RM0440 §40.5.19 DMA; RM0440 §40.8.4 `CR3` | `PeripheralTest.SignalsUsartDmaRequestsForTxAndRx`; `Stm32G4Test.IntegratedUsartAndSpiTransfersReachDmaThroughDmamux` | Request/flag subset with an integrated DMA route; no baud-paced transfer timing. |
| Status-observation RX provider refill | No hardware equivalent | `PeripheralTest.UsartAcknowledgementsIdleRefillAndInterruptGates` | **Simulator contract**: the provider is pulled when firmware observes an empty receiver. |

### SPI

| Behavior | Reference | Named test evidence | Fidelity / limits |
|---|---|---|---|
| DR write transfers bytes/halfwords per access width, callback/echo response, RX queue consumed by DR read, RXNE/TXE/brief BSY, transfer log | RM0440 §42.5.9 transfer procedure; RM0440 §42.5.10 status flags; RM0440 §42.9.3 `SR`; RM0440 §42.9.4 `DR` | `PeripheralTest.SpiConsumesFramesAndSignalsReceiveDma`; `PeripheralTest.ModelsUsartAndSpiDataPaths` | Immediate modeled response; only byte/halfword widths regardless of configured DS. No serial clock/polarity/phase/NSS sequencing. |
| CR2 RXNEIE/TXEIE interrupt gates and RXDMAEN/TXDMAEN requests | RM0440 §42.9.2 `CR2` | `PeripheralTest.SpiConsumesFramesAndSignalsReceiveDma`; `PeripheralTest.ReportsPeripheralInterruptLevelsAndRependsOnEnable`; `Stm32G4Test.IntegratedUsartAndSpiTransfersReachDmaThroughDmamux` | Request/gate subset with an integrated DMA route; FIFO depth/CRC/underrun/overrun/I2S unsupported. |

### DMA and DMAMUX

| Behavior | Reference | Named test evidence | Fidelity / limits |
|---|---|---|---|
| Byte/halfword/word widths in both directions with PINC/MINC matrices | RM0440 §12.4.5 channel controls; RM0440 §12.4.6 data width | `PeripheralTest.DmaByteHalfwordWordAndBothDirectionsHonorIncrementBits` | Tested width/direction/increment subset; no alignment/FIFO/arbitration enforcement. |
| CNDTR decrement/reload, circular mode keeps EN, TCIF/TEIF flags, disable on complete/error, TCIE/TEIE IRQ, IFCR individual and global clears | RM0440 §12.4.3 transfers; RM0440 §12.4.5 `CIRC`; RM0440 §12.4.7 error management; RM0440 §12.6.1 `ISR`; RM0440 §12.6.2 `IFCR`; RM0440 §12.6.3 `CCRx`; RM0440 §12.6.4 `CNDTRx` | `PeripheralTest.DmaWidthsIncrementsCircularAndFailureFlags`; `PeripheralTest.CompletesDmaAndWatchdogSideEffects`; `PeripheralTest.DmaGlobalFlagClearDeassertsOnlySelectedChannel` | No half-transfer progress, transfer timing, bursts beyond serial draining, or hardware bus-error detail. Requests are software/callback-driven. |
| DMAMUX request selector retained, routing generation stable, CSR read-only with no modeled overrun source | RM0440 §13.6.1 `CxCR`; RM0440 §13.6.2 `CSR` | `PeripheralTest.DmamuxRequestSelectorAndReadOnlyClearStatus`; `PeripheralTest.CompletesDmaAndWatchdogSideEffects` | Selector/storage subset. Overrun flags are never generated, so CFR clearing of a set SOFx is untestable; request generators and synchronization logic are absent. |
| Peripheral-to-DMA routing through DMAMUX selectors | RM0440 §13.6.1 `CxCR` | `Stm32G4Test.Adc5RoutesItsDmaRequestAndDedicatedInterrupt`; `Stm32G4Test.IntegratedUsartAndSpiTransfersReachDmaThroughDmamux` | Tested ADC5/USART1-TX/SPI1-RX routes; not every request ID end to end. |

### FDCAN

Only RX FIFO 0 (three elements) and three TX buffers are functional. There is
no bit timing, arbitration, ACK/retry, error accounting, TX event FIFO,
RX FIFO 1, or full M_CAN configuration-rule conformance. XIDAM is stored but
not applied.

| Behavior | Reference | Named test evidence | Fidelity / limits |
|---|---|---|---|
| INIT/CCE/clock-stop transitions | RM0440 §44.4.6 `CCCR` | `FdcanTest.ModelsClockStopAndInitTransitions` | Tested control subset. |
| Shared message-RAM bounds, little-endian element layout | RM0440 §44.3.6 message RAM | `FdcanTest.ExposesSharedMessageRamAsMmio`; `FdcanTest.TransmitsAndReceivesThroughMessageRam` | Fixed three-slice layout; detached TX completes locally. |
| Standard range filter, nonmatch reject, FIFO0 full/lost, acknowledge release and overwrite payload | RM0440 §44.3.11 standard filters; RM0440 §44.3.6 message RAM; RM0440 §44.3.7 FIFO acknowledge; RM0440 §44.4.19 `RXGFC`; RM0440 §44.4.22 `RXF0S`; RM0440 §44.4.23 `RXF0A` | `FdcanTest.FdcanStandardRangeFifoFullAndAck`; `FdcanTest.FiltersAgainstThePerMessageRamLists` | Tested FIFO0 subset including overwritten-element payload. |
| Extended range filter and nonmatch reject; standard and extended dual-ID and classic-mask filters | RM0440 §44.3.11 standard filters; RM0440 §44.3.12 extended filters | `FdcanTest.FdcanExtendedRangeFilterAndNonmatchReject`; `FdcanTest.FdcanStandardDualIdAndMaskFiltersAcceptOnlyMatches`; `FdcanTest.FdcanExtendedDualIdAndMaskFiltersAcceptOnlyMatches` | Tested filter forms; remaining mask/dual-ID combinations and XIDAM-gated matching are not asserted. |
| Classic/FD payload encode through DLC 9, TX buffer requests/status and invalid-bit rejection | RM0440 §44.3.8 RX element; RM0440 §44.3.9 TX element; RM0440 §44.4.26 TX buffer configuration | `FdcanTest.FdcanStandardRangeFifoFullAndAck`; `FdcanTest.FdcanTransmitBuffersTrackRequestsAndIgnoreInvalidBits`; `FdcanTest.TransmitsAndReceivesThroughMessageRam` | Tested buffer/payload subset; TX queue/cancel and error behavior are simplified. |
| IR W1C with IE/ILS/ILE line gating on both interrupt lines | RM0440 §44.4.15 `IR`; RM0440 §44.4.16 `IE`; RM0440 §44.4.17 `ILS`; RM0440 §44.4.18 `ILE` | `FdcanTest.GatesPendingEventsThroughInterruptRegisters`; `Stm32G4Test.FdcanInterruptRependsUntilSourceCleared`; `Stm32G4Test.FdcanInterruptLineRoutingAndDisableUpdateNvicLevels` | Tested gate/line subset. |

### SYSCFG and EXTI

Only EXTI lines 0–15 are modeled; bank-2 lines, EMR event output and other
SYSCFG functions are storage-only or unsupported.

| Behavior | Reference | Named test evidence | Fidelity / limits |
|---|---|---|---|
| EXTICR1–4 port routing across PA–PG with reserved values rejected | RM0440 §10.2.3 `EXTICR1`; RM0440 §10.2.4 `EXTICR2`; RM0440 §10.2.5 `EXTICR3`; RM0440 §10.2.6 `EXTICR4` | `Stm32G4Test.SyscfgRoutesEveryExtiLineAcrossAllPortsAndRejectsReserved`; `Stm32G4Test.ExtiRoutesGpioEdgeToNvic` | Tested routing subset. |
| GPIO edge trigger, dedicated 0–4 and shared 5–9/10–15 outputs, IMR gating and PR W1C | RM0440 §15.3.3 edge trigger; RM0440 §15.3.4 interrupt behavior; RM0440 §15.4 EXTI block diagram; RM0440 §15.5.1 `IMR1`; RM0440 §15.5.3 `RTSR1`; RM0440 §15.5.4 `FTSR1`; RM0440 §15.5.6 `PR1` | `Stm32G4Test.ExtiDedicatedAndSharedInterruptOutputsAssertAndClear`; `Stm32G4Test.ExtiRoutesGpioEdgeToNvic`; `Stm32G4Test.ExtiIgnoresUnroutedPortAndFallingWithoutTrigger`; `Stm32G4Test.ExtiSharedLinesOrIntoSingleIrq` | Tested line/bank-1 subset. |
| SWIER W1S software trigger, selective PR clear, masking preserves latched PR | RM0440 §15.5.5 `SWIER1`; RM0440 §15.5.6 `PR1` | `Stm32G4Test.ExtiSoftwareTriggerW1SAndSelectivePendingClear` | Tested software-trigger subset. |

## Cortex-M system window

The separate `0xe0000000..0xe00fffff` system device implements:

- SysTick CTRL/LOAD/VAL/COUNTFLAG and deterministic interrupt pending;
- NVIC enable, pending, active, software-trigger, and 240 priority bytes with four implemented priority bits;
- SCB CPUID, ICSR, VTOR, keyed AIRCR reset request, SCR, CCR, SHPR, SHCSR, MMFAR/BFAR storage, and CFSR/HFSR write-one-to-clear views;
- CPACR FPU-enable visibility, DEMCR, FPCCR, DWT CTRL, and gated DWT CYCCNT;
- priority selection under PRIMASK, BASEPRI, FAULTMASK, and current active exception.

BASEPRI reads, writes, and arbitration use the same four implemented priority bits
as NVIC/SHPR. Peripheral IRQ inputs are level-sensitive: an uncleared enabled source
re-pends on exception return, and shared sources remain asserted until all are
acknowledged or disabled. Software-pended interrupts remain independently latched.

Unit tests cover SysTick wrap, COUNTFLAG, NVIC enable/masking/priority, BASEPRI_MAX,
PendSV, CPACR, AIRCR, basic/extended exception entry and return, preservation of
re-pended nested interrupts, peripheral IRQ acknowledgment, shared IRQs, and reset/teardown. MPU, ITM/SWO, breakpoint/watchpoint comparators, debug transport, and automatic fault escalation are not modeled. Lazy FP preservation has model tests but is not independently checked against PM0214 here.

### System-window evidence

RM0440 §14.1 specifies 102 device IRQs and four priority bits; RM0440 §14.2
specifies the SysTick calibration literal. RM0440 §14.3, Table 100, is the
peripheral-to-NVIC routing authority. Remaining architectural register details
are delegated to PM0214 by RM0440 §14.1 and are **not independently
reference-verified here**.

| Behavior | Reference / scope | Named test evidence | Fidelity / limitation |
|---|---|---|---|
| SysTick CALIB reads `0x3e8` and ignores writes | RM0440 §14.2 | `CortexMTest.ReportsReadOnlySysTickCalibrationValue` | Conformant device-specific literal. |
| All 102 device IRQ priority bytes expose only bits 7:4 | RM0440 §14.1 | `CortexMTest.AllDeviceInterruptPrioritiesImplementExactlyFourBits`; `CortexMTest.ModelsNvicAndScb` | Conformant priority width. The generic NVIC exposes 240 IRQ slots, more than this device implements. |
| SysTick wrap at LOAD+1 cycles, interrupt pending, COUNTFLAG read-to-clear | RM0440 §14.1 delegates register semantics to PM0214 | `CortexMTest.ModelsSysTick`; `CortexMTest.MasksSysTickReloadAndClearsCountFlagOnCurrentWrite` | Tested model contract, not full SysTick conformance; alternate clock selection is not modeled. |
| Enable/pending/active state, pulse latching and level re-pending | RM0440 §14.1 delegates to PM0214 | `CortexMTest.ModelsNvicAndScb`; `CortexMTest.SetsAndClearsInterruptEnablePendingAndActiveBits`; `CortexMTest.SamplesInterruptLevelsWithoutInventingPendingEdges`; `CortexMTest.ResetClearsInterruptLevelsAndPendingSummary` | Tested model contract; 240 generic IRQ slots. |
| Masking, priority arbitration, active-handler preemption and selection-cache invalidation | RM0440 §14.1 priority width; PM0214 required for full arbitration proof | `CortexMTest.BasepriArbitrationUsesOnlyImplementedPriorityBits`; `CortexMTest.TakablePendingCacheTracksMasksPrioritiesAndLifecycle`; `CortexMTest.ModelsNvicAndScb` | Tested model contract; AIRCR priority grouping is not modeled. |
| PendSV, CPACR FPU-enable state and keyed AIRCR reset-request consumption | RM0440 §14.1 delegates to PM0214 | `CortexMTest.ModelsNvicAndScb`; `CortexMTest.SetsAndClearsSystemExceptionPendingBits`; `CortexMTest.RequiresAircrKeyAndAlignsVectorTableBase` | Tested model contract, not automatic reset/restart. |
| SHPR four-bit priorities and byte-lane preservation | RM0440 §14.1 priority width; PM0214 for register layout | `CortexMTest.MasksSystemHandlerPriorityBytesAndPreservesNeighborLanes` | Tested model contract. |
| DWT cycle count enable and 32-bit wrap | RM0440 §47.2 delegates core debug details to architecture documentation | `CortexMTest.CycleCounterAdvancesOnlyWhenEnabledAndWraps` | Model contract; DEMCR.TRCENA does not gate counting. |
| SCB/CoreDebug/FPCCR register storage, lane merging, reset, CPUID read-only literal and access bounds | RM0440 §14.1 delegates to PM0214 | `CortexMTest.StoresSystemControlLanesAndResetsThem`; `CortexMTest.RejectsCrossRegisterSystemAccesses` | Storage/model contract, not a claim of all register bits' reset/access accuracy. CFSR/HFSR have no modeled fault-generation path; clearing initially-zero status does not prove fault conformance. |
| Basic/extended frames, thread stack selection, nested return and lazy floating-point preservation | RM0440 §14.1 delegates to PM0214 | `ExceptionTest.StacksAndReturnsBasicFrame`; `ExceptionTest.StacksAndReturnsExtendedFloatingPointFrame`; `ExceptionTest.RestoresThreadStackSelectionFromExcReturn`; `ExceptionTest.NestedReturnPreservesRependedOuterInterrupt`; `ExceptionTest.DefersFloatingPointStackingUntilHandlerTouch`; `ExceptionTest.NestedEntryFallsBackToEagerStacking` | Architectural model tests; no RM0440 proof of cycle-accurate exception entry/return. |
| ADC5, FDCAN, TIM, USART and EXTI interrupt routing/shared-source OR | RM0440 §14.3, Table 100 | `Stm32G4Test.Adc5RoutesItsDmaRequestAndDedicatedInterrupt`; `Stm32G4Test.FdcanInterruptLineRoutingAndDisableUpdateNvicLevels`; `Stm32G4Test.SharedTimerIrqRemainsAssertedUntilAllSourcesClear`; `Stm32G4Test.UsartInterruptStopsRependingAfterReceiveConsumed`; `Stm32G4Test.ExtiSharedLinesOrIntoSingleIrq` | Tested routed subset, not every entry of Table 100. |

## Virtual CAN and multi-board world

`run-network` creates named in-process buses, attaches configured FDCAN instances, and dispatches equal-time boards in configuration order; the instruction quantum caps only same-timestamp fairness bursts. CAN delivery is synchronous in node attachment order; loopback is optional per node. Frame validation covers 11- and 29-bit identifiers, classic CAN limits, CAN-FD DLC mapping, and BRS validity. Trace records include normalized transmit and receive frames. Their `dlc` field is the encoded CAN DLC and `length` is the decoded payload byte count; device-facing sources use `board.device`, while bus-facing sources use `bus/node`.

The CLI option `--inject-can BUS[@TIME_MS]:ID:HEXDATA` schedules an external frame
on the shared event loop and may be repeated. A missing `@TIME_MS` means time zero.

The configured bitrate is informational. The bus does not model arbitration priority, simultaneous transmit, serialization delay, physical errors, termination, or load.

## Interpreting unknown MMIO

The `unknown_mmio_addresses` counter records only addresses that miss every
top-level routed block. It does not count generic storage inside a modeled block or
accesses absorbed by the ADC-common, SYSCFG, and EXTI sparse stubs. The tables above,
not a zero unknown-address count, define peripheral fidelity. Strict MMIO remains
useful for finding entirely unrouted ranges.

Representative tests are in `tests/unit/peripheral_test.cpp`,
`tests/unit/fdcan_test.cpp`, `tests/unit/stm32g4_test.cpp`,
`tests/unit/adc_clock_test.cpp`,
`tests/unit/cortexm_test.cpp`, `tests/unit/exceptions_test.cpp`,
`tests/unit/can_bus_test.cpp`, and `tests/unit/world_test.cpp`. External firmware
validation is described in [Testing](testing.md).
