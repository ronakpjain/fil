#include "fil/cortexm/system_control.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include "../fixture_support.hpp"

#include <cstdint>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace {

// DMA/locality tests need an explicitly selected and enabled ADC kernel clock.
auto createAdcMcu(fil::sim::EventLoop& events, fil::sim::TraceRecorder& trace,
                  fil::cortexm::SystemControl& system) {
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true);
    if (mcu) {
        EXPECT_TRUE(mcu.value()->router().write(0x2104cU, fil::mem::AccessSize::word,
                                               (1U << 13U) | (1U << 14U), {}));
        EXPECT_TRUE(mcu.value()->router().write(0x21088U, fil::mem::AccessSize::word,
                                               (2U << 28U) | (2U << 30U), {}));
    }
    return mcu;
}

TEST(Stm32G4Test, AdcCommonCkmodeClocksSeparateGroupsAndHonorsPartialCcrWrites) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::stm32g4::AdcPeripheral adc1("ADC1", &events, &trace);
    fil::stm32g4::AdcPeripheral adc2("ADC2", &events, &trace);
    fil::stm32g4::AdcPeripheral adc3("ADC3", &events, &trace);
    fil::stm32g4::AdcPeripheral adc4("ADC4", &events, &trace);
    fil::stm32g4::AdcPeripheral adc5("ADC5", &events, &trace);
    adc1.setSampleHistoryEnabled(true);
    adc2.setSampleHistoryEnabled(true);
    adc3.setSampleHistoryEnabled(true);
    adc4.setSampleHistoryEnabled(true);
    adc5.setSampleHistoryEnabled(true);
    fil::stm32g4::AdcCommonPeripheral adc12("ADC12_COMMON", {&adc1, &adc2}, &events, &trace);
    fil::stm32g4::AdcCommonPeripheral adc345("ADC345_COMMON", {&adc3, &adc4, &adc5}, &events, &trace);
    adc12.setSystemClockHz(170'000'000U);
    adc345.setSystemClockHz(170'000'000U);
    constexpr auto word = fil::mem::AccessSize::word;
    constexpr auto half = fil::mem::AccessSize::halfword;
    // A halfword write to CCR[31:16] exercises merge/write-mask behavior.
    ASSERT_TRUE(adc12.write(0x0aU, half, 3U, {})); // CKMODE=11: HCLK/4
    ASSERT_TRUE(adc345.write(0x0aU, half, 1U, {})); // CKMODE=01: HCLK
    ASSERT_TRUE(adc1.write(0x08U, word, 1U, {}));
    ASSERT_TRUE(adc2.write(0x08U, word, 1U, {}));
    ASSERT_TRUE(adc3.write(0x08U, word, 1U, {}));
    ASSERT_TRUE(adc4.write(0x08U, word, 1U, {}));
    ASSERT_TRUE(adc5.write(0x08U, word, 1U, {}));
    for (auto* adc : {&adc1, &adc2, &adc3, &adc4, &adc5}) {
        ASSERT_TRUE(adc->write(0x08U, word, 1U | (1U << 2U), {}));
    }
    EXPECT_EQ(events.nextScheduledTime(), 89U)
        << "ADC3/4 use 170 MHz HCLK (15 conversion cycles, rounded up)";
    ASSERT_EQ(events.runDueEvents(1000U).events_executed, 5U);
    EXPECT_EQ(adc1.samples().size(), 1U);
    EXPECT_EQ(adc2.samples().size(), 1U);
    EXPECT_EQ(adc3.samples().size(), 1U);
    EXPECT_EQ(adc4.samples().size(), 1U);
    EXPECT_EQ(adc5.samples().size(), 1U);
    EXPECT_EQ(adc1.samples().front().time_ns, 353U)
        << "ADC1/2 use 42.5 MHz HCLK/4 without suppressing conversions";
    EXPECT_EQ(adc2.samples().front().time_ns, 353U);
    EXPECT_EQ(adc3.samples().front().time_ns, 89U);
    EXPECT_EQ(adc4.samples().front().time_ns, 89U);
    EXPECT_EQ(adc5.samples().front().time_ns, 89U);

    // A mid-conversion CCR change carries forward elapsed clock cycles rather
    // than leaving the already-armed event at its old timestamp.
    fil::sim::EventLoop switched_events;
    fil::stm32g4::AdcPeripheral switched_adc("ADC1", &switched_events, &trace);
    fil::stm32g4::AdcCommonPeripheral switched_common(
        "ADC12_COMMON", {&switched_adc}, &switched_events, &trace);
    switched_common.setSystemClockHz(170'000'000U);
    ASSERT_TRUE(switched_common.write(0x0aU, half, 3U, {}));
    ASSERT_TRUE(switched_adc.write(0x08U, word, 1U, {}));
    ASSERT_TRUE(switched_adc.write(0x08U, word, 1U | (1U << 2U), {}));
    ASSERT_EQ(switched_events.advanceBy(100U).events_executed, 0U);
    switched_adc.setInputClockHz(170'000'000U); // RCC input changes may occur while active.
    EXPECT_EQ(switched_events.nextScheduledTime(), 164U);
    ASSERT_EQ(switched_events.runDueEvents(164U).events_executed, 1U);
    EXPECT_EQ(switched_events.now(), 164U);

    // Lazy conversions have a logical deadline despite having no queued event.
    fil::sim::EventLoop lazy_events;
    fil::stm32g4::AdcPeripheral lazy_adc("ADC1", &lazy_events, &trace);
    fil::stm32g4::AdcCommonPeripheral lazy_common(
        "ADC12_COMMON", {&lazy_adc}, &lazy_events, &trace);
    lazy_common.setSystemClockHz(170'000'000U);
    ASSERT_TRUE(lazy_common.write(0x0aU, half, 1U, {}));
    lazy_adc.setSampleHistoryEnabled(false);
    ASSERT_TRUE(lazy_adc.write(0x0cU, word, 1U << 13U, {}));
    ASSERT_TRUE(lazy_adc.write(0x08U, word, 1U, {}));
    ASSERT_TRUE(lazy_adc.write(0x08U, word, 1U | (1U << 2U), {}));
    ASSERT_EQ(lazy_events.advanceBy(40U).events_executed, 0U);
    lazy_adc.setInputClockHz(85'000'000U); // Retain the lazy deadline on an input-clock change.
    lazy_adc.setSampleHistoryEnabled(true);
    EXPECT_EQ(lazy_events.nextScheduledTime(), 138U);
    ASSERT_EQ(lazy_events.runDueEvents(138U).events_executed, 1U);
    EXPECT_EQ(lazy_adc.samples().front().time_ns, 138U);

    // A decimation gap uses a specialized skipped-scan event. Clock changes
    // must retain its scan count rather than turn the landing into a sample.
    fil::sim::EventLoop gap_events;
    fil::stm32g4::AdcPeripheral gap_adc("ADC1", &gap_events, &trace);
    fil::stm32g4::AdcCommonPeripheral gap_common(
        "ADC12_COMMON", {&gap_adc}, &gap_events, &trace);
    gap_common.setSystemClockHz(170'000'000U);
    gap_adc.setSampleHistoryEnabled(true);
    gap_adc.setDecimation(2U);
    ASSERT_TRUE(gap_common.write(0x0aU, half, 1U, {}));
    ASSERT_TRUE(gap_adc.write(0x0cU, word, 1U << 13U, {}));
    ASSERT_TRUE(gap_adc.write(0x08U, word, 1U, {}));
    ASSERT_TRUE(gap_adc.write(0x08U, word, 1U | (1U << 2U), {}));
    ASSERT_EQ(gap_events.runDueEvents(89U).events_executed, 1U);
    ASSERT_TRUE(gap_events.advanceBy(11U).events_executed == 0U);
    gap_adc.setInputClockHz(85'000'000U);
    EXPECT_EQ(gap_events.nextScheduledTime(), 256U);
    ASSERT_EQ(gap_events.runDueEvents(256U).events_executed, 1U);
    EXPECT_EQ(gap_adc.samples().size(), 1U) << "the gap landing remains suppressed";
    ASSERT_EQ(gap_events.runDueEvents(433U).events_executed, 1U);
    ASSERT_EQ(gap_adc.samples().size(), 2U);
    EXPECT_EQ(gap_adc.samples().back().time_ns, 433U);
}

TEST(Stm32G4Test, AdcCommonClockTracksRccPllChanges) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true, 16'000'000U);
    ASSERT_TRUE(mcu);
    constexpr auto word = fil::mem::AccessSize::word;
    constexpr auto half = fil::mem::AccessSize::halfword;
    // PLL source HSE=16 MHz, M=1, N=85, R=8 => SYSCLK/HCLK=170 MHz.
    ASSERT_TRUE(mcu.value()->router().write(0x2100cU, word,
        3U | (85U << 8U) | (3U << 25U), {}));
    ASSERT_TRUE(mcu.value()->router().write(0x21000U, word,
        (1U << 16U) | (1U << 24U), {}));
    ASSERT_TRUE(mcu.value()->router().write(0x21008U, word, 3U, {}));
    ASSERT_EQ(mcu.value()->rcc().systemClockHz(), 170'000'000U);
    ASSERT_TRUE(mcu.value()->router().write(0x2104cU, word, 1U << 13U, {}));
    auto* adc = mcu.value()->adc("ADC1");
    ASSERT_NE(adc, nullptr);
    ASSERT_TRUE(mcu.value()->router().write(0x1000030aU, half, 3U, {}));
    ASSERT_TRUE(adc->write(0x08U, word, 1U, {}));
    ASSERT_TRUE(adc->write(0x08U, word, 1U | (1U << 2U), {}));
    ASSERT_EQ(events.runDueEvents(400U).events_executed, 1U);
    ASSERT_EQ(adc->samples().size(), 1U);
    EXPECT_EQ(adc->samples().front().time_ns, 353U)
        << "RCC SYSCLK updates the ADC12 group, whose CKMODE=11 divides HCLK by four";
}

TEST(Stm32G4Test, Adc5RoutesItsDmaRequestAndDedicatedInterrupt) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    trace.setEnabled(false);
    fil::cortexm::SystemControl system;
    auto mcu = createAdcMcu(events, trace, system);
    ASSERT_TRUE(mcu);
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRam(0x20000000U, 0x1000U, "ram"));
    ASSERT_TRUE(memory.mapMmio(0x40000000U, 0x20000000U,
                               mcu.value()->router(), "peripherals"));
    mcu.value()->attachMemory(memory);
    auto& router = mcu.value()->router();
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(router.write(0x20800U, word, 39U, {})); // DMAMUX request ADC5.
    ASSERT_TRUE(router.write(0x2000cU, word, 1U, {}));
    ASSERT_TRUE(router.write(0x20010U, word, 0x50000640U, {}));
    ASSERT_TRUE(router.write(0x20014U, word, 0x20000000U, {}));
    ASSERT_TRUE(router.write(0x20008U, word, 1U | (1U << 8U) | (1U << 10U), {}));
    ASSERT_TRUE(system.write(0xe104U, word, 1U << 30U, {})); // IRQ62 = ADC5.
    auto* adc5 = mcu.value()->adc("ADC5");
    ASSERT_NE(adc5, nullptr);
    adc5->setChannelValue(0U, 0x5a5U);
    ASSERT_TRUE(adc5->write(0x04U, word, 1U << 2U, {})); // EOC interrupt.
    ASSERT_TRUE(adc5->write(0x08U, word, 1U, {}));
    ASSERT_TRUE(adc5->write(0x08U, word, 1U | (1U << 2U), {}));
    ASSERT_EQ(events.runDueEvents(5000U).events_executed, 1U);
    const auto transferred = memory.read16(0x20000000U);
    ASSERT_TRUE(transferred);
    EXPECT_EQ(transferred.value(), 0x5a5U);
    const auto remaining = router.read(0x2000cU, word, {});
    ASSERT_TRUE(remaining);
    EXPECT_EQ(remaining.value(), 0U);
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 78U)
        << "ADC5 asserts external IRQ62 (exception number 78), not ADC3's IRQ47";
}

TEST(Stm32G4Test, CertifiesOnlyTrustedAdcEventsAndRechecksPendingHooks) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    trace.setEnabled(false);
    fil::cortexm::SystemControl system;
    auto mcu = createAdcMcu(events, trace, system);
    ASSERT_TRUE(mcu);
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRam(0x20000000U, 0x1000U, "ram"));
    ASSERT_TRUE(memory.mapMmio(0x40000000U, 0x20000000U,
                              mcu.value()->router(), "peripherals"));
    mcu.value()->attachMemory(memory);
    auto& peripherals = mcu.value()->router();
    ASSERT_TRUE(peripherals.write(0x20800U, fil::mem::AccessSize::word, 5U, {}));
    ASSERT_TRUE(peripherals.write(0x2000cU, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_TRUE(peripherals.write(0x20010U, fil::mem::AccessSize::word, 0x50000040U, {}));
    ASSERT_TRUE(peripherals.write(0x20014U, fil::mem::AccessSize::word, 0x20000000U, {}));
    ASSERT_TRUE(peripherals.write(0x20008U, fil::mem::AccessSize::word,
                                  1U | (1U << 8U) | (1U << 10U), {}));
    mcu.value()->setAdcDiagnosticsEnabled(false);
    auto* adc = mcu.value()->adc("ADC1");
    ASSERT_NE(adc, nullptr);
    adc->setConversionDelay(10U);
    adc->setChannelValue(0U, 321U);
    std::vector<fil::sim::EventObservation> observations;
    static_cast<void>(events.exchangeObservationBarrier(
        [&](fil::sim::SimTimeNs, fil::sim::EventOwner, fil::sim::EventObservation observation) {
            observations.push_back(observation);
        }));
    {
        auto owner = events.useOwner(2U);
        ASSERT_TRUE(adc->write(0x0cU, fil::mem::AccessSize::word, (1U << 13U) | 1U, {}));
        ASSERT_TRUE(adc->write(0x08U, fil::mem::AccessSize::word, 1U | (1U << 2U), {}));
    }
    ASSERT_EQ(events.runOwnedEvents(2U, 10U).events_executed, 1U);
    ASSERT_EQ(observations.size(), 1U);
    EXPECT_EQ(observations.back(), fil::sim::EventObservation::owner_local);
    const auto dma_value = memory.read16(0x20000000U);
    ASSERT_TRUE(dma_value);
    EXPECT_EQ(dma_value.value(), 321U) << "certified ADC DMA writes into RAM without a global callback";

    observations.clear();
    adc->setChannelProvider([](std::uint32_t, fil::sim::SimTimeNs) { return std::uint16_t{123U}; });
    ASSERT_EQ(events.runOwnedEvents(2U, 20U).events_executed, 1U);
    ASSERT_EQ(observations.size(), 1U);
    EXPECT_EQ(observations.back(), fil::sim::EventObservation::global)
        << "a public provider replacement invalidates certification before dispatch";
}

TEST(Stm32G4Test, PublicAdcHookChangesForceGlobalDispatch) {
    for (const std::uint32_t mutation : {0U, 1U, 2U}) {
        fil::sim::EventLoop events;
        fil::sim::TraceRecorder trace;
        trace.setEnabled(false);
        fil::cortexm::SystemControl system;
        auto mcu = createAdcMcu(events, trace, system);
        ASSERT_TRUE(mcu);
        fil::mem::MemoryBus memory;
        ASSERT_TRUE(memory.mapRam(0x20000000U, 0x1000U, "ram"));
        ASSERT_TRUE(memory.mapMmio(0x40000000U, 0x20000000U,
                                   mcu.value()->router(), "peripherals"));
        mcu.value()->attachMemory(memory);
        auto& peripherals = mcu.value()->router();
        ASSERT_TRUE(peripherals.write(0x20800U, fil::mem::AccessSize::word, 5U, {}));
        ASSERT_TRUE(peripherals.write(0x2000cU, fil::mem::AccessSize::word, 1U, {}));
        ASSERT_TRUE(peripherals.write(0x20010U, fil::mem::AccessSize::word, 0x50000040U, {}));
        ASSERT_TRUE(peripherals.write(0x20014U, fil::mem::AccessSize::word, 0x20000000U, {}));
        ASSERT_TRUE(peripherals.write(0x20008U, fil::mem::AccessSize::word,
                                      1U | (1U << 8U) | (1U << 10U), {}));
        mcu.value()->setAdcDiagnosticsEnabled(false);
        auto* adc = mcu.value()->adc("ADC1");
        ASSERT_NE(adc, nullptr);
        adc->setConversionDelay(10U);
        adc->setChannelValue(0U, 321U);
        {
            auto owner = events.useOwner(2U);
            ASSERT_TRUE(adc->write(0x0cU, fil::mem::AccessSize::word,
                                   (1U << 13U) | 1U, {}));
            ASSERT_TRUE(adc->write(0x08U, fil::mem::AccessSize::word,
                                   1U | (1U << 2U), {}));
        }
        if (mutation == 0U) {
            adc->setSampleCallback([](const fil::stm32g4::AdcSample&) {});
        } else if (mutation == 1U) {
            adc->setInterruptCallback([]() {});
        } else {
            adc->setInterruptLevelCallback([](std::uint32_t, bool) {});
        }
        std::vector<fil::sim::EventObservation> observations;
        static_cast<void>(events.exchangeObservationBarrier(
            [&](fil::sim::SimTimeNs, fil::sim::EventOwner,
                fil::sim::EventObservation observation) { observations.push_back(observation); }));
        ASSERT_EQ(events.runOwnedEvents(2U, 10U).events_executed, 1U);
        ASSERT_EQ(observations.size(), 1U);
        EXPECT_EQ(observations.front(), fil::sim::EventObservation::global)
            << "public ADC hook mutation " << mutation << " invalidates owner-local trust";
    }
}

TEST(Stm32G4Test, ObservationBarrierRechecksAfterProviderReplacement) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    trace.setEnabled(false);
    fil::cortexm::SystemControl system;
    auto mcu = createAdcMcu(events, trace, system);
    ASSERT_TRUE(mcu);
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRam(0x20000000U, 0x1000U, "ram"));
    ASSERT_TRUE(memory.mapMmio(0x40000000U, 0x20000000U,
                               mcu.value()->router(), "peripherals"));
    mcu.value()->attachMemory(memory);
    auto& peripherals = mcu.value()->router();
    ASSERT_TRUE(peripherals.write(0x20800U, fil::mem::AccessSize::word, 5U, {}));
    ASSERT_TRUE(peripherals.write(0x2000cU, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_TRUE(peripherals.write(0x20010U, fil::mem::AccessSize::word, 0x50000040U, {}));
    ASSERT_TRUE(peripherals.write(0x20014U, fil::mem::AccessSize::word, 0x20000000U, {}));
    ASSERT_TRUE(peripherals.write(0x20008U, fil::mem::AccessSize::word,
                                  1U | (1U << 8U) | (1U << 10U), {}));
    mcu.value()->setAdcDiagnosticsEnabled(false);
    auto* adc = mcu.value()->adc("ADC1");
    ASSERT_NE(adc, nullptr);
    adc->setConversionDelay(10U);
    std::vector<fil::sim::EventObservation> observations;
    static_cast<void>(events.exchangeObservationBarrier(
        [&](fil::sim::SimTimeNs, fil::sim::EventOwner,
            fil::sim::EventObservation observation) {
            observations.push_back(observation);
            if (observation == fil::sim::EventObservation::owner_local) {
                adc->setChannelProvider([](std::uint32_t, fil::sim::SimTimeNs) {
                    return std::uint16_t{123U};
                });
            }
        }));
    {
        auto owner = events.useOwner(2U);
        ASSERT_TRUE(adc->write(0x0cU, fil::mem::AccessSize::word,
                               (1U << 13U) | 1U, {}));
        ASSERT_TRUE(adc->write(0x08U, fil::mem::AccessSize::word,
                               1U | (1U << 2U), {}));
    }
    ASSERT_EQ(events.runOwnedEvents(2U, 10U).events_executed, 1U);
    EXPECT_EQ(observations, (std::vector<fil::sim::EventObservation>{
        fil::sim::EventObservation::owner_local, fil::sim::EventObservation::global
    })) << "provider replacement during observation adds a downgrade barrier before callback";
}

TEST(Stm32G4Test, ReplacingAttachedMemoryBusInvalidatesAdcDmaLocality) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    trace.setEnabled(false);
    fil::cortexm::SystemControl system;
    auto mcu = createAdcMcu(events, trace, system);
    ASSERT_TRUE(mcu);
    fil::mem::MemoryBus trusted_router;
    ASSERT_TRUE(trusted_router.mapRam(0x20000000U, 0x1000U, "trusted-ram"));
    ASSERT_TRUE(trusted_router.mapMmio(0x40000000U, 0x20000000U,
                                       mcu.value()->router(), "peripherals"));
    mcu.value()->attachMemory(trusted_router);
    auto& peripherals = mcu.value()->router();
    ASSERT_TRUE(peripherals.write(0x20800U, fil::mem::AccessSize::word, 5U, {}));
    ASSERT_TRUE(peripherals.write(0x2000cU, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_TRUE(peripherals.write(0x20010U, fil::mem::AccessSize::word, 0x50000040U, {}));
    ASSERT_TRUE(peripherals.write(0x20014U, fil::mem::AccessSize::word, 0x20000000U, {}));
    ASSERT_TRUE(peripherals.write(0x20008U, fil::mem::AccessSize::word,
                                  1U | (1U << 8U) | (1U << 10U), {}));
    mcu.value()->setAdcDiagnosticsEnabled(false);
    auto* adc = mcu.value()->adc("ADC1");
    ASSERT_NE(adc, nullptr);
    adc->setConversionDelay(10U);
    adc->setChannelValue(0U, 321U);
    {
        auto owner = events.useOwner(2U);
        ASSERT_TRUE(adc->write(0x0cU, fil::mem::AccessSize::word,
                               (1U << 13U) | 1U, {}));
        ASSERT_TRUE(adc->write(0x08U, fil::mem::AccessSize::word,
                               1U | (1U << 2U), {}));
    }
    fil::mem::MemoryBus source_fake_bus;
    fil::stm32g4::UnknownMmioDevice source_fake_peripherals("source-fake-peripherals", 0x40000000U);
    ASSERT_TRUE(source_fake_bus.mapRam(0x20000000U, 0x1000U, "source-fake-ram"));
    ASSERT_TRUE(source_fake_bus.mapMmio(0x40000000U, 0x20000000U,
                                        source_fake_peripherals, "source-fake-peripherals"));
    mcu.value()->attachMemory(source_fake_bus);
    std::vector<fil::sim::EventObservation> observations;
    static_cast<void>(events.exchangeObservationBarrier(
        [&](fil::sim::SimTimeNs, fil::sim::EventOwner,
            fil::sim::EventObservation observation) { observations.push_back(observation); }));
    ASSERT_EQ(events.runOwnedEvents(2U, 10U).events_executed, 1U);
    ASSERT_EQ(observations.size(), 1U);
    EXPECT_EQ(observations.front(), fil::sim::EventObservation::global)
        << "an equivalent source-fake mapping is not the trusted router MemoryBus";
}

TEST(Stm32G4Test, PartiallyConsumedAdcDmaRechecksNextDestinationRange) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    trace.setEnabled(false);
    fil::cortexm::SystemControl system;
    auto mcu = createAdcMcu(events, trace, system);
    ASSERT_TRUE(mcu);
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRam(0x20000000U, 0x1000U, "ram"));
    ASSERT_TRUE(memory.mapMmio(0x40000000U, 0x20000000U,
                              mcu.value()->router(), "peripherals"));
    mcu.value()->attachMemory(memory);
    auto& peripherals = mcu.value()->router();
    ASSERT_TRUE(peripherals.write(0x20800U, fil::mem::AccessSize::word, 5U, {}));
    ASSERT_TRUE(peripherals.write(0x2000cU, fil::mem::AccessSize::word, 2U, {}));
    ASSERT_TRUE(peripherals.write(0x20010U, fil::mem::AccessSize::word, 0x50000040U, {}));
    ASSERT_TRUE(peripherals.write(0x20014U, fil::mem::AccessSize::word, 0x20000ffeU, {}));
    ASSERT_TRUE(peripherals.write(0x20008U, fil::mem::AccessSize::word,
                                  1U | (1U << 7U) | (1U << 8U) | (1U << 10U), {}));
    mcu.value()->setAdcDiagnosticsEnabled(false);
    auto* adc = mcu.value()->adc("ADC1");
    ASSERT_NE(adc, nullptr);
    adc->setConversionDelay(10U);
    adc->setChannelValue(0U, 321U);
    std::vector<fil::sim::EventObservation> observations;
    static_cast<void>(events.exchangeObservationBarrier(
        [&](fil::sim::SimTimeNs, fil::sim::EventOwner, fil::sim::EventObservation observation) {
            observations.push_back(observation);
        }));
    {
        auto owner = events.useOwner(2U);
        ASSERT_TRUE(adc->write(0x0cU, fil::mem::AccessSize::word, (1U << 13U) | 1U, {}));
        ASSERT_TRUE(adc->write(0x08U, fil::mem::AccessSize::word, 1U | (1U << 2U), {}));
    }
    EXPECT_EQ(events.runOwnedEvents(2U, 10U).events_executed, 1U);
    ASSERT_EQ(observations.size(), 1U);
    EXPECT_EQ(observations.back(), fil::sim::EventObservation::owner_local);
    observations.clear();
    EXPECT_EQ(events.runOwnedEvents(2U, 20U).events_executed, 1U);
    ASSERT_EQ(observations.size(), 1U);
    EXPECT_EQ(observations.back(), fil::sim::EventObservation::global)
        << "the effective next halfword is outside RAM, despite a RAM starting address";
}

TEST(Stm32G4Test, FdcanInterruptRependsUntilSourceCleared) {
    using Fdcan = fil::stm32g4::FdcanPeripheral;
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true);
    ASSERT_TRUE(mcu);
    auto* fdcan = mcu.value()->fdcan("FDCAN1");
    ASSERT_NE(fdcan, nullptr);
    ASSERT_TRUE(system.write(0xe100U, fil::mem::AccessSize::word, 1U << 21U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::cccrOffset, fil::mem::AccessSize::word, 0U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::ieOffset, fil::mem::AccessSize::word,
                            Fdcan::interruptTransmissionComplete, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::ileOffset, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::txbtieOffset, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::txbarOffset, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_EQ(system.nextPending(0U, 0U, 0U), 37U);

    system.enter(37U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
    system.leave(37U); // ISR did not acknowledge the peripheral source.
    EXPECT_TRUE(system.hasEnabledPending());
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 37U);

    system.enter(37U);
    ASSERT_TRUE(fdcan->write(Fdcan::irOffset, fil::mem::AccessSize::word,
                            Fdcan::interruptTransmissionComplete, {}));
    system.leave(37U);
    EXPECT_FALSE(system.hasEnabledPending());
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, TimerInterruptRependsUntilSourceCleared) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true);
    ASSERT_TRUE(mcu);
    auto& timer = mcu.value()->router(); // TIM2 is at peripheral offset zero.
    ASSERT_TRUE(system.write(0xe100U, fil::mem::AccessSize::word, 1U << 28U, {}));
    ASSERT_TRUE(timer.write(0x0cU, fil::mem::AccessSize::word, 1U, {})); // DIER.UIE
    ASSERT_TRUE(timer.write(0x14U, fil::mem::AccessSize::word, 1U, {})); // EGR.UG
    ASSERT_EQ(system.nextPending(0U, 0U, 0U), 44U);
    system.enter(44U);
    system.leave(44U);
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 44U);
    system.enter(44U);
    ASSERT_TRUE(timer.write(0x10U, fil::mem::AccessSize::word, 0U, {})); // Clear SR.UIF.
    system.leave(44U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, UsartInterruptStopsRependingAfterReceiveConsumed) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system);
    ASSERT_TRUE(mcu);
    auto* usart = mcu.value()->usart("USART1");
    ASSERT_NE(usart, nullptr);
    ASSERT_TRUE(system.write(0xe104U, fil::mem::AccessSize::word, 1U << 5U, {}));
    ASSERT_TRUE(usart->write(0U, fil::mem::AccessSize::word, 1U | (1U << 5U), {}));
    usart->injectRx(0x42U);
    ASSERT_EQ(system.nextPending(0U, 0U, 0U), 53U);
    system.enter(53U);
    system.leave(53U);
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 53U);
    system.enter(53U);
    const auto received = usart->read(0x24U, fil::mem::AccessSize::word, {});
    ASSERT_TRUE(received);
    EXPECT_EQ(received.value(), 0x42U);
    system.leave(53U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, SharedTimerIrqRemainsAssertedUntilAllSourcesClear) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system);
    ASSERT_TRUE(mcu);
    auto& router = mcu.value()->router();
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(system.write(0xe100U, word, 1U << 25U, {}));
    // TIM1 update and TIM16 share IRQ25. Both have UIE/UIF set.
    for (const std::uint32_t base : {0x12c00U, 0x14400U}) {
        ASSERT_TRUE(router.write(base + 0x0cU, word, 1U, {}));
        ASSERT_TRUE(router.write(base + 0x14U, word, 1U, {}));
    }
    ASSERT_EQ(system.nextPending(0U, 0U, 0U), 41U);
    system.enter(41U);
    ASSERT_TRUE(router.write(0x12c10U, word, 0U, {})); // Clear only TIM1.
    system.leave(41U);
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 41U);
    system.enter(41U);
    ASSERT_TRUE(router.write(0x1440cU, word, 0U, {})); // Disable TIM16 interrupt.
    system.leave(41U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
    ASSERT_TRUE(router.write(0x1440cU, word, 1U, {})); // UIF still set: assert again.
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 41U);
    system.enter(41U);
    mcu.value()->reset();
    system.leave(41U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, FdcanInterruptLineRoutingAndDisableUpdateNvicLevels) {
    using Fdcan = fil::stm32g4::FdcanPeripheral;
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system);
    ASSERT_TRUE(mcu);
    auto* fdcan = mcu.value()->fdcan("FDCAN1");
    ASSERT_NE(fdcan, nullptr);
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(system.write(0xe100U, word, (1U << 21U) | (1U << 22U), {}));
    ASSERT_TRUE(fdcan->write(Fdcan::cccrOffset, word, 0U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::ieOffset, word, Fdcan::interruptTransmissionComplete, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::ileOffset, word, 3U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::txbtieOffset, word, 1U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::txbarOffset, word, 1U, {}));
    ASSERT_EQ(system.nextPending(0U, 0U, 0U), 37U);
    system.enter(37U);
    ASSERT_TRUE(fdcan->write(Fdcan::ilsOffset, word, 1U << 2U, {}));
    system.leave(37U);
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 38U);
    system.enter(38U);
    ASSERT_TRUE(fdcan->write(Fdcan::ileOffset, word, 0U, {}));
    system.leave(38U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
    ASSERT_TRUE(fdcan->write(Fdcan::ileOffset, word, 2U, {}));
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 38U);
    system.enter(38U);
    fdcan->reset();
    system.leave(38U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, ResetDoesNotRelatchStaleSharedInterruptSources) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system);
    ASSERT_TRUE(mcu);
    constexpr auto word = fil::mem::AccessSize::word;
    for (const std::uint32_t base : {0x12c00U, 0x14400U}) {
        ASSERT_TRUE(mcu.value()->router().write(base + 0x0cU, word, 1U, {}));
        ASSERT_TRUE(mcu.value()->router().write(base + 0x14U, word, 1U, {}));
    }
    // Board::reset resets SystemControl before the peripherals.
    system.reset(0x08000000U);
    mcu.value()->reset();
    ASSERT_TRUE(system.write(0xe100U, word, 1U << 25U, {}));
    EXPECT_FALSE(system.hasEnabledPending());
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, DestructionDeassertsPeripheralInterruptLines) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system);
    ASSERT_TRUE(mcu);
    ASSERT_TRUE(system.write(0xe100U, fil::mem::AccessSize::word, 1U << 28U, {}));
    ASSERT_TRUE(mcu.value()->router().write(0x0cU, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_TRUE(mcu.value()->router().write(0x14U, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_EQ(system.nextPending(0U, 0U, 0U), 44U);
    system.enter(44U);
    mcu.value().reset();
    system.leave(44U);
    EXPECT_FALSE(system.hasEnabledPending());
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, RoutesIntegratedPeripherals) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true, 16'000'000U);
    EXPECT_TRUE(mcu.hasValue()) << "constructs integrated STM32G4 peripheral map";
    if (!mcu) return;

    EXPECT_TRUE(mcu.value()->adc("ADC1")->sampleHistoryEnabled())
        << "integrated ADC diagnostics default to enabled";
    mcu.value()->setAdcDiagnosticsEnabled(false);
    EXPECT_TRUE(!mcu.value()->adc("ADC1")->sampleHistoryEnabled() &&
                !mcu.value()->adc("ADC4")->sampleHistoryEnabled())
        << "integrated ADC diagnostics toggle applies to every ADC";

    auto rcc_write = mcu.value()->router().write(0x00021000U, fil::mem::AccessSize::word, 1U << 8U, {});
    auto rcc_read = mcu.value()->router().read(0x00021000U, fil::mem::AccessSize::word, {});
    EXPECT_TRUE(rcc_write && rcc_read && (rcc_read.value() & (1U << 10U)) != 0)
        << "routes RCC and asserts HSI ready";
    auto hse_enable = mcu.value()->router().write(
        0x00021000U, fil::mem::AccessSize::word, 1U << 16U, {}
    );
    auto hse_select = mcu.value()->router().write(
        0x00021008U, fil::mem::AccessSize::word, 2U, {}
    );
    EXPECT_TRUE(hse_enable && hse_select && mcu.value()->rcc().systemClockHz() == 16'000'000U)
        << "uses the configured HSE frequency";

    fil::config::BoardConfig board;
    board.gpio.push_back({"PB7", "output", false, true});
    fil::test::TemporaryDirectory directory{"fil-stm32g4-uart"};
    const auto uart_log = directory.root() / "uart.bin";
    board.usart.push_back({"USART1", uart_log, {0x42U}});
    board.spi.push_back({"SPI1", "echo"});
    EXPECT_TRUE(mcu.value()->configure(board).hasValue()) << "applies board device configuration";
    auto gpio_write = mcu.value()->router().write(0x08000418U, fil::mem::AccessSize::word, 1U << 7U, {});
    EXPECT_TRUE(gpio_write && mcu.value()->gpio("GPIOB")->output(7))
        << "routes GPIO BSRR side effect";
    auto uart_read = mcu.value()->router().read(0x00013824U, fil::mem::AccessSize::word, {});
    EXPECT_TRUE(uart_read && (uart_read.value() & 0xffU) == 0x42U)
        << "routes scripted USART RX byte";
    auto uart_write = mcu.value()->router().write(0x00013828U, fil::mem::AccessSize::word, 0x5aU, {});
    std::ifstream uart_input(uart_log, std::ios::binary);
    const std::int32_t logged_byte = uart_input.get();
    EXPECT_TRUE(uart_write && logged_byte == 0x5a) << "writes configured USART TX byte log";
    auto fdcan_read = mcu.value()->router().read(
        fil::stm32g4::FdcanPeripheral::baseAddresses[0] - 0x40000000U +
            fil::stm32g4::FdcanPeripheral::cccrOffset,
        fil::mem::AccessSize::word,
        {}
    );
    EXPECT_TRUE(fdcan_read && mcu.value()->fdcan("FDCAN1") != nullptr)
        << "routes and exposes FDCAN controllers";
    auto ram_write = mcu.value()->router().write(
        fil::stm32g4::FdcanMessageRam::baseAddress - 0x40000000U,
        fil::mem::AccessSize::word,
        0x12345678U,
        {}
    );
    auto ram_read = mcu.value()->router().read(
        fil::stm32g4::FdcanMessageRam::baseAddress - 0x40000000U,
        fil::mem::AccessSize::word,
        {}
    );
    EXPECT_TRUE(ram_write && ram_read && ram_read.value() == 0x12345678U)
        << "routes shared FDCAN message RAM";
}

TEST(Stm32G4Test, ExtiRoutesGpioEdgeToNvic) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system);
    ASSERT_TRUE(mcu);
    constexpr auto word = fil::mem::AccessSize::word;
    // Enable EXTI0 NVIC line and route EXTI0 to PB via SYSCFG EXTICR1.
    ASSERT_TRUE(system.write(0xe100U, word, 1U << 6U, {}));
    ASSERT_TRUE(mcu.value()->router().write(0x00010008U, word, 0x1U, {}));
    ASSERT_TRUE(mcu.value()->router().write(0x00010400U, word, 1U << 0U, {})); // IMR1.IM0
    ASSERT_TRUE(mcu.value()->router().write(0x00010408U, word, 1U << 0U, {})); // RTSR1.RT0
    auto* gpiob = mcu.value()->gpio("GPIOB");
    ASSERT_NE(gpiob, nullptr);
    gpiob->setInput(0, false);
    EXPECT_FALSE(system.hasEnabledPending());
    gpiob->setInput(0, true); // Rising edge on PB0 -> EXTI0.
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 22U); // 16+6
    system.enter(22U);
    // Clear PR1.PIF0 like dashboard EXTI0_IRQHandler does.
    ASSERT_TRUE(mcu.value()->router().write(0x00010414U, word, 1U << 0U, {}));
    system.leave(22U);
    EXPECT_FALSE(system.hasEnabledPending());
}

TEST(Stm32G4Test, ExtiIgnoresUnroutedPortAndFallingWithoutTrigger) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system);
    ASSERT_TRUE(mcu);
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(system.write(0xe100U, word, 1U << 6U, {}));
    // Default SYSCFG routes EXTI0 to PA; PB0 edge must not pend.
    ASSERT_TRUE(mcu.value()->router().write(0x00010400U, word, 1U << 0U, {}));
    ASSERT_TRUE(mcu.value()->router().write(0x00010408U, word, 1U << 0U, {}));
    auto* gpiob = mcu.value()->gpio("GPIOB");
    ASSERT_NE(gpiob, nullptr);
    gpiob->setInput(0, false);
    gpiob->setInput(0, true);
    EXPECT_FALSE(system.hasEnabledPending());
    // Route to PB but only rising armed; falling edge must not pend.
    ASSERT_TRUE(mcu.value()->router().write(0x00010008U, word, 0x1U, {}));
    gpiob->setInput(0, true);
    gpiob->setInput(0, false); // Falling with only RTSR set.
    EXPECT_FALSE(system.hasEnabledPending());
    // Arm falling too; now falling pends.
    ASSERT_TRUE(mcu.value()->router().write(0x0001040cU, word, 1U << 0U, {}));
    gpiob->setInput(0, true);
    gpiob->setInput(0, false);
    EXPECT_TRUE(system.hasEnabledPending());
}

TEST(Stm32G4Test, ExtiSharedLinesOrIntoSingleIrq) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system);
    ASSERT_TRUE(mcu);
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(system.write(0xe100U, word, 1U << 23U, {})); // EXTI9_5.
    // Route EXTI5 to PA (default), EXTI6 to PC via EXTICR2.
    ASSERT_TRUE(mcu.value()->router().write(0x0001000cU, word, 0x2U << 8U, {}));
    ASSERT_TRUE(mcu.value()->router().write(0x00010400U, word, (1U << 5U) | (1U << 6U), {}));
    ASSERT_TRUE(mcu.value()->router().write(0x00010408U, word, (1U << 5U) | (1U << 6U), {}));
    auto* gpioa = mcu.value()->gpio("GPIOA");
    auto* gpioc = mcu.value()->gpio("GPIOC");
    ASSERT_NE(gpioa, nullptr);
    ASSERT_NE(gpioc, nullptr);
    gpioa->setInput(5, false);
    gpioc->setInput(6, false);
    gpioa->setInput(5, true);
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 39U); // 16+23
    gpioc->setInput(6, true);
    system.enter(39U);
    ASSERT_TRUE(mcu.value()->router().write(0x00010414U, word, 1U << 5U, {}));
    system.leave(39U);
    // EXTI6 still pending, shared IRQ remains asserted.
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 39U);
    system.enter(39U);
    ASSERT_TRUE(mcu.value()->router().write(0x00010414U, word, 1U << 6U, {}));
    system.leave(39U);
    EXPECT_FALSE(system.hasEnabledPending());
}

} // namespace
