#include "fil/cortexm/system_control.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include "../fixture_support.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace {

TEST(Stm32G4Test, CertifiesOnlyTrustedAdcEventsAndRechecksPendingHooks) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    trace.setEnabled(false);
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true);
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
    adc->setChannelProvider([](unsigned int, fil::sim::SimTimeNs) { return std::uint16_t{123U}; });
    ASSERT_EQ(events.runOwnedEvents(2U, 20U).events_executed, 1U);
    ASSERT_EQ(observations.size(), 1U);
    EXPECT_EQ(observations.back(), fil::sim::EventObservation::global)
        << "a public provider replacement invalidates certification before dispatch";
}

TEST(Stm32G4Test, PublicAdcHookChangesForceGlobalDispatch) {
    for (const unsigned int mutation : {0U, 1U, 2U}) {
        fil::sim::EventLoop events;
        fil::sim::TraceRecorder trace;
        trace.setEnabled(false);
        fil::cortexm::SystemControl system;
        auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true);
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
            adc->setInterruptLevelCallback([](unsigned int, bool) {});
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
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true);
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
                adc->setChannelProvider([](unsigned int, fil::sim::SimTimeNs) {
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
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true);
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
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true);
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
    const int logged_byte = uart_input.get();
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
