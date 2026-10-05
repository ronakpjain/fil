#include "fil/cortexm/system_control.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <gtest/gtest.h>

namespace {

// Encodings are from RM0440 sections 7.4 and 21.8.2, not from the model.
class AdcClockTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto result = fil::stm32g4::Stm32G4::create(events, trace, system, true, 16'000'000U);
        ASSERT_TRUE(result);
        mcu = std::move(result).value();
        writeRcc(0x4cU, (1U << 13U) | (1U << 14U)); // AHB2ENR ADC12/345EN
    }
    void writeRcc(const std::uint32_t offset, const std::uint32_t value) {
        ASSERT_TRUE(mcu->router().write(0x21000U + offset, word, value, {}));
    }
    void writeCommon(const std::uint32_t value, const bool group345 = false) {
        ASSERT_TRUE(mcu->router().write(group345 ? 0x10000708U : 0x10000308U,
                                       word, value, {}));
    }
    void start(fil::stm32g4::AdcPeripheral* adc) {
        ASSERT_NE(adc, nullptr);
        ASSERT_TRUE(adc->write(0x08U, word, 1U, {}));
        ASSERT_TRUE(adc->write(0x08U, word, 5U, {}));
    }
    static std::uint64_t delay(const std::uint64_t hz) {
        return (15'000'000'000ULL + hz - 1U) / hz;
    }
    void expectConversion(fil::stm32g4::AdcPeripheral* adc, const std::uint64_t hz) {
        const auto before = adc->samples().size();
        const auto deadline = events.now() + delay(hz);
        start(adc);
        ASSERT_EQ(events.runDueEvents(deadline - 1U).events_executed, 0U);
        EXPECT_EQ(adc->samples().size(), before);
        ASSERT_EQ(events.runDueEvents(deadline).events_executed, 1U);
        ASSERT_EQ(adc->samples().size(), before + 1U);
        EXPECT_EQ(adc->samples().back().time_ns, deadline);
        ASSERT_TRUE(adc->write(0x08U, word, 1U << 1U, {})); // disable before CCR changes
    }
    static constexpr auto word = fil::mem::AccessSize::word;
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    std::unique_ptr<fil::stm32g4::Stm32G4> mcu;
};

TEST_F(AdcClockTest, AsynchronousSysclkHonorsEveryPrescalerAndIgnoresHpre) {
    constexpr std::array<std::uint32_t, 12> divisors{1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256};
    writeRcc(0x88U, 2U << 28U); // ADC12SEL = SYSCLK
    writeRcc(0x08U, 1U | (9U << 4U)); // HSI SYSCLK, HCLK = SYSCLK/4
    auto* adc = mcu->adc("ADC1");
    for (std::uint32_t presc = 0; presc < divisors.size(); ++presc) {
        SCOPED_TRACE(presc);
        writeCommon(presc << 18U);
        expectConversion(adc, 16'000'000U / divisors[presc]);
    }
}

TEST_F(AdcClockTest, SynchronousModesHonorEveryAhbDividerAndIgnoreAsyncPrescaler) {
    constexpr std::array<std::uint32_t, 16> divisors{
        1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 8, 16, 64, 128, 256, 512};
    auto* adc = mcu->adc("ADC1");
    for (std::uint32_t hpre = 0; hpre < divisors.size(); ++hpre) {
        writeRcc(0x08U, 1U | (hpre << 4U));
        for (std::uint32_t mode = hpre < 8U ? 1U : 2U; mode <= 3U; ++mode) {
            SCOPED_TRACE(hpre);
            SCOPED_TRACE(mode);
            writeCommon((mode << 16U) | (11U << 18U));
            expectConversion(adc, 16'000'000U / divisors[hpre] / (1U << (mode - 1U)));
        }
    }
}

TEST_F(AdcClockTest, IndependentGroupsSelectPllpOrSysclkWithoutChangingSysclk) {
    // HSI / M=2 * N=24 = 192 MHz VCO, P=8 => 24 MHz. SYSCLK stays HSI.
    writeRcc(0x0cU, 2U | (1U << 4U) | (24U << 8U) | (1U << 16U) | (8U << 27U));
    writeRcc(0x00U, (1U << 8U) | (1U << 24U));
    writeRcc(0x88U, (1U << 28U) | (2U << 30U));
    EXPECT_EQ(mcu->rcc().systemClockHz(), 16'000'000U);
    writeCommon(0U);
    writeCommon(0U, true);
    auto* adc1 = mcu->adc("ADC1");
    auto* adc3 = mcu->adc("ADC3");
    start(adc1);
    start(adc3);
    ASSERT_EQ(events.runDueEvents(625U).events_executed, 1U);
    ASSERT_EQ(adc1->samples().size(), 1U);
    EXPECT_TRUE(adc3->samples().empty());
    ASSERT_EQ(events.runDueEvents(938U).events_executed, 1U);
    ASSERT_EQ(adc3->samples().size(), 1U);
    EXPECT_EQ(adc3->samples().front().time_ns, 938U);
}

TEST_F(AdcClockTest, PllpSupportsLegacyAndProgrammableDividers) {
    // HSE=16MHz /2 * N=34 => 272MHz VCO. All PLLP divisor encodings.
    writeRcc(0x00U, (1U << 8U) | (1U << 16U) | (1U << 24U));
    writeRcc(0x88U, 1U << 28U);
    auto* adc = mcu->adc("ADC1");
    for (std::uint32_t p = 2U; p <= 31U; ++p) {
        SCOPED_TRACE(p);
        writeRcc(0x0cU, 3U | (1U << 4U) | (34U << 8U) | (1U << 16U) | (p << 27U));
        expectConversion(adc, 272'000'000U / p);
    }
    for (const bool legacy17 : {false, true}) {
        writeRcc(0x0cU, 3U | (1U << 4U) | (34U << 8U) | (1U << 16U)
                          | (legacy17 ? 1U << 17U : 0U));
        expectConversion(adc, 272'000'000U / (legacy17 ? 17U : 7U));
    }
}

TEST_F(AdcClockTest, MissingAndReservedAsyncSourcesProduceNoConversions) {
    auto* adc = mcu->adc("ADC1");
    for (const std::uint32_t selector : {0U, 1U, 3U}) {
        SCOPED_TRACE(selector);
        writeRcc(0x88U, selector << 28U); // PLL is disabled for selector 1
        start(adc);
        ASSERT_EQ(events.advanceBy(1'000'000U).events_executed, 0U);
        EXPECT_TRUE(adc->samples().empty());
        ASSERT_TRUE(adc->write(0x08U, word, 1U << 1U, {}));
    }
    writeRcc(0x88U, 2U << 28U);
    for (std::uint32_t presc = 12U; presc < 16U; ++presc) {
        writeCommon(presc << 18U);
        start(adc);
        ASSERT_EQ(events.advanceBy(1'000'000U).events_executed, 0U);
        EXPECT_TRUE(adc->samples().empty());
        ASSERT_TRUE(adc->write(0x08U, word, 1U << 1U, {}));
    }
}

TEST_F(AdcClockTest, DisabledPllpOutputAndMissingPllSourceDoNotFallBack) {
    auto* adc = mcu->adc("ADC1");
    writeRcc(0x88U, 1U << 28U);
    writeRcc(0x00U, (1U << 8U) | (1U << 24U));
    for (const auto config : {
             2U | (24U << 8U) | (8U << 27U), // output disabled
             (24U << 8U) | (1U << 16U) | (8U << 27U), // no source
             1U | (24U << 8U) | (1U << 16U) | (8U << 27U), // reserved source (no MSI on G4)
             3U | (24U << 8U) | (1U << 16U) | (8U << 27U), // HSE disabled
             2U | (24U << 8U) | (1U << 16U) | (1U << 27U) // reserved P divider
         }) {
        writeRcc(0x0cU, config);
        start(adc);
        ASSERT_EQ(events.advanceBy(1'000'000U).events_executed, 0U);
        EXPECT_TRUE(adc->samples().empty());
        ASSERT_TRUE(adc->write(0x08U, word, 1U << 1U, {}));
    }
}

TEST_F(AdcClockTest, ClockMuxStopAndResumePreservesRemainingConversionProgress) {
    auto* adc = mcu->adc("ADC1");
    writeRcc(0x88U, 2U << 28U);
    start(adc); // 938 ns deadline at 16 MHz
    ASSERT_EQ(events.advanceBy(400U).events_executed, 0U);
    writeRcc(0x88U, 0U);
    ASSERT_EQ(events.advanceBy(10'000U).events_executed, 0U);
    EXPECT_TRUE(adc->samples().empty());
    writeRcc(0x88U, 2U << 28U);
    ASSERT_EQ(events.runDueEvents(10'937U).events_executed, 0U);
    ASSERT_EQ(events.runDueEvents(10'938U).events_executed, 1U);
    ASSERT_EQ(adc->samples().size(), 1U);
    EXPECT_EQ(adc->samples().front().time_ns, 10'938U);
}

TEST_F(AdcClockTest, RccAhbDividerChangeReschedulesSynchronousConversion) {
    auto* adc = mcu->adc("ADC1");
    writeCommon(2U << 16U); // HCLK/2: 1875 ns
    start(adc);
    ASSERT_EQ(events.advanceBy(375U).events_executed, 0U);
    writeRcc(0x08U, 1U | (8U << 4U)); // HCLK /2, ADC remaining time doubles
    ASSERT_EQ(events.runDueEvents(3374U).events_executed, 0U);
    ASSERT_EQ(events.runDueEvents(3375U).events_executed, 1U);
    ASSERT_EQ(adc->samples().size(), 1U);
    EXPECT_EQ(adc->samples().front().time_ns, 3375U);
}

TEST_F(AdcClockTest, ConversionStartedWithoutAClockWaitsUntilSourceIsSelected) {
    auto* adc = mcu->adc("ADC1");
    start(adc);
    ASSERT_EQ(events.advanceBy(10'000U).events_executed, 0U);
    EXPECT_TRUE(adc->samples().empty());
    writeRcc(0x88U, 2U << 28U);
    ASSERT_EQ(events.runDueEvents(10'937U).events_executed, 0U);
    ASSERT_EQ(events.runDueEvents(10'938U).events_executed, 1U);
    EXPECT_EQ(adc->samples().front().time_ns, 10'938U);
}

TEST_F(AdcClockTest, SynchronousBusClockGatePausesAndResumesConversion) {
    auto* adc = mcu->adc("ADC1");
    writeCommon(3U << 16U); // HCLK/4: 3750 ns
    start(adc);
    ASSERT_EQ(events.advanceBy(750U).events_executed, 0U);
    writeRcc(0x4cU, 1U << 14U); // gate ADC12 only
    ASSERT_EQ(events.advanceBy(10'000U).events_executed, 0U);
    EXPECT_TRUE(adc->samples().empty());
    writeRcc(0x4cU, (1U << 13U) | (1U << 14U));
    ASSERT_EQ(events.runDueEvents(13'749U).events_executed, 0U);
    ASSERT_EQ(events.runDueEvents(13'750U).events_executed, 1U);
    EXPECT_EQ(adc->samples().front().time_ns, 13'750U);
}

TEST_F(AdcClockTest, ClockStopPreservesLazyContinuousDeadline) {
    auto* adc = mcu->adc("ADC1");
    trace.setEnabled(false);
    adc->setSampleHistoryEnabled(false);
    ASSERT_TRUE(adc->write(0x0cU, word, 1U << 13U, {}));
    writeRcc(0x88U, 2U << 28U);
    start(adc);
    ASSERT_EQ(events.advanceBy(400U).events_executed, 0U);
    writeRcc(0x88U, 0U);
    ASSERT_EQ(events.advanceBy(10'000U).events_executed, 0U);
    ASSERT_TRUE(adc->read(0x00U, word, {}));
    writeRcc(0x88U, 2U << 28U);
    adc->setSampleHistoryEnabled(true);
    ASSERT_EQ(events.runDueEvents(10'937U).events_executed, 0U);
    ASSERT_EQ(events.runDueEvents(10'938U).events_executed, 1U);
    ASSERT_EQ(adc->samples().size(), 1U);
    EXPECT_EQ(adc->samples().front().time_ns, 10'938U);
}

TEST_F(AdcClockTest, ClockStopPreservesDecimationGapLanding) {
    auto* adc = mcu->adc("ADC1");
    adc->setDecimation(2U);
    ASSERT_TRUE(adc->write(0x0cU, word, 1U << 13U, {}));
    writeRcc(0x88U, 2U << 28U);
    start(adc);
    ASSERT_EQ(events.runDueEvents(938U).events_executed, 1U);
    ASSERT_EQ(events.advanceBy(62U).events_executed, 0U);
    writeRcc(0x88U, 0U);
    ASSERT_EQ(events.advanceBy(10'000U).events_executed, 0U);
    writeRcc(0x88U, 2U << 28U);
    ASSERT_EQ(events.runDueEvents(11'876U).events_executed, 1U);
    EXPECT_EQ(adc->samples().size(), 1U); // skipped scan remains skipped
    ASSERT_EQ(events.runDueEvents(12'814U).events_executed, 1U);
    ASSERT_EQ(adc->samples().size(), 2U);
    EXPECT_EQ(adc->samples().back().time_ns, 12'814U);
}

TEST_F(AdcClockTest, DisablingSuspendedConversionDoesNotResurrectItOnResume) {
    auto* adc = mcu->adc("ADC1");
    writeRcc(0x88U, 2U << 28U);
    start(adc);
    ASSERT_EQ(events.advanceBy(400U).events_executed, 0U);
    writeRcc(0x88U, 0U);
    ASSERT_TRUE(adc->write(0x08U, word, 1U << 1U, {}));
    writeRcc(0x88U, 2U << 28U);
    ASSERT_EQ(events.advanceBy(10'000U).events_executed, 0U);
    EXPECT_TRUE(adc->samples().empty());
    expectConversion(adc, 16'000'000U);
}

TEST_F(AdcClockTest, FullMachineResetClearsLiveAdcClockInputs) {
    writeRcc(0x88U, 2U << 28U);
    expectConversion(mcu->adc("ADC1"), 16'000'000U);
    mcu->reset();
    start(mcu->adc("ADC1"));
    ASSERT_EQ(events.advanceBy(10'000U).events_executed, 0U);
    EXPECT_TRUE(mcu->adc("ADC1")->samples().empty());
    writeRcc(0x4cU, 1U << 13U);
    writeRcc(0x88U, 2U << 28U);
    const auto deadline = events.now() + 938U;
    ASSERT_EQ(events.runDueEvents(deadline).events_executed, 1U);
    EXPECT_EQ(mcu->adc("ADC1")->samples().front().time_ns, deadline);
}

TEST_F(AdcClockTest, CcrClockFieldsCannotChangeWhileAnyGroupMemberIsEnabled) {
    writeCommon(3U << 16U);
    auto* adc2 = mcu->adc("ADC2");
    ASSERT_TRUE(adc2->write(0x08U, word, 1U, {}));
    writeCommon((1U << 16U) | (5U << 18U) | (1U << 22U));
    const auto ccr = mcu->router().read(0x10000308U, word, {});
    ASSERT_TRUE(ccr);
    EXPECT_EQ(ccr.value() & ((3U << 16U) | (15U << 18U)), 3U << 16U);
    EXPECT_NE(ccr.value() & (1U << 22U), 0U); // unrelated VREFEN remains writable
    ASSERT_TRUE(adc2->write(0x08U, word, 1U << 1U, {}));
    writeCommon(2U << 16U);
    expectConversion(mcu->adc("ADC1"), 8'000'000U);
}

TEST_F(AdcClockTest, PllpFrequencyIsIndependentOfPllrDivider) {
    // Nonintegral VCO source/M deliberately exercises the order of operations.
    writeRcc(0x00U, (1U << 8U) | (1U << 16U) | (1U << 24U));
    writeRcc(0x88U, 1U << 28U);
    for (std::uint32_t r = 0U; r < 4U; ++r) {
        writeRcc(0x0cU, 3U | (2U << 4U) | (31U << 8U) | (1U << 16U)
                          | (7U << 27U) | (r << 25U));
        expectConversion(mcu->adc("ADC1"), (16'000'000ULL * 31U) / (3U * 7U));
    }
}

TEST_F(AdcClockTest, RccGroupResetCancelsOnlyItsOwnConversionsAndCommonRegisters) {
    writeRcc(0x88U, (2U << 28U) | (2U << 30U));
    auto* adc1 = mcu->adc("ADC1");
    auto* adc3 = mcu->adc("ADC3");
    start(adc1);
    start(adc3);
    ASSERT_EQ(events.advanceBy(400U).events_executed, 0U);
    writeRcc(0x2cU, 1U << 13U); // AHB2RSTR ADC12RST
    ASSERT_EQ(events.runDueEvents(938U).events_executed, 1U);
    EXPECT_TRUE(adc1->samples().empty());
    ASSERT_EQ(adc3->samples().size(), 1U);
    const auto control = adc1->read(0x08U, word, {});
    ASSERT_TRUE(control);
    EXPECT_EQ(control.value() & 5U, 0U);
    writeRcc(0x2cU, 0U);
    expectConversion(adc1, 16'000'000U);
    EXPECT_EQ(adc3->samples().size(), 1U);
}

TEST_F(AdcClockTest, SynchronousDivideByOneRejectsDividedAhbClock) {
    writeRcc(0x08U, 1U | (8U << 4U));
    writeCommon(1U << 16U);
    auto* adc = mcu->adc("ADC1");
    start(adc);
    ASSERT_EQ(events.advanceBy(10'000U).events_executed, 0U);
    EXPECT_TRUE(adc->samples().empty());
    writeRcc(0x08U, 1U); // permitted HPRE restores the pending clock budget
    ASSERT_EQ(events.runDueEvents(10'938U).events_executed, 1U);
    ASSERT_EQ(adc->samples().size(), 1U);
    EXPECT_EQ(adc->samples().front().time_ns, 10'938U);
}

TEST_F(AdcClockTest, StoppingClockInsideSampleCallbackSuspendsTheNextRank) {
    writeRcc(0x88U, 2U << 28U);
    auto* adc = mcu->adc("ADC1");
    ASSERT_TRUE(adc->write(0x30U, word, 1U | (1U << 6U) | (2U << 12U), {}));
    adc->setSampleCallback([&](const fil::stm32g4::AdcSample&) {
        if (adc->samples().size() == 1U) writeRcc(0x88U, 0U);
    });
    start(adc);
    ASSERT_EQ(events.runDueEvents(938U).events_executed, 1U);
    ASSERT_EQ(events.advanceBy(10'000U).events_executed, 0U);
    ASSERT_EQ(adc->samples().size(), 1U);
    writeRcc(0x88U, 2U << 28U);
    ASSERT_EQ(events.runDueEvents(11'876U).events_executed, 1U);
    ASSERT_EQ(adc->samples().size(), 2U);
    EXPECT_EQ(adc->samples().back().time_ns, 11'876U);
    EXPECT_EQ(adc->samples().back().channel, 2U);
}

TEST_F(AdcClockTest, BothGroupsHonorBusGatesInEverySynchronousMode) {
    for (const bool group345 : {false, true}) {
        auto* adc = mcu->adc(group345 ? "ADC3" : "ADC1");
        for (std::uint32_t mode = 1U; mode <= 3U; ++mode) {
            SCOPED_TRACE(group345);
            SCOPED_TRACE(mode);
            writeCommon(mode << 16U, group345);
            writeRcc(0x4cU, group345 ? 1U << 13U : 1U << 14U);
            const auto before = adc->samples().size();
            start(adc);
            ASSERT_EQ(events.advanceBy(10'000U).events_executed, 0U);
            EXPECT_EQ(adc->samples().size(), before);
            writeRcc(0x4cU, (1U << 13U) | (1U << 14U));
            const auto deadline = events.now() + delay(16'000'000U / (1U << (mode - 1U)));
            ASSERT_EQ(events.runDueEvents(deadline).events_executed, 1U);
            ASSERT_EQ(adc->samples().size(), before + 1U);
            EXPECT_EQ(adc->samples().back().time_ns, deadline);
            ASSERT_TRUE(adc->write(0x08U, word, 1U << 1U, {}));
        }
    }
}

TEST_F(AdcClockTest, AsynchronousKernelIsIndependentOfTheBusClockGate) {
    writeRcc(0x88U, (2U << 28U) | (2U << 30U));
    auto* adc1 = mcu->adc("ADC1");
    auto* adc3 = mcu->adc("ADC3");
    start(adc1);
    start(adc3);
    ASSERT_EQ(events.advanceBy(400U).events_executed, 0U);
    writeRcc(0x4cU, 0U); // Gate interface HCLK, not the asynchronous kernel.
    ASSERT_EQ(events.runDueEvents(938U).events_executed, 2U);
    ASSERT_EQ(adc1->samples().size(), 1U);
    ASSERT_EQ(adc3->samples().size(), 1U);
    EXPECT_EQ(adc1->samples().front().time_ns, 938U);
    EXPECT_EQ(adc3->samples().front().time_ns, 938U);
}

TEST_F(AdcClockTest, StoppingClockOnFinalSingleShotSampleDoesNotRestartOnResume) {
    writeRcc(0x88U, 2U << 28U);
    auto* adc = mcu->adc("ADC1");
    adc->setSampleCallback([&](const fil::stm32g4::AdcSample&) { writeRcc(0x88U, 0U); });
    start(adc);
    ASSERT_EQ(events.runDueEvents(938U).events_executed, 1U);
    ASSERT_EQ(adc->samples().size(), 1U);
    const auto control = adc->read(0x08U, word, {});
    ASSERT_TRUE(control);
    EXPECT_EQ(control.value() & (1U << 2U), 0U);
    writeRcc(0x88U, 2U << 28U);
    ASSERT_EQ(events.advanceBy(10'000U).events_executed, 0U);
    EXPECT_EQ(adc->samples().size(), 1U);
}

} // namespace
