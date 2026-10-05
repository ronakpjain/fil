#include "fil/cortexm/system_control.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace {

// RM0440 section 14.2 specifies the STM32G4 read-only calibration literal.
TEST(CortexMTest, ReportsReadOnlySysTickCalibrationValue) {
    fil::cortexm::SystemControl system;
    constexpr auto word = fil::mem::AccessSize::word;
    auto calibration = system.read(0xe01cU, word, {});
    ASSERT_TRUE(calibration);
    EXPECT_EQ(calibration.value(), 0x3e8U);
    ASSERT_TRUE(system.write(0xe01cU, word, 0xffffffffU, {}));
    calibration = system.read(0xe01cU, word, {});
    ASSERT_TRUE(calibration);
    EXPECT_EQ(calibration.value(), 0x3e8U);
}

// RM0440 section 14.1 specifies four implemented priority bits (16 levels).
TEST(CortexMTest, AllDeviceInterruptPrioritiesImplementExactlyFourBits) {
    fil::cortexm::SystemControl system;
    for (std::uint32_t irq = 0U; irq < 102U; ++irq) {
        for (const std::uint32_t value : {0U, 0x0fU, 0x10U, 0x7fU, 0xa5U, 0xffU}) {
            SCOPED_TRACE(irq);
            SCOPED_TRACE(value);
            ASSERT_TRUE(system.write(0xe400U + irq, fil::mem::AccessSize::byte, value, {}));
            const auto priority = system.read(0xe400U + irq, fil::mem::AccessSize::byte, {});
            ASSERT_TRUE(priority);
            EXPECT_EQ(priority.value(), value & 0xf0U);
        }
    }
}

TEST(CortexMTest, ModelsSysTick) {
    fil::cortexm::SystemControl system;
    EXPECT_TRUE(system.write(0xe014U, fil::mem::AccessSize::word, 3U, {}).hasValue())
        << "writes SysTick LOAD";
    EXPECT_TRUE(system.write(0xe010U, fil::mem::AccessSize::word, 3U, {}).hasValue())
        << "enables SysTick interrupt";
    system.advanceCycles(3);
    EXPECT_TRUE(!system.nextPending(0, 0, 0).has_value())
        << "SysTick does not pend before LOAD+1 cycles";
    system.advanceCycles(1);
    const auto next = system.nextPending(0, 0, 0);
    EXPECT_TRUE(next == static_cast<std::uint16_t>(fil::cortexm::ExceptionNumber::sys_tick))
        << "pends SysTick deterministically on wrap";
    const auto ctrl = system.read(0xe010U, fil::mem::AccessSize::word, {});
    const auto ctrl_again = system.read(0xe010U, fil::mem::AccessSize::word, {});
    EXPECT_TRUE(ctrl && (ctrl.value() & (1U << 16U)) != 0) << "reports SysTick COUNTFLAG";
    EXPECT_TRUE(ctrl_again && (ctrl_again.value() & (1U << 16U)) == 0)
        << "clears COUNTFLAG on read";
}

TEST(CortexMTest, SamplesInterruptLevelsWithoutInventingPendingEdges) {
    fil::cortexm::SystemControl system;
    constexpr auto word = fil::mem::AccessSize::word;
    system.setInterruptLine(5U, true);
    EXPECT_FALSE(system.hasEnabledPending()); // Latched even while disabled.
    ASSERT_TRUE(system.write(0xe100U, word, 1U << 5U, {}));
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 21U);
    system.clearPending(21U);
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 21U);
    ASSERT_TRUE(system.write(0xe280U, word, 1U << 5U, {}));
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 21U);

    system.enter(21U);
    system.setInterruptLine(5U, true); // Same level, not a new edge.
    const auto pending = system.read(0xe200U, word, {});
    ASSERT_TRUE(pending);
    EXPECT_EQ(pending.value() & (1U << 5U), 0U);
    system.setInterruptLine(5U, false); // Handler acknowledges source.
    system.leave(21U);
    EXPECT_FALSE(system.hasEnabledPending());

    system.setInterruptLine(5U, true);
    system.setInterruptLine(5U, false); // A pulse still latches pending.
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 21U);
    system.enter(21U);
    system.setInterruptLine(5U, true); // New edge during active handler.
    system.setInterruptLine(5U, false);
    system.leave(21U);
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 21U);
    system.enter(21U);
    system.leave(21U);
    EXPECT_FALSE(system.hasEnabledPending());
}

TEST(CortexMTest, ResetClearsInterruptLevelsAndPendingSummary) {
    fil::cortexm::SystemControl system;
    ASSERT_TRUE(system.write(0xe100U, fil::mem::AccessSize::word, 1U, {}));
    system.setInterruptLine(0U, true);
    system.reset(0x08000000U);
    ASSERT_TRUE(system.write(0xe100U, fil::mem::AccessSize::word, 1U, {}));
    EXPECT_FALSE(system.hasEnabledPending());
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
    system.setInterruptLine(240U, true); // Outside implemented IRQ range.
    EXPECT_FALSE(system.hasEnabledPending());
}

TEST(CortexMTest, TakablePendingCacheTracksMasksPrioritiesAndLifecycle) {
    fil::cortexm::SystemControl system;
    constexpr auto word = fil::mem::AccessSize::word;

    ASSERT_TRUE(system.write(0xe100U, word, 0x3U, {})); // IRQ0 and IRQ1 enabled.
    ASSERT_TRUE(system.write(0xe400U, word, 0x4020U, {})); // IRQ0=0x20, IRQ1=0x40.
    system.pend(16U);
    system.pend(17U);
    EXPECT_TRUE(system.hasTakablePending(0U, 0U, 0U));
    EXPECT_FALSE(system.hasTakablePending(1U, 0U, 0U));
    EXPECT_FALSE(system.hasTakablePending(0U, 0x20U, 0U));
    EXPECT_FALSE(system.hasTakablePending(0U, 0U, 1U));
    EXPECT_FALSE(system.hasTakablePending(0U, 0U, 1U)); // Cache the masked null result.
    EXPECT_TRUE(system.hasTakablePending(0U, 0U, 0U)); // Unmask without a generation change.

    ASSERT_TRUE(system.write(0xe401U, fil::mem::AccessSize::byte, 0x10U, {}));
    system.clearPending(16U);
    EXPECT_TRUE(system.hasTakablePending(0U, 0U, 0U)); // IRQ1 now outranks IRQ0.

    // A pending peripheral line is visible after its NVIC enable is written.
    system.reset(0x08000000U);
    system.setInterruptLine(5U, true);
    EXPECT_FALSE(system.hasTakablePending(0U, 0U, 0U));
    ASSERT_TRUE(system.write(0xe100U, word, 1U << 5U, {}));
    EXPECT_TRUE(system.hasTakablePending(0U, 0U, 0U));

    // Active exception changes affect nesting; a lower-urgency pending IRQ
    // cannot preempt until the current exception returns.
    ASSERT_TRUE(system.write(0xe405U, fil::mem::AccessSize::byte, 0x40U, {}));
    ASSERT_TRUE(system.write(0xe406U, fil::mem::AccessSize::byte, 0x60U, {}));
    system.enter(21U);
    ASSERT_TRUE(system.write(0xe100U, word, 1U << 6U, {})); // Enable the nested IRQ too.
    system.pend(22U);
    EXPECT_FALSE(system.hasTakablePending(0U, 0U, 0U));
    ASSERT_TRUE(system.write(0xe406U, fil::mem::AccessSize::byte, 0x10U, {}));
    EXPECT_TRUE(system.hasTakablePending(0U, 0U, 0U));
    system.leave(21U);
    EXPECT_TRUE(system.hasTakablePending(0U, 0U, 0U));

    system.reset(0x08000000U);
    EXPECT_FALSE(system.hasTakablePending(0U, 0U, 0U));
}

TEST(CortexMTest, BasepriArbitrationUsesOnlyImplementedPriorityBits) {
    fil::cortexm::SystemControl system;
    ASSERT_TRUE(system.write(0xe100U, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_TRUE(system.write(0xe400U, fil::mem::AccessSize::byte, 0x10U, {}));
    system.pend(16U);
    EXPECT_EQ(system.nextPending(0U, 0x0fU, 0U), 16U);
    EXPECT_EQ(system.nextPending(0U, 0x20U, 0U), 16U);
    EXPECT_FALSE(system.nextPending(0U, 0x10U, 0U));
    EXPECT_FALSE(system.nextPending(0U, 0x1fU, 0U));
}

TEST(CortexMTest, ModelsNvicAndScb) {
    fil::cortexm::SystemControl system(0x08000000U);
    EXPECT_TRUE(system.write(0xe100U, fil::mem::AccessSize::word, 1U << 5U, {}).hasValue())
        << "enables external IRQ";
    EXPECT_TRUE(system.write(0xe405U, fil::mem::AccessSize::byte, 0x80U, {}).hasValue())
        << "programs IRQ priority byte";
    system.pend(21U);
    EXPECT_TRUE(system.nextPending(0, 0, 0) == 21U) << "selects enabled external IRQ";
    EXPECT_TRUE(!system.nextPending(1, 0, 0)) << "PRIMASK masks external IRQ";
    EXPECT_TRUE(!system.nextPending(0, 0x80U, 0)) << "BASEPRI masks equal-priority IRQ";
    system.clearPending(21U);
    system.pend(22U);
    EXPECT_TRUE(!system.nextPending(0, 0, 0)) << "ignores a pending but disabled external IRQ";
    system.pend(21U);
    EXPECT_TRUE(system.write(0xe406U, fil::mem::AccessSize::byte, 0xffU, {}).hasValue())
        << "writes all priority bits";
    const auto priority_word = system.read(0xe404U, fil::mem::AccessSize::word, {});
    EXPECT_TRUE(priority_word && ((priority_word.value() >> 16U) & 0xffU) == 0xf0U)
        << "implements four NVIC priority bits";

    EXPECT_TRUE(system.write(0xed04U, fil::mem::AccessSize::word, 1U << 28U, {}).hasValue())
        << "pends PendSV through ICSR";
    EXPECT_TRUE(system.nextPending(0, 0, 0) == 14U)
        << "lower exception number wins equal-priority tie";
    system.enter(11U);
    EXPECT_TRUE(!system.nextPending(0, 0, 0))
        << "equal-priority PendSV cannot preempt an active SVC";
    system.leave(11U);
    EXPECT_TRUE(system.write(0xed88U, fil::mem::AccessSize::word, 0x00f00000U, {}).hasValue())
        << "writes CPACR";
    EXPECT_TRUE(system.fpuEnabled()) << "recognizes full CP10/CP11 access";
    EXPECT_TRUE(system.write(0xed0cU, fil::mem::AccessSize::word, 0x05fa0004U, {}).hasValue())
        << "writes keyed AIRCR reset request";
    EXPECT_TRUE(system.consumeResetRequest() && !system.consumeResetRequest())
        << "consumes reset request once";
}

// RM0440 14.1 delegates these architectural registers to PM0214. These
// tests pin the modeled contract; they are not a substitute for that audit.
TEST(CortexMTest, SetsAndClearsInterruptEnablePendingAndActiveBits) {
    fil::cortexm::SystemControl system;
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(system.write(0xe100U, word, 1U << 17U, {}));
    ASSERT_TRUE(system.write(0xef00U, word, 17U, {}));
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 33U);
    ASSERT_TRUE(system.write(0xe280U, word, 1U << 17U, {}));
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
    ASSERT_TRUE(system.write(0xe200U, word, 1U << 17U, {}));
    system.enter(33U);
    EXPECT_EQ(system.read(0xe300U, word, {}).value(), 1U << 17U);
    EXPECT_EQ(system.read(0xe200U, word, {}).value(), 0U);
    system.leave(33U);
    EXPECT_EQ(system.read(0xe300U, word, {}).value(), 0U);
    ASSERT_TRUE(system.write(0xe180U, word, 1U << 17U, {}));
    EXPECT_EQ(system.read(0xe100U, word, {}).value(), 0U);
}

TEST(CortexMTest, SetsAndClearsSystemExceptionPendingBits) {
    fil::cortexm::SystemControl system;
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(system.write(0xed04U, word, (1U << 28U) | (1U << 26U), {}));
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 14U);
    const auto status = system.read(0xed04U, word, {});
    ASSERT_TRUE(status);
    EXPECT_EQ((status.value() >> 12U) & 0x1ffU, 14U);
    ASSERT_TRUE(system.write(0xed04U, word, 1U << 27U, {}));
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 15U);
    ASSERT_TRUE(system.write(0xed04U, word, 1U << 25U, {}));
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(CortexMTest, MasksSystemHandlerPriorityBytesAndPreservesNeighborLanes) {
    fil::cortexm::SystemControl system;
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(system.write(0xed18U, word, 0xabcdef12U, {}));
    EXPECT_EQ(system.read(0xed18U, word, {}).value(), 0xa0c0e010U);
    ASSERT_TRUE(system.write(0xed19U, fil::mem::AccessSize::byte, 0x35U, {}));
    EXPECT_EQ(system.read(0xed18U, word, {}).value(), 0xa0c03010U);
    EXPECT_EQ(system.priority(5U), 0x30U);
}

TEST(CortexMTest, RequiresAircrKeyAndAlignsVectorTableBase) {
    fil::cortexm::SystemControl system;
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(system.write(0xed0cU, word, 0x12340004U, {}));
    EXPECT_FALSE(system.resetRequested());
    ASSERT_TRUE(system.write(0xed0cU, word, 0x05fa0304U, {}));
    EXPECT_TRUE(system.consumeResetRequest());
    EXPECT_FALSE(system.consumeResetRequest());
    EXPECT_EQ(system.read(0xed0cU, word, {}).value(), 0xfa050300U);
    ASSERT_TRUE(system.write(0xed08U, word, 0x080081ffU, {}));
    EXPECT_EQ(system.vectorBase(), 0x08008180U);
}

TEST(CortexMTest, CycleCounterAdvancesOnlyWhenEnabledAndWraps) {
    fil::cortexm::SystemControl system;
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(system.write(0x1004U, word, 0xfffffffeU, {}));
    system.advanceCycles(10U);
    EXPECT_EQ(system.read(0x1004U, word, {}).value(), 0xfffffffeU);
    ASSERT_TRUE(system.write(0x1000U, word, 1U, {}));
    system.advanceCycles(3U);
    EXPECT_EQ(system.read(0x1004U, word, {}).value(), 1U);
    ASSERT_TRUE(system.write(0x1000U, word, 0U, {}));
    system.advanceCycles(100U);
    EXPECT_EQ(system.read(0x1004U, word, {}).value(), 1U);
}

TEST(CortexMTest, StoresSystemControlLanesAndResetsThem) {
    fil::cortexm::SystemControl system;
    constexpr auto word = fil::mem::AccessSize::word;
    for (const auto offset : {0xed10U, 0xed14U, 0xed24U, 0xed34U, 0xed38U,
                              0xed88U, 0xedfcU, 0xef34U}) {
        ASSERT_TRUE(system.write(offset, word, 0x12345678U, {}));
        ASSERT_TRUE(system.write(offset + 2U, fil::mem::AccessSize::halfword, 0xabcdU, {}));
        EXPECT_EQ(system.read(offset, word, {}).value(), 0xabcd5678U);
    }
    EXPECT_FALSE(system.fpuEnabled());
    ASSERT_TRUE(system.write(0xed88U, word, 0x00f00000U, {}));
    EXPECT_TRUE(system.fpuEnabled());
    system.reset(0x08008000U);
    EXPECT_EQ(system.ccr(), 1U << 9U);
    EXPECT_EQ(system.vectorBase(), 0x08008000U);
    EXPECT_EQ(system.fpccr(), 0U);
    EXPECT_FALSE(system.fpuEnabled());
    EXPECT_EQ(system.read(0xed00U, word, {}).value(), 0x410fc241U);
    ASSERT_TRUE(system.write(0xed00U, word, 0U, {}));
    EXPECT_EQ(system.read(0xed00U, word, {}).value(), 0x410fc241U);
}

TEST(CortexMTest, RejectsCrossRegisterSystemAccesses) {
    fil::cortexm::SystemControl system;
    EXPECT_FALSE(system.read(0xe013U, fil::mem::AccessSize::halfword, {}));
    EXPECT_FALSE(system.write(0xe013U, fil::mem::AccessSize::halfword, 0U, {}));
    EXPECT_FALSE(system.read(0x100000U, fil::mem::AccessSize::word, {}));
    EXPECT_FALSE(system.write(0x100000U, fil::mem::AccessSize::word, 0U, {}));
}

TEST(CortexMTest, MasksSysTickReloadAndClearsCountFlagOnCurrentWrite) {
    fil::cortexm::SystemControl system;
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(system.write(0xe014U, word, 0xff000003U, {}));
    EXPECT_EQ(system.read(0xe014U, word, {}).value(), 3U);
    ASSERT_TRUE(system.write(0xe010U, word, 1U, {}));
    system.advanceCycles(4U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
    ASSERT_TRUE(system.write(0xe018U, word, 0xffffffffU, {}));
    EXPECT_EQ(system.read(0xe018U, word, {}).value(), 0U);
    EXPECT_EQ(system.read(0xe010U, word, {}).value() & (1U << 16U), 0U);
    ASSERT_TRUE(system.write(0xe010U, word, 0U, {}));
    system.advanceCycles(10U);
    EXPECT_EQ(system.read(0xe018U, word, {}).value(), 0U);
}

} // namespace
