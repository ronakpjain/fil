#include "fil/cortexm/system_control.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace {

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

} // namespace
