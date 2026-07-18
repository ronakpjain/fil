#include "fil/cortexm/system_control.hpp"
#include "../test_support.hpp"

#include <cstdint>

namespace {

void modelsSysTick() {
    fil::cortexm::SystemControl system;
    fil::test::check(system.write(0xe014U, fil::mem::AccessSize::word, 3U, {}).hasValue(), "writes SysTick LOAD");
    fil::test::check(system.write(0xe010U, fil::mem::AccessSize::word, 3U, {}).hasValue(), "enables SysTick interrupt");
    system.advanceCycles(3);
    fil::test::check(
        !system.nextPending(0, 0, 0).has_value(),
        "SysTick does not pend before LOAD+1 cycles"
    );
    system.advanceCycles(1);
    const auto next = system.nextPending(0, 0, 0);
    fil::test::check(
        next == static_cast<std::uint16_t>(fil::cortexm::ExceptionNumber::sys_tick),
        "pends SysTick deterministically on wrap"
    );
    const auto ctrl = system.read(0xe010U, fil::mem::AccessSize::word, {});
    const auto ctrl_again = system.read(0xe010U, fil::mem::AccessSize::word, {});
    fil::test::check(ctrl && (ctrl.value() & (1U << 16U)) != 0, "reports SysTick COUNTFLAG");
    fil::test::check(ctrl_again && (ctrl_again.value() & (1U << 16U)) == 0, "clears COUNTFLAG on read");
}

void modelsNvicAndScb() {
    fil::cortexm::SystemControl system(0x08000000U);
    fil::test::check(system.write(0xe100U, fil::mem::AccessSize::word, 1U << 5U, {}).hasValue(), "enables external IRQ");
    fil::test::check(system.write(0xe405U, fil::mem::AccessSize::byte, 0x80U, {}).hasValue(), "programs IRQ priority byte");
    system.pend(21U);
    fil::test::check(system.nextPending(0, 0, 0) == 21U, "selects enabled external IRQ");
    fil::test::check(!system.nextPending(1, 0, 0), "PRIMASK masks external IRQ");
    fil::test::check(!system.nextPending(0, 0x80U, 0), "BASEPRI masks equal-priority IRQ");
    system.clearPending(21U);
    system.pend(22U);
    fil::test::check(!system.nextPending(0, 0, 0), "ignores a pending but disabled external IRQ");
    system.pend(21U);
    fil::test::check(system.write(0xe406U, fil::mem::AccessSize::byte, 0xffU, {}).hasValue(), "writes all priority bits");
    const auto priority_word = system.read(0xe404U, fil::mem::AccessSize::word, {});
    fil::test::check(priority_word && ((priority_word.value() >> 16U) & 0xffU) == 0xf0U, "implements four NVIC priority bits");

    fil::test::check(system.write(0xed04U, fil::mem::AccessSize::word, 1U << 28U, {}).hasValue(), "pends PendSV through ICSR");
    fil::test::check(system.nextPending(0, 0, 0) == 14U, "lower exception number wins equal-priority tie");
    system.enter(11U);
    fil::test::check(!system.nextPending(0, 0, 0), "equal-priority PendSV cannot preempt an active SVC");
    system.leave(11U);
    fil::test::check(system.write(0xed88U, fil::mem::AccessSize::word, 0x00f00000U, {}).hasValue(), "writes CPACR");
    fil::test::check(system.fpuEnabled(), "recognizes full CP10/CP11 access");
    fil::test::check(system.write(0xed0cU, fil::mem::AccessSize::word, 0x05fa0004U, {}).hasValue(), "writes keyed AIRCR reset request");
    fil::test::check(system.consumeResetRequest() && !system.consumeResetRequest(), "consumes reset request once");
}

} // namespace

void runCortexMTests() {
    modelsSysTick();
    modelsNvicAndScb();
}
