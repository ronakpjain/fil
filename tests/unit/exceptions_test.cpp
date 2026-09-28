#include "fil/cortexm/exceptions.hpp"
#include "fil/cortexm/system_control.hpp"
#include "fil/cpu/cortex_m4.hpp"
#include "fil/mem/memory_bus.hpp"

#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstdint>

namespace {

TEST(ExceptionTest, StacksAndReturnsBasicFrame) {
    fil::mem::MemoryBus memory;
    EXPECT_TRUE(memory.mapRom(0x08000000U, 0x200U, "flash").hasValue()) << "maps exception vectors";
    EXPECT_TRUE(memory.mapRam(0x20000000U, 0x1000U, "ram").hasValue()) << "maps exception stack";
    const std::array<std::uint8_t, 4> handler{0x01U, 0x01U, 0x00U, 0x08U};
    EXPECT_TRUE(memory.loadBytes(0x0800002cU, handler).hasValue()) << "loads SVC vector";
    fil::cortexm::SystemControl system(0x08000000U);
    fil::cortexm::ExceptionController exceptions(memory, system);
    fil::cpu::CpuState state;
    state.msp = 0x20001000U;
    state.r[13] = state.msp;
    state.r[15] = 0x08000080U;
    state.r[0] = 0x12345678U;
    state.r[14] = 0x08000041U;
    state.xpsr = fil::cpu::xpsr_t | fil::cpu::xpsr_z;

    auto entered = exceptions.enter(state, 11U);
    EXPECT_TRUE(entered.hasValue()) << "enters SVC exception";
    EXPECT_TRUE(state.r[15] == 0x08000100U && state.ipsr() == 11U)
        << "vectors to SVC handler in handler mode";
    EXPECT_TRUE(state.r[14] == 0xfffffff9U && state.msp == 0x20000fe0U)
        << "installs EXC_RETURN and basic frame";
    const auto stacked_r0 = memory.read32(state.msp);
    EXPECT_TRUE(stacked_r0 && stacked_r0.value() == 0x12345678U)
        << "stacks core registers at fixed offsets";

    auto returned = exceptions.exceptionReturn(state, 0xfffffff9U);
    EXPECT_TRUE(returned.hasValue()) << "returns from SVC exception";
    EXPECT_TRUE(state.r[15] == 0x08000080U && state.ipsr() == 0U) << "restores thread PC and mode";
    EXPECT_TRUE(state.msp == 0x20001000U && state.r[0] == 0x12345678U)
        << "restores stack pointer and registers";
}

TEST(ExceptionTest, NestedReturnPreservesRependedOuterInterrupt) {
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRom(0x08000000U, 0x200U, "flash"));
    ASSERT_TRUE(memory.mapRam(0x20000000U, 0x1000U, "ram"));
    const std::array<std::uint8_t, 8> handlers{
        0x01U, 0x01U, 0x00U, 0x08U, 0x21U, 0x01U, 0x00U, 0x08U,
    };
    ASSERT_TRUE(memory.loadBytes(0x08000040U, handlers));
    fil::cortexm::SystemControl system;
    fil::cortexm::ExceptionController exceptions(memory, system);
    fil::cpu::CpuState state;
    state.msp = state.r[13] = 0x20001000U;
    state.r[15] = 0x08000080U;
    state.xpsr = fil::cpu::xpsr_t;
    ASSERT_TRUE(system.write(0xe100U, fil::mem::AccessSize::word, 3U, {}));
    ASSERT_TRUE(system.write(0xe400U, fil::mem::AccessSize::word, 0x4080U, {}));

    system.pend(16U);
    auto entered = exceptions.enterPending(state);
    ASSERT_TRUE(entered && entered.value());
    EXPECT_EQ(state.ipsr(), 16U);
    system.pend(17U);
    entered = exceptions.enterPending(state);
    ASSERT_TRUE(entered && entered.value());
    EXPECT_EQ(state.ipsr(), 17U);
    system.pend(16U); // A second occurrence while the outer handler is preempted.

    ASSERT_TRUE(exceptions.exceptionReturn(state, 0xfffffff1U));
    EXPECT_EQ(state.ipsr(), 16U);
    EXPECT_EQ(system.activeException(), 16U);
    const auto pending = system.read(0xe200U, fil::mem::AccessSize::word, {});
    ASSERT_TRUE(pending);
    EXPECT_EQ(pending.value() & 1U, 1U);
    const auto active = system.read(0xe300U, fil::mem::AccessSize::word, {});
    ASSERT_TRUE(active);
    EXPECT_EQ(active.value() & 3U, 1U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U)); // Cannot preempt itself.

    ASSERT_TRUE(exceptions.exceptionReturn(state, 0xfffffff9U));
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 16U);
    entered = exceptions.enterPending(state);
    ASSERT_TRUE(entered && entered.value());
    EXPECT_EQ(state.ipsr(), 16U);
    ASSERT_TRUE(exceptions.exceptionReturn(state, 0xfffffff9U));
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(ExceptionTest, StacksAndReturnsExtendedFloatingPointFrame) {
    fil::mem::MemoryBus memory;
    EXPECT_TRUE(memory.mapRom(0x08000000U, 0x200U, "fp-flash").hasValue())
        << "maps FP exception vectors";
    EXPECT_TRUE(memory.mapRam(0x20000000U, 0x1000U, "fp-ram").hasValue())
        << "maps FP exception stack";
    const std::array<std::uint8_t, 4> handler{0x01U, 0x01U, 0x00U, 0x08U};
    EXPECT_TRUE(memory.loadBytes(0x0800003cU, handler).hasValue()) << "loads SysTick vector";
    fil::cortexm::SystemControl system(0x08000000U);
    fil::cortexm::ExceptionController exceptions(memory, system);
    fil::cpu::CpuState state;
    state.msp = 0x20001000U;
    state.r[13] = state.msp;
    state.r[15] = 0x08000080U;
    state.xpsr = fil::cpu::xpsr_t;
    state.control = 4U;
    state.s[0] = 1.5F;
    state.s[15] = -2.25F;
    state.fpscr = 0x01000000U;

    auto entered = exceptions.enter(state, 15U);
    EXPECT_TRUE(entered.hasValue()) << "enters an exception with active FP context";
    EXPECT_TRUE(state.r[14] == 0xffffffe9U && state.msp == 0x20000f98U)
        << "uses extended-frame EXC_RETURN and reserves 104 bytes";
    const auto stacked_s0 = memory.read32(state.msp);
    const auto stacked_s15 = memory.read32(state.msp + 60U);
    const auto stacked_xpsr = memory.read32(state.msp + 100U);
    EXPECT_TRUE(stacked_s0 && stacked_s0.value() == std::bit_cast<std::uint32_t>(1.5F))
        << "stacks S0 at the extended frame base";
    EXPECT_TRUE(stacked_s15 && stacked_s15.value() == std::bit_cast<std::uint32_t>(-2.25F))
        << "stacks S15 in the extended frame";
    EXPECT_TRUE(stacked_xpsr && (stacked_xpsr.value() & fil::cpu::xpsr_t) != 0U)
        << "places the basic frame after floating-point state";

    state.s[0] = 0.0F;
    state.s[15] = 0.0F;
    state.fpscr = 0U;
    auto returned = exceptions.exceptionReturn(state, 0xffffffe9U);
    EXPECT_TRUE(returned.hasValue()) << "returns from an extended floating-point frame";
    EXPECT_TRUE(state.s[0] == 1.5F && state.s[15] == -2.25F && state.fpscr == 0x01000000U)
        << "restores S0-S15 and FPSCR";
    EXPECT_TRUE(state.msp == 0x20001000U && state.r[15] == 0x08000080U)
        << "restores the pre-exception stack and PC after an extended frame";
}

TEST(ExceptionTest, RestoresThreadStackSelectionFromExcReturn) {
    fil::mem::MemoryBus memory;
    EXPECT_TRUE(memory.mapRom(0x08000000U, 0x200U, "psp-flash").hasValue())
        << "maps PSP exception vectors";
    EXPECT_TRUE(memory.mapRam(0x20000000U, 0x1000U, "psp-ram").hasValue())
        << "maps PSP exception stack";
    const std::array<std::uint8_t, 4> handler{0x01U, 0x01U, 0x00U, 0x08U};
    EXPECT_TRUE(memory.loadBytes(0x0800002cU, handler).hasValue()) << "loads PSP SVC vector";
    fil::cortexm::SystemControl system(0x08000000U);
    fil::cortexm::ExceptionController exceptions(memory, system);
    fil::cpu::CpuState state;
    state.msp = 0x20001000U;
    state.psp = 0x20000f00U;
    state.r[13] = state.psp;
    state.r[15] = 0x08000080U;
    state.xpsr = fil::cpu::xpsr_t;
    state.control = 2U;

    EXPECT_TRUE(exceptions.enter(state, 11U).hasValue()) << "enters SVC from PSP thread mode";
    state.control = 0U;
    EXPECT_TRUE(exceptions.exceptionReturn(state, 0xfffffffdU).hasValue())
        << "returns to PSP thread mode";
    EXPECT_TRUE((state.control & 2U) != 0U && state.r[13] == state.psp)
        << "EXC_RETURN bit 2 restores CONTROL.SPSEL and visible SP";
}

TEST(ExceptionTest, DefersFloatingPointStackingUntilHandlerTouch) {
    fil::mem::MemoryBus memory;
    EXPECT_TRUE(memory.mapRom(0x08000000U, 0x200U, "lazy-flash").hasValue())
        << "maps lazy exception vectors";
    EXPECT_TRUE(memory.mapRam(0x20000000U, 0x1000U, "lazy-ram").hasValue())
        << "maps lazy exception stack";
    const std::array<std::uint8_t, 4> handler{0x01U, 0x01U, 0x00U, 0x08U};
    EXPECT_TRUE(memory.loadBytes(0x0800003cU, handler).hasValue()) << "loads SysTick vector";
    fil::cortexm::SystemControl system(0x08000000U);
    fil::cortexm::ExceptionController exceptions(memory, system);
    fil::cpu::CpuState state;
    state.msp = 0x20001000U;
    state.r[13] = state.msp;
    state.r[15] = 0x08000080U;
    state.xpsr = fil::cpu::xpsr_t;
    state.control = 4U;
    state.s[0] = 1.5F;
    state.s[15] = -2.25F;
    state.fpscr = 0x01000000U;

    EXPECT_TRUE(system.fpccr() == 0U) << "FPCCR reset leaves lazy stacking disabled";
    system.setFpccr(0xc0000000U);
    EXPECT_TRUE((system.fpccr() & 0x40000000U) != 0U) << "firmware-style FPCCR write enables LSPEN";
    EXPECT_TRUE(exceptions.enter(state, 15U).hasValue())
        << "enters an exception with active FP context";
    EXPECT_TRUE(state.r[14] == 0xffffffe9U && state.msp == 0x20000f98U)
        << "reserves the full extended frame including the FP area";
    EXPECT_TRUE(state.fp_lazy_active && state.fp_lazy_base == 0x20000f98U)
        << "records a pending lazy FP reservation at the frame base";
    const auto reserved_s0 = memory.read32(state.msp);
    EXPECT_TRUE(reserved_s0 && reserved_s0.value() == 0U)
        << "leaves the reserved FP area unwritten";
    const auto stacked_xpsr = memory.read32(state.msp + 100U);
    EXPECT_TRUE(stacked_xpsr && (stacked_xpsr.value() & fil::cpu::xpsr_t) != 0U)
        << "still stacks the basic frame after the reserved area";

    state.s[0] = 0.0F;
    state.s[15] = 0.0F;
    state.fpscr = 0U;
    EXPECT_TRUE(exceptions.exceptionReturn(state, 0xffffffe9U).hasValue())
        << "returns from an untouched lazy frame";
    EXPECT_TRUE(!state.fp_lazy_active)
        << "clears the pending lazy reservation on return";
    EXPECT_TRUE(state.msp == 0x20001000U && state.r[15] == 0x08000080U)
        << "restores stack and PC without reading the reserved area";
    EXPECT_TRUE(state.s[0] == 0.0F && state.s[15] == 0.0F && state.fpscr == 0U)
        << "leaves FP state untouched when nothing was stacked";
}

TEST(ExceptionTest, NestedEntryFallsBackToEagerStacking) {
    fil::mem::MemoryBus memory;
    EXPECT_TRUE(memory.mapRom(0x08000000U, 0x200U, "nest-flash").hasValue())
        << "maps nested exception vectors";
    EXPECT_TRUE(memory.mapRam(0x20000000U, 0x1000U, "nest-ram").hasValue())
        << "maps nested exception stack";
    const std::array<std::uint8_t, 4> handler{0x01U, 0x01U, 0x00U, 0x08U};
    EXPECT_TRUE(memory.loadBytes(0x0800003cU, handler).hasValue()) << "loads SysTick vector";
    EXPECT_TRUE(memory.loadBytes(0x08000038U, handler).hasValue()) << "loads PendSV vector";
    fil::cortexm::SystemControl system(0x08000000U);
    system.setFpccr(0xc0000000U);
    fil::cortexm::ExceptionController exceptions(memory, system);
    fil::cpu::CpuState state;
    state.msp = 0x20001000U;
    state.r[13] = state.msp;
    state.r[15] = 0x08000080U;
    state.xpsr = fil::cpu::xpsr_t;
    state.control = 4U;
    state.s[0] = 1.5F;
    state.fpscr = 0x01000000U;

    EXPECT_TRUE(exceptions.enter(state, 15U).hasValue()) << "enters outer exception lazily";
    EXPECT_TRUE(state.fp_lazy_active) << "outer reservation is pending";
    const std::uint32_t outer_sp = state.msp;
    EXPECT_TRUE(exceptions.enter(state, 14U).hasValue()) << "enters nested exception";
    EXPECT_TRUE(!state.fp_lazy_active) << "nested entry abandons lazy mode for eager stacking";
    const auto nested_s0 = memory.read32(state.msp);
    EXPECT_TRUE(nested_s0 && nested_s0.value() == std::bit_cast<std::uint32_t>(1.5F))
        << "nested level stacks FP state eagerly";
    EXPECT_TRUE(state.msp < outer_sp) << "nested frame reserves below the outer frame";
}

} // namespace
