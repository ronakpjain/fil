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

} // namespace
