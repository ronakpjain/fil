#include "fil/cortexm/exceptions.hpp"
#include "fil/cortexm/system_control.hpp"
#include "fil/cpu/cortex_m4.hpp"
#include "fil/mem/memory_bus.hpp"
#include "../test_support.hpp"

#include <array>
#include <bit>
#include <cstdint>

namespace {

void stacksAndReturnsBasicFrame() {
    fil::mem::MemoryBus memory;
    fil::test::check(memory.mapRom(0x08000000U, 0x200U, "flash").hasValue(), "maps exception vectors");
    fil::test::check(memory.mapRam(0x20000000U, 0x1000U, "ram").hasValue(), "maps exception stack");
    const std::array<std::uint8_t, 4> handler{0x01U, 0x01U, 0x00U, 0x08U};
    fil::test::check(memory.loadBytes(0x0800002cU, handler).hasValue(), "loads SVC vector");
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
    fil::test::check(entered.hasValue(), "enters SVC exception");
    fil::test::check(state.r[15] == 0x08000100U && state.ipsr() == 11U, "vectors to SVC handler in handler mode");
    fil::test::check(state.r[14] == 0xfffffff9U && state.msp == 0x20000fe0U, "installs EXC_RETURN and basic frame");
    const auto stacked_r0 = memory.read32(state.msp);
    fil::test::check(stacked_r0 && stacked_r0.value() == 0x12345678U, "stacks core registers at fixed offsets");

    auto returned = exceptions.exceptionReturn(state, 0xfffffff9U);
    fil::test::check(returned.hasValue(), "returns from SVC exception");
    fil::test::check(state.r[15] == 0x08000080U && state.ipsr() == 0U, "restores thread PC and mode");
    fil::test::check(state.msp == 0x20001000U && state.r[0] == 0x12345678U, "restores stack pointer and registers");
}

void stacksAndReturnsExtendedFloatingPointFrame() {
    fil::mem::MemoryBus memory;
    fil::test::check(memory.mapRom(0x08000000U, 0x200U, "fp-flash").hasValue(), "maps FP exception vectors");
    fil::test::check(memory.mapRam(0x20000000U, 0x1000U, "fp-ram").hasValue(), "maps FP exception stack");
    const std::array<std::uint8_t, 4> handler{0x01U, 0x01U, 0x00U, 0x08U};
    fil::test::check(memory.loadBytes(0x0800003cU, handler).hasValue(), "loads SysTick vector");
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
    fil::test::check(entered.hasValue(), "enters an exception with active FP context");
    fil::test::check(state.r[14] == 0xffffffe9U && state.msp == 0x20000f98U,
                     "uses extended-frame EXC_RETURN and reserves 104 bytes");
    const auto stacked_s0 = memory.read32(state.msp);
    const auto stacked_s15 = memory.read32(state.msp + 60U);
    const auto stacked_xpsr = memory.read32(state.msp + 100U);
    fil::test::check(stacked_s0 && stacked_s0.value() == std::bit_cast<std::uint32_t>(1.5F),
                     "stacks S0 at the extended frame base");
    fil::test::check(stacked_s15 && stacked_s15.value() == std::bit_cast<std::uint32_t>(-2.25F),
                     "stacks S15 in the extended frame");
    fil::test::check(stacked_xpsr && (stacked_xpsr.value() & fil::cpu::xpsr_t) != 0U,
                     "places the basic frame after floating-point state");

    state.s[0] = 0.0F;
    state.s[15] = 0.0F;
    state.fpscr = 0U;
    auto returned = exceptions.exceptionReturn(state, 0xffffffe9U);
    fil::test::check(returned.hasValue(), "returns from an extended floating-point frame");
    fil::test::check(state.s[0] == 1.5F && state.s[15] == -2.25F && state.fpscr == 0x01000000U,
                     "restores S0-S15 and FPSCR");
    fil::test::check(state.msp == 0x20001000U && state.r[15] == 0x08000080U,
                     "restores the pre-exception stack and PC after an extended frame");
}

void restoresThreadStackSelectionFromExcReturn() {
    fil::mem::MemoryBus memory;
    fil::test::check(memory.mapRom(0x08000000U, 0x200U, "psp-flash").hasValue(), "maps PSP exception vectors");
    fil::test::check(memory.mapRam(0x20000000U, 0x1000U, "psp-ram").hasValue(), "maps PSP exception stack");
    const std::array<std::uint8_t, 4> handler{0x01U, 0x01U, 0x00U, 0x08U};
    fil::test::check(memory.loadBytes(0x0800002cU, handler).hasValue(), "loads PSP SVC vector");
    fil::cortexm::SystemControl system(0x08000000U);
    fil::cortexm::ExceptionController exceptions(memory, system);
    fil::cpu::CpuState state;
    state.msp = 0x20001000U;
    state.psp = 0x20000f00U;
    state.r[13] = state.psp;
    state.r[15] = 0x08000080U;
    state.xpsr = fil::cpu::xpsr_t;
    state.control = 2U;

    fil::test::check(exceptions.enter(state, 11U).hasValue(), "enters SVC from PSP thread mode");
    state.control = 0U;
    fil::test::check(exceptions.exceptionReturn(state, 0xfffffffdU).hasValue(), "returns to PSP thread mode");
    fil::test::check((state.control & 2U) != 0U && state.r[13] == state.psp,
                     "EXC_RETURN bit 2 restores CONTROL.SPSEL and visible SP");
}

} // namespace

void runExceptionTests() {
    stacksAndReturnsBasicFrame();
    stacksAndReturnsExtendedFloatingPointFrame();
    restoresThreadStackSelectionFromExcReturn();
}
