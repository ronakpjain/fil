#include "fil/cpu/cortex_m4.hpp"
#include "fil/cpu/instruction.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace {

TEST(AluTest, AddWithCarryCoversFlagEdges) {
    const auto wrapped = fil::cpu::addWithCarry(0xffffffffU, 0U, true);
    EXPECT_TRUE(wrapped.value == 0U) << "addWithCarry wraps at 32 bits";
    EXPECT_TRUE(wrapped.z && wrapped.c && !wrapped.n && !wrapped.v)
        << "addWithCarry reports zero and carry";

    const auto positive_overflow = fil::cpu::addWithCarry(0x7fffffffU, 1U, false);
    EXPECT_TRUE(positive_overflow.value == 0x80000000U) << "addWithCarry retains overflow result";
    EXPECT_TRUE(positive_overflow.n && positive_overflow.v && !positive_overflow.c)
        << "addWithCarry separates overflow from carry";

    const auto subtraction = fil::cpu::addWithCarry(3U, ~5U, true);
    EXPECT_TRUE(subtraction.value == 0xfffffffeU && !subtraction.c)
        << "addWithCarry implements borrow-producing subtraction";
}

TEST(AluTest, ShiftsCoverArchitecturalBoundaries) {
    using fil::cpu::ShiftType;
    const auto unchanged = fil::cpu::shiftC(0x80000001U, ShiftType::lsl, 0, true);
    const auto lsl32 = fil::cpu::shiftC(0x00000001U, ShiftType::lsl, 32, false);
    const auto lsl33 = fil::cpu::shiftC(0xffffffffU, ShiftType::lsl, 33, true);
    EXPECT_TRUE(unchanged.value == 0x80000001U && unchanged.carry)
        << "zero LSL preserves value and carry";
    EXPECT_TRUE(lsl32.value == 0U && lsl32.carry) << "LSL by 32 exports bit zero";
    EXPECT_TRUE(lsl33.value == 0U && !lsl33.carry) << "large LSL clears result and carry";

    const auto lsr32 = fil::cpu::shiftC(0x80000001U, ShiftType::lsr, 32, false);
    const auto asr31 = fil::cpu::shiftC(0x80000000U, ShiftType::asr, 31, false);
    const auto asr40 = fil::cpu::shiftC(0x80000000U, ShiftType::asr, 40, false);
    EXPECT_TRUE(lsr32.value == 0U && lsr32.carry) << "LSR by 32 exports the sign bit";
    EXPECT_TRUE(asr31.value == 0xffffffffU && !asr31.carry) << "ASR sign-fills below 32";
    EXPECT_TRUE(asr40.value == 0xffffffffU && asr40.carry) << "large ASR saturates at sign bits";

    const auto ror4 = fil::cpu::shiftC(0x12345678U, ShiftType::ror, 4, false);
    const auto ror32 = fil::cpu::shiftC(0x80000001U, ShiftType::ror, 32, false);
    const auto rrx = fil::cpu::shiftC(0x00000003U, ShiftType::rrx, 1, true);
    EXPECT_TRUE(ror4.value == 0x81234567U && ror4.carry) << "ROR rotates and reports result bit 31";
    EXPECT_TRUE(ror32.value == 0x80000001U && ror32.carry)
        << "ROR by a word preserves value and exports bit 31";
    EXPECT_TRUE(rrx.value == 0x80000001U && rrx.carry) << "RRX shifts old carry through bit 31";
}

TEST(AluTest, ConditionsAndItStateAreIndependentHelpers) {
    using fil::cpu::Condition;
    const std::uint32_t flags = fil::cpu::xpsr_n | fil::cpu::xpsr_z | fil::cpu::xpsr_c;
    EXPECT_TRUE(fil::cpu::conditionPasses(Condition::eq, flags)) << "EQ observes Z";
    EXPECT_TRUE(!fil::cpu::conditionPasses(Condition::ne, flags)) << "NE inverts Z";
    EXPECT_TRUE(fil::cpu::conditionPasses(Condition::ls, flags)) << "LS accepts Z despite carry";
    EXPECT_TRUE(!fil::cpu::conditionPasses(Condition::ge, flags)) << "GE compares N with V";
    EXPECT_TRUE(fil::cpu::conditionPasses(Condition::le, flags)) << "LE accepts Z";
    EXPECT_TRUE(fil::cpu::conditionPasses(Condition::al, 0)) << "AL always passes";
    EXPECT_TRUE(!fil::cpu::conditionPasses(Condition::nv, flags)) << "NV never passes";

    EXPECT_TRUE(fil::cpu::advanceItState(0x08U) == 0U) << "single-slot IT block terminates";
    EXPECT_TRUE(fil::cpu::advanceItState(0x0cU) == 0x18U) << "multi-slot IT block shifts its mask";
    EXPECT_TRUE(fil::cpu::advanceItState(0x18U) == 0U) << "shifted final IT slot terminates";
}

} // namespace
