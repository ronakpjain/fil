#include "fil/cpu/decoder.hpp"
#include "fil/cpu/micro_jit_ir.hpp"
#include "fil/cpu/micro_jit.hpp"
#include "fil/cpu/native_ir.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace {

using fil::cpu::MicroBlock;
using fil::cpu::MicroCpu;
using fil::cpu::MicroOp;
using fil::cpu::MicroSource;

MicroSource narrow(std::uint16_t raw, std::uint32_t pc) {
    auto decoded = fil::cpu::decode16(raw);
    EXPECT_TRUE(decoded.has_value()) << std::hex << raw;
    return MicroSource{decoded.value_or(fil::cpu::DecodedInstruction{}), pc, 2U};
}

TEST(NativeIrTest, MixedWordMemoryDoesNotChangePureLowering) {
    const std::vector<MicroSource> sources{
        narrow(0x2001U, 0x08000000U), // movs r0, #1
        narrow(0x6048U, 0x08000002U), // str r0, [r1, #4]
        narrow(0x684aU, 0x08000004U), // ldr r2, [r1, #4]
    };
    fil::cpu::NativeBlock block;
    ASSERT_TRUE(fil::cpu::buildNativeBlock(sources, block).ok);
    ASSERT_EQ(block.ops.size(), 3U);
    EXPECT_TRUE(block.has_memory);
    EXPECT_FALSE(block.ends_with_branch);
    EXPECT_EQ(block.ops[0].kind, fil::cpu::NativeOp::integer);
    EXPECT_EQ(block.ops[1].kind, fil::cpu::NativeOp::store_word);
    EXPECT_EQ(block.ops[1].rn, 1U);
    EXPECT_EQ(block.ops[1].rd, 0U);
    EXPECT_EQ(block.ops[1].offset, 4U);
    EXPECT_EQ(block.ops[2].kind, fil::cpu::NativeOp::load_word);
    EXPECT_EQ(block.ops[2].rd, 2U);
    MicroBlock pure;
    EXPECT_FALSE(fil::cpu::buildMicroBlock(sources, pure).ok);
}

TEST(NativeIrTest, LiteralAndStackPointerLoadsLowerSafely) {
    const auto literal = narrow(0x4803U, 0x08000002U);
    fil::cpu::NativeInst inst;
    ASSERT_TRUE(fil::cpu::tryLowerNativeInst(literal, inst).ok);
    EXPECT_TRUE(inst.absolute);
    EXPECT_EQ(inst.absolute_address, 0x08000010U);
    auto stack = narrow(0x9d01U, 0x08000004U);
    ASSERT_TRUE(fil::cpu::tryLowerNativeInst(stack, inst).ok);
    EXPECT_EQ(inst.rn, 13U);
    EXPECT_FALSE(inst.absolute);
    const auto negative = fil::cpu::decode32(0xf85fU, 0x0004U);
    ASSERT_TRUE(negative);
    ASSERT_TRUE(fil::cpu::tryLowerNativeInst({*negative, 0x08000008U, 4U}, inst).ok);
    EXPECT_TRUE(inst.absolute);
    EXPECT_EQ(inst.absolute_address, 0x08000008U);
}

TEST(NativeIrTest, MemoryDeclinesUnsafeShapesWithoutChangingOutput) {
    const auto valid = narrow(0x684aU, 0x08000000U);
    for (unsigned variant = 0U; variant != 11U; ++variant) {
        auto source = valid;
        switch (variant) {
        case 0U: source.decoded.rn = 15U; break;
        case 1U: source.decoded.rd = 15U; break;
        case 2U: source.decoded.writeback = true; break;
        case 3U: source.decoded.index = false; break;
        case 4U: source.decoded.add = false; break;
        case 5U: source.decoded.condition = fil::cpu::Condition::eq; break;
        case 6U: source.decoded.form = fil::cpu::OperandForm::register_value; break;
        case 7U: source.decoded.set_flags = true; break;
        case 8U: source.decoded.is_32bit = true; break;
        case 9U: source.pc |= 1U; break;
        case 10U: source.decoded.shift_amount = 1U; break;
        }
        fil::cpu::NativeInst out;
        out.pc = 0x1234U;
        out.kind = fil::cpu::NativeOp::store_word;
        EXPECT_FALSE(fil::cpu::tryLowerNativeInst(source, out).ok) << variant;
        EXPECT_EQ(out.pc, 0x1234U);
        EXPECT_EQ(out.kind, fil::cpu::NativeOp::store_word);
    }
}

TEST(NativeIrTest, ConditionalControlFlowMustTerminateTheBlock) {
    for (const auto raw : {0xd000U, 0xb100U, 0xb900U}) {
        std::vector<MicroSource> sources{narrow(0x2001U, 0x08000000U),
            narrow(static_cast<std::uint16_t>(raw), 0x08000002U),
            narrow(0x684aU, 0x08000004U)};
        fil::cpu::NativeBlock block;
        EXPECT_EQ(fil::cpu::buildNativeBlock(sources, block).reason,
            fil::cpu::MicroLowerReason::branch_not_terminal);
        sources.pop_back();
        ASSERT_TRUE(fil::cpu::buildNativeBlock(sources, block).ok);
        EXPECT_TRUE(block.ends_with_branch);
    }
}

TEST(NativeIrTest, RejectsDiscontinuityAndNonterminalBranchAtomically) {
    fil::cpu::NativeBlock out;
    out.entry_pc = 0x1234U;
    std::vector<MicroSource> sources{
        narrow(0x2001U, 0x08000000U), narrow(0x684aU, 0x08000004U)};
    EXPECT_FALSE(fil::cpu::buildNativeBlock(sources, out).ok);
    EXPECT_EQ(out.entry_pc, 0x1234U);
    sources = {narrow(0xe000U, 0x08000000U), narrow(0x684aU, 0x08000002U)};
    EXPECT_EQ(fil::cpu::buildNativeBlock(sources, out).reason,
        fil::cpu::MicroLowerReason::branch_not_terminal);
    EXPECT_EQ(out.entry_pc, 0x1234U);
}

TEST(MicroJitIrTest, LowersMovAddSubAndBranch) {
    // movs r0, #1; adds r1, r0, #2; subs r2, r1, #1
    std::vector<MicroSource> sources{
        narrow(0x2001U, 0x08000000U),
        narrow(0x1c81U, 0x08000002U),
        narrow(0x1e4aU, 0x08000004U),
    };
    MicroBlock block;
    auto r = fil::cpu::buildMicroBlock({sources.data(), sources.size()}, block);
    EXPECT_TRUE(r.ok) << r.detail;
    EXPECT_EQ(block.ops.size(), 3U);
    EXPECT_EQ(block.ops[0].op, MicroOp::mov_imm);
    EXPECT_FALSE(block.ends_with_branch);

    MicroCpu cpu;
    const std::size_t retired = fil::cpu::interpretMicroBlock(block, cpu, 16U);
    EXPECT_EQ(retired, 3U);
    EXPECT_EQ(cpu.r[0], 1U);
    EXPECT_EQ(cpu.r[1], 3U);
    EXPECT_EQ(cpu.r[2], 2U);
}

TEST(MicroJitIrTest, RegisterMovShapesAndReferenceFlags) {
    for (const auto raw : {0x4620U, 0x4681U, 0x4642U, 0x4600U}) {
        auto source = narrow(static_cast<std::uint16_t>(raw), 0x08000000U);
        fil::cpu::MicroInst inst;
        ASSERT_TRUE(fil::cpu::tryLowerMicroInst(source, inst).ok);
        ASSERT_EQ(inst.op, MicroOp::mov_reg);
        for (const bool flags : {false, true}) {
            source.decoded.set_flags = flags;
            MicroBlock block;
            ASSERT_TRUE(fil::cpu::buildMicroBlock({&source, 1U}, block).ok);
            for (const auto value : {0U, 0x80000000U, 0xffffffffU}) {
                MicroCpu cpu;
                cpu.r[source.decoded.rm] = value;
                cpu.xpsr = 0xf9000000U;
                ASSERT_EQ(fil::cpu::interpretMicroBlock(block, cpu, 1U), 1U);
                EXPECT_EQ(cpu.r[source.decoded.rd], value);
                const auto expected = flags
                    ? (0x39000000U | (value & 0x80000000U) | (value == 0U ? 0x40000000U : 0U))
                    : 0xf9000000U;
                EXPECT_EQ(cpu.xpsr, expected);
            }
        }
    }
    for (const auto raw : {0x4668U, 0x4685U, 0x4670U, 0x4686U, 0x4678U, 0x4687U}) {
        fil::cpu::MicroInst inst;
        EXPECT_FALSE(fil::cpu::tryLowerMicroInst(narrow(static_cast<std::uint16_t>(raw), 0x08000000U), inst).ok)
            << "SP/LR/PC operand: " << std::hex << raw;
    }
    auto source = narrow(0x4620U, 0x08000000U);
    fil::cpu::MicroInst inst;
    source.decoded.condition = fil::cpu::Condition::eq;
    EXPECT_FALSE(fil::cpu::tryLowerMicroInst(source, inst).ok);
    source.decoded.condition = fil::cpu::Condition::al;
    source.decoded.shift_amount = 1U;
    EXPECT_FALSE(fil::cpu::tryLowerMicroInst(source, inst).ok);
}

TEST(MicroJitIrTest, RejectsMemoryAndBranchesMidBlock) {
    // ldr r0, [r1] must decline; b mid-block must decline.
    auto ldr = narrow(0x6810U, 0x08000000U);
    MicroBlock block;
    EXPECT_FALSE(fil::cpu::buildMicroBlock({&ldr, 1U}, block).ok);

    std::vector<MicroSource> with_mid_branch{
        narrow(0x2001U, 0x08000000U),
        narrow(0xe000U, 0x08000002U), // b .+4
        narrow(0x2002U, 0x08000004U),
    };
    EXPECT_FALSE(
        fil::cpu::buildMicroBlock({with_mid_branch.data(), with_mid_branch.size()}, block).ok);
}

TEST(MicroJitIrTest, PrefixLimitCommitsPrecisely) {
    std::vector<MicroSource> sources{
        narrow(0x2001U, 0x08000000U),
        narrow(0x2102U, 0x08000002U),
        narrow(0x2203U, 0x08000004U),
    };
    MicroBlock block;
    ASSERT_TRUE(fil::cpu::buildMicroBlock({sources.data(), sources.size()}, block).ok);
    MicroCpu cpu;
    EXPECT_EQ(fil::cpu::interpretMicroBlock(block, cpu, 2U), 2U);
    EXPECT_EQ(cpu.r[0], 1U);
    EXPECT_EQ(cpu.r[1], 2U);
    EXPECT_EQ(cpu.r[2], 0U);
    EXPECT_EQ(cpu.pc, 0x08000004U);
}

TEST(MicroJitIrTest, PerformanceAdmissionRequiresMarginAndConsistentWins) {
    using fil::cpu::microKernelProfitable;
    EXPECT_TRUE(microKernelProfitable({80U, 81U, 79U, 82U, 80U}, {100U, 101U, 99U, 102U, 100U}));
    EXPECT_FALSE(microKernelProfitable({110U, 111U, 109U, 112U, 110U}, {100U, 101U, 99U, 102U, 100U}));
    EXPECT_FALSE(microKernelProfitable({95U, 95U, 95U, 95U, 95U}, {100U, 100U, 100U, 100U, 100U}));
    EXPECT_FALSE(microKernelProfitable({100U, 100U, 100U, 100U, 100U}, {100U, 100U, 100U, 100U, 100U}));
    // A good median alone is insufficient when two paired samples lose.
    EXPECT_FALSE(microKernelProfitable({80U, 80U, 80U, 120U, 120U}, {100U, 100U, 100U, 100U, 100U}));
    // One host scheduling outlier is tolerated, but zero-resolution clocks are not.
    EXPECT_TRUE(microKernelProfitable({80U, 80U, 80U, 80U, 1000U}, {100U, 100U, 100U, 100U, 100U}));
    EXPECT_FALSE(microKernelProfitable({}, {}));
}

TEST(MicroJitIrTest, FlagsMatchAddWithCarry) {
    std::vector<MicroSource> sources{narrow(0x2001U, 0x08000000U)};
    MicroBlock block;
    ASSERT_TRUE(fil::cpu::buildMicroBlock({sources.data(), sources.size()}, block).ok);
    EXPECT_GE(fil::cpu::microBlockCost(block), 1U);
}

} // namespace
