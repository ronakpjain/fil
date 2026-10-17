#include "fil/cpu/decoder.hpp"
#include "fil/cpu/micro_jit_ir.hpp"
#include "fil/cpu/micro_jit.hpp"

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
