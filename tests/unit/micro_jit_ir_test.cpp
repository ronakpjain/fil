#include "fil/cpu/decoder.hpp"
#include "fil/cpu/micro_jit_ir.hpp"

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

TEST(MicroJitIrTest, FlagsMatchAddWithCarry) {
    std::vector<MicroSource> sources{narrow(0x2001U, 0x08000000U)};
    MicroBlock block;
    ASSERT_TRUE(fil::cpu::buildMicroBlock({sources.data(), sources.size()}, block).ok);
    EXPECT_GE(fil::cpu::microBlockCost(block), 1U);
}

} // namespace
