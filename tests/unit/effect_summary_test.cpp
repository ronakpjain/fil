#include "fil/cpu/effect_summary.hpp"
#include "fil/sim/lane_horizon.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace {

using fil::cpu::EffectSummary;
using fil::cpu::MicroOp;
using fil::cpu::NativeOp;
using fil::cpu::NativeTraceOp;
using fil::cpu::PrivateMemoryFacts;
using fil::cpu::PrivateRegion;

NativeTraceOp integerOp(const MicroOp op, const std::uint16_t cycles = 1U) {
    NativeTraceOp result;
    result.native.kind = NativeOp::integer;
    result.native.integer.op = op;
    result.execution_cycles = cycles;
    return result;
}

NativeTraceOp memoryOp(
    const NativeOp kind, const std::uint32_t address, const std::uint16_t cycles = 2U
) {
    NativeTraceOp result;
    result.native.kind = kind;
    result.native.absolute = true;
    result.native.absolute_address = address;
    result.execution_cycles = cycles;
    return result;
}

constexpr std::array<PrivateRegion, 2> regions{{
    {0x08000000U, 0x08010000U},
    {0x20000000U, 0x20001000U},
}};

TEST(EffectSummaryTest, PureIntegerPrefixHasNoSharedEffect) {
    const std::vector<NativeTraceOp> ops{
        integerOp(MicroOp::mov_imm),
        integerOp(MicroOp::add_imm),
        integerOp(MicroOp::cmp_imm),
        integerOp(MicroOp::orr_reg),
    };
    const auto summary = fil::cpu::summarizeNativeEffects(ops);
    EXPECT_FALSE(summary.reaches_shared);
    EXPECT_FALSE(summary.unknown_control_flow);
    EXPECT_EQ(summary.local_ops, 4U);
    EXPECT_EQ(summary.minimum_cycles, 4U);
    EXPECT_EQ(summary.firstExternalOp(), 4U);
}

TEST(EffectSummaryTest, PrivateLiteralAndStackAccessesStayLocal) {
    std::vector<NativeTraceOp> ops{
        memoryOp(NativeOp::load_word, 0x08000080U),
        integerOp(MicroOp::add_imm),
    };
    auto load = memoryOp(NativeOp::store_word, 0U);
    load.native.absolute = false;
    load.native.rn = 13U;
    load.native.offset = 0xfffffff8U; // SP - 8
    ops.push_back(load);
    PrivateMemoryFacts facts;
    facts.regions = regions;
    facts.stack_pointer = 0x20000800U;
    const auto summary = fil::cpu::summarizeNativeEffects(ops, facts);
    EXPECT_FALSE(summary.reaches_shared);
    EXPECT_EQ(summary.local_ops, 3U);
    EXPECT_EQ(summary.minimum_cycles, 5U);
}

TEST(EffectSummaryTest, MmioStopsThePrefixAtItsEarliestOperation) {
    const std::vector<NativeTraceOp> ops{
        integerOp(MicroOp::mov_imm),
        integerOp(MicroOp::add_imm),
        memoryOp(NativeOp::load_word, 0x40000000U),
        integerOp(MicroOp::add_imm),
    };
    PrivateMemoryFacts facts;
    facts.regions = regions;
    const auto summary = fil::cpu::summarizeNativeEffects(ops, facts);
    EXPECT_TRUE(summary.reaches_shared);
    EXPECT_EQ(summary.local_ops, 2U);
    EXPECT_EQ(summary.minimum_cycles, 2U);
    EXPECT_EQ(summary.firstExternalOp(), 2U);
}

TEST(EffectSummaryTest, UnknownBaseAndControlFlowRemainConservative) {
    auto indirect = memoryOp(NativeOp::load_word, 0U);
    indirect.native.absolute = false;
    indirect.native.rn = 3U;
    indirect.native.offset = 0U;
    std::vector<NativeTraceOp> ops{integerOp(MicroOp::mov_imm), indirect};
    EXPECT_TRUE(fil::cpu::summarizeNativeEffects(ops).reaches_shared);

    ops = {integerOp(MicroOp::add_imm), integerOp(MicroOp::b_cond), integerOp(MicroOp::add_imm)};
    const auto branch = fil::cpu::summarizeNativeEffects(ops);
    EXPECT_TRUE(branch.reaches_shared);
    EXPECT_TRUE(branch.unknown_control_flow);
    EXPECT_EQ(branch.local_ops, 1U);
    EXPECT_EQ(branch.minimum_cycles, 1U);

    ops = {integerOp(MicroOp::add_imm), integerOp(MicroOp::b)};
    const auto terminal = fil::cpu::summarizeNativeEffects(ops);
    EXPECT_FALSE(terminal.reaches_shared);
    EXPECT_TRUE(terminal.unknown_control_flow);
    EXPECT_EQ(terminal.local_ops, 2U);
}

TEST(EffectSummaryTest, StackTransfersTrackSpAndDeclineOutsideFacts) {
    NativeTraceOp push;
    push.decoded.kind = fil::cpu::InstrKind::push;
    push.decoded.register_list = 0x3U;
    push.execution_cycles = 2U;
    PrivateMemoryFacts facts;
    facts.regions = regions;
    facts.stack_pointer = 0x20000800U;
    std::vector<NativeTraceOp> ops{push, push};
    const auto summary = fil::cpu::summarizeNativeEffects(ops, facts);
    EXPECT_FALSE(summary.reaches_shared);
    EXPECT_EQ(summary.local_ops, 2U);

    facts.stack_pointer = 0x20000004U;
    const auto declined = fil::cpu::summarizeNativeEffects(ops, facts);
    EXPECT_TRUE(declined.reaches_shared);
    EXPECT_EQ(declined.local_ops, 0U);
}

TEST(LaneHorizonTest, FrontierIsTheEarliestFiniteBound) {
    using fil::sim::HorizonProof;
    using fil::sim::conservativeFrontier;
    const std::array<HorizonProof, 3> lanes{{
        {std::nullopt, 1U},
        {30U, 1U},
        {20U, 1U},
    }};
    EXPECT_EQ(conservativeFrontier(lanes, 40U, 50U), 20U);
    EXPECT_EQ(conservativeFrontier(lanes, 10U, 50U), 10U);
    EXPECT_EQ(conservativeFrontier(lanes, std::nullopt, 5U), 5U);
    EXPECT_EQ(conservativeFrontier(lanes), 20U);
    const std::array<HorizonProof, 1> unbounded{{{std::nullopt, 1U}}};
    EXPECT_FALSE(conservativeFrontier(unbounded).has_value());
}

TEST(LaneHorizonTest, CycleConversionSaturatesAndKeepsFractionOut) {
    using fil::sim::horizonFromCycles;
    // 170 cycles at 170 MHz is exactly one microsecond.
    EXPECT_EQ(horizonFromCycles(10U, 170U, 170'000'000U), 1'010U);
    EXPECT_EQ(horizonFromCycles(0U, 170'000'000U, 170'000'000U), 1'000'000'000U);
    EXPECT_EQ(horizonFromCycles(5U, ~std::uint64_t{0}, 1U),
              std::numeric_limits<fil::sim::SimTimeNs>::max());
    EXPECT_FALSE(horizonFromCycles(0U, 1U, 0U).has_value());
}

} // namespace
