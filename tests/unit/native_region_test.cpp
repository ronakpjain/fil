#include "fil/cpu/native_region.hpp"
#include "fil/cpu/cortex_m4.hpp"
#include "fil/cpu/micro_jit.hpp"
#include "fil/mem/memory_bus.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>

namespace {
using namespace fil::cpu;
constexpr std::uint32_t pc = 0x08004000U;

template <std::size_t N>
auto code(const std::array<std::uint16_t, N>& words) {
    std::array<std::uint8_t, N * 2U> bytes{};
    for (std::size_t i = 0U; i < N; ++i) {
        bytes[i * 2U] = static_cast<std::uint8_t>(words[i]);
        bytes[i * 2U + 1U] = static_cast<std::uint8_t>(words[i] >> 8U);
    }
    return bytes;
}

TEST(NativeRegionTest, FormsBothConditionalEdgesAndARealBackedgeWithoutUnrolling) {
    const auto bytes = code(std::array<std::uint16_t, 4>{0x3001U,0x3901U,0xd1fcU,0xbe00U});
    NativeRegionPlan plan;
    std::string error;
    ASSERT_TRUE(buildNativeRegion(bytes, pc, pc, plan, error)) << error;
    ASSERT_EQ(plan.count, 4U);
    EXPECT_FALSE(plan.capacity_limited);
    const auto& branch = plan.ops[2];
    ASSERT_EQ(branch.flow, RegionFlow::conditional);
    EXPECT_EQ(plan.ops[branch.edges[0]].pc, pc + 6U);
    EXPECT_EQ(branch.edges[1], 0U);
    EXPECT_EQ(plan.ops[3].flow, RegionFlow::checked);
}

TEST(NativeRegionTest, InvalidSpansAreAtomicAndCapacityCutsAreExplicitEdges) {
    const std::array<std::uint8_t, 4> bytes{};
    NativeRegionPlan plan; plan.count = 42U;
    std::string error;
    EXPECT_FALSE(buildNativeRegion(bytes, pc, pc + 1U, plan, error));
    EXPECT_EQ(plan.count, 42U);
    EXPECT_FALSE(buildNativeRegion(bytes, UINT32_MAX - 1U, UINT32_MAX - 1U, plan, error));
    EXPECT_EQ(plan.count, 42U);
    std::array<std::uint16_t, NativeRegionPlan::max_ops + 4U> words;
    words.fill(0xbf00U);
    const auto large = code(words);
    ASSERT_TRUE(buildNativeRegion(large, pc, pc, plan, error)) << error;
    EXPECT_EQ(plan.count, NativeRegionPlan::max_ops);
    EXPECT_TRUE(plan.capacity_limited);
    EXPECT_EQ(plan.ops[plan.count - 1U].edges[0], NativeRegionOp::external);
}

TEST(NativeRegionTest, DirectCallsIncludeTheirReturnContinuationWithoutPredictingReturns) {
    const auto bytes = code(std::array<std::uint16_t, 6>{
        0xf000U,0xf802U,0x2009U,0xbe00U,0x3101U,0x4770U});
    NativeRegionPlan plan; std::string error;
    ASSERT_TRUE(buildNativeRegion(bytes, pc, pc, plan, error)) << error;
    const auto& call = plan.ops[0];
    ASSERT_EQ(call.flow, RegionFlow::call);
    EXPECT_EQ(plan.ops[call.edges[0]].pc, pc + 8U);
    EXPECT_EQ(plan.ops[call.edges[1]].pc, pc + 4U);
    ASSERT_EQ(plan.count, 5U);
    bool return_found = false;
    for (std::size_t i = 0U; i < plan.count; ++i) {
        if (plan.ops[i].pc == pc + 10U) {
            return_found = true;
            EXPECT_EQ(plan.ops[i].flow, RegionFlow::indirect);
            EXPECT_EQ(plan.ops[i].edges[0], NativeRegionOp::external);
        }
    }
    EXPECT_TRUE(return_found);
}

#if defined(__x86_64__) && !defined(_WIN32)
TEST(NativeRegionTest, EmittedLoopsMatchCheckedCpuAtEveryInstructionAndCycleCut) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t, 4>{0x3001U,0x3901U,0xd1fcU,0xbe00U});
    std::string error;
    auto jit = MicroJit::create(error); ASSERT_TRUE(jit) << error;
    const auto function = jit->compileRegion(bytes, pc, pc, error);
    ASSERT_NE(function, nullptr) << error;
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRom(pc, bytes.size(), "region"));
    ASSERT_TRUE(memory.loadBytes(pc, bytes));
    CortexM4 reference(memory);
    for (const auto stalls : {std::array<std::uint32_t,2>{0U,0U},
                             std::array<std::uint32_t,2>{0U,3U},
                             std::array<std::uint32_t,2>{2U,4U}})
    for (const bool have_fetch : {false, true})
    for (const auto entry_offset : {0U,2U,4U,6U})
    for (const auto iterations : {0U,1U,4U})
    for (std::uint32_t limit = 0U; limit <= 13U; ++limit)
    for (std::uint64_t credit = 0U; credit <= 40U; ++credit) {
        CpuState state;
        state.r[0] = 0x7ffffffeU; state.r[1] = iterations;
        state.r[15] = pc + entry_offset; state.instruction_address = pc + 100U;
        state.xpsr |= xpsr_c | xpsr_v | xpsr_q;
        reference.state() = state;
        NativeRegionContext context;
        context.cpu_state = &state; context.instruction_limit = limit; context.cycle_limit = credit;
        context.sequential_fetch_stall = stalls[0]; context.nonsequential_fetch_stall = stalls[1];
        context.have_fetch = have_fetch; context.fetch_end = pc;
        std::uint32_t retired = 0U, fetch_end = pc;
        std::uint64_t cycles = 0U;
        bool have = have_fetch;
        NativeRegionExit reason;
        for (;;) {
            if (retired == limit) { reason = NativeRegionExit::instruction_budget; break; }
            const auto next = reference.state().r[15];
            if (next == pc + 6U) { reason = NativeRegionExit::checked_instruction; break; }
            const auto before = reference.state();
            const auto step = reference.stepFast();
            ASSERT_EQ(step.reason, StopReason::step_complete);
            const auto charge = step.cycles + stalls[have && next == fetch_end ? 0U : 1U];
            if (charge > credit - cycles) {
                reference.state() = before;
                reason = NativeRegionExit::cycle_budget;
                break;
            }
            cycles += charge; ++retired;
            have = true; fetch_end = next + step.instruction_size;
        }
        function(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state())) << limit << '/' << credit << '/' << iterations;
        EXPECT_EQ(context.exit_reason, reason);
        EXPECT_EQ(context.retired_count, retired);
        EXPECT_EQ(context.retired_cycles, cycles);
        EXPECT_EQ(context.have_fetch, have);
        EXPECT_EQ(context.fetch_end, fetch_end);
    }
}

TEST(NativeRegionTest, MillionInstructionLoopStaysInOneCompiledInvocation) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t, 2>{0x3001U,0xe7fdU});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error);
    ASSERT_NE(function, nullptr) << error;
    CpuState state; state.r[15] = pc;
    NativeRegionContext context;
    context.cpu_state = &state; context.instruction_limit = 1'000'000U;
    context.cycle_limit = UINT64_MAX;
    function(&context);
    EXPECT_EQ(context.exit_reason, NativeRegionExit::instruction_budget);
    EXPECT_EQ(context.retired_count, 1'000'000U);
    EXPECT_EQ(state.r[0], 500'000U);
    EXPECT_EQ(state.r[15], pc);
    EXPECT_EQ(state.instruction_address, pc + 2U);
}

TEST(NativeRegionTest, CallsAndCheckedMemoryExitsPreserveStateAndZeroCredit) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t, 6>{
        0xf000U,0xf802U,0x2009U,0xbe00U,0x3101U,0x4770U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error);
    ASSERT_NE(function, nullptr) << error;
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRom(pc, bytes.size(), "call")); ASSERT_TRUE(memory.loadBytes(pc, bytes));
    CortexM4 reference(memory); reference.state().r[15] = pc;
    auto state = reference.state();
    std::uint64_t cycles = 0U;
    for (unsigned i = 0U; i < 4U; ++i) cycles += reference.stepFast().cycles;
    NativeRegionContext context; context.cpu_state = &state;
    context.instruction_limit = 100U; context.cycle_limit = UINT64_MAX;
    function(&context);
    EXPECT_TRUE(bitwiseEqual(state, reference.state()));
    EXPECT_EQ(context.retired_count, 4U);
    EXPECT_EQ(context.retired_cycles, cycles);
    EXPECT_EQ(context.exit_reason, NativeRegionExit::checked_instruction);
    // A region with a memory first instruction exits before the actual access.
    const auto load = code(std::array<std::uint16_t, 1>{0x6808U});
    const auto load_function = jit->compileRegion(load, pc, pc, error); ASSERT_NE(load_function, nullptr);
    state.r[15] = pc; const auto before = state;
    context.cycle_limit = 0U; context.have_fetch = false; context.fetch_end = 123U;
    load_function(&context);
    EXPECT_TRUE(bitwiseEqual(state, before));
    EXPECT_EQ(context.retired_count, 0U);
    EXPECT_EQ(context.retired_cycles, 0U);
    EXPECT_EQ(context.exit_reason, NativeRegionExit::checked_instruction);
    EXPECT_FALSE(context.have_fetch); EXPECT_EQ(context.fetch_end, 123U);
    state.setItState(0x08U); const auto it_before = state;
    load_function(&context);
    EXPECT_TRUE(bitwiseEqual(state, it_before));
    EXPECT_EQ(context.exit_reason, NativeRegionExit::invalid_state);
    function(nullptr);
}

TEST(NativeRegionTest, ActualIndirectTargetsKeepExactCostsAndRejectExceptionalReturns) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t, 3>{0x4770U,0x2009U,0xbe00U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr);
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRom(pc, bytes.size(), "return")); ASSERT_TRUE(memory.loadBytes(pc, bytes));
    CortexM4 reference(memory);
    for (const auto target : {pc | 1U, pc + 3U, pc + 33U, pc + 2U, 0xffffffd1U,
                             0xfffffff1U, 0xfffffff9U, 0xfffffffdU,
                             0xffffffe1U, 0xffffffe9U, 0xffffffedU})
    for (std::uint64_t credit = 0U; credit <= 12U; ++credit) {
        CpuState state; state.r[15] = pc; state.r[14] = target;
        state.instruction_address = pc + 100U;
        const auto initial = state;
        reference.state() = state;
        NativeRegionContext context; context.cpu_state = &state;
        context.instruction_limit = 3U; context.cycle_limit = credit;
        function(&context);
        if ((target & 1U) == 0U || isExceptionReturn(target)) {
            EXPECT_EQ(context.exit_reason, NativeRegionExit::checked_instruction);
            EXPECT_EQ(context.retired_count, 0U);
            EXPECT_TRUE(bitwiseEqual(state, initial));
            continue;
        }
        std::uint32_t retired = 0U; std::uint64_t cycles = 0U;
        while (retired < 3U && reference.state().r[15] == pc) {
            const auto before = reference.state();
            const auto step = reference.stepFast();
            if (step.cycles > credit - cycles) { reference.state() = before; break; }
            cycles += step.cycles; ++retired;
        }
        EXPECT_EQ(context.retired_count, retired);
        EXPECT_EQ(context.retired_cycles, cycles);
        EXPECT_TRUE(bitwiseEqual(state, reference.state()));
    }
    // BLX reads an allocated ordinary guest register, updates LR, and follows
    // the actual target into a compiled continuation without a target profile.
    const auto blx = code(std::array<std::uint16_t, 3>{0x4780U,0x3101U,0xbe00U});
    const auto call = jit->compileRegion(blx, pc, pc, error); ASSERT_NE(call, nullptr);
    CpuState state; state.r[0] = pc + 3U; state.r[15] = pc;
    NativeRegionContext context; context.cpu_state = &state;
    context.instruction_limit = 10U; context.cycle_limit = UINT64_MAX;
    call(&context);
    EXPECT_EQ(context.retired_count, 2U);
    EXPECT_EQ(context.exit_reason, NativeRegionExit::checked_instruction);
    EXPECT_EQ(state.r[14], pc + 3U);
    EXPECT_EQ(state.r[1], 1U);
    EXPECT_EQ(state.r[15], pc + 4U);
}

TEST(NativeRegionTest, LoopAdmissionUsesWideCycleArithmeticAndUncompiledEntriesExitUnchanged) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t, 2>{0x4600U,0xe7fdU});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr);
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRom(pc, bytes.size(), "wide-cost")); ASSERT_TRUE(memory.loadBytes(pc, bytes));
    CortexM4 reference(memory); reference.state().r[15] = pc;
    const auto initial = reference.state();
    std::uint64_t full_cost = 0U;
    for (unsigned i = 0U; i < 2U; ++i)
        full_cost += reference.stepFast().cycles + std::uint64_t{UINT32_MAX};
    ASSERT_GT(full_cost, UINT32_MAX);
    for (const auto credit : {full_cost - 1U, full_cost, full_cost + 1U}) {
        auto state = initial;
        NativeRegionContext context; context.cpu_state = &state;
        context.instruction_limit = 2U; context.cycle_limit = credit;
        context.sequential_fetch_stall = context.nonsequential_fetch_stall = UINT32_MAX;
        function(&context);
        if (credit >= full_cost) {
            EXPECT_EQ(context.exit_reason, NativeRegionExit::instruction_budget);
            EXPECT_EQ(context.retired_count, 2U);
            EXPECT_EQ(context.retired_cycles, full_cost);
            EXPECT_TRUE(bitwiseEqual(state, reference.state()));
        } else {
            EXPECT_EQ(context.exit_reason, NativeRegionExit::cycle_budget);
            EXPECT_EQ(context.retired_count, 1U);
            EXPECT_EQ(state.r[15], pc + 2U);
        }
    }
    auto state = initial; state.r[15] = pc + 0x1000U;
    const auto before = state;
    NativeRegionContext context; context.cpu_state = &state;
    context.instruction_limit = 100U; context.cycle_limit = UINT64_MAX;
    function(&context);
    EXPECT_EQ(context.exit_reason, NativeRegionExit::external_edge);
    EXPECT_EQ(context.retired_count, 0U);
    EXPECT_TRUE(bitwiseEqual(state, before));
}
#endif
} // namespace
