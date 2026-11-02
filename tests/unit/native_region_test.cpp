#include "fil/cpu/native_region.hpp"
#include "fil/cpu/cortex_m4.hpp"
#include "fil/cpu/micro_jit.hpp"
#include "fil/mem/memory_bus.hpp"

#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstdint>
#include <limits>

namespace {
using namespace fil::cpu;
constexpr std::uint32_t pc = 0x08004000U;
constexpr std::uint32_t ram = 0x20000000U;

class RegionPort final : public fil::mem::MmioDevice {
public:
    unsigned reads{0U}, writes{0U};
    fil::mem::MemoryResult<std::uint64_t> read(std::uint32_t, fil::mem::AccessSize,
        const fil::mem::AccessContext&) override { ++reads; return std::uint64_t{0U}; }
    fil::mem::MemoryResult<std::uint64_t> write(std::uint32_t, fil::mem::AccessSize,
        std::uint64_t, const fil::mem::AccessContext&) override { ++writes; return std::uint64_t{0U}; }
    std::string_view name() const noexcept override { return "region-port"; }
};

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

TEST(NativeRegionTest, WriteBindingsRequireCommittedModesAndNonExecutableRam) {
    if constexpr (std::endian::native != std::endian::little) GTEST_SKIP();
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRam(ram, 32U, "data"));
    ASSERT_TRUE(memory.mapRam(ram + 64U, 16U, "code", true));
    ASSERT_TRUE(memory.mapRom(pc, 16U, "rom"));
    ASSERT_TRUE(memory.mapFlash(pc + 64U, 16U, "flash"));
    ASSERT_TRUE(memory.mapAlias(ram + 128U, ram, 32U, "alias"));
    RegionPort port;
    ASSERT_TRUE(memory.mapMmio(0x40000000U, 32U, port, "port"));
    fil::mem::MemoryBus::DirectWriteWindow binding;
    EXPECT_FALSE(memory.directWriteWindow(ram, binding));
    memory.setReadFootprintTracking(false);
    EXPECT_FALSE(memory.directWriteWindow(ram, binding));
    memory.setWriteJournalTracking(false);
    ASSERT_TRUE(memory.directWriteWindow(ram + 1U, binding));
    const auto initial = binding;
    EXPECT_EQ(binding.begin, ram); EXPECT_EQ(binding.size, 32U);
    ASSERT_NE(binding.bytes, nullptr); ASSERT_NE(binding.mutation_sequence, nullptr);
    ASSERT_NE(binding.side_effect_generation, nullptr);
    for (const auto address : {ram + 64U, pc, pc + 64U, ram + 128U, 0x40000000U, 0x60000000U}) {
        EXPECT_FALSE(memory.directWriteWindow(address, binding));
        EXPECT_EQ(binding.bytes, initial.bytes);
        EXPECT_EQ(binding.begin, initial.begin); EXPECT_EQ(binding.size, initial.size);
        EXPECT_EQ(binding.mutation_sequence, initial.mutation_sequence);
        EXPECT_EQ(binding.side_effect_generation, initial.side_effect_generation);
    }
    {
        fil::mem::MemoryBus::StoreFootprintScope store_tracking(memory);
        EXPECT_FALSE(memory.directWriteWindow(ram, binding));
        EXPECT_EQ(binding.bytes, initial.bytes);
    }
    memory.setReadFootprintTracking(true);
    EXPECT_FALSE(memory.directWriteWindow(ram, binding));
    EXPECT_EQ(binding.bytes, initial.bytes);
    EXPECT_EQ(port.reads, 0U); EXPECT_EQ(port.writes, 0U);
}

TEST(NativeRegionTest, CanonicalNarrowAndWideStackFramesContinueGraphFormation) {
    const auto narrow = code(std::array<std::uint16_t, 2>{0xb503U,0xbd03U});
    const auto wide = code(std::array<std::uint16_t, 4>{0xe92dU,0x5ff0U,0xe8bdU,0x9ff0U});
    for (const bool broad : {false, true}) {
        const std::span<const std::uint8_t> bytes = broad
            ? std::span<const std::uint8_t>(wide) : std::span<const std::uint8_t>(narrow);
        NativeRegionPlan plan; std::string error;
        ASSERT_TRUE(buildNativeRegion(bytes, pc, pc, plan, error)) << error;
        ASSERT_EQ(plan.count, 2U);
        EXPECT_TRUE(plan.ops[0].stack.push());
        EXPECT_EQ(plan.ops[0].stack.count(), broad ? 10U : 3U);
        EXPECT_EQ(plan.ops[0].flow, RegionFlow::next);
        EXPECT_EQ(plan.ops[0].edges[0], 1U);
        EXPECT_TRUE(plan.ops[1].stack.loadsPc());
        EXPECT_EQ(plan.ops[1].flow, RegionFlow::indirect);
        EXPECT_EQ(plan.ops[1].stack.count(), broad ? 10U : 3U);
    }
}

#if defined(__x86_64__) && !defined(_WIN32)
TEST(NativeRegionTest, StackCallsMatchCpuRamAndTimingAcrossAllBudgetCutsAndBanks) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    const auto narrow = code(std::array<std::uint16_t, 9>{
        0x2055U,0xf000U,0xf802U,0x3201U,0xbe00U,0xb503U,0x2007U,0x3101U,0xbd03U});
    const auto wide = code(std::array<std::uint16_t, 11>{
        0x2455U,0xf000U,0xf802U,0x3201U,0xbe00U,0xe92dU,0x5ff0U,0x2407U,0x3501U,0xe8bdU,0x9ff0U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    for (const bool broad : {false, true}) {
        const std::span<const std::uint8_t> bytes = broad
            ? std::span<const std::uint8_t>(wide) : std::span<const std::uint8_t>(narrow);
        const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr) << error;
        fil::mem::MemoryBus memory, oracle;
        for (auto* bus : {&memory, &oracle}) {
            ASSERT_TRUE(bus->mapRom(pc, static_cast<std::uint32_t>(bytes.size()), "stack-call"));
            ASSERT_TRUE(bus->loadBytes(pc, bytes));
            ASSERT_TRUE(bus->mapRam(ram, 128U, "stack"));
            bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
        }
        CortexM4 reference(oracle);
        NativeRegionContext binding;
        ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read));
        ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
        for (const auto mode : {0U,1U,2U})
        for (const auto stalls : {std::array<std::uint32_t,2>{0U,0U},
                                 std::array<std::uint32_t,2>{0U,3U},
                                 std::array<std::uint32_t,2>{2U,4U}})
        for (std::uint32_t limit = 0U; limit <= 8U; ++limit)
        for (std::uint64_t credit = 0U; credit <= 64U; ++credit) {
            for (auto* bus : {&memory, &oracle})
                for (std::uint32_t offset = 0U; offset < 128U; offset += 8U)
                    ASSERT_TRUE(bus->write64(ram + offset, 0xaaaaaaaa55555555ULL));
            CpuState state;
            for (std::uint32_t reg = 0U; reg < 13U; ++reg) state.r[reg] = 0x10203040U + reg * 0x01010101U;
            state.r[13] = ram + 4U; state.msp = ram + 112U; state.psp = ram + 80U;
            state.r[14] = 0x87654321U; state.r[15] = pc; state.instruction_address = pc + 100U;
            state.control = mode == 0U ? 0U : 2U;
            state.xpsr |= xpsr_c | xpsr_v | xpsr_q | (mode == 2U ? 3U : 0U);
            reference.state() = state;
            auto context = binding; context.cpu_state = &state;
            context.instruction_limit = limit; context.cycle_limit = credit;
            context.sequential_fetch_stall = stalls[0]; context.nonsequential_fetch_stall = stalls[1];
            std::uint32_t retired = 0U, fetch_end = 0U;
            std::uint64_t cycles = 0U; bool have = false;
            NativeRegionExit reason;
            for (;;) {
                if (retired == limit) { reason = NativeRegionExit::instruction_budget; break; }
                const auto next = reference.state().r[15];
                if (next == pc + 8U) { reason = NativeRegionExit::checked_instruction; break; }
                std::uint16_t pipeline = 0U;
                const auto before = reference.state();
                FastStepResult step;
                const bool predictable = reference.peekPredictableCycles(pipeline);
                if (!predictable) {
                    // The only unpredictable operation in this fixture is a
                    // backed POP PC: it reads RAM but cannot modify it.
                    ASSERT_EQ(next, pc + (broad ? 18U : 16U));
                    step = reference.stepFast(); pipeline = step.cycles;
                }
                const auto charge = pipeline + stalls[have && next == fetch_end ? 0U : 1U];
                if (charge > credit - cycles) {
                    reference.state() = before;
                    reason = NativeRegionExit::cycle_budget; break;
                }
                if (predictable) step = reference.stepFast();
                ASSERT_EQ(step.reason, StopReason::step_complete); ASSERT_EQ(step.cycles, pipeline);
                ++retired; cycles += charge; have = true; fetch_end = next + step.instruction_size;
            }
            function(&context);
            EXPECT_TRUE(bitwiseEqual(state, reference.state())) << broad << '/' << mode << '/' << limit << '/' << credit;
            EXPECT_EQ(context.exit_reason, reason); EXPECT_EQ(context.retired_count, retired);
            EXPECT_EQ(context.retired_cycles, cycles); EXPECT_EQ(context.have_fetch, have);
            EXPECT_EQ(context.fetch_end, fetch_end);
            for (std::uint32_t offset = 0U; offset < 128U; offset += 8U)
                EXPECT_EQ(memory.read64(ram + offset).value(), oracle.read64(ram + offset).value());
            EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
            EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
        }
    }
}

TEST(NativeRegionTest, StackAdmissionAndExceptionalReturnsDeclineWithoutPartialEffects) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    for (const auto word : {0xb503U,0xbd03U}) {
        const bool push = word == 0xb503U;
        const auto bytes = code(std::array<std::uint16_t, 1>{static_cast<std::uint16_t>(word)});
        const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr);
        fil::mem::MemoryBus memory; RegionPort port;
        ASSERT_TRUE(memory.mapRam(ram, 16U, "short-frame"));
        ASSERT_TRUE(memory.mapMmio(0x40000000U, 16U, port, "port"));
        memory.setReadFootprintTracking(false); memory.setWriteJournalTracking(false);
        NativeRegionContext binding;
        ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read));
        ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
        for (const auto sp : {4U,ram + 8U,0x40000008U,0xffffffffU}) {
            CpuState state; state.r[15] = pc; state.r[0] = 0x12345678U;
            state.msp = state.r[13] = sp;
            const auto initial = state;
            const auto checkpoint = memory.sideEffectCheckpoint();
            auto context = binding; context.cpu_state = &state;
            context.instruction_limit = 1U; context.cycle_limit = UINT64_MAX;
            function(&context);
            EXPECT_EQ(context.exit_reason, NativeRegionExit::checked_instruction) << push << '/' << sp;
            EXPECT_EQ(context.retired_count, 0U); EXPECT_EQ(context.retired_cycles, 0U);
            EXPECT_TRUE(bitwiseEqual(state, initial));
            EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, checkpoint.mutation_sequence);
            EXPECT_EQ(memory.sideEffectCheckpoint().mmio_generation, checkpoint.mmio_generation);
            EXPECT_EQ(memory.read64(ram).value(), 0U);
            EXPECT_EQ(memory.read64(ram + 8U).value(), 0U);
        }
        EXPECT_EQ(port.reads, 0U); EXPECT_EQ(port.writes, 0U);
        if (!push) {
            for (const auto target : {pc + 2U,0xfffffff1U,0xfffffff9U,0xfffffffdU,
                                     0xffffffe1U,0xffffffe9U,0xffffffedU}) {
                ASSERT_TRUE(memory.write32(ram, 0xdeadbeefU));
                ASSERT_TRUE(memory.write32(ram + 4U, 0x87654321U));
                ASSERT_TRUE(memory.write32(ram + 8U, target));
                CpuState state; state.r[15] = pc; state.r[0] = 1U; state.r[1] = 2U;
                state.msp = ram; state.r[13] = ram + 12U;
                const auto initial = state;
                const auto checkpoint = memory.sideEffectCheckpoint();
                auto context = binding; context.cpu_state = &state;
                context.instruction_limit = 1U; context.cycle_limit = UINT64_MAX;
                function(&context);
                EXPECT_EQ(context.exit_reason, NativeRegionExit::checked_instruction);
                EXPECT_EQ(context.retired_count, 0U);
                EXPECT_TRUE(bitwiseEqual(state, initial));
                EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, checkpoint.mutation_sequence);
            }
        }
    }
}

TEST(NativeRegionTest, PlainStackPopsPreserveLrDataWithoutInventingAReturn) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t, 5>{
        0xe92dU,0x5001U,0xe8bdU,0x5001U,0xbe00U}); // r0, r12, LR; no PC
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr);
    fil::mem::MemoryBus memory, oracle;
    for (auto* bus : {&memory, &oracle}) {
        ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "plain-pop")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
        ASSERT_TRUE(bus->mapRam(ram, 64U, "stack"));
        bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
    }
    CortexM4 reference(oracle);
    CpuState state; state.r[15] = pc; state.r[14] = 0xffffffe9U;
    state.r[0] = 0x12345678U; state.r[12] = 0x87654321U;
    state.r[13] = state.msp = ram + 64U;
    reference.state() = state;
    std::uint64_t cycles = 0U;
    for (unsigned i = 0U; i < 2U; ++i) {
        const auto step = reference.stepFast(); ASSERT_EQ(step.reason, StopReason::step_complete);
        cycles += step.cycles;
    }
    NativeRegionContext context; context.cpu_state = &state;
    context.instruction_limit = 10U; context.cycle_limit = UINT64_MAX;
    ASSERT_TRUE(memory.directReadWindow(ram, context.ram_read));
    ASSERT_TRUE(memory.directWriteWindow(ram, context.ram_write));
    function(&context);
    EXPECT_EQ(context.retired_count, 2U); EXPECT_EQ(context.retired_cycles, cycles);
    EXPECT_EQ(context.exit_reason, NativeRegionExit::checked_instruction);
    EXPECT_TRUE(bitwiseEqual(state, reference.state()));
    EXPECT_FALSE(state.pending_exc_return);
    for (std::uint32_t offset = 0U; offset < 64U; offset += 8U)
        EXPECT_EQ(memory.read64(ram + offset).value(), oracle.read64(ram + offset).value());
    EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
}

TEST(NativeRegionTest, PopPcUsesActualInternalOrExternalTargetsWithExactRefillCosts) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t, 1>{0xbd03U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr);
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRom(pc, bytes.size(), "return")); ASSERT_TRUE(memory.loadBytes(pc, bytes));
    ASSERT_TRUE(memory.mapRam(ram, 32U, "stack"));
    memory.setReadFootprintTracking(false); memory.setWriteJournalTracking(false);
    CortexM4 reference(memory);
    for (const auto target : {pc | 1U,pc + 3U,pc + 33U,0x12345679U,0xffffffd1U}) {
        ASSERT_TRUE(memory.write32(ram, 0xdeadbeefU));
        ASSERT_TRUE(memory.write32(ram + 4U, 0x87654321U));
        ASSERT_TRUE(memory.write32(ram + 8U, target));
        CpuState state; state.r[15] = pc; state.msp = ram; state.r[13] = ram + 24U;
        reference.state() = state;
        const auto expected = reference.stepFast(); ASSERT_EQ(expected.reason, StopReason::step_complete);
        NativeRegionContext context; context.cpu_state = &state;
        context.instruction_limit = 1U; context.cycle_limit = UINT64_MAX;
        ASSERT_TRUE(memory.directReadWindow(ram, context.ram_read));
        function(&context);
        EXPECT_EQ(context.retired_count, 1U);
        EXPECT_EQ(context.retired_cycles, expected.cycles);
        EXPECT_TRUE(bitwiseEqual(state, reference.state()));
        EXPECT_EQ(context.exit_reason, target == (pc | 1U)
            ? NativeRegionExit::instruction_budget : NativeRegionExit::external_edge);
    }
}

TEST(NativeRegionTest, GuardedRamLoopMatchesCpuMemoryAndCountersAtEveryCut) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t, 6>{
        0x6808U,0x3001U,0x6008U,0x3a01U,0xd1faU,0xbe00U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr);
    fil::mem::MemoryBus memory, oracle;
    for (auto* bus : {&memory, &oracle}) {
        ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "loop")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
        ASSERT_TRUE(bus->mapRam(ram, 16U, "data"));
        bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
    }
    CortexM4 reference(oracle);
    NativeRegionContext binding;
    ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read));
    ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
    for (const auto iterations : {0U,1U,3U})
    for (const auto stalls : {std::array<std::uint32_t,2>{0U,0U}, std::array<std::uint32_t,2>{0U,3U}})
    for (std::uint32_t limit = 0U; limit <= 17U; ++limit)
    for (std::uint64_t credit = 0U; credit <= 45U; ++credit) {
        ASSERT_TRUE(memory.write32(ram, 0x7ffffffeU)); ASSERT_TRUE(oracle.write32(ram, 0x7ffffffeU));
        CpuState state; state.r[15] = pc; state.r[1] = ram; state.r[2] = iterations;
        state.instruction_address = pc + 100U; state.xpsr |= xpsr_c | xpsr_q;
        reference.state() = state;
        auto context = binding; context.cpu_state = &state;
        context.instruction_limit = limit; context.cycle_limit = credit;
        context.sequential_fetch_stall = stalls[0]; context.nonsequential_fetch_stall = stalls[1];
        std::uint32_t retired = 0U, fetch_end = 0U;
        std::uint64_t cycles = 0U; bool have = false;
        NativeRegionExit reason;
        for (;;) {
            if (retired == limit) { reason = NativeRegionExit::instruction_budget; break; }
            const auto next = reference.state().r[15];
            if (next == pc + 10U) { reason = NativeRegionExit::checked_instruction; break; }
            std::uint16_t pipeline = 0U; ASSERT_TRUE(reference.peekPredictableCycles(pipeline));
            const auto charge = pipeline + stalls[have && next == fetch_end ? 0U : 1U];
            if (charge > credit - cycles) { reason = NativeRegionExit::cycle_budget; break; }
            const auto step = reference.stepFast(); ASSERT_EQ(step.reason, StopReason::step_complete);
            ASSERT_EQ(step.cycles, pipeline);
            ++retired; cycles += charge; have = true; fetch_end = next + step.instruction_size;
        }
        function(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state())) << limit << '/' << credit;
        EXPECT_EQ(context.exit_reason, reason); EXPECT_EQ(context.retired_count, retired);
        EXPECT_EQ(context.retired_cycles, cycles); EXPECT_EQ(context.have_fetch, have);
        EXPECT_EQ(context.fetch_end, fetch_end);
        EXPECT_EQ(memory.read32(ram).value(), oracle.read32(ram).value());
        EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
        EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
        EXPECT_EQ(memory.executionGeneration(), oracle.executionGeneration());
        EXPECT_EQ(memory.readFootprint().words, oracle.readFootprint().words);
    }
}

TEST(NativeRegionTest, StoresRetainExactChangedByteCountsForEveryWidthAndAlignment) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    for (const auto word : {0x6008U,0x7008U,0x8008U}) {
        const auto bytes = code(std::array<std::uint16_t, 2>{static_cast<std::uint16_t>(word),0xbe00U});
        const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr);
        fil::mem::MemoryBus memory, oracle;
        for (auto* bus : {&memory, &oracle}) {
            ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "store")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
            ASSERT_TRUE(bus->mapRam(ram, 16U, "data"));
            bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
        }
        CortexM4 reference(oracle);
        NativeRegionContext binding;
        ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
        for (auto old : {0U,0xffffffffU,0x80808080U,0x7f7f7f7fU,0x11223344U})
        for (auto value : {0U,0xffffffffU,0x00800001U,0x01007f80U,0xffaa5500U,0x11223344U})
        for (std::uint32_t alignment = 0U; alignment < 4U; ++alignment) {
            ASSERT_TRUE(memory.write64(ram, old | (std::uint64_t{old} << 32U)));
            ASSERT_TRUE(oracle.write64(ram, old | (std::uint64_t{old} << 32U)));
            CpuState state; state.r[15] = pc; state.r[0] = value; state.r[1] = ram + alignment;
            state.xpsr |= xpsr_c | xpsr_v | xpsr_q;
            reference.state() = state;
            const auto expected = reference.stepFast(); ASSERT_EQ(expected.reason, StopReason::step_complete);
            auto context = binding; context.cpu_state = &state;
            context.instruction_limit = 1U; context.cycle_limit = UINT64_MAX;
            function(&context);
            EXPECT_TRUE(bitwiseEqual(state, reference.state()));
            EXPECT_EQ(context.retired_count, 1U); EXPECT_EQ(context.retired_cycles, expected.cycles);
            EXPECT_EQ(memory.read64(ram).value(), oracle.read64(ram).value());
            EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
            EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
        }
    }
}

TEST(NativeRegionTest, LoadsCoverSignedWidthsUnalignedDataAndBothReadBindings) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    for (const auto first : {0x680aU,0x780aU,0x880aU,0xf991U,0xf9b1U}) {
        const bool wide = first >= 0xf000U;
        const auto bytes = code(std::array<std::uint16_t, 3>{static_cast<std::uint16_t>(first),
            static_cast<std::uint16_t>(wide ? 0x2000U : 0xbe00U),0xbe00U});
        const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr);
        fil::mem::MemoryBus memory;
        ASSERT_TRUE(memory.mapRom(pc, bytes.size(), "load")); ASSERT_TRUE(memory.loadBytes(pc, bytes));
        ASSERT_TRUE(memory.mapRam(ram, 16U, "data"));
        ASSERT_TRUE(memory.write64(ram, 0x0180ff007f8001ffULL));
        memory.setReadFootprintTracking(false);
        CortexM4 reference(memory);
        for (const bool secondary : {false, true})
        for (std::uint32_t alignment = 0U; alignment < 4U; ++alignment) {
            CpuState state; state.r[15] = pc; state.r[1] = ram + alignment;
            state.xpsr |= xpsr_c | xpsr_v | xpsr_q;
            reference.state() = state;
            const auto expected = reference.stepFast(); ASSERT_EQ(expected.reason, StopReason::step_complete);
            NativeRegionContext context; context.cpu_state = &state;
            context.instruction_limit = 1U; context.cycle_limit = UINT64_MAX;
            ASSERT_TRUE(memory.directReadWindow(ram, secondary ? context.code_read : context.ram_read));
            function(&context);
            EXPECT_EQ(context.retired_count, 1U);
            EXPECT_EQ(context.retired_cycles, expected.cycles);
            EXPECT_TRUE(bitwiseEqual(state, reference.state()));
        }
    }
}

TEST(NativeRegionTest, MissesAndTinyBindingsExitBeforeMemoryOrPeripheralEffects) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    for (const auto word : {0x6808U,0x6008U,0x8808U,0x8008U}) {
        const auto bytes = code(std::array<std::uint16_t, 2>{static_cast<std::uint16_t>(word),0xbe00U});
        const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr);
        fil::mem::MemoryBus memory; RegionPort port;
        ASSERT_TRUE(memory.mapRam(ram, 3U, "tiny"));
        ASSERT_TRUE(memory.mapMmio(0x40000000U, 16U, port, "port"));
        memory.setReadFootprintTracking(false); memory.setWriteJournalTracking(false);
        NativeRegionContext binding;
        ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read));
        ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
        for (const auto address : {ram + 2U, ram - 1U, 0x40000000U, 0xffffffffU}) {
            CpuState state; state.r[15] = pc; state.r[0] = 0x12345678U; state.r[1] = address;
            const auto initial = state;
            const auto checkpoint = memory.sideEffectCheckpoint();
            auto context = binding; context.cpu_state = &state;
            context.instruction_limit = 10U; context.cycle_limit = UINT64_MAX;
            function(&context);
            EXPECT_EQ(context.exit_reason, NativeRegionExit::checked_instruction);
            EXPECT_EQ(context.retired_count, 0U); EXPECT_EQ(context.retired_cycles, 0U);
            EXPECT_TRUE(bitwiseEqual(state, initial));
            EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, checkpoint.mutation_sequence);
            EXPECT_EQ(memory.sideEffectCheckpoint().mmio_generation, checkpoint.mmio_generation);
        }
        EXPECT_EQ(port.reads, 0U); EXPECT_EQ(port.writes, 0U);
    }
}

TEST(NativeRegionTest, StackRelativeScalarAccessUsesTheActiveBankRatherThanTheR13Mirror) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    for (const auto word : {0x9a00U,0x9200U}) {
        const auto bytes = code(std::array<std::uint16_t, 2>{static_cast<std::uint16_t>(word),0xbe00U});
        const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr);
        fil::mem::MemoryBus memory, oracle;
        for (auto* bus : {&memory, &oracle}) {
            ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "stack-relative")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
            ASSERT_TRUE(bus->mapRam(ram, 64U, "stack"));
            bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
        }
        CortexM4 reference(oracle);
        for (const auto mode : {0U,1U,2U}) {
            for (auto* bus : {&memory, &oracle}) {
                ASSERT_TRUE(bus->write32(ram + 16U, 0x7fffffffU));
                ASSERT_TRUE(bus->write32(ram + 40U, 0x80ffffffU));
            }
            CpuState state; state.r[15] = pc; state.r[2] = 0x12345680U;
            state.r[13] = ram + 4U; state.msp = ram + 16U; state.psp = ram + 40U;
            state.control = mode == 0U ? 0U : 2U;
            state.xpsr |= xpsr_c | xpsr_q | (mode == 2U ? 3U : 0U);
            reference.state() = state;
            const auto expected = reference.stepFast(); ASSERT_EQ(expected.reason, StopReason::step_complete);
            NativeRegionContext context; context.cpu_state = &state;
            context.instruction_limit = 1U; context.cycle_limit = UINT64_MAX;
            ASSERT_TRUE(memory.directReadWindow(ram, context.ram_read));
            ASSERT_TRUE(memory.directWriteWindow(ram, context.ram_write));
            function(&context);
            EXPECT_EQ(context.retired_count, 1U); EXPECT_EQ(context.retired_cycles, expected.cycles);
            EXPECT_TRUE(bitwiseEqual(state, reference.state()));
            EXPECT_EQ(memory.read32(ram + 16U).value(), oracle.read32(ram + 16U).value());
            EXPECT_EQ(memory.read32(ram + 40U).value(), oracle.read32(ram + 40U).value());
            EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
        }
    }
}

TEST(NativeRegionTest, LiteralLoadsReadLiveBackingThroughEitherBinding) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t, 4>{0x4800U,0xbe00U,0xcdefU,0x89abU});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr);
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRom(pc, bytes.size(), "literal")); ASSERT_TRUE(memory.loadBytes(pc, bytes));
    memory.setReadFootprintTracking(false);
    CortexM4 reference(memory);
    for (const bool secondary : {false, true}) {
        CpuState state; state.r[15] = pc; state.xpsr |= xpsr_c | xpsr_v;
        reference.state() = state;
        const auto expected = reference.stepFast(); ASSERT_EQ(expected.reason, StopReason::step_complete);
        NativeRegionContext context; context.cpu_state = &state;
        context.instruction_limit = 1U; context.cycle_limit = UINT64_MAX;
        ASSERT_TRUE(memory.directReadWindow(pc, secondary ? context.ram_read : context.code_read));
        function(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state()));
        EXPECT_EQ(state.r[0], 0x89abcdefU);
        EXPECT_EQ(context.retired_count, 1U); EXPECT_EQ(context.retired_cycles, expected.cycles);
    }
}

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
