#include "fil/cpu/forward_execution.hpp"
#include "fil/mem/memory_bus.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>
#include <vector>

namespace {

constexpr std::uint32_t flash_base = 0x08000000U;
constexpr std::uint32_t ram_base = 0x20000000U;

std::vector<std::uint8_t> halfwords(const std::initializer_list<std::uint16_t> words) {
    std::vector<std::uint8_t> bytes;
    for (const auto word : words) {
        bytes.push_back(static_cast<std::uint8_t>(word));
        bytes.push_back(static_cast<std::uint8_t>(word >> 8U));
    }
    return bytes;
}

struct Fixture {
    fil::mem::MemoryBus bus;
    fil::cpu::CortexM4 cpu{bus};

    explicit Fixture(const std::initializer_list<std::uint16_t> code) {
        EXPECT_TRUE(bus.mapRam(flash_base, 256U, "forward-code", true).hasValue());
        EXPECT_TRUE(bus.mapRam(ram_base, 256U, "forward-ram").hasValue());
        const auto bytes = halfwords(code);
        for (std::size_t i = 0; i < bytes.size(); ++i) {
            EXPECT_TRUE(bus.write8(flash_base + static_cast<std::uint32_t>(i), bytes[i],
                {fil::mem::AccessType::data_write, 0U}).hasValue());
        }
        cpu.state() = fil::cpu::CpuState{};
        cpu.state().msp = ram_base + 0xf0U;
        cpu.state().r[13] = cpu.state().msp;
    }

    void restart() {
        cpu.state() = fil::cpu::CpuState{};
        cpu.state().msp = ram_base + 0xf0U;
        cpu.state().r[13] = cpu.state().msp;
        cpu.state().r[15] = flash_base;
        cpu.state().instruction_address = flash_base;
    }

    void warm() {
        restart();
        EXPECT_EQ(cpu.stepFast().reason, fil::cpu::StopReason::step_complete);
        restart();
    }
};

std::uint64_t elapsedNs(void* context, const std::uint64_t cycles) {
    const auto* hz = static_cast<const std::uint64_t*>(context);
    return cycles * 1'000'000'000ULL / *hz;
}

TEST(ForwardExecutionTest, RetiresOnePredictableInstructionAndPublishesReadyTime) {
    Fixture fixture({0x2001U, 0x3001U}); // MOVS r0,#1 ; ADDS r0,#1
    fixture.warm();
    std::uint64_t hz = 170'000'000U;
    fil::cpu::ForwardExecutionLimit limit;
    limit.instruction_credit = 1U;
    limit.cycle_credit = 4U;
    limit.elapsedNs = elapsedNs;
    limit.elapsed_context = &hz;
    const auto exit = fil::cpu::forwardCheckedStep(fixture.cpu, 20U, limit);
    ASSERT_EQ(exit.reason, fil::cpu::ForwardExitReason::completed);
    EXPECT_FALSE(exit.declined_before_effect);
    EXPECT_EQ(exit.completed_instructions, 1U);
    EXPECT_EQ(exit.completed_cycles, 1U);
    EXPECT_EQ(exit.next_pc, flash_base + 2U);
    EXPECT_EQ(fixture.cpu.state().r[0], 1U);
    EXPECT_EQ(exit.ready_time_ns, 20U + 1'000'000'000ULL / hz);
}

TEST(ForwardExecutionTest, DeclinesBeforeEffectWhenCreditOrHorizonIsExhausted) {
    Fixture fixture({0x2001U, 0x3001U});
    fixture.warm();
    const auto before = fixture.cpu.state();
    fil::cpu::ForwardExecutionLimit limit;
    limit.instruction_credit = 0U;
    limit.cycle_credit = 4U;
    auto exit = fil::cpu::forwardCheckedStep(fixture.cpu, 0U, limit);
    EXPECT_EQ(exit.reason, fil::cpu::ForwardExitReason::instruction_limit);
    EXPECT_TRUE(exit.declined_before_effect);
    limit.instruction_credit = 1U;
    limit.cycle_credit = 0U;
    exit = fil::cpu::forwardCheckedStep(fixture.cpu, 0U, limit);
    EXPECT_EQ(exit.reason, fil::cpu::ForwardExitReason::cycle_limit);
    EXPECT_TRUE(exit.declined_before_effect);
    limit.cycle_credit = 4U;
    limit.deadline_ns = 0U;
    exit = fil::cpu::forwardCheckedStep(fixture.cpu, 0U, limit);
    EXPECT_EQ(exit.reason, fil::cpu::ForwardExitReason::deadline);
    limit.deadline_ns.reset();
    limit.shared_frontier_ns = 0U;
    exit = fil::cpu::forwardCheckedStep(fixture.cpu, 0U, limit);
    EXPECT_EQ(exit.reason, fil::cpu::ForwardExitReason::shared_frontier);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(fixture.cpu.state(), before));
}

TEST(ForwardExecutionTest, UnpredictableCostStaysOnCheckedExecution) {
    // TBB is refused by peekPredictableCycles: its target depends on loaded
    // table bytes, so the forward admission must decline without an effect.
    Fixture fixture({0xE8D0U, 0xF001U, 0x3001U});
    const std::uint32_t table = ram_base + 0x20U;
    ASSERT_TRUE(fixture.bus.write8(table, 0U,
        {fil::mem::AccessType::data_write, 0U}).hasValue());
    fixture.cpu.state().r[0] = table;
    fixture.cpu.state().r[15] = flash_base;
    fixture.cpu.state().instruction_address = flash_base;
    ASSERT_EQ(fixture.cpu.stepFast().reason, fil::cpu::StopReason::step_complete);
    fixture.restart();
    const auto before = fixture.cpu.state();
    fil::cpu::ForwardExecutionLimit limit;
    limit.instruction_credit = 1U;
    limit.cycle_credit = 10U;
    const auto exit = fil::cpu::forwardCheckedStep(fixture.cpu, 0U, limit);
    EXPECT_EQ(exit.reason, fil::cpu::ForwardExitReason::checked_fallback);
    EXPECT_TRUE(exit.declined_before_effect);
    EXPECT_EQ(exit.completed_instructions, 0U);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(fixture.cpu.state(), before));
}

TEST(ForwardExecutionTest, DeferredCompletionKeepsOneInFlightTail) {
    Fixture fixture({0x2001U, 0x3001U});
    fixture.warm();
    fil::cpu::ForwardExecutionLimit limit;
    limit.instruction_credit = 1U;
    limit.cycle_credit = 4U;
    auto exit = fil::cpu::forwardCheckedStep(fixture.cpu, 7U, limit, true);
    ASSERT_EQ(exit.reason, fil::cpu::ForwardExitReason::issued_pending_completion);
    EXPECT_FALSE(exit.declined_before_effect);
    EXPECT_TRUE(exit.has_in_flight);
    EXPECT_EQ(exit.completed_instructions, 0U);
    EXPECT_EQ(exit.in_flight_pc, flash_base);
    EXPECT_EQ(fixture.cpu.state().r[0], 1U);
    EXPECT_TRUE(fil::cpu::publishInFlightCompletion(exit));
    EXPECT_EQ(exit.reason, fil::cpu::ForwardExitReason::completed);
    EXPECT_EQ(exit.completed_instructions, 1U);
    EXPECT_FALSE(exit.has_in_flight);
    EXPECT_FALSE(fil::cpu::publishInFlightCompletion(exit));
}

} // namespace
