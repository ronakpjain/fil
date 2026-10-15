#include "fil/cpu/cortex_m4.hpp"
#include "fil/mem/memory_bus.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string_view>
#include <vector>

namespace {

constexpr std::uint32_t code_base = 0x08000000U;
constexpr std::uint32_t ram_base = 0x20000000U;

std::vector<std::uint8_t> halfwords(const std::initializer_list<std::uint16_t> words) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(words.size() * 2U);
    for (const auto word : words) {
        bytes.push_back(static_cast<std::uint8_t>(word));
        bytes.push_back(static_cast<std::uint8_t>(word >> 8U));
    }
    return bytes;
}

fil::mem::MemoryBus makeBus() {
    fil::mem::MemoryBus bus;
    EXPECT_TRUE(bus.mapRom(code_base, 256U, "block-fast-code").hasValue());
    EXPECT_TRUE(bus.mapRam(ram_base, 256U, "block-fast-data").hasValue());
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    return bus;
}

void prepare(fil::cpu::CortexM4 &cpu, const std::uint32_t pc = code_base) {
    cpu.state() = fil::cpu::CpuState{};
    cpu.state().r[15] = pc;
    cpu.state().instruction_address = pc;
    cpu.state().msp = ram_base + 0xf0U;
    cpu.state().r[13] = cpu.state().msp;
}

bool warmReversibleBlock(fil::cpu::CortexM4 &cpu, const fil::cpu::CpuState &initial,
                         const std::size_t limit) {
    fil::cpu::CortexM4::JitBlockPreview preview{};
    bool ready = false;
    for (std::uint32_t probe = 0U; probe < 100U && !ready; ++probe) {
        cpu.state() = initial;
        ready = cpu.prepareAndPeekJitBlock(limit, preview);
    }
    if (!ready) return false;
    for (std::uint32_t use = 0U; use < 513U; ++use) {
        cpu.state() = initial;
        if (!cpu.tryStepReversibleJitBlock(limit)) return false;
    }
    cpu.state() = initial;
    return true;
}

class TestMmio final : public fil::mem::MmioDevice {
  public:
    fil::mem::MemoryResult<std::uint64_t> read(std::uint32_t, fil::mem::AccessSize,
                                               const fil::mem::AccessContext &) override {
        return std::uint64_t{0U};
    }
    fil::mem::MemoryResult<std::uint64_t> write(std::uint32_t, fil::mem::AccessSize, std::uint64_t,
                                                const fil::mem::AccessContext &) override {
        ++writes;
        return std::uint64_t{0U};
    }
    std::string_view name() const noexcept override { return "block-fast-test-mmio"; }
    fil::mem::MmioDomain domain(std::uint32_t, fil::mem::AccessSize) const noexcept override {
        return fil::mem::MmioDomain::shared;
    }
    std::uint32_t writes{0U};
};

TEST(BlockFastPathTest, ArithmeticBlockMatchesCappedReferencesAndBranchBoundary) {
    for (const bool trusted : {true, false}) {
        SCOPED_TRACE(trusted ? "trusted reversible route" : "guarded fast-only route");
        auto bus = makeBus();
        bus.setAllMmioTrapping(trusted);
        // MOVS; ADDS; SUBS; CMP; terminal B-to-self.
        ASSERT_TRUE(
            bus.loadBytes(code_base, halfwords({0x2001U, 0x3001U, 0x3801U, 0x2802U, 0xE7FEU}))
                .hasValue());
        ASSERT_TRUE(bus.write32(ram_base, 0xDEADBEEFU).hasValue());
        fil::cpu::CortexM4 cpu(bus);
        fil::cpu::CortexM4 reference(bus);
        prepare(cpu);
        prepare(reference);
        auto initial = cpu.state();
        initial.r[0] = 7U;
        cpu.state() = initial;

        constexpr std::size_t block_length = 5U;
        ASSERT_TRUE(warmReversibleBlock(cpu, initial, block_length));

        for (std::size_t cap = 1U; cap <= block_length; ++cap) {
            cpu.state() = initial;
            reference.state() = initial;
            std::array<std::uint16_t, block_length> cycles{};
            std::uint32_t sum = 0U;
            for (std::size_t i = 0U; i < cap; ++i) {
                const auto expected = reference.stepFast();
                ASSERT_EQ(expected.reason, fil::cpu::StopReason::step_complete);
                cycles[i] = expected.cycles;
                sum += expected.cycles;
            }

            const auto actual = cpu.tryStepReversibleJitBlock(cap);
            ASSERT_TRUE(actual);
            EXPECT_EQ(actual->execution.count, cap);
            EXPECT_EQ(actual->execution.result.instructions, cap);
            EXPECT_EQ(actual->execution.result.cycles, sum);
            EXPECT_EQ(actual->execution.result.instruction_address,
                      reference.state().instruction_address);
            EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
            for (std::size_t i = 0U; i < cap; ++i) {
                EXPECT_EQ(actual->execution.pcs[i], code_base + 2U * i);
                EXPECT_EQ(actual->instruction_cycles[i], cycles[i]);
            }
            EXPECT_EQ(bus.read32(ram_base).value(), 0xDEADBEEFU);
        }
        // Including B leaves the same PC as stopping just before it, but adds the
        // branch's architectural refill cycles (checked against the reference above).
        cpu.state() = initial;
        const auto before_branch = cpu.tryStepReversibleJitBlock(block_length - 1U);
        ASSERT_TRUE(before_branch);
        EXPECT_EQ(cpu.state().r[15], code_base + 8U);
        cpu.state() = initial;
        const auto through_branch = cpu.tryStepReversibleJitBlock(block_length);
        ASSERT_TRUE(through_branch);
        EXPECT_EQ(cpu.state().r[15], code_base + 8U);
    }
}

TEST(BlockFastPathTest, ArithmeticPrefixFallsBackToRamStoreSuffix) {
    auto bus = makeBus();
    ASSERT_TRUE(
        bus.loadBytes(code_base, halfwords({0x2005U, 0x3002U, 0xBF00U, 0xBF00U, 0x6008U, 0xBF00U}))
            .hasValue());
    auto oracle_bus = makeBus();
    ASSERT_TRUE(
        oracle_bus
            .loadBytes(code_base, halfwords({0x2005U, 0x3002U, 0xBF00U, 0xBF00U, 0x6008U, 0xBF00U}))
            .hasValue());
    fil::cpu::CortexM4 cpu(bus);
    fil::cpu::CortexM4 reference(oracle_bus);
    prepare(cpu);
    prepare(reference);
    fil::cpu::CpuState initial = cpu.state();
    initial.r[1] = ram_base;
    cpu.state() = initial;
    ASSERT_TRUE(warmReversibleBlock(cpu, initial, 6U));
    ASSERT_TRUE(bus.write32(ram_base, 0xA5A5A5A5U).hasValue());

    cpu.state() = initial;
    reference.state() = initial;
    std::array<std::uint16_t, 6U> expected_cycles{};
    std::uint32_t total_cycles = 0U;
    for (auto &cycles : expected_cycles) {
        const auto expected = reference.stepFast();
        ASSERT_EQ(expected.reason, fil::cpu::StopReason::step_complete);
        cycles = expected.cycles;
        total_cycles += cycles;
    }
    const auto before = cpu.jitStats();
    const auto actual = cpu.tryStepReversibleJitBlock(6U);
    ASSERT_TRUE(actual);
    ASSERT_EQ(actual->execution.count, 6U);
    EXPECT_EQ(actual->execution.result.instructions, 6U);
    for (std::size_t i = 0U; i < actual->execution.count; ++i) {
        EXPECT_EQ(actual->execution.pcs[i], code_base + 2U * i);
        EXPECT_EQ(actual->execution.sizes[i], 2U);
        EXPECT_EQ(actual->instruction_cycles[i], expected_cycles[i]);
    }
    EXPECT_EQ(actual->execution.result.instruction_size, 2U);
    EXPECT_EQ(actual->execution.result.raw, 0xBF00U);
    EXPECT_EQ(actual->execution.result.cycles, total_cycles);
    EXPECT_FALSE(actual->execution.memory_free);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
    EXPECT_EQ(bus.read32(ram_base).value(), 7U);
    EXPECT_EQ(cpu.jitStats().block_instructions - before.block_instructions, 6U);
}

TEST(BlockFastPathTest, UnsupportedAndMmioFirstInstructionsRejectReversibleExecution) {
    for (const auto raw : std::array<std::uint16_t, 3>{0x4348U, 0xBF30U, 0x6008U}) {
        // MUL is a cached fast handler but not micro-JIT-supported; WFI is a
        // non-reversible system operation; the store targets trapped MMIO.
        auto bus = makeBus();
        TestMmio device;
        if (raw == 0x6008U)
            ASSERT_TRUE(bus.mapMmio(0x40000000U, 16U, device, "block-fast-mmio").hasValue());
        ASSERT_TRUE(bus.loadBytes(code_base, halfwords({raw, 0x2001U, 0xBF00U})).hasValue());
        fil::cpu::CortexM4 cpu(bus);
        fil::cpu::CortexM4 reference(bus);
        prepare(cpu);
        cpu.state().r[1] = 0x40000000U;
        const auto initial = cpu.state();
        bool executed_cached = false;
        for (std::uint32_t use = 0U; use < 600U; ++use) {
            cpu.state() = initial;
            const auto outcome = cpu.tryStepReversibleJitBlock(3U);
            if (raw == 0x4348U && outcome) {
                executed_cached = true;
                reference.state() = initial;
                for (std::uint8_t i = 0U; i < outcome->execution.count; ++i)
                    ASSERT_EQ(reference.stepFast().reason, fil::cpu::StopReason::step_complete);
                EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
            } else {
                EXPECT_FALSE(outcome);
                EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), initial));
            }
        }
        EXPECT_EQ(executed_cached, raw == 0x4348U);
        EXPECT_EQ(device.writes, 0U);
    }
}

struct FetchAttempts {
    mutable std::array<std::uint32_t, 32U> pcs{};
    mutable std::size_t count{0U};
};

std::uint16_t fetchStall(const void *context, const std::uint32_t pc, const bool sequential) {
    const auto &attempts = *static_cast<const FetchAttempts *>(context);
    if (attempts.count < attempts.pcs.size())
        attempts.pcs[attempts.count] = pc;
    ++attempts.count;
    return sequential ? 1U : 2U;
}

TEST(BlockFastPathTest, BudgetedBlockMatchesCachedCutsAndFetchAccounting) {
    auto bus = makeBus();
    ASSERT_TRUE(bus.loadBytes(code_base, halfwords({0x2001U, 0x3001U, 0x3801U, 0x2802U, 0xE7FEU}))
                    .hasValue());
    fil::cpu::CortexM4 cpu(bus);
    fil::cpu::CortexM4 reference(bus);
    prepare(cpu);
    const auto initial = cpu.state();
    ASSERT_TRUE(warmReversibleBlock(cpu, initial, 5U));
    fil::cpu::CortexM4::JitBlockPreview preview{};
    for (std::uint32_t i = 0U; i < 100U; ++i) {
        reference.state() = initial;
        static_cast<void>(reference.prepareAndPeekJitBlock(5U, preview));
    }
    for (std::uint64_t credit = 0U; credit <= 16U; ++credit) {
        SCOPED_TRACE(credit);
        cpu.state() = initial;
        reference.state() = initial;
        FetchAttempts actual_fetches{};
        FetchAttempts expected_fetches{};
        fil::cpu::CortexM4::ReversibleCycleBudget actual_budget{};
        actual_budget.remaining_cycles = credit;
        actual_budget.fetch_stall = fetchStall;
        actual_budget.context = &actual_fetches;
        auto expected_budget = actual_budget;
        expected_budget.context = &expected_fetches;
        const auto expected = reference.tryStepBudgetedReversibleJitBlock(5U, expected_budget);
        const auto actual = cpu.tryStepBudgetedReversibleJitBlock(5U, actual_budget);
        ASSERT_EQ(actual.has_value(), expected.has_value());
        EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
        EXPECT_EQ(actual_budget.remaining_cycles, expected_budget.remaining_cycles);
        EXPECT_EQ(actual_budget.have_fetch, expected_budget.have_fetch);
        EXPECT_EQ(actual_budget.fetch_end, expected_budget.fetch_end);
        EXPECT_EQ(actual_budget.total_instruction_cycles, expected_budget.total_instruction_cycles);
        EXPECT_EQ(actual_fetches.count, expected_fetches.count);
        EXPECT_EQ(actual_fetches.pcs, expected_fetches.pcs);
        if (!actual) {
            continue;
        }
        EXPECT_EQ(actual->execution.count, expected->execution.count);
        EXPECT_EQ(actual->execution.result.instructions, expected->execution.result.instructions);
        EXPECT_EQ(actual->execution.result.cycles, expected->execution.result.cycles);
        EXPECT_EQ(actual->instruction_cycles, expected->instruction_cycles);
        EXPECT_EQ(actual->execution.pcs, expected->execution.pcs);
        EXPECT_EQ(actual->execution.sizes, expected->execution.sizes);
    }
}

} // namespace
