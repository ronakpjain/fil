#include "fil/cpu/cortex_m4.hpp"
#include "fil/cpu/decoder.hpp"
#include "fil/cpu/jit.hpp"
#include "fil/mem/memory_bus.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <string_view>
#include <vector>

namespace {

constexpr std::uint32_t flash_base = 0x08000000U;
constexpr std::uint32_t ram_base = 0x20000000U;

[[nodiscard]] std::vector<std::uint8_t> halfwords(
    const std::initializer_list<std::uint16_t> words) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(words.size() * 2U);
    for (const std::uint16_t word : words) {
        bytes.push_back(static_cast<std::uint8_t>(word));
        bytes.push_back(static_cast<std::uint8_t>(word >> 8U));
    }
    return bytes;
}

fil::mem::MemoryBus basicBus() {
    fil::mem::MemoryBus bus;
    EXPECT_TRUE(bus.mapRom(flash_base, 256U, "jit-test-flash").hasValue());
    EXPECT_TRUE(bus.mapRam(ram_base, 256U, "jit-test-ram").hasValue());
    return bus;
}

void prepare(fil::cpu::CortexM4& cpu, const std::uint32_t pc = flash_base) {
    cpu.state() = fil::cpu::CpuState{};
    cpu.state().r[15] = pc;
    cpu.state().instruction_address = pc;
    cpu.state().msp = ram_base + 0xf0U;
    cpu.state().r[13] = cpu.state().msp;
}

struct FakeFetchStalls {
    std::uint16_t nonsequential{0U};
    std::uint16_t sequential{0U};
};

struct FetchCallbackLog {
    const fil::cpu::CortexM4::ReversibleCycleBudget* budget{nullptr};
    std::array<std::uint32_t, 16U> pcs{};
    std::array<bool, 16U> sequential{};
    std::array<std::uint64_t, 16U> remaining{};
    std::array<std::uint32_t, 16U> fetch_end{};
    std::array<bool, 16U> have_fetch{};
    std::size_t count{0U};
};

std::uint16_t logFetchStall(const void* context, const std::uint32_t pc, const bool sequential) {
    auto& log = *static_cast<FetchCallbackLog*>(const_cast<void*>(context));
    if (log.count < log.pcs.size()) {
        log.pcs[log.count] = pc;
        log.sequential[log.count] = sequential;
        if (log.budget) {
            log.remaining[log.count] = log.budget->remaining_cycles;
            log.fetch_end[log.count] = log.budget->fetch_end;
            log.have_fetch[log.count] = log.budget->have_fetch;
        }
    }
    ++log.count;
    return sequential ? 1U : 3U;
}

std::uint16_t fakeFetchStall(
    const void* context, const std::uint32_t, const bool sequential) {
    const auto& stalls = *static_cast<const FakeFetchStalls*>(context);
    return sequential ? stalls.sequential : stalls.nonsequential;
}

class SharedMmio final : public fil::mem::MmioDevice {
public:
    fil::mem::MemoryResult<std::uint64_t> read(
        std::uint32_t, fil::mem::AccessSize,
        const fil::mem::AccessContext&
    ) override {
        ++reads;
        return std::uint64_t{0x12345678U};
    }
    fil::mem::MemoryResult<std::uint64_t> write(
        std::uint32_t, fil::mem::AccessSize, std::uint64_t,
        const fil::mem::AccessContext&
    ) override { return std::uint64_t{0}; }
    std::string_view name() const noexcept override { return "jit-shared-mmio"; }
    fil::mem::MmioDomain domain(
        std::uint32_t, fil::mem::AccessSize
    ) const noexcept override { return fil::mem::MmioDomain::shared; }

    std::int32_t reads{0};
};

TEST(JitTest, CachedBlocksMatchInterpreterAcrossMicroPrefixCuts) {
    for (const bool unsupported_first : {false, true}) {
        auto bus = basicBus();
        std::array<std::uint16_t, 7> code{
            0x2001U, 0x3001U, 0xBF00U, 0xBF00U, 0x4348U, 0xBF00U, 0xBE00U};
        if (unsupported_first) code[0] = 0x4348U;
        ASSERT_TRUE(bus.loadBytes(flash_base, halfwords(
            {code[0], code[1], code[2], code[3], code[4], code[5], code[6]})).hasValue());
        fil::cpu::CortexM4 cpu(bus), reference(bus);
        prepare(cpu); prepare(reference);
        cpu.state().r[1] = reference.state().r[1] = 7U;
        const auto initial = cpu.state();
        for (unsigned i = 0; i < 100U; ++i) {
            cpu.state() = initial;
            static_cast<void>(cpu.tryStepJitBlock(6U));
        }
        for (std::size_t cap = 1; cap <= 6U; ++cap) {
            cpu.state() = reference.state() = initial;
            std::uint32_t cycles = 0U;
            for (std::size_t i = 0; i < cap; ++i) {
                const auto step = reference.stepFast();
                ASSERT_EQ(step.reason, fil::cpu::StopReason::step_complete);
                cycles += step.cycles;
            }
            const auto block = cpu.tryStepJitBlock(cap);
            ASSERT_TRUE(block);
            EXPECT_EQ(block->count, cap);
            EXPECT_EQ(block->result.cycles, cycles);
            EXPECT_EQ(block->result.instruction_address, reference.state().instruction_address);
            EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
            for (std::size_t i = 0; i < cap; ++i)
                EXPECT_EQ(block->pcs[i], flash_base + 2U * i);
        }
    }
}

TEST(JitTest, CachedBranchMatchesInterpreterCyclesAndNextFetchFault) {
    constexpr std::uint32_t ram = 0x20000000U;
    fil::mem::MemoryBus bus, oracle;
    ASSERT_TRUE(bus.mapRam(ram, 256U, "jit-branch", true).hasValue());
    ASSERT_TRUE(oracle.mapRam(ram, 256U, "jit-branch-oracle", true).hasValue());
    const std::array<std::uint8_t, 4> code{0x00U, 0xBFU, 0x7DU, 0xE0U};
    ASSERT_TRUE(bus.loadBytes(ram, code).hasValue());
    ASSERT_TRUE(oracle.loadBytes(ram, code).hasValue());
    fil::cpu::CortexM4 cpu(bus), reference(oracle);
    prepare(cpu, ram); prepare(reference, ram);
    const auto initial = cpu.state();
    for (unsigned i = 0; i < 100U; ++i) {
        cpu.state() = initial;
        static_cast<void>(cpu.tryStepJitBlock(2U));
    }
    cpu.state() = reference.state() = initial;
    const auto expected0 = reference.stepFast(), expected1 = reference.stepFast();
    const auto block = cpu.tryStepJitBlock(2U);
    ASSERT_TRUE(block);
    EXPECT_EQ(block->result.cycles, expected0.cycles + expected1.cycles);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
    const auto expected_fault = reference.stepFast();
    const auto actual_fault = cpu.stepJitFast();
    EXPECT_EQ(actual_fault.reason, expected_fault.reason);
    EXPECT_EQ(actual_fault.cycles, expected_fault.cycles);
    EXPECT_EQ(cpu.lastDiagnostic().message, reference.lastDiagnostic().message);
}

TEST(JitTest, DecliningStandaloneReversibleBranchRestoresEntireEntryState) {
    auto bus = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0x4700U})));
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    cpu.state().r[0] = flash_base + 8U; // BX to ARM state must decline.
    cpu.state().instruction_address = flash_base - 2U;
    for (std::uint32_t i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock(true));
    ASSERT_TRUE(cpu.prepareJitBlock(true));
    const auto initial = cpu.state();
    // Exercise the non-trusted fast-only path, not the RAM guard's executor.
    ASSERT_FALSE(bus.reversibleRamOnly());
    EXPECT_FALSE(cpu.tryStepReversibleJitBlock(1U));
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), initial));
}

TEST(JitTest, ReversibleSpanLinksStandaloneCallWithoutChangingOrdinaryAdmission) {
    auto bus = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0xf000U, 0xf802U, 0xbf00U, 0xe7feU, 0xbf00U})));
    fil::cpu::CortexM4 cpu(bus);
    fil::cpu::CortexM4 reference(bus);
    prepare(cpu);
    prepare(reference);
    for (std::uint32_t i = 0U; i < 70U; ++i) EXPECT_FALSE(cpu.prepareJitBlock());
    ASSERT_TRUE(cpu.prepareJitBlock(true));
    EXPECT_FALSE(cpu.prepareJitBlock()) << "ordinary compilation still rejects one-op blocks";
    const auto expected = reference.stepFast();
    ASSERT_EQ(expected.instructions, 1U);
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    FakeFetchStalls stalls{3U, 0U};
    fil::cpu::CortexM4::ReversibleCycleBudget budget;
    budget.context = &stalls;
    budget.fetch_stall = fakeFetchStall;
    budget.remaining_cycles = expected.cycles + stalls.nonsequential - 1U;
    const auto initial = cpu.state();
    EXPECT_FALSE(cpu.tryStepBudgetedReversibleJitBlock(1U, budget));
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), initial));
    budget.remaining_cycles = expected.cycles + stalls.nonsequential;
    const auto result = cpu.tryStepBudgetedReversibleJitBlock(1U, budget);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->execution.count, 1U);
    EXPECT_EQ(result->execution.result.cycles, expected.cycles);
    EXPECT_EQ(budget.total_instruction_cycles[0], expected.cycles + stalls.nonsequential);
    EXPECT_EQ(budget.remaining_cycles, 0U);
    EXPECT_EQ(budget.fetch_end, flash_base + 4U);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
}

TEST(JitTest, ReversibleTimedBlockCommitsFastPrefixWithExactCosts) {
    auto bus = basicBus();
    // ADDS r0,#1; BEQ (not taken); UDIV r2,r5,r6; STR r0,[r1];
    // MOV pc,r0 is admitted as the final generic terminator but must not run.
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({
        0x3001U, 0xD000U, 0xFBB5U, 0xF2F6U, 0x6008U, 0x4687U
    })).hasValue());
    fil::cpu::CortexM4 cpu(bus);
    fil::cpu::CortexM4 reference(bus);
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    prepare(cpu);
    prepare(reference);
    cpu.state().r[0] = reference.state().r[0] = 4U;
    cpu.state().r[1] = reference.state().r[1] = ram_base;
    cpu.state().r[5] = reference.state().r[5] = 101U;
    cpu.state().r[6] = reference.state().r[6] = 10U;
    for (std::uint32_t i = 0U; i < 70U; ++i) {
        static_cast<void>(cpu.prepareJitBlock());
    }
    ASSERT_TRUE(cpu.jitBlockReady());

    const auto timed = cpu.tryStepReversibleJitBlock();
    ASSERT_TRUE(timed);
    ASSERT_EQ(timed->execution.count, 4U) << "pc=0x" << std::hex
        << cpu.state().r[15] << " reason="
        << fil::cpu::stopReasonName(timed->execution.result.reason);
    EXPECT_EQ(timed->execution.result.instructions, 4U);
    EXPECT_EQ(cpu.state().r[15], flash_base + 10U)
        << "generic MOV-to-PC was not executed";
    EXPECT_EQ(cpu.state().r[0], 5U);
    EXPECT_EQ(cpu.state().r[2], 10U);
    const auto stored = bus.read32(ram_base);
    ASSERT_TRUE(stored);
    EXPECT_EQ(stored.value(), 5U);

    std::uint32_t sum = 0U;
    for (std::uint8_t i = 0U; i < timed->execution.count; ++i) {
        const auto expected = reference.stepFast();
        ASSERT_EQ(expected.reason, fil::cpu::StopReason::step_complete);
        EXPECT_EQ(timed->execution.pcs[i], expected.instruction_address);
        EXPECT_EQ(timed->instruction_cycles[i], expected.cycles);
        sum += timed->instruction_cycles[i];
    }
    EXPECT_EQ(timed->execution.result.cycles, sum);
    EXPECT_EQ(timed->instruction_cycles[1], 1U)
        << "failed conditional branch retires for one cycle";
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
}

TEST(JitTest, ReversibleTimedBlockTimesSignedDivideExactly) {
    auto bus = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base,
        halfwords({0xFB93U, 0xF1F2U, 0xBF00U})).hasValue()); // SDIV r1,r3,r2; NOP
    fil::cpu::CortexM4 cpu(bus);
    fil::cpu::CortexM4 reference(bus);
    prepare(cpu);
    prepare(reference);
    cpu.state().r[3] = reference.state().r[3]
        = std::bit_cast<std::uint32_t>(-10);
    cpu.state().r[2] = reference.state().r[2] = 3U;
    for (std::uint32_t i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
    const auto timed = cpu.tryStepReversibleJitBlock();
    ASSERT_TRUE(timed);
    ASSERT_EQ(timed->execution.count, 2U);
    const auto divided = reference.stepFast();
    ASSERT_EQ(divided.reason, fil::cpu::StopReason::step_complete);
    EXPECT_EQ(timed->instruction_cycles[0], divided.cycles);
    EXPECT_EQ(cpu.state().r[1], std::bit_cast<std::uint32_t>(-3));
    EXPECT_EQ(timed->instruction_cycles[1], reference.stepFast().cycles);
    EXPECT_EQ(timed->execution.result.cycles,
        static_cast<std::uint16_t>(timed->instruction_cycles[0]
            + timed->instruction_cycles[1]));
}

TEST(JitTest, ReversibleTimedBlockStopsAtBranchWithRefillCost) {
    auto bus = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0xBF00U, 0xE000U, 0x2009U})).hasValue());
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    fil::cpu::CortexM4 cpu(bus);
    fil::cpu::CortexM4 reference(bus);
    prepare(cpu);
    prepare(reference);
    for (std::uint32_t i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
    const auto timed = cpu.tryStepReversibleJitBlock();
    ASSERT_TRUE(timed);
    ASSERT_EQ(timed->execution.count, 2U);
    std::uint32_t sum = 0U;
    for (std::uint8_t i = 0U; i < timed->execution.count; ++i) {
        const auto expected = reference.stepFast();
        ASSERT_EQ(expected.reason, fil::cpu::StopReason::step_complete);
        EXPECT_EQ(timed->execution.pcs[i], expected.instruction_address);
        EXPECT_EQ(timed->instruction_cycles[i], expected.cycles);
        sum += timed->instruction_cycles[i];
    }
    EXPECT_EQ(timed->execution.result.cycles, sum);
    EXPECT_EQ(timed->execution.result.cycles, 5U);
    EXPECT_EQ(cpu.state().r[15], flash_base + 6U);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
}

TEST(JitTest, ReversibleTimedBlockReturnsCommittedPrefixOnDecline) {
    constexpr std::uint32_t mmio_base = 0x40000000U;
    auto bus = basicBus();
    SharedMmio device;
    ASSERT_TRUE(bus.mapMmio(mmio_base, 16U, device, "capsule-mmio").hasValue());
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0x2001U, 0x6008U, 0xBF00U})).hasValue());
    bus.setSharedMmioTrapping(true);
    bus.setAllMmioTrapping(true);
    bus.setReversibleRamOnly(true);
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    cpu.state().r[1] = mmio_base;
    for (std::uint32_t i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
    const auto timed = cpu.tryStepReversibleJitBlock();
    ASSERT_TRUE(timed);
    ASSERT_EQ(timed->execution.count, 1U);
    EXPECT_TRUE(timed->execution.memory_free)
        << "declined MMIO instruction is not part of the committed prefix";
    EXPECT_EQ(timed->instruction_cycles[0], 1U);
    EXPECT_EQ(timed->execution.result.cycles, 1U);
    EXPECT_EQ(cpu.state().r[15], flash_base + 2U)
        << "declined MMIO store remains the next instruction";
    EXPECT_EQ(device.reads, 0);

    // If the first instruction declines, no empty-prefix outcome is returned
    // and no guest state is changed.
    auto fault_bus = basicBus();
    ASSERT_TRUE(fault_bus.loadBytes(flash_base, halfwords({0x6808U, 0xBF00U})).hasValue());
    fil::cpu::CortexM4 fault_cpu(fault_bus);
    prepare(fault_cpu);
    fault_cpu.state().r[1] = 0x50000000U;
    for (std::uint32_t i = 0U; i < 70U; ++i) static_cast<void>(fault_cpu.prepareJitBlock());
    const auto fault_before = fault_cpu.state();
    EXPECT_FALSE(fault_cpu.tryStepReversibleJitBlock().has_value());
    EXPECT_TRUE(fil::cpu::bitwiseEqual(fault_cpu.state(), fault_before));

    auto executable_bus = basicBus();
    ASSERT_TRUE(executable_bus.mapRam(
        ram_base + 0x1000U, 0x100U, "capsule-executable-ram", true).hasValue());
    ASSERT_TRUE(executable_bus.loadBytes(
        flash_base, halfwords({0x6008U, 0xBF00U})).hasValue());
    executable_bus.setAllMmioTrapping(true);
    executable_bus.setReversibleRamOnly(true);
    fil::cpu::CortexM4 executable_cpu(executable_bus);
    prepare(executable_cpu);
    executable_cpu.state().r[0] = 0x2001U;
    executable_cpu.state().r[1] = ram_base + 0x1000U;
    for (std::uint32_t i = 0U; i < 70U; ++i) {
        static_cast<void>(executable_cpu.prepareJitBlock());
    }
    const auto executable_before = executable_cpu.state();
    EXPECT_FALSE(executable_cpu.tryStepReversibleJitBlock().has_value());
    EXPECT_TRUE(fil::cpu::bitwiseEqual(executable_cpu.state(), executable_before));
    const auto unchanged = executable_bus.read32(ram_base + 0x1000U);
    ASSERT_TRUE(unchanged);
    EXPECT_EQ(unchanged.value(), 0U)
        << "reversible store restriction refuses executable RAM writes";
}

TEST(JitTest, FixedFetchTimingMatchesCallbackForEveryCycleBudget) {
    for (std::uint64_t credit = 0U; credit <= 18U; ++credit) {
        SCOPED_TRACE(credit);
        auto left = basicBus();
        auto right = basicBus();
        const auto code = halfwords({0xF04FU, 0x0001U, 0xF100U, 0x0002U, 0xBF00U});
        ASSERT_TRUE(left.loadBytes(flash_base, code));
        ASSERT_TRUE(right.loadBytes(flash_base, code));
        left.setReversibleRamOnly(true); left.setAllMmioTrapping(true);
        right.setReversibleRamOnly(true); right.setAllMmioTrapping(true);
        fil::cpu::CortexM4 snapshot(left), callback(right);
        prepare(snapshot); prepare(callback);
        for (unsigned i = 0U; i < 70U; ++i) {
            static_cast<void>(snapshot.prepareJitBlock(true));
            static_cast<void>(callback.prepareJitBlock(true));
        }
        fil::cpu::CortexM4::ReversibleCycleBudget a, b;
        a.remaining_cycles = b.remaining_cycles = credit;
        a.fetch_timing = {flash_base, flash_base + 0x10000U, 0U, 0U, 4U, 0U};
        b.fetch_stall = +[](const void*, std::uint32_t pc, bool sequential) -> std::uint16_t {
            return !sequential && pc >= flash_base && pc < flash_base + 0x10000U ? 4U : 0U;
        };
        const auto x = snapshot.tryStepBudgetedReversibleJitBlock(3U, a);
        const auto y = callback.tryStepBudgetedReversibleJitBlock(3U, b);
        ASSERT_EQ(x.has_value(), y.has_value());
        if (x) {
            EXPECT_EQ(x->execution.count, y->execution.count);
            EXPECT_EQ(x->execution.result.cycles, y->execution.result.cycles);
            for (std::size_t i = 0U; i < x->execution.count; ++i) {
                EXPECT_EQ(x->instruction_cycles[i], y->instruction_cycles[i]);
                EXPECT_EQ(a.total_instruction_cycles[i], b.total_instruction_cycles[i]);
            }
        }
        EXPECT_TRUE(fil::cpu::bitwiseEqual(snapshot.state(), callback.state()));
        EXPECT_EQ(a.remaining_cycles, b.remaining_cycles);
        EXPECT_EQ(a.have_fetch, b.have_fetch);
        EXPECT_EQ(a.fetch_end, b.fetch_end);
    }
    fil::cpu::CortexM4::ReversibleCycleBudget override;
    override.fetch_timing.nonsequential_stall = 15U;
    override.fetch_stall = +[](const void*, std::uint32_t, bool) -> std::uint16_t { return 7U; };
    EXPECT_EQ(override.fetchStallCycles(flash_base, false), 7U);
}

TEST(JitTest, BudgetedReversiblePreparedOpsMatchInterpreterAndExactFetchCosts) {
    auto bus = basicBus();
    auto oracle = basicBus();
    const auto code = halfwords({
        0x4600U,             // mov r0,r0 (no flags, destination/source alias)
        0xF04FU, 0x0207U,    // mov.w r2,#7 (no flags)
        0xF103U, 0x0301U,    // add.w r3,r3,#1 (destination/source alias)
        0xF1A5U, 0x0501U,    // sub.w r5,r5,#1 (destination/source alias)
        0x684EU,             // ldr r6,[r1,#4]
        0x604EU,             // str r6,[r1,#4]
        0xBF00U});
    ASSERT_TRUE(bus.loadBytes(flash_base, code).hasValue());
    ASSERT_TRUE(oracle.loadBytes(flash_base, code).hasValue());
    bus.setReversibleRamOnly(true);
    oracle.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    oracle.setAllMmioTrapping(true);
    fil::cpu::CortexM4 cpu(bus), reference(oracle);
    prepare(cpu); prepare(reference);
    // The prepared word handlers must retain the bus's supported unaligned
    // access behavior as well as ordinary aligned RAM semantics.
    cpu.state().r[1] = reference.state().r[1] = ram_base + 0x41U;
    cpu.state().r[3] = reference.state().r[3] = 99U;
    cpu.state().r[5] = reference.state().r[5] = 101U;
    ASSERT_TRUE(bus.write32(ram_base + 0x45U, 0x12345678U,
        {fil::mem::AccessType::data_write, flash_base}));
    ASSERT_TRUE(oracle.write32(ram_base + 0x45U, 0x12345678U,
        {fil::mem::AccessType::data_write, flash_base}));
    for (unsigned i = 0U; i < 70U; ++i) {
        static_cast<void>(cpu.prepareJitBlock(true));
    }

    FakeFetchStalls stalls{3U, 1U};
    fil::cpu::CortexM4::ReversibleCycleBudget budget{};
    budget.remaining_cycles = 100U;
    budget.fetch_stall = fakeFetchStall;
    budget.context = &stalls;
    const auto result = cpu.tryStepBudgetedReversibleJitBlock(6U, budget);
    ASSERT_TRUE(result);
    ASSERT_EQ(result->execution.count, 6U);
    EXPECT_EQ(cpu.state().r[2], 7U);
    std::uint32_t expected_pipeline_cycles = 0U;
    std::uint32_t expected_charged_cycles = 0U;
    std::uint32_t fetch_end = 0U;
    for (std::size_t i = 0U; i < result->execution.count; ++i) {
        const auto step = reference.stepFast();
        ASSERT_EQ(step.reason, fil::cpu::StopReason::step_complete);
        expected_pipeline_cycles += step.cycles;
        const bool sequential = i != 0U;
        const auto stall = fakeFetchStall(&stalls, step.instruction_address, sequential);
        expected_charged_cycles += step.cycles + stall;
        EXPECT_EQ(result->instruction_cycles[i], step.cycles);
        EXPECT_EQ(budget.total_instruction_cycles[i], step.cycles + stall);
        fetch_end = step.instruction_address + step.instruction_size;
    }
    EXPECT_EQ(result->execution.result.cycles, expected_pipeline_cycles);
    EXPECT_EQ(budget.remaining_cycles, 100U - expected_charged_cycles);
    EXPECT_EQ(budget.fetch_end, fetch_end);
    EXPECT_TRUE(budget.have_fetch);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
    EXPECT_EQ(bus.read32(ram_base + 0x45U).value(),
              oracle.read32(ram_base + 0x45U).value());
}

TEST(JitTest, ReversibleFinalMetadataMatchesEveryRetiredPrefixAndMemoryDecline) {
    for (const bool decline_memory : {false, true}) {
        for (std::size_t limit = 1U; limit <= 6U; ++limit) {
            SCOPED_TRACE(decline_memory);
            SCOPED_TRACE(limit);
            auto bus = basicBus();
            auto oracle = basicBus();
            const auto code = halfwords({0xF04FU, 0x0001U, 0xBF00U, 0x2102U,
                0x681AU, 0xF101U, 0x0103U, 0x6059U, 0xBF00U});
            ASSERT_TRUE(bus.loadBytes(flash_base, code));
            ASSERT_TRUE(oracle.loadBytes(flash_base, code));
            bus.setReversibleRamOnly(true);
            bus.setAllMmioTrapping(true);
            fil::cpu::CortexM4 cpu(bus), reference(oracle);
            prepare(cpu); prepare(reference);
            cpu.state().r[3] = reference.state().r[3] = decline_memory ? 0x40000000U : ram_base;
            for (unsigned warm = 0U; warm < 70U; ++warm)
                static_cast<void>(cpu.prepareJitBlock(true));
            fil::cpu::CortexM4::ReversibleCycleBudget budget;
            budget.remaining_cycles = 100U;
            const auto result = cpu.tryStepBudgetedReversibleJitBlock(limit, budget);
            ASSERT_TRUE(result);
            const auto expected_count = decline_memory ? std::min(limit, std::size_t{3U}) : limit;
            ASSERT_EQ(result->execution.count, expected_count);
            fil::cpu::FastStepResult last;
            std::uint32_t cycles = 0U;
            for (std::size_t i = 0U; i < expected_count; ++i) {
                last = reference.stepFast();
                ASSERT_EQ(last.reason, fil::cpu::StopReason::step_complete);
                cycles += last.cycles;
            }
            EXPECT_EQ(result->execution.result.instruction_address, last.instruction_address);
            EXPECT_EQ(result->execution.result.raw, last.raw);
            EXPECT_EQ(result->execution.result.instruction_size, last.instruction_size);
            EXPECT_EQ(result->execution.result.suppress_loop_observation, last.suppress_loop_observation);
            EXPECT_EQ(result->execution.result.instructions, expected_count);
            EXPECT_EQ(result->execution.result.cycles, cycles);
            EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
        }
    }
}

TEST(JitTest, WarmedNativeTraceStitchesRealBlBxLrAndMatchesCheckedCuts) {
    if (!fil::cpu::MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter";
    // BL targets the BX LR at +12; return continues at +4 and runs successors.
    const auto words = halfwords({0xf000U, 0xf804U, 0xbf00U, 0xbf00U, 0xbf00U,
        0xbf00U, 0x4770U, 0xbf00U, 0xbf00U, 0xbf00U, 0xbf00U, 0xbf00U});
    auto bus = basicBus(), oracle = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, words));
    ASSERT_TRUE(oracle.loadBytes(flash_base, words));
    bus.setReversibleRamOnly(true); bus.setAllMmioTrapping(true);
    oracle.setReversibleRamOnly(true); oracle.setAllMmioTrapping(true);
    fil::cpu::CortexM4 candidate(bus), checked(oracle);
    prepare(candidate); prepare(checked); checked.setNativeMemoryJitEnabled(false);
    // Repeatedly execute through the checked/native block path until the real
    // BL and BX have been observed and the trace is actually entered.
    candidate.state().r[15] = flash_base;
    candidate.state().r[14] = 0U;
    for (unsigned i = 0U; i < 4000U && candidate.jitStats().trace_instructions == 0U; ++i) {
        if (candidate.state().r[15] >= flash_base + 24U) {
            candidate.state().r[15] = flash_base;
            candidate.state().r[14] = 0U;
        }
        fil::cpu::CortexM4::ReversibleCycleBudget warm; warm.remaining_cycles = 1000U;
        if (!candidate.tryStepBudgetedReversibleJitBlock(64U, warm))
            ASSERT_EQ(candidate.stepFast().reason, fil::cpu::StopReason::step_complete);
    }
    ASSERT_GT(candidate.jitStats().trace_executions, 0U)
        << "trace compilations=" << candidate.jitStats().trace_compilations
        << " state pc=" << std::hex << candidate.state().r[15]
        << " bx profile was not observed or trace preparation declined";
    ASSERT_GE(candidate.jitStats().trace_compiled_instructions, 8U);

    fil::cpu::CpuState initial; initial.r[15] = flash_base;
    initial.instruction_address = flash_base + 64U;
    const auto trace_before = candidate.jitStats().trace_instructions;
    bool observed_trace = false;
    for (std::size_t limit = 1U; limit <= 8U; ++limit)
    for (std::uint32_t credit = 0U; credit <= 20U; ++credit) {
        candidate.state() = checked.state() = initial;
        fil::cpu::CortexM4::ReversibleCycleBudget actual_budget;
        actual_budget.remaining_cycles = credit;
        actual_budget.fetch_timing = {flash_base, flash_base + 64U,
            flash_base + 64U, flash_base + 128U, 3U, 1U};
        auto expected_budget = actual_budget;
        const auto actual = candidate.tryStepBudgetedReversibleJitBlock(limit, actual_budget);
        if (!actual) {
            EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
            continue;
        }
        observed_trace |= candidate.jitStats().trace_instructions > trace_before;
        std::size_t expected_count = 0U;
        std::uint32_t expected_execution_cycles = 0U;
        for (std::size_t i = 0U; i < actual->execution.count; ++i) {
            const auto pc = checked.state().r[15];
            const auto stall = expected_budget.fetchStallCycles(pc,
                expected_budget.have_fetch && pc == expected_budget.fetch_end);
            const auto expected = checked.stepFast();
            ASSERT_EQ(expected.reason, fil::cpu::StopReason::step_complete);
            const auto charged = static_cast<std::uint32_t>(expected.cycles) + stall;
            ASSERT_LE(charged, expected_budget.remaining_cycles);
            expected_budget.remaining_cycles -= charged;
            expected_budget.have_fetch = true;
            expected_budget.fetch_end = pc + expected.instruction_size;
            ++expected_count;
            expected_execution_cycles += expected.cycles;
            EXPECT_EQ(actual->execution.pcs[i], expected.instruction_address);
            EXPECT_EQ(actual->execution.sizes[i], expected.instruction_size);
            EXPECT_EQ(actual->instruction_cycles[i], expected.cycles);
            EXPECT_EQ(actual_budget.total_instruction_cycles[i], charged);
        }
        EXPECT_EQ(actual->execution.count, expected_count);
        EXPECT_EQ(actual->execution.result.instructions, expected_count);
        EXPECT_EQ(actual->execution.result.cycles, expected_execution_cycles);
        EXPECT_EQ(actual_budget.remaining_cycles, expected_budget.remaining_cycles);
        EXPECT_EQ(actual_budget.have_fetch, expected_budget.have_fetch);
        EXPECT_EQ(actual_budget.fetch_end, expected_budget.fetch_end);
        EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
    }
    EXPECT_TRUE(observed_trace);
}

TEST(JitTest, CheckedBlxSeedsObservedEdgeAndNativeTraceExecutesTheCall) {
    if (!fil::cpu::MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter";
    constexpr std::uint32_t call_pc = flash_base;
    constexpr std::uint32_t callee = flash_base + 0x42U; // exercise target bit 1
    auto words = std::vector<std::uint16_t>(64U, 0xbf00U);
    words[0] = 0x4780U; // BLX r0
    words[0x21U + 8U] = 0x4770U; // BX LR after eight callee NOPs
    std::vector<std::uint8_t> code;
    code.reserve(words.size() * 2U);
    for (const auto word : words) {
        code.push_back(static_cast<std::uint8_t>(word));
        code.push_back(static_cast<std::uint8_t>(word >> 8U));
    }
    auto bus = basicBus(), oracle = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, code));
    ASSERT_TRUE(oracle.loadBytes(flash_base, code));
    bus.setReversibleRamOnly(true); bus.setAllMmioTrapping(true);
    oracle.setReversibleRamOnly(true); oracle.setAllMmioTrapping(true);
    fil::cpu::CortexM4 candidate(bus), checked(oracle);
    prepare(candidate); prepare(checked); checked.setNativeMemoryJitEnabled(false);
    candidate.state().r[0] = checked.state().r[0] = callee | 1U;

    // Execute both edges through the checked architectural executor first.
    auto call = candidate.stepFast();
    ASSERT_EQ(call.reason, fil::cpu::StopReason::step_complete);
    EXPECT_EQ(candidate.state().r[15], callee);
    for (unsigned i = 0U; i < 9U; ++i) {
        auto result = candidate.stepFast();
        ASSERT_EQ(result.reason, fil::cpu::StopReason::step_complete);
    }
    ASSERT_EQ(candidate.state().r[15], call_pc + 2U);

    // The successful checked BLX must have recorded its actual aligned PC edge.
    // Warm the real call-site entry (the trace cache requires 64 visits), while
    // executing the callee through checked instructions between visits.
    auto initial = candidate.state();
    initial.r[15] = call_pc;
    initial.r[14] = 0U;
    initial.r[0] = callee | 1U;
    candidate.state() = checked.state() = initial;
    for (unsigned visit = 0U; visit < 96U && candidate.jitStats().trace_executions == 0U; ++visit) {
        candidate.state().r[15] = call_pc;
        fil::cpu::CortexM4::ReversibleCycleBudget warm; warm.remaining_cycles = 1000U;
        const auto warmed = candidate.tryStepBudgetedReversibleJitBlock(64U, warm);
        if (!warmed && candidate.state().r[15] == call_pc)
            ASSERT_EQ(candidate.stepFast().reason, fil::cpu::StopReason::step_complete);
        if (candidate.state().r[15] == callee) {
            for (unsigned i = 0U; i < 9U; ++i)
                ASSERT_EQ(candidate.stepFast().reason, fil::cpu::StopReason::step_complete);
        }
    }
    ASSERT_GT(candidate.jitStats().trace_executions, 0U)
        << "checked BLX profile did not seed a caller/callee trace";
    EXPECT_GT(candidate.jitStats().trace_instructions, 0U);

    candidate.state() = checked.state() = initial;
    fil::cpu::CortexM4::ReversibleCycleBudget budget;
    budget.remaining_cycles = 1000U;
    const auto native = candidate.tryStepBudgetedReversibleJitBlock(64U, budget);
    ASSERT_TRUE(native);
    ASSERT_GT(native->execution.count, 0U);

    for (std::size_t i = 0U; i < native->execution.count; ++i) {
        const auto expected = checked.stepFast();
        ASSERT_EQ(expected.reason, fil::cpu::StopReason::step_complete);
        EXPECT_EQ(native->execution.pcs[i], expected.instruction_address);
        EXPECT_EQ(native->instruction_cycles[i], expected.cycles);
    }
    EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));

    // A changed, non-Thumb operand must leave native execution before BLX
    // retirement; the checked fallback then writes LR before reporting fault.
    auto fault_start = initial;
    fault_start.r[0] = callee; // bit 0 clear
    fault_start.r[14] = 0x12345678U;
    candidate.state() = checked.state() = fault_start;
    fil::cpu::CortexM4::ReversibleCycleBudget mismatch_budget;
    mismatch_budget.remaining_cycles = 1000U;
    const auto mismatch = candidate.tryStepBudgetedReversibleJitBlock(64U, mismatch_budget);
    EXPECT_GT(candidate.jitStats().trace_guard_mismatches, 0U);
    const auto actual_reason = mismatch ? mismatch->execution.result.reason
                                        : candidate.stepFast().reason;
    const auto expected = checked.stepFast();
    EXPECT_EQ(actual_reason, expected.reason);
    EXPECT_NE(expected.reason, fil::cpu::StopReason::step_complete);
    EXPECT_EQ(candidate.state().r[14], (call_pc + 2U) | 1U);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));

    // An EXC_RETURN-shaped odd operand is not a BLX exception return. The
    // checked fallback preserves BLX's LR-before-target-fetch behavior, then
    // both CPUs fault identically when fetching from the token-shaped address.
    auto exc_start = initial;
    exc_start.r[0] = 0xfffffff1U;
    exc_start.r[14] = 0x87654321U;
    candidate.state() = checked.state() = exc_start;
    fil::cpu::CortexM4::ReversibleCycleBudget exc_budget;
    exc_budget.remaining_cycles = 1000U;
    const auto exc_attempt = candidate.tryStepBudgetedReversibleJitBlock(64U, exc_budget);
    const auto actual_call_reason = exc_attempt ? exc_attempt->execution.result.reason
                                                : candidate.stepFast().reason;
    const auto expected_call = checked.stepFast();
    EXPECT_EQ(actual_call_reason, expected_call.reason);
    EXPECT_EQ(candidate.state().r[15], 0xfffffff0U);
    EXPECT_EQ(candidate.state().r[14], (call_pc + 2U) | 1U);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
    const auto actual_fetch = candidate.stepFast();
    const auto expected_fetch = checked.stepFast();
    EXPECT_EQ(actual_fetch.reason, expected_fetch.reason);
    EXPECT_NE(actual_fetch.reason, fil::cpu::StopReason::step_complete);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
}

TEST(JitTest, NativeTraceMatchesCheckedPrefixesBudgetsAndChangedBranchInputs) {
    if (!fil::cpu::MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter";
    const auto code = halfwords({0xbf00U, 0x6008U, 0x680aU, 0x3202U, 0x2a03U,
        0xd102U, 0x3301U, 0x604bU, 0x684cU, 0xe7f5U});
    auto bus = basicBus(), oracle = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, code));
    ASSERT_TRUE(oracle.loadBytes(flash_base, code));
    for (auto* memory : {&bus, &oracle}) {
        memory->setReversibleRamOnly(true);
        memory->setAllMmioTrapping(true);
    }
    fil::cpu::CortexM4 candidate(bus), checked(oracle);
    prepare(candidate); prepare(checked);
    checked.setNativeMemoryJitEnabled(false);
    candidate.state().r[0] = 1U;
    candidate.state().r[1] = ram_base;
    for (unsigned i = 0U; i < 4000U && candidate.jitStats().trace_instructions == 0U; ++i) {
        fil::cpu::CortexM4::ReversibleCycleBudget warm;
        warm.remaining_cycles = 1000U;
        if (!candidate.tryStepBudgetedReversibleJitBlock(64U, warm))
            ASSERT_EQ(candidate.stepFast().reason, fil::cpu::StopReason::step_complete);
    }
    ASSERT_GT(candidate.jitStats().trace_instructions, 0U);
    fil::cpu::CpuState initial;
    initial.r[15] = flash_base;
    initial.instruction_address = flash_base + 64U;
    initial.r[1] = ram_base;
    initial.r[3] = 5U;
    initial.xpsr = fil::cpu::xpsr_t | fil::cpu::xpsr_q | fil::cpu::xpsr_v;
    for (const auto input : {1U, 2U}) {
        for (std::size_t limit = 1U; limit <= 10U; ++limit) {
            for (std::uint32_t credit = 0U; credit <= 48U; ++credit) {
                for (unsigned fetch = 0U; fetch < 3U; ++fetch) {
                    SCOPED_TRACE(::testing::Message() << input << '/' << limit << '/' << credit << '/' << fetch);
                    candidate.state() = checked.state() = initial;
                    candidate.state().r[0] = checked.state().r[0] = input;
                    for (auto* memory : {&bus, &oracle}) {
                        ASSERT_TRUE(memory->write32(ram_base, 0U));
                        ASSERT_TRUE(memory->write32(ram_base + 4U, 0U));
                        memory->restoreReadFootprint({});
                    }
                    fil::cpu::CortexM4::ReversibleCycleBudget actual_budget;
                    actual_budget.remaining_cycles = credit;
                    actual_budget.have_fetch = fetch != 0U;
                    actual_budget.fetch_end = fetch == 1U ? flash_base : flash_base + 64U;
                    // Reuse the same trace across distinct complete timing snapshots;
                    // vary both stalls and the covered range to exercise cache misses.
                    actual_budget.fetch_timing = {
                        flash_base, flash_base + (credit % 2U == 0U ? 128U : 2U),
                        flash_base + 64U, flash_base + 128U,
                        static_cast<std::uint16_t>(2U + credit % 5U),
                        static_cast<std::uint16_t>(credit % 3U)};
                    auto expected_budget = actual_budget;
                    std::size_t retired = 0U;
                    while (retired < limit) {
                        const auto actual = candidate.tryStepBudgetedReversibleJitBlock(limit - retired, actual_budget);
                        if (!actual) {
                            EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
                            break; // Conservative admission may leave otherwise usable credit.
                        }
                        std::uint32_t expected_cycles = 0U;
                        for (std::size_t i = 0U; i < actual->execution.count; ++i) {
                            const auto pc = checked.state().r[15];
                            const auto stall = expected_budget.fetchStallCycles(pc,
                                expected_budget.have_fetch && pc == expected_budget.fetch_end);
                            const auto expected = checked.stepFast();
                            ASSERT_EQ(expected.reason, fil::cpu::StopReason::step_complete);
                            const auto charged = static_cast<std::uint32_t>(expected.cycles) + stall;
                            ASSERT_LE(charged, expected_budget.remaining_cycles);
                            expected_budget.remaining_cycles -= charged;
                            expected_budget.have_fetch = true;
                            expected_budget.fetch_end = pc + expected.instruction_size;
                            expected_cycles += expected.cycles;
                            EXPECT_EQ(actual->execution.pcs[i], expected.instruction_address);
                            EXPECT_EQ(actual->execution.sizes[i], expected.instruction_size);
                            EXPECT_EQ(actual->instruction_cycles[i], expected.cycles);
                            EXPECT_EQ(actual_budget.total_instruction_cycles[i], charged);
                        }
                        EXPECT_EQ(actual->execution.result.cycles, expected_cycles);
                        EXPECT_EQ(actual_budget.remaining_cycles, expected_budget.remaining_cycles);
                        EXPECT_EQ(actual_budget.have_fetch, expected_budget.have_fetch);
                        EXPECT_EQ(actual_budget.fetch_end, expected_budget.fetch_end);
                        EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
                        EXPECT_EQ(bus.readFootprint().words, oracle.readFootprint().words);
                        EXPECT_EQ(bus.read32(ram_base).value(), oracle.read32(ram_base).value());
                        EXPECT_EQ(bus.read32(ram_base + 4U).value(), oracle.read32(ram_base + 4U).value());
                        retired += actual->execution.count;
                    }
                }
            }
        }
    }
    EXPECT_GT(candidate.jitStats().trace_guard_mismatches, 0U);
}

TEST(JitTest, NativeTraceCachesValidChargePrefixAcrossTimingKeys) {
    if (!fil::cpu::MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter";
    const auto code = halfwords({0xbf00U, 0x6008U, 0x680aU, 0x3202U, 0x2a03U,
        0xd102U, 0x3301U, 0x604bU, 0x684cU, 0xe7f5U});
    auto bus = basicBus(), oracle = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, code));
    ASSERT_TRUE(oracle.loadBytes(flash_base, code));
    for (auto* memory : {&bus, &oracle}) {
        memory->setReversibleRamOnly(true);
        memory->setAllMmioTrapping(true);
    }
    fil::cpu::CortexM4 candidate(bus), manual(oracle);
    prepare(candidate);
    candidate.state().r[0] = 1U;
    candidate.state().r[1] = ram_base;
    manual.setNativeMemoryJitEnabled(false);
    for (unsigned i = 0U; i < 4000U && candidate.jitStats().trace_instructions == 0U; ++i) {
        fil::cpu::CortexM4::ReversibleCycleBudget warm;
        warm.remaining_cycles = 1000U;
        if (!candidate.tryStepCommittedRamSpan(64U, warm))
            ASSERT_EQ(candidate.stepFast().reason, fil::cpu::StopReason::step_complete);
    }
    ASSERT_GT(candidate.jitStats().trace_instructions, 0U);

    struct TimingCase {
        std::uint16_t nonsequential;
        std::uint16_t sequential;
        bool have_fetch;
        std::size_t expected;
    };
    // Observable prefix lengths: an oversized first charge declines, a valid
    // first charge followed by an oversized sequential charge retires one
    // instruction, and restoring small charges under either sequencing key
    // recovers the full requested prefix.
    constexpr std::size_t full = 10U;
    for (const auto timing : std::array<TimingCase, 5>{{
            {std::numeric_limits<std::uint16_t>::max(), 0U, false, 0U},
            {0U, std::numeric_limits<std::uint16_t>::max(), false, 1U},
            {0U, std::numeric_limits<std::uint16_t>::max(), true, 0U},
            {0U, 0U, false, full}, {0U, 0U, true, full}}}) {
        SCOPED_TRACE(::testing::Message() << timing.nonsequential << '/'
            << timing.sequential << '/' << timing.have_fetch);
        prepare(candidate);
        candidate.state().r[0] = 1U;
        candidate.state().r[1] = ram_base;
        manual.state() = candidate.state();
        ASSERT_TRUE(bus.write32(ram_base, 0U));
        ASSERT_TRUE(oracle.write32(ram_base, 0U));
        fil::cpu::CortexM4::ReversibleCycleBudget budget;
        budget.remaining_cycles = 100000U;
        budget.have_fetch = timing.have_fetch;
        budget.fetch_end = timing.have_fetch ? flash_base : flash_base + 64U;
        budget.fetch_timing = {flash_base, flash_base + 128U, 0U, 0U,
            timing.nonsequential, timing.sequential};
        budget.total_instruction_cycles.fill(0xBEEFU);
        const auto actual = candidate.tryStepBudgetedReversibleJitBlock(full, budget);
        if (timing.expected == 0U) {
            EXPECT_FALSE(actual);
            EXPECT_EQ(budget.remaining_cycles, 100000U);
            EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), manual.state()));
            continue;
        }
        ASSERT_TRUE(actual);
        // Independent oracle: single-step the same instructions with explicit
        // fetch-timing arithmetic to derive the expected committed prefix.
        std::uint64_t charged = 0U;
        bool sequential_source = timing.have_fetch;
        std::uint32_t fetch_end = budget.fetch_end;
        for (std::size_t i = 0U; i < timing.expected; ++i) {
            const auto pc = manual.state().r[15];
            const bool sequential = sequential_source && pc == fetch_end;
            const auto stepped = manual.stepFast();
            ASSERT_EQ(stepped.reason, fil::cpu::StopReason::step_complete);
            const auto charge = static_cast<std::uint32_t>(stepped.cycles)
                + (sequential ? timing.sequential : timing.nonsequential);
            ASSERT_LE(charge, std::numeric_limits<std::uint16_t>::max());
            charged += charge;
            sequential_source = true;
            fetch_end = pc + stepped.instruction_size;
        }
        EXPECT_EQ(actual->execution.count, timing.expected);
        EXPECT_EQ(actual->execution.result.instructions, timing.expected);
        EXPECT_EQ(budget.remaining_cycles, 100000U - charged);
        EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), manual.state()));
        EXPECT_TRUE(std::all_of(budget.total_instruction_cycles.begin() + timing.expected,
            budget.total_instruction_cycles.end(), [](const auto value) {
                return value == 0xBEEFU;
            }));
    }
}

TEST(JitTest, NativeTraceFirstFetchBiasMatchesCheckedCostsAcrossEntryModes) {
    if (!fil::cpu::MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter";
    // LDR r0,[r1]; ADDS r2,#1; B loop: supported memory and pure operations.
    const auto code = halfwords({0x6808U, 0x3201U, 0xe7fcU});
    auto bus = basicBus(), oracle = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, code));
    ASSERT_TRUE(oracle.loadBytes(flash_base, code));
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    fil::cpu::CortexM4 candidate(bus), checked(oracle);
    prepare(candidate);
    candidate.state().r[1] = ram_base;
    for (unsigned i = 0U; i < 4000U; ++i) {
        fil::cpu::CortexM4::ReversibleCycleBudget warm;
        warm.remaining_cycles = 1000U;
        if (!candidate.tryStepCommittedRamSpan(64U, warm))
            ASSERT_EQ(candidate.stepFast().reason, fil::cpu::StopReason::step_complete);
    }
    ASSERT_GT(candidate.jitStats().trace_instructions, 0U);
    for (const auto stalls : std::array<std::array<std::uint16_t, 2U>, 7U>{{
            {9U, 0U}, {0U, 9U}, {7U, 2U}, {2U, 7U}, {0U, 0U},
            {65535U, 0U}, {0U, 65535U}}}) {
        // Alternate modes without changing timing: the cached normalized table
        // must work in either direction, including a more costly sequential fetch.
        for (const auto credit : {0U, 1U, 2U, 5U, 12U, 40U, 128U, 65535U, 100000U}) {
            for (const bool detailed : {false, true}) {
                for (const bool sequential : {false, true, false, true}) {
                    SCOPED_TRACE(::testing::Message() << stalls[0] << '/' << stalls[1]
                        << " credit=" << credit << " sequential=" << sequential
                        << " detailed=" << detailed);
                    prepare(candidate); prepare(checked);
                    candidate.state().r[1] = checked.state().r[1] = ram_base;
                    fil::cpu::CortexM4::ReversibleCycleBudget budget;
                    budget.remaining_cycles = credit;
                    budget.have_fetch = sequential;
                    budget.fetch_end = flash_base;
                    budget.fetch_timing = {flash_base, flash_base + 128U, 0U, 0U,
                        stalls[0], stalls[1]};
                    budget.total_instruction_cycles.fill(0xBEEFU);
                    const auto limit = detailed ? 1U : 64U;
                    std::uint64_t charged = 0U;
                    std::uint64_t execution = 0U;
                    unsigned count = 0U;
                    auto fetch_end = flash_base;
                    bool have_fetch = sequential;
                    std::uint16_t first_charge = 0U;
                    while (count < limit) {
                        const auto before = checked.state();
                        const auto pc = before.r[15];
                        const auto step = checked.stepFast();
                        ASSERT_EQ(step.reason, fil::cpu::StopReason::step_complete);
                        const auto cost = static_cast<std::uint32_t>(step.cycles)
                            + stalls[have_fetch && pc == fetch_end ? 1U : 0U];
                        if (cost > std::numeric_limits<std::uint16_t>::max()
                            || charged + cost > credit) {
                            checked.state() = before;
                            break;
                        }
                        if (count == 0U) first_charge = static_cast<std::uint16_t>(cost);
                        charged += cost;
                        execution += step.cycles;
                        ++count;
                        have_fetch = true;
                        fetch_end = pc + step.instruction_size;
                    }
                    const auto result = detailed
                        ? [&]() -> std::optional<fil::cpu::FastStepResult> {
                            auto out = candidate.tryStepBudgetedReversibleJitBlock(limit, budget);
                            if (!out) return std::nullopt;
                            return out->execution.result;
                        }()
                        : candidate.tryStepCommittedRamSpan(limit, budget);
                    ASSERT_EQ(result.has_value(), count != 0U);
                    if (result) {
                        EXPECT_EQ(result->instructions, count);
                        EXPECT_EQ(result->cycles, execution);
                        if (detailed) EXPECT_EQ(budget.total_instruction_cycles[0], first_charge);
                    }
                    EXPECT_EQ(budget.remaining_cycles, credit - charged);
                    EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
                }
            }
        }
    }
}

TEST(JitTest, CheckedSingleAdmitsTakenBranchRefillBeforeCommit) {
    auto bus = basicBus();
    // B . has base cost 2 plus a taken-path refill of 2 cycles.
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0xe7feU})));
    fil::cpu::CortexM4 candidate(bus);
    prepare(candidate);
    const auto before = candidate.state();
    fil::cpu::CortexM4::ReversibleCycleBudget budget;
    budget.remaining_cycles = 3U;
    EXPECT_FALSE(candidate.tryStepCheckedSingle(budget));
    EXPECT_EQ(budget.remaining_cycles, 3U);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), before));
    budget.remaining_cycles = 4U;
    const auto result = candidate.tryStepCheckedSingle(budget);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->cycles, 4U);
    EXPECT_EQ(budget.remaining_cycles, 0U);
    EXPECT_EQ(candidate.state().r[15], flash_base);
}

TEST(JitTest, CheckedSingleAdmitsDataDependentDivideBeforeCommit) {
    auto bus = basicBus();
    // UDIV r0,r1,r2; divisor with no leading zeros takes the full 12 cycles.
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0xfbb1U, 0xf0f2U})));
    fil::cpu::CortexM4 candidate(bus);
    prepare(candidate); candidate.state().r[1] = 42U; candidate.state().r[2] = 0x80000000U;
    const auto before = candidate.state();
    fil::cpu::CortexM4::ReversibleCycleBudget budget;
    budget.remaining_cycles = 11U;
    EXPECT_FALSE(candidate.tryStepCheckedSingle(budget));
    EXPECT_EQ(budget.remaining_cycles, 11U);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), before));
    budget.remaining_cycles = 12U;
    const auto result = candidate.tryStepCheckedSingle(budget);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->cycles, 12U);
    EXPECT_EQ(budget.remaining_cycles, 0U);
}

TEST(JitTest, CompareBranchCommittedSpansMatchCheckedInstructionCycles) {
    if (!fil::cpu::MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter";
    // LDR r2,[r1]; CBZ/CBNZ r2,skip; ADDS r3,#1; STR r3,[r1,#4]; B loop.
    for (const auto branch : {0xb102U, 0xb902U}) {
        const auto code = halfwords({0x680aU, static_cast<std::uint16_t>(branch),
            0x3301U, 0x604bU, 0xe7faU});
        auto bus = basicBus(), oracle = basicBus();
        ASSERT_TRUE(bus.loadBytes(flash_base, code));
        ASSERT_TRUE(oracle.loadBytes(flash_base, code));
        bus.setReversibleRamOnly(true); bus.setAllMmioTrapping(true);
        fil::cpu::CortexM4 candidate(bus), checked(oracle);
        prepare(candidate); candidate.state().r[1] = ram_base;
        for (unsigned i = 0; i < 4000U; ++i) {
            fil::cpu::CortexM4::ReversibleCycleBudget warm; warm.remaining_cycles = 128U;
            if (!candidate.tryStepCommittedRamSpan(64U, warm))
                ASSERT_EQ(candidate.stepFast().reason, fil::cpu::StopReason::step_complete);
        }
        ASSERT_GT(candidate.jitStats().trace_instructions, 0U);
        for (const auto value : {0U, 1U}) {
            for (std::size_t limit = 1; limit <= 64; ++limit) {
                prepare(candidate); prepare(checked);
                candidate.state().r[1] = checked.state().r[1] = ram_base;
                ASSERT_TRUE(bus.write32(ram_base, value));
                ASSERT_TRUE(oracle.write32(ram_base, value));
                ASSERT_TRUE(bus.write32(ram_base + 4U, 0U));
                ASSERT_TRUE(oracle.write32(ram_base + 4U, 0U));
                fil::cpu::CortexM4::ReversibleCycleBudget budget; budget.remaining_cycles = 10000U;
                const auto span = candidate.tryStepCommittedRamSpan(limit, budget);
                if (!span) continue;
                std::uint64_t cycles = 0U;
                for (std::size_t i = 0; i < span->instructions; ++i) {
                    const auto step = checked.stepFast();
                    ASSERT_EQ(step.reason, fil::cpu::StopReason::step_complete);
                    cycles += step.cycles;
                }
                EXPECT_EQ(10000U - budget.remaining_cycles, cycles) << branch << '/' << value << '/' << limit;
                EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
            }
        }
    }
}

TEST(JitTest, CommittedRamSpanUnrollsLoopsWithEveryIterationRetired) {
    if (!fil::cpu::MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter";
    // LDR r2,[r1]; ADDS r2,#1; STR r2,[r1]; CMP r2,r0; BCC loop; BKPT.
    const auto code = halfwords({0x680aU, 0x3201U, 0x600aU, 0x4282U, 0xd3faU, 0xbe00U});
    auto bus = basicBus(), oracle = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, code));
    ASSERT_TRUE(oracle.loadBytes(flash_base, code));
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    fil::cpu::CortexM4 candidate(bus), checked(oracle);
    prepare(candidate);
    candidate.state().r[0] = 0xffffU;
    candidate.state().r[1] = ram_base;
    for (unsigned i = 0U; i < 4000U && candidate.jitStats().trace_instructions == 0U; ++i) {
        fil::cpu::CortexM4::ReversibleCycleBudget warm;
        warm.remaining_cycles = 1000U;
        if (!candidate.tryStepCommittedRamSpan(64U, warm))
            ASSERT_EQ(candidate.stepFast().reason, fil::cpu::StopReason::step_complete);
    }
    ASSERT_GT(candidate.jitStats().trace_instructions, 0U);
    prepare(candidate);
    candidate.state().r[0] = 0xffffU;
    candidate.state().r[1] = ram_base;
    fil::cpu::CortexM4::ReversibleCycleBudget detailed_budget;
    detailed_budget.remaining_cycles = 1000U;
    const auto bounded_detail = candidate.tryStepBudgetedReversibleJitBlock(64U, detailed_budget);
    ASSERT_TRUE(bounded_detail);
    EXPECT_LE(bounded_detail->execution.count, fil::cpu::CortexM4::JitStepOutcome::max_block);
    bool saw_long_span = false;
    for (const auto stop_after : {0xffffU, 5U}) {
        for (std::size_t limit = 1U; limit <= 64U; ++limit) {
            for (const auto credit : {0U, 1U, 7U, 16U, 31U, 63U, 127U, 255U}) {
                SCOPED_TRACE(::testing::Message() << stop_after << '/' << limit << '/' << credit);
                prepare(candidate); prepare(checked);
                candidate.state().r[0] = checked.state().r[0] = stop_after;
                candidate.state().r[1] = checked.state().r[1] = ram_base;
                ASSERT_TRUE(bus.write32(ram_base, 0U));
                ASSERT_TRUE(oracle.write32(ram_base, 0U));
                fil::cpu::CortexM4::ReversibleCycleBudget budget;
                budget.remaining_cycles = credit;
                budget.total_instruction_cycles.fill(0xBEEFU);
                budget.fetch_timing = {flash_base, flash_base + 256U, 0U, 0U, 4U, 1U};
                budget.have_fetch = (limit & 1U) != 0U;
                budget.fetch_end = flash_base;
                bool have_fetch = budget.have_fetch;
                auto fetch_end = budget.fetch_end;
                const auto actual = candidate.tryStepCommittedRamSpan(limit, budget);
                std::uint64_t cycles = 0U, charged_cycles = 0U;
                if (actual) {
                    EXPECT_LE(actual->instructions, limit);
                    saw_long_span |= actual->instructions > 16U;
                    for (std::uint32_t i = 0U; i < actual->instructions; ++i) {
                        const auto pc = checked.state().r[15];
                        const auto step = checked.stepFast();
                        ASSERT_EQ(step.reason, fil::cpu::StopReason::step_complete);
                        cycles += step.cycles;
                        charged_cycles += step.cycles + budget.fetch_timing.stall(
                            pc, have_fetch && pc == fetch_end);
                        have_fetch = true;
                        fetch_end = pc + step.instruction_size;
                    }
                    EXPECT_EQ(actual->cycles, cycles);
                }
                EXPECT_EQ(credit - budget.remaining_cycles, charged_cycles);
                EXPECT_EQ(budget.have_fetch, have_fetch);
                EXPECT_EQ(budget.fetch_end, fetch_end);
                EXPECT_TRUE(std::all_of(budget.total_instruction_cycles.begin(),
                    budget.total_instruction_cycles.end(), [](const std::uint16_t value) {
                        return value == 0xBEEFU;
                    }));
                EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
                EXPECT_EQ(bus.read32(ram_base).value(), oracle.read32(ram_base).value());
            }
        }
    }
    EXPECT_TRUE(saw_long_span);
}

TEST(JitTest, CommittedUnrolledTraceDeclinesLaterLoopCarriedAddress) {
    if (!fil::cpu::MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter";
    const auto code = halfwords({0x680aU, 0x3104U, 0x3301U, 0xe7fbU});
    auto bus = basicBus(), oracle = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, code));
    ASSERT_TRUE(oracle.loadBytes(flash_base, code));
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    fil::cpu::CortexM4 candidate(bus), checked(oracle);
    for (unsigned i = 0U; i < 4000U && candidate.jitStats().trace_instructions == 0U; ++i) {
        prepare(candidate);
        candidate.state().r[1] = ram_base;
        fil::cpu::CortexM4::ReversibleCycleBudget warm;
        warm.remaining_cycles = 1000U;
        static_cast<void>(candidate.tryStepCommittedRamSpan(64U, warm));
    }
    ASSERT_GT(candidate.jitStats().trace_instructions, 0U);
    prepare(candidate); prepare(checked);
    candidate.state().r[1] = checked.state().r[1] = ram_base + 252U;
    ASSERT_TRUE(bus.write32(ram_base + 252U, 99U));
    ASSERT_TRUE(oracle.write32(ram_base + 252U, 99U));
    fil::cpu::CortexM4::ReversibleCycleBudget budget;
    budget.remaining_cycles = 1000U;
    const auto actual = candidate.tryStepCommittedRamSpan(64U, budget);
    ASSERT_TRUE(actual);
    EXPECT_EQ(actual->instructions, 4U);
    std::uint64_t cycles = 0U;
    for (unsigned i = 0U; i < 4U; ++i) {
        const auto step = checked.stepFast();
        ASSERT_EQ(step.reason, fil::cpu::StopReason::step_complete);
        cycles += step.cycles;
    }
    EXPECT_EQ(actual->cycles, cycles);
    EXPECT_EQ(1000U - budget.remaining_cycles, cycles);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
}

TEST(JitTest, CompactCommittedFallbackPreservesDetailedResultAndFetchOrdering) {
    // A one-op pure prefix avoids host-timing-dependent micro-kernel admission;
    // otherwise the two independent CPUs can choose different dispatch paths.
    const auto code = halfwords({0x2001U, 0x6008U, 0x680aU, 0x3001U, 0x2a02U, 0xd100U});
    auto compact_bus = basicBus(), detailed_bus = basicBus();
    ASSERT_TRUE(compact_bus.loadBytes(flash_base, code));
    ASSERT_TRUE(detailed_bus.loadBytes(flash_base, code));
    for (auto* memory : {&compact_bus, &detailed_bus}) {
        memory->setReversibleRamOnly(true);
        memory->setAllMmioTrapping(true);
    }
    fil::cpu::CortexM4 compact(compact_bus), detailed(detailed_bus);
    prepare(compact); prepare(detailed);
    compact.setNativeMemoryJitEnabled(false);
    detailed.setNativeMemoryJitEnabled(false);
    for (unsigned i = 0U; i < 70U; ++i) {
        static_cast<void>(compact.prepareJitBlock(true));
        static_cast<void>(detailed.prepareJitBlock(true));
    }
    ASSERT_EQ(compact.jitStats().micro_compilations, 0U);
    ASSERT_EQ(detailed.jitStats().micro_compilations, 0U);

    for (std::uint32_t credit = 0U; credit <= 20U; ++credit) {
        SCOPED_TRACE(credit);
        prepare(compact); prepare(detailed);
        fil::cpu::CortexM4::ReversibleCycleBudget a, b;
        a.remaining_cycles = b.remaining_cycles = credit;
        compact.state().r[1] = detailed.state().r[1] = ram_base;
        FetchCallbackLog compact_log, detailed_log;
        compact_log.budget = &a;
        detailed_log.budget = &b;
        a.fetch_stall = b.fetch_stall = logFetchStall;
        a.context = &compact_log;
        b.context = &detailed_log;
        a.total_instruction_cycles.fill(0xBEEFU);
        const auto actual = compact.tryStepCommittedRamSpan(6U, a);
        const auto expected = detailed.tryStepBudgetedReversibleJitBlock(6U, b);
        ASSERT_EQ(actual.has_value(), expected.has_value());
        EXPECT_EQ(compact_log.count, detailed_log.count);
        for (std::size_t i = 0U; i < compact_log.count && i < compact_log.pcs.size(); ++i) {
            EXPECT_EQ(compact_log.pcs[i], detailed_log.pcs[i]);
            EXPECT_EQ(compact_log.sequential[i], detailed_log.sequential[i]);
            EXPECT_EQ(compact_log.remaining[i], detailed_log.remaining[i]);
            EXPECT_EQ(compact_log.fetch_end[i], detailed_log.fetch_end[i]);
            EXPECT_EQ(compact_log.have_fetch[i], detailed_log.have_fetch[i]);
        }
        if (actual) {
            const auto& ref = expected->execution.result;
            EXPECT_EQ(actual->reason, ref.reason);
            EXPECT_EQ(actual->instructions, ref.instructions);
            EXPECT_EQ(actual->cycles, ref.cycles);
            EXPECT_EQ(actual->instruction_address, ref.instruction_address);
            EXPECT_EQ(actual->raw, ref.raw);
            EXPECT_EQ(actual->instruction_size, ref.instruction_size);
            EXPECT_EQ(actual->suppress_loop_observation, ref.suppress_loop_observation);
        }
        EXPECT_TRUE(fil::cpu::bitwiseEqual(compact.state(), detailed.state()));
        EXPECT_EQ(compact_bus.read32(ram_base).value(), detailed_bus.read32(ram_base).value());
        EXPECT_EQ(a.remaining_cycles, b.remaining_cycles);
        EXPECT_EQ(a.have_fetch, b.have_fetch);
        EXPECT_EQ(a.fetch_end, b.fetch_end);
        EXPECT_TRUE(std::all_of(a.total_instruction_cycles.begin(),
            a.total_instruction_cycles.end(), [](const std::uint16_t value) {
                return value == 0xBEEFU;
            }));
    }
}

TEST(JitTest, CompactItFallbackKeepsCycleArrayUntouchedAtEveryBudgetCut) {
    const auto code = halfwords({0xbf08U, 0x3001U, 0xbf00U}); // IT EQ; ADDS r0, #1; NOP
    auto compact_bus = basicBus(), detailed_bus = basicBus();
    ASSERT_TRUE(compact_bus.loadBytes(flash_base, code));
    ASSERT_TRUE(detailed_bus.loadBytes(flash_base, code));
    for (auto* memory : {&compact_bus, &detailed_bus}) {
        memory->setReversibleRamOnly(true);
        memory->setAllMmioTrapping(true);
    }
    fil::cpu::CortexM4 compact(compact_bus), detailed(detailed_bus);
    compact.setNativeMemoryJitEnabled(false);
    detailed.setNativeMemoryJitEnabled(false);
    const FakeFetchStalls stalls{2U, 1U};
    for (const bool zero : {false, true}) {
        for (std::uint32_t credit = 0U; credit <= 8U; ++credit) {
            SCOPED_TRACE(credit);
            SCOPED_TRACE(zero);
            prepare(compact); prepare(detailed);
            if (zero) {
                compact.state().xpsr |= 1U << 30U;
                detailed.state().xpsr |= 1U << 30U;
            }
            fil::cpu::CortexM4::ReversibleCycleBudget a, b;
            a.remaining_cycles = b.remaining_cycles = credit;
            a.fetch_stall = b.fetch_stall = fakeFetchStall;
            a.context = b.context = &stalls;
            a.total_instruction_cycles.fill(0xBEEFU);
            for (unsigned step = 0U; step < 2U; ++step) {
                const auto actual = compact.tryStepCommittedRamSpan(8U, a);
                const auto expected = detailed.tryStepBudgetedReversibleJitBlock(8U, b);
                ASSERT_EQ(actual.has_value(), expected.has_value());
                EXPECT_TRUE(fil::cpu::bitwiseEqual(compact.state(), detailed.state()));
                EXPECT_EQ(a.remaining_cycles, b.remaining_cycles);
                EXPECT_EQ(a.have_fetch, b.have_fetch);
                EXPECT_EQ(a.fetch_end, b.fetch_end);
                EXPECT_TRUE(std::all_of(a.total_instruction_cycles.begin(),
                    a.total_instruction_cycles.end(), [](auto value) { return value == 0xBEEFU; }));
                if (!actual) break;
                EXPECT_EQ(actual->instructions, expected->execution.result.instructions);
                EXPECT_EQ(actual->cycles, expected->execution.result.cycles);
                EXPECT_EQ(actual->instruction_address, expected->execution.result.instruction_address);
            }
        }
    }
}

TEST(JitTest, CommittedRamSpanMatchesDetailedReversiblePrefixes) {
    if (!fil::cpu::MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter";
    const auto code = halfwords({0xbf00U, 0x6008U, 0x680aU, 0x3202U, 0x2a03U,
        0xd102U, 0x3301U, 0x604bU, 0x684cU, 0xe7f5U});
    auto compact_bus = basicBus(), detailed_bus = basicBus();
    ASSERT_TRUE(compact_bus.loadBytes(flash_base, code));
    ASSERT_TRUE(detailed_bus.loadBytes(flash_base, code));
    for (auto* memory : {&compact_bus, &detailed_bus}) {
        memory->setReversibleRamOnly(true);
        memory->setAllMmioTrapping(true);
    }
    fil::cpu::CortexM4 compact(compact_bus), detailed(detailed_bus);
    prepare(compact); prepare(detailed);
    compact.state().r[0] = 1U;
    compact.state().r[1] = ram_base;
    detailed.state().r[0] = 1U;
    detailed.state().r[1] = ram_base;
    for (unsigned i = 0U; i < 4000U && compact.jitStats().trace_instructions == 0U; ++i) {
        fil::cpu::CortexM4::ReversibleCycleBudget warm;
        warm.remaining_cycles = 1000U;
        if (!compact.tryStepCommittedRamSpan(64U, warm))
            ASSERT_EQ(compact.stepFast().reason, fil::cpu::StopReason::step_complete);
    }
    ASSERT_GT(compact.jitStats().trace_instructions, 0U);
    for (unsigned i = 0U; i < 4000U && detailed.jitStats().trace_instructions == 0U; ++i) {
        fil::cpu::CortexM4::ReversibleCycleBudget warm;
        warm.remaining_cycles = 1000U;
        if (!detailed.tryStepBudgetedReversibleJitBlock(64U, warm))
            ASSERT_EQ(detailed.stepFast().reason, fil::cpu::StopReason::step_complete);
    }
    ASSERT_GT(detailed.jitStats().trace_instructions, 0U);

    fil::cpu::CpuState initial;
    initial.r[15] = flash_base;
    initial.instruction_address = flash_base + 64U;
    initial.r[1] = ram_base;
    initial.r[3] = 5U;
    initial.xpsr = fil::cpu::xpsr_t | fil::cpu::xpsr_q | fil::cpu::xpsr_v;
    const auto callback = +[](const void*, const std::uint32_t pc,
                              const bool sequential) -> std::uint16_t {
        return pc >= flash_base && pc < flash_base + 128U
            ? static_cast<std::uint16_t>(sequential ? 1U : 3U) : 0U;
    };
    for (const std::uint32_t input : {1U, 2U}) {
        for (const std::uint32_t address : {ram_base, flash_base}) {
            for (std::size_t limit = 1U; limit <= 6U; ++limit) {
                for (std::uint32_t credit = 0U; credit <= 18U; ++credit) {
                    for (unsigned fetch = 0U; fetch < 2U; ++fetch) {
                        for (unsigned use_callback = 0U; use_callback < 2U; ++use_callback) {
                            SCOPED_TRACE(::testing::Message() << input << '/' << address << '/'
                                << limit << '/' << credit << '/' << fetch << '/' << use_callback);
                            compact.state() = detailed.state() = initial;
                            compact.state().r[0] = detailed.state().r[0] = input;
                            compact.state().r[1] = detailed.state().r[1] = address;
                            for (auto* memory : {&compact_bus, &detailed_bus}) {
                                ASSERT_TRUE(memory->write32(ram_base, 0U));
                                ASSERT_TRUE(memory->write32(ram_base + 4U, 0U));
                            }
                            fil::cpu::CortexM4::ReversibleCycleBudget a, b;
                            a.remaining_cycles = b.remaining_cycles = credit;
                            a.have_fetch = b.have_fetch = fetch != 0U;
                            a.fetch_end = b.fetch_end = fetch != 0U
                                ? flash_base : flash_base + 64U;
                            a.fetch_timing = b.fetch_timing = {
                                flash_base, flash_base + 128U, 0U, 0U, 3U, 1U};
                            if (use_callback != 0U) {
                                a.fetch_stall = b.fetch_stall = callback;
                            }
                            a.total_instruction_cycles.fill(0xBEEFU);
                            const auto actual = compact.tryStepCommittedRamSpan(limit, a);
                            const auto expected = detailed.tryStepBudgetedReversibleJitBlock(limit, b);
                            ASSERT_EQ(actual.has_value(), expected.has_value());
                            if (actual) {
                                const auto& ref = expected->execution.result;
                                EXPECT_EQ(actual->reason, ref.reason);
                                EXPECT_EQ(actual->instructions, ref.instructions);
                                EXPECT_EQ(actual->cycles, ref.cycles);
                                EXPECT_EQ(actual->instruction_address, ref.instruction_address);
                                EXPECT_EQ(actual->raw, ref.raw);
                                EXPECT_EQ(actual->instruction_size, ref.instruction_size);
                                EXPECT_EQ(actual->suppress_loop_observation,
                                    ref.suppress_loop_observation);
                            }
                            EXPECT_TRUE(fil::cpu::bitwiseEqual(compact.state(), detailed.state()));
                            EXPECT_EQ(a.remaining_cycles, b.remaining_cycles);
                            EXPECT_EQ(a.have_fetch, b.have_fetch);
                            EXPECT_EQ(a.fetch_end, b.fetch_end);
                            EXPECT_TRUE(std::all_of(a.total_instruction_cycles.begin(),
                                a.total_instruction_cycles.end(), [](const std::uint16_t cycles) {
                                    return cycles == 0xBEEFU;
                                }));
                            EXPECT_EQ(compact_bus.read32(ram_base).value(),
                                detailed_bus.read32(ram_base).value());
                            EXPECT_EQ(compact_bus.read32(ram_base + 4U).value(),
                                detailed_bus.read32(ram_base + 4U).value());
                        }
                    }
                }
            }
        }
    }
}

TEST(JitTest, NativeStackCallTraceMatchesEveryCutAndArchitecturalStackMode) {
    if (!fil::cpu::MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter";
    const auto code = halfwords({0xf000U, 0xf804U, 0x680aU, 0x2a00U, 0xd0faU, 0xbe00U,
        0xb508U, 0x684bU, 0x2b00U, 0xd100U, 0x3401U, 0x608cU, 0xbd08U});
    auto bus = basicBus(), oracle = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, code));
    ASSERT_TRUE(oracle.loadBytes(flash_base, code));
    for (auto* memory : {&bus, &oracle}) {
        memory->setReversibleRamOnly(true);
        memory->setAllMmioTrapping(true);
    }
    fil::cpu::CortexM4 candidate(bus), checked(oracle);
    prepare(candidate); prepare(checked);
    checked.setNativeMemoryJitEnabled(false);
    candidate.state().r[1] = ram_base;
    for (unsigned i = 0U; i < 4000U && candidate.jitStats().trace_instructions == 0U; ++i) {
        fil::cpu::CortexM4::ReversibleCycleBudget warm;
        warm.remaining_cycles = 1000U;
        if (!candidate.tryStepBudgetedReversibleJitBlock(64U, warm))
            ASSERT_EQ(candidate.stepFast().reason, fil::cpu::StopReason::step_complete);
    }
    ASSERT_GT(candidate.jitStats().trace_instructions, 0U);
    fil::cpu::CpuState initial;
    initial.r[15] = flash_base;
    initial.instruction_address = flash_base + 64U;
    initial.r[1] = ram_base;
    initial.r[3] = 9U;
    initial.r[4] = 3U;
    initial.msp = ram_base + 0xf0U;
    initial.psp = ram_base + 0xc0U;
    initial.r[13] = ram_base + 0x80U; // Deliberately stale mirror before PUSH.
    for (unsigned mode = 0U; mode < 3U; ++mode) {
        initial.control = mode == 0U ? 0U : 2U;
        initial.xpsr = fil::cpu::xpsr_t | fil::cpu::xpsr_q | (mode == 2U ? 3U : 0U);
        for (const auto changed_input : {0U, 1U}) {
            for (std::size_t limit = 1U; limit <= 11U; ++limit) {
                for (std::uint32_t credit = 0U; credit <= 64U; ++credit) {
                    SCOPED_TRACE(::testing::Message() << mode << '/' << changed_input << '/' << limit << '/' << credit);
                    candidate.state() = checked.state() = initial;
                    for (auto* memory : {&bus, &oracle}) {
                        for (std::uint32_t offset = 0U; offset < 256U; offset += 4U)
                            ASSERT_TRUE(memory->write32(ram_base + offset, offset == 4U ? changed_input : 0U));
                        memory->restoreReadFootprint({});
                    }
                    fil::cpu::CortexM4::ReversibleCycleBudget actual_budget;
                    actual_budget.remaining_cycles = credit;
                    actual_budget.have_fetch = true;
                    actual_budget.fetch_end = flash_base;
                    actual_budget.fetch_timing = {flash_base, flash_base + 128U, 0U, 0U, 4U, 1U};
                    auto expected_budget = actual_budget;
                    std::size_t retired = 0U;
                    while (retired < limit) {
                        const auto actual = candidate.tryStepBudgetedReversibleJitBlock(limit - retired, actual_budget);
                        if (!actual) {
                            EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
                            break;
                        }
                        std::uint32_t expected_cycles = 0U;
                        for (std::size_t i = 0U; i < actual->execution.count; ++i) {
                            const auto pc = checked.state().r[15];
                            const auto stall = expected_budget.fetchStallCycles(pc,
                                expected_budget.have_fetch && pc == expected_budget.fetch_end);
                            const auto expected = checked.stepFast();
                            ASSERT_EQ(expected.reason, fil::cpu::StopReason::step_complete);
                            const auto charged = static_cast<std::uint32_t>(expected.cycles) + stall;
                            ASSERT_LE(charged, expected_budget.remaining_cycles);
                            expected_budget.remaining_cycles -= charged;
                            expected_budget.have_fetch = true;
                            expected_budget.fetch_end = pc + expected.instruction_size;
                            expected_cycles += expected.cycles;
                            EXPECT_EQ(actual->execution.pcs[i], expected.instruction_address);
                            EXPECT_EQ(actual->instruction_cycles[i], expected.cycles);
                            EXPECT_EQ(actual_budget.total_instruction_cycles[i], charged);
                        }
                        EXPECT_EQ(actual->execution.result.cycles, expected_cycles);
                        EXPECT_EQ(actual_budget.remaining_cycles, expected_budget.remaining_cycles);
                        EXPECT_EQ(actual_budget.fetch_end, expected_budget.fetch_end);
                        EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
                        const auto actual_reads = bus.readFootprint(), expected_reads = oracle.readFootprint();
                        EXPECT_EQ(actual_reads.words, expected_reads.words);
                        for (std::uint32_t offset = 0U; offset < 256U; offset += 4U)
                            EXPECT_EQ(bus.read32(ram_base + offset).value(), oracle.read32(ram_base + offset).value());
                        bus.restoreReadFootprint(actual_reads);
                        oracle.restoreReadFootprint(expected_reads);
                        retired += actual->execution.count;
                    }
                }
            }
        }
    }
    EXPECT_GT(candidate.jitStats().trace_guard_mismatches, 0U);
}

TEST(JitTest, NativeTraceDeclinesCallbacksPendingStateAndStaleCodeGenerations) {
    if (!fil::cpu::MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter";
    const auto code = halfwords({0xbf00U, 0x6008U, 0x680aU, 0x3202U, 0x2a03U,
        0xd102U, 0x3301U, 0x604bU, 0x684cU, 0xe7f5U});
    auto bus = basicBus(), oracle = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, code));
    ASSERT_TRUE(oracle.loadBytes(flash_base, code));
    bus.setReversibleRamOnly(true); bus.setAllMmioTrapping(true);
    fil::cpu::CortexM4 candidate(bus), checked(oracle);
    prepare(candidate); prepare(checked);
    checked.setNativeMemoryJitEnabled(false);
    candidate.state().r[0] = 1U; candidate.state().r[1] = ram_base;
    for (unsigned i = 0U; i < 4000U && candidate.jitStats().trace_instructions == 0U; ++i) {
        fil::cpu::CortexM4::ReversibleCycleBudget budget; budget.remaining_cycles = 1000U;
        if (!candidate.tryStepBudgetedReversibleJitBlock(64U, budget))
            ASSERT_EQ(candidate.stepFast().reason, fil::cpu::StopReason::step_complete);
    }
    ASSERT_GT(candidate.jitStats().trace_instructions, 0U);
    const auto trace_calls = candidate.jitStats().trace_executions;
    auto initial = candidate.state(); initial.r[15] = flash_base;
    initial.instruction_address = flash_base + 64U; initial.r[0] = 1U;
    for (unsigned invalid = 0U; invalid < 4U; ++invalid) {
        candidate.state() = initial;
        if (invalid == 0U) candidate.state().halted = true;
        if (invalid == 1U) candidate.state().thumb = false;
        if (invalid == 2U) candidate.state().pending_exception = 3U;
        if (invalid == 3U) candidate.state().pending_exc_return = 0xfffffffdU;
        const auto before = candidate.state();
        fil::cpu::CortexM4::ReversibleCycleBudget budget; budget.remaining_cycles = 100U;
        EXPECT_FALSE(candidate.tryStepBudgetedReversibleJitBlock(10U, budget));
        EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), before));
        EXPECT_EQ(budget.remaining_cycles, 100U);
    }
    candidate.state() = checked.state() = initial;
    unsigned callbacks = 0U;
    fil::cpu::CortexM4::ReversibleCycleBudget callback_budget;
    callback_budget.remaining_cycles = 100U; callback_budget.context = &callbacks;
    callback_budget.fetch_stall = [](const void* context, std::uint32_t, bool) -> std::uint16_t {
        ++*static_cast<unsigned*>(const_cast<void*>(context)); return 0U;
    };
    const auto callback_result = candidate.tryStepBudgetedReversibleJitBlock(10U, callback_budget);
    ASSERT_TRUE(callback_result);
    EXPECT_EQ(callbacks, callback_result->execution.count);
    for (unsigned i = 0U; i < callback_result->execution.count; ++i)
        ASSERT_EQ(checked.stepFast().reason, fil::cpu::StopReason::step_complete);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
    EXPECT_EQ(candidate.jitStats().trace_executions, trace_calls);
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0x2007U})));
    ASSERT_TRUE(oracle.loadBytes(flash_base, halfwords({0x2007U})));
    candidate.state() = checked.state() = initial;
    fil::cpu::CortexM4::ReversibleCycleBudget changed; changed.remaining_cycles = 100U;
    const auto result = candidate.tryStepBudgetedReversibleJitBlock(10U, changed);
    ASSERT_TRUE(result);
    for (unsigned i = 0U; i < result->execution.count; ++i)
        ASSERT_EQ(checked.stepFast().reason, fil::cpu::StopReason::step_complete);
    EXPECT_EQ(candidate.state().r[0], 7U);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.state(), checked.state()));
    EXPECT_EQ(candidate.jitStats().trace_executions, trace_calls);
}

TEST(JitTest, NativeFirstMemoryDeclineRetriesOnlyAfterAddressChanges) {
    if (!fil::cpu::MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter";
    auto memory = basicBus();
    ASSERT_TRUE(memory.loadBytes(flash_base, halfwords({0x680aU, 0x3201U, 0xbf00U, 0xbe00U})));
    ASSERT_TRUE(memory.write32(ram_base, 5U));
    SharedMmio mmio;
    ASSERT_TRUE(memory.mapMmio(0x40000000U, 16U, mmio, "mmio"));
    memory.setReversibleRamOnly(true);
    memory.setAllMmioTrapping(true);
    fil::cpu::CortexM4 cpu(memory);
    prepare(cpu);
    cpu.state().r[1] = 0x40000000U;
    for (unsigned i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock(true));
    fil::cpu::CortexM4::ReversibleCycleBudget budget;
    budget.remaining_cycles = 100U;
    ASSERT_FALSE(cpu.tryStepBudgetedReversibleJitBlock(2U, budget));
    const auto attempts = cpu.jitStats().native_executions;
    ASSERT_GT(attempts, 0U);
    EXPECT_FALSE(cpu.tryStepBudgetedReversibleJitBlock(2U, budget));
    EXPECT_EQ(cpu.jitStats().native_executions, attempts);
    EXPECT_EQ(mmio.reads, 0);
    cpu.state().r[1] = ram_base;
    const auto success = cpu.tryStepBudgetedReversibleJitBlock(2U, budget);
    ASSERT_TRUE(success);
    EXPECT_EQ(success->execution.count, 2U);
    EXPECT_GT(cpu.jitStats().native_executions, attempts);
    EXPECT_EQ(cpu.state().r[2], 6U);
}

TEST(JitTest, MixedNativeReversibleMatchesCachedAtEveryBudgetAndDecline) {
    std::uint64_t native_total = 0U;
    for (const bool decline : {false, true}) {
        for (std::size_t limit = 1U; limit <= 6U; ++limit) {
            for (const auto credit : {0U, 1U, 2U, 3U, 5U, 7U, 11U, 100U}) {
                SCOPED_TRACE(decline);
                SCOPED_TRACE(limit);
                SCOPED_TRACE(credit);
                auto memory = basicBus();
                auto cached_memory = basicBus();
                const auto code = halfwords({0x2007U, 0x6008U, 0x681aU,
                    0x3201U, 0x604aU, 0xbf00U, 0xe7feU});
                ASSERT_TRUE(memory.loadBytes(flash_base, code));
                ASSERT_TRUE(cached_memory.loadBytes(flash_base, code));
                fil::cpu::CortexM4 native(memory), cached(cached_memory);
                native.setNativeMemoryJitEnabled(true);
                cached.setNativeMemoryJitEnabled(false);
                for (auto* cpu : {&native, &cached}) {
                    prepare(*cpu);
                    cpu->state().r[1] = ram_base;
                    cpu->state().r[3] = decline ? 0x40000000U : ram_base;
                    cpu->state().xpsr = 0x39000000U;
                    for (unsigned i = 0U; i < 70U; ++i)
                        static_cast<void>(cpu->prepareJitBlock(true));
                }
                memory.setReversibleRamOnly(true);
                memory.setAllMmioTrapping(true);
                cached_memory.setReversibleRamOnly(true);
                cached_memory.setAllMmioTrapping(true);
                fil::cpu::CortexM4::ReversibleCycleBudget a, b;
                a.remaining_cycles = b.remaining_cycles = credit;
                a.fetch_timing = b.fetch_timing = {flash_base, flash_base + 256U,
                    0U, 0U, 3U, 1U};
                const auto expected = cached.tryStepBudgetedReversibleJitBlock(limit, b);
                const auto actual = native.tryStepBudgetedReversibleJitBlock(limit, a);
                ASSERT_EQ(actual.has_value(), expected.has_value());
                EXPECT_TRUE(fil::cpu::bitwiseEqual(native.state(), cached.state()));
                EXPECT_EQ(memory.read32(ram_base).value(), cached_memory.read32(ram_base).value());
                EXPECT_EQ(memory.read32(ram_base + 4U).value(), cached_memory.read32(ram_base + 4U).value());
                EXPECT_EQ(a.remaining_cycles, b.remaining_cycles);
                EXPECT_EQ(a.have_fetch, b.have_fetch);
                EXPECT_EQ(a.fetch_end, b.fetch_end);
                EXPECT_EQ(memory.readFootprint().words, cached_memory.readFootprint().words);
                EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence,
                    cached_memory.sideEffectCheckpoint().mutation_sequence);
                if (actual) {
                    ASSERT_EQ(actual->execution.count, expected->execution.count);
                    EXPECT_EQ(actual->execution.result.cycles, expected->execution.result.cycles);
                    for (std::size_t i = 0U; i < actual->execution.count; ++i) {
                        EXPECT_EQ(actual->instruction_cycles[i], expected->instruction_cycles[i]);
                        EXPECT_EQ(a.total_instruction_cycles[i], b.total_instruction_cycles[i]);
                    }
                }
                native_total += native.nativeMemoryInstructions();
            }
        }
    }
#if defined(__x86_64__) && defined(FIL_ENABLE_MICRO_JIT) && FIL_ENABLE_MICRO_JIT
    EXPECT_GT(native_total, 0U) << "differential test must exercise the mixed native backend";
#else
    static_cast<void>(native_total); // Unsupported mixed backends retain exact cached execution.
#endif
}

TEST(JitTest, BudgetedReversiblePreparedMemoryDeclineRestoresSpeculativeState) {
    const auto check_decline = [](const std::uint16_t instruction,
                                  const std::uint32_t address) {
        auto bus = basicBus();
        ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({instruction, 0xBF00U})).hasValue());
        ASSERT_TRUE(bus.mapRam(ram_base + 0x1000U, 256U,
            "jit-reversible-executable", true).hasValue());
        bus.setReversibleRamOnly(true);
        bus.setAllMmioTrapping(true);
        fil::cpu::CortexM4 cpu(bus);
        prepare(cpu);
        cpu.state().r[0] = 0xCAFEBABEU;
        cpu.state().r[1] = address;
        cpu.state().instruction_address = flash_base - 2U;
        for (unsigned i = 0U; i < 70U; ++i) {
            static_cast<void>(cpu.prepareJitBlock(true));
        }
        fil::cpu::CortexM4::ReversibleCycleBudget budget{};
        budget.remaining_cycles = 50U;
        const auto before = cpu.state();
        const auto generation = bus.executionGeneration();
        const auto result = cpu.tryStepBudgetedReversibleJitBlock(2U, budget);
        EXPECT_FALSE(result) << "address=0x" << std::hex << address;
        EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), before));
        EXPECT_EQ(budget.remaining_cycles, 50U);
        EXPECT_FALSE(budget.have_fetch);
        EXPECT_EQ(bus.executionGeneration(), generation);
    };
    // LDR declines only when no backed bytes exist; read-only flash is valid.
    for (const std::uint32_t address : {0x40000000U, 0x50000000U}) {
        check_decline(0x6848U, address); // ldr r0,[r1,#4]
    }
    // STR additionally declines read-only and out-of-range backed-memory writes.
    for (const std::uint32_t address : {
             0x40000000U, 0x50000000U, flash_base, ram_base + 0xffU,
             ram_base + 0x1000U}) {
        check_decline(0x6048U, address); // str r0,[r1,#4]
    }
}

TEST(JitTest, BudgetedReversibleFailedConditionalCannotOverrunNativePrefix) {
    auto bus = basicBus();
    // nop; bne +0 (failed with Z set); movs r0,#1; movs r1,#2
    // A failed conditional updates the native stencil's failed-branch mask.
    // The two-instruction request must stop before either MOV.
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({
        0xBF00U, 0xD100U, 0x2001U, 0x2102U, 0xBF00U
    })).hasValue());
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    cpu.state().xpsr |= fil::cpu::xpsr_z; // BNE is not taken.
    for (std::uint32_t i = 0U; i < 70U; ++i) {
        static_cast<void>(cpu.prepareJitBlock());
    }

    fil::cpu::CortexM4::ReversibleCycleBudget budget{};
    budget.remaining_cycles = 100U;
    const auto result = cpu.tryStepBudgetedReversibleJitBlock(2U, budget);
    ASSERT_TRUE(result);
    ASSERT_EQ(result->execution.count, 2U);
    EXPECT_EQ(result->execution.pcs[0], flash_base);
    EXPECT_EQ(result->execution.pcs[1], flash_base + 2U);
    EXPECT_EQ(cpu.state().r[15], flash_base + 4U);
    EXPECT_EQ(cpu.state().instruction_address, flash_base + 2U);
    EXPECT_EQ(cpu.state().r[0], 0U);
    EXPECT_EQ(cpu.state().r[1], 0U);
    EXPECT_EQ(budget.total_instruction_cycles[0] + budget.total_instruction_cycles[1],
              result->execution.result.cycles);
}

TEST(JitTest, BudgetedReversiblePrefixDebitsExactCommittedCosts) {
    auto bus = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({
        0x3001U, 0xD000U, 0xFBB5U, 0xF2F6U, 0x6008U, 0x4687U
    })).hasValue());
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    fil::cpu::CortexM4 cpu(bus);
    fil::cpu::CortexM4 reference(bus);
    prepare(cpu);
    prepare(reference);
    cpu.state().r[1] = reference.state().r[1] = ram_base;
    cpu.state().r[5] = reference.state().r[5] = 101U;
    cpu.state().r[6] = reference.state().r[6] = 10U;
    for (std::uint32_t i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());

    FakeFetchStalls stalls{3U, 1U};
    fil::cpu::CortexM4::ReversibleCycleBudget budget{};
    budget.fetch_stall = fakeFetchStall;
    budget.context = &stalls;
    const auto initial = cpu.state();
    for (const std::uint64_t credit : {0U, 1U, 3U}) {
        budget.remaining_cycles = credit;
        EXPECT_FALSE(cpu.tryStepBudgetedReversibleJitBlock(16U, budget));
        EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), initial));
        EXPECT_EQ(budget.remaining_cycles, credit);
        EXPECT_FALSE(budget.have_fetch);
    }

    budget.remaining_cycles = 4U;
    auto first = cpu.tryStepBudgetedReversibleJitBlock(16U, budget);
    ASSERT_TRUE(first);
    ASSERT_EQ(first->execution.count, 1U);
    EXPECT_EQ(first->instruction_cycles[0], 1U);
    EXPECT_EQ(budget.total_instruction_cycles[0], 4U);
    EXPECT_EQ(budget.remaining_cycles, 0U);
    EXPECT_TRUE(budget.have_fetch);
    EXPECT_EQ(budget.fetch_end, flash_base + 2U);

    // Prepare the next entry PC after the first committed capsule.
    for (std::uint32_t i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
    // Failed BEQ (ADDS cleared Z), UDIV, and STR each include the exact
    // sequential-fetch surcharge; the final generic MOV-to-PC is not run.
    budget.remaining_cycles = 10U;
    auto suffix = cpu.tryStepBudgetedReversibleJitBlock(16U, budget);
    ASSERT_TRUE(suffix);
    ASSERT_EQ(suffix->execution.count, 3U);
    EXPECT_EQ(suffix->instruction_cycles[0], 1U); // failed conditional branch
    EXPECT_EQ(suffix->instruction_cycles[1], 4U); // UDIV r5/r6 = 10, divisor 10
    EXPECT_EQ(budget.total_instruction_cycles[0], 2U);
    EXPECT_EQ(budget.total_instruction_cycles[1], 5U);
    EXPECT_EQ(budget.total_instruction_cycles[2], 3U);
    EXPECT_EQ(budget.remaining_cycles, 0U);
    EXPECT_EQ(budget.fetch_end, flash_base + 10U);
    EXPECT_EQ(cpu.state().r[15], flash_base + 10U);

    std::uint32_t reference_cycles = 0U;
    for (std::uint32_t i = 0U; i < 4U; ++i) {
        const auto step = reference.stepFast();
        ASSERT_EQ(step.reason, fil::cpu::StopReason::step_complete);
        reference_cycles += step.cycles;
    }
    EXPECT_EQ(first->execution.result.cycles + suffix->execution.result.cycles,
        reference_cycles);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
    const auto stored = bus.read32(ram_base);
    ASSERT_TRUE(stored);
    EXPECT_EQ(stored.value(), cpu.state().r[0]);
}

TEST(JitTest, PerformanceAdmissionPreservesStateAndReusesKernelVerdicts) {
    if (!fil::cpu::MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter";
    auto bus = basicBus();
    const auto code = halfwords({0x4620U, 0x4681U, 0x464aU, 0xe000U});
    ASSERT_TRUE(bus.loadBytes(flash_base, code));
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    cpu.state().r[4] = 0x87654321U;
    const auto initial = cpu.state();
    const auto warm = [&] {
        const auto checkpoint = bus.sideEffectCheckpoint();
        const auto footprint = bus.readFootprint();
        for (std::uint32_t i = 0U; i < 70U; ++i)
            static_cast<void>(cpu.prepareJitBlock());
        EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), initial));
        const auto after = bus.sideEffectCheckpoint();
        EXPECT_EQ(after.mutation_sequence, checkpoint.mutation_sequence);
        EXPECT_EQ(after.mmio_generation, checkpoint.mmio_generation);
        EXPECT_EQ(after.journal_generation, checkpoint.journal_generation);
        // Decode fetches may read flash; calibration must not touch RAM.
        EXPECT_EQ(bus.readFootprint().words, footprint.words);
    };
    warm();
    const auto first = cpu.jitStats();
    ASSERT_EQ(first.micro_admitted + first.micro_rejected, 1U);
    ASSERT_GT(first.micro_admission_cost_ns, 0U);
    EXPECT_EQ(first.micro_executions, 0U);
    EXPECT_EQ(first.micro_instructions, 0U);
    EXPECT_EQ(first.block_executions, 0U);
    EXPECT_EQ(first.block_instructions, 0U);
    // Re-decode identical code after generation invalidation. The immutable
    // native kernel and its measured verdict must both be reused.
    ASSERT_TRUE(bus.loadBytes(flash_base, code));
    warm();
    const auto second = cpu.jitStats();
    EXPECT_EQ(second.micro_compilations, first.micro_compilations);
    EXPECT_EQ(second.micro_admitted, first.micro_admitted);
    EXPECT_EQ(second.micro_rejected, first.micro_rejected);
    EXPECT_EQ(second.micro_admission_cost_ns, first.micro_admission_cost_ns);
}

TEST(JitTest, RegisterMovNativePrefixesKeepCyclesBranchesAndMemoryDeclinesExact) {
    for (const bool zero : {false, true}) {
        for (const std::size_t requested : {1U, 2U, 3U, 4U, 5U}) {
            auto bus = basicBus();
            // mov r0,r4; mov r9,r0; mov r2,r9; beq +0; ldr r2,[r3]; nop.
            ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({
                0x4620U, 0x4681U, 0x464aU, 0xd000U, 0x681aU, 0xbf00U})).hasValue());
            bus.setReversibleRamOnly(true);
            bus.setAllMmioTrapping(true);
            fil::cpu::CortexM4 cpu(bus), reference(bus);
            prepare(cpu);
            prepare(reference);
            cpu.state().r[4] = reference.state().r[4] = 0x87654321U;
            cpu.state().r[3] = reference.state().r[3] = 0x40000000U;
            cpu.state().xpsr = reference.state().xpsr = 0xb9000000U
                | (zero ? 0x40000000U : 0U);
            for (std::uint32_t i = 0U; i < 70U; ++i)
                static_cast<void>(cpu.prepareJitBlock());
            FakeFetchStalls stalls{3U, 1U};
            fil::cpu::CortexM4::ReversibleCycleBudget budget{};
            budget.remaining_cycles = 100U;
            budget.fetch_stall = fakeFetchStall;
            budget.context = &stalls;
            const auto result = cpu.tryStepBudgetedReversibleJitBlock(requested, budget);
            ASSERT_TRUE(result);
            const auto count = std::min<std::size_t>(requested, 4U);
            ASSERT_EQ(result->execution.count, count);
            std::uint64_t total = 0U;
            std::uint32_t cycles = 0U;
            for (std::size_t i = 0U; i < count; ++i) {
                const auto step = reference.stepFast();
                ASSERT_EQ(step.reason, fil::cpu::StopReason::step_complete);
                EXPECT_EQ(result->instruction_cycles[i], step.cycles);
                EXPECT_EQ(budget.total_instruction_cycles[i], step.cycles + (i == 0U ? 3U : 1U));
                total += budget.total_instruction_cycles[i];
                cycles += step.cycles;
            }
            EXPECT_EQ(result->execution.result.cycles, cycles);
            EXPECT_EQ(budget.remaining_cycles, 100U - total);
            EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
            EXPECT_EQ(budget.fetch_end, flash_base + static_cast<std::uint32_t>(2U * count));
            if (fil::cpu::MicroJit::hostSupported()) {
                const auto stats = cpu.jitStats();
                EXPECT_GT(stats.micro_admitted + stats.micro_rejected, 0U);
                EXPECT_TRUE(stats.micro_instructions == 0U || stats.micro_instructions == count);
            }
        }
    }
}

TEST(JitTest, BudgetedReversibleBlockRejectsBeforeTakenBranchSideEffects) {
    auto bus = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0xBF00U, 0xE7FFU})).hasValue());
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    for (std::uint32_t i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());

    fil::cpu::CortexM4::ReversibleCycleBudget budget{};
    budget.remaining_cycles = 2U;
    const auto prefix = cpu.tryStepBudgetedReversibleJitBlock(16U, budget);
    ASSERT_TRUE(prefix);
    ASSERT_EQ(prefix->execution.count, 1U);
    EXPECT_EQ(cpu.state().r[15], flash_base + 2U);
    EXPECT_EQ(prefix->execution.result.cycles, 1U);
    EXPECT_EQ(budget.total_instruction_cycles[0], 1U);
    EXPECT_EQ(budget.remaining_cycles, 1U)
        << "branch's conservative refill bound rejected it before execution";
}

TEST(JitTest, BudgetedReversibleDeclineDoesNotSpendCreditOnDeclinedMemory) {
    constexpr std::uint32_t mmio_base = 0x40000000U;
    auto bus = basicBus();
    SharedMmio device;
    ASSERT_TRUE(bus.mapMmio(mmio_base, 16U, device, "budget-mmio").hasValue());
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0x2001U, 0x6008U, 0xBF00U})).hasValue());
    bus.setAllMmioTrapping(true);
    bus.setReversibleRamOnly(true);
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    cpu.state().r[1] = mmio_base;
    for (std::uint32_t i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
    FakeFetchStalls stalls{2U, 1U};
    fil::cpu::CortexM4::ReversibleCycleBudget budget{};
    budget.remaining_cycles = 3U;
    budget.fetch_stall = fakeFetchStall;
    budget.context = &stalls;
    const auto prefix = cpu.tryStepBudgetedReversibleJitBlock(16U, budget);
    ASSERT_TRUE(prefix);
    ASSERT_EQ(prefix->execution.count, 1U);
    EXPECT_EQ(budget.total_instruction_cycles[0], 3U);
    EXPECT_EQ(budget.remaining_cycles, 0U);
    EXPECT_EQ(budget.fetch_end, flash_base + 2U);
    EXPECT_EQ(cpu.state().r[15], flash_base + 2U)
        << "declined MMIO store is not speculatively consumed";
    EXPECT_EQ(device.reads, 0);

    auto executable_bus = basicBus();
    ASSERT_TRUE(executable_bus.mapRam(
        ram_base + 0x1000U, 0x100U, "budget-executable-ram", true).hasValue());
    ASSERT_TRUE(executable_bus.loadBytes(
        flash_base, halfwords({0x6008U, 0xBF00U})).hasValue());
    executable_bus.setAllMmioTrapping(true);
    executable_bus.setReversibleRamOnly(true);
    fil::cpu::CortexM4 executable_cpu(executable_bus);
    prepare(executable_cpu);
    executable_cpu.state().r[0] = 0x2001U;
    executable_cpu.state().r[1] = ram_base + 0x1000U;
    for (std::uint32_t i = 0U; i < 70U; ++i) {
        static_cast<void>(executable_cpu.prepareJitBlock());
    }
    fil::cpu::CortexM4::ReversibleCycleBudget exec_budget{};
    exec_budget.remaining_cycles = 20U;
    const auto executable_before = executable_cpu.state();
    EXPECT_FALSE(executable_cpu.tryStepBudgetedReversibleJitBlock(16U, exec_budget));
    EXPECT_TRUE(fil::cpu::bitwiseEqual(executable_cpu.state(), executable_before));
    EXPECT_EQ(exec_budget.remaining_cycles, 20U);
    EXPECT_FALSE(exec_budget.have_fetch);
    const auto unchanged = executable_bus.read32(ram_base + 0x1000U);
    ASSERT_TRUE(unchanged);
    EXPECT_EQ(unchanged.value(), 0U);
}

TEST(JitTest, ReversibleScalarFallbackPreservesEveryItMaskAndPredicate) {
    for (std::uint16_t mask = 1U; mask < 16U; ++mask) {
        for (const bool zero : {false, true}) {
            auto bus = basicBus();
            ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({
                static_cast<std::uint16_t>(0xBF10U | mask), // IT NE, mask
                0x2001U, // MOVS r0,#1 (implicit flags suppressed in IT)
                0xBF00U
            })).hasValue());
            bus.setReversibleRamOnly(true);
            bus.setAllMmioTrapping(true);
            fil::cpu::CortexM4 cpu(bus);
            fil::cpu::CortexM4 reference(bus);
            prepare(cpu);
            prepare(reference);
            cpu.state().xpsr = reference.state().xpsr = fil::cpu::xpsr_t
                | (zero ? fil::cpu::xpsr_z : 0U);
            ASSERT_EQ(cpu.stepFast().reason, fil::cpu::StopReason::step_complete);
            ASSERT_EQ(reference.stepFast().reason, fil::cpu::StopReason::step_complete);
            const auto expected = reference.stepFast();
            ASSERT_EQ(expected.reason, fil::cpu::StopReason::step_complete);
            fil::cpu::CortexM4::ReversibleCycleBudget budget{};
            budget.remaining_cycles = 100U;
            const auto actual = cpu.tryStepBudgetedReversibleJitBlock(1U, budget);
            ASSERT_TRUE(actual) << "mask=" << mask << " zero=" << zero;
            ASSERT_EQ(actual->execution.count, 1U);
            EXPECT_EQ(actual->execution.result.cycles, expected.cycles);
            EXPECT_EQ(actual->execution.pcs[0], expected.instruction_address);
            EXPECT_EQ(budget.total_instruction_cycles[0], expected.cycles);
            EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()))
                << "mask=" << mask << " zero=" << zero;
        }
    }
}

TEST(JitTest, CompleteItBlocksMatchStepFastAcrossMasksConditionsAndFlags) {
    constexpr std::array<std::uint32_t, 4> flags{
        fil::cpu::xpsr_n, fil::cpu::xpsr_z, fil::cpu::xpsr_c, fil::cpu::xpsr_v};
    for (std::uint16_t mask = 1U; mask < 16U; ++mask) {
        for (std::uint16_t condition = 0U; condition < 14U; ++condition) {
            auto bus = basicBus();
            ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({
                static_cast<std::uint16_t>(0xBF00U
                    | (static_cast<std::uint32_t>(condition) << 4U) | mask),
                0x2001U, 0x2002U, 0x2003U, 0x2004U
            })).hasValue());
            bus.setReversibleRamOnly(true);
            bus.setAllMmioTrapping(true);
            fil::cpu::CortexM4 cpu(bus);
            fil::cpu::CortexM4 reference(bus);
            for (std::uint32_t combination = 0U; combination < 16U; ++combination) {
                prepare(cpu);
                prepare(reference);
                std::uint32_t apsr = 0U;
                for (std::uint32_t bit = 0U; bit < flags.size(); ++bit) {
                    if ((combination & (1U << bit)) != 0U) apsr |= flags[bit];
                }
                cpu.state().xpsr = reference.state().xpsr = fil::cpu::xpsr_t | apsr;
                ASSERT_EQ(reference.stepFast().reason, fil::cpu::StopReason::step_complete);
                ASSERT_EQ(cpu.stepFast().reason, fil::cpu::StopReason::step_complete);
                ASSERT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
                while (fil::cpu::inItBlock(reference.state().it_state)) {
                    const auto expected = reference.stepFast();
                    ASSERT_EQ(expected.reason, fil::cpu::StopReason::step_complete);
                    const auto actual = cpu.tryStepReversibleJitBlock(1U);
                    ASSERT_TRUE(actual) << "mask=" << mask << " cond=" << condition
                                        << " flags=" << combination;
                    ASSERT_EQ(actual->execution.count, 1U);
                    EXPECT_EQ(actual->execution.pcs[0], expected.instruction_address);
                    EXPECT_EQ(actual->instruction_cycles[0], expected.cycles);
                    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()))
                        << "mask=" << mask << " cond=" << condition
                        << " flags=" << combination;
                }
            }
        }
    }
}

TEST(JitTest, ItFallbackBudgetRequiresConservativeCreditAndFetchStalls) {
    auto bus = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0xBF1CU, 0x2001U, 0x2002U})).hasValue());
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    fil::cpu::CortexM4 cpu(bus);
    fil::cpu::CortexM4 reference(bus);
    prepare(cpu);
    prepare(reference);
    ASSERT_EQ(cpu.stepFast().reason, fil::cpu::StopReason::step_complete);
    ASSERT_EQ(reference.stepFast().reason, fil::cpu::StopReason::step_complete);
    const auto before = cpu.state();

    FakeFetchStalls stalls{3U, 2U};
    fil::cpu::CortexM4::ReversibleCycleBudget budget{};
    budget.context = &stalls;
    budget.fetch_stall = fakeFetchStall;
    const auto expected_first = reference.stepFast();
    budget.remaining_cycles = expected_first.cycles + stalls.nonsequential - 1U;
    EXPECT_FALSE(cpu.tryStepBudgetedReversibleJitBlock(1U, budget));
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), before));
    EXPECT_EQ(budget.remaining_cycles, expected_first.cycles + stalls.nonsequential - 1U);
    EXPECT_FALSE(budget.have_fetch);

    budget.remaining_cycles = expected_first.cycles + stalls.nonsequential;
    const auto first = cpu.tryStepBudgetedReversibleJitBlock(1U, budget);
    ASSERT_TRUE(first);
    EXPECT_EQ(budget.total_instruction_cycles[0],
        expected_first.cycles + stalls.nonsequential);
    EXPECT_EQ(budget.remaining_cycles, 0U);
    EXPECT_EQ(budget.fetch_end, flash_base + 4U);

    const auto expected_second = reference.stepFast();
    budget.remaining_cycles = expected_second.cycles + stalls.sequential;
    const auto second = cpu.tryStepBudgetedReversibleJitBlock(1U, budget);
    ASSERT_TRUE(second);
    EXPECT_EQ(budget.total_instruction_cycles[0], expected_second.cycles + stalls.sequential);
    EXPECT_EQ(budget.remaining_cycles, 0U);
    EXPECT_EQ(budget.fetch_end, flash_base + 6U);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));

    auto max_bus = basicBus();
    ASSERT_TRUE(max_bus.loadBytes(flash_base, halfwords({0xBF18U, 0x2001U})).hasValue());
    max_bus.setReversibleRamOnly(true);
    max_bus.setAllMmioTrapping(true);
    fil::cpu::CortexM4 max_cpu(max_bus);
    prepare(max_cpu);
    ASSERT_EQ(max_cpu.stepFast().reason, fil::cpu::StopReason::step_complete);
    const auto max_before = max_cpu.state();
    FakeFetchStalls maximal{std::numeric_limits<std::uint16_t>::max(), 0U};
    fil::cpu::CortexM4::ReversibleCycleBudget max_budget{};
    max_budget.remaining_cycles = std::numeric_limits<std::uint16_t>::max();
    max_budget.context = &maximal;
    max_budget.fetch_stall = fakeFetchStall;
    EXPECT_FALSE(max_cpu.tryStepBudgetedReversibleJitBlock(1U, max_budget));
    EXPECT_TRUE(fil::cpu::bitwiseEqual(max_cpu.state(), max_before));
    EXPECT_EQ(max_budget.remaining_cycles, std::numeric_limits<std::uint16_t>::max());
    EXPECT_FALSE(max_budget.have_fetch);
}

TEST(JitTest, UntimedReversibleFallbackExecutesItAndThenItsScalarOp) {
    auto bus = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0xBF18U, 0x3001U})).hasValue());
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    fil::cpu::CortexM4 cpu(bus);
    fil::cpu::CortexM4 reference(bus);
    prepare(cpu);
    prepare(reference);

    const auto it = cpu.tryStepReversibleJitBlock(1U);
    ASSERT_TRUE(it);
    ASSERT_EQ(it->execution.count, 1U);
    EXPECT_EQ(it->execution.pcs[0], flash_base);
    ASSERT_EQ(reference.stepFast().reason, fil::cpu::StopReason::step_complete);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));

    const auto expected = reference.stepFast();
    const auto scalar = cpu.tryStepReversibleJitBlock(1U);
    ASSERT_TRUE(scalar);
    ASSERT_EQ(scalar->execution.count, 1U);
    EXPECT_EQ(scalar->instruction_cycles[0], expected.cycles);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
}

TEST(JitTest, ReversibleItFallbackRejectsUnsafeOpsAndBudgetShortfallIsInert) {
    const std::array<std::vector<std::uint16_t>, 10> unsafe_ops{{
        {0x6008U},                 // store
        {0xEE30U, 0x0A00U},       // VFP
        {0xDF00U},                 // SVC
        {0xBE00U},                 // BKPT
        {0xBF08U},                 // nested IT
        {0x4678U},                 // MOV r0,PC
        {0x4348U},                 // MUL
        {0xFB93U, 0xF1F2U},       // SDIV
        {0xFBB5U, 0xF2F6U},       // UDIV
        {0x4687U}                  // MOV PC,r0
    }};
    for (const auto& op : unsafe_ops) {
        auto bus = basicBus();
        std::vector<std::uint16_t> words{0xBF18U};
        words.insert(words.end(), op.begin(), op.end());
        std::vector<std::uint8_t> code;
        for (const auto word : words) {
            code.push_back(static_cast<std::uint8_t>(word));
            code.push_back(static_cast<std::uint8_t>(word >> 8U));
        }
        ASSERT_TRUE(bus.loadBytes(flash_base, code).hasValue());
        bus.setReversibleRamOnly(true);
        bus.setAllMmioTrapping(true);
        fil::cpu::CortexM4 cpu(bus);
        prepare(cpu);
        ASSERT_EQ(cpu.stepFast().reason, fil::cpu::StopReason::step_complete);
        const auto before = cpu.state();
        fil::cpu::CortexM4::ReversibleCycleBudget budget{};
        budget.remaining_cycles = 100U;
        EXPECT_FALSE(cpu.tryStepBudgetedReversibleJitBlock(1U, budget));
        EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), before));
        EXPECT_EQ(budget.remaining_cycles, 100U);
        EXPECT_FALSE(budget.have_fetch);
    }

    auto bus = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0xBF18U, 0x2001U})).hasValue());
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    ASSERT_EQ(cpu.stepFast().reason, fil::cpu::StopReason::step_complete);
    const auto before = cpu.state();
    fil::cpu::CortexM4::ReversibleCycleBudget budget{};
    budget.remaining_cycles = 0U;
    EXPECT_FALSE(cpu.tryStepBudgetedReversibleJitBlock(1U, budget));
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), before));
    EXPECT_EQ(budget.remaining_cycles, 0U);
    EXPECT_FALSE(budget.have_fetch);
}

TEST(JitTest, ItFallbackRejectsHaltedAndPendingEntriesAndColdNonItAlu) {
    for (const std::uint32_t scenario : {0U, 1U, 2U}) {
        auto bus = basicBus();
        ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0xBF18U, 0x2001U})).hasValue());
        bus.setReversibleRamOnly(true);
        bus.setAllMmioTrapping(true);
        fil::cpu::CortexM4 cpu(bus);
        prepare(cpu);
        ASSERT_EQ(cpu.stepFast().reason, fil::cpu::StopReason::step_complete);
        if (scenario == 0U) cpu.state().halted = true;
        if (scenario == 1U) cpu.state().pending_exception = 16U;
        if (scenario == 2U) cpu.state().pending_exc_return = 0xFFFFFFFDU;
        const auto before = cpu.state();
        EXPECT_FALSE(cpu.tryStepReversibleJitBlock(1U));
        EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), before));
    }

    auto bus = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0x3001U})).hasValue());
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    const auto before = cpu.state();
    EXPECT_FALSE(cpu.tryStepReversibleJitBlock(1U));
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), before));
}

TEST(JitTest, ClassifiesBoundaries) {
    using fil::cpu::InstrKind;
    fil::cpu::DecodedInstruction ins{};
    ins.kind = InstrKind::add;
    ins.rd = 0U;
    EXPECT_EQ(fil::cpu::classifyJitBoundary(ins), fil::cpu::JitBoundary::none);
    ins.rd = 15U;
    EXPECT_EQ(fil::cpu::classifyJitBoundary(ins), fil::cpu::JitBoundary::control_flow);
    ins.kind = InstrKind::ldr;
    ins.rd = 0U;
    EXPECT_EQ(fil::cpu::classifyJitBoundary(ins), fil::cpu::JitBoundary::memory_may_trap);
    ins.kind = InstrKind::b;
    EXPECT_EQ(fil::cpu::classifyJitBoundary(ins), fil::cpu::JitBoundary::control_flow);
    ins.kind = InstrKind::svc;
    EXPECT_EQ(fil::cpu::classifyJitBoundary(ins), fil::cpu::JitBoundary::exception_or_system);
    ins.kind = InstrKind::vadd;
    EXPECT_EQ(fil::cpu::classifyJitBoundary(ins), fil::cpu::JitBoundary::floating_point);
}

TEST(JitTest, PlansMaximalPrefix) {
    using fil::cpu::InstrKind;
    std::array<fil::cpu::DecodedInstruction, 4> seq{};
    seq[0].kind = InstrKind::movw;
    seq[1].kind = InstrKind::add;
    seq[2].kind = InstrKind::cmp;
    seq[3].kind = InstrKind::ldr;
    const auto plan = fil::cpu::planJitBlock(seq);
    EXPECT_EQ(plan.translated_instructions, 4U);
    EXPECT_EQ(plan.boundary, fil::cpu::JitBoundary::none);
    // Memory ops are included in blocks; planner continues through them.
    seq[3].kind = InstrKind::svc;
    const auto plan2 = fil::cpu::planJitBlock(seq);
    EXPECT_EQ(plan2.translated_instructions, 3U);
    EXPECT_EQ(plan2.boundary, fil::cpu::JitBoundary::exception_or_system);
}

TEST(JitTest, BlockExecutesMultiInstructionWithExactSemantics) {
    auto bus = basicBus();
    // movs r0,#16; movs r1,#32; nop; movs r2,#7; nop
    const auto code = halfwords({0x2010U, 0x2120U, 0xBF00U, 0x2207U, 0xBF00U});
    ASSERT_TRUE(bus.loadBytes(flash_base, code).hasValue());
    fil::cpu::CortexM4 cpu(bus);

    // Reference: interpret one pass single-stepped.
    prepare(cpu);
    for (std::int32_t i = 0; i < 5; ++i) {
        const auto s = cpu.stepFast();
        ASSERT_EQ(s.reason, fil::cpu::StopReason::step_complete);
    }
    const auto ref_r0 = cpu.state().r[0];
    const auto ref_r1 = cpu.state().r[1];
    const auto ref_r2 = cpu.state().r[2];
    EXPECT_EQ(ref_r0, 16U);
    EXPECT_EQ(ref_r1, 32U);
    EXPECT_EQ(ref_r2, 7U);

    // Warm the JIT hot counter at the same entry PC, then execute a block.
    std::optional<fil::cpu::CortexM4::JitStepOutcome> block;
    for (std::int32_t i = 0; i < 70; ++i) {
        prepare(cpu);
        block = cpu.tryStepJitBlock();
        if (block && block->count >= 2U) break;
    }
    ASSERT_TRUE(block.has_value()) << "hot path auto-compiles";
    EXPECT_GE(block->count, 2U) << "block covers multiple instructions";
    EXPECT_EQ(block->result.reason, fil::cpu::StopReason::step_complete);
    EXPECT_EQ(cpu.state().r[0], ref_r0) << "JIT block matches interpreter r0";
    EXPECT_EQ(cpu.state().r[1], ref_r1) << "JIT block matches interpreter r1";
    EXPECT_EQ(cpu.state().r[2], ref_r2) << "JIT block matches interpreter r2";
    EXPECT_GT(cpu.jitStats().block_executions, 0U);
    EXPECT_GT(cpu.jitStats().compilations, 0U);
}

TEST(JitTest, BlockMatchesInterpreterOnRamLoadStore) {    auto bus = basicBus();
    // movs r0,#0xAB; lsls r0,r0,#8 ... simpler: movs + str/ldr word via
    // literal-free encoding: use movs r0,#5; str r0,[sp,#0]; ldr r1,[sp,#0]
    // str r0,[sp,#0] = 0x9000|... actually STR rt,[sp,#imm]: 0x9000|(rt<<8)|imm8/4
    // Use rt=0, imm=0 -> 0x9000; LDR r1,[sp,#0] -> 0x9900|... 0x9800|(1<<8)=0x9900? 0x9800 is LDR rt,[sp,#imm]
    const auto code = halfwords({0x2005U, 0x9000U, 0x9900U, 0xBF00U});
    ASSERT_TRUE(bus.loadBytes(flash_base, code).hasValue());
    fil::cpu::CortexM4 cpu(bus);

    prepare(cpu);
    for (std::int32_t i = 0; i < 4; ++i) {
        const auto s = cpu.stepFast();
        ASSERT_EQ(s.reason, fil::cpu::StopReason::step_complete);
    }
    const auto ref_r0 = cpu.state().r[0];
    const auto ref_r1 = cpu.state().r[1];

    std::optional<fil::cpu::CortexM4::JitStepOutcome> block;
    for (std::int32_t i = 0; i < 70; ++i) {
        prepare(cpu);
        block = cpu.tryStepJitBlock();
        if (block && block->count >= 2U) break;
    }
    ASSERT_TRUE(block.has_value()) << "RAM block auto-compiles";
    EXPECT_EQ(cpu.state().r[0], ref_r0);
    EXPECT_EQ(cpu.state().r[1], ref_r1);
}

// Differential test for inlined JIT handlers: the same straight-line code
// must produce bit-identical CPU state and cycle counts via JIT blocks and
// the single-step interpreter across varied registers and flags.
TEST(JitTest, WarmBlockCommitsPrefixAndRestartsMmioInstructionExactlyOnce) {
    constexpr std::uint32_t mmio_base = 0x40000000U;
    auto bus = basicBus();
    SharedMmio device;
    ASSERT_TRUE(bus.mapMmio(mmio_base, 16U, device, "jit-shared-mmio").hasValue());
    // movs r0,#7; ldr r1,[r2]; nop
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0x2007U, 0x6811U, 0xBF00U})).hasValue());
    fil::cpu::CortexM4 cpu(bus);

    // Keep the shared device trapped while warming/compiling so no warm-up
    // execution can commit its read.
    bus.setSharedMmioTrapping(true);
    for (std::int32_t i = 0; i < 60 && !cpu.jitBlockReady(); ++i) {
        prepare(cpu);
        cpu.state().r[2] = mmio_base;
        const auto outcome = cpu.tryStepJitBlock();
        if (!outcome) continue;
        // A compiled block executes immediately; reset to the entry next round.
    }
    prepare(cpu);
    cpu.state().r[2] = mmio_base;
    ASSERT_TRUE(cpu.jitBlockReady());
    const auto pure_prefix = cpu.peekJitBlock();
    ASSERT_TRUE(pure_prefix.has_value());
    EXPECT_EQ(pure_prefix->count, 1U)
        << "exact preview stops before the MMIO load";
    EXPECT_EQ(pure_prefix->pcs[0], flash_base);

    const auto trapped = cpu.tryStepJitBlock();
    ASSERT_TRUE(trapped.has_value());
    EXPECT_EQ(trapped->result.reason, fil::cpu::StopReason::step_complete);
    EXPECT_EQ(trapped->count, 1U) << "only the committed prefix is reported";
    EXPECT_EQ(cpu.state().r[15], flash_base + 2U)
        << "restart PC identifies the trapped LDR, not its fallthrough";
    EXPECT_EQ(cpu.state().r[0], 7U);
    EXPECT_EQ(device.reads, 0) << "trap occurs before device side effects";

    bus.setSharedMmioTrapping(false);
    const auto resumed = cpu.stepFast();
    EXPECT_EQ(resumed.reason, fil::cpu::StopReason::step_complete);
    EXPECT_EQ(cpu.state().r[15], flash_base + 4U);
    EXPECT_EQ(cpu.state().r[1], 0x12345678U);
    EXPECT_EQ(device.reads, 1) << "resumed MMIO access commits exactly once";
}

TEST(JitTest, PrepareBlockIsNonExecutingAndNegativeCachesColdRegions) {
    auto bus = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0xBF00U})).hasValue());
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    const auto initial = cpu.state();

    for (std::int32_t i = 0; i < 55; ++i) {
        EXPECT_FALSE(cpu.prepareJitBlock()) << "single-op region is not compilable";
        EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), initial));
    }
    EXPECT_FALSE(cpu.jitBlockReady());
    // Change the execution generation and make a multi-op region available;
    // the negative cache must be discarded and the hot entry can compile.
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0xBF00U, 0xBF00U})).hasValue());
    EXPECT_TRUE(cpu.prepareJitBlock());
    EXPECT_TRUE(cpu.jitBlockReady());
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), initial));
}

TEST(JitTest, JitExecutionCapAndPurePreviewAreBounded) {
    auto bus = basicBus();
    // movs r0,#1; adds r0,#2; nop; nop
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0x2001U, 0x3002U, 0xBF00U, 0xBF00U})).hasValue());
    fil::cpu::CortexM4 cpu(bus);
    std::optional<fil::cpu::CortexM4::JitStepOutcome> block;
    for (std::int32_t i = 0; i < 60; ++i) {
        prepare(cpu);
        block = cpu.tryStepJitBlock(1U);
        if (cpu.jitStats().compilations != 0U) break;
    }
    ASSERT_TRUE(block.has_value());
    EXPECT_EQ(block->count, 1U);
    EXPECT_EQ(cpu.state().r[15], flash_base + 2U);

    prepare(cpu);
    const auto before = cpu.state();
    EXPECT_FALSE(cpu.tryStepJitBlock(0U).has_value());
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), before))
        << "zero cap does not execute an instruction";

    // Warm the block at its entry, then inspect a capped prefix without running it.
    for (std::int32_t i = 0; i < 60 && !cpu.jitBlockReady(); ++i) {
        prepare(cpu);
        static_cast<void>(cpu.tryStepJitBlock(1U));
    }
    prepare(cpu);
    ASSERT_TRUE(cpu.jitBlockReady());
    const auto preview = cpu.peekJitBlock(2U);
    ASSERT_TRUE(preview.has_value());
    EXPECT_EQ(preview->count, 2U);
    EXPECT_EQ(preview->pcs[0], flash_base);
    EXPECT_EQ(preview->pcs[1], flash_base + 2U);
    fil::cpu::CortexM4::JitBlockPreview fused_preview{};
    ASSERT_TRUE(cpu.prepareAndPeekJitBlock(2U, fused_preview));
    EXPECT_EQ(fused_preview.count, preview->count);
    EXPECT_EQ(fused_preview.max_cycles, preview->max_cycles);
    EXPECT_EQ(fused_preview.exact_cycle_prefix_count,
        preview->exact_cycle_prefix_count);
    for (std::size_t i = 0; i < preview->count; ++i) {
        EXPECT_EQ(fused_preview.pcs[i], preview->pcs[i]);
        EXPECT_EQ(fused_preview.sizes[i], preview->sizes[i]);
        EXPECT_EQ(fused_preview.instruction_cycles[i], preview->instruction_cycles[i]);
    }
    const auto execution = cpu.tryStepJitBlock(2U);
    ASSERT_TRUE(execution.has_value());
    EXPECT_EQ(execution->count, 2U);
    EXPECT_LE(execution->result.cycles, preview->max_cycles);
}

TEST(JitTest, SelfModifyingStoreExitsBlockBeforeStaleOpcode) {
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRom(flash_base, 256U, "jit-test-flash").hasValue());
    ASSERT_TRUE(bus.mapRam(ram_base, 256U, "jit-executable-ram", true).hasValue());
    // nop; strh r0,[r1]; movs r2,#1; nop. The STRH changes the next
    // instruction from movs r2,#1 to movs r2,#9.
    ASSERT_TRUE(bus.loadBytes(ram_base, halfwords({0xBF00U, 0x8008U, 0x2201U, 0xBF00U})).hasValue());
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu, ram_base);
    cpu.state().r[0] = 0x2209U;
    cpu.state().r[1] = ram_base + 4U;

    std::optional<fil::cpu::CortexM4::JitStepOutcome> block;
    for (std::int32_t i = 0; i < 60; ++i) {
        prepare(cpu, ram_base);
        cpu.state().r[0] = 0x2209U;
        cpu.state().r[1] = ram_base + 4U;
        block = cpu.tryStepJitBlock();
        if (block && block->count >= 2U) break;
    }
    ASSERT_TRUE(block.has_value()) << "compilations=" << cpu.jitStats().compilations
        << " executions=" << cpu.jitStats().block_executions
        << " fallbacks=" << cpu.jitStats().fallbacks
        << " pc=0x" << std::hex << cpu.state().r[15];
    EXPECT_EQ(block->count, 2U) << "JIT exits immediately after executable RAM changes";
    EXPECT_EQ(cpu.state().r[15], ram_base + 4U);
    EXPECT_EQ(cpu.state().r[2], 0U) << "stale upcoming opcode was not executed";

    const auto replacement = cpu.stepFast();
    ASSERT_EQ(replacement.reason, fil::cpu::StopReason::step_complete);
    EXPECT_EQ(cpu.state().r[2], 9U) << "next fetch observes the replacement opcode";
    ASSERT_EQ(cpu.stepFast().reason, fil::cpu::StopReason::step_complete);

    fil::mem::MemoryBus reference_bus;
    ASSERT_TRUE(reference_bus.mapRom(flash_base, 256U, "jit-reference-flash").hasValue());
    ASSERT_TRUE(reference_bus.mapRam(ram_base, 256U, "jit-reference-executable-ram", true).hasValue());
    ASSERT_TRUE(reference_bus.loadBytes(
        ram_base, halfwords({0xBF00U, 0x8008U, 0x2201U, 0xBF00U})
    ).hasValue());
    fil::cpu::CortexM4 reference(reference_bus);
    prepare(reference, ram_base);
    reference.state().r[0] = 0x2209U;
    reference.state().r[1] = ram_base + 4U;
    for (std::int32_t i = 0; i < 4; ++i) {
        ASSERT_EQ(reference.stepFast().reason, fil::cpu::StopReason::step_complete);
    }
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
}

TEST(JitTest, SingleInstructionJitMatchesInterpreterAndFallbackCases) {
    auto check = [](const std::vector<std::uint8_t>& code, const auto& configure,
                    const bool warm_cache) {
        auto jit_bus = basicBus();
        auto ref_bus = basicBus();
        ASSERT_TRUE(jit_bus.loadBytes(flash_base, code).hasValue());
        ASSERT_TRUE(ref_bus.loadBytes(flash_base, code).hasValue());
        fil::cpu::CortexM4 jit(jit_bus);
        fil::cpu::CortexM4 reference(ref_bus);
        prepare(jit);
        prepare(reference);
        configure(jit.state());
        configure(reference.state());
        const auto initial = jit.state();
        if (warm_cache) {
            static_cast<void>(jit.stepFast());
            jit.state() = initial;
        }
        const auto actual = jit.stepJitFast();
        const auto expected = reference.stepFast();
        EXPECT_EQ(actual.reason, expected.reason);
        EXPECT_EQ(actual.instruction_address, expected.instruction_address);
        EXPECT_EQ(actual.raw, expected.raw);
        EXPECT_EQ(actual.instruction_size, expected.instruction_size);
        EXPECT_EQ(actual.instructions, expected.instructions);
        EXPECT_EQ(actual.cycles, expected.cycles);
        EXPECT_EQ(actual.suppress_loop_observation, expected.suppress_loop_observation);
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.state(), reference.state()));
        if (expected.reason != fil::cpu::StopReason::step_complete) {
            EXPECT_EQ(jit.lastDiagnostic().instruction_address,
                      reference.lastDiagnostic().instruction_address);
            EXPECT_EQ(jit.lastDiagnostic().raw, reference.lastDiagnostic().raw);
            EXPECT_EQ(jit.lastDiagnostic().message, reference.lastDiagnostic().message);
        }
    };

    // Specialized immediate operation and direct branch.
    check(halfwords({0x2007U}), [](fil::cpu::CpuState&) {}, true);
    check(halfwords({0xE000U}), [](fil::cpu::CpuState&) {}, true);
    // Failed conditional branch stays on the interpreter path.
    check(halfwords({0xD100U}), [](fil::cpu::CpuState& state) {
        state.xpsr |= fil::cpu::xpsr_z;
    }, true);
    // Active IT state also delegates without pre-mutating architectural state.
    check(halfwords({0x2001U}), [](fil::cpu::CpuState& state) {
        state.setItState(0x08U);
    }, true);
}

TEST(JitTest, WarmSingleCacheRespectsCodeAndArchitecturalStateChanges) {
    auto jit_bus = basicBus();
    auto ref_bus = basicBus();
    for (auto* bus : {&jit_bus, &ref_bus}) {
        ASSERT_TRUE(bus->loadBytes(flash_base, halfwords({0x2001U})).hasValue());
    }
    fil::cpu::CortexM4 jit(jit_bus);
    fil::cpu::CortexM4 reference(ref_bus);
    prepare(jit);
    ASSERT_EQ(jit.stepJitFast().reason, fil::cpu::StopReason::step_complete);
    for (auto* bus : {&jit_bus, &ref_bus}) {
        ASSERT_TRUE(bus->loadBytes(flash_base, halfwords({0x2009U})).hasValue());
    }
    for (const std::uint32_t scenario : {0U, 1U, 2U, 3U, 4U}) {
        prepare(jit);
        if (scenario == 1U) jit.state().setItState(0x08U); // Failed EQ must not write r0.
        if (scenario == 2U) jit.state().halted = true;
        if (scenario == 3U) jit.state().thumb = false;
        if (scenario == 4U) jit.state().xpsr &= ~fil::cpu::xpsr_t;
        reference.state() = jit.state();
        const auto actual = jit.stepJitFast();
        const auto expected = reference.stepFast();
        EXPECT_EQ(actual.reason, expected.reason);
        EXPECT_EQ(actual.raw, expected.raw);
        EXPECT_EQ(actual.instructions, expected.instructions);
        EXPECT_EQ(actual.cycles, expected.cycles);
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.state(), reference.state()));
        if (scenario == 0U) {
            EXPECT_EQ(jit.state().r[0], 9U);
        }
        if (actual.reason != fil::cpu::StopReason::step_complete) {
            EXPECT_EQ(jit.lastDiagnostic().message, reference.lastDiagnostic().message);
        }
    }
}

TEST(JitTest, CachedUnconditionalFastPathMatchesInterpreter) {
    for (const auto& code : {
            halfwords({0x3001U}), // ADDS r0,#1
            halfwords({0x6808U}), // LDR r0,[r1]
            halfwords({0xE000U}), // B to the instruction after fallthrough
        }) {
        auto jit_bus = basicBus();
        auto ref_bus = basicBus();
        ASSERT_TRUE(jit_bus.loadBytes(flash_base, code).hasValue());
        ASSERT_TRUE(ref_bus.loadBytes(flash_base, code).hasValue());
        fil::cpu::CortexM4 jit(jit_bus);
        fil::cpu::CortexM4 reference(ref_bus);
        prepare(jit);
        prepare(reference);
        jit.state().r[0] = reference.state().r[0] = 0xffffffffU;
        jit.state().r[1] = reference.state().r[1] = ram_base;
        ASSERT_TRUE(jit_bus.write32(ram_base, 0x12345678U).hasValue());
        ASSERT_TRUE(ref_bus.write32(ram_base, 0x12345678U).hasValue());
        for (std::uint32_t i = 0U; i < 4U; ++i) {
            const auto actual = jit.stepJitFast();
            const auto expected = reference.stepFast();
            EXPECT_EQ(actual.reason, expected.reason);
            EXPECT_EQ(actual.raw, expected.raw);
            EXPECT_EQ(actual.instruction_size, expected.instruction_size);
            EXPECT_EQ(actual.instructions, expected.instructions);
            EXPECT_EQ(actual.cycles, expected.cycles);
            EXPECT_EQ(actual.suppress_loop_observation,
                expected.suppress_loop_observation);
            EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.state(), reference.state()));
            if (actual.reason != fil::cpu::StopReason::step_complete) break;
            jit.state().r[15] = flash_base;
            reference.state().r[15] = flash_base;
        }
    }
}

TEST(JitTest, SingleInstructionJitFallsBackForMmioAndFaults) {
    auto compare_fallback = [](const bool mapped_mmio, const bool trap) {
        constexpr std::uint32_t mmio_base = 0x40000000U;
        auto jit_bus = basicBus();
        auto ref_bus = basicBus();
        SharedMmio jit_device;
        SharedMmio ref_device;
        if (mapped_mmio) {
            ASSERT_TRUE(jit_bus.mapMmio(mmio_base, 16U, jit_device, "jit-mmio").hasValue());
            ASSERT_TRUE(ref_bus.mapMmio(mmio_base, 16U, ref_device, "ref-mmio").hasValue());
        }
        ASSERT_TRUE(jit_bus.loadBytes(flash_base, halfwords({0x6811U})).hasValue());
        ASSERT_TRUE(ref_bus.loadBytes(flash_base, halfwords({0x6811U})).hasValue());
        jit_bus.setSharedMmioTrapping(trap);
        ref_bus.setSharedMmioTrapping(trap);
        fil::cpu::CortexM4 jit(jit_bus);
        fil::cpu::CortexM4 reference(ref_bus);
        prepare(jit);
        prepare(reference);
        jit.state().r[2] = mapped_mmio ? mmio_base : 0x50000000U;
        reference.state().r[2] = jit.state().r[2];
        const auto initial = jit.state();
        static_cast<void>(jit.stepFast()); // Cache decode; may trap or fault.
        jit.state() = initial;
        jit_device.reads = 0;
        const auto actual = jit.stepJitFast();
        const auto expected = reference.stepFast();
        EXPECT_EQ(actual.reason, expected.reason);
        EXPECT_EQ(actual.raw, expected.raw);
        EXPECT_EQ(actual.instructions, expected.instructions);
        EXPECT_EQ(actual.cycles, expected.cycles);
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.state(), reference.state()));
        EXPECT_EQ(jit_device.reads, ref_device.reads);
    };
    compare_fallback(true, true);
    compare_fallback(false, false);
}

TEST(JitTest, RegisterAluSpecializationsMatchInterpreterAcrossSeeds) {
    struct Encoding {
        std::vector<std::uint8_t> bytes;
        fil::cpu::InstrKind kind;
    };
    const std::vector<Encoding> encodings{
        {halfwords({0x4608U}), fil::cpu::InstrKind::mov}, // MOV r0,r1
        {halfwords({0x1888U}), fil::cpu::InstrKind::add}, // ADD r0,r1,r2
        {halfwords({0x1A88U}), fil::cpu::InstrKind::sub}, // SUB r0,r1,r2
        {halfwords({0x4148U}), fil::cpu::InstrKind::adc}, // ADCS r0,r1
        {halfwords({0x4188U}), fil::cpu::InstrKind::sbc}, // SBCS r0,r1
        {halfwords({0x448DU}), fil::cpu::InstrKind::add}, // ADD sp,r1
        {halfwords({0x4668U}), fil::cpu::InstrKind::mov}, // MOV r0,sp
        {halfwords({0x4288U}), fil::cpu::InstrKind::cmp}, // CMP r0,r1
        {halfwords({0x42C8U}), fil::cpu::InstrKind::cmn}, // CMN r0,r1
        {halfwords({0x4008U}), fil::cpu::InstrKind::and_},
        {halfwords({0x4048U}), fil::cpu::InstrKind::eor},
        {halfwords({0x4308U}), fil::cpu::InstrKind::orr},
        {halfwords({0x4388U}), fil::cpu::InstrKind::bic},
        {halfwords({0x43C8U}), fil::cpu::InstrKind::mvn},
        {halfwords({0x4348U}), fil::cpu::InstrKind::mul},
        {halfwords({0x4088U}), fil::cpu::InstrKind::lsl},
        {halfwords({0x40C8U}), fil::cpu::InstrKind::lsr},
        {halfwords({0x4108U}), fil::cpu::InstrKind::asr},
        {halfwords({0x41C8U}), fil::cpu::InstrKind::ror},
        {halfwords({0xFA03U, 0xF000U}), fil::cpu::InstrKind::lsl}, // Thumb-2 register shift
        {halfwords({0xEBB6U, 0x0FA5U}), fil::cpu::InstrKind::cmp}, // CMP.W r6,r5,ASR #2
        {halfwords({0xEA4FU, 0x0C93U}), fil::cpu::InstrKind::mov}, // MOV.W r12,r3,LSR #2
        {halfwords({0xEA11U, 0x0082U}), fil::cpu::InstrKind::and_}, // ANDS.W r0,r1,r2,LSL #2
        {halfwords({0xEA61U, 0x0082U}), fil::cpu::InstrKind::orn}, // ORN.W r0,r1,r2,LSL #2
        {halfwords({0xEB11U, 0x0082U}), fil::cpu::InstrKind::add}, // ADDS.W r0,r1,r2,LSL #2
        {halfwords({0xEBB1U, 0x0082U}), fil::cpu::InstrKind::sub}, // SUBS.W r0,r1,r2,LSL #2
        {halfwords({0xEB51U, 0x0082U}), fil::cpu::InstrKind::adc}, // ADCS.W r0,r1,r2,LSL #2
        {halfwords({0xEB71U, 0x0082U}), fil::cpu::InstrKind::sbc}, // SBCS.W r0,r1,r2,LSL #2
        {halfwords({0xEB0DU, 0x0001U}), fil::cpu::InstrKind::add}, // ADD.W r0,sp,r1
        {halfwords({0x4248U}), fil::cpu::InstrKind::rsb}, // RSBS r0,r1,#0
        {halfwords({0x421AU}), fil::cpu::InstrKind::tst}, // TST r2,r3
        {halfwords({0xFB01U, 0x3002U}), fil::cpu::InstrKind::mla}, // MLA r0,r1,r2,r3
        {halfwords({0xFBA2U, 0x0103U}), fil::cpu::InstrKind::umull}, // UMULL r0,r1,r2,r3
        {halfwords({0xFAB1U, 0xF081U}), fil::cpu::InstrKind::clz}, // CLZ r0,r1
        {halfwords({0xBA08U}), fil::cpu::InstrKind::rev}, // REV r0,r1
        {halfwords({0xBA48U}), fil::cpu::InstrKind::rev16}, // REV16 r0,r1
        {halfwords({0xBAC8U}), fil::cpu::InstrKind::revsh}, // REVSH r0,r1
        {halfwords({0xF36FU, 0x0007U}), fil::cpu::InstrKind::bfc}, // BFC r0,#0,#8
        {halfwords({0xF3C1U, 0x2007U}), fil::cpu::InstrKind::ubfx}, // UBFX r0,r1,#8,#8
        {halfwords({0xB248U}), fil::cpu::InstrKind::sxtb}, // SXTB r0,r1
        {halfwords({0xB2C8U}), fil::cpu::InstrKind::uxtb}, // UXTB r0,r1
        {halfwords({0xB208U}), fil::cpu::InstrKind::sxth}, // SXTH r0,r1
        {halfwords({0xB288U}), fil::cpu::InstrKind::uxth}, // UXTH r0,r1
        {halfwords({0xFA81U, 0xF042U}), fil::cpu::InstrKind::uadd8}, // UADD8 r0,r1,r2
        {halfwords({0xFAA1U, 0xF082U}), fil::cpu::InstrKind::sel}, // SEL r0,r1,r2
        {halfwords({0xD0FDU}), fil::cpu::InstrKind::b}, // BEQ.N (conditional branch)
        {halfwords({0xF47FU, 0xAFFCU}), fil::cpu::InstrKind::b}, // BNE.W (conditional branch)
        {halfwords({0x4708U}), fil::cpu::InstrKind::bx}, // BX r1 (declines unless Thumb target)
        {halfwords({0x6808U}), fil::cpu::InstrKind::ldr}, // LDR r0,[r1] (RAM/decline)
        {halfwords({0x6048U}), fil::cpu::InstrKind::str}, // STR r0,[r1,#4]
        {halfwords({0x7848U}), fil::cpu::InstrKind::ldrb}, // LDRB r0,[r1,#1]
        {halfwords({0x8048U}), fil::cpu::InstrKind::strh}, // STRH r0,[r1,#2]
        {halfwords({0xE891U, 0x0005U}), fil::cpu::InstrKind::ldm}, // LDMIA.W r1,{r0,r2}
        {halfwords({0xE881U, 0x0005U}), fil::cpu::InstrKind::stm}, // STMIA.W r1,{r0,r2}
        {halfwords({0xB4F0U}), fil::cpu::InstrKind::push}, // PUSH {r4-r7}
        {halfwords({0xBCF0U}), fil::cpu::InstrKind::pop}, // POP {r4-r7}
        {halfwords({0xBDF0U}), fil::cpu::InstrKind::pop}, // POP {r4-r7,pc} (return)
    };
    for (const auto& encoding : encodings) {
        std::optional<fil::cpu::DecodedInstruction> decoded;
        if (encoding.bytes.size() == 2U) {
            const std::uint16_t raw = static_cast<std::uint16_t>(
                static_cast<std::uint16_t>(encoding.bytes[0])
                | (static_cast<std::uint16_t>(encoding.bytes[1]) << 8U));
            decoded = fil::cpu::decode16(raw);
        } else {
            const std::uint16_t first = static_cast<std::uint16_t>(
                static_cast<std::uint16_t>(encoding.bytes[0])
                | (static_cast<std::uint16_t>(encoding.bytes[1]) << 8U));
            const std::uint16_t second = static_cast<std::uint16_t>(
                static_cast<std::uint16_t>(encoding.bytes[2])
                | (static_cast<std::uint16_t>(encoding.bytes[3]) << 8U));
            decoded = fil::cpu::decode32(first, second);
        }
        ASSERT_TRUE(decoded.has_value());
        ASSERT_EQ(decoded->kind, encoding.kind);

        for (std::uint32_t trial = 0; trial < 20U; ++trial) {
            auto jit_bus = basicBus();
            auto ref_bus = basicBus();
            ASSERT_TRUE(jit_bus.loadBytes(flash_base, encoding.bytes).hasValue());
            ASSERT_TRUE(ref_bus.loadBytes(flash_base, encoding.bytes).hasValue());
            fil::cpu::CortexM4 jit(jit_bus);
            fil::cpu::CortexM4 reference(ref_bus);
            prepare(jit);
            prepare(reference);
            std::uint32_t random = 0xA511E9B3U
                ^ (trial * 0x9E3779B9U)
                ^ static_cast<std::uint32_t>(encoding.kind);
            for (std::size_t reg = 0; reg < 13U; ++reg) {
                random = random * 1664525U + 1013904223U;
                jit.state().r[reg] = random ^ (random >> 11U);
                reference.state().r[reg] = jit.state().r[reg];
            }
            random = random * 1664525U + 1013904223U;
            jit.state().xpsr = fil::cpu::xpsr_t | (random & 0xF0000000U);
            reference.state().xpsr = jit.state().xpsr;
            jit.state().control = (trial & 1U) != 0U ? 2U : 0U;
            jit.state().psp = ram_base + 0xc0U;
            jit.state().r[13] = jit.state().control != 0U ? jit.state().psp : jit.state().msp;
            reference.state() = jit.state();
            const auto initial = jit.state();
            static_cast<void>(jit.stepFast()); // Populate decoded specialization metadata.
            jit.state() = initial;

            const auto actual = jit.stepJitFast();
            const auto expected = reference.stepFast();
            EXPECT_EQ(actual.reason, expected.reason);
            EXPECT_EQ(actual.raw, expected.raw);
            EXPECT_EQ(actual.instruction_size, expected.instruction_size);
            EXPECT_EQ(actual.instructions, expected.instructions);
            EXPECT_EQ(actual.cycles, expected.cycles);
            EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.state(), reference.state()))
                << "kind=" << static_cast<std::int32_t>(encoding.kind) << " trial=" << trial;
        }
    }
}

TEST(JitTest, CachedArithmeticPreservesNzcvAtCarryAndOverflowEdges) {
    for (const auto opcode : {0x1840U, 0x1a40U, 0x4148U, 0x4188U, 0x4288U, 0x4248U}) {
        auto jit_bus = basicBus();
        auto ref_bus = basicBus();
        for (auto* bus : {&jit_bus, &ref_bus}) {
            ASSERT_TRUE(bus->loadBytes(flash_base,
                halfwords({static_cast<std::uint16_t>(opcode)})).hasValue());
        }
        fil::cpu::CortexM4 jit(jit_bus);
        fil::cpu::CortexM4 reference(ref_bus);
        prepare(jit);
        static_cast<void>(jit.stepFast()); // Warm the cache, then vary its live operands.
        for (const auto left : {0U, 1U, 0x7fffffffU, 0x80000000U, 0xffffffffU}) {
            for (const auto right : {0U, 1U, 0x7fffffffU, 0x80000000U, 0xffffffffU}) {
                for (const bool carry : {false, true}) {
                    prepare(jit);
                    jit.state().r[0] = left;
                    jit.state().r[1] = right;
                    jit.state().xpsr |= fil::cpu::xpsr_q | fil::cpu::xpsr_ge_mask
                        | fil::cpu::xpsr_n | fil::cpu::xpsr_z | fil::cpu::xpsr_v;
                    if (carry) jit.state().xpsr |= fil::cpu::xpsr_c;
                    reference.state() = jit.state();
                    const auto actual = jit.stepJitFast();
                    const auto expected = reference.stepFast();
                    EXPECT_EQ(actual.reason, expected.reason);
                    EXPECT_EQ(actual.cycles, expected.cycles);
                    EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.state(), reference.state()))
                        << "opcode=" << std::hex << opcode << " left=" << left
                        << " right=" << right << " carry=" << carry;
                }
            }
        }
    }
}

TEST(JitTest, MemoryHandlersMatchInterpreterAcrossSeeds) {
    struct Encoding {
        std::vector<std::uint8_t> bytes;
        fil::cpu::InstrKind kind;
    };
    const std::vector<Encoding> encodings{
        {halfwords({0x6808U}), fil::cpu::InstrKind::ldr}, // LDR r0,[r1]
        {halfwords({0x6048U}), fil::cpu::InstrKind::str}, // STR r0,[r1,#4]
        {halfwords({0x7848U}), fil::cpu::InstrKind::ldrb}, // LDRB r0,[r1,#1]
        {halfwords({0x8048U}), fil::cpu::InstrKind::strh}, // STRH r0,[r1,#2]
        {halfwords({0xE891U, 0x0005U}), fil::cpu::InstrKind::ldm}, // LDMIA.W r1,{r0,r2}
        {halfwords({0xE881U, 0x0005U}), fil::cpu::InstrKind::stm}, // STMIA.W r1,{r0,r2}
        {halfwords({0xB4F0U}), fil::cpu::InstrKind::push}, // PUSH {r4-r7}
        {halfwords({0xBCF0U}), fil::cpu::InstrKind::pop}, // POP {r4-r7}
        {halfwords({0xBDF0U}), fil::cpu::InstrKind::pop}, // POP {r4-r7,pc}
        {halfwords({0x4708U}), fil::cpu::InstrKind::bx}, // BX r1
    };
    for (const auto& encoding : encodings) {
        for (std::uint32_t trial = 0U; trial < 25U; ++trial) {
            auto jit_bus = basicBus();
            auto ref_bus = basicBus();
            ASSERT_TRUE(jit_bus.loadBytes(flash_base, encoding.bytes).hasValue());
            ASSERT_TRUE(ref_bus.loadBytes(flash_base, encoding.bytes).hasValue());
            // Seed RAM identically so success paths compare data, not just state.
            std::uint32_t seed = 0x12345678U ^ (static_cast<std::uint32_t>(trial) * 0x9E3779B1U);
            for (std::uint32_t addr = ram_base; addr < ram_base + 256U; addr += 4U) {
                seed = seed * 1664525U + 1013904223U;
                const std::uint32_t word = seed ^ (seed >> 13U);
                const std::vector<std::uint8_t> word_bytes{
                    static_cast<std::uint8_t>(word), static_cast<std::uint8_t>(word >> 8U),
                    static_cast<std::uint8_t>(word >> 16U),
                    static_cast<std::uint8_t>(word >> 24U)};
                ASSERT_TRUE(jit_bus.loadBytes(addr, word_bytes).hasValue());
                ASSERT_TRUE(ref_bus.loadBytes(addr, word_bytes).hasValue());
            }
            fil::cpu::CortexM4 jit(jit_bus);
            fil::cpu::CortexM4 reference(ref_bus);
            prepare(jit);
            prepare(reference);
            // AimMemOps at RAM (success), alternating SP-relative and r1-based.
            std::uint32_t s = 0xABCDEF01U ^ (static_cast<std::uint32_t>(trial) * 0x85EBCA6BU);
            for (std::size_t r = 0; r < 13U; ++r) {
                s = s * 1664525U + 1013904223U;
                jit.state().r[r] = ram_base + (s % 240U);
                reference.state().r[r] = jit.state().r[r];
            }
            jit.state().r[13] = ram_base + 0xC0U;
            reference.state().r[13] = ram_base + 0xC0U;
            // BX targets: alternate valid Thumb RAM/flash targets and faults.
            if (encoding.kind == fil::cpu::InstrKind::bx) {
                jit.state().r[1] = (trial % 2U == 0U)
                    ? (flash_base + 1U) : (ram_base + 0x33U);
                reference.state().r[1] = jit.state().r[1];
            }
            // POP {r4-r7,pc}: plant the PC word (5th transfer word) covering
            // valid Thumb targets, exception-return tokens, and faults.
            if (encoding.bytes.size() == 2U && encoding.bytes[1] == 0xBDU) {
                const std::uint32_t target = (trial % 3U == 0U)
                    ? (flash_base + 1U)
                    : (trial % 3U == 1U ? 0xFFFFFFFDU : (ram_base + 2U));
                auto poke_pc = [&](fil::mem::MemoryBus& bus) {
                    const std::vector<std::uint8_t> target_bytes{
                        static_cast<std::uint8_t>(target),
                        static_cast<std::uint8_t>(target >> 8U),
                        static_cast<std::uint8_t>(target >> 16U),
                        static_cast<std::uint8_t>(target >> 24U)};
                    EXPECT_TRUE(bus.loadBytes(ram_base + 0xC0U + 16U, target_bytes)
                        .hasValue());
                };
                poke_pc(jit_bus);
                poke_pc(ref_bus);
            }
            const auto initial = jit.state();
            static_cast<void>(jit.stepFast()); // Populate decode metadata.
            jit.state() = initial;
            const auto actual = jit.stepJitFast();
            const auto expected = reference.stepFast();
            EXPECT_EQ(actual.reason, expected.reason);
            EXPECT_EQ(actual.instructions, expected.instructions);
            EXPECT_EQ(actual.cycles, expected.cycles);
            EXPECT_EQ(actual.suppress_loop_observation, expected.suppress_loop_observation);
            EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.state(), reference.state()))
                << "kind=" << static_cast<std::int32_t>(encoding.kind) << " trial=" << trial;
            // Compare full RAM contents for data-path equivalence.
            for (std::uint32_t addr = ram_base; addr < ram_base + 256U; addr += 4U) {
                const auto jaw = jit_bus.read32(addr, {});
                const auto raw = ref_bus.read32(addr, {});
                ASSERT_TRUE(jaw.hasValue() && raw.hasValue());
                EXPECT_EQ(jaw.value(), raw.value())
                    << "RAM mismatch at 0x" << std::hex << addr;
            }
        }
    }
}

TEST(JitTest, SpecialProgramCounterAluFormsStayOnInterpreterPath) {
    for (const std::uint16_t raw : std::array<std::uint16_t, 2>{0x4687U, 0x4487U}) {
        auto jit_bus = basicBus();
        auto ref_bus = basicBus();
        ASSERT_TRUE(jit_bus.loadBytes(flash_base, halfwords({raw})).hasValue());
        ASSERT_TRUE(ref_bus.loadBytes(flash_base, halfwords({raw})).hasValue());
        fil::cpu::CortexM4 jit(jit_bus);
        fil::cpu::CortexM4 reference(ref_bus);
        prepare(jit);
        prepare(reference);
        jit.state().r[0] = flash_base + 0x21U;
        reference.state().r[0] = jit.state().r[0];
        const auto initial = jit.state();
        static_cast<void>(jit.stepFast());
        jit.state() = initial;
        const auto actual = jit.stepJitFast();
        const auto expected = reference.stepFast();
        EXPECT_EQ(actual.reason, expected.reason);
        EXPECT_EQ(actual.cycles, expected.cycles);
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.state(), reference.state()));
    }
}

TEST(JitTest, InlinedHandlersMatchInterpreter) {
    // movs r0,#0x12; adds r0,#0x34; subs r1,#1; cmp r2,#0x45;
    // movs r0,#0; nop; adds r3,r1,r2 (register form, generic path)
    const auto code = halfwords(
        {0x2012U, 0x3034U, 0x3901U, 0x2A45U, 0x2000U, 0xBF00U, 0x188BU});
    for (std::int32_t trial = 0; trial < 25; ++trial) {
        const std::uint32_t seed = 0x243F6A88U ^ (static_cast<std::uint32_t>(trial) * 0x9E3779B1U);
        auto init = [&](fil::cpu::CortexM4& cpu) {
            prepare(cpu);
            std::uint32_t s = seed;
            for (std::size_t r = 0; r < 8U; ++r) {
                s = s * 1664525U + 1013904223U;
                cpu.state().r[r] = s ^ (s >> 13U);
            }
            s = s * 1664525U + 1013904223U;
            // Keep T bit set, vary N/Z/C/V (exclude Q/GE/IT/ISR noise is
            // fine: interpreter and JIT see identical xpsr).
            cpu.state().xpsr = 0x01000000U | (s & 0xF00FF000U);
        };
        auto ref_bus = basicBus();
        ASSERT_TRUE(ref_bus.loadBytes(flash_base, code).hasValue());
        fil::cpu::CortexM4 ref(ref_bus);
        init(ref);
        std::uint64_t ref_cycles = 0;
        for (std::int32_t i = 0; i < 7; ++i) {
            const auto s = ref.stepFast();
            ASSERT_EQ(s.reason, fil::cpu::StopReason::step_complete);
            ref_cycles += s.cycles;
        }
        auto jit_bus = basicBus();
        ASSERT_TRUE(jit_bus.loadBytes(flash_base, code).hasValue());
        fil::cpu::CortexM4 jit(jit_bus);
        // Warm up compilation, then run once from the trial state.
        std::optional<fil::cpu::CortexM4::JitStepOutcome> block;
        for (std::int32_t i = 0; i < 60; ++i) {
            init(jit);
            block = jit.tryStepJitBlock();
            if (block && block->count == 7U) break;
        }
        ASSERT_TRUE(block.has_value() && block->count == 7U)
            << "trial " << trial << ": full 7-op block compiles";
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.state(), ref.state()))
            << "trial " << trial << ": JIT state matches interpreter";
        EXPECT_EQ(block->result.cycles, static_cast<std::uint16_t>(ref_cycles))
            << "trial " << trial << ": JIT cycles match interpreter";
    }
}

TEST(JitTest, ExactPreviewAgreesWithExecutionIncludingBranchFallthrough) {
    const std::array programs{
        halfwords({0xBF00U, 0xE7FDU}), // nop; unconditional backwards branch
        halfwords({0xBF00U, 0xE7FFU}), // nop; branch target equals fallthrough
        halfwords({0xBF00U, 0xF000U, 0xF800U}), // nop; BL target equals fallthrough
        halfwords({0xBF00U, 0xF000U, 0xB800U}), // nop; B.W target equals fallthrough
        halfwords({0x3001U, 0x4048U, 0xE7FCU}), // add; eor; backwards branch
    };
    for (const auto& code : programs) {
        auto bus = basicBus();
        ASSERT_TRUE(bus.loadBytes(flash_base, code));
        fil::cpu::CortexM4 cpu(bus);
        prepare(cpu);
        for (std::uint32_t i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
        for (const auto limit : {2U, 16U}) {
            prepare(cpu);
            cpu.state().r[0] = 0xffffffffU;
            cpu.state().r[1] = 0x12345678U;
            const auto preview = cpu.peekJitBlock(limit);
            ASSERT_TRUE(preview);
            ASSERT_TRUE(preview->cycles_exact);
            fil::cpu::CortexM4::JitBlockPreview fused_preview{};
            ASSERT_TRUE(cpu.prepareAndPeekExactJitBlock(limit, fused_preview));
            EXPECT_EQ(fused_preview.count, preview->count);
            EXPECT_EQ(fused_preview.max_cycles, preview->max_cycles);
            EXPECT_EQ(fused_preview.exact_cycle_prefix_count,
                preview->exact_cycle_prefix_count);
            for (std::size_t i = 0; i < preview->count; ++i) {
                EXPECT_EQ(fused_preview.pcs[i], preview->pcs[i]);
                EXPECT_EQ(fused_preview.sizes[i], preview->sizes[i]);
                EXPECT_EQ(fused_preview.instruction_cycles[i],
                    preview->instruction_cycles[i]);
            }
            const auto result = cpu.tryStepJitBlock(limit);
            ASSERT_TRUE(result);
            EXPECT_EQ(result->result.reason, fil::cpu::StopReason::step_complete);
            EXPECT_EQ(result->result.instructions, preview->count);
            EXPECT_EQ(result->result.cycles, preview->max_cycles);
        }
    }
    auto bus = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0xBF00U, 0xB100U}))); // CBZ
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    for (std::uint32_t i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
    const auto preview = cpu.peekJitBlock();
    ASSERT_TRUE(preview);
    EXPECT_FALSE(preview->cycles_exact);
}

TEST(JitTest, LookaheadFaultsRemainPreciseAndDoNotFaultDuringPreparation) {
    for (const bool missing_wide_halfword : {false, true}) {
        auto ref_bus = basicBus();
        auto jit_bus = basicBus();
        const auto code = missing_wide_halfword
            ? halfwords({0xBF00U, 0xBF00U, 0xF000U})
            : halfwords({0xBF00U, 0xBF00U});
        const std::uint32_t start = flash_base + 256U - static_cast<std::uint32_t>(code.size());
        ASSERT_TRUE(ref_bus.loadBytes(start, code));
        ASSERT_TRUE(jit_bus.loadBytes(start, code));
        fil::cpu::CortexM4 reference(ref_bus);
        fil::cpu::CortexM4 jit(jit_bus);
        prepare(reference, start);
        prepare(jit, start);
        const auto initial = jit.state();
        for (std::uint32_t i = 0U; i < 70U; ++i) static_cast<void>(jit.prepareJitBlock());
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.state(), initial));
        const auto block = jit.tryStepJitBlock();
        ASSERT_TRUE(block);
        ASSERT_EQ(block->count, 2U);
        for (std::uint32_t i = 0U; i < 2U; ++i) {
            ASSERT_EQ(reference.stepFast().reason, fil::cpu::StopReason::step_complete);
        }
        const auto exact_fault = reference.step();
        const auto jit_fault = jit.step();
        EXPECT_EQ(jit_fault.reason, exact_fault.reason);
        EXPECT_EQ(jit_fault.instructions, exact_fault.instructions);
        EXPECT_EQ(jit_fault.cycles, exact_fault.cycles);
        EXPECT_EQ(jit_fault.diagnostic.instruction_address, exact_fault.diagnostic.instruction_address);
        EXPECT_EQ(jit_fault.diagnostic.raw, exact_fault.diagnostic.raw);
        EXPECT_EQ(jit_fault.diagnostic.instruction_size, exact_fault.diagnostic.instruction_size);
        EXPECT_EQ(jit_fault.diagnostic.message, exact_fault.diagnostic.message);
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.state(), reference.state()));
    }
}

} // namespace
