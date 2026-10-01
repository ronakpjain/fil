#include "fil/cpu/cortex_m4.hpp"
#include "fil/cpu/decoder.hpp"
#include "fil/cpu/jit.hpp"
#include "fil/mem/memory_bus.hpp"

#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
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

    int reads{0};
};

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
    for (unsigned int i = 0U; i < 70U; ++i) {
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
    for (unsigned int i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
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
    for (unsigned int i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
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
    for (unsigned int i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
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
    for (unsigned int i = 0U; i < 70U; ++i) static_cast<void>(fault_cpu.prepareJitBlock());
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
    for (unsigned int i = 0U; i < 70U; ++i) {
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
    for (unsigned int i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());

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
    for (unsigned int i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
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
    for (unsigned int i = 0U; i < 4U; ++i) {
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

TEST(JitTest, BudgetedReversibleBlockRejectsBeforeTakenBranchSideEffects) {
    auto bus = basicBus();
    ASSERT_TRUE(bus.loadBytes(flash_base, halfwords({0xBF00U, 0xE7FFU})).hasValue());
    bus.setReversibleRamOnly(true);
    bus.setAllMmioTrapping(true);
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    for (unsigned int i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());

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
    for (unsigned int i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
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
    for (unsigned int i = 0U; i < 70U; ++i) {
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
    for (int i = 0; i < 5; ++i) {
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
    for (int i = 0; i < 70; ++i) {
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
    for (int i = 0; i < 4; ++i) {
        const auto s = cpu.stepFast();
        ASSERT_EQ(s.reason, fil::cpu::StopReason::step_complete);
    }
    const auto ref_r0 = cpu.state().r[0];
    const auto ref_r1 = cpu.state().r[1];

    std::optional<fil::cpu::CortexM4::JitStepOutcome> block;
    for (int i = 0; i < 70; ++i) {
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
    for (int i = 0; i < 60 && !cpu.jitBlockReady(); ++i) {
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

    for (int i = 0; i < 55; ++i) {
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
    for (int i = 0; i < 60; ++i) {
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
    for (int i = 0; i < 60 && !cpu.jitBlockReady(); ++i) {
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
    for (int i = 0; i < 60; ++i) {
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
    for (int i = 0; i < 4; ++i) {
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
        jit.setNativeSingleInstructionJitEnabled(false);
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
    jit.setNativeSingleInstructionJitEnabled(false);
    prepare(jit);
    ASSERT_EQ(jit.stepJitFast().reason, fil::cpu::StopReason::step_complete);
    for (auto* bus : {&jit_bus, &ref_bus}) {
        ASSERT_TRUE(bus->loadBytes(flash_base, halfwords({0x2009U})).hasValue());
    }
    for (const unsigned int scenario : {0U, 1U, 2U, 3U, 4U}) {
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
        if (scenario == 0U) EXPECT_EQ(jit.state().r[0], 9U);
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
        jit.setNativeSingleInstructionJitEnabled(false);
        prepare(jit);
        prepare(reference);
        jit.state().r[0] = reference.state().r[0] = 0xffffffffU;
        jit.state().r[1] = reference.state().r[1] = ram_base;
        ASSERT_TRUE(jit_bus.write32(ram_base, 0x12345678U).hasValue());
        ASSERT_TRUE(ref_bus.write32(ram_base, 0x12345678U).hasValue());
        for (unsigned int i = 0U; i < 4U; ++i) {
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
        jit.setNativeSingleInstructionJitEnabled(false);
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
            jit.setNativeSingleInstructionJitEnabled(false);
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
                << "kind=" << static_cast<int>(encoding.kind) << " trial=" << trial;
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
        jit.setNativeSingleInstructionJitEnabled(false);
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
        for (unsigned int trial = 0U; trial < 25U; ++trial) {
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
            jit.setNativeSingleInstructionJitEnabled(false);
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
                << "kind=" << static_cast<int>(encoding.kind) << " trial=" << trial;
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
    for (int trial = 0; trial < 25; ++trial) {
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
        for (int i = 0; i < 7; ++i) {
            const auto s = ref.stepFast();
            ASSERT_EQ(s.reason, fil::cpu::StopReason::step_complete);
            ref_cycles += s.cycles;
        }
        auto jit_bus = basicBus();
        ASSERT_TRUE(jit_bus.loadBytes(flash_base, code).hasValue());
        fil::cpu::CortexM4 jit(jit_bus);
        // Warm up compilation, then run once from the trial state.
        std::optional<fil::cpu::CortexM4::JitStepOutcome> block;
        for (int i = 0; i < 60; ++i) {
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
        for (unsigned int i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
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
    for (unsigned int i = 0U; i < 70U; ++i) static_cast<void>(cpu.prepareJitBlock());
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
        for (unsigned int i = 0U; i < 70U; ++i) static_cast<void>(jit.prepareJitBlock());
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.state(), initial));
        const auto block = jit.tryStepJitBlock();
        ASSERT_TRUE(block);
        ASSERT_EQ(block->count, 2U);
        for (unsigned int i = 0U; i < 2U; ++i) {
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
