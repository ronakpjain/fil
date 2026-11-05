#include "fil/cpu/native_region.hpp"
#include "fil/cpu/cortex_m4.hpp"
#include "fil/cpu/decoder.hpp"
#include "fil/cpu/micro_jit.hpp"
#include "fil/mem/memory_bus.hpp"
#include "fil/mem/mmio_router.hpp"
#include "fil/stm32g4/peripheral.hpp"

#include <gtest/gtest.h>

#include <algorithm>
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

class GuardedWordPort final : public fil::mem::MmioDevice {
public:
    std::uint32_t value{0U}, writes{0U};
    bool blocked{false};
    static bool execute(void* context, std::uint32_t input) noexcept {
        auto& port = *static_cast<GuardedWordPort*>(context);
        if (port.blocked) return false;
        port.value = input; ++port.writes;
#if defined(__x86_64__)
        asm volatile("" ::: "rdi", "rsi", "rdx", "rcx", "r8", "r9", "r10", "r11", "cc", "memory");
#elif defined(__aarch64__)
        asm volatile("" ::: "x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8", "x9",
            "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "cc", "memory");
#endif
        return true;
    }
    DirectWriteWord directWriteWord(std::uint32_t offset) noexcept override {
        return offset == 0U ? DirectWriteWord{&execute, this} : DirectWriteWord{};
    }
    fil::mem::MemoryResult<std::uint64_t> read(std::uint32_t, fil::mem::AccessSize,
        const fil::mem::AccessContext&) override { return value; }
    fil::mem::MemoryResult<std::uint64_t> write(std::uint32_t, fil::mem::AccessSize,
        std::uint64_t input, const fil::mem::AccessContext&) override {
        value = static_cast<std::uint32_t>(input); ++writes; return std::uint64_t{0U};
    }
    std::string_view name() const noexcept override { return "guarded-word"; }
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

TEST(NativeRegionTest, CodeWindowsIncludeCheckedSuccessorsAndBoundaryWideFetches) {
    auto bytes = code(std::array<std::uint16_t, 5>{0xbe00U,0x3001U,0xf000U,0xf800U,0xbe00U});
    NativeRegionPlan plan; std::string error;
    ASSERT_TRUE(buildNativeRegionWindow(bytes, pc, pc, 6U, plan, error)) << error;
    ASSERT_EQ(plan.count, 3U);
    EXPECT_EQ(plan.ops[0].flow, RegionFlow::checked);
    EXPECT_EQ(plan.ops[1].flow, RegionFlow::next);
    EXPECT_EQ(plan.ops[1].edges[0], 2U);
    EXPECT_EQ(plan.ops[2].size, 4U);
    EXPECT_EQ(plan.ops[2].flow, RegionFlow::call);
    EXPECT_EQ(plan.ops[2].edges[0], NativeRegionOp::external);
    ASSERT_TRUE(buildNativeRegionWindow(bytes, pc, pc + 6U, 4U, plan, error));
    EXPECT_EQ(plan.ops[0].pc, pc + 6U); // legal entry into a wide instruction's second halfword
    const auto initial = plan;
    for (const auto size : {0U,1U,12U,514U}) {
        EXPECT_FALSE(buildNativeRegionWindow(bytes, pc, pc, size, plan, error));
        EXPECT_EQ(plan.count, initial.count); EXPECT_EQ(plan.entry_pc, initial.entry_pc);
    }
}

TEST(NativeRegionTest, RegionCapabilityMatchesTheAvailableEmitter) {
    if (!MicroJit::hostSupported()) {
        EXPECT_FALSE(MicroJit::regionHostSupported());
        return;
    }
    const auto bytes = code(std::array<std::uint16_t,2U>{0x2001U,0xbe00U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit) << error;
    const auto function = jit->compileRegion(bytes, pc, pc, error);
    if (MicroJit::regionHostSupported()) EXPECT_NE(function, nullptr) << error;
    else {
        EXPECT_EQ(function, nullptr);
        EXPECT_NE(error.find("host emitter unavailable"), std::string::npos) << error;
    }
}

TEST(NativeRegionTest, LinkedProgramKeepsRegistersFlagsMemoryAndBudgetsAcrossWindows) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    std::array<std::uint16_t, 520U> words; words.fill(0xbe00U);
    words[0] = 0x3001U; words[1] = 0xe0fdU; // ADD r0; B window 1
    words[255] = 0xbf00U; // sequential window crossing is also an entry
    words[256] = 0x3102U; // ADD r1, #2
    words[257] = 0xf000U; words[258] = 0xf8fdU; // BL window 2
    words[259] = 0x6020U; // STR r0, [r4]
    words[260] = 0x3a01U; words[261] = 0xd000U; // SUB r2; BEQ checked
    words[262] = 0xe6f8U; // B window 0
    words[512] = 0x3003U; words[513] = 0x3105U; words[514] = 0x4770U;
    const auto bytes = code(words);
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto program = jit->compileProgram(bytes, pc, error); ASSERT_TRUE(program) << error;
    EXPECT_EQ(jit->compilations(), 3U);
    EXPECT_EQ(program->base(), pc); EXPECT_EQ(program->size(), bytes.size());
    EXPECT_EQ(program->entryFor(pc - 2U), nullptr);
    EXPECT_EQ(program->entryFor(pc + 1U), nullptr);
    EXPECT_EQ(program->entryFor(pc + bytes.size()), nullptr);
    fil::mem::MemoryBus memory, oracle;
    for (auto* bus : {&memory, &oracle}) {
        ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "linked")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
        ASSERT_TRUE(bus->mapRam(ram, 16U, "data"));
        bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
    }
    CortexM4 reference(oracle);
    NativeRegionContext binding;
    ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read));
    ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
    for (const auto stalls : {std::array<std::uint32_t,2>{0U,0U},
                             std::array<std::uint32_t,2>{1U,3U}})
    for (const auto entry : {pc,pc + 510U})
    for (std::uint32_t limit = 0U; limit <= 31U; ++limit)
    for (std::uint64_t credit = 0U; credit <= 100U; ++credit) {
        ASSERT_TRUE(memory.write32(ram, 0xffffffffU)); ASSERT_TRUE(oracle.write32(ram, 0xffffffffU));
        CpuState state; state.r[15] = entry; state.r[0] = 0x7ffffffeU;
        state.r[1] = 0xfffffffeU; state.r[2] = 3U; state.r[4] = ram;
        state.r[14] = 0x12345679U; state.xpsr |= xpsr_c | xpsr_v | xpsr_q;
        state.instruction_address = pc + 16U;
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
            if (next == pc + 526U) { reason = NativeRegionExit::checked_instruction; break; }
            std::uint16_t pipeline = 0U;
            ASSERT_TRUE(reference.peekPredictableCycles(pipeline)) << std::hex << next;
            const auto charge = pipeline + stalls[have && next == fetch_end ? 0U : 1U];
            if (charge > credit - cycles) { reason = NativeRegionExit::cycle_budget; break; }
            const auto step = reference.stepFast();
            ASSERT_EQ(step.reason, StopReason::step_complete); ASSERT_EQ(step.cycles, pipeline);
            ++retired; cycles += charge; have = true; fetch_end = next + step.instruction_size;
        }
        const auto function = program->entryFor(entry); ASSERT_NE(function, nullptr);
        function(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state())) << limit << '/' << credit << '/' << entry;
        EXPECT_EQ(context.exit_reason, reason); EXPECT_EQ(context.retired_count, retired);
        EXPECT_EQ(context.retired_cycles, cycles); EXPECT_EQ(context.have_fetch, have);
        EXPECT_EQ(context.fetch_end, fetch_end);
        EXPECT_EQ(memory.read32(ram).value(), oracle.read32(ram).value());
        EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
        EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
    }
    EXPECT_EQ(jit->compilations(), 3U); // all entries were emitted before any execution
}

TEST(NativeRegionTest, ProgramLinksEveryHalfwordIncludingWideBoundaryInteriors) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    std::array<std::uint16_t,520U> words; words.fill(0xbe00U);
    words[0] = 0x4700U; // BX r0
    words[255] = 0xf240U; words[256] = 0x0207U; // MOVW spans two windows; its second half is a valid LSL.
    words[257] = 0x3101U; words[511] = 0x3001U; words[512] = 0xbf00U;
    const auto bytes = code(words);
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto program = jit->compileProgram(bytes, pc, error); ASSERT_TRUE(program) << error;
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRom(pc, bytes.size(), "halfword-linkage")); ASSERT_TRUE(memory.loadBytes(pc, bytes));
    memory.setReadFootprintTracking(false);
    CortexM4 reference(memory);
    std::array<NativeRegionPlan,3U> plans;
    for (std::uint32_t i = 0U; i < plans.size(); ++i) {
        const auto offset = i * NativeProgram::window_bytes;
        ASSERT_TRUE(buildNativeRegionWindow(bytes, pc, pc + offset,
            std::min<std::uint32_t>(NativeProgram::window_bytes, bytes.size() - offset), plans[i], error));
    }
    for (std::uint32_t target = pc - 2U; target <= pc + bytes.size(); target += 2U)
    for (const auto stall : {0U,2U})
    for (std::uint32_t limit = 0U; limit <= 4U; ++limit)
    for (std::uint64_t credit = 0U; credit <= 16U; ++credit) {
        CpuState state; state.r[15] = pc; state.r[0] = target | 1U;
        state.r[1] = 0xffffffffU; state.r[2] = 0x87654321U; state.xpsr |= xpsr_c | xpsr_v;
        reference.state() = state;
        NativeRegionContext context; context.cpu_state = &state;
        context.instruction_limit = limit; context.cycle_limit = credit;
        context.sequential_fetch_stall = stall; context.nonsequential_fetch_stall = stall;
        std::uint32_t retired = 0U, fetch_end = 0U; std::uint64_t cycles = 0U;
        NativeRegionExit reason;
        for (;;) {
            const auto next = reference.state().r[15];
            if (next < pc || next - pc >= bytes.size()) { reason = NativeRegionExit::external_edge; break; }
            if (retired == limit) { reason = NativeRegionExit::instruction_budget; break; }
            const auto& op = plans[(next - pc) / NativeProgram::window_bytes]
                .ops[((next - pc) % NativeProgram::window_bytes) / 2U];
            if (op.flow == RegionFlow::checked) { reason = NativeRegionExit::checked_instruction; break; }
            std::uint16_t pipeline = 0U; ASSERT_TRUE(reference.peekPredictableCycles(pipeline));
            const auto charge = pipeline + stall;
            if (charge > credit - cycles) { reason = NativeRegionExit::cycle_budget; break; }
            const auto step = reference.stepFast(); ASSERT_EQ(step.reason, StopReason::step_complete);
            ++retired; cycles += charge; fetch_end = next + step.instruction_size;
        }
        program->entryFor(pc)(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state())) << target << '/' << limit << '/' << credit;
        EXPECT_EQ(context.retired_count, retired); EXPECT_EQ(context.retired_cycles, cycles);
        EXPECT_EQ(context.exit_reason, reason); EXPECT_EQ(context.fetch_end, fetch_end);
    }
}

TEST(NativeRegionTest, CheckedServicesResumeWithPublishedStateAndExactCumulativeBudgets) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    std::array<std::uint16_t, 266U> words; words.fill(0xbe00U);
    words[0] = 0x2003U; words[1] = 0x6008U; // MOVS r0,3; STR r0,[r1]
    words[2] = 0x6822U; words[3] = 0x3201U; // checked MMIO LDR r2,[r4]; ADD r2,1
    words[4] = 0xe0faU; // B window 1
    words[256] = 0x4350U; // checked MULS r0,r2
    words[257] = 0x3001U; words[258] = 0x6008U; words[259] = 0xbf00U;
    const auto bytes = code(words);
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto program = jit->compileProgram(bytes, pc, error); ASSERT_TRUE(program) << error;
    fil::mem::MemoryBus memory, oracle;
    RegionPort port, reference_port;
    for (auto* bus : {&memory, &oracle}) {
        ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "services")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
        ASSERT_TRUE(bus->mapRam(ram, 32U, "data"));
        ASSERT_TRUE(bus->mapMmio(0x40000000U, 16U, bus == &memory ? port : reference_port, "port"));
        bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
    }
    CortexM4 checked(memory), reference(oracle);
    NativeRegionContext binding;
    ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read));
    ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
    struct Service { CortexM4& cpu; fil::mem::MemoryBus& memory; std::uint64_t mutation; };
    for (const auto stalls : {std::array<std::uint32_t,2>{0U,0U},
                             std::array<std::uint32_t,2>{2U,4U}})
    for (std::uint32_t limit = 0U; limit <= 12U; ++limit)
    for (std::uint64_t credit = 0U; credit <= 65U; ++credit) {
        ASSERT_TRUE(memory.write32(ram, 0xdeadbeefU)); ASSERT_TRUE(oracle.write32(ram, 0xdeadbeefU));
        CpuState state; state.r[15] = pc; state.r[1] = ram; state.r[4] = 0x40000000U;
        state.r[2] = 0xdeadbeefU; state.r[13] = state.msp = ram + 32U;
        state.xpsr |= xpsr_c | xpsr_v | xpsr_q;
        checked.state() = state; reference.state() = state;
        Service service{checked, memory, memory.sideEffectCheckpoint().mutation_sequence};
        auto context = binding; context.cpu_state = &checked.state();
        context.instruction_limit = limit; context.cycle_limit = credit;
        context.sequential_fetch_stall = stalls[0]; context.nonsequential_fetch_stall = stalls[1];
        context.checked_context = &service;
        context.checked_step = +[](NativeRegionContext* ctx) noexcept -> bool {
            auto& svc = *static_cast<Service*>(ctx->checked_context);
            EXPECT_EQ(ctx->cpu_state, &svc.cpu.state());
            const auto next = ctx->cpu_state->r[15];
            if (next == pc + 4U || next == pc + 512U) {
                EXPECT_EQ(ctx->cpu_state->r[0], 3U);
                EXPECT_EQ(svc.memory.read32(ram).value(), 3U);
                EXPECT_EQ(svc.memory.sideEffectCheckpoint().mutation_sequence, svc.mutation + 4U);
            } else {
                EXPECT_EQ(next, pc + 520U);
                EXPECT_EQ(ctx->cpu_state->r[0], 4U);
                EXPECT_EQ(svc.memory.read32(ram).value(), 4U);
                EXPECT_EQ(svc.memory.sideEffectCheckpoint().mutation_sequence, svc.mutation + 5U);
                return false;
            }
            std::uint16_t pipeline = 0U;
            if (!svc.cpu.peekPredictableCycles(pipeline)) return false;
            const auto charge = pipeline + (ctx->have_fetch && next == ctx->fetch_end
                ? ctx->sequential_fetch_stall : ctx->nonsequential_fetch_stall);
            if (charge > ctx->cycle_limit - ctx->retired_cycles) return false;
            const auto step = svc.cpu.stepFast();
            EXPECT_EQ(step.reason, StopReason::step_complete);
            ++ctx->retired_count; ctx->retired_cycles += charge;
            ctx->have_fetch = true; ctx->fetch_end = next + step.instruction_size;
            return true;
        };
        std::uint32_t retired = 0U, checked_count = 0U, fetch_end = 0U;
        std::uint64_t cycles = 0U; bool have = false;
        NativeRegionExit reason;
        for (;;) {
            if (retired == limit) { reason = NativeRegionExit::instruction_budget; break; }
            const auto next = reference.state().r[15];
            if (next == pc + 520U) { reason = NativeRegionExit::checked_instruction; break; }
            std::uint16_t pipeline = 0U; ASSERT_TRUE(reference.peekPredictableCycles(pipeline));
            const auto charge = pipeline + stalls[have && next == fetch_end ? 0U : 1U];
            const bool service_op = next == pc + 4U || next == pc + 512U;
            if (charge > credit - cycles) {
                reason = service_op ? NativeRegionExit::checked_instruction : NativeRegionExit::cycle_budget;
                break;
            }
            const auto step = reference.stepFast(); ASSERT_EQ(step.reason, StopReason::step_complete);
            ++retired; checked_count += service_op; cycles += charge;
            have = true; fetch_end = next + step.instruction_size;
        }
        program->entryFor(pc)(&context);
        EXPECT_TRUE(bitwiseEqual(checked.state(), reference.state())) << limit << '/' << credit;
        EXPECT_EQ(context.retired_count, retired); EXPECT_EQ(context.retired_cycles, cycles);
        EXPECT_EQ(context.checked_count, checked_count); EXPECT_EQ(context.exit_reason, reason);
        EXPECT_EQ(context.have_fetch, have); EXPECT_EQ(context.fetch_end, fetch_end);
        EXPECT_EQ(memory.read32(ram).value(), oracle.read32(ram).value());
        EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
        EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
        EXPECT_EQ(port.reads, reference_port.reads);
    }
}

TEST(NativeRegionTest, CheckedServicesCanRetireAndStopOrLeaveAnItBoundary) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    for (const auto first : {0x4350U,0xbf08U}) for (const bool resume : {false,true}) {
        const auto bytes = code(std::array<std::uint16_t,2U>{static_cast<std::uint16_t>(first),0xbe00U});
        const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr);
        fil::mem::MemoryBus memory;
        ASSERT_TRUE(memory.mapRom(pc, bytes.size(), "checked-stop")); ASSERT_TRUE(memory.loadBytes(pc, bytes));
        CortexM4 cpu(memory); cpu.state().r[15] = pc; cpu.state().r[0] = 7U; cpu.state().r[2] = 3U;
        struct Service { CortexM4& cpu; bool resume; } service{cpu, resume};
        NativeRegionContext context; context.cpu_state = &cpu.state();
        context.instruction_limit = 1U; context.cycle_limit = 100U;
        context.checked_context = &service;
        context.checked_step = +[](NativeRegionContext* ctx) noexcept {
            auto& svc = *static_cast<Service*>(ctx->checked_context);
            const auto step = svc.cpu.stepFast();
            ctx->retired_count += step.instructions; ctx->retired_cycles += step.cycles;
            ctx->have_fetch = true; ctx->fetch_end = step.instruction_address + step.instruction_size;
            return svc.resume;
        };
        function(&context);
        EXPECT_EQ(context.retired_count, 1U); EXPECT_EQ(context.checked_count, 1U);
        EXPECT_EQ(context.retired_cycles, 1U); EXPECT_EQ(cpu.state().r[15], pc + 2U);
        EXPECT_EQ(context.exit_reason, resume && first != 0xbf08U
            ? NativeRegionExit::external_edge : NativeRegionExit::checked_instruction);
        EXPECT_EQ(cpu.state().it_state != 0U, first == 0xbf08U);
        EXPECT_EQ(cpu.state().r[0], first == 0x4350U ? 21U : 7U);
    }
}

void checkIndexedTransfer(MicroJit& jit, std::uint16_t first, std::uint16_t second) {
    const auto decoded = decode32(first, second); ASSERT_TRUE(decoded);
    const auto bytes = code(std::array<std::uint16_t,3U>{first,second,0xbe00U});
    NativeRegionPlan plan; std::string error;
    ASSERT_TRUE(buildNativeRegion(bytes, pc, pc, plan, error));
    ASSERT_EQ(plan.ops[0].flow, RegionFlow::next) << std::hex << first << '/' << second;
    const auto function = jit.compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr) << error;
    fil::mem::MemoryBus memory, oracle;
    for (auto* bus : {&memory, &oracle}) {
        ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "indexed")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
        ASSERT_TRUE(bus->mapRam(ram, 64U, "data"));
        bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
    }
    CortexM4 reference(oracle);
    NativeRegionContext binding;
    ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read));
    ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
    for (const unsigned bank : {0U,1U,2U})
    for (const auto stalls : {std::array<std::uint32_t,2U>{0U,0U},
                             std::array<std::uint32_t,2U>{2U,4U}})
    for (std::uint32_t limit = 0U; limit < 3U; ++limit)
    for (std::uint64_t credit = 0U; credit <= 16U; ++credit) {
        for (std::uint32_t offset = 0U; offset < 64U; offset += 8U) {
            const auto value = 0x89abcdef80706050ULL + offset * 0x0101010101010101ULL;
            ASSERT_TRUE(memory.write64(ram + offset, value)); ASSERT_TRUE(oracle.write64(ram + offset, value));
        }
        CpuState state; state.r[15] = pc; state.r[0] = 0x80fedcbaU;
        state.r[2] = 3U; state.r[1] = ram + 32U; state.r[14] = ram + 32U;
        state.msp = ram + 32U; state.psp = ram + 24U; state.r[13] = ram + 60U;
        state.control = bank == 0U ? 0U : 2U;
        state.xpsr |= xpsr_c | xpsr_v | xpsr_q | (bank == 2U ? 3U : 0U);
        reference.state() = state;
        auto context = binding; context.cpu_state = &state;
        context.instruction_limit = limit; context.cycle_limit = credit;
        context.sequential_fetch_stall = stalls[0]; context.nonsequential_fetch_stall = stalls[1];
        std::uint16_t pipeline = 0U; ASSERT_TRUE(reference.peekPredictableCycles(pipeline));
        const auto charge = pipeline + stalls[1];
        const bool retires = limit != 0U && credit >= charge;
        if (retires) {
            ASSERT_EQ(reference.stepFast().reason, StopReason::step_complete);
        }
        function(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state()))
            << std::hex << first << '/' << second << '/' << std::dec << bank << '/' << limit << '/' << credit;
        EXPECT_EQ(context.retired_count, retires ? 1U : 0U);
        EXPECT_EQ(context.retired_cycles, retires ? charge : 0U);
        EXPECT_EQ(context.exit_reason, limit == 0U ? NativeRegionExit::instruction_budget
            : !retires ? NativeRegionExit::cycle_budget
            : limit == 1U ? NativeRegionExit::instruction_budget : NativeRegionExit::checked_instruction);
        for (std::uint32_t offset = 0U; offset < 64U; offset += 8U)
            EXPECT_EQ(memory.read64(ram + offset).value(), oracle.read64(ram + offset).value());
        EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
        EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
    }
    for (const auto base : {0x60000000U,0xfffffffeU}) {
        CpuState state; state.r[15] = pc; state.r[0] = 0xdeadbeefU; state.r[2] = 3U;
        state.r[decoded->rn] = base; state.r[13] = state.msp = state.psp = base;
        const auto initial = state; const auto before = memory.sideEffectCheckpoint();
        auto context = binding; context.cpu_state = &state;
        context.instruction_limit = 2U; context.cycle_limit = 100U;
        function(&context);
        EXPECT_TRUE(bitwiseEqual(state, initial)); EXPECT_EQ(context.retired_count, 0U);
        EXPECT_EQ(context.retired_cycles, 0U); EXPECT_EQ(context.exit_reason, NativeRegionExit::checked_instruction);
        EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, before.mutation_sequence);
    }
}

TEST(NativeRegionTest, IndexedTransfersKeepWritebackAliasesActiveStackBanksAndExactCuts) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    for (const auto opcode : {0xf800U,0xf810U,0xf820U,0xf830U,0xf840U,0xf850U,0xf910U,0xf930U})
    for (const auto mode : {0x0b00U,0x0f00U,0x0900U,0x0d00U,0x0c00U})
    for (const auto regs : {std::array<unsigned,2U>{0U,1U},
                           std::array<unsigned,2U>{1U,1U},
                           std::array<unsigned,2U>{0U,13U},
                           std::array<unsigned,2U>{0U,14U},
                           std::array<unsigned,2U>{14U,1U}}) {
        checkIndexedTransfer(*jit, static_cast<std::uint16_t>(opcode | regs[1]),
            static_cast<std::uint16_t>((regs[0] << 12U) | mode | 3U));
    }
}

TEST(NativeRegionTest, RegisterOffsetTransfersUsePreAccessOperandsAndEveryLegalShift) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    for (const auto opcode : {0xf800U,0xf810U,0xf820U,0xf830U,0xf840U,0xf850U,0xf910U,0xf930U})
    for (unsigned shift = 0U; shift < 4U; ++shift)
    for (const auto rd : {0U,1U,2U,14U}) {
        checkIndexedTransfer(*jit, static_cast<std::uint16_t>(opcode | 1U),
            static_cast<std::uint16_t>((rd << 12U) | (shift << 4U) | 2U));
    }
}

TEST(NativeRegionTest, PredicatedMovesMatchEveryConditionFlagAndBudgetCut) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    fil::mem::MemoryBus memory; ASSERT_TRUE(memory.mapRom(pc, 16U, "predicated"));
    CortexM4 checked(memory), reference(memory);
    for (unsigned condition = 0U; condition < 14U; ++condition)
    for (const unsigned else_register : {0U,5U}) {
        const auto it = static_cast<std::uint16_t>(0xbf00U | (condition << 4U)
            | ((condition & 1U) != 0U ? 4U : 12U));
        const auto bytes = code(std::array<std::uint16_t,4U>{it,0x206fU,
            static_cast<std::uint16_t>(0x20deU | (else_register << 8U)),0xbe00U});
        ASSERT_TRUE(memory.loadBytes(pc, bytes));
        NativeRegionPlan plan; ASSERT_TRUE(buildNativeRegion(bytes, pc, pc, plan, error));
        ASSERT_TRUE(plan.ops[0].predicated_pair.valid);
        const auto program = jit->compileProgram(bytes, pc, error); ASSERT_TRUE(program) << error;
        for (unsigned flags = 0U; flags < 16U; ++flags)
        for (const auto stalls : {std::array<std::uint32_t,2U>{0U,0U},
                                 std::array<std::uint32_t,2U>{1U,3U}})
        for (const bool have_fetch : {false,true})
        for (const bool services : {false,true})
        for (std::uint32_t limit = 0U; limit <= 4U; ++limit)
        for (std::uint64_t credit = 0U; credit <= 16U; ++credit) {
            CpuState state; state.r[15] = pc; state.r[0] = 0x12345678U; state.r[5] = 0x87654321U;
            state.instruction_address = pc + 100U;
            state.xpsr = xpsr_t | xpsr_q | (0xaU << 16U) | (flags << 28U)
                | ((flags & 1U) != 0U ? xpsr_it_mask : 0U);
            checked.state() = state; reference.state() = state;
            NativeRegionContext context; context.cpu_state = &checked.state();
            context.instruction_limit = limit; context.cycle_limit = credit;
            context.sequential_fetch_stall = stalls[0]; context.nonsequential_fetch_stall = stalls[1];
            context.have_fetch = have_fetch; context.fetch_end = have_fetch ? pc : 0U;
            context.checked_context = &checked;
            if (services) context.checked_step = +[](NativeRegionContext* ctx) noexcept -> bool {
                auto& cpu = *static_cast<CortexM4*>(ctx->checked_context);
                const auto next = cpu.state().r[15];
                if (next == pc + 6U || ctx->retired_count == ctx->instruction_limit) return false;
                std::uint16_t pipeline = 0U;
                if (!cpu.peekPredictableCycles(pipeline)) return false;
                const auto charge = pipeline + (ctx->have_fetch && ctx->fetch_end == next
                    ? ctx->sequential_fetch_stall : ctx->nonsequential_fetch_stall);
                if (charge > ctx->cycle_limit - ctx->retired_cycles) return false;
                const auto step = cpu.stepFast(); EXPECT_EQ(step.reason, StopReason::step_complete);
                ctx->retired_count += step.instructions; ctx->retired_cycles += charge;
                ctx->have_fetch = true; ctx->fetch_end = next + step.instruction_size;
                return true;
            };
            const auto group_charge = 3U + stalls[0] * 2U + stalls[have_fetch ? 0U : 1U];
            const bool native = limit >= 3U && credit >= group_charge;
            std::uint32_t retired = 0U, fetch_end = context.fetch_end;
            std::uint64_t cycles = 0U; bool have = have_fetch;
            if (native || services) {
                while (retired < limit && retired < 3U) {
                    const auto next = reference.state().r[15];
                    const auto charge = 1U + stalls[have && fetch_end == next ? 0U : 1U];
                    if (charge > credit - cycles) break;
                    const auto step = reference.stepFast(); ASSERT_EQ(step.reason, StopReason::step_complete);
                    ++retired; cycles += charge; have = true; fetch_end = next + step.instruction_size;
                }
            }
            program->entryFor(pc)(&context);
            EXPECT_TRUE(bitwiseEqual(checked.state(), reference.state()))
                << condition << '/' << flags << '/' << else_register << '/' << services << '/' << limit << '/' << credit;
            EXPECT_EQ(context.retired_count, retired); EXPECT_EQ(context.retired_cycles, cycles);
            EXPECT_EQ(context.have_fetch, have); EXPECT_EQ(context.fetch_end, fetch_end);
            EXPECT_EQ(context.checked_count, native ? 0U : retired);
            EXPECT_EQ(context.exit_reason, limit == 0U || (retired == 3U && limit == 3U)
                ? NativeRegionExit::instruction_budget : NativeRegionExit::checked_instruction);
        }
    }
}

TEST(NativeRegionTest, CheckedServicesCloseEveryItMaskWithPerInstructionCredit) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    fil::mem::MemoryBus memory; ASSERT_TRUE(memory.mapRom(pc, 16U, "it-masks"));
    CortexM4 checked(memory), reference(memory);
    for (unsigned mask = 1U; mask < 16U; ++mask) {
        const auto bytes = code(std::array<std::uint16_t,6U>{static_cast<std::uint16_t>(0xbf10U | mask),
            0x2001U,0x2102U,0x2203U,0x2304U,0xbe00U});
        ASSERT_TRUE(memory.loadBytes(pc, bytes));
        const auto program = jit->compileProgram(bytes, pc, error); ASSERT_TRUE(program);
        for (const bool zero : {false,true})
        for (std::uint32_t limit = 0U; limit <= 7U; ++limit)
        for (std::uint64_t credit = 0U; credit <= 8U; ++credit) {
            CpuState state; state.r[15] = pc; state.r[0] = state.r[1] = state.r[2] = state.r[3] = 77U;
            state.xpsr |= xpsr_c | xpsr_v | xpsr_q | (zero ? xpsr_z : 0U);
            checked.state() = state; reference.state() = state;
            NativeRegionContext context; context.cpu_state = &checked.state(); context.checked_context = &checked;
            context.instruction_limit = limit; context.cycle_limit = credit;
            context.checked_step = +[](NativeRegionContext* ctx) noexcept {
                auto& cpu = *static_cast<CortexM4*>(ctx->checked_context);
                if (cpu.state().r[15] == pc + 10U || ctx->retired_count == ctx->instruction_limit
                    || ctx->retired_cycles == ctx->cycle_limit) return false;
                const auto step = cpu.stepFast(); EXPECT_EQ(step.reason, StopReason::step_complete);
                ctx->retired_count += step.instructions; ctx->retired_cycles += step.cycles;
                ctx->have_fetch = true; ctx->fetch_end = step.instruction_address + step.instruction_size;
                return true;
            };
            const auto count = std::min<std::uint64_t>({5U,limit,credit});
            for (std::uint64_t i = 0U; i < count; ++i)
                ASSERT_EQ(reference.stepFast().reason, StopReason::step_complete);
            program->entryFor(pc)(&context);
            EXPECT_TRUE(bitwiseEqual(checked.state(), reference.state())) << mask << '/' << zero << '/' << limit << '/' << credit;
            EXPECT_EQ(context.retired_count, count); EXPECT_EQ(context.retired_cycles, count);
            EXPECT_LE(context.checked_count, context.retired_count);
        }
    }
}

TEST(NativeRegionTest, CheckedItServiceCanRemoveItselfBeforeTheNextPredicate) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t,4U>{0xbf0cU,0x2001U,0x2002U,0xbe00U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr);
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRom(pc, bytes.size(), "service-removal")); ASSERT_TRUE(memory.loadBytes(pc, bytes));
    CortexM4 cpu(memory); cpu.state().r[15] = pc;
    NativeRegionContext context; context.cpu_state = &cpu.state(); context.checked_context = &cpu;
    context.instruction_limit = 2U; context.cycle_limit = 100U;
    context.checked_step = +[](NativeRegionContext* ctx) noexcept {
        auto& checked = *static_cast<CortexM4*>(ctx->checked_context);
        const auto step = checked.stepFast();
        ctx->retired_count += step.instructions; ctx->retired_cycles += step.cycles;
        ctx->have_fetch = true; ctx->fetch_end = step.instruction_address + step.instruction_size;
        ctx->checked_step = nullptr;
        return true;
    };
    function(&context);
    EXPECT_EQ(context.retired_count, 1U); EXPECT_EQ(context.checked_count, 1U);
    EXPECT_EQ(context.exit_reason, NativeRegionExit::checked_instruction);
    EXPECT_EQ(cpu.state().r[15], pc + 2U); EXPECT_NE(cpu.state().it_state, 0U);
}

TEST(NativeRegionTest, PredicatedPairConstituentsRemainOrdinaryStandaloneEntryPoints) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t,4U>{0xbf14U,0x2001U,0x2000U,0xbe00U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto program = jit->compileProgram(bytes, pc, error); ASSERT_TRUE(program);
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRom(pc, bytes.size(), "mov-entry")); ASSERT_TRUE(memory.loadBytes(pc, bytes));
    CortexM4 reference(memory);
    for (const auto offset : {2U,4U}) {
        CpuState state; state.r[15] = pc + offset; state.r[0] = 0xdeadbeefU;
        state.xpsr |= xpsr_n | xpsr_c | xpsr_v | xpsr_q;
        reference.state() = state; ASSERT_EQ(reference.stepFast().reason, StopReason::step_complete);
        NativeRegionContext context; context.cpu_state = &state;
        context.instruction_limit = 1U; context.cycle_limit = 100U;
        program->entryFor(state.r[15])(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state()));
        EXPECT_EQ(context.retired_count, 1U); EXPECT_EQ(context.checked_count, 0U);
    }
}

TEST(NativeRegionTest, MixedSpansPublishOnlyTheCompletedPrefixAtEveryMemoryMiss) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t,9U>{
        0x2001U,0x7008U,0x3101U,0x681aU,0x4410U,0x8008U,0x782cU,0x3801U,0xbe00U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr) << error;
    fil::mem::MemoryBus memory, oracle; RegionPort port;
    for (auto* bus : {&memory, &oracle}) {
        ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "mixed-span")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
        ASSERT_TRUE(bus->mapRam(ram, 32U, "data"));
        bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
    }
    ASSERT_TRUE(memory.mapMmio(0x40000000U, 16U, port, "port"));
    CortexM4 reference(oracle);
    NativeRegionContext binding;
    ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read));
    ASSERT_TRUE(memory.directReadWindow(pc, binding.code_read));
    ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
    for (const auto miss_pc : {0U,pc + 6U,pc + 12U})
    for (const auto stalls : {std::array<std::uint32_t,2U>{0U,0U},
                             std::array<std::uint32_t,2U>{0U,3U},
                             std::array<std::uint32_t,2U>{2U,4U}})
    for (const bool have_fetch : {false,true})
    for (std::uint32_t limit = 0U; limit <= 10U; ++limit)
    for (std::uint64_t credit = 0U; credit <= 48U; ++credit) {
        for (std::uint32_t offset = 0U; offset < 32U; offset += 8U) {
            const auto value = 0x8070605040302010ULL + offset;
            ASSERT_TRUE(memory.write64(ram + offset, value)); ASSERT_TRUE(oracle.write64(ram + offset, value));
        }
        CpuState state; state.r[15] = pc; state.instruction_address = pc + 100U;
        state.r[0] = 0xdeadbeefU; state.r[1] = ram;
        state.r[2] = 0x87654321U; state.r[3] = miss_pc == pc + 6U ? 0x40000000U : ram + 8U;
        state.r[4] = 0x89abcdefU; state.r[5] = miss_pc == pc + 12U ? 0x40000000U : ram + 16U;
        state.xpsr |= xpsr_c | xpsr_v | xpsr_q;
        reference.state() = state;
        auto context = binding; context.cpu_state = &state;
        context.instruction_limit = limit; context.cycle_limit = credit;
        context.sequential_fetch_stall = stalls[0]; context.nonsequential_fetch_stall = stalls[1];
        context.have_fetch = have_fetch; context.fetch_end = have_fetch ? pc : 0U;
        std::uint32_t retired = 0U, fetch_end = context.fetch_end;
        std::uint64_t cycles = 0U; bool have = have_fetch;
        NativeRegionExit reason;
        for (;;) {
            if (retired == limit) { reason = NativeRegionExit::instruction_budget; break; }
            const auto next = reference.state().r[15];
            if (next == pc + 16U || next == miss_pc) { reason = NativeRegionExit::checked_instruction; break; }
            std::uint16_t pipeline = 0U; ASSERT_TRUE(reference.peekPredictableCycles(pipeline));
            const auto charge = pipeline + stalls[have && next == fetch_end ? 0U : 1U];
            if (charge > credit - cycles) { reason = NativeRegionExit::cycle_budget; break; }
            const auto step = reference.stepFast(); ASSERT_EQ(step.reason, StopReason::step_complete);
            ++retired; cycles += charge; have = true; fetch_end = next + step.instruction_size;
        }
        function(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state())) << miss_pc << '/' << limit << '/' << credit;
        EXPECT_EQ(context.retired_count, retired); EXPECT_EQ(context.retired_cycles, cycles);
        EXPECT_EQ(context.exit_reason, reason); EXPECT_EQ(context.fetch_end, fetch_end); EXPECT_EQ(context.have_fetch, have);
        for (std::uint32_t offset = 0U; offset < 32U; offset += 8U)
            EXPECT_EQ(memory.read64(ram + offset).value(), oracle.read64(ram + offset).value());
        EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
        EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
        EXPECT_EQ(port.reads, 0U); EXPECT_EQ(port.writes, 0U);
    }
}

TEST(NativeRegionTest, MixedSpanStackWritebackAndMissingReadBindingsKeepPreciseBankState) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t,7U>{0xf84dU,0x0d04U,0x2009U,0xf85dU,0x2b04U,0x3201U,0xbe00U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr) << error;
    fil::mem::MemoryBus memory, oracle;
    for (auto* bus : {&memory, &oracle}) {
        ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "span-stack")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
        ASSERT_TRUE(bus->mapRam(ram, 64U, "stack"));
        bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
    }
    CortexM4 reference(oracle);
    NativeRegionContext binding;
    ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read));
    ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
    for (const bool read_bound : {false,true})
    for (unsigned bank = 0U; bank < 3U; ++bank)
    for (std::uint32_t limit = 0U; limit <= 6U; ++limit)
    for (std::uint64_t credit = 0U; credit <= 32U; ++credit) {
        for (std::uint32_t offset = 0U; offset < 64U; offset += 8U) {
            ASSERT_TRUE(memory.write64(ram + offset, UINT64_MAX)); ASSERT_TRUE(oracle.write64(ram + offset, UINT64_MAX));
        }
        CpuState state; state.r[15] = pc; state.r[0] = 0x12345678U; state.r[2] = 0xdeadbeefU;
        state.msp = ram + 64U; state.psp = ram + 32U; state.r[13] = ram + 8U;
        state.control = bank == 0U ? 0U : 2U;
        state.xpsr |= xpsr_c | xpsr_v | xpsr_q | (bank == 2U ? 3U : 0U);
        reference.state() = state;
        auto context = binding; context.cpu_state = &state;
        if (!read_bound) context.ram_read = {};
        context.instruction_limit = limit; context.cycle_limit = credit;
        context.sequential_fetch_stall = 1U; context.nonsequential_fetch_stall = 3U;
        std::uint32_t retired = 0U, fetch_end = 0U;
        std::uint64_t cycles = 0U; bool have = false;
        NativeRegionExit reason;
        for (;;) {
            if (retired == limit) { reason = NativeRegionExit::instruction_budget; break; }
            const auto next = reference.state().r[15];
            if (next == pc + 12U || (!read_bound && next == pc + 6U)) {
                reason = NativeRegionExit::checked_instruction; break;
            }
            std::uint16_t pipeline = 0U; ASSERT_TRUE(reference.peekPredictableCycles(pipeline));
            const auto charge = pipeline + (have && next == fetch_end ? 1U : 3U);
            if (charge > credit - cycles) { reason = NativeRegionExit::cycle_budget; break; }
            const auto step = reference.stepFast(); ASSERT_EQ(step.reason, StopReason::step_complete);
            ++retired; cycles += charge; have = true; fetch_end = next + step.instruction_size;
        }
        function(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state())) << read_bound << '/' << bank << '/' << limit << '/' << credit;
        EXPECT_EQ(context.retired_count, retired); EXPECT_EQ(context.retired_cycles, cycles);
        EXPECT_EQ(context.exit_reason, reason); EXPECT_EQ(context.fetch_end, fetch_end); EXPECT_EQ(context.have_fetch, have);
        EXPECT_EQ(memory.read64(ram + 24U).value(), oracle.read64(ram + 24U).value());
        EXPECT_EQ(memory.read64(ram + 56U).value(), oracle.read64(ram + 56U).value());
        EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
        EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
    }
}

TEST(NativeRegionTest, LinearSpanWideChargesAndInteriorEntriesMatchCheckedExecution) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t,4U>{0x2001U,0x3001U,0x7008U,0xbe00U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto program = jit->compileProgram(bytes, pc, error); ASSERT_TRUE(program) << error;
    fil::mem::MemoryBus memory, oracle;
    for (auto* bus : {&memory, &oracle}) {
        ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "wide-span")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
        ASSERT_TRUE(bus->mapRam(ram, 8U, "data"));
        bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
    }
    CortexM4 reference(oracle);
    NativeRegionContext binding;
    ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
    constexpr auto maximum_stall = std::uint64_t{UINT32_MAX};
    for (const auto offset : {0U,2U,4U})
    for (const bool write_bound : {false,true})
    for (const bool have_fetch : {false,true})
    for (std::uint32_t limit = 0U; limit <= 4U; ++limit)
    for (const auto credit : {std::uint64_t{0U},maximum_stall,maximum_stall + 1U,
                             maximum_stall * 2U + 4U,maximum_stall * 3U + 4U,UINT64_MAX}) {
        ASSERT_TRUE(memory.write8(ram, 0x55U)); ASSERT_TRUE(oracle.write8(ram, 0x55U));
        CpuState state; state.r[15] = pc + offset; state.r[0] = 255U; state.r[1] = ram;
        state.xpsr |= xpsr_c | xpsr_v | xpsr_q;
        reference.state() = state;
        auto context = binding; context.cpu_state = &state;
        if (!write_bound) context.ram_write = {};
        context.instruction_limit = limit; context.cycle_limit = credit;
        context.sequential_fetch_stall = UINT32_MAX - 2U;
        context.nonsequential_fetch_stall = UINT32_MAX;
        context.have_fetch = have_fetch; context.fetch_end = have_fetch ? state.r[15] : 0U;
        std::uint32_t retired = 0U, fetch_end = context.fetch_end;
        std::uint64_t cycles = 0U; bool have = have_fetch;
        NativeRegionExit reason;
        for (;;) {
            if (retired == limit) { reason = NativeRegionExit::instruction_budget; break; }
            const auto next = reference.state().r[15];
            if (next == pc + 6U || (!write_bound && next == pc + 4U)) {
                reason = NativeRegionExit::checked_instruction; break;
            }
            std::uint16_t pipeline = 0U; ASSERT_TRUE(reference.peekPredictableCycles(pipeline));
            const auto charge = static_cast<std::uint64_t>(pipeline)
                + (have && next == fetch_end ? UINT32_MAX - 2U : UINT32_MAX);
            if (charge > credit - cycles) { reason = NativeRegionExit::cycle_budget; break; }
            const auto step = reference.stepFast(); ASSERT_EQ(step.reason, StopReason::step_complete);
            ++retired; cycles += charge; have = true; fetch_end = next + step.instruction_size;
        }
        program->entryFor(state.r[15])(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state())) << offset << '/' << write_bound << '/' << limit << '/' << credit;
        EXPECT_EQ(context.retired_count, retired); EXPECT_EQ(context.retired_cycles, cycles);
        EXPECT_EQ(context.exit_reason, reason); EXPECT_EQ(context.fetch_end, fetch_end); EXPECT_EQ(context.have_fetch, have);
        EXPECT_EQ(memory.read8(ram).value(), oracle.read8(ram).value());
        EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
        EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
    }
}

TEST(NativeRegionTest, GuardedMmioWritesPreserveCallerRegistersWritebackAndEveryPartialExit) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t,10U>{
        0x2001U,0x6008U,0x3001U,0x6010U,0xf843U,0x0b04U,0x3001U,0x6020U,0x6835U,0xbe00U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr) << error;
    fil::mem::MemoryBus memory, oracle; GuardedWordPort first, second, ref_first, ref_second;
    for (auto* bus : {&memory,&oracle}) {
        ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "guarded-writes")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
        ASSERT_TRUE(bus->mapRam(ram, 8U, "ram"));
        bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
    }
    ASSERT_TRUE(memory.mapMmio(0x40001000U, 4U, first, "first"));
    ASSERT_TRUE(memory.mapMmio(0x40002000U, 4U, second, "second"));
    ASSERT_TRUE(oracle.mapMmio(0x40001000U, 4U, ref_first, "first"));
    ASSERT_TRUE(oracle.mapMmio(0x40002000U, 4U, ref_second, "second"));
    NativeRegionContext binding;
    ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read)); ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
    ASSERT_TRUE(memory.directMmioWriteWord(0x40001000U, binding.mmio_write[0], binding.mmio_effects));
    ASSERT_TRUE(memory.directMmioWriteWord(0x40002000U, binding.mmio_write[1], binding.mmio_effects));
    CortexM4 reference(oracle);
    {
        CpuState state; state.r[15] = pc; const auto initial = state;
        auto invalid = binding; invalid.cpu_state = &state; invalid.mmio_effects = {};
        invalid.instruction_limit = 20U; invalid.cycle_limit = 100U; function(&invalid);
        EXPECT_EQ(invalid.exit_reason, NativeRegionExit::invalid_state);
        EXPECT_EQ(invalid.retired_count, 0U); EXPECT_TRUE(bitwiseEqual(state, initial));
        EXPECT_EQ(first.writes, 0U); EXPECT_EQ(second.writes, 0U);
    }
    for (const auto blocked : {0U,1U,2U,3U})
    for (const bool late_miss : {false,true})
    for (const auto stall : {0U,3U})
    for (std::uint32_t limit = 0U; limit <= 11U; ++limit)
    for (std::uint64_t credit = 0U; credit <= 64U; ++credit) {
        for (auto* port : {&first,&second,&ref_first,&ref_second}) { port->value = 0x87654321U; port->writes = 0U; }
        first.blocked = (blocked & 1U) != 0U; second.blocked = (blocked & 2U) != 0U;
        ASSERT_TRUE(memory.write32(ram, 0x12345678U)); ASSERT_TRUE(oracle.write32(ram, 0x12345678U));
        CpuState state; state.r[15] = pc; state.r[1] = state.r[3] = 0x40001000U; state.r[2] = ram;
        state.r[4] = 0x40002000U; state.r[6] = late_miss ? 0x40003000U : ram;
        state.r[7] = 0xabcdef01U; state.r[8] = 0x87654321U; state.r[9] = 0x11223344U;
        state.xpsr |= xpsr_c | xpsr_v | xpsr_q; reference.state() = state;
        auto context = binding; context.cpu_state = &state; context.instruction_limit = limit; context.cycle_limit = credit;
        context.sequential_fetch_stall = stall; context.nonsequential_fetch_stall = stall;
        std::uint32_t retired = 0U; std::uint64_t cycles = 0U; NativeRegionExit reason;
        for (;;) {
            if (retired == limit) { reason = NativeRegionExit::instruction_budget; break; }
            const auto next = reference.state().r[15];
            if (next == pc + 18U || ((next == pc + 2U || next == pc + 8U) && (blocked & 1U))
                || (next == pc + 14U && (blocked & 2U)) || (next == pc + 16U && late_miss)) {
                // A device guard runs only after cycle admission. Insufficient
                // credit therefore reports the ordinary budget exit first.
                std::uint16_t pipeline = 0U; ASSERT_TRUE(reference.peekPredictableCycles(pipeline));
                reason = next != pc + 18U && next != pc + 16U && pipeline + stall > credit - cycles
                    ? NativeRegionExit::cycle_budget : NativeRegionExit::checked_instruction;
                break;
            }
            std::uint16_t pipeline = 0U; ASSERT_TRUE(reference.peekPredictableCycles(pipeline));
            const auto charge = pipeline + stall;
            if (charge > credit - cycles) { reason = NativeRegionExit::cycle_budget; break; }
            ASSERT_EQ(reference.stepFast().reason, StopReason::step_complete); ++retired; cycles += charge;
        }
        function(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state())) << blocked << '/' << late_miss << '/' << limit << '/' << credit;
        EXPECT_EQ(context.retired_count, retired); EXPECT_EQ(context.retired_cycles, cycles);
        EXPECT_EQ(context.exit_reason, reason) << blocked << '/' << late_miss << '/' << limit << '/' << credit;
        EXPECT_EQ(first.value, ref_first.value); EXPECT_EQ(first.writes, ref_first.writes);
        EXPECT_EQ(second.value, ref_second.value); EXPECT_EQ(second.writes, ref_second.writes);
        EXPECT_EQ(memory.read32(ram).value(), oracle.read32(ram).value());
        EXPECT_EQ(memory.sideEffectCheckpoint().mmio_generation, oracle.sideEffectCheckpoint().mmio_generation);
        EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
        EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
    }
}

TEST(NativeRegionTest, DirectMmioLoadsPublishExactEffectsAtEveryBudgetAndLaterGuardMiss) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t,8U>{
        0x2001U,0x680aU,0x3001U,0x6823U,0x602aU,0x683eU,0x2802U,0xbe00U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr) << error;
    fil::mem::MemoryBus memory, oracle;
    fil::stm32g4::DmaPeripheral dma1("DMA1", 1U), dma2("DMA2", 1U);
    fil::stm32g4::DmaPeripheral ref1("DMA1", 1U), ref2("DMA2", 1U);
    fil::mem::MmioRouter router(0x40020000U, 0x1000U, "devices"), ref_router(0x40020000U, 0x1000U, "devices");
    ASSERT_TRUE(router.map(0U, dma1.size(), dma1, "dma1")); ASSERT_TRUE(router.map(0x400U, dma2.size(), dma2, "dma2"));
    ASSERT_TRUE(ref_router.map(0U, ref1.size(), ref1, "dma1")); ASSERT_TRUE(ref_router.map(0x400U, ref2.size(), ref2, "dma2"));
    for (auto* device : {&dma1,&dma2,&ref1,&ref2}) {
        ASSERT_TRUE(device->write(12U, fil::mem::AccessSize::word, 1U, {}));
        ASSERT_TRUE(device->write(8U, fil::mem::AccessSize::word, 1U, {})); ASSERT_TRUE(device->request(1U));
    }
    for (auto* bus : {&memory,&oracle}) {
        ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "mmio-loads")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
        ASSERT_TRUE(bus->mapRam(ram, 8U, "data"));
        bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
    }
    ASSERT_TRUE(memory.mapMmio(0x40020000U, 0x1000U, router, "devices"));
    ASSERT_TRUE(oracle.mapMmio(0x40020000U, 0x1000U, ref_router, "devices"));
    CortexM4 reference(oracle);
    NativeRegionContext binding;
    ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read)); ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
    ASSERT_TRUE(memory.directMmioReadWord(0x40020000U, binding.mmio_read[0], binding.mmio_effects));
    ASSERT_TRUE(memory.directMmioReadWord(0x40020400U, binding.mmio_read[1], binding.mmio_effects));
    {
        CpuState state; state.r[15] = pc; state.r[1] = 0x40020000U;
        const auto initial = state; const auto checkpoint = memory.sideEffectCheckpoint();
        auto invalid = binding; invalid.cpu_state = &state;
        invalid.instruction_limit = 10U; invalid.cycle_limit = 100U; invalid.mmio_effects = {};
        function(&invalid);
        EXPECT_EQ(invalid.exit_reason, NativeRegionExit::invalid_state);
        EXPECT_EQ(invalid.retired_count, 0U); EXPECT_TRUE(bitwiseEqual(state, initial));
        const auto store_bytes = code(std::array<std::uint16_t,2U>{0x600aU,0xbe00U}); // STR r2,[r1]
        const auto store = jit->compileRegion(store_bytes, pc, pc, error); ASSERT_NE(store, nullptr) << error;
        auto read_only = binding; read_only.cpu_state = &state;
        read_only.instruction_limit = 10U; read_only.cycle_limit = 100U;
        store(&read_only);
        EXPECT_EQ(read_only.exit_reason, NativeRegionExit::checked_instruction);
        EXPECT_EQ(read_only.retired_count, 0U); EXPECT_TRUE(bitwiseEqual(state, initial));
        EXPECT_EQ(memory.sideEffectCheckpoint().mmio_generation, checkpoint.mmio_generation);
        EXPECT_EQ(*binding.mmio_read[0].value, 9U);
    }
    for (const auto mask : {0U,1U,2U,3U})
    for (const bool late_miss : {false,true})
    for (const auto stall : {0U,3U})
    for (std::uint32_t limit = 0U; limit <= 9U; ++limit)
    for (std::uint64_t credit = 0U; credit <= 48U; ++credit) {
        ASSERT_TRUE(memory.write32(ram, 0x12345678U)); ASSERT_TRUE(oracle.write32(ram, 0x12345678U));
        CpuState state; state.r[15] = pc; state.r[1] = 0x40020000U; state.r[4] = 0x40020400U;
        state.r[5] = ram; state.r[7] = late_miss ? 0x40020800U : ram;
        state.xpsr |= xpsr_c | xpsr_v | xpsr_q; reference.state() = state;
        auto context = binding; context.cpu_state = &state;
        for (std::size_t slot = 0U; slot < 2U; ++slot) if ((mask & (1U << slot)) == 0U) context.mmio_read[slot] = {};
        context.instruction_limit = limit; context.cycle_limit = credit;
        context.sequential_fetch_stall = stall; context.nonsequential_fetch_stall = stall;
        std::uint32_t retired = 0U; std::uint64_t cycles = 0U;
        NativeRegionExit reason;
        for (;;) {
            if (retired == limit) { reason = NativeRegionExit::instruction_budget; break; }
            const auto next = reference.state().r[15];
            if (next == pc + 14U || (next == pc + 2U && !(mask & 1U))
                || (next == pc + 6U && !(mask & 2U)) || (next == pc + 10U && late_miss)) {
                reason = NativeRegionExit::checked_instruction; break;
            }
            std::uint16_t pipeline = 0U; ASSERT_TRUE(reference.peekPredictableCycles(pipeline));
            const auto charge = pipeline + stall;
            if (charge > credit - cycles) { reason = NativeRegionExit::cycle_budget; break; }
            ASSERT_EQ(reference.stepFast().reason, StopReason::step_complete); ++retired; cycles += charge;
        }
        function(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state())) << mask << '/' << late_miss << '/' << limit << '/' << credit;
        EXPECT_EQ(context.retired_count, retired); EXPECT_EQ(context.retired_cycles, cycles); EXPECT_EQ(context.exit_reason, reason);
        EXPECT_EQ(memory.read32(ram).value(), oracle.read32(ram).value());
        EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
        EXPECT_EQ(memory.sideEffectCheckpoint().mmio_generation, oracle.sideEffectCheckpoint().mmio_generation);
        EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
    }
}

TEST(NativeRegionTest, DenseReturnDispatchPreservesWideInstructionGapsAndExternalEdges) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t,8U>{
        0xf000U,0xf804U,0xf240U,0x0200U,0x3301U,0xbe00U,0x4700U,0xbe00U});
    std::string error; NativeRegionPlan plan;
    ASSERT_TRUE(buildNativeRegion(bytes, pc, pc, plan, error));
    ASSERT_EQ(plan.count, 5U);
    auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr) << error;
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRom(pc, bytes.size(), "return-dispatch")); ASSERT_TRUE(memory.loadBytes(pc, bytes));
    CortexM4 reference(memory);
    for (std::uint32_t target = pc - 2U; target <= pc + 16U; target += 2U)
    for (std::uint32_t limit = 0U; limit <= 12U; ++limit)
    for (std::uint64_t credit = 0U; credit <= 40U; ++credit) {
        CpuState state; state.r[15] = pc; state.r[0] = target | 1U;
        state.r[2] = 0xdeadbeefU; state.r[3] = 0xffffffffU;
        state.xpsr |= xpsr_c | xpsr_v; reference.state() = state;
        NativeRegionContext context; context.cpu_state = &state;
        context.instruction_limit = limit; context.cycle_limit = credit;
        std::uint32_t retired = 0U, fetch_end = 0U; std::uint64_t cycles = 0U;
        NativeRegionExit reason;
        for (;;) {
            const auto next = reference.state().r[15];
            const auto found = std::find_if(plan.ops.begin(), plan.ops.begin() + plan.count,
                [&](const auto& op) { return op.pc == next; });
            if (found == plan.ops.begin() + plan.count) { reason = NativeRegionExit::external_edge; break; }
            if (retired == limit) { reason = NativeRegionExit::instruction_budget; break; }
            if (found->flow == RegionFlow::checked) { reason = NativeRegionExit::checked_instruction; break; }
            std::uint16_t charge = 0U; ASSERT_TRUE(reference.peekPredictableCycles(charge));
            if (charge > credit - cycles) { reason = NativeRegionExit::cycle_budget; break; }
            const auto step = reference.stepFast(); ASSERT_EQ(step.reason, StopReason::step_complete);
            ++retired; cycles += charge; fetch_end = next + step.instruction_size;
        }
        function(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state())) << target << '/' << limit << '/' << credit;
        EXPECT_EQ(context.retired_count, retired); EXPECT_EQ(context.retired_cycles, cycles);
        EXPECT_EQ(context.exit_reason, reason); EXPECT_EQ(context.fetch_end, fetch_end);
    }
}

TEST(NativeRegionTest, SpanFlagElisionPreservesCarryAndGuardExitFlags) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    // ADDS; MOVS; CMP; ADD with RRX; ADDS; LDR; SUBS; MOVS; CMP; BKPT.
    const auto bytes = code(std::array<std::uint16_t,11U>{
        0x3001U,0x2200U,0x2800U,0xeb01U,0x0330U,0x3201U,
        0x682cU,0x3801U,0x2201U,0x2a00U,0xbe00U});
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileRegion(bytes, pc, pc, error); ASSERT_NE(function, nullptr) << error;
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRom(pc, bytes.size(), "span-flags")); ASSERT_TRUE(memory.loadBytes(pc, bytes));
    ASSERT_TRUE(memory.mapRam(ram, 4U, "data")); ASSERT_TRUE(memory.write32(ram, 0x87654321U));
    memory.setReadFootprintTracking(false); memory.setWriteJournalTracking(false);
    CortexM4 reference(memory);
    NativeRegionContext binding;
    ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read));
    for (std::uint32_t flags = 0U; flags < 16U; ++flags)
    for (const auto input : {0U,0x7fffffffU,0xffffffffU})
    for (const bool bound : {false,true})
    for (std::uint32_t limit = 0U; limit <= 10U; ++limit)
    for (std::uint64_t credit = 0U; credit <= 20U; ++credit) {
        CpuState state; state.r[15] = pc; state.r[0] = input;
        state.r[1] = 0x12345679U; state.r[5] = ram; state.xpsr |= flags << 28U;
        reference.state() = state;
        auto context = binding; context.cpu_state = &state;
        if (!bound) context.ram_read = {};
        context.instruction_limit = limit; context.cycle_limit = credit;
        std::uint32_t retired = 0U; std::uint64_t cycles = 0U;
        for (;;) {
            if (retired == limit) break;
            const auto next = reference.state().r[15];
            if (next == pc + 20U || (!bound && next == pc + 12U)) break;
            std::uint16_t charge = 0U; ASSERT_TRUE(reference.peekPredictableCycles(charge));
            if (charge > credit - cycles) break;
            ASSERT_EQ(reference.stepFast().reason, StopReason::step_complete);
            ++retired; cycles += charge;
        }
        function(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state())) << flags << '/' << input << '/' << bound << '/' << limit << '/' << credit;
        EXPECT_EQ(context.retired_count, retired); EXPECT_EQ(context.retired_cycles, cycles);
    }
}

TEST(NativeRegionTest, ResidentStackPublishesBeforeControlWritesAndReloadsTheSelectedBank) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    const auto bytes = code(std::array<std::uint16_t,7U>{0xf84dU,0x0d04U,0xf381U,0x8814U,0xf85dU,0x2b04U,0xbe00U});
    const auto control = decode32(0xf381U,0x8814U); ASSERT_TRUE(control); ASSERT_EQ(control->kind, InstrKind::msr);
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto program = jit->compileProgram(bytes, pc, error); ASSERT_TRUE(program) << error;
    fil::mem::MemoryBus memory, oracle;
    for (auto* bus : {&memory, &oracle}) {
        ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "stack-control")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
        ASSERT_TRUE(bus->mapRam(ram, 64U, "stack"));
        bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
    }
    CortexM4 checked(memory), reference(oracle);
    NativeRegionContext binding;
    ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read));
    ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
    for (const bool handler : {false,true}) for (const bool resume : {false,true}) {
        for (auto* bus : {&memory, &oracle}) {
            ASSERT_TRUE(bus->write32(ram + 16U, 0x55667788U)); ASSERT_TRUE(bus->write32(ram + 28U, 0xffffffffU));
        }
        CpuState state; state.r[15] = pc; state.r[0] = 0x11223344U; state.r[1] = 2U;
        state.r[2] = 0xdeadbeefU; state.r[13] = ram + 8U; state.msp = ram + 32U; state.psp = ram + 16U;
        state.xpsr |= xpsr_c | xpsr_v | xpsr_q | (handler ? 3U : 0U);
        checked.state() = state; reference.state() = state;
        struct Service { CortexM4& cpu; fil::mem::MemoryBus& bus; bool resume; } service{checked,memory,resume};
        auto context = binding; context.cpu_state = &checked.state(); context.checked_context = &service;
        context.instruction_limit = 3U; context.cycle_limit = 100U;
        context.checked_step = +[](NativeRegionContext* ctx) noexcept {
            auto& svc = *static_cast<Service*>(ctx->checked_context);
            EXPECT_EQ(svc.cpu.state().r[15], pc + 4U);
            EXPECT_EQ(svc.cpu.state().msp, ram + 28U); EXPECT_EQ(svc.cpu.state().r[13], ram + 28U);
            EXPECT_EQ(svc.bus.read32(ram + 28U).value(), 0x11223344U);
            const auto step = svc.cpu.stepFast(); EXPECT_EQ(step.reason, StopReason::step_complete);
            ctx->retired_count += step.instructions; ctx->retired_cycles += step.cycles;
            ctx->have_fetch = true; ctx->fetch_end = step.instruction_address + step.instruction_size;
            return svc.resume;
        };
        std::uint64_t cycles = 0U;
        for (unsigned n = 0U; n < (resume ? 3U : 2U); ++n) {
            const auto step = reference.stepFast(); ASSERT_EQ(step.reason, StopReason::step_complete); cycles += step.cycles;
        }
        program->entryFor(pc)(&context);
        EXPECT_TRUE(bitwiseEqual(checked.state(), reference.state())) << handler << '/' << resume;
        EXPECT_EQ(context.retired_count, resume ? 3U : 2U); EXPECT_EQ(context.checked_count, 1U);
        EXPECT_EQ(context.retired_cycles, cycles);
        EXPECT_EQ(context.exit_reason, resume ? NativeRegionExit::instruction_budget : NativeRegionExit::checked_instruction);
        EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
        EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
    }
}

TEST(NativeRegionTest, ResidentStackSurvivesLinkedWindowsAndRestoresAnInitiallyStaleMirror) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
    std::array<std::uint16_t,516U> words; words.fill(0xbe00U);
    words[0] = 0xf000U; words[1] = 0xf8feU; // BL pc+512
    words[256] = 0xb501U; words[257] = 0xe0fdU; // PUSH r0,LR; B pc+1024
    words[512] = 0x2007U; words[513] = 0xbd01U; // MOV r0,7; POP r0,PC
    const auto bytes = code(words);
    std::string error; auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto program = jit->compileProgram(bytes, pc, error); ASSERT_TRUE(program) << error;
    fil::mem::MemoryBus memory, oracle;
    for (auto* bus : {&memory, &oracle}) {
        ASSERT_TRUE(bus->mapRom(pc, bytes.size(), "stack-windows")); ASSERT_TRUE(bus->loadBytes(pc, bytes));
        ASSERT_TRUE(bus->mapRam(ram, 64U, "stack"));
        bus->setReadFootprintTracking(false); bus->setWriteJournalTracking(false);
    }
    CortexM4 reference(oracle);
    NativeRegionContext binding;
    ASSERT_TRUE(memory.directReadWindow(ram, binding.ram_read));
    ASSERT_TRUE(memory.directWriteWindow(ram, binding.ram_write));
    for (unsigned bank = 0U; bank < 3U; ++bank) {
        for (auto* bus : {&memory, &oracle}) {
            ASSERT_TRUE(bus->write64(ram + 24U, 0U)); ASSERT_TRUE(bus->write64(ram + 56U, 0U));
        }
        CpuState state; state.r[15] = pc; state.r[0] = 0x12345678U;
        state.r[13] = ram + 8U; state.msp = ram + 64U; state.psp = ram + 32U;
        state.control = bank == 0U ? 0U : 2U;
        state.xpsr |= xpsr_c | xpsr_v | xpsr_q | (bank == 2U ? 3U : 0U);
        reference.state() = state;
        std::uint64_t cycles = 0U;
        for (unsigned n = 0U; n < 5U; ++n) {
            const auto step = reference.stepFast(); ASSERT_EQ(step.reason, StopReason::step_complete); cycles += step.cycles;
        }
        auto context = binding; context.cpu_state = &state;
        context.instruction_limit = 5U; context.cycle_limit = 100U;
        program->entryFor(pc)(&context);
        EXPECT_TRUE(bitwiseEqual(state, reference.state())) << bank;
        EXPECT_EQ(context.retired_count, 5U); EXPECT_EQ(context.retired_cycles, cycles);
        EXPECT_EQ(context.exit_reason, NativeRegionExit::instruction_budget);
        EXPECT_EQ(memory.read64(ram + 24U).value(), oracle.read64(ram + 24U).value());
        EXPECT_EQ(memory.read64(ram + 56U).value(), oracle.read64(ram + 56U).value());
        EXPECT_EQ(memory.sideEffectCheckpoint().mutation_sequence, oracle.sideEffectCheckpoint().mutation_sequence);
        EXPECT_EQ(memory.sideEffectGeneration(), oracle.sideEffectGeneration());
    }
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

TEST(NativeRegionTest, StackCallsMatchCpuRamAndTimingAcrossAllBudgetCutsAndBanks) {
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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
    if (!MicroJit::regionHostSupported()) GTEST_SKIP();
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

} // namespace
