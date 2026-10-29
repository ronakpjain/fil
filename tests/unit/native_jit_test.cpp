#include "fil/cpu/micro_jit.hpp"
#include "fil/cpu/decoder.hpp"
#include "fil/cpu/native_execution.hpp"
#include "fil/cpu/native_ir.hpp"

#include <gtest/gtest.h>

#include <array>
#include <string>

namespace {
using namespace fil::cpu;

TEST(NativeJitTest, DecodedLiteralAndStackAccessesMatchExactEveryPrefix) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t flash = 0x08000000U, ram = 0x20000000U;
    const std::array<std::uint16_t, 4> words{0x4903U, 0x9a01U, 0x600aU, 0xbf00U};
    std::array<std::uint8_t, 32> image{};
    std::array<MicroSource, 4> sources;
    for (std::size_t i = 0U; i < words.size(); ++i) {
        image[2U * i] = static_cast<std::uint8_t>(words[i]);
        image[2U * i + 1U] = static_cast<std::uint8_t>(words[i] >> 8U);
        const auto decoded = decode16(words[i]);
        ASSERT_TRUE(decoded);
        sources[i] = {*decoded, flash + static_cast<std::uint32_t>(2U * i), 2U};
    }
    for (unsigned i = 0U; i < 4U; ++i)
        image[16U + i] = static_cast<std::uint8_t>(ram >> (8U * i));
    NativeBlock block;
    ASSERT_TRUE(buildNativeBlock(sources, block).ok);
    std::string error;
    auto jit = MicroJit::create(error);
    ASSERT_TRUE(jit);
    const auto function = jit->compileNative(block, error);
    ASSERT_NE(function, nullptr) << error;
    const std::array<std::uint32_t, 4> costs{2U, 2U, 2U, 1U};
    for (std::uint32_t limit = 0U; limit <= words.size(); ++limit) {
        fil::mem::MemoryBus bus, oracle;
        for (auto* memory : {&bus, &oracle}) {
            ASSERT_TRUE(memory->mapRom(flash, image.size(), "flash"));
            ASSERT_TRUE(memory->mapRam(ram, 64U, "ram"));
            ASSERT_TRUE(memory->loadBytes(flash, image));
            ASSERT_TRUE(memory->write32(ram + 20U, 0xcafebabeU));
        }
        CpuState state;
        state.r[15] = flash;
        state.instruction_address = flash;
        state.r[13] = state.msp = ram + 16U;
        state.xpsr = xpsr_t | xpsr_n | xpsr_c | xpsr_q;
        CortexM4 reference(oracle);
        reference.state() = state;
        NativeExecutionContext context{&state, &bus, limit, 100U};
        context.charged_cycles = costs.data();
        context.charged_cycles_count = costs.size();
        function(&context);
        for (std::uint32_t i = 0U; i < limit; ++i)
            ASSERT_EQ(reference.stepFast().reason, StopReason::step_complete);
        ASSERT_EQ(context.retired_count, limit);
        EXPECT_TRUE(bitwiseEqual(state, reference.state()));
        EXPECT_EQ(bus.read32(ram).value(), oracle.read32(ram).value());
        EXPECT_EQ(bus.readFootprint().words, oracle.readFootprint().words);
    }
}

TEST(NativeJitTest, StartupShapesDoNotPreventSteadyStateCompilation) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    std::string error;
    auto jit = MicroJit::create(error);
    ASSERT_TRUE(jit);
    NativeFunction first = nullptr;
    NativeBlock initial;
    for (std::uint32_t i = 0U; i < 256U; ++i) {
        NativeBlock block;
        block.entry_pc = 0x08004000U + i * 2U;
        NativeInst nop;
        nop.pc = nop.integer.pc = block.entry_pc;
        nop.size = nop.integer.size = 2U;
        block.ops.push_back(nop);
        const auto function = jit->compileNative(block, error);
        ASSERT_NE(function, nullptr) << i << ": " << error;
        if (i == 0U) { first = function; initial = block; }
    }
    EXPECT_EQ(jit->compileNative(initial, error), first);
    CpuState state;
    fil::mem::MemoryBus memory;
    const std::array<std::uint32_t, 1> cost{1U};
    NativeExecutionContext context{&state, &memory, 1U, 1U};
    context.charged_cycles = cost.data(); context.charged_cycles_count = 1U;
    first(&context);
    EXPECT_EQ(context.retired_count, 1U);
    EXPECT_EQ(state.r[15], initial.entry_pc + 2U);
}

TEST(NativeJitTest, RejectsMalformedDirectIrBeforeEmission) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    std::string error;
    auto jit = MicroJit::create(error);
    ASSERT_NE(jit, nullptr);
    NativeBlock valid;
    valid.entry_pc = 0x08000000U;
    NativeInst mov;
    mov.pc = valid.entry_pc; mov.size = 2U;
    mov.integer.pc = mov.pc; mov.integer.size = mov.size;
    mov.integer.op = MicroOp::mov_imm;
    valid.ops.push_back(mov);
    for (unsigned variant = 0U; variant < 8U; ++variant) {
        auto block = valid;
        switch (variant) {
        case 0U: block.ops[0].kind = static_cast<NativeOp>(255U); break;
        case 1U: block.ops[0].integer.rd = 15U; break;
        case 2U: block.ops[0].integer.rn = 255U; break;
        case 3U: block.ops[0].integer.op = static_cast<MicroOp>(255U); break;
        case 4U: block.ops[0].integer.pc += 2U; break;
        case 5U: block.ops[0].integer.size = 4U; break;
        case 6U: block.entry_pc += 2U; break;
        case 7U: block.ops.push_back(mov); break;
        }
        EXPECT_EQ(jit->compileNative(block, error), nullptr) << variant;
    }
    EXPECT_NE(jit->compileNative(valid, error), nullptr) << error;
}

TEST(NativeJitTest, AlternatingIntegerMemoryPreciseLimitsAndFlags) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    std::string error;
    auto jit = MicroJit::create(error);
    ASSERT_NE(jit, nullptr) << error;
    NativeBlock block;
    block.entry_pc = 0x08000000U;
    NativeInst mov; mov.kind = NativeOp::integer; mov.pc = block.entry_pc; mov.size = 2U;
    mov.integer.op = MicroOp::mov_imm; mov.integer.rd = 0U; mov.integer.imm = 0x12345678U;
    mov.integer.pc = mov.pc; mov.integer.size = mov.size;
    NativeInst store; store.kind = NativeOp::store_word; store.pc = mov.pc + 2U; store.size = 2U; store.rn = 1U; store.rd = 0U;
    NativeInst load = store; load.kind = NativeOp::load_word; load.pc = store.pc + 2U; load.rd = 2U;
    block.ops = {mov, store, load};
    const auto fn = jit->compileNative(block, error);
    ASSERT_NE(fn, nullptr) << error;
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 16U, "ram"));
    CpuState cpu;
    cpu.r[1] = 0x20000000U;
    cpu.xpsr = xpsr_t | xpsr_n | xpsr_c | xpsr_q | 0x00050000U;
    const auto retained = cpu.xpsr & ~(xpsr_n | xpsr_z | xpsr_c | xpsr_v);
    const std::array<std::uint32_t, 3> costs{2U, 3U, 4U};
    NativeExecutionContext ctx{&cpu, &bus, 3U, 9U};
    ctx.charged_cycles = costs.data(); ctx.charged_cycles_count = costs.size();
    fn(&ctx);
    EXPECT_EQ(ctx.retired_count, 3U);
    EXPECT_EQ(ctx.retired_cycles, 9U);
    EXPECT_EQ(cpu.r[2], 0x12345678U);
    EXPECT_EQ(cpu.r[15], block.ops.back().pc + 2U);
    EXPECT_EQ(cpu.instruction_address, block.ops.back().pc);
    EXPECT_EQ(cpu.xpsr & ~(xpsr_n | xpsr_z | xpsr_c | xpsr_v), retained);

    for (std::uint32_t prefix = 0U; prefix <= 3U; ++prefix) {
        cpu = CpuState{}; cpu.r[1] = 0x20000000U;
        ctx.retired_count = 99U; ctx.retired_cycles = 0U; ctx.instruction_limit = prefix;
        fn(&ctx);
        EXPECT_EQ(ctx.retired_count, prefix);
        EXPECT_EQ(cpu.r[15], prefix == 0U ? 0U : block.entry_pc + prefix * 2U);
    }
    ctx.instruction_limit = 3U; ctx.cycle_limit = 0U; ctx.retired_cycles = 0U;
    fn(&ctx);
    EXPECT_EQ(ctx.retired_count, 0U);
    EXPECT_EQ(ctx.exit_reason, NativeExitReason::cycle_limit);
}

TEST(NativeJitTest, AliasedStoreExecutableDeclineAndRepeatedExecution) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    std::string error;
    auto jit = MicroJit::create(error); ASSERT_NE(jit, nullptr);
    NativeBlock block; block.entry_pc = 0x08003000U;
    NativeInst st; st.kind = NativeOp::store_word; st.pc = block.entry_pc;
    st.size = 2U; st.rn = 1U; st.rd = 1U; // address and source alias
    block.ops.push_back(st);
    auto fn = jit->compileNative(block, error); ASSERT_NE(fn, nullptr) << error;
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 8U, "ram"));
    ASSERT_TRUE(bus.mapRam(0x20000100U, 8U, "executable", true));
    CpuState cpu; cpu.r[1] = 0x20000000U;
    const std::array<std::uint32_t, 1> cost{1U};
    NativeExecutionContext ctx{&cpu, &bus, 1U, 1U};
    ctx.charged_cycles = cost.data(); ctx.charged_cycles_count = 1U;
    fn(&ctx);
    EXPECT_EQ(ctx.retired_count, 1U);
    std::uint32_t value = 0U;
    ASSERT_TRUE(bus.tryFastRead32(0x20000000U, value, {fil::mem::AccessType::data_read, 0U}));
    EXPECT_EQ(value, 0x20000000U);

    // Same immutable kernel can be called repeatedly with independent contexts.
    cpu = CpuState{}; cpu.r[1] = 0x20000100U;
    ctx.retired_cycles = 0U; ctx.retired_count = 0U;
    fn(&ctx);
    EXPECT_EQ(ctx.retired_count, 0U);
    EXPECT_EQ(ctx.exit_reason, NativeExitReason::helper_declined);
    EXPECT_EQ(ctx.failing_instruction_pc, st.pc);
}

TEST(NativeJitTest, HelperDeclineAfterCommittedStoreAndBranchDecline) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    std::string error;
    auto jit = MicroJit::create(error); ASSERT_NE(jit, nullptr);
    NativeBlock block; block.entry_pc = 0x08001000U;
    NativeInst st; st.kind = NativeOp::store_word; st.pc = block.entry_pc; st.size = 2U; st.rn = 1U; st.rd = 0U;
    NativeInst ld; ld.kind = NativeOp::load_word; ld.pc = st.pc + 2U; ld.size = 2U; ld.rn = 2U; ld.rd = 3U;
    block.ops = {st, ld};
    auto fn = jit->compileNative(block, error); ASSERT_NE(fn, nullptr) << error;
    fil::mem::MemoryBus bus; ASSERT_TRUE(bus.mapRam(0x20000000U, 4U, "ram"));
    CpuState cpu; cpu.r[0] = 0xcafebabeU; cpu.r[1] = 0x20000000U; cpu.r[2] = 0x40000000U;
    std::array<std::uint32_t, 2> costs{1U, 1U};
    NativeExecutionContext ctx{&cpu, &bus, 2U, 2U};
    ctx.charged_cycles = costs.data(); ctx.charged_cycles_count = costs.size();
    fn(&ctx);
    EXPECT_EQ(ctx.retired_count, 1U);
    EXPECT_EQ(ctx.exit_reason, NativeExitReason::helper_declined);
    EXPECT_EQ(ctx.failing_instruction_pc, ld.pc);
    EXPECT_EQ(cpu.r[15], ld.pc);
    std::uint32_t value = 0U;
    ASSERT_TRUE(bus.tryFastRead32(0x20000000U, value, {fil::mem::AccessType::data_read, 0U}));
    EXPECT_EQ(value, 0xcafebabeU);
    NativeBlock branch; branch.entry_pc = 0x08002000U;
    NativeInst b; b.kind = NativeOp::integer; b.pc = branch.entry_pc; b.size = 2U; b.integer.op = MicroOp::b;
    branch.ops.push_back(b);
    EXPECT_EQ(jit->compileNative(branch, error), nullptr);
}

} // namespace
