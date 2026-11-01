#include "fil/cpu/micro_jit.hpp"
#include "fil/cpu/decoder.hpp"
#include "fil/cpu/native_execution.hpp"
#include "fil/cpu/native_ir.hpp"
#include "fil/cpu/native_trace.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <string>
#include <vector>

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
    for (std::uint32_t variant = 0U; variant < 15U; ++variant) {
        const auto mode = variant / 5U;
        const auto limit = variant % 5U;
        fil::mem::MemoryBus bus, oracle;
        for (auto* memory : {&bus, &oracle}) {
            ASSERT_TRUE(memory->mapRom(flash, image.size(), "flash"));
            ASSERT_TRUE(memory->mapRam(ram, 64U, "ram"));
            ASSERT_TRUE(memory->loadBytes(flash, image));
            ASSERT_TRUE(memory->write32(ram + 20U, 0xcafebabeU));
            ASSERT_TRUE(memory->write32(ram + 36U, 0xbadc0ffeU));
        }
        CpuState state;
        state.r[15] = flash;
        state.instruction_address = flash;
        state.r[13] = ram + 48U; // Deliberately stale architectural mirror.
        state.msp = ram + 16U;
        state.psp = ram + 32U;
        state.control = mode == 0U ? 0U : 2U;
        state.xpsr = xpsr_t | xpsr_n | xpsr_c | xpsr_q | (mode == 2U ? 3U : 0U);
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

TEST(NativeJitTest, NativeTraceCheckedPushPopCommitAndGuardedPopPc) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t flash = 0x08000000U, ram = 0x20000000U;
    std::array<NativeTraceOp, 2> ops{};
    const auto push = decode16(0xb403U); // PUSH {r0,r1}
    const auto pop = decode16(0xbc03U);  // POP {r0,r1}
    ASSERT_TRUE(push); ASSERT_TRUE(pop);
    ops[0].decoded = *push; ops[0].raw = push->raw; ops[0].pc = flash;
    ops[0].size = 2U; ops[0].expected_next_pc = flash + 2U;
    ops[1].decoded = *pop; ops[1].raw = pop->raw; ops[1].pc = flash + 2U;
    ops[1].size = 2U; ops[1].expected_next_pc = flash + 4U;
    NativeTracePlan plan; std::string error;
    ASSERT_TRUE(buildNativeTrace(ops, plan, error)) << error;
    auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto function = jit->compileNativeTrace(plan, error); ASSERT_NE(function, nullptr) << error;
    fil::mem::MemoryBus bus; ASSERT_TRUE(bus.mapRam(ram, 64U, "ram"));
    CpuState cpu; cpu.msp = ram + 32U; cpu.r[0] = 0x12345678U; cpu.r[1] = 0xabcdef01U;
    cpu.r[15] = flash; cpu.xpsr = xpsr_t;
    const std::array<std::uint32_t, 2> costs{2U, 3U};
    NativeExecutionContext context{&cpu, &bus, 2U, 5U};
    context.charged_cycles = costs.data(); context.charged_cycles_count = costs.size();
    function(&context);
    EXPECT_EQ(context.retired_count, 2U);
    EXPECT_EQ(cpu.r[0], 0x12345678U); EXPECT_EQ(cpu.r[1], 0xabcdef01U);
    EXPECT_EQ(cpu.readRegister(13U), ram + 32U);
    EXPECT_EQ(cpu.r[15], flash + 4U);
    EXPECT_EQ(context.retired_cycles, 5U);
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

TEST(NativeJitTest, EmitsDirectInstructionStencilsForBlocksAndTraces) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    const auto decoded = decode16(0x2001U);
    ASSERT_TRUE(decoded);
    const MicroSource source{*decoded, 0x08000000U, 2U};
    NativeBlock block;
    ASSERT_TRUE(buildNativeBlock({&source, 1U}, block).ok);
    std::string error;
    auto jit = MicroJit::create(error);
    ASSERT_TRUE(jit) << error;
    ASSERT_NE(jit->compileNative(block, error), nullptr) << error;
    // Even one guest instruction emits admission, execution and commit code.
    EXPECT_GT(jit->bytesEmitted(), 32U);
    const auto before = jit->bytesEmitted();
    NativeTraceOp op;
    op.decoded = *decoded; op.raw = decoded->raw; op.pc = source.pc;
    op.size = 2U; op.execution_cycles = 1U; op.expected_next_pc = source.pc + 2U;
    NativeTracePlan plan;
    ASSERT_TRUE(buildNativeTrace({&op, 1U}, plan, error)) << error;
    ASSERT_NE(jit->compileNativeTrace(plan, error), nullptr) << error;
    EXPECT_GT(jit->bytesEmitted() - before, 32U);
}

TEST(NativeJitTest, IntegerStencilsMatchPortableOracleForEveryFlagPattern) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08010000U;
    std::vector<MicroInst> cases;
    for (unsigned op = static_cast<unsigned>(MicroOp::nop);
         op <= static_cast<unsigned>(MicroOp::rrx); ++op) {
        for (bool flags : {false, true}) {
            MicroInst inst;
            inst.op = static_cast<MicroOp>(op);
            inst.pc = pc; inst.size = 4U; inst.rd = 1U; inst.rn = 1U; inst.rm = 2U;
            inst.imm = 0x87654321U; inst.shift_amount = 7U; inst.set_flags = flags;
            cases.push_back(inst);
            if (inst.op == MicroOp::add_shift || inst.op == MicroOp::sub_shift) {
                inst.shift = ShiftType::rrx; inst.shift_amount = 0U; cases.push_back(inst);
            }
            if (inst.op == MicroOp::lsl || inst.op == MicroOp::lsr || inst.op == MicroOp::asr) {
                if (!flags) {
                    inst.shift_immediate = false; cases.push_back(inst);
                }
                if (inst.op != MicroOp::lsl) {
                    inst.shift_immediate = true; inst.shift_amount = 32U;
                    inst.rd = inst.rm; cases.push_back(inst);
                }
            }
        }
    }
    for (auto kind : {ExtendKind::uxtb, ExtendKind::uxth, ExtendKind::sxtb, ExtendKind::sxth}) {
        for (auto rotate : {0U, 8U, 16U, 24U}) {
            MicroInst inst;
            inst.op = MicroOp::extend; inst.extend_kind = kind; inst.imm = rotate;
            inst.pc = pc; inst.size = 4U; inst.rd = inst.rm = 2U;
            cases.push_back(inst);
        }
    }
    for (const auto& inst : cases) {
        SCOPED_TRACE(static_cast<unsigned>(inst.op));
        NativeInst native;
        native.pc = pc; native.size = 4U; native.integer = inst;
        NativeBlock block;
        block.entry_pc = pc; block.ops = {native};
        std::string error;
        auto jit = MicroJit::create(error);
        ASSERT_TRUE(jit) << error;
        const auto fn = jit->compileNative(block, error);
        ASSERT_NE(fn, nullptr) << error;
        const std::array<std::uint32_t, 1> costs{3U};
        for (std::uint32_t flags = 0; flags < 16U; ++flags) {
            for (auto value : {0U, 1U, 31U, 32U, 33U, 255U, 256U, 0x80fe7f01U, 0xffffffffU}) {
                CpuState state;
                for (unsigned r = 0; r < 16U; ++r) state.r[r] = 0xfedcba98U + r;
                state.r[2] = value; state.xpsr = (flags << 28U) | 0x090f0000U;
                MicroCpu oracle;
                for (unsigned r = 0; r < oracle.r.size(); ++r) oracle.r[r] = state.r[r];
                oracle.lr = state.r[14]; oracle.xpsr = state.xpsr; oracle.pc = state.r[15];
                ASSERT_EQ(interpretMicroOps({&inst, 1U}, oracle, 1U), 1U);
                auto expected = state;
                for (unsigned r = 0; r < oracle.r.size(); ++r) expected.r[r] = oracle.r[r];
                expected.r[14] = oracle.lr; expected.r[15] = oracle.pc;
                expected.xpsr = oracle.xpsr; expected.instruction_address = pc;
                NativeExecutionContext ctx{&state, nullptr, 1U, 3U};
                ctx.charged_cycles = costs.data(); ctx.charged_cycles_count = 1U;
                fn(&ctx);
                ASSERT_TRUE(bitwiseEqual(state, expected)) << "NZCV=" << flags << " value=" << value;
                EXPECT_EQ(ctx.retired_count, 1U); EXPECT_EQ(ctx.retired_cycles, 3U);
                EXPECT_EQ(ctx.exit_reason, NativeExitReason::none);
            }
        }
    }
}

TEST(NativeJitTest, AllMemoryWidthsAndShortCostArraysRetirePreciselyAtEveryCut) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08018000U, ram = 0x20000000U;
    // STR/STRB/STRH, LDR/LDRB/LDRH.
    constexpr std::array<std::uint16_t, 6> words{
        0x6001U, 0x7101U, 0x8101U, 0x6802U, 0x7903U, 0x8904U};
    std::array<MicroSource, 6> sources;
    std::array<NativeTraceOp, 6> trace;
    for (std::size_t i = 0; i < words.size(); ++i) {
        const auto decoded = decode16(words[i]);
        ASSERT_TRUE(decoded);
        sources[i] = {*decoded, pc + static_cast<std::uint32_t>(i * 2U), 2U};
        trace[i].decoded = *decoded; trace[i].raw = decoded->raw; trace[i].pc = sources[i].pc;
        trace[i].size = 2U; trace[i].execution_cycles = 2U;
        trace[i].expected_next_pc = sources[i].pc + 2U;
    }
    // Add normalized absolute accesses for signed helper results in blocks.
    NativeBlock block;
    ASSERT_TRUE(buildNativeBlock(std::span{sources}.first(6U), block).ok);
    for (auto kind : {NativeOp::load_byte_signed, NativeOp::load_half_signed}) {
        NativeInst inst;
        inst.kind = kind; inst.rd = kind == NativeOp::load_byte_signed ? 5U : 7U;
        inst.pc = pc + static_cast<std::uint32_t>(block.ops.size() * 2U); inst.size = 2U;
        inst.absolute = true; inst.absolute_address = ram + 4U;
        block.ops.push_back(inst);
    }
    NativeTracePlan plan;
    std::string error;
    ASSERT_TRUE(buildNativeTrace(std::span{trace}.first(6U), plan, error)) << error;
    auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto block_fn = jit->compileNative(block, error); ASSERT_NE(block_fn, nullptr) << error;
    const auto trace_fn = jit->compileNativeTrace(plan, error); ASSERT_NE(trace_fn, nullptr) << error;
    const std::array<std::uint32_t, 8> costs{2U, 2U, 2U, 2U, 2U, 2U, 2U, 2U};
    for (bool use_trace : {false, true}) {
        const auto count = use_trace ? 6U : 8U;
        for (std::uint32_t limit = 0; limit <= count; ++limit)
        for (std::uint32_t available = 0; available <= count; ++available) {
            fil::mem::MemoryBus memory;
            ASSERT_TRUE(memory.mapRam(ram, 16U, "ram"));
            ASSERT_TRUE(memory.write32(ram + 4U, 0x80808080U));
            CpuState state;
            state.r[0] = ram; state.r[1] = 0x89abcdefU;
            state.xpsr = 0xf90f0000U; state.r[15] = pc;
            NativeExecutionContext ctx{&state, &memory, limit, 100U};
            ctx.charged_cycles = costs.data(); ctx.charged_cycles_count = available;
            (use_trace ? trace_fn : block_fn)(&ctx);
            const auto retired = std::min(limit, available);
            EXPECT_EQ(ctx.retired_count, retired); EXPECT_EQ(ctx.retired_cycles, retired * 2U);
            EXPECT_EQ(ctx.exit_reason, available < limit ? NativeExitReason::unsupported_instruction
                : limit < count ? NativeExitReason::instruction_limit : NativeExitReason::none);
            EXPECT_EQ(state.r[15], pc + retired * 2U);
            EXPECT_EQ(state.xpsr, 0xf90f0000U);
            if (retired >= 1U) EXPECT_EQ(memory.read32(ram).value(), 0x89abcdefU);
            if (retired >= 2U) EXPECT_EQ(memory.read32(ram + 4U).value(), 0x808080efU);
            if (retired >= 3U) EXPECT_EQ(memory.read16(ram + 8U).value(), 0xcdefU);
            if (retired >= 4U) EXPECT_EQ(state.r[2], 0x89abcdefU);
            if (retired >= 5U) EXPECT_EQ(state.r[3], 0xefU);
            if (retired >= 6U) EXPECT_EQ(state.r[4], 0xcdefU);
            if (retired >= 7U) EXPECT_EQ(state.r[5], 0xffffffefU);
            if (retired >= 8U) EXPECT_EQ(state.r[7], 0xffff80efU);
        }
    }
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

TEST(NativeJitTest, GuardedDecodedBranchTraceRetiresOnlyMatchingEdge) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08000000U;
    const auto branch = decode16(0xd000U); // BEQ, direct target pc + 4
    const auto nop = decode16(0xbf00U);
    ASSERT_TRUE(branch); ASSERT_TRUE(nop);
    std::array<NativeTraceOp, 2> source{};
    source[0].decoded = *branch; source[0].raw = 0xd000U; source[0].pc = pc;
    source[0].size = 2U; source[0].execution_cycles = 3U;
    source[0].expected_taken = true; source[0].expected_next_pc = pc + 4U;
    source[1].decoded = *nop; source[1].raw = 0xbf00U; source[1].pc = pc + 4U;
    source[1].size = 2U; source[1].execution_cycles = 2U;
    source[1].expected_next_pc = pc + 6U;
    NativeTracePlan plan; std::string error;
    ASSERT_TRUE(buildNativeTrace(source, plan, error)) << error;
    auto jit = MicroJit::create(error); ASSERT_NE(jit, nullptr);
    const auto fn = jit->compileNativeTrace(plan, error); ASSERT_NE(fn, nullptr) << error;
    fil::mem::MemoryBus bus;
    CpuState cpu; cpu.xpsr = xpsr_t | xpsr_z; cpu.r[15] = pc;
    const std::array<std::uint32_t, 2> costs{3U, 2U};
    NativeExecutionContext ctx{&cpu, &bus, 2U, 5U};
    ctx.charged_cycles = costs.data(); ctx.charged_cycles_count = costs.size();
    fn(&ctx);
    EXPECT_EQ(ctx.retired_count, 2U); EXPECT_EQ(ctx.retired_cycles, 5U);
    EXPECT_EQ(cpu.r[15], pc + 6U); EXPECT_EQ(cpu.instruction_address, pc + 4U);
    cpu = CpuState{}; cpu.r[15] = pc;
    ctx.retired_cycles = 0U; ctx.retired_count = 0U;
    fn(&ctx);
    EXPECT_EQ(ctx.retired_count, 0U);
    EXPECT_EQ(ctx.retired_cycles, 0U);
    EXPECT_EQ(ctx.exit_reason, NativeExitReason::branch_mismatch);
    EXPECT_EQ(ctx.failing_instruction_pc, pc); EXPECT_EQ(cpu.r[15], pc);
}

TEST(NativeJitTest, FlagProducerPrecedesGuardAndBLCommitsDirectEdge) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08011000U;
    const auto movs = decode16(0x2000U); // MOVS r0, #0 sets Z
    const auto beq = decode16(0xd000U);
    ASSERT_TRUE(movs); ASSERT_TRUE(beq);
    std::array<NativeTraceOp, 2> source{};
    source[0].decoded = *movs; source[0].raw = 0x2000U;
    source[0].pc = pc; source[0].size = 2U; source[0].execution_cycles = 1U;
    source[0].expected_next_pc = pc + 2U;
    source[1].decoded = *beq; source[1].raw = 0xd000U;
    source[1].pc = pc + 2U; source[1].size = 2U; source[1].execution_cycles = 2U;
    source[1].expected_taken = true; source[1].expected_next_pc = pc + 6U;
    NativeTracePlan plan; std::string error;
    ASSERT_TRUE(buildNativeTrace(source, plan, error)) << error;
    auto jit = MicroJit::create(error); ASSERT_NE(jit, nullptr);
    auto fn = jit->compileNativeTrace(plan, error); ASSERT_NE(fn, nullptr) << error;
    fil::mem::MemoryBus bus; CpuState cpu; cpu.r[15] = pc;
    cpu.xpsr = xpsr_t; // Initial Z=0; producer must set it before the guard.
    const std::array<std::uint32_t, 2> costs{1U, 2U};
    NativeExecutionContext ctx{&cpu, &bus, 2U, 3U};
    ctx.charged_cycles = costs.data(); ctx.charged_cycles_count = costs.size();
    fn(&ctx);
    EXPECT_EQ(ctx.retired_count, 2U); EXPECT_EQ(cpu.r[0], 0U);
    EXPECT_NE(cpu.xpsr & xpsr_z, 0U); EXPECT_EQ(cpu.r[15], pc + 6U);

    DecodedInstruction bl{}; bl.kind = InstrKind::bl; bl.condition = Condition::al;
    bl.form = OperandForm::immediate; bl.is_32bit = true; bl.branch_offset = 8;
    bl.raw = 0xf000f800U;
    NativeTraceOp call{}; call.decoded = bl; call.raw = 0xf000f800U; call.pc = pc + 0x20U;
    call.size = 4U; call.execution_cycles = 3U; call.expected_taken = true;
    call.expected_next_pc = call.pc + 12U;
    ASSERT_TRUE(buildNativeTrace({&call, 1U}, plan, error)) << error;
    fn = jit->compileNativeTrace(plan, error); ASSERT_NE(fn, nullptr) << error;
    cpu = CpuState{}; cpu.r[15] = call.pc;
    const std::array<std::uint32_t, 1> call_cost{3U};
    ctx = NativeExecutionContext{&cpu, &bus, 1U, 3U};
    ctx.charged_cycles = call_cost.data(); ctx.charged_cycles_count = 1U;
    fn(&ctx);
    EXPECT_EQ(ctx.retired_count, 1U); EXPECT_EQ(cpu.r[15], call.expected_next_pc);
    EXPECT_EQ(cpu.r[14], (call.pc + 5U) | 1U);
}

TEST(NativeJitTest, StorePersistsBeforeUnretiredMismatchAndHelperDecline) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08012000U, ram = 0x20000000U;
    const auto store = decode16(0x6008U); // STR r0, [r1]
    const auto branch = decode16(0xd000U);
    const auto load = decode16(0x6813U); // LDR r3, [r2]
    ASSERT_TRUE(store); ASSERT_TRUE(branch); ASSERT_TRUE(load);
    std::string error; auto jit = MicroJit::create(error); ASSERT_NE(jit, nullptr);
    fil::mem::MemoryBus bus; ASSERT_TRUE(bus.mapRam(ram, 16U, "ram"));
    NativeTracePlan plan;
    std::array<NativeTraceOp, 2> ops{};
    ops[0].decoded = *store; ops[0].raw = 0x6008U; ops[0].pc = pc;
    ops[0].size = 2U; ops[0].execution_cycles = 2U; ops[0].expected_next_pc = pc + 2U;
    ops[1].decoded = *branch; ops[1].raw = 0xd000U; ops[1].pc = pc + 2U;
    ops[1].size = 2U; ops[1].execution_cycles = 1U;
    ops[1].expected_taken = true; ops[1].expected_next_pc = pc + 6U;
    ASSERT_TRUE(buildNativeTrace(ops, plan, error)) << error;
    auto fn = jit->compileNativeTrace(plan, error); ASSERT_NE(fn, nullptr) << error;
    CpuState cpu; cpu.r[0] = 0xcafebabeU; cpu.r[1] = ram; cpu.r[15] = pc;
    const auto checkpoint = bus.sideEffectCheckpoint();
    const std::array<std::uint32_t, 2> costs{2U, 1U};
    NativeExecutionContext ctx{&cpu, &bus, 2U, 3U};
    ctx.charged_cycles = costs.data(); ctx.charged_cycles_count = costs.size();
    fn(&ctx);
    std::uint32_t value = 0U;
    ASSERT_TRUE(bus.tryFastRead32(ram, value, {fil::mem::AccessType::data_read, 0U}));
    EXPECT_EQ(value, 0xcafebabeU); EXPECT_EQ(bus.mutationSequence(), checkpoint.mutation_sequence + 4U);
    EXPECT_EQ(ctx.retired_count, 1U); EXPECT_EQ(ctx.retired_cycles, 2U);
    EXPECT_EQ(ctx.exit_reason, NativeExitReason::branch_mismatch);
    EXPECT_EQ(cpu.r[15], pc + 2U);
    ASSERT_TRUE(bus.restoreSideEffects(checkpoint));

    ops[1].decoded = *load; ops[1].raw = 0x6813U; ops[1].pc = pc + 2U;
    ops[1].size = 2U; ops[1].expected_taken = false; ops[1].expected_next_pc = pc + 4U;
    ASSERT_TRUE(buildNativeTrace(ops, plan, error)) << error;
    fn = jit->compileNativeTrace(plan, error); ASSERT_NE(fn, nullptr) << error;
    cpu = CpuState{}; cpu.r[0] = 0x12345678U; cpu.r[1] = ram;
    cpu.r[2] = 0x40000000U; cpu.r[15] = pc;
    ctx = NativeExecutionContext{&cpu, &bus, 2U, 3U};
    ctx.charged_cycles = costs.data(); ctx.charged_cycles_count = costs.size();
    fn(&ctx);
    EXPECT_EQ(ctx.retired_count, 1U); EXPECT_EQ(ctx.retired_cycles, 2U);
    EXPECT_EQ(ctx.exit_reason, NativeExitReason::helper_declined);
    EXPECT_EQ(ctx.failing_instruction_pc, pc + 2U);
}

TEST(NativeJitTest, TraceLimitsAndCycleAdmissionRetireExactPrefixes) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08013000U, ram = 0x20000000U;
    const auto movs = decode16(0x2001U), store = decode16(0x6008U), branch = decode16(0xd000U);
    ASSERT_TRUE(movs); ASSERT_TRUE(store); ASSERT_TRUE(branch);
    std::array<NativeTraceOp, 3> ops{};
    ops[0].decoded = *movs; ops[0].raw = 0x2001U; ops[0].pc = pc;
    ops[0].size = 2U; ops[0].execution_cycles = 1U; ops[0].expected_next_pc = pc + 2U;
    ops[1].decoded = *store; ops[1].raw = 0x6008U; ops[1].pc = pc + 2U;
    ops[1].size = 2U; ops[1].execution_cycles = 2U; ops[1].expected_next_pc = pc + 4U;
    ops[2].decoded = *branch; ops[2].raw = 0xd000U; ops[2].pc = pc + 4U;
    ops[2].size = 2U; ops[2].execution_cycles = 2U;
    ops[2].expected_taken = false; ops[2].expected_next_pc = pc + 6U;
    NativeTracePlan plan; std::string error;
    ASSERT_TRUE(buildNativeTrace(ops, plan, error)) << error;
    auto jit = MicroJit::create(error); ASSERT_NE(jit, nullptr);
    const auto fn = jit->compileNativeTrace(plan, error); ASSERT_NE(fn, nullptr) << error;
    fil::mem::MemoryBus bus; ASSERT_TRUE(bus.mapRam(ram, 16U, "ram"));
    const std::array<std::uint32_t, 3> costs{1U, 2U, 2U};
    for (std::uint32_t limit = 0; limit <= 3U; ++limit) {
        CpuState cpu; cpu.r[1] = ram; cpu.r[15] = pc;
        NativeExecutionContext ctx{&cpu, &bus, limit, 100U};
        ctx.charged_cycles = costs.data(); ctx.charged_cycles_count = costs.size();
        fn(&ctx);
        EXPECT_EQ(ctx.retired_count, limit);
        EXPECT_EQ(ctx.retired_cycles, limit == 0U ? 0U : limit == 1U ? 1U : limit == 2U ? 3U : 5U);
    }
    for (const std::uint64_t budget : {0U, 1U, 2U, 3U, 4U, 5U}) {
        CpuState cpu; cpu.r[1] = ram; cpu.r[15] = pc;
        NativeExecutionContext ctx{&cpu, &bus, 3U, budget};
        ctx.charged_cycles = costs.data(); ctx.charged_cycles_count = costs.size();
        fn(&ctx);
        const auto prefix = budget == 0U ? 0U : budget <= 2U ? 1U : budget < 5U ? 2U : 3U;
        EXPECT_EQ(ctx.retired_count, prefix) << budget;
        EXPECT_EQ(ctx.retired_cycles, prefix == 0U ? 0U : prefix == 1U ? 1U : prefix == 2U ? 3U : 5U);
        if (budget < 5U) EXPECT_EQ(ctx.exit_reason, NativeExitReason::cycle_limit);
    }
}

TEST(NativeJitTest, TraceValidationRejectsBadEdgesAndOverlongWithoutMutatingOutput) {
    const auto nop = decode16(0xbf00U); ASSERT_TRUE(nop);
    NativeTraceOp good{}; good.decoded = *nop; good.raw = 0xbf00U;
    good.pc = 0x08014000U; good.size = 2U; good.execution_cycles = 1U;
    good.expected_next_pc = good.pc + 2U;
    NativeTracePlan plan; std::string error;
    ASSERT_TRUE(buildNativeTrace({&good, 1U}, plan, error)) << error;
    const auto old_pc = plan.ops.front().pc;
    plan.generation = 99U;
    auto malformed = good; malformed.expected_next_pc += 2U;
    EXPECT_FALSE(buildNativeTrace({&malformed, 1U}, plan, error));
    ASSERT_EQ(plan.ops.size(), 1U); EXPECT_EQ(plan.ops.front().pc, old_pc);
    EXPECT_EQ(plan.generation, 99U); EXPECT_EQ(plan.function, nullptr);
    std::array<NativeTraceOp, NativeTracePlan::max_ops + 1U> too_many{};
    for (std::size_t i = 0; i < too_many.size(); ++i) {
        too_many[i] = good; too_many[i].pc = good.pc + static_cast<std::uint32_t>(2U * i);
        too_many[i].expected_next_pc = too_many[i].pc + 2U;
    }
    EXPECT_FALSE(buildNativeTrace(too_many, plan, error));
    ASSERT_EQ(plan.ops.size(), 1U); EXPECT_EQ(plan.ops.front().pc, old_pc);
}

TEST(NativeJitTest, BxLrIsTraceOnlyAndRequiresCanonicalOperands) {
    const auto bx = decode16(0x4770U); ASSERT_TRUE(bx);
    NativeInst lowered;
    ASSERT_TRUE(tryLowerNativeInst({*bx, 0x08014d00U, 2U}, lowered).ok);
    EXPECT_EQ(lowered.kind, NativeOp::bx_lr);
    auto malformed = *bx; malformed.condition = Condition::eq;
    EXPECT_FALSE(tryLowerNativeInst({malformed, 0x08014d00U, 2U}, lowered).ok);
    malformed = *bx; malformed.rn = 1U;
    EXPECT_FALSE(tryLowerNativeInst({malformed, 0x08014d00U, 2U}, lowered).ok);
    NativeBlock block;
    const std::array<MicroSource, 1> block_source{{{*bx, 0x08014d00U, 2U}}};
    EXPECT_FALSE(buildNativeBlock(block_source, block).ok);
    block.entry_pc = 0x08014d00U; block.ops.push_back(lowered);
    std::string compile_error;
    auto jit = MicroJit::create(compile_error); ASSERT_TRUE(jit);
    EXPECT_EQ(jit->compileNative(block, compile_error), nullptr);

    NativeTraceOp op{}; op.decoded = *bx; op.raw = 0x4770U; op.pc = 0x08014d00U;
    op.size = 2U; op.execution_cycles = 3U; op.expected_taken = true;
    NativeTracePlan plan; std::string error;
    for (const auto target : {0xfffffff0U, 0xfffffff2U, 0xfffffff4U, 0xfffffff6U,
                              0xfffffff8U, 0xfffffffaU, 0xfffffffcU, 0xfffffffeU}) {
        op.expected_next_pc = target;
        EXPECT_FALSE(buildNativeTrace({&op, 1U}, plan, error)) << std::hex << target;
    }
}

TEST(NativeJitTest, GuardedBlxRegisterStitchesAndExitsBeforeLinkOnMismatch) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08014c00U;
    const auto blx = decode16(0x4780U); ASSERT_TRUE(blx);
    const auto nop = decode16(0xbf00U); ASSERT_TRUE(nop);
    NativeInst lowered;
    ASSERT_TRUE(tryLowerNativeInst({*blx, pc, 2U}, lowered).ok);
    ASSERT_EQ(lowered.kind, NativeOp::blx_reg);
    NativeTraceOp ops[2]{};
    ops[0].decoded = *blx; ops[0].raw = blx->raw; ops[0].pc = pc;
    ops[0].size = 2U; ops[0].execution_cycles = 3U; ops[0].expected_taken = true;
    ops[0].expected_next_pc = pc + 10U; // bit 1 set in the halfword target
    ops[1].decoded = *nop; ops[1].raw = nop->raw; ops[1].pc = pc + 10U;
    ops[1].size = 2U; ops[1].execution_cycles = 1U; ops[1].expected_next_pc = pc + 12U;
    NativeTracePlan plan; std::string error;
    ASSERT_TRUE(buildNativeTrace(ops, plan, error)) << error;
    auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto fn = jit->compileNativeTrace(plan, error); ASSERT_NE(fn, nullptr) << error;
    const std::array<std::uint32_t, 2> costs{3U, 1U};
    for (const std::uint32_t input : {pc + 11U, pc + 9U, pc + 13U, 0xfffffff1U}) {
        for (std::uint32_t limit = 0U; limit <= 2U; ++limit)
        for (std::uint64_t credit = 0U; credit <= 4U; ++credit) {
            CpuState actual; actual.r[0] = input; actual.r[14] = 0x12345678U; actual.r[15] = pc;
            CpuState expected = actual;
            fil::mem::MemoryBus memory, oracle;
            NativeExecutionContext a{&actual, &memory, limit, credit};
            NativeExecutionContext e{&expected, &oracle, limit, credit};
            a.charged_cycles = e.charged_cycles = costs.data();
            a.charged_cycles_count = e.charged_cycles_count = costs.size();
            interpretNativeTrace(plan, e); fn(&a);
            EXPECT_EQ(a.retired_count, e.retired_count);
            EXPECT_EQ(a.retired_cycles, e.retired_cycles);
            EXPECT_EQ(a.exit_reason, e.exit_reason);
            EXPECT_EQ(a.failing_instruction_pc, e.failing_instruction_pc);
            EXPECT_TRUE(bitwiseEqual(actual, expected));
            if (input != pc + 11U) {
                EXPECT_EQ(a.retired_count, 0U);
                EXPECT_EQ(actual.r[14], 0x12345678U);
            }
        }
    }
}

TEST(NativeJitTest, BlxLrAliasGuardsOldValueBeforeWritingLink) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08014c80U, target = pc + 10U;
    const auto blx = decode16(0x47f0U); ASSERT_TRUE(blx);
    const auto nop = decode16(0xbf00U); ASSERT_TRUE(nop);
    NativeTraceOp ops[2]{};
    ops[0].decoded = *blx; ops[0].raw = blx->raw; ops[0].pc = pc;
    ops[0].size = 2U; ops[0].execution_cycles = 3U; ops[0].expected_taken = true;
    ops[0].expected_next_pc = target;
    ops[1].decoded = *nop; ops[1].raw = nop->raw; ops[1].pc = target;
    ops[1].size = 2U; ops[1].execution_cycles = 1U; ops[1].expected_next_pc = target + 2U;
    NativeTracePlan plan; std::string error;
    ASSERT_TRUE(buildNativeTrace(ops, plan, error)) << error;
    auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto fn = jit->compileNativeTrace(plan, error); ASSERT_NE(fn, nullptr) << error;
    const std::array<std::uint32_t, 2> costs{3U, 1U};
    for (const std::uint32_t old_lr : {target | 1U, target, target + 5U}) {
        CpuState actual; actual.r[14] = old_lr; actual.r[15] = pc;
        CpuState expected = actual;
        fil::mem::MemoryBus memory, oracle;
        NativeExecutionContext a{&actual, &memory, 2U, 4U};
        NativeExecutionContext e{&expected, &oracle, 2U, 4U};
        a.charged_cycles = e.charged_cycles = costs.data();
        a.charged_cycles_count = e.charged_cycles_count = costs.size();
        interpretNativeTrace(plan, e); fn(&a);
        EXPECT_EQ(a.retired_count, e.retired_count);
        EXPECT_EQ(a.exit_reason, e.exit_reason);
        EXPECT_TRUE(bitwiseEqual(actual, expected));
        if (old_lr == (target | 1U)) {
            EXPECT_EQ(a.retired_count, 2U);
            EXPECT_EQ(actual.r[14], (pc + 2U) | 1U);
        } else {
            EXPECT_EQ(a.retired_count, 0U);
            EXPECT_EQ(actual.r[14], old_lr);
        }
    }
}

TEST(NativeJitTest, EmittedCallBxLrChainMatchesPortableAtEveryCut) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08014e00U;
    DecodedInstruction bl{}; bl.kind = InstrKind::bl; bl.condition = Condition::al;
    bl.form = OperandForm::immediate; bl.is_32bit = true; bl.branch_offset = 2;
    bl.raw = 0xf000f800U;
    const auto bx = decode16(0x4770U); ASSERT_TRUE(bx);
    std::array<NativeTraceOp, 2> ops{};
    ops[0].decoded = bl; ops[0].raw = bl.raw; ops[0].pc = pc;
    ops[0].size = 4U; ops[0].execution_cycles = 3U;
    ops[0].expected_taken = true; ops[0].expected_next_pc = pc + 6U;
    ops[1].decoded = *bx; ops[1].raw = bx->raw; ops[1].pc = pc + 6U;
    ops[1].size = 2U; ops[1].execution_cycles = 3U;
    ops[1].expected_taken = true; ops[1].expected_next_pc = pc + 4U;
    NativeTracePlan plan; std::string error;
    ASSERT_TRUE(buildNativeTrace(ops, plan, error)) << error;
    auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto fn = jit->compileNativeTrace(plan, error); ASSERT_NE(fn, nullptr) << error;
    const std::array<std::uint32_t, 2> costs{3U, 3U};
    for (std::uint32_t limit = 0U; limit <= 2U; ++limit)
    for (std::uint64_t credit = 0U; credit <= 6U; ++credit) {
        CpuState actual; actual.r[15] = pc;
        CpuState expected = actual;
        fil::mem::MemoryBus memory, oracle;
        NativeExecutionContext a{&actual, &memory, limit, credit};
        NativeExecutionContext e{&expected, &oracle, limit, credit};
        a.charged_cycles = e.charged_cycles = costs.data();
        a.charged_cycles_count = e.charged_cycles_count = costs.size();
        interpretNativeTrace(plan, e); fn(&a);
        EXPECT_EQ(a.retired_count, e.retired_count) << limit << "," << credit;
        EXPECT_EQ(a.retired_cycles, e.retired_cycles) << limit << "," << credit;
        EXPECT_EQ(a.exit_reason, e.exit_reason) << limit << "," << credit;
        EXPECT_TRUE(bitwiseEqual(actual, expected)) << limit << "," << credit;
        if (limit == 2U && credit == 6U) {
            EXPECT_EQ(a.retired_count, 2U); EXPECT_EQ(actual.r[15], pc + 4U);
        }
    }
}

TEST(NativeJitTest, EmittedBxLrReturnIsGuardedBeforeRetirement) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08014f00U, target = pc + 0x40U;
    const auto bx = decode16(0x4770U); ASSERT_TRUE(bx);
    NativeTraceOp op{}; op.decoded = *bx; op.raw = 0x4770U; op.pc = pc;
    op.size = 2U; op.execution_cycles = 3U; op.expected_taken = true;
    op.expected_next_pc = target;
    NativeTracePlan plan; std::string error;
    ASSERT_TRUE(buildNativeTrace({&op, 1U}, plan, error)) << error;
    auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto fn = jit->compileNativeTrace(plan, error); ASSERT_NE(fn, nullptr) << error;
    const std::array<std::uint32_t, 1> cost{3U};
    for (const auto lr : {target | 1U, target, 0xfffffff1U, target + 3U}) {
        CpuState actual; actual.r[14] = lr; actual.r[15] = pc;
        CpuState expected = actual;
        fil::mem::MemoryBus memory, oracle;
        NativeExecutionContext a{&actual, &memory, 1U, 3U};
        NativeExecutionContext e{&expected, &oracle, 1U, 3U};
        a.charged_cycles = e.charged_cycles = cost.data();
        a.charged_cycles_count = e.charged_cycles_count = 1U;
        interpretNativeTrace(plan, e); fn(&a);
        EXPECT_EQ(a.retired_count, e.retired_count);
        EXPECT_EQ(a.retired_cycles, e.retired_cycles);
        EXPECT_EQ(a.exit_reason, e.exit_reason);
        EXPECT_EQ(a.failing_instruction_pc, e.failing_instruction_pc);
        EXPECT_TRUE(bitwiseEqual(actual, expected));
        if (lr == (target | 1U)) EXPECT_EQ(a.retired_count, 1U);
        else { EXPECT_EQ(a.retired_count, 0U); EXPECT_EQ(a.exit_reason, NativeExitReason::branch_mismatch); }
    }
}

TEST(NativeJitTest, EmittedStackReturnTraceMatchesPortableFallbackAtEveryBoundary) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08015000U, ram = 0x20000000U;
    const std::array<std::uint16_t, 5> words{0xb501U, 0x2009U, 0xbd01U, 0x2200U, 0x680bU};
    const std::array<std::uint32_t, 5> pcs{pc, pc + 2U, pc + 4U, pc + 16U, pc + 18U};
    const std::array<std::uint32_t, 5> costs{3U, 1U, 5U, 1U, 2U};
    std::array<NativeTraceOp, 5> ops;
    for (std::size_t i = 0U; i < ops.size(); ++i) {
        const auto decoded = decode16(words[i]); ASSERT_TRUE(decoded);
        ops[i].decoded = *decoded; ops[i].raw = decoded->raw;
        ops[i].pc = pcs[i]; ops[i].size = 2U;
        ops[i].execution_cycles = static_cast<std::uint16_t>(costs[i]);
        ops[i].expected_next_pc = i == 2U ? pc + 16U : pcs[i] + 2U;
        ops[i].expected_taken = i == 2U;
    }
    NativeTracePlan plan; std::string error;
    ASSERT_TRUE(buildNativeTrace(ops, plan, error)) << error;
    auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    const auto fn = jit->compileNativeTrace(plan, error); ASSERT_NE(fn, nullptr) << error;
    for (unsigned mode = 0U; mode < 3U; ++mode) {
        for (const bool mismatch : {false, true}) {
            for (const bool bad_address : {false, true})
            for (std::uint32_t limit = 0U; limit <= ops.size(); ++limit) {
                for (std::uint64_t credit = 0U; credit <= 12U; ++credit) {
                    fil::mem::MemoryBus memory, oracle;
                    ASSERT_TRUE(memory.mapRam(ram, 128U, "stack"));
                    ASSERT_TRUE(oracle.mapRam(ram, 128U, "stack"));
                    CpuState state;
                    state.r[0] = 0x11223344U; state.r[14] = pc + (mismatch ? 19U : 17U);
                    state.r[1] = bad_address ? 0x40000000U : ram;
                    state.r[15] = pc; state.instruction_address = pc + 64U;
                    state.msp = ram + 64U; state.psp = ram + 96U; state.r[13] = ram + 32U;
                    state.control = mode == 0U ? 0U : 2U;
                    state.xpsr |= xpsr_c | xpsr_q | (mode == 2U ? 3U : 0U);
                    const auto stack = state.readRegister(13U) - 8U;
                    auto reference = state;
                    NativeExecutionContext context{&state, &memory, limit, credit};
                    NativeExecutionContext expected{&reference, &oracle, limit, credit};
                    context.charged_cycles = expected.charged_cycles = costs.data();
                    context.charged_cycles_count = expected.charged_cycles_count = costs.size();
                    interpretNativeTrace(plan, expected);
                    fn(&context);
                    EXPECT_EQ(context.retired_count, expected.retired_count);
                    EXPECT_EQ(context.retired_cycles, expected.retired_cycles);
                    EXPECT_EQ(context.exit_reason, expected.exit_reason);
                    EXPECT_EQ(context.failing_instruction_pc, expected.failing_instruction_pc);
                    EXPECT_TRUE(bitwiseEqual(state, reference));
                    EXPECT_EQ(memory.readFootprint().words, oracle.readFootprint().words);
                    EXPECT_EQ(memory.read32(stack).value(), oracle.read32(stack).value());
                    EXPECT_EQ(memory.read32(stack + 4U).value(), oracle.read32(stack + 4U).value());
                    EXPECT_EQ(context.current_instruction_pc, expected.current_instruction_pc);
                    if (!mismatch && !bad_address && limit == 5U && credit == 12U) {
                        EXPECT_EQ(context.retired_count, 5U);
                        EXPECT_EQ(state.r[15], pc + 20U);
                        EXPECT_NE(state.xpsr & xpsr_z, 0U);
                        EXPECT_EQ(state.r[0], 0x11223344U);
                        EXPECT_EQ(state.readRegister(13U), stack + 8U);
                    }
                }
            }
        }
    }
}

TEST(NativeJitTest, DeferredMetadataAndLiveCyclesMatchOracleAcrossExitMatrix) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08016000U, ram = 0x20000000U;
    const auto movs = decode16(0x2000U); // MOVS r0,#0 establishes Z
    const auto load = decode16(0x6811U); // LDR r1,[r2]
    const auto beq = decode16(0xd000U);
    ASSERT_TRUE(movs); ASSERT_TRUE(load); ASSERT_TRUE(beq);
    std::array<NativeTraceOp, 3> ops{};
    ops[0].decoded=*movs; ops[0].raw=0x2000U; ops[0].pc=pc;
    ops[0].size=2U; ops[0].execution_cycles=2U; ops[0].expected_next_pc=pc+2U;
    ops[1].decoded=*load; ops[1].raw=0x6811U; ops[1].pc=pc+2U;
    ops[1].size=2U; ops[1].execution_cycles=3U; ops[1].expected_next_pc=pc+4U;
    ops[2].decoded=*beq; ops[2].raw=0xd000U; ops[2].pc=pc+4U;
    ops[2].size=2U; ops[2].execution_cycles=1U; ops[2].expected_taken=true;
    ops[2].expected_next_pc=pc+8U;
    NativeTracePlan plan; std::string error;
    ASSERT_TRUE(buildNativeTrace(ops,plan,error)) << error;
    auto jit=MicroJit::create(error); ASSERT_TRUE(jit);
    const auto fn=jit->compileNativeTrace(plan,error); ASSERT_NE(fn,nullptr) << error;
    const std::array<std::uint32_t,3> costs{2U,3U,1U};
    constexpr std::array<std::uint64_t,4> initial_values{0U,5U,UINT64_MAX-1U,UINT64_MAX};
    constexpr std::array<std::uint64_t,5> budgets{0U,4U,5U,6U,UINT64_MAX};
    for (bool bad_address : {false,true}) for (std::uint32_t limit=0;limit<=3U;++limit)
    for (const std::uint64_t initial_cycles : initial_values)
    for (const std::uint64_t budget : budgets) {
        fil::mem::MemoryBus memory, oracle;
        ASSERT_TRUE(memory.mapRam(ram,16U,"ram")); ASSERT_TRUE(oracle.mapRam(ram,16U,"ram"));
        ASSERT_TRUE(memory.write32(ram+4U,0xdecafbadU)); ASSERT_TRUE(oracle.write32(ram+4U,0xdecafbadU));
        CpuState state; state.r[15]=0xfeed0010U; state.instruction_address=0xfeed0020U;
        state.xpsr=xpsr_t|xpsr_n|xpsr_c|xpsr_q|0x00030000U; state.r[2]=bad_address?0x40000000U:ram+4U;
        CpuState reference=state;
        NativeExecutionContext actual{&state,&memory,limit,budget};
        NativeExecutionContext expected{&reference,&oracle,limit,budget};
        actual.retired_count=expected.retired_count=77U;
        actual.current_instruction_pc=expected.current_instruction_pc=0xaaaa5555U;
        actual.failing_instruction_pc=expected.failing_instruction_pc=0xbbbb6666U;
        actual.exit_reason=expected.exit_reason=NativeExitReason::control_flow;
        actual.fetch_cycles=expected.fetch_cycles=19U;
        actual.retired_cycles=expected.retired_cycles=initial_cycles;
        actual.charged_cycles=expected.charged_cycles=costs.data();
        actual.charged_cycles_count=expected.charged_cycles_count=costs.size();
        interpretNativeTrace(plan,expected); fn(&actual);
        EXPECT_EQ(actual.retired_count,expected.retired_count);
        EXPECT_EQ(actual.retired_cycles,expected.retired_cycles);
        EXPECT_EQ(actual.current_instruction_pc,expected.current_instruction_pc);
        EXPECT_EQ(actual.failing_instruction_pc,expected.failing_instruction_pc);
        EXPECT_EQ(actual.exit_reason,expected.exit_reason);
        EXPECT_EQ(actual.fetch_cycles,expected.fetch_cycles);
        EXPECT_EQ(actual.instruction_limit,expected.instruction_limit);
        EXPECT_EQ(actual.cycle_limit,expected.cycle_limit);
        EXPECT_EQ(actual.charged_cycles,expected.charged_cycles);
        EXPECT_EQ(actual.charged_cycles_count,expected.charged_cycles_count);
        EXPECT_EQ(actual.cpu_state,&state); EXPECT_EQ(actual.memory_bus,&memory);
        EXPECT_TRUE(bitwiseEqual(state,reference));
        EXPECT_EQ(memory.readFootprint().words,oracle.readFootprint().words);
        EXPECT_EQ(memory.mutationSequence(),oracle.mutationSequence());
    }
}

TEST(NativeJitTest, LiveNzcvSurvivesRamAndStackHelpersAtEveryPrefixAndCredit) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08016800U, ram = 0x20000000U;
    const std::array decoded{decode16(0x0064U), decode16(0x600cU), // LSLS r4,#1; STR r4,[r1]
        decode32(0xeb15U, 0x0536U), decode16(0x680fU),          // ADDS r5,r5,r6,RRX; LDR r7,[r1]
        decode16(0xb510U), decode16(0xbc10U),                  // PUSH {r4,lr}; POP {r4}
        decode32(0xeb18U, 0x0837U)};                           // ADDS r8,r8,r7,RRX
    constexpr std::array<std::uint8_t, 7> sizes{2U, 2U, 4U, 2U, 2U, 2U, 4U};
    constexpr std::array<std::uint32_t, 7> costs{1U, 2U, 1U, 2U, 3U, 2U, 1U};
    std::array<NativeTraceOp, 7> ops{};
    auto next_pc = pc;
    for (std::size_t i = 0U; i < ops.size(); ++i) {
        ASSERT_TRUE(decoded[i]);
        ops[i].decoded = *decoded[i]; ops[i].raw = decoded[i]->raw;
        ops[i].pc = next_pc; ops[i].size = sizes[i];
        next_pc += sizes[i]; ops[i].expected_next_pc = next_pc;
        ops[i].execution_cycles = static_cast<std::uint16_t>(costs[i]);
    }
    NativeTracePlan plan; std::string error;
    ASSERT_TRUE(buildNativeTrace(ops, plan, error)) << error;
    auto jit = MicroJit::create(error); ASSERT_TRUE(jit) << error;
    const auto function = jit->compileNativeTrace(plan, error);
    ASSERT_NE(function, nullptr) << error;
    for (std::uint32_t flags = 0U; flags < 16U; ++flags)
    for (std::uint32_t mode = 0U; mode < 3U; ++mode)
    for (std::uint32_t limit = 0U; limit <= ops.size(); ++limit)
    for (std::uint64_t credit = 0U; credit <= 13U; ++credit) {
        SCOPED_TRACE(flags);
        SCOPED_TRACE(mode);
        SCOPED_TRACE(limit);
        SCOPED_TRACE(credit);
        fil::mem::MemoryBus memory, oracle;
        ASSERT_TRUE(memory.mapRam(ram, 128U, "ram"));
        ASSERT_TRUE(oracle.mapRam(ram, 128U, "ram"));
        CpuState state;
        for (std::size_t reg = 0U; reg < state.r.size(); ++reg)
            state.r[reg] = 0xa5000000U + static_cast<std::uint32_t>(reg);
        state.r[1] = mode == 1U ? 0x40000000U : ram + 4U;
        state.r[4] = (flags & 1U) ? 0x80000001U : 0x40000000U;
        state.r[6] = (flags & 2U) ? 3U : 2U;
        state.r[15] = 0xfeed0010U; state.instruction_address = 0xfeed0020U;
        state.xpsr = xpsr_t | xpsr_q | xpsr_ge_mask | (flags << 28U)
            | ((flags & 4U) ? 3U : 0U);
        state.control = 2U;
        state.msp = mode == 2U ? 0x40000040U : ram + 64U;
        state.psp = mode == 2U ? 0x40000060U : ram + 96U;
        auto reference = state;
        NativeExecutionContext actual{&state, &memory, limit, credit};
        NativeExecutionContext expected{&reference, &oracle, limit, credit};
        actual.charged_cycles = expected.charged_cycles = costs.data();
        actual.charged_cycles_count = expected.charged_cycles_count =
            static_cast<std::uint32_t>(costs.size());
        actual.current_instruction_pc = expected.current_instruction_pc = 0xaaaa5555U;
        actual.failing_instruction_pc = expected.failing_instruction_pc = 0xbbbb6666U;
        interpretNativeTrace(plan, expected); function(&actual);
        EXPECT_EQ(actual.retired_count, expected.retired_count);
        EXPECT_EQ(actual.retired_cycles, expected.retired_cycles);
        EXPECT_EQ(actual.current_instruction_pc, expected.current_instruction_pc);
        EXPECT_EQ(actual.failing_instruction_pc, expected.failing_instruction_pc);
        EXPECT_EQ(actual.exit_reason, expected.exit_reason);
        EXPECT_TRUE(bitwiseEqual(state, reference));
        EXPECT_EQ(memory.readFootprint().words, oracle.readFootprint().words);
        EXPECT_EQ(memory.mutationSequence(), oracle.mutationSequence());
        for (std::uint32_t offset = 0U; offset < 128U; offset += 4U)
            EXPECT_EQ(memory.read32(ram + offset).value(), oracle.read32(ram + offset).value());
    }
}

TEST(NativeJitTest, TracePreservesHostCalleeSavedRegistersAcrossStackHelpers) {
#if (defined(__x86_64__) || defined(__aarch64__)) && (defined(__GNUC__) || defined(__clang__)) && !defined(_WIN32)
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08017000U, ram = 0x20000000U;
    const auto push = decode16(0xb501U), pop = decode16(0xbd01U);
    ASSERT_TRUE(push); ASSERT_TRUE(pop);
    std::array<NativeTraceOp, 2> ops;
    ops[0].decoded = *push; ops[0].raw = push->raw; ops[0].pc = pc;
    ops[0].size = 2U; ops[0].execution_cycles = 3U; ops[0].expected_next_pc = pc + 2U;
    ops[1].decoded = *pop; ops[1].raw = pop->raw; ops[1].pc = pc + 2U;
    ops[1].size = 2U; ops[1].execution_cycles = 5U;
    ops[1].expected_next_pc = pc + 16U; ops[1].expected_taken = true;
    NativeTracePlan plan; std::string error;
    ASSERT_TRUE(buildNativeTrace(ops, plan, error)) << error;
    auto jit = MicroJit::create(error); ASSERT_TRUE(jit);
    auto function = jit->compileNativeTrace(plan, error); ASSERT_NE(function, nullptr) << error;
    fil::mem::MemoryBus memory; ASSERT_TRUE(memory.mapRam(ram, 64U, "stack"));
    CpuState state; state.r[0] = 123U; state.r[14] = pc + 17U;
    state.r[15] = pc; state.r[13] = state.msp = ram + 32U;
    const std::array<std::uint32_t, 2> costs{3U, 5U};
    NativeExecutionContext context{&state, &memory, 2U, 8U};
    context.charged_cycles = costs.data(); context.charged_cycles_count = costs.size();
#if defined(__aarch64__)
    std::array<std::uint64_t, 11> observed{};
    register auto* argument asm("x0") = &context;
    register auto target asm("x16") = function;
    register auto* output asm("x2") = observed.data();
    // Save the caller's nonvolatile registers and LR, plus the output pointer.
    // Keep SP aligned and do not touch Darwin's reserved x18.
    asm volatile(
        "sub sp, sp, #112\n\t"
        "stp x19, x20, [sp, #0]\n\tstp x21, x22, [sp, #16]\n\t"
        "stp x23, x24, [sp, #32]\n\tstp x25, x26, [sp, #48]\n\t"
        "stp x27, x28, [sp, #64]\n\tstp x29, x30, [sp, #80]\n\t"
        "str x2, [sp, #96]\n\t"
        "mov x19, #0x5a5a\n\tmov x20, x19\n\tmov x21, x19\n\tmov x22, x19\n\t"
        "mov x23, x19\n\tmov x24, x19\n\tmov x25, x19\n\tmov x26, x19\n\t"
        "mov x27, x19\n\tmov x28, x19\n\tmov x29, x19\n\t"
        "blr x16\n\tldr x2, [sp, #96]\n\t"
        "stp x19, x20, [x2, #0]\n\tstp x21, x22, [x2, #16]\n\t"
        "stp x23, x24, [x2, #32]\n\tstp x25, x26, [x2, #48]\n\t"
        "stp x27, x28, [x2, #64]\n\tstr x29, [x2, #80]\n\t"
        "ldp x19, x20, [sp, #0]\n\tldp x21, x22, [sp, #16]\n\t"
        "ldp x23, x24, [sp, #32]\n\tldp x25, x26, [sp, #48]\n\t"
        "ldp x27, x28, [sp, #64]\n\tldp x29, x30, [sp, #80]\n\t"
        "add sp, sp, #112"
        : "+r"(argument), "+r"(target), "+r"(output)
        :
        : "x1", "x3", "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11",
          "x12", "x13", "x14", "x15", "x17", "x30", "cc", "memory",
          "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7",
          "v8", "v9", "v10", "v11", "v12", "v13", "v14", "v15",
          "v16", "v17", "v18", "v19", "v20", "v21", "v22", "v23",
          "v24", "v25", "v26", "v27", "v28", "v29", "v30", "v31");
    for (const auto value : observed) EXPECT_EQ(value, 0x5a5aU);
#else
    std::array<std::uint64_t, 6> observed{};
    auto* argument = &context;
    // Six register saves plus the saved output pointer and padding preserve
    // 16-byte call-site alignment. Restore the compiler's own frame/registers.
    asm volatile(
        "push %%rbp\n\tpush %%rbx\n\tpush %%r12\n\tpush %%r13\n\tpush %%r14\n\tpush %%r15\n\t"
        "push %%rdx\n\tsub $8, %%rsp\n\t"
        "movabs $0x1122334455667788, %%rbp\n\tmovabs $0x1122334455667788, %%rbx\n\t"
        "movabs $0x1122334455667788, %%r12\n\tmovabs $0x1122334455667788, %%r13\n\t"
        "movabs $0x1122334455667788, %%r14\n\tmovabs $0x1122334455667788, %%r15\n\t"
        "call *%%rax\n\tadd $8, %%rsp\n\tpop %%rdx\n\t"
        "mov %%rbp, 0(%%rdx)\n\tmov %%rbx, 8(%%rdx)\n\tmov %%r12, 16(%%rdx)\n\t"
        "mov %%r13, 24(%%rdx)\n\tmov %%r14, 32(%%rdx)\n\tmov %%r15, 40(%%rdx)\n\t"
        "pop %%r15\n\tpop %%r14\n\tpop %%r13\n\tpop %%r12\n\tpop %%rbx\n\tpop %%rbp"
        : "+a"(function), "+D"(argument)
        : "d"(observed.data())
        : "rcx", "rsi", "r8", "r9", "r10", "r11", "cc", "memory");
    for (const auto value : observed) EXPECT_EQ(value, 0x1122334455667788ULL);
#endif
    EXPECT_EQ(context.retired_count, 2U);
    EXPECT_EQ(context.retired_cycles, 8U);
    EXPECT_EQ(state.r[15], pc + 16U);
#else
    GTEST_SKIP() << "AArch64 or SysV x86-64 ABI harness";
#endif
}

TEST(NativeJitTest, NullTraceInputsDeclineWithoutHostDereferenceOrRetirement) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    const auto decoded = decode16(0xbf00U);
    ASSERT_TRUE(decoded);
    NativeTraceOp op;
    op.decoded = *decoded; op.raw = decoded->raw;
    op.pc = 0x08015000U; op.size = 2U;
    op.execution_cycles = 1U; op.expected_next_pc = op.pc + 2U;
    NativeTracePlan plan;
    std::string error;
    ASSERT_TRUE(buildNativeTrace({&op, 1U}, plan, error));
    auto jit = MicroJit::create(error);
    ASSERT_TRUE(jit);
    const auto fn = jit->compileNativeTrace(plan, error);
    ASSERT_NE(fn, nullptr) << error;
    fn(nullptr);
    CpuState state;
    state.r[15] = op.pc;
    state.xpsr |= xpsr_n | xpsr_q;
    const auto before = state;
    fil::mem::MemoryBus memory;
    NativeExecutionContext context{&state, &memory, 1U, 100U};
    context.charged_cycles_count = 1U; // Pointer deliberately remains null.
    fn(&context);
    EXPECT_EQ(context.retired_count, 0U);
    EXPECT_EQ(context.retired_cycles, 0U);
    EXPECT_EQ(context.exit_reason, NativeExitReason::unsupported_instruction);
    EXPECT_EQ(context.failing_instruction_pc, op.pc);
    EXPECT_TRUE(bitwiseEqual(state, before));
    context.instruction_limit = 0U;
    fn(&context);
    EXPECT_EQ(context.exit_reason, NativeExitReason::instruction_limit);
    context.cpu_state = nullptr;
    context.retired_count = 77U;
    fn(&context);
    EXPECT_EQ(context.retired_count, 77U);
}

TEST(NativeJitTest, EveryConditionalBranchGuardsPackedNZCV) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no native emitter on this host";
    constexpr std::uint32_t pc = 0x08010000U;
    constexpr std::array<std::uint32_t, 4> masks{xpsr_n, xpsr_z, xpsr_c, xpsr_v};
    std::string error;
    auto jit = MicroJit::create(error); ASSERT_NE(jit, nullptr);
    fil::mem::MemoryBus bus;
    const std::array<std::uint32_t, 1> costs{1U};
    for (unsigned cond = 0; cond < 14U; ++cond) {
        const auto encoding = static_cast<std::uint16_t>(0xd000U | (cond << 8U));
        const auto decoded = decode16(encoding); ASSERT_TRUE(decoded) << cond;
        for (unsigned flags = 0; flags < 16U; ++flags) {
            std::uint32_t psr = xpsr_t;
            for (unsigned bit = 0; bit < 4U; ++bit)
                if ((flags & (1U << bit)) != 0U) psr |= masks[bit];
            const bool actual = conditionPasses(decoded->condition, psr);
            for (unsigned expect = 0; expect < 2U; ++expect) {
                NativeTraceOp src{};
                src.decoded = *decoded; src.raw = encoding; src.pc = pc; src.size = 2U;
                src.execution_cycles = 1U; src.expected_taken = expect != 0U;
                src.expected_next_pc = expect != 0U ? pc + 4U : pc + 2U;
                NativeTracePlan plan;
                ASSERT_TRUE(buildNativeTrace({&src, 1U}, plan, error)) << error;
                const auto fn = jit->compileNativeTrace(plan, error);
                ASSERT_NE(fn, nullptr) << error;
                CpuState cpu; cpu.xpsr = psr; cpu.r[15] = pc;
                NativeExecutionContext ctx{&cpu, &bus, 1U, 1U};
                ctx.charged_cycles = costs.data(); ctx.charged_cycles_count = costs.size();
                auto reference = cpu;
                auto expected = ctx;
                expected.cpu_state = &reference;
                interpretNativeTrace(plan, expected);
                fn(&ctx);
                EXPECT_TRUE(bitwiseEqual(cpu, reference));
                EXPECT_EQ(ctx.retired_count, expected.retired_count);
                EXPECT_EQ(ctx.exit_reason, expected.exit_reason);
                if (actual == (expect != 0U)) {
                    EXPECT_EQ(ctx.retired_count, 1U) << cond << '/' << flags << '/' << expect;
                    EXPECT_EQ(ctx.retired_cycles, 1U);
                    EXPECT_EQ(cpu.r[15], src.expected_next_pc);
                    EXPECT_EQ(ctx.exit_reason, NativeExitReason::none);
                } else {
                    EXPECT_EQ(ctx.retired_count, 0U) << cond << '/' << flags << '/' << expect;
                    EXPECT_EQ(ctx.retired_cycles, 0U);
                    EXPECT_EQ(cpu.r[15], pc);
                    EXPECT_EQ(ctx.exit_reason, NativeExitReason::branch_mismatch);
                }
            }
        }
    }
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
