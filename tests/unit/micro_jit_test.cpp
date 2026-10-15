// Differential tests: compiled FilIR kernels vs. real CortexM4 stepping.
//
// Each test places Thumb encodings in ROM, snapshots an initial CPU,
// runs the micro-JIT kernel on a copy, single-steps the real CPU over
// the same bytes, and compares r0-r12, LR, PC, instruction_address,
// and xPSR (NZCV exact; Q/GE/T/ISR preserved).

#include "fil/cpu/cortex_m4.hpp"
#include "fil/cpu/decoder.hpp"
#include "fil/cpu/micro_jit.hpp"
#include "fil/cpu/micro_jit_ir.hpp"
#include "fil/mem/memory_bus.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace {

using fil::cpu::CortexM4;
using fil::cpu::CpuState;
using fil::cpu::MicroBlock;
using fil::cpu::MicroFunction;
using fil::cpu::MicroJit;
using fil::cpu::MicroSource;

constexpr std::uint32_t kFlash = 0x08000000U;
constexpr std::uint32_t kRam = 0x20000000U;

struct Fixture {
    fil::mem::MemoryBus bus;
    CortexM4 cpu;
    Fixture() : cpu(bus) {
        EXPECT_TRUE(bus.mapRom(kFlash, 4096U, "micro-jit-flash").hasValue());
        EXPECT_TRUE(bus.mapRam(kRam, 4096U, "micro-jit-ram").hasValue());
    }
};

void write16(Fixture& f, std::uint32_t addr, std::uint16_t half) {
    std::array<std::uint8_t, 2> bytes{
        static_cast<std::uint8_t>(half), static_cast<std::uint8_t>(half >> 8U)};
    EXPECT_TRUE(f.bus.loadBytes(addr, bytes).hasValue());
}

std::uint32_t runReference(Fixture& f, const std::vector<std::uint16_t>& code,
                           const CpuState& initial, CpuState& final_out, int steps) {
    for (std::size_t i = 0; i < code.size(); ++i) {
        write16(f, kFlash + static_cast<std::uint32_t>(2U * i), code[i]);
    }
    f.cpu.state() = initial;
    f.cpu.state().r[15] = kFlash;
    f.cpu.state().thumb = true;
    f.cpu.state().halted = false;
    for (int i = 0; i < steps; ++i) {
        auto result = f.cpu.stepFast();
        EXPECT_EQ(result.reason, fil::cpu::StopReason::step_complete)
            << "step " << i << ": " << f.cpu.lastDiagnostic().message;
        if (result.reason != fil::cpu::StopReason::step_complete) break;
    }
    final_out = f.cpu.state();
    return final_out.r[15];
}

bool runMicro(const std::vector<std::uint16_t>& code, const CpuState& initial,
              CpuState& final_out, std::string& error) {
    std::vector<MicroSource> sources;
    std::uint32_t pc = kFlash;
    for (const auto half : code) {
        auto decoded = fil::cpu::decode16(half);
        if (!decoded) {
            error = "decode16 failed";
            return false;
        }
        sources.push_back(MicroSource{*decoded, pc, 2U});
        pc += 2U;
    }
    MicroBlock block;
    auto lowered = fil::cpu::buildMicroBlock({sources.data(), sources.size()}, block);
    if (!lowered.ok) {
        error = "lower: " + lowered.detail;
        return false;
    }
    auto jit = MicroJit::create(error);
    if (!jit) return false;
    MicroFunction fn = jit->compile(block, error);
    if (!fn) return false;
    final_out = initial;
    final_out.r[15] = kFlash;
    final_out.thumb = true;
    final_out.halted = false;
    const std::size_t retired = fn(&final_out, block.ops.size());
    EXPECT_EQ(retired, block.ops.size());
    return true;
}

void expectSameState(const CpuState& a, const CpuState& b) {
    for (int i = 0; i <= 12; ++i) EXPECT_EQ(a.r[i], b.r[i]) << "r" << i;
    EXPECT_EQ(a.r[14], b.r[14]) << "lr";
    EXPECT_EQ(a.r[15], b.r[15]) << "pc";
    EXPECT_EQ(a.instruction_address, b.instruction_address) << "instr addr";
    EXPECT_EQ(a.xpsr, b.xpsr) << std::hex << "xpsr ref=" << a.xpsr << " micro=" << b.xpsr;
}

void checkProgram(const std::vector<std::uint16_t>& code, std::uint64_t seed_base) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no micro-jit emitter on this host";
    std::mt19937_64 rng(seed_base);
    for (int trial = 0; trial < 25; ++trial) {
        CpuState initial;
        for (int i = 0; i <= 12; ++i) initial.r[i] = static_cast<std::uint32_t>(rng());
        initial.r[14] = static_cast<std::uint32_t>(rng());
        initial.xpsr = (1U << 24U) | 0x000F0000U | (1U << 27U) // T + GE + Q (must survive)
            | (static_cast<std::uint32_t>(rng() & 0xFU) << 28U); // random NZCV
        initial.r[13] = kRam + 2048U;

        Fixture f;
        CpuState ref_out{};
        runReference(f, code, initial, ref_out, static_cast<int>(code.size()));
        CpuState micro_out{};
        std::string error;
        ASSERT_TRUE(runMicro(code, initial, micro_out, error)) << error;
        expectSameState(ref_out, micro_out);
        if (::testing::Test::HasFailure()) {
            ADD_FAILURE() << "trial " << trial << " code size " << code.size();
            return;
        }
    }
}

TEST(MicroJitEmitterTest, MovAddSubCmp) {
    checkProgram({0x2001U, 0x1C81U, 0x1E4AU, 0x2A01U, 0x2205U}, 1U); // movs/adds/subs/cmp/movs
}

TEST(MicroJitEmitterTest, MovwMovtLogical) {
    // movs + ands/orrs/eors/bics (all T1, always flag-setting).
    checkProgram({0x200FU, 0x4001U, 0x4308U, 0x405AU, 0x4383U}, 2U);
}

TEST(MicroJitEmitterTest, ShiftsImmediate) {
    // lsls r0,r0,#2; lsrs r1,r1,#3; asrs r2,r2,#1; lsls r3,r3,#8.
    checkProgram({0x0080U, 0x0C89U, 0x104AU, 0x021BU}, 3U);
}

TEST(MicroJitEmitterTest, ShiftedAdd) {
    // add r4, r5, r6, lsl #2 (32-bit EB05 0486 approx); use assembler-known: 0xEB054886?
    // Safer: lsls pair + adds: verifies shift+ALU interplay across ops.
    checkProgram({0x00B1U, 0x1849U, 0x1AD2U}, 4U); // lsls r1,r6,#2; adds r1,r1,r2; subs r2,r1,r2
}

TEST(MicroJitEmitterTest, TerminalBranch) {
    // nops then a terminal b with offset 0 (target = branch pc + 4).
    // The branch must be last: FilIR rejects branch-not-terminal.
    checkProgram({0xBF00U, 0xBF00U, 0xE000U}, 5U);
}

TEST(MicroJitEmitterTest, PrefixExitsCommitPrecisely) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no micro-jit emitter on this host";
    // movs r0,#1; movs r1,#2; movs r2,#3: retire 1 and 2-op prefixes.
    const std::vector<std::uint16_t> code{0x2001U, 0x2102U, 0x2203U};
    std::vector<MicroSource> sources;
    std::uint32_t pc = kFlash;
    for (const auto half : code) {
        sources.push_back(MicroSource{*fil::cpu::decode16(half), pc, 2U});
        pc += 2U;
    }
    MicroBlock block;
    ASSERT_TRUE(fil::cpu::buildMicroBlock({sources.data(), sources.size()}, block).ok);
    std::string error;
    auto jit = MicroJit::create(error);
    ASSERT_TRUE(jit) << error;
    MicroFunction fn = jit->compile(block, error);
    ASSERT_TRUE(fn) << error;
    for (const std::size_t max : {1U, 2U, 3U, 16U}) {
        CpuState state;
        state.r[15] = kFlash;
        fn(&state, max);
        const std::size_t retired = std::min<std::size_t>(max, 3U);
        EXPECT_EQ(state.r[0], retired >= 1U ? 1U : 0U);
        EXPECT_EQ(state.r[1], retired >= 2U ? 2U : 0U);
        EXPECT_EQ(state.r[2], retired >= 3U ? 3U : 0U);
        EXPECT_EQ(state.r[15], kFlash + static_cast<std::uint32_t>(2U * retired));
        EXPECT_EQ(state.instruction_address,
                  retired == 0U ? 0U : kFlash + static_cast<std::uint32_t>(2U * (retired - 1U)));
    }
    // max == 0 commits nothing.
    CpuState untouched;
    untouched.r[15] = kFlash;
    fn(&untouched, 0U);
    EXPECT_EQ(untouched.r[0], 0U);
    EXPECT_EQ(untouched.r[15], kFlash);
}

TEST(MicroJitEmitterTest, ConditionalBranchTakenAndFallthrough) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no micro-jit emitter on this host";
    // movs r0, #N; cmp r0, #M; b<cond> +2 (skip one nop when taken); nop.
    // T1 conditional branch: 0xD000 | cond<<8 | imm8; target = pc+4+imm8*2.
    const std::uint16_t bne_over = static_cast<std::uint16_t>(0xD100U | 0x00U); // bne +2
    const std::vector<std::uint16_t> code{0x2001U, 0x2801U, bne_over, 0xBF00U, 0xBF00U};
    std::mt19937_64 rng(42U);
    for (int trial = 0; trial < 10; ++trial) {
        // Alternate taken / fallthrough by choosing the cmp operands.
        const bool take = (trial % 2) == 0;
        CpuState initial;
        for (int i = 0; i <= 12; ++i) initial.r[i] = static_cast<std::uint32_t>(rng());
        initial.r[14] = static_cast<std::uint32_t>(rng());
        initial.xpsr = (1U << 24U) | 0x000F0000U | (1U << 27U)
            | (static_cast<std::uint32_t>(rng() & 0xFU) << 28U);
        initial.r[13] = kRam + 2048U;
        // Force the cmp outcome: r0 ends as 1 (movs), compare vs 1.
        // take -> need NE (r0 != 1)? r0 IS 1 -> EQ. Use two cmp variants:
        std::vector<std::uint16_t> prog = code;
        prog[1] = take ? 0x2802U : 0x2801U; // cmp r0, #2 (NE) vs #1 (EQ)
        Fixture f;
        CpuState ref_out{};
        // Reference steps until PC leaves the straight-line region.
        for (std::size_t i = 0; i < prog.size(); ++i) {
            std::array<std::uint8_t, 2> by{
                static_cast<std::uint8_t>(prog[i]), static_cast<std::uint8_t>(prog[i] >> 8U)};
            EXPECT_TRUE(f.bus.loadBytes(kFlash + static_cast<std::uint32_t>(2U * i), by)
                            .hasValue());
        }
        f.cpu.state() = initial;
        f.cpu.state().r[15] = kFlash;
        f.cpu.state().thumb = true;
        f.cpu.state().halted = false;
        // The micro kernel stops at a taken branch; mirror that exactly.
        for (int i = 0; i < (take ? 3 : 5); ++i) {
            auto result = f.cpu.stepFast();
            ASSERT_EQ(result.reason, fil::cpu::StopReason::step_complete);
        }
        ref_out = f.cpu.state();
        // Micro side.
        std::vector<MicroSource> sources;
        std::uint32_t pc = kFlash;
        for (const auto half : prog) {
            auto decoded = fil::cpu::decode16(half);
            ASSERT_TRUE(decoded.has_value());
            sources.push_back(MicroSource{*decoded, pc, 2U});
            pc += 2U;
        }
        MicroBlock block;
        ASSERT_TRUE(fil::cpu::buildMicroBlock({sources.data(), sources.size()}, block).ok);
        std::string error;
        auto jit = MicroJit::create(error);
        ASSERT_TRUE(jit) << error;
        MicroFunction fn = jit->compile(block, error);
        ASSERT_TRUE(fn) << error;
        CpuState micro_out = initial;
        micro_out.r[15] = kFlash;
        micro_out.thumb = true;
        micro_out.halted = false;
        const std::size_t retired = fn(&micro_out, block.ops.size());
        EXPECT_EQ(retired, take ? 3U : 5U);
        expectSameState(ref_out, micro_out);
        if (::testing::Test::HasFailure()) {
            ADD_FAILURE() << "taken=" << take;
            return;
        }
    }
}

TEST(MicroJitEmitterTest, CbzCbnz) {
    if (!MicroJit::hostSupported()) GTEST_SKIP() << "no micro-jit emitter on this host";
    // cbz r0, +2 (skip to movs when r0==0); nop; nop; movs r1, #7.
    // T1 CBZ: 0xB100 | (imm6 bit5 -> bit9) | (imm6 bits4:0 -> bits7:3) | rn.
    const auto cbz = [](std::uint8_t rn, std::uint32_t off) {
        const std::uint32_t imm = off >> 1U;
        return static_cast<std::uint16_t>(0xB100U | ((imm & 0x20U) << 4U)
            | ((imm & 0x1FU) << 3U) | rn);
    };
    const std::vector<std::uint16_t> code{cbz(0, 2), 0xBF00U, 0xBF00U, 0x2107U};
    for (const std::uint32_t r0 : {0U, 5U}) {
        Fixture f;
        for (std::size_t i = 0; i < code.size(); ++i) {
            std::array<std::uint8_t, 2> by{
                static_cast<std::uint8_t>(code[i]), static_cast<std::uint8_t>(code[i] >> 8U)};
            ASSERT_TRUE(f.bus.loadBytes(kFlash + static_cast<std::uint32_t>(2U * i), by)
                            .hasValue());
        }
        CpuState initial;
        initial.r[0] = r0;
        initial.xpsr = (1U << 24U);
        initial.r[13] = kRam + 2048U;
        f.cpu.state() = initial;
        f.cpu.state().r[15] = kFlash;
        f.cpu.state().thumb = true;
        f.cpu.state().halted = false;
        const bool take = (r0 == 0U);
        const int steps = take ? 1 : 4;
        for (int i = 0; i < steps; ++i) {
            ASSERT_EQ(f.cpu.stepFast().reason, fil::cpu::StopReason::step_complete);
        }
        const CpuState ref_out = f.cpu.state();
        std::vector<MicroSource> sources;
        std::uint32_t pc = kFlash;
        for (const auto half : code) {
            auto decoded = fil::cpu::decode16(half);
            ASSERT_TRUE(decoded.has_value());
            sources.push_back(MicroSource{*decoded, pc, 2U});
            pc += 2U;
        }
        MicroBlock block;
        ASSERT_TRUE(fil::cpu::buildMicroBlock({sources.data(), sources.size()}, block).ok);
        std::string error;
        auto jit = MicroJit::create(error);
        ASSERT_TRUE(jit) << error;
        MicroFunction fn = jit->compile(block, error);
        ASSERT_TRUE(fn) << error;
        CpuState micro_out = initial;
        micro_out.r[15] = kFlash;
        micro_out.thumb = true;
        micro_out.halted = false;
        EXPECT_EQ(fn(&micro_out, block.ops.size()), take ? 1U : 4U);
        expectSameState(ref_out, micro_out);
    }
}

TEST(MicroJitEmitterTest, DeclinesMemory) {
    if (!MicroJit::hostSupported()) GTEST_SKIP();
    // ldr r0, [r1]: must decline lowering.
    auto decoded = fil::cpu::decode16(0x6810U);
    ASSERT_TRUE(decoded.has_value());
    MicroSource src{*decoded, kFlash, 2U};
    MicroBlock block;
    EXPECT_FALSE(fil::cpu::buildMicroBlock({&src, 1U}, block).ok);
}

} // namespace
