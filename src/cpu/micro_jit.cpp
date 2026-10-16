// Bespoke FilIR stencil emitter.
//
// AArch64 only; other hosts compile to a stub whose hostSupported() is
// false so every site keeps its cached-handler behavior with no CLI or
// build churn. Guest regs map to host regs for the whole block:
//
//   x0         CpuState* (preserved)
//   w2..w14    guest r0..r12
//   w15        guest LR (r14)
//   w16        retained xPSR bits (~N/Z/C/V: Q, GE, IT, ISR, T)
//   w17/x17    scratch (x18 avoided: reserved on Darwin)
//   w1         second scratch (amount materialization)
//
// Guest N/Z/C/V ride the host flags between ops. Prologue seeds them
// with `msr nzcv, x16` from the loaded xPSR; every flag-setting stencil
// maintains exact guest flag semantics (verified by differential tests),
// and the epilogue merges host N/Z/C/V back over the retained bits.
//
// CpuState field offsets come from offsetof: nothing is hardcoded.

#include "fil/cpu/micro_jit.hpp"

#include "fil/cpu/cortex_m4.hpp"
#include "fil/cpu/micro_jit_ir.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <mutex>
#include <vector>

#if defined(FIL_MICRO_JIT_DUMP)
#include <iomanip>
#include <iostream>
#endif

#if defined(__aarch64__) || defined(_M_ARM64)
#define FIL_MICRO_JIT_AARCH64 1
#include <sys/mman.h>
#elif defined(__x86_64__) || defined(_M_X64)
#define FIL_MICRO_JIT_X86_64 1
#include <sys/mman.h>
#endif

namespace fil::cpu {
namespace {

// Guest r[i] -> host w(i+2); guest LR -> w15.
constexpr std::uint32_t hostReg(std::uint8_t guest) noexcept {
    return guest == 14U ? 15U : static_cast<std::uint32_t>(2U + guest);
}

bool sameMicroInst(const MicroInst& a, const MicroInst& b) noexcept {
    return a.op == b.op && a.rd == b.rd && a.rn == b.rn && a.rm == b.rm
        && a.imm == b.imm && a.condition == b.condition && a.shift == b.shift
        && a.shift_amount == b.shift_amount && a.set_flags == b.set_flags
        && a.carry_valid == b.carry_valid && a.carry == b.carry
        && a.shift_immediate == b.shift_immediate && a.branch_offset == b.branch_offset
        && a.pc == b.pc && a.size == b.size;
}

bool sameMicroBlock(const MicroBlock& a, const MicroBlock& b) noexcept {
    if (a.entry_pc != b.entry_pc || a.ends_with_branch != b.ends_with_branch
        || a.ops.size() != b.ops.size()) return false;
    for (std::size_t i = 0U; i < a.ops.size(); ++i) {
        if (!sameMicroInst(a.ops[i], b.ops[i])) return false;
    }
    return true;
}

} // namespace

#if defined(FIL_MICRO_JIT_AARCH64) || defined(FIL_MICRO_JIT_X86_64)
#define FIL_MICRO_JIT_NATIVE 1
#endif

#ifdef FIL_MICRO_JIT_AARCH64

class Emitter {
public:
    std::vector<std::uint32_t> words;

    void emit(std::uint32_t w) { words.push_back(w); }

    // --- data processing -------------------------------------------------
    void movz(std::uint32_t rd, std::uint32_t imm16, std::uint32_t hw) {
        emit(0x52800000U | ((hw & 3U) << 21U) | ((imm16 & 0xffffU) << 5U) | (rd & 31U));
    }
    void movk(std::uint32_t rd, std::uint32_t imm16, std::uint32_t hw) {
        emit(0x72800000U | ((hw & 3U) << 21U) | ((imm16 & 0xffffU) << 5U) | (rd & 31U));
    }
    void movImm(std::uint32_t rd, std::uint32_t value) {
        const std::uint32_t lo = value & 0xffffU;
        const std::uint32_t hi = (value >> 16U) & 0xffffU;
        if (hi == 0U) {
            movz(rd, lo, 0U);
        } else if (lo == 0U) {
            movz(rd, hi, 1U);
        } else {
            movz(rd, lo, 0U);
            movk(rd, hi, 1U);
        }
    }
    // add/sub (op: 0=add,1=adds,2=sub,3=subs), rn, rm-or-imm handled by caller.
    void addSubImm(std::uint32_t op, std::uint32_t rd, std::uint32_t rn, std::uint32_t imm) {
        static constexpr std::uint32_t kBase[4] = {0x11000000U, 0x31000000U, 0x51000000U, 0x71000000U};
        if (imm < 4096U) {
            emit(kBase[op] | ((imm & 0xfffU) << 10U) | ((rn & 31U) << 5U) | (rd & 31U));
        } else if ((imm & 0xfffU) == 0U && (imm >> 12U) < 4096U) {
            emit(kBase[op] | (1U << 22U) | (((imm >> 12U) & 0xfffU) << 10U) | ((rn & 31U) << 5U) | (rd & 31U));
        } else {
            movImm(17U, imm);
            addSubReg(op, rd, rn, 17U);
        }
    }
    void addSubReg(std::uint32_t op, std::uint32_t rd, std::uint32_t rn, std::uint32_t rm) {
        static constexpr std::uint32_t kBase[4] = {0x0B000000U, 0x2B000000U, 0x4B000000U, 0x6B000000U};
        emit(kBase[op] | ((rm & 31U) << 16U) | ((rn & 31U) << 5U) | (rd & 31U));
    }
    void logicReg(std::uint32_t op, std::uint32_t rd, std::uint32_t rn, std::uint32_t rm) {
        // op: 0=and, 1=ands(tst), 2=orr(mov), 3=eor, 4=bic.
        static constexpr std::uint32_t kBase[5] = {
            0x0A000000U, 0x6A000000U, 0x2A000000U, 0x4A000000U, 0x0A200000U};
        emit(kBase[op] | ((rm & 31U) << 16U) | ((rn & 31U) << 5U) | (rd & 31U));
    }
    void andImm(std::uint32_t rd, std::uint32_t rn, std::uint32_t immr, std::uint32_t imms) {
        emit(0x12000000U | ((immr & 63U) << 16U) | ((imms & 63U) << 10U) | ((rn & 31U) << 5U) | (rd & 31U));
    }
    // Logical-shift pair: clears the top nibble without a logical-
    // immediate mask (immune to invert/rotate pitfalls of BIC #imm).
    void clearTopNibble(std::uint32_t rd) {
        ubfm(rd, rd, 28U, 27U); // lsl #4
        ubfm(rd, rd, 4U, 31U); // lsr #4
    }
    // reg-shifted-vector: kind 0=lslv, 1=lsrv, 2=asrv, 3=rorv.
    void shiftV(std::uint32_t kind, std::uint32_t rd, std::uint32_t rn, std::uint32_t rm) {
        static constexpr std::uint32_t kBase[4] = {0x1AC02000U, 0x1AC02400U, 0x1AC02800U, 0x1AC02C00U};
        emit(kBase[kind] | ((rm & 31U) << 16U) | ((rn & 31U) << 5U) | (rd & 31U));
    }
    void ubfm(std::uint32_t rd, std::uint32_t rn, std::uint32_t immr, std::uint32_t imms) {
        emit(0x53000000U | ((immr & 31U) << 16U) | ((imms & 31U) << 10U) | ((rn & 31U) << 5U) | (rd & 31U));
    }
    void ubfm64(std::uint32_t rd, std::uint32_t rn, std::uint32_t immr, std::uint32_t imms) {
        emit(0xD3400000U | ((immr & 63U) << 16U) | ((imms & 63U) << 10U) | ((rn & 31U) << 5U) | (rd & 31U));
    }
    void sbfmAsr(std::uint32_t rd, std::uint32_t rn, std::uint32_t amount) {
        emit(0x13000000U | ((amount & 31U) << 16U) | (31U << 10U) | ((rn & 31U) << 5U) | (rd & 31U));
    }
    void bfi(std::uint32_t rd, std::uint32_t rn, std::uint32_t lsb, std::uint32_t width) {
        emit(0x33000000U | ((((32U - lsb) & 31U) & 63U) << 16U) | (((width - 1U) & 63U) << 10U)
            | ((rn & 31U) << 5U) | (rd & 31U));
    }
    void bfi64(std::uint32_t rd, std::uint32_t rn, std::uint32_t lsb, std::uint32_t width) {
        emit(0xB3400000U | ((((64U - lsb) & 63U) & 63U) << 16U) | (((width - 1U) & 63U) << 10U)
            | ((rn & 31U) << 5U) | (rd & 31U));
    }
    void lsr64(std::uint32_t rd, std::uint32_t rn, std::uint32_t amount) {
        emit(0xD3400000U | ((amount & 63U) << 16U) | (63U << 10U) | ((rn & 31U) << 5U) | (rd & 31U));
    }
    void mrsNzcv(std::uint32_t rt) { emit(0xD53B4200U | (rt & 31U)); }
    void msrNzcv(std::uint32_t rt) { emit(0xD51B4200U | (rt & 31U)); }
    void cmpImm(std::uint32_t rn, std::uint32_t imm) {
        emit(0x71000000U | ((imm & 0xfffU) << 10U) | ((rn & 31U) << 5U) | 31U);
    }
    void cmp64(std::uint32_t rn, std::uint32_t rm) {
        emit(0xEB00001FU | ((rm & 31U) << 16U) | ((rn & 31U) << 5U)); // subs xzr, rn, rm
    }
    // The native block has at most 16 ops. Clamp the size_t scheduler limit
    // to that bound before placing it in the 16-bit x16[47:32] field.
    // Failed-branch flags occupy x16[63:48], so the fields never overlap.
    void clampMax(std::uint32_t block_max) {
        movImm(17U, block_max);
        cmp64(1U, 17U);
        csel(1U, 1U, 17U, 3U); // LO: keep requested max, otherwise block_max
    }
    void stashMax() {
        bfi64(16U, 1U, 32U, 16U);
    }
    // Compare stashed max against `count` (clobbers x17, preserves flags).
    // Exit iff max < count. Mask to the dedicated 16-bit limit field so the
    // failed-branch mask in x16[63:48] cannot affect the scheduler.
    void checkMax(std::uint32_t count, std::size_t exit_label) {
        ubfm64(17U, 16U, 32U, 47U); // ubfx x17, x16, #32, #16
        sub64Imm(17U, 17U, count);
        lsr64(17U, 17U, 63U); // wrap bit -> bit0
        cbnzX17(exit_label);
    }
    void csel(std::uint32_t rd, std::uint32_t rn, std::uint32_t rm, std::uint32_t cond) {
        emit(0x1A800000U | ((rm & 31U) << 16U) | ((cond & 15U) << 12U) | ((rn & 31U) << 5U) | (rd & 31U));
    }
    // Branches (targets patched by patchBranches): conditional LS, CBZ, B.
    struct BranchFixup {
        std::size_t word{0};
        std::size_t target{0};
        std::uint8_t kind{0}; // 0=unused, 1=cbz/cbnz (imm19), 2=b (imm26)
    };
    std::vector<BranchFixup> fixups;
    // Returns a label id bound to the NEXT emitted word.
    std::vector<std::size_t> labels;
    std::size_t makeLabel() {
        labels.push_back(words.size());
        return labels.size() - 1U;
    }
    void bindLabel(std::size_t id) { labels[id] = words.size(); }
    void cbzX1(std::size_t label) {
        fixups.push_back({words.size(), label, 1U});
        emit(0xB4000000U | 1U); // cbz x1, <patched>
    }
    void cbnzX17(std::size_t label) {
        fixups.push_back({words.size(), label, 1U});
        emit(0xB5000000U | 17U); // cbnz x17, <patched>
    }
    void sub64Imm(std::uint32_t rd, std::uint32_t rn, std::uint32_t imm) {
        emit(0xD1000000U | ((imm & 0xfffU) << 10U) | ((rn & 31U) << 5U) | (rd & 31U));
    }
    void cbzW(std::uint32_t rt, std::size_t label) {
        fixups.push_back({words.size(), label, 1U});
        emit(0x34000000U | (rt & 31U)); // cbz Wt, <patched>
    }
    void cbnzW(std::uint32_t rt, std::size_t label) {
        fixups.push_back({words.size(), label, 1U});
        emit(0x35000000U | (rt & 31U)); // cbnz Wt, <patched>
    }
    void movRet(std::uint32_t count) {
        emit(0xD2800000U | ((count & 0xffffU) << 5U)); // movz x0, #count
    }
    // Combines the retired count with the failed-branch mask (x16[63:48])
    // into the x0 return value. x1 holds the mask after the epilogue below.
    void movRetMasked(std::uint32_t count) {
        movRet(count);
        bfi64(0U, 1U, 32U, 16U); // x0[47:32] = failed mask
    }
    // Marks op i as a failed conditional branch in x16[63:48].
    // ORR-imm with a single high bit (N=1 size-64 pattern, R=(64-p)%64).
    void markFailed(std::uint32_t op_index) {
        const std::uint32_t bit = 48U + op_index;
        const std::uint32_t r = (64U - bit) & 63U;
        emit(0xB2400000U | ((r & 63U) << 16U) | (0U << 10U) | (16U << 5U) | 16U);
    }
    // Evaluates a guest condition from the host flags without disturbing
    // them (mrs/ubfm/eor/cbz only) and jumps to `taken` when it passes.
    // Returns false for NV (rejected at lowering; defensive). Uses w1/x17.
    // The fallthrough (failed) path records op_index in the x16 mask.
    bool emitCondTaken(Condition cond, std::uint32_t op_index,
                       std::size_t taken, std::string& error) {
        mrsNzcv(17U);
        const auto flag = [&](std::uint32_t bit) { ubfm(17U, 17U, bit, bit); };
        switch (cond) {
        case Condition::eq: flag(30U); cbnzW(17U, taken); markFailed(op_index); return true;
        case Condition::ne: flag(30U); cbzW(17U, taken); markFailed(op_index); return true;
        case Condition::cs: flag(29U); cbnzW(17U, taken); markFailed(op_index); return true;
        case Condition::cc: flag(29U); cbzW(17U, taken); markFailed(op_index); return true;
        case Condition::mi: flag(31U); cbnzW(17U, taken); markFailed(op_index); return true;
        case Condition::pl: flag(31U); cbzW(17U, taken); markFailed(op_index); return true;
        case Condition::vs: flag(28U); cbnzW(17U, taken); markFailed(op_index); return true;
        case Condition::vc: flag(28U); cbzW(17U, taken); markFailed(op_index); return true;
        case Condition::hi: { // C && !Z
            flag(29U);
            const auto done = makeLabel();
            cbzW(17U, done);
            mrsNzcv(17U);
            flag(30U);
            cbzW(17U, taken);
            bindLabel(done);
            markFailed(op_index);
            return true;
        }
        case Condition::ls: // !C || Z
            flag(29U);
            cbzW(17U, taken);
            mrsNzcv(17U);
            flag(30U);
            cbnzW(17U, taken);
            markFailed(op_index);
            return true;
        case Condition::ge: // N == V
        case Condition::lt: { // N != V
            ubfm(1U, 17U, 31U, 31U); // N -> w1
            flag(28U); // V -> w17
            logicReg(3U, 1U, 1U, 17U); // eor w1, w1, w17 (no flags)
            if (cond == Condition::ge) cbzW(1U, taken);
            else cbnzW(1U, taken);
            markFailed(op_index);
            return true;
        }
        case Condition::gt: { // !Z && N == V
            flag(30U);
            const auto done = makeLabel();
            cbnzW(17U, done);
            mrsNzcv(17U);
            ubfm(1U, 17U, 31U, 31U);
            flag(28U);
            logicReg(3U, 1U, 1U, 17U);
            cbzW(1U, taken);
            bindLabel(done);
            markFailed(op_index);
            return true;
        }
        case Condition::le: // Z || N != V
            flag(30U);
            cbnzW(17U, taken);
            mrsNzcv(17U);
            ubfm(1U, 17U, 31U, 31U);
            flag(28U);
            logicReg(3U, 1U, 1U, 17U);
            cbnzW(1U, taken);
            markFailed(op_index);
            return true;
        case Condition::al: b(taken); return true;
        case Condition::nv:
            error = "micro-jit: NV condition";
            return false;
        }
        error = "micro-jit: unknown condition";
        return false;
    }
    void b(std::size_t label) {
        fixups.push_back({words.size(), label, 2U});
        emit(0x14000000U); // b <patched>
    }
    bool patchBranches(std::string& error) {
        // fix.word/target are word indices; cond-branch immediates count
        // words (the x4 scaling is architectural), so deltas here are
        // already word-scaled and always aligned by construction.
        for (const auto& fix : fixups) {
            const std::int64_t delta = static_cast<std::int64_t>(labels[fix.target])
                - static_cast<std::int64_t>(fix.word);
            if (fix.kind == 2U) {
                if (delta < -(1LL << 25) || delta > ((1LL << 25) - 1LL)) {
                    error = "micro-jit: branch out of range";
                    return false;
                }
                words[fix.word] |= static_cast<std::uint32_t>(delta & 0x03ffffffU);
            } else {
                if (delta < -(1LL << 18) || delta > ((1LL << 18) - 1LL)) {
                    error = "micro-jit: branch out of range";
                    return false;
                }
                words[fix.word] |= static_cast<std::uint32_t>(
                    (static_cast<std::uint32_t>(delta) & 0x7ffffU) << 5U);
            }
        }
        return true;
    }
    // --- loads/stores -----------------------------------------------------
    void ldrW(std::uint32_t rt, std::size_t byte_off) {
        emit(0xB9400000U | (static_cast<std::uint32_t>(byte_off / 4U) << 10U) | (rt & 31U));
    }
    void strW(std::uint32_t rt, std::size_t byte_off) {
        emit(0xB9000000U | (static_cast<std::uint32_t>(byte_off / 4U) << 10U) | (rt & 31U));
    }
    void ldpW(std::uint32_t rt1, std::uint32_t rt2, std::size_t byte_off) {
        emit(0x29400000U | (static_cast<std::uint32_t>(byte_off / 4U) << 15U) | ((rt2 & 31U) << 10U) | ((rt1 & 31U)));
    }
    void stpW(std::uint32_t rt1, std::uint32_t rt2, std::size_t byte_off) {
        emit(0x29000000U | (static_cast<std::uint32_t>(byte_off / 4U) << 15U) | ((rt2 & 31U) << 10U) | ((rt1 & 31U)));
    }
    void ret() { emit(0xD65F03C0U); }

    // Sets host N/Z from wD, restoring entry C/V afterwards.
    // (On this host ANDS clears C/V rather than preserving them, so
    // every N/Z-only update must explicitly save and restore C/V.)
    void setNzKeepCv(std::uint32_t wd) {
        mrsNzcv(17U);
        logicReg(1U, 31U, wd, wd); // tst wD, wD: N/Z new, C/V clobbered
        mrsNzcv(1U);
        lsr64(17U, 17U, 28U); // x17[1:0] = saved V, saved C
        bfi(1U, 17U, 28U, 2U);
        msrNzcv(1U);
    }
    // Bridge: host C = bit0 of wC, N/Z from wD, V restored.
    void setNzCarry(std::uint32_t wd, std::uint32_t wc) {
        mrsNzcv(17U);
        bfi64(17U, wc, 29U, 1U); // C new, V saved, N/Z stale
        msrNzcv(17U);
        logicReg(1U, 31U, wd, wd); // N/Z new, C/V clobbered
        mrsNzcv(1U);
        lsr64(17U, 17U, 28U); // x17[1:0] = saved V, new C
        bfi(1U, 17U, 28U, 2U);
        msrNzcv(1U);
    }
    // Flag-setting AND: ANDS sets N/Z, then restore C/V.
    void andsKeepCv(std::uint32_t rd, std::uint32_t rn, std::uint32_t rm) {
        mrsNzcv(17U);
        emit(0x6A000000U | ((rm & 31U) << 16U) | ((rn & 31U) << 5U) | (rd & 31U));
        mrsNzcv(1U);
        lsr64(17U, 17U, 28U);
        bfi(1U, 17U, 28U, 2U);
        msrNzcv(1U);
    }
};

// Shift value into wD (carry discarded). Returns false for forms needing a
// runtime amount the caller must handle separately.
bool emitShiftValue(Emitter& e, std::uint32_t wd, std::uint32_t wn,
                    ShiftType type, std::uint32_t amount) {
    switch (type) {
    case ShiftType::lsl:
        if (amount == 0U) {
            if (wd != wn) e.logicReg(2U, wd, 31U, wn); // mov
            return true;
        }
        if (amount >= 32U) return false;
        e.movImm(1U, amount);
        e.shiftV(0U, wd, wn, 1U);
        return true;
    case ShiftType::lsr:
        if (amount == 0U || amount > 32U) return false;
        if (amount == 32U) {
            e.logicReg(2U, wd, 31U, 31U); // mov wzr
            return true;
        }
        e.movImm(1U, amount);
        e.shiftV(1U, wd, wn, 1U);
        return true;
    case ShiftType::asr:
        if (amount == 0U || amount > 32U) return false;
        if (amount == 32U) {
            e.sbfmAsr(wd, wn, 31U);
            return true;
        }
        e.movImm(1U, amount);
        e.shiftV(2U, wd, wn, 1U);
        return true;
    case ShiftType::ror: {
        const std::uint32_t rotation = amount & 31U;
        if (rotation == 0U) {
            if (wd != wn) e.logicReg(2U, wd, 31U, wn);
            return true;
        }
        e.movImm(1U, rotation);
        e.shiftV(3U, wd, wn, 1U);
        return true;
    }
    case ShiftType::rrx:
        // (oldC << 31) | (value >> 1); old C is host C (== guest C).
        e.mrsNzcv(17U);
        e.ubfm(wd, wn, 1U, 31U); // lsr #1
        e.lsr64(17U, 17U, 29U);
        e.bfi(wd, 17U, 31U, 1U);
        return true;
    }
    return false;
}

bool emitOp(Emitter& e, const MicroInst& op, std::string& error) {
    const std::uint32_t wd = hostReg(op.rd);
    const std::uint32_t wn = hostReg(op.rn);
    const std::uint32_t wm = hostReg(op.rm);
    switch (op.op) {
    case MicroOp::nop: return true;
    case MicroOp::mov_imm:
        e.movImm(wd, op.imm);
        if (op.set_flags) {
            if (op.carry_valid) {
                e.movImm(1U, op.carry ? 1U : 0U);
                e.setNzCarry(wd, 1U);
            } else {
                e.setNzKeepCv(wd);
            }
        }
        return true;
    case MicroOp::movw:
        e.movz(wd, op.imm & 0xffffU, 0U);
        return true;
    case MicroOp::movt:
        e.movImm(1U, (op.imm & 0xffffU) << 16U);
        e.bfi(wd, 1U, 0U, 16U);
        return true;
    case MicroOp::add_imm:
        e.addSubImm(op.set_flags ? 1U : 0U, wd, wn, op.imm);
        return true;
    case MicroOp::sub_imm:
        e.addSubImm(op.set_flags ? 3U : 2U, wd, wn, op.imm);
        return true;
    case MicroOp::cmp_imm:
        e.addSubImm(3U, 31U, wn, op.imm);
        return true;
    case MicroOp::cmn_imm:
        e.addSubImm(1U, 31U, wn, op.imm);
        return true;
    case MicroOp::add_reg:
        e.addSubReg(op.set_flags ? 1U : 0U, wd, wn, wm);
        return true;
    case MicroOp::sub_reg:
        e.addSubReg(op.set_flags ? 3U : 2U, wd, wn, wm);
        return true;
    case MicroOp::and_reg:
        if (op.set_flags) e.andsKeepCv(wd, wn, wm);
        else e.logicReg(0U, wd, wn, wm);
        return true;
    case MicroOp::orr_reg:
        e.logicReg(2U, wd, wn, wm);
        if (op.set_flags) e.setNzKeepCv(wd);
        return true;
    case MicroOp::eor_reg:
        e.logicReg(3U, wd, wn, wm);
        if (op.set_flags) e.setNzKeepCv(wd);
        return true;
    case MicroOp::bic_reg:
        // BIC reg has no S-variant: plain op, then N/Z with C/V kept.
        e.logicReg(4U, wd, wn, wm);
        if (op.set_flags) e.setNzKeepCv(wd);
        return true;
    case MicroOp::add_shift:
    case MicroOp::sub_shift: {
        if (!emitShiftValue(e, 17U, wm, op.shift, op.shift_amount)) {
            error = "unsupported alu shift amount";
            return false;
        }
        e.addSubReg(op.op == MicroOp::add_shift ? (op.set_flags ? 1U : 0U) : (op.set_flags ? 3U : 2U),
                    wd, wn, 17U);
        return true;
    }
    case MicroOp::lsl:
    case MicroOp::lsr:
    case MicroOp::asr:
    case MicroOp::ror:
    case MicroOp::rrx: {
        ShiftType type = op.shift;
        if (op.op == MicroOp::lsl) type = ShiftType::lsl;
        else if (op.op == MicroOp::lsr) type = ShiftType::lsr;
        else if (op.op == MicroOp::asr) type = ShiftType::asr;
        else if (op.op == MicroOp::ror) type = ShiftType::ror;
        else type = ShiftType::rrx;
        if (!op.shift_immediate) {
            // Register amount, no flags (lowering declines flags here).
            // amount = rm & 0xff; fix up results where *v differs from shiftC.
            e.andImm(1U, wm, 0U, 7U); // w1 = amount
            if (type == ShiftType::ror) {
                e.shiftV(3U, wd, wn, 1U); // rorv masks to 5 bits: exact.
                return true;
            }
            if (type == ShiftType::rrx) {
                error = "register rrx unsupported";
                return false;
            }
            const std::uint32_t kind = type == ShiftType::lsl ? 0U : type == ShiftType::lsr ? 1U : 2U;
            if (type == ShiftType::asr) {
                // amount >= 32 -> sign fill; else asrv.
                e.sbfmAsr(17U, wn, 31U); // fill = 0 / 0xffffffff
                e.shiftV(kind, wd, wn, 1U);
                e.cmpImm(1U, 32U);
                e.csel(wd, 17U, wd, 2U); // hs: amount >= 32 -> fill
                return true;
            }
            e.shiftV(kind, wd, wn, 1U);
            e.cmpImm(1U, 32U);
            e.csel(wd, 31U, wd, 2U); // hs: amount >= 32 -> 0
            return true;
        }
        const std::uint32_t amount = type == ShiftType::rrx ? 1U : op.shift_amount;
        // Immediate form reads the shifted value from rm (executor parity).
        const std::uint32_t wv = wm;
        // Shift results land in w17 first: extracting carry from the source
        // afterwards stays exact when rd == rm (self-shifts).
        // Returns the register holding the result (17 or wd for direct forms).
        std::uint32_t carry_lsb = 0U; // bit position in source holding carry
        bool carry_is_result_msb = false;
        bool result_in_temp = false;
        switch (type) {
        case ShiftType::lsl:
            if (amount == 0U) {
                if (wd != wv) e.logicReg(2U, wd, 31U, wv);
                if (op.set_flags) e.setNzKeepCv(wd);
                return true;
            }
            if (amount >= 32U) {
                error = "lsl amount >= 32";
                return false;
            }
            e.movImm(1U, amount);
            e.shiftV(0U, 17U, wv, 1U);
            result_in_temp = true;
            carry_lsb = 32U - amount;
            break;
        case ShiftType::lsr:
            if (amount == 0U || amount > 32U) {
                error = "lsr amount out of range";
                return false;
            }
            if (amount == 32U) {
                e.ubfm(1U, wv, 31U, 31U); // carry first: zeroing wd may clobber wv
                e.logicReg(2U, wd, 31U, 31U);
                carry_lsb = 32U; // sentinel: carry already in w1
            } else {
                e.movImm(1U, amount);
                e.shiftV(1U, 17U, wv, 1U);
                result_in_temp = true;
            }
            carry_lsb = amount - 1U;
            break;
        case ShiftType::asr:
            if (amount == 0U || amount > 32U) {
                error = "asr amount out of range";
                return false;
            }
            if (amount == 32U) {
                e.ubfm(1U, wv, 31U, 31U); // carry first: fill may clobber wv
                e.sbfmAsr(wd, wv, 31U);
                carry_lsb = 32U; // sentinel: carry already in w1
            } else {
                e.movImm(1U, amount);
                e.shiftV(2U, 17U, wv, 1U);
                result_in_temp = true;
            }
            carry_lsb = amount - 1U;
            break;
        case ShiftType::ror: {
            const std::uint32_t rotation = amount & 31U;
            if (rotation == 0U) {
                if (wd != wv) e.logicReg(2U, wd, 31U, wv);
                carry_lsb = 31U;
            } else {
                e.movImm(1U, rotation);
                e.shiftV(3U, 17U, wv, 1U);
                result_in_temp = true;
                carry_is_result_msb = true;
            }
            break;
        }
        case ShiftType::rrx:
            e.andImm(1U, wv, 0U, 0U); // carry = value & 1 (before shift)
            if (!emitShiftValue(e, wd, wv, ShiftType::rrx, 1U)) {
                error = "rrx emission failed";
                return false;
            }
            if (op.set_flags) e.setNzCarry(wd, 1U);
            return true;
        }
        // Commit a temp result before any flag work (frees w17 for bridges).
        auto commitTemp = [&]() {
            if (result_in_temp && wd != 17U) e.logicReg(2U, wd, 31U, 17U);
        };
        if (!op.set_flags) {
            commitTemp();
            return true;
        }
        if (carry_is_result_msb) {
            commitTemp();
            e.ubfm(1U, wd, 31U, 31U);
            e.setNzCarry(wd, 1U);
        } else if (carry_lsb == 32U) {
            // Carry was extracted before a clobbering result write.
            commitTemp();
            e.setNzCarry(wd, 1U);
        } else {
            // carry = source bit carry_lsb (source still intact: the result
            // is in w17, or this form wrote wd without reading it back).
            e.ubfm(1U, wv, carry_lsb, carry_lsb);
            commitTemp();
            e.setNzCarry(wd, 1U);
        }
        return true;
    }
    case MicroOp::b:
    case MicroOp::bl:
        return true; // terminal PC/LR effects are compile-time constants
    case MicroOp::b_cond:
    case MicroOp::cbz:
    case MicroOp::cbnz:
        // Control transfer is emitted by compile() (needs taken stubs).
        error = "micro-jit: conditional branch in emitOp";
        return false;
    }
    error = "unhandled micro op";
    return false;
}

#endif // FIL_MICRO_JIT_AARCH64

#ifdef FIL_MICRO_JIT_X86_64

// x86_64 stencil emitter.
//
// The SysV entry is fn(CpuState* rdi, size_t rsi); the outcome packs the
// retired count (low 32) with the failed-conditional-branch mask (high 32)
// in rax. Guest N/Z/C/V cannot ride the host flags (x86 CF/ZF/SF/OF do not
// match ARM C/Z/N/V across SUB: ARM C is NOT-borrow), so the flags live in
// r8d at their ARM bit positions (N=31, Z=30, C=29, V=28) and every
// flag-setting stencil rebuilds them with SETcc. Register assignment uses
// caller-saved registers only (rdi keeps the entry CpuState* untouched),
// so kernels use no stack at all:
//
//   rdi  CpuState* (entry value, never clobbered)
//   esi  clamped scheduler prefix limit
//   r8d  guest NZCV
//   r9d  retained xPSR bits (~N/Z/C/V)
//   r11d failed-branch mask
//   eax/ecx/edx/r10d scratch (plus ah for a fourth flag accumulator)
//
// Guest registers stream through CpuState memory per op, so every prefix
// exit is precise by construction and no write masks are needed.

class X86Emitter {
public:
    std::vector<std::uint8_t> code;
    struct Fixup {
        std::size_t pos{0}; // offset of the rel32 field
        std::size_t label{0};
    };
    std::vector<Fixup> fixups;
    std::vector<std::size_t> labels; // label -> code offset, SIZE_MAX unbound

    std::size_t makeLabel() {
        labels.push_back(static_cast<std::size_t>(-1));
        return labels.size() - 1U;
    }
    void bindLabel(std::size_t label) { labels[label] = code.size(); }

    void u8(std::uint8_t b) { code.push_back(b); }
    void u32(std::uint32_t w) {
        code.push_back(static_cast<std::uint8_t>(w));
        code.push_back(static_cast<std::uint8_t>(w >> 8U));
        code.push_back(static_cast<std::uint8_t>(w >> 16U));
        code.push_back(static_cast<std::uint8_t>(w >> 24U));
    }
    static std::uint8_t rex(bool w, bool r, bool b) {
        return static_cast<std::uint8_t>(0x40U | (w ? 8U : 0U) | (r ? 4U : 0U) | (b ? 1U : 0U));
    }
    static std::uint8_t modrm(std::uint32_t mod, std::uint32_t reg, std::uint32_t rm) {
        return static_cast<std::uint8_t>((mod << 6U) | ((reg & 7U) << 3U) | (rm & 7U));
    }
    // ModR/M + displacement for [base + disp].
    void mem(std::uint32_t reg, std::uint32_t base, std::uint32_t disp) {
        const auto disp32 = static_cast<std::int32_t>(disp);
        if (disp32 >= -128 && disp32 <= 127 && ((base & 7U) != 5U)) {
            u8(modrm(1U, reg, base));
            u8(static_cast<std::uint8_t>(disp32));
        } else {
            u8(modrm(2U, reg, base));
            u32(disp);
        }
    }
    void prefixRex(bool w, std::uint32_t reg, std::uint32_t rm) {
        const bool r = (reg & 8U) != 0U;
        const bool b = (rm & 8U) != 0U;
        if (w || r || b) u8(rex(w, r, b));
    }
    void pushReg(std::uint32_t r) {
        if ((r & 8U) != 0U) u8(0x41U);
        u8(static_cast<std::uint8_t>(0x50U | (r & 7U)));
    }
    void popReg(std::uint32_t r) {
        if ((r & 8U) != 0U) u8(0x41U);
        u8(static_cast<std::uint8_t>(0x58U | (r & 7U)));
    }
    void ret() { u8(0xC3U); }
    // mov r64, r64.
    void movRR64(std::uint32_t dst, std::uint32_t src) {
        u8(rex(true, (dst & 8U) != 0U, (src & 8U) != 0U));
        u8(0x8BU);
        u8(modrm(3U, dst, src));
    }
    // mov r32, r32.
    void movRR(std::uint32_t dst, std::uint32_t src) {
        prefixRex(false, dst, src);
        u8(0x8BU);
        u8(modrm(3U, dst, src));
    }
    void movR32M(std::uint32_t dst, std::uint32_t base, std::uint32_t disp) {
        prefixRex(false, dst, base);
        u8(0x8BU);
        mem(dst, base & 7U, disp);
    }
    void movM32R(std::uint32_t base, std::uint32_t disp, std::uint32_t src) {
        prefixRex(false, src, base);
        u8(0x89U);
        mem(src, base & 7U, disp);
    }
    void movR32Imm(std::uint32_t dst, std::uint32_t imm) {
        if ((dst & 8U) != 0U) u8(0x41U);
        u8(static_cast<std::uint8_t>(0xB8U | (dst & 7U)));
        u32(imm);
    }
    void movM32Imm(std::uint32_t base, std::uint32_t disp, std::uint32_t imm) {
        if ((base & 8U) != 0U) u8(0x41U);
        u8(0xC7U);
        mem(0U, base & 7U, disp);
        u32(imm);
    }
    // ALU ext: 0=add, 1=or, 4=and, 5=sub, 6=xor, 7=cmp.
    void aluRR(std::uint32_t op, std::uint32_t dst, std::uint32_t src) {
        static constexpr std::uint8_t kOpcodes[8] = {
            0x01U, 0x09U, 0x00U, 0x00U, 0x21U, 0x29U, 0x31U, 0x39U};
        prefixRex(false, src, dst);
        u8(kOpcodes[op & 7U]);
        u8(modrm(3U, src, dst));
    }
    // ALU r32, [base + disp].
    void aluRM(std::uint32_t op, std::uint32_t dst, std::uint32_t base, std::uint32_t disp) {
        static constexpr std::uint8_t kOpcodes[8] = {
            0x03U, 0x0BU, 0x00U, 0x00U, 0x23U, 0x2BU, 0x33U, 0x3BU};
        prefixRex(false, dst, base);
        u8(kOpcodes[op & 7U]);
        mem(dst, base & 7U, disp);
    }
    void aluRImm(std::uint32_t op, std::uint32_t dst, std::uint32_t imm) {
        const auto simm = static_cast<std::int32_t>(imm);
        const bool narrow = simm >= -128 && simm <= 127;
        if ((dst & 8U) != 0U) u8(0x41U);
        u8(narrow ? 0x83U : 0x81U);
        u8(modrm(3U, op & 7U, dst));
        if (narrow) u8(static_cast<std::uint8_t>(simm));
        else u32(imm);
    }
    void testRR(std::uint32_t a, std::uint32_t b) {
        prefixRex(false, b, a);
        u8(0x85U);
        u8(modrm(3U, b, a));
    }
    void testRImm(std::uint32_t r, std::uint32_t imm) {
        if ((r & 8U) != 0U) u8(0x41U);
        u8(0xF7U);
        u8(modrm(3U, 0U, r));
        u32(imm);
    }
    // cmp dword [base + disp], imm8.
    void cmpMImm8(std::uint32_t base, std::uint32_t disp, std::uint8_t imm) {
        if ((base & 8U) != 0U) u8(0x41U);
        u8(0x83U);
        mem(7U, base & 7U, disp);
        u8(imm);
    }
    void cmpR32Imm(std::uint32_t r, std::uint32_t imm) { aluRImm(7U, r, imm); }
    void cmpR64Imm8(std::uint32_t r, std::uint8_t imm) {
        u8(rex(true, false, (r & 8U) != 0U));
        u8(0x83U);
        u8(modrm(3U, 7U, r));
        u8(imm);
    }
    void notR(std::uint32_t r) {
        if ((r & 8U) != 0U) u8(0x41U);
        u8(0xF7U);
        u8(modrm(3U, 2U, r));
    }
    // kind: 1=ror, 4=shl, 5=shr, 7=sar.
    void shImm(std::uint32_t kind, std::uint32_t r, std::uint8_t amount) {
        if ((r & 8U) != 0U) u8(0x41U);
        u8(0xC1U);
        u8(modrm(3U, kind & 7U, r));
        u8(amount);
    }
    void shCl(std::uint32_t kind, std::uint32_t r) {
        if ((r & 8U) != 0U) u8(0x41U);
        u8(0xD3U);
        u8(modrm(3U, kind & 7U, r));
    }
    void shR64Imm(std::uint32_t kind, std::uint32_t r, std::uint8_t amount) {
        u8(rex(true, false, (r & 8U) != 0U));
        u8(0xC1U);
        u8(modrm(3U, kind & 7U, r));
        u8(amount);
    }
    void orR64R64(std::uint32_t dst, std::uint32_t src) {
        u8(rex(true, (dst & 8U) != 0U, (src & 8U) != 0U));
        u8(0x0BU);
        u8(modrm(3U, dst, src));
    }
    // SETcc r8. cc: 0=O, 2=C/B, 3=NC, 4=Z, 5=NZ, 8=S.
    void setcc(std::uint32_t cc, std::uint32_t r) {
        if ((r & 8U) != 0U) u8(0x41U);
        u8(0x0FU);
        u8(static_cast<std::uint8_t>(0x90U | (cc & 15U)));
        u8(modrm(3U, 0U, r));
    }
    // movzx eax, ah: zero-extends the AH flag accumulator. Never takes a
    // REX prefix (AH is only encodable without one); callers use eax/ah
    // solely as a dead scratch pair here.
    void movzxEaxAh() {
        u8(0x0FU);
        u8(0xB6U);
        u8(modrm(3U, 0U, 4U));
    }
    // x86 cc: 2=B, 3=NB, 4=Z, 5=NZ, 6=BE, 8=S, 12=L, 13=NL, 14=LE, 15=NLE.
    void jcc(std::uint32_t cc, std::size_t label) {
        u8(0x0FU);
        u8(static_cast<std::uint8_t>(0x80U | (cc & 15U)));
        fixups.push_back({code.size(), label});
        u32(0U);
    }
    void jmp(std::size_t label) {
        u8(0xE9U);
        fixups.push_back({code.size(), label});
        u32(0U);
    }
    bool patchBranches(std::string& error) {
        for (const auto& fix : fixups) {
            if (fix.label >= labels.size() || labels[fix.label] == static_cast<std::size_t>(-1)) {
                error = "micro-jit: unbound label";
                return false;
            }
            const std::int64_t delta = static_cast<std::int64_t>(labels[fix.label])
                - static_cast<std::int64_t>(fix.pos + 4U);
            if (delta < INT32_MIN || delta > INT32_MAX) {
                error = "micro-jit: branch out of range";
                return false;
            }
            const auto rel = static_cast<std::int32_t>(delta);
            std::memcpy(&code[fix.pos], &rel, sizeof(rel));
        }
        return true;
    }
};

namespace x86detail {

constexpr std::uint32_t kEax = 0U;
constexpr std::uint32_t kEcx = 1U;
constexpr std::uint32_t kEdx = 2U;
constexpr std::uint32_t kR10 = 10U;
constexpr std::uint32_t kFailed = 11U; // r11d failed-branch mask
constexpr std::uint32_t kMax = 6U; // esi clamped scheduler limit (entry rsi)
constexpr std::uint32_t kNzcv = 8U; // r8d guest NZCV at ARM positions
constexpr std::uint32_t kRetain = 9U; // r9d retained xPSR bits (~NZCV)
constexpr std::uint32_t kState = 7U; // rdi CpuState* (entry value kept)

constexpr std::uint32_t kFlagN = 1U << 31U;
constexpr std::uint32_t kFlagZ = 1U << 30U;
constexpr std::uint32_t kFlagC = 1U << 29U;
constexpr std::uint32_t kFlagV = 1U << 28U;

struct Offsets {
    std::uint32_t r{0};
    std::uint32_t xpsr{0};
    std::uint32_t addr{0};
};

inline std::uint32_t guestDisp(const Offsets& o, std::uint8_t guest) {
    return o.r + (guest == 14U ? 56U : static_cast<std::uint32_t>(guest) * 4U);
}

inline void loadGuest(X86Emitter& e, std::uint32_t tmp, const Offsets& o, std::uint8_t guest) {
    e.movR32M(tmp, kState, guestDisp(o, guest));
}

inline void storeGuest(X86Emitter& e, const Offsets& o, std::uint8_t guest, std::uint32_t tmp) {
    e.movM32R(kState, guestDisp(o, guest), tmp);
}

// Zero the accumulators used by the flag finishers. Emitted before the
// flag-setting ALU op (xor overwrites flags; the op restores them).
inline void beginFlags(X86Emitter& e) {
    e.aluRR(6U, kEcx, kEcx); // xor ecx, ecx
    e.aluRR(6U, kEdx, kEdx);
    e.aluRR(6U, kR10, kR10);
}

// Rebuilds guest NZCV from an ADD/SUB result in eax into r8d. C comes from
// CF (setnc for SUB: ARM C is NOT-borrow), V from OF, N/Z from the result
// (N via the AH accumulator: eax is dead after the test/store). When
// do_store, the result is committed to the guest register first: stores
// leave flags untouched, so the capture stays exact.
inline void endAddSub(X86Emitter& e, const Offsets& o, bool is_sub,
                      std::uint8_t rd, bool do_store) {
    e.setcc(is_sub ? 3U : 2U, kEcx); // C
    e.setcc(0U, kEdx); // V
    if (do_store) e.movM32R(kState, guestDisp(o, rd), kEax);
    e.testRR(kEax, kEax);
    e.setcc(4U, kR10); // Z
    e.setcc(8U, 4U); // N -> ah (eax dead: stored/tested above)
    e.movzxEaxAh();
    e.shImm(4U, kEcx, 29U);
    e.shImm(4U, kEdx, 28U);
    e.shImm(4U, kR10, 30U);
    e.shImm(4U, kEax, 31U);
    e.aluRR(1U, kEcx, kEdx);
    e.aluRR(1U, kEcx, kR10);
    e.aluRR(1U, kEcx, kEax);
    e.movRR(kNzcv, kEcx);
}

// Rebuilds guest NZCV from a logical/shift result into r8d and commits the
// result to the guest register first (stores leave flags untouched). V is
// always preserved; C is preserved or taken from ecx (0/1) when new_carry.
// The V/C extraction reuses eax once the result is committed and tested.
inline void endLogic(X86Emitter& e, const Offsets& o, bool new_carry, std::uint8_t rd) {
    e.movM32R(kState, guestDisp(o, rd), kEax);
    e.aluRR(6U, kEdx, kEdx);
    e.aluRR(6U, kR10, kR10);
    e.testRR(kEax, kEax);
    e.setcc(8U, kEdx); // N
    e.setcc(4U, kR10); // Z
    e.shImm(4U, kEdx, 31U);
    e.shImm(4U, kR10, 30U);
    if (new_carry) {
        e.shImm(4U, kEcx, 29U);
        e.movRR(kEax, kNzcv);
        e.aluRImm(4U, kEax, kFlagV); // preserved V from old NZCV
        e.aluRR(1U, kEcx, kEax);
        e.aluRR(1U, kEcx, kEdx);
        e.aluRR(1U, kEcx, kR10);
        e.movRR(kNzcv, kEcx);
    } else {
        e.movRR(kEax, kNzcv);
        e.aluRImm(4U, kEax, kFlagC | kFlagV); // preserved C/V
        e.aluRR(1U, kEax, kEdx);
        e.aluRR(1U, kEax, kR10);
        e.movRR(kNzcv, kEax);
    }
}

// Computes shiftC(value, type, amount) into dst (carry discarded) for the
// admitted ALU-shift amounts. Mirrors the AArch64 accept set: lsl/lsr/asr
// reject out-of-range amounts so lowering falls back instead.
inline bool shiftValue(X86Emitter& e, std::uint32_t dst, const Offsets& o,
                       std::uint8_t src, ShiftType type, std::uint32_t amount,
                       std::string& error) {
    switch (type) {
    case ShiftType::lsl:
        if (amount >= 32U) {
            error = "lsl amount >= 32";
            return false;
        }
        loadGuest(e, dst, o, src);
        if (amount != 0U) e.shImm(4U, dst, static_cast<std::uint8_t>(amount));
        return true;
    case ShiftType::lsr:
        if (amount == 0U || amount > 32U) {
            error = "lsr amount out of range";
            return false;
        }
        if (amount == 32U) {
            e.aluRR(6U, dst, dst); // xor: result 0
            return true;
        }
        loadGuest(e, dst, o, src);
        e.shImm(5U, dst, static_cast<std::uint8_t>(amount));
        return true;
    case ShiftType::asr:
        if (amount == 0U || amount > 32U) {
            error = "asr amount out of range";
            return false;
        }
        loadGuest(e, dst, o, src);
        e.shImm(7U, dst, static_cast<std::uint8_t>(amount == 32U ? 31U : amount));
        return true;
    case ShiftType::ror: {
        const std::uint32_t rotation = amount & 31U;
        loadGuest(e, dst, o, src);
        if (rotation != 0U) e.shImm(1U, dst, static_cast<std::uint8_t>(rotation));
        return true;
    }
    case ShiftType::rrx:
        // (oldC << 31) | (value >> 1); old C is guest C in r8d.
        loadGuest(e, dst, o, src);
        e.movRR(kEcx, kNzcv);
        e.shImm(5U, kEcx, 29U);
        e.aluRImm(4U, kEcx, 1U);
        e.shImm(5U, dst, 1U);
        e.shImm(4U, kEcx, 31U);
        e.aluRR(1U, dst, kEcx);
        return true;
    }
    error = "unsupported shift type";
    return false;
}

inline bool emitX86Op(X86Emitter& e, const Offsets& o, const MicroInst& op, std::string& error) {
    switch (op.op) {
    case MicroOp::nop:
        return true;
    case MicroOp::mov_imm:
        e.movM32Imm(kState, guestDisp(o, op.rd), op.imm);
        if (op.set_flags) {
            // N/Z from the constant; C is the shifter carry (or retained),
            // V is always retained. Folded at JIT-compile time.
            const std::uint32_t keep = op.carry_valid ? kFlagV : (kFlagC | kFlagV);
            std::uint32_t set = 0U;
            if (op.carry_valid && op.carry) set |= kFlagC;
            if ((op.imm & kFlagN) != 0U) set |= kFlagN;
            if (op.imm == 0U) set |= kFlagZ;
            e.aluRImm(4U, kNzcv, keep);
            if (set != 0U) e.aluRImm(1U, kNzcv, set);
        }
        return true;
    case MicroOp::movw:
        e.movM32Imm(kState, guestDisp(o, op.rd), op.imm & 0xffffU);
        return true;
    case MicroOp::movt:
        loadGuest(e, kEax, o, op.rd);
        e.aluRImm(4U, kEax, 0xffffU);
        e.aluRImm(1U, kEax, (op.imm & 0xffffU) << 16U);
        storeGuest(e, o, op.rd, kEax);
        return true;
    case MicroOp::add_imm:
    case MicroOp::sub_imm: {
        const bool is_sub = op.op == MicroOp::sub_imm;
        loadGuest(e, kEax, o, op.rn);
        if (op.set_flags) beginFlags(e);
        e.aluRImm(is_sub ? 5U : 0U, kEax, op.imm);
        if (op.set_flags) endAddSub(e, o, is_sub, op.rd, true);
        else storeGuest(e, o, op.rd, kEax);
        return true;
    }
    case MicroOp::cmp_imm:
    case MicroOp::cmn_imm: {
        const bool is_sub = op.op == MicroOp::cmp_imm;
        loadGuest(e, kEax, o, op.rn);
        beginFlags(e);
        e.aluRImm(is_sub ? 5U : 0U, kEax, op.imm);
        endAddSub(e, o, is_sub, 0U, false);
        return true;
    }
    case MicroOp::add_reg:
    case MicroOp::sub_reg: {
        const bool is_sub = op.op == MicroOp::sub_reg;
        loadGuest(e, kEax, o, op.rn);
        if (op.set_flags) beginFlags(e); // xor flags die in the add/sub below
        e.aluRM(is_sub ? 5U : 0U, kEax, kState, guestDisp(o, op.rm));
        if (op.set_flags) endAddSub(e, o, is_sub, op.rd, true);
        else storeGuest(e, o, op.rd, kEax);
        return true;
    }
    case MicroOp::and_reg:
    case MicroOp::orr_reg:
    case MicroOp::eor_reg: {
        const std::uint32_t alu = op.op == MicroOp::and_reg ? 4U : op.op == MicroOp::orr_reg ? 1U : 6U;
        loadGuest(e, kEax, o, op.rn);
        e.aluRM(alu, kEax, kState, guestDisp(o, op.rm));
        if (op.set_flags) endLogic(e, o, false, op.rd); // C/V retained
        else storeGuest(e, o, op.rd, kEax);
        return true;
    }
    case MicroOp::bic_reg:
        // BIC has no flag-setting host form: plain op, then N/Z with C/V kept.
        loadGuest(e, kEax, o, op.rm);
        e.notR(kEax); // NOT leaves flags untouched
        e.aluRM(4U, kEax, kState, guestDisp(o, op.rn));
        if (op.set_flags) endLogic(e, o, false, op.rd);
        else storeGuest(e, o, op.rd, kEax);
        return true;
    case MicroOp::add_shift:
    case MicroOp::sub_shift: {
        const bool is_sub = op.op == MicroOp::sub_shift;
        loadGuest(e, kEax, o, op.rn);
        if (op.set_flags) beginFlags(e); // zeros die in shift/alu below; add sets fresh flags
        if (!shiftValue(e, kEdx, o, op.rm, op.shift, op.shift_amount, error)) return false;
        e.aluRR(is_sub ? 5U : 0U, kEax, kEdx);
        if (op.set_flags) endAddSub(e, o, is_sub, op.rd, true);
        else storeGuest(e, o, op.rd, kEax);
        return true;
    }
    case MicroOp::lsl:
    case MicroOp::lsr:
    case MicroOp::asr:
    case MicroOp::ror:
    case MicroOp::rrx: {
        ShiftType type = op.shift;
        if (op.op == MicroOp::lsl) type = ShiftType::lsl;
        else if (op.op == MicroOp::lsr) type = ShiftType::lsr;
        else if (op.op == MicroOp::asr) type = ShiftType::asr;
        else if (op.op == MicroOp::ror) type = ShiftType::ror;
        else type = ShiftType::rrx;
        if (!op.shift_immediate) {
            // Register amount, never flag-setting (lowering declines flags
            // here; decline so the block keeps its cached-handler behavior).
            if (op.set_flags) {
                error = "register shift with flags unsupported";
                return false;
            }
            if (type == ShiftType::rrx) {
                error = "register rrx unsupported";
                return false;
            }
            loadGuest(e, kEax, o, op.rn);
            loadGuest(e, kEcx, o, op.rm);
            e.aluRImm(4U, kEcx, 0xffU);
            if (type == ShiftType::ror) {
                e.shCl(1U, kEax); // ror masks the count to 5 bits: exact
                storeGuest(e, o, op.rd, kEax);
                return true;
            }
            const std::size_t shift_done = e.makeLabel();
            const std::size_t shift_big = e.makeLabel();
            e.cmpR32Imm(kEcx, 32U);
            e.jcc(3U, shift_big); // jae: amount >= 32
            e.shCl(type == ShiftType::lsl ? 4U : type == ShiftType::lsr ? 5U : 7U, kEax);
            e.jmp(shift_done);
            e.bindLabel(shift_big);
            if (type == ShiftType::asr) {
                e.shImm(7U, kEax, 31U); // sign fill
            } else {
                e.aluRR(6U, kEax, kEax); // lsl/lsr by >= 32 is zero
            }
            e.bindLabel(shift_done);
            storeGuest(e, o, op.rd, kEax);
            return true;
        }
        const std::uint32_t amount = type == ShiftType::rrx ? 1U : op.shift_amount;
        // Immediate form reads the shifted value from rm (executor parity).
        switch (type) {
        case ShiftType::lsl:
            if (amount >= 32U) {
                if (amount == 0U) break; // handled below
                error = "lsl amount >= 32";
                return false;
            }
            break;
        case ShiftType::lsr:
        case ShiftType::asr:
            if (amount == 0U || amount > 32U) {
                error = type == ShiftType::lsr ? "lsr amount out of range" : "asr amount out of range";
                return false;
            }
            break;
        case ShiftType::ror:
        case ShiftType::rrx:
            break;
        }
        if (!op.set_flags) {
            if (!shiftValue(e, kEax, o, op.rm, type, type == ShiftType::rrx ? 1U : amount, error))
                return false;
            storeGuest(e, o, op.rd, kEax);
            return true;
        }
        // Flag-setting immediate shifts: N/Z from the result, V retained,
        // C is the last bit shifted out (shiftC parity).
        loadGuest(e, kEax, o, op.rm);
        switch (type) {
        case ShiftType::lsl:
            if (amount == 0U) {
                endLogic(e, o, false, op.rd); // result kept, C/V retained
                break;
            }
            e.aluRR(6U, kEcx, kEcx);
            e.shImm(4U, kEax, static_cast<std::uint8_t>(amount));
            e.setcc(2U, kEcx); // CF is the last bit shifted out
            endLogic(e, o, true, op.rd);
            break;
        case ShiftType::lsr:
            if (amount == 32U) {
                e.movRR(kEcx, kEax);
                e.shImm(5U, kEcx, 31U); // carry = old bit 31
                e.aluRR(6U, kEax, kEax); // result 0
            } else {
                e.aluRR(6U, kEcx, kEcx);
                e.shImm(5U, kEax, static_cast<std::uint8_t>(amount));
                e.setcc(2U, kEcx);
            }
            endLogic(e, o, true, op.rd);
            break;
        case ShiftType::asr:
            if (amount == 32U) {
                e.movRR(kEcx, kEax);
                e.shImm(5U, kEcx, 31U); // carry = old bit 31
                e.shImm(7U, kEax, 31U); // sign fill
            } else {
                e.aluRR(6U, kEcx, kEcx);
                e.shImm(7U, kEax, static_cast<std::uint8_t>(amount));
                e.setcc(2U, kEcx);
            }
            endLogic(e, o, true, op.rd);
            break;
        case ShiftType::ror: {
            const std::uint32_t rotation = amount & 31U;
            if (rotation == 0U && amount != 0U) {
                e.movRR(kEcx, kEax);
                e.shImm(5U, kEcx, 31U); // carry = bit 31
                endLogic(e, o, true, op.rd);
            } else if (rotation == 0U) {
                endLogic(e, o, false, op.rd); // amount 0 keeps value and carry
            } else {
                e.aluRR(6U, kEcx, kEcx);
                e.shImm(1U, kEax, static_cast<std::uint8_t>(rotation));
                e.setcc(2U, kEcx); // CF is the rotated-out bit (new bit 31)
                endLogic(e, o, true, op.rd);
            }
            break;
        }
        case ShiftType::rrx:
            e.movRR(kEcx, kEax);
            e.aluRImm(4U, kEcx, 1U); // new carry = old bit 0
            e.shImm(5U, kEax, 1U);
            e.movRR(kEdx, kNzcv);
            e.aluRImm(4U, kEdx, kFlagC);
            e.shImm(4U, kEdx, 2U); // old C (bit 29) -> bit 31
            e.aluRR(1U, kEax, kEdx);
            endLogic(e, o, true, op.rd);
            break;
        }
        return true;
    }
    case MicroOp::b:
    case MicroOp::bl:
        return true; // terminal PC/LR effects are compile-time constants
    case MicroOp::b_cond:
    case MicroOp::cbz:
    case MicroOp::cbnz:
        // Control transfer is emitted by compileX86() (needs taken stubs).
        error = "micro-jit: conditional branch in emitX86Op";
        return false;
    }
    error = "unhandled micro op";
    return false;
}

// Evaluates a guest condition from r8d without disturbing it and jumps to
// `taken` when it passes. Uses eax/ecx as scratch. The fallthrough (failed)
// path records op_index in the r11d mask. Returns false for NV (rejected at
// lowering; defensive).
inline bool emitX86CondTaken(X86Emitter& e, Condition cond, std::uint32_t op_index,
                             std::size_t taken, std::string& error) {
    const auto markFailed = [&]() {
        e.aluRImm(1U, kFailed, 1U << op_index);
    };
    const auto testFlag = [&](std::uint32_t mask) { e.testRImm(kNzcv, mask); };
    // Extracts N (bit 31) and V (bit 28) into eax/ecx bit 0 and compares.
    const auto cmpNv = [&]() {
        e.movRR(kEax, kNzcv);
        e.shImm(5U, kEax, 31U);
        e.movRR(kEcx, kNzcv);
        e.shImm(5U, kEcx, 28U);
        e.aluRImm(4U, kEcx, 1U);
        e.aluRR(7U, kEax, kEcx); // cmp eax, ecx
    };
    switch (cond) {
    case Condition::eq: testFlag(kFlagZ); e.jcc(5U, taken); markFailed(); return true;
    case Condition::ne: testFlag(kFlagZ); e.jcc(4U, taken); markFailed(); return true;
    case Condition::cs: testFlag(kFlagC); e.jcc(5U, taken); markFailed(); return true;
    case Condition::cc: testFlag(kFlagC); e.jcc(4U, taken); markFailed(); return true;
    case Condition::mi: testFlag(kFlagN); e.jcc(5U, taken); markFailed(); return true;
    case Condition::pl: testFlag(kFlagN); e.jcc(4U, taken); markFailed(); return true;
    case Condition::vs: testFlag(kFlagV); e.jcc(5U, taken); markFailed(); return true;
    case Condition::vc: testFlag(kFlagV); e.jcc(4U, taken); markFailed(); return true;
    case Condition::hi: { // C && !Z
        testFlag(kFlagC);
        const std::size_t done = e.makeLabel();
        e.jcc(4U, done);
        testFlag(kFlagZ);
        e.jcc(4U, taken);
        e.bindLabel(done);
        markFailed();
        return true;
    }
    case Condition::ls: // !C || Z
        testFlag(kFlagC);
        e.jcc(4U, taken);
        testFlag(kFlagZ);
        e.jcc(5U, taken);
        markFailed();
        return true;
    case Condition::ge: // N == V
    case Condition::lt: // N != V
        cmpNv();
        e.jcc(cond == Condition::ge ? 4U : 5U, taken);
        markFailed();
        return true;
    case Condition::gt: { // !Z && N == V
        testFlag(kFlagZ);
        const std::size_t done = e.makeLabel();
        e.jcc(5U, done);
        cmpNv();
        e.jcc(4U, taken);
        e.bindLabel(done);
        markFailed();
        return true;
    }
    case Condition::le: // Z || N != V
        testFlag(kFlagZ);
        e.jcc(5U, taken);
        cmpNv();
        e.jcc(5U, taken);
        markFailed();
        return true;
    case Condition::al:
        e.jmp(taken);
        return true;
    case Condition::nv:
        error = "micro-jit: NV condition";
        return false;
    }
    error = "micro-jit: unknown condition";
    return false;
}

} // namespace x86detail

#endif // FIL_MICRO_JIT_X86_64

struct MicroJit::Impl {
#ifdef FIL_MICRO_JIT_NATIVE
    struct CacheEntry {
        MicroBlock block;
        MicroFunction function{nullptr};
    };
    static constexpr std::size_t kCacheEntries = 128U;

    std::mutex mutex;
    std::vector<CacheEntry> cache;
    std::uint8_t* base{nullptr};
    std::size_t capacity{0};
    std::size_t used{0};
    bool sealed{false};

    static constexpr std::size_t kArenaBytes = 4U * 1024U * 1024U;

    ~Impl() {
        if (base) munmap(base, capacity);
    }

    bool ensure(std::string& error) {
        if (base) return true;
        void* mapping = mmap(nullptr, kArenaBytes, PROT_READ | PROT_WRITE,
                             MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        if (mapping == MAP_FAILED) {
            error = "micro-jit mmap failed";
            return false;
        }
        base = static_cast<std::uint8_t*>(mapping);
        capacity = kArenaBytes;
        return true;
    }

    void* append(const void* data, std::size_t bytes, std::string& error, std::size_t& out_bytes) {
        const std::size_t aligned = (bytes + 15U) & ~15U;
        if (used + aligned > capacity) {
            error = "micro-jit arena exhausted";
            // The arena was made writable before emission; restore RX so
            // previously compiled kernels stay executable after we decline.
            if (base != nullptr && !sealed) {
                std::string seal_error;
                seal(base, 0U, seal_error);
            }
            return nullptr;
        }
        std::uint8_t* slot = base + used;
        std::memcpy(slot, data, bytes);
        if (aligned > bytes) std::memset(slot + bytes, 0, aligned - bytes);
        used += aligned;
        out_bytes = aligned;
        return slot;
    }

    // Whole-arena RW<->RX transitions around each batch of writes.
    // Compiles are rare (once per block shape), so two mprotect calls
    // per compile are negligible next to the saved dispatch overhead.
    bool makeWritable(std::string& error) {
        if (!sealed) return true;
        if (mprotect(base, capacity, PROT_READ | PROT_WRITE) != 0) {
            error = "micro-jit mprotect RW failed";
            return false;
        }
        sealed = false;
        return true;
    }

    bool seal(void* start, std::size_t bytes, std::string& error) {
        __builtin___clear_cache(static_cast<char*>(start), static_cast<char*>(start) + bytes);
        if (mprotect(base, capacity, PROT_READ | PROT_EXEC) != 0) {
            error = "micro-jit mprotect RX failed";
            return false;
        }
        sealed = true;
        return true;
    }
#endif
};

MicroJit::~MicroJit() = default;

bool MicroJit::hostSupported() noexcept {
#if defined(FIL_ENABLE_MICRO_JIT) && FIL_ENABLE_MICRO_JIT
#if defined(FIL_MICRO_JIT_AARCH64) || defined(FIL_MICRO_JIT_X86_64)
    return true;
#else
    return false;
#endif
#else
    return false;
#endif
}

std::unique_ptr<MicroJit> MicroJit::create(std::string& error) {
    auto jit = std::unique_ptr<MicroJit>(new MicroJit());
    jit->impl_ = std::unique_ptr<Impl>(new Impl());
#ifdef FIL_MICRO_JIT_NATIVE
    std::lock_guard lock(jit->impl_->mutex);
    if (!jit->impl_->ensure(error)) return nullptr;
#else
    (void)error;
#endif
    return jit;
}

MicroFunction MicroJit::compile(const MicroBlock& block, std::string& error) {
#ifdef FIL_MICRO_JIT_X86_64
    return compileX86(block, error);
#endif
#ifdef FIL_MICRO_JIT_AARCH64
    if (block.ops.empty() || block.ops.size() > MicroBlock::max_ops) {
        error = "micro-jit: bad block size";
        return nullptr;
    }
    // Serialize compilation and cache lookup: a hit reuses immutable code
    // while the arena remains owned by this MicroJit instance.
    std::lock_guard lock(impl_->mutex);
    for (auto it = impl_->cache.begin(); it != impl_->cache.end(); ++it) {
        if (sameMicroBlock(it->block, block)) {
            const MicroFunction function = it->function;
            if (std::next(it) != impl_->cache.end()) {
                Impl::CacheEntry recent = std::move(*it);
                impl_->cache.erase(it);
                impl_->cache.push_back(std::move(recent));
            }
            return function;
        }
    }
    // Resolve terminal PC/LR effects now: branchTarget() + BL link value.
    std::uint32_t final_pc = 0U;
    std::uint32_t last_pc = 0U;
    bool have_lr = false;
    std::uint32_t lr_value = 0U;
    for (const auto& op : block.ops) {
        last_pc = op.pc;
        if (op.op == MicroOp::b || op.op == MicroOp::bl) {
            final_pc = static_cast<std::uint32_t>(static_cast<std::int64_t>(op.pc) + 4 + op.branch_offset) & ~1U;
            if (op.op == MicroOp::bl) {
                have_lr = true;
                lr_value = (op.pc + op.size) | 1U;
            }
        } else {
            final_pc = op.pc + op.size;
        }
    }

    const std::size_t r_off = offsetof(CpuState, r);
    const std::size_t xpsr_off = offsetof(CpuState, xpsr);
    const std::size_t addr_off = offsetof(CpuState, instruction_address);

    // Mask bits 0..12 are r0-r12; bit 13 represents LR (r14).
    Emitter e;
    std::uint16_t read_mask = 0U;
    std::uint16_t write_mask = 0U;
    std::vector<std::uint16_t> cumulative_write_masks{0U};
    const auto reg_bit = [](std::uint8_t reg) -> std::uint16_t {
        return reg <= 12U ? static_cast<std::uint16_t>(1U << reg)
            : reg == 14U ? static_cast<std::uint16_t>(1U << 13U) : 0U;
    };
    for (const auto& op : block.ops) {
        std::uint16_t op_write_mask = 0U;
        const auto reads = [&](std::uint8_t reg) { read_mask |= reg_bit(reg); };
        const auto writes = [&](std::uint8_t reg) { op_write_mask |= reg_bit(reg); };
        switch (op.op) {
        case MicroOp::mov_imm: case MicroOp::movw: writes(op.rd); break;
        case MicroOp::movt: reads(op.rd); writes(op.rd); break;
        case MicroOp::add_imm: case MicroOp::sub_imm: reads(op.rn); writes(op.rd); break;
        case MicroOp::cmp_imm: case MicroOp::cmn_imm: reads(op.rn); break;
        case MicroOp::add_reg: case MicroOp::sub_reg:
        case MicroOp::and_reg: case MicroOp::orr_reg: case MicroOp::eor_reg: case MicroOp::bic_reg:
            reads(op.rn); reads(op.rm); writes(op.rd); break;
        case MicroOp::add_shift: case MicroOp::sub_shift:
            reads(op.rn); reads(op.rm); writes(op.rd); break;
        case MicroOp::lsl: case MicroOp::lsr: case MicroOp::asr: case MicroOp::ror: case MicroOp::rrx:
            if (op.shift_immediate) reads(op.rm);
            else { reads(op.rn); reads(op.rm); }
            writes(op.rd); break;
        case MicroOp::cbz: case MicroOp::cbnz: reads(op.rn); break;
        case MicroOp::bl: op_write_mask |= static_cast<std::uint16_t>(1U << 13U); break;
        case MicroOp::nop: case MicroOp::b: case MicroOp::b_cond: break;
        }
        write_mask |= op_write_mask;
        cumulative_write_masks.push_back(static_cast<std::uint16_t>(
            cumulative_write_masks.back() | op_write_mask));
    }
    // Load every register referenced by the block, including destinations,
    // so a prefix ending before a later write can safely commit the old value.
    const std::uint16_t load_mask = static_cast<std::uint16_t>(read_mask | write_mask);
    const auto emitRegisterTransfers = [&](std::uint16_t mask, bool store) {
        auto transferOne = [&](std::uint32_t guest) {
            const std::size_t off = guest == 13U ? r_off + 56U : r_off + guest * 4U;
            const std::uint32_t host = guest == 13U ? 15U : 2U + guest;
            if (store) e.strW(host, off); else e.ldrW(host, off);
        };
        for (std::uint32_t guest = 0U; guest < 13U;) {
            if ((mask & (1U << guest)) == 0U) { ++guest; continue; }
            if (guest < 12U && (mask & (1U << (guest + 1U))) != 0U) {
                const std::size_t off = r_off + guest * 4U;
                if (store) e.stpW(2U + guest, 3U + guest, off);
                else e.ldpW(2U + guest, 3U + guest, off);
                guest += 2U;
            } else {
                transferOne(guest++);
            }
        }
        if ((mask & (1U << 13U)) != 0U) transferOne(13U);
    };

    // Clamp size_t max to the bounded block length before it is packed into
    // the 16-bit limit field; this also preserves max==0 and max>=block size.
    // Do this before loading guest NZCV because CMP changes host flags.
    e.clampMax(static_cast<std::uint32_t>(block.ops.size()));
    // Prologue: seed host flags from guest xPSR and load only touched
    // registers. x1 carries the clamped scheduler prefix limit.
    e.ldrW(16U, xpsr_off);
    e.msrNzcv(16U);
    e.clearTopNibble(16U); // retained xPSR bits (~N/Z/C/V)
    const std::size_t done_label = e.makeLabel();
    e.cbzX1(done_label); // max == 0 commits nothing.
    e.stashMax(); // high32 of x16; low32 keeps retained bits.
    emitRegisterTransfers(load_mask, false);
    // Commit epilogue shared by the full path and every prefix exit:
    // merge flags, store regs, then constant r15/instruction_address.
    // Prefix exits skip the BL link update (a terminal BL only runs full).
    const auto emitEpilogue = [&](std::uint32_t pc_value, std::uint32_t ia_value,
                                  bool link, std::uint16_t store_mask) {
        if (link && have_lr) e.movImm(15U, lr_value);
        e.lsr64(1U, 16U, 48U); // failed-branch mask -> x1 before merge
        e.mrsNzcv(17U);
        e.logicReg(2U, 16U, 16U, 17U); // orr w16, w16, w17
        e.strW(16U, xpsr_off);
        emitRegisterTransfers(store_mask, true);
        e.movImm(17U, pc_value);
        e.strW(17U, r_off + 60U);
        e.movImm(17U, ia_value);
        e.strW(17U, addr_off);
    };
    std::vector<std::size_t> exit_labels;
    exit_labels.reserve(block.ops.size());
    for (std::size_t i = 0; i < block.ops.size(); ++i) exit_labels.push_back(e.makeLabel());
    // Taken-branch stubs, one per conditional op (bound after the epilogues).
    std::vector<std::size_t> taken_labels(block.ops.size(), 0U);
    std::vector<bool> has_taken(block.ops.size(), false);
    std::vector<std::uint32_t> taken_targets(block.ops.size(), 0U);
    for (std::size_t i = 0; i < block.ops.size(); ++i) {
        const auto& op = block.ops[i];
        if (op.op == MicroOp::b_cond) {
            taken_labels[i] = e.makeLabel();
            has_taken[i] = true;
            taken_targets[i] = static_cast<std::uint32_t>(
                static_cast<std::int64_t>(op.pc) + 4 + op.branch_offset) & ~1U;
        } else if (op.op == MicroOp::cbz || op.op == MicroOp::cbnz) {
            taken_labels[i] = e.makeLabel();
            has_taken[i] = true;
            taken_targets[i] = static_cast<std::uint32_t>(
                static_cast<std::int64_t>(op.pc) + 4 + op.imm) & ~1U;
        }
    }
    for (std::size_t i = 0; i < block.ops.size(); ++i) {
        const auto& op = block.ops[i];
        if (op.op == MicroOp::b_cond) {
            if (!e.emitCondTaken(op.condition, static_cast<std::uint32_t>(i), taken_labels[i], error)) return nullptr;
        } else if (op.op == MicroOp::cbz) {
            e.cbzW(hostReg(op.rn), taken_labels[i]); // taken when zero
        } else if (op.op == MicroOp::cbnz) {
            e.cbnzW(hostReg(op.rn), taken_labels[i]);
        } else if (!emitOp(e, op, error)) {
            return nullptr;
        }
        if (i + 1U < block.ops.size()) {
            // After op i, i+1 ops retired: exit iff max < i+2.
            e.checkMax(static_cast<std::uint32_t>(i + 2U), exit_labels[i + 1U]);
        }
    }
    emitEpilogue(final_pc, last_pc, true, cumulative_write_masks[block.ops.size()]);
    e.movRetMasked(static_cast<std::uint32_t>(block.ops.size()));
    const std::size_t end_label = e.makeLabel();
    e.b(end_label);
    for (std::size_t i = 1U; i < block.ops.size(); ++i) {
        e.bindLabel(exit_labels[i]);
        // Retired ops [0, i): next PC is pcs[i], last address pcs[i-1].
        emitEpilogue(block.ops[i].pc, block.ops[i - 1U].pc, false,
                     cumulative_write_masks[i]);
        e.movRetMasked(static_cast<std::uint32_t>(i));
        e.ret();
    }
    for (std::size_t i = 0; i < block.ops.size(); ++i) {
        if (!has_taken[i]) continue;
        e.bindLabel(taken_labels[i]);
        emitEpilogue(taken_targets[i], block.ops[i].pc, false,
                     cumulative_write_masks[i + 1U]);
        e.movRetMasked(static_cast<std::uint32_t>(i + 1U));
        e.ret();
    }
    e.bindLabel(done_label);
    e.movRet(0U);
    e.bindLabel(end_label);
    e.ret();
    if (!e.patchBranches(error)) return nullptr;

#if defined(FIL_MICRO_JIT_DUMP)
    for (std::size_t i = 0; i < e.words.size(); ++i) {
        std::clog << "  micro[" << i << "] = " << std::hex << std::setw(8)
                  << std::setfill('0') << e.words[i] << std::dec << '\n';
    }
#endif
    if (!impl_->makeWritable(error)) return nullptr;
    std::size_t bytes = 0U;
    void* slot = impl_->append(e.words.data(), e.words.size() * sizeof(std::uint32_t), error, bytes);
    if (!slot) return nullptr;
    if (!impl_->seal(slot, bytes, error)) return nullptr;
    ++compilations_;
    bytes_emitted_ += bytes;
    MicroFunction fn = nullptr;
    std::memcpy(&fn, &slot, sizeof(fn));
    if (impl_->cache.size() == Impl::kCacheEntries) impl_->cache.erase(impl_->cache.begin());
    impl_->cache.push_back(Impl::CacheEntry{block, fn});
    return fn;
#else
    (void)block;
    error = "micro-jit unsupported on this host";
    return nullptr;
#endif
}

#ifdef FIL_MICRO_JIT_X86_64

MicroFunction MicroJit::compileX86(const MicroBlock& block, std::string& error) {
    using namespace x86detail;
    if (block.ops.empty() || block.ops.size() > MicroBlock::max_ops) {
        error = "micro-jit: bad block size";
        return nullptr;
    }
    // Serialize compilation and cache lookup: a hit reuses immutable code
    // while the arena remains owned by this MicroJit instance.
    std::lock_guard lock(impl_->mutex);
    for (auto it = impl_->cache.begin(); it != impl_->cache.end(); ++it) {
        if (sameMicroBlock(it->block, block)) {
            const MicroFunction function = it->function;
            if (std::next(it) != impl_->cache.end()) {
                Impl::CacheEntry recent = std::move(*it);
                impl_->cache.erase(it);
                impl_->cache.push_back(std::move(recent));
            }
            return function;
        }
    }
    // Resolve terminal PC/LR effects now: branchTarget() + BL link value.
    std::uint32_t final_pc = 0U;
    std::uint32_t last_pc = 0U;
    bool have_lr = false;
    std::uint32_t lr_value = 0U;
    for (const auto& op : block.ops) {
        last_pc = op.pc;
        if (op.op == MicroOp::b || op.op == MicroOp::bl) {
            final_pc = static_cast<std::uint32_t>(static_cast<std::int64_t>(op.pc) + 4 + op.branch_offset) & ~1U;
            if (op.op == MicroOp::bl) {
                have_lr = true;
                lr_value = (op.pc + op.size) | 1U;
            }
        } else {
            final_pc = op.pc + op.size;
        }
    }

    const Offsets o{
        static_cast<std::uint32_t>(offsetof(CpuState, r)),
        static_cast<std::uint32_t>(offsetof(CpuState, xpsr)),
        static_cast<std::uint32_t>(offsetof(CpuState, instruction_address)),
    };

    X86Emitter e;
    // Prologue: rdi already holds CpuState*; clamp the size_t scheduler
    // limit (entry rsi) into esi, seed guest NZCV from xPSR, and clear the
    // failed-branch mask. Caller-saved registers only: no stack traffic.
    e.cmpR64Imm8(kMax, static_cast<std::uint8_t>(block.ops.size()));
    const std::size_t max_ok = e.makeLabel();
    e.jcc(6U, max_ok); // jbe: keep requested max
    e.movR32Imm(kMax, static_cast<std::uint32_t>(block.ops.size()));
    e.bindLabel(max_ok);
    const std::size_t done_label = e.makeLabel();
    e.testRR(kMax, kMax);
    e.jcc(4U, done_label); // max == 0 commits nothing
    e.movR32M(kRetain, kState, o.xpsr);
    e.movRR(kNzcv, kRetain);
    e.aluRImm(4U, kRetain, ~(kFlagN | kFlagZ | kFlagC | kFlagV)); // retained bits
    e.aluRImm(4U, kNzcv, kFlagN | kFlagZ | kFlagC | kFlagV); // guest NZCV
    e.aluRR(6U, kFailed, kFailed); // xor: failed mask = 0

    // Commit epilogue shared by the full path and every prefix exit. Guest
    // registers are already in CpuState memory; only flags, r15, and
    // instruction_address remain. Prefix exits skip the BL link update.
    const auto emitEpilogue = [&](std::uint32_t pc_value, std::uint32_t ia_value, bool link) {
        if (link && have_lr) e.movM32Imm(kState, o.r + 56U, lr_value);
        e.movRR(kEax, kRetain);
        e.aluRR(1U, kEax, kNzcv); // or: xpsr = retained | NZCV
        e.movM32R(kState, o.xpsr, kEax);
        e.movM32Imm(kState, o.r + 60U, pc_value);
        e.movM32Imm(kState, o.addr, ia_value);
    };
    const auto emitReturn = [&](std::uint32_t retired) {
        e.movR32Imm(kEax, retired);
        e.shR64Imm(4U, kFailed, 32U); // failed mask -> high 32
        e.orR64R64(kEax, kFailed); // rax = retired | failed << 32
        e.ret();
    };

    std::vector<std::size_t> exit_labels;
    exit_labels.reserve(block.ops.size());
    for (std::size_t i = 0; i < block.ops.size(); ++i) exit_labels.push_back(e.makeLabel());
    // Taken-branch stubs, one per conditional op (bound after the epilogues).
    std::vector<std::size_t> taken_labels(block.ops.size(), 0U);
    std::vector<bool> has_taken(block.ops.size(), false);
    std::vector<std::uint32_t> taken_targets(block.ops.size(), 0U);
    for (std::size_t i = 0; i < block.ops.size(); ++i) {
        const auto& op = block.ops[i];
        if (op.op == MicroOp::b_cond) {
            taken_labels[i] = e.makeLabel();
            has_taken[i] = true;
            taken_targets[i] = static_cast<std::uint32_t>(
                static_cast<std::int64_t>(op.pc) + 4 + op.branch_offset) & ~1U;
        } else if (op.op == MicroOp::cbz || op.op == MicroOp::cbnz) {
            taken_labels[i] = e.makeLabel();
            has_taken[i] = true;
            taken_targets[i] = static_cast<std::uint32_t>(
                static_cast<std::int64_t>(op.pc) + 4 + op.imm) & ~1U;
        }
    }
    for (std::size_t i = 0; i < block.ops.size(); ++i) {
        const auto& op = block.ops[i];
        if (op.op == MicroOp::b_cond) {
            if (!emitX86CondTaken(e, op.condition, static_cast<std::uint32_t>(i),
                                 taken_labels[i], error))
                return nullptr;
        } else if (op.op == MicroOp::cbz || op.op == MicroOp::cbnz) {
            e.cmpMImm8(kState, guestDisp(o, op.rn), 0U);
            e.jcc(op.op == MicroOp::cbz ? 4U : 5U, taken_labels[i]);
        } else if (!emitX86Op(e, o, op, error)) {
            return nullptr;
        }
        if (i + 1U < block.ops.size()) {
            // After op i, i+1 ops retired: exit iff max < i+2.
            e.cmpR32Imm(kMax, static_cast<std::uint32_t>(i + 2U));
            e.jcc(2U, exit_labels[i + 1U]); // jb
        }
    }
    emitEpilogue(final_pc, last_pc, true);
    emitReturn(static_cast<std::uint32_t>(block.ops.size()));
    const std::size_t end_label = e.makeLabel();
    e.jmp(end_label);
    for (std::size_t i = 1U; i < block.ops.size(); ++i) {
        e.bindLabel(exit_labels[i]);
        // Retired ops [0, i): next PC is pcs[i], last address pcs[i-1].
        emitEpilogue(block.ops[i].pc, block.ops[i - 1U].pc, false);
        emitReturn(static_cast<std::uint32_t>(i));
    }
    for (std::size_t i = 0; i < block.ops.size(); ++i) {
        if (!has_taken[i]) continue;
        e.bindLabel(taken_labels[i]);
        emitEpilogue(taken_targets[i], block.ops[i].pc, false);
        emitReturn(static_cast<std::uint32_t>(i + 1U));
    }
    e.bindLabel(done_label);
    e.aluRR(6U, kEax, kEax); // xor eax, eax: max == 0 returns 0
    e.ret();
    e.bindLabel(end_label);
    e.ret();
    if (!e.patchBranches(error)) return nullptr;

    if (!impl_->makeWritable(error)) return nullptr;
    std::size_t bytes = 0U;
    void* slot = impl_->append(e.code.data(), e.code.size(), error, bytes);
    if (!slot) return nullptr;
    if (!impl_->seal(slot, bytes, error)) return nullptr;
    ++compilations_;
    bytes_emitted_ += bytes;
    MicroFunction fn = nullptr;
    std::memcpy(&fn, &slot, sizeof(fn));
    if (impl_->cache.size() == Impl::kCacheEntries) impl_->cache.erase(impl_->cache.begin());
    impl_->cache.push_back(Impl::CacheEntry{block, fn});
    return fn;
}

#endif // FIL_MICRO_JIT_X86_64

} // namespace fil::cpu
