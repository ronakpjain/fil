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

#include <array>
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
#endif

namespace fil::cpu {
namespace {

constexpr std::uint32_t kFlagMask = xpsr_n | xpsr_z | xpsr_c | xpsr_v;

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
        lsr64(17U, 16U, 32U);
        ubfm(17U, 17U, 0U, 15U);
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

struct MicroJit::Impl {
#ifdef FIL_MICRO_JIT_AARCH64
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

    void* append(const std::vector<std::uint32_t>& words, std::string& error, std::size_t& out_bytes) {
        const std::size_t bytes = words.size() * sizeof(std::uint32_t);
        const std::size_t aligned = (bytes + 15U) & ~15U;
        if (used + aligned > capacity) {
            error = "micro-jit arena exhausted";
            return nullptr;
        }
        std::uint8_t* slot = base + used;
        std::memcpy(slot, words.data(), bytes);
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
#ifdef FIL_MICRO_JIT_AARCH64
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
#ifdef FIL_MICRO_JIT_AARCH64
    std::lock_guard lock(jit->impl_->mutex);
    if (!jit->impl_->ensure(error)) return nullptr;
#else
    (void)error;
#endif
    return jit;
}

MicroFunction MicroJit::compile(const MicroBlock& block, std::string& error) {
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
    void* slot = impl_->append(e.words, error, bytes);
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

} // namespace fil::cpu
