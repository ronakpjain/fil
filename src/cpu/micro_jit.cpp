// Bespoke FilIR stencil emitters.
//
// The pure micro-block backend emits AArch64 or x86_64 stencils. The mixed
// native backend emits integer/memory/trace stencils on both hosts; other
// hosts compile to a stub. Guest regs map to
// host regs for the whole pure block:
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
#include "fil/cpu/native_ir.hpp"
#include "fil/cpu/native_execution.hpp"
#include "fil/cpu/native_stack.hpp"
#include "fil/cpu/native_trace.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
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

bool microKernelProfitable(std::array<std::uint64_t, 5> native_ns,
                           std::array<std::uint64_t, 5> handler_ns) noexcept {
    std::size_t wins = 0U;
    for (std::size_t i = 0U; i < native_ns.size(); ++i) {
        if (native_ns[i] != 0U && native_ns[i] < handler_ns[i]) ++wins;
    }
    std::sort(native_ns.begin(), native_ns.end());
    std::sort(handler_ns.begin(), handler_ns.end());
    const auto native = native_ns[2];
    const auto handler = handler_ns[2];
    // Subtraction avoids overflow even for saturated timing samples.
    return wins >= 4U && native != 0U && handler != 0U
        && native < handler && native <= handler - handler / 10U;
}

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
        && a.shift_immediate == b.shift_immediate && a.extend_kind == b.extend_kind
        && a.branch_offset == b.branch_offset
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
    // SBFM 32-bit: sign-extending bitfield move (used for SXTB/SXTH).
    void sbfm(std::uint32_t rd, std::uint32_t rn, std::uint32_t immr, std::uint32_t imms) {
        emit(0x13000000U | ((immr & 31U) << 16U) | ((imms & 31U) << 10U) | ((rn & 31U) << 5U) | (rd & 31U));
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
        std::uint8_t kind{0}; // 0=unused, 1=cbz/cbnz/b.cond (imm19), 2=b (imm26)
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
    void bCond(std::uint32_t condition, std::size_t label) {
        fixups.push_back({words.size(), label, 1U});
        emit(0x54000000U | (condition & 15U));
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
        // Shifted operands land in w17; standalone RRX keeps outgoing carry in w1.
        {
            const auto scratch = wd == 17U ? 1U : 17U;
            e.mrsNzcv(scratch);
            e.ubfm(wd, wn, 1U, 31U); // lsr #1
            e.lsr64(scratch, scratch, 29U);
            e.bfi(wd, scratch, 31U, 1U);
        }
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
    case MicroOp::mov_reg:
        if (wd != wm) e.logicReg(2U, wd, 31U, wm);
        if (op.set_flags) e.setNzKeepCv(wd);
        return true;
    case MicroOp::movw:
        e.movz(wd, op.imm & 0xffffU, 0U);
        return true;
    case MicroOp::movt:
        e.movk(wd, op.imm & 0xffffU, 1U);
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
    case MicroOp::cmp_reg:
        // SUBS XZR, rn, rm: NZCV only, no register writeback.
        e.addSubReg(3U, 31U, wn, wm);
        return true;
    case MicroOp::cmn_reg:
        // ADDS XZR, rn, rm: NZCV only, no register writeback.
        e.addSubReg(1U, 31U, wn, wm);
        return true;
    case MicroOp::tst_reg:
        e.andsKeepCv(31U, wn, wm);
        return true;
    case MicroOp::extend: {
        // Rotate first, then extract: imms is an end bit, not a width;
        // a wrapping UBFM/SBFM bitfield is not a rotated extract.
        const std::uint32_t amount = op.imm & 31U;
        const std::uint32_t width = (op.extend_kind == ExtendKind::uxtb
            || op.extend_kind == ExtendKind::sxtb) ? 7U : 15U;
        const bool signed_extend = op.extend_kind == ExtendKind::sxtb
            || op.extend_kind == ExtendKind::sxth;
        if (amount != 0U) {
            e.movImm(1U, amount);
            e.shiftV(3U, 17U, wm, 1U);
        }
        const auto source = amount == 0U ? wm : 17U;
        if (signed_extend) e.sbfm(wd, source, 0U, width);
        else e.ubfm(wd, source, 0U, width);
        return true;
    }
    case MicroOp::ubfx: {
        // UBFM with immr=lsb, imms=lsb+width-1 extracts the bitfield.
        e.ubfm(wd, wn, op.shift_amount & 31U,
            (static_cast<std::uint32_t>(op.shift_amount) + op.imm - 1U) & 31U);
        return true;
    }
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
            if (op.set_flags && type != ShiftType::rrx) {
                // x1 keeps amount[7:0], outgoing carry[8], and the old NZCV
                // in its high word. Derive carry before committing wd so all
                // source/destination aliases retain their architectural value.
                e.andImm(1U, wm, 0U, 7U);
                e.mrsNzcv(17U); e.bfi64(1U, 17U, 32U, 32U);
                const auto zero = e.makeLabel(), finish = e.makeLabel(), done = e.makeLabel();
                e.cbzW(1U, zero);
                if (type == ShiftType::ror) {
                    e.shiftV(3U, wd, wn, 1U);
                    e.ubfm(17U, wd, 31U, 31U); e.bfi64(1U, 17U, 8U, 1U);
                } else {
                    const auto big = e.makeLabel();
                    e.cmpImm(1U, 32U); e.bCond(2U, big); // HS: amount >=32
                    if (type == ShiftType::lsl) {
                        e.movImm(17U, 32U); e.addSubReg(2U, 17U, 17U, 1U);
                    } else e.addSubImm(2U, 17U, 1U, 1U);
                    e.shiftV(1U, 17U, wn, 17U); e.ubfm(17U, 17U, 0U, 0U);
                    e.bfi64(1U, 17U, 8U, 1U);
                    e.shiftV(type == ShiftType::lsl ? 0U : type == ShiftType::lsr ? 1U : 2U,
                        wd, wn, 1U);
                    e.b(finish);
                    e.bindLabel(big);
                    if (type == ShiftType::asr) {
                        e.ubfm(17U, wn, 31U, 31U); e.bfi64(1U, 17U, 8U, 1U);
                        e.sbfmAsr(wd, wn, 31U);
                    } else {
                        const auto oversized = e.makeLabel();
                        e.bCond(8U, oversized); // HI: shifts >32 have zero carry.
                        const auto bit = type == ShiftType::lsl ? 0U : 31U;
                        e.ubfm(17U, wn, bit, bit); e.bfi64(1U, 17U, 8U, 1U);
                        e.bindLabel(oversized);
                        e.logicReg(2U, wd, 31U, 31U);
                    }
                }
                e.bindLabel(finish);
                e.lsr64(17U, 1U, 32U); e.msrNzcv(17U);
                e.ubfm(1U, 1U, 8U, 8U); e.setNzCarry(wd, 1U); e.b(done);
                e.bindLabel(zero);
                e.logicReg(2U, wd, 31U, wn);
                e.lsr64(17U, 1U, 32U); e.msrNzcv(17U); e.setNzKeepCv(wd);
                e.bindLabel(done);
                return true;
            }
            // Register amount without flags.
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
            // Preserve guest NZCV above the amount in x1 across range-check CMP.
            e.mrsNzcv(17U);
            e.bfi64(1U, 17U, 32U, 32U);
            if (type == ShiftType::asr) {
                // amount >= 32 -> sign fill; else asrv.
                e.sbfmAsr(17U, wn, 31U); // fill = 0 / 0xffffffff
                e.shiftV(kind, wd, wn, 1U);
                e.cmpImm(1U, 32U);
                e.csel(wd, 17U, wd, 2U); // hs: amount >= 32 -> fill
                e.lsr64(17U, 1U, 32U);
                e.msrNzcv(17U);
                return true;
            }
            e.shiftV(kind, wd, wn, 1U);
            e.cmpImm(1U, 32U);
            e.csel(wd, 31U, wd, 2U); // hs: amount >= 32 -> 0
            e.lsr64(17U, 1U, 32U);
            e.msrNzcv(17U);
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
                carry_lsb = amount - 1U;
            }
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
                carry_lsb = amount - 1U;
            }
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

#include "aarch64_native_emit.hpp"

#endif // FIL_MICRO_JIT_AARCH64

#ifdef FIL_MICRO_JIT_X86_64
#include "x86_native_emit.hpp"
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
    struct NativeCacheEntry { NativeBlock block; NativeFunction function{nullptr}; };
    std::deque<NativeCacheEntry> native_cache;
    // One immutable record per committed prefix, plus the complete trace.
    // A shared native exit epilogue indexes these records; they are data, not
    // executable stencils repeated for each possible exit reason.
    struct TracePrefixPc {
        std::uint32_t current, previous, next, reserved;
    };
    static_assert(sizeof(TracePrefixPc) == 16U);
    struct TraceCacheEntry {
        NativeTracePlan plan;
        NativeFunction function{nullptr};
        std::vector<TracePrefixPc> prefix_pcs;
    };
    std::deque<TraceCacheEntry> trace_cache;
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
        // UBSan's indirect-function check probes the eight bytes before an
        // entry point. Keep a mapped, zero metadata prefix (not a compiler
        // function signature), including for the first kernel in the arena.
        constexpr std::size_t probe_prefix = 16U;
        const std::size_t aligned = (bytes + 15U) & ~15U;
        if (used + probe_prefix + aligned > capacity) {
            error = "micro-jit arena exhausted";
            // The arena was made writable before emission; restore RX so
            // previously compiled kernels stay executable after we decline.
            if (base != nullptr && !sealed) {
                std::string seal_error;
                seal(base, 0U, seal_error);
            }
            return nullptr;
        }
        std::memset(base + used, 0, probe_prefix);
        std::uint8_t* slot = base + used + probe_prefix;
        std::memcpy(slot, data, bytes);
        if (aligned > bytes) std::memset(slot + bytes, 0, aligned - bytes);
        used += probe_prefix + aligned;
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

namespace {

[[maybe_unused]] void executeNativeTrace(NativeExecutionContext* ctx,
                                          const NativeTracePlan* plan) noexcept {
    if (ctx == nullptr || ctx->cpu_state == nullptr || plan == nullptr) return;
    ctx->retired_count = 0U;
    ctx->exit_reason = NativeExitReason::none;
    CpuState& state = *ctx->cpu_state;
    // A non-null prefix is a public caller assertion, but still validate its
    // complete selected range and aggregate credit before honoring it.
    const bool admitted = nativeAdmittedCyclePrefixValid(ctx);
    for (std::size_t i = 0; i < plan->ops.size(); ++i) {
        const auto& op = plan->ops[i];
        ctx->current_instruction_pc = op.pc;
        if (ctx->retired_count >= ctx->instruction_limit || ctx->boundary_stop) {
            ctx->exit_reason = NativeExitReason::instruction_limit; break;
        }
        if (!admitted && (ctx->charged_cycles == nullptr || i >= ctx->charged_cycles_count)) {
            ctx->failing_instruction_pc = op.pc;
            ctx->exit_reason = NativeExitReason::unsupported_instruction; break;
        }
        const auto cycles = admitted ? 0U : ctx->charged_cycles[i];
        const auto& inst = op.native.integer;
        const bool compare_branch = inst.op == MicroOp::cbz || inst.op == MicroOp::cbnz;
        const bool is_cond = inst.op == MicroOp::b_cond || compare_branch;
        const bool bx_lr = op.native.kind == NativeOp::bx_lr;
        const bool blx_reg = op.native.kind == NativeOp::blx_reg;
        const auto blx_target = blx_reg ? state.r[op.native.rn] : 0U;
        const bool taken = bx_lr || blx_reg || inst.op == MicroOp::b || inst.op == MicroOp::bl
            || (compare_branch ? ((state.readRegister(inst.rn) == 0U) == (inst.op == MicroOp::cbz))
                               : (is_cond && conditionPasses(inst.condition, state.xpsr)));
        if (((inst.op == MicroOp::b || is_cond || inst.op == MicroOp::bl)
             && taken != op.expected_taken)
            || (bx_lr && ((state.r[14] & 1U) == 0U
                || state.r[14] != (op.expected_next_pc | 1U)
                || (state.r[14] & 0xfffffff0U) == 0xfffffff0U))
            || (blx_reg && (blx_target != (op.expected_next_pc | 1U)
                || (blx_target & 0xfffffff0U) == 0xfffffff0U))) {
            ctx->failing_instruction_pc = op.pc;
            ctx->exit_reason = NativeExitReason::branch_mismatch;
            break;
        }
        if (!admitted && (ctx->retired_cycles > ctx->cycle_limit
            || cycles > ctx->cycle_limit - ctx->retired_cycles)) {
            ctx->exit_reason = NativeExitReason::cycle_limit; break;
        }
        const bool stack_op = op.decoded.kind == InstrKind::push || op.decoded.kind == InstrKind::pop;
        if (stack_op) {
            const auto result = nativeStackTransfer(ctx, op.pc, op.decoded.register_list,
                op.decoded.kind == InstrKind::push ? 1U : 0U, op.expected_next_pc);
            if (result == NativeStackResult::dynamic_return) {
                state.instruction_address = op.pc;
                if (!admitted) ctx->retired_cycles += cycles;
                ++ctx->retired_count;
                ctx->exit_reason = NativeExitReason::dynamic_return;
                break;
            }
            if (result != NativeStackResult::success) {
                ctx->failing_instruction_pc = op.pc;
                ctx->exit_reason = result == NativeStackResult::branch_mismatch
                    ? NativeExitReason::branch_mismatch : NativeExitReason::helper_declined;
                break;
            }
        }
        if (!stack_op && nativeOpIsMemory(op.native.kind)) {
            // The MMIO bridge observes exact progress even when the admitted
            // kernel otherwise defers cumulative cycle publication to exit.
            if (admitted) ctx->retired_cycles = ctx->admitted_cycle_prefix[i];
            const auto& memop = op.native;
            const std::uint32_t addr = memop.absolute ? memop.absolute_address
                : state.readRegister(memop.rn) + memop.offset;
            std::uint32_t loaded = 0U;
            const std::uint32_t width = nativeOpWidth(memop.kind);
            const bool mem_load = nativeOpIsLoad(memop.kind);
            const bool ok = mem_load
                ? (width == 1U ? nativeRead8(ctx, op.pc, addr, &loaded)
                               : width == 2U ? nativeRead16(ctx, op.pc, addr, &loaded)
                                            : nativeRead32(ctx, op.pc, addr, &loaded))
                : (width == 1U ? nativeWrite8(ctx, op.pc, addr, state.r[memop.rd])
                               : width == 2U ? nativeWrite16(ctx, op.pc, addr, state.r[memop.rd])
                                            : nativeWrite32(ctx, op.pc, addr, state.r[memop.rd]));
            if (!ok) {
                ctx->failing_instruction_pc = op.pc;
                ctx->exit_reason = NativeExitReason::helper_declined; break;
            }
            if (mem_load) {
                if (memop.kind == NativeOp::load_byte_signed || memop.kind == NativeOp::load_half_signed) {
                    const std::uint32_t sign_bit = width == 1U ? 0x80U : 0x8000U;
                    const std::uint32_t fill = width == 1U ? 0xffffff00U : 0xffff0000U;
                    if ((loaded & sign_bit) != 0U) loaded |= fill;
                }
                state.r[memop.rd] = loaded;
            }
        }
        if (bx_lr) {
            // The target guard above ensures a Thumb return and excludes the
            // architectural exception-return token range before any effects.
        } else if (inst.op == MicroOp::b || is_cond || inst.op == MicroOp::bl) {
            if (inst.op == MicroOp::bl) state.r[14] = (op.pc + op.size) | 1U;
        } else if (blx_reg) {
            state.r[14] = (op.pc + op.size) | 1U;
        } else if (!stack_op && op.native.kind == NativeOp::integer) {
            MicroCpu guest;
            for (std::size_t r = 0; r < guest.r.size(); ++r) guest.r[r] = state.r[r];
            guest.lr = state.r[14]; guest.pc = state.r[15]; guest.xpsr = state.xpsr;
            (void)interpretMicroOps({&inst, 1U}, guest, 1U);
            for (std::size_t r = 0; r < guest.r.size(); ++r) state.r[r] = guest.r[r];
            state.r[14] = guest.lr; state.xpsr = guest.xpsr;
        }
        state.r[15] = op.expected_next_pc;
        state.instruction_address = op.pc;
        if (!admitted) ctx->retired_cycles += cycles;
        ++ctx->retired_count;
    }
    if (admitted) ctx->retired_cycles = ctx->admitted_cycle_prefix[ctx->retired_count];
}

bool sameNativeBlock(const NativeBlock& a, const NativeBlock& b) noexcept {
    if (a.entry_pc != b.entry_pc || a.ops.size() != b.ops.size()) return false;
    for (std::size_t i = 0; i < a.ops.size(); ++i) {
        const auto& x = a.ops[i]; const auto& y = b.ops[i];
        if (x.kind != y.kind || x.pc != y.pc || x.size != y.size || x.rd != y.rd
            || x.rn != y.rn || x.offset != y.offset || x.absolute != y.absolute
            || x.absolute_address != y.absolute_address
            || (x.kind == NativeOp::integer && !sameMicroInst(x.integer, y.integer))) return false;
    }
    return true;
}

} // namespace

void interpretNativeTrace(const NativeTracePlan& plan, NativeExecutionContext& context) noexcept {
    executeNativeTrace(&context, &plan);
}

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
        case MicroOp::mov_reg: reads(op.rm); writes(op.rd); break;
        case MicroOp::movt: reads(op.rd); writes(op.rd); break;
        case MicroOp::add_imm: case MicroOp::sub_imm: reads(op.rn); writes(op.rd); break;
        case MicroOp::cmp_imm: case MicroOp::cmn_imm: reads(op.rn); break;
        case MicroOp::cmp_reg: case MicroOp::cmn_reg: case MicroOp::tst_reg:
            reads(op.rn); reads(op.rm); break;
        case MicroOp::extend: reads(op.rm); writes(op.rd); break;
        case MicroOp::ubfx: reads(op.rn); writes(op.rd); break;
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

NativeFunction MicroJit::compileNative(const NativeBlock& block, std::string& error) {
#if defined(FIL_ENABLE_MICRO_JIT) && FIL_ENABLE_MICRO_JIT && defined(FIL_MICRO_JIT_NATIVE)
    if (block.ops.empty() || block.ops.size() > NativeBlock::max_ops) {
        error = "native-jit: invalid block size";
        return nullptr;
    }
    auto expected_pc = block.entry_pc;
    for (std::size_t i = 0; i < block.ops.size(); ++i) {
        const auto& op = block.ops[i];
        if (op.kind == NativeOp::bx_lr || op.kind == NativeOp::blx_reg) {
            error = "native-jit: indirect branch is trace-only";
            return nullptr;
        }
        if (op.pc != expected_pc
            || static_cast<unsigned>(op.kind) > static_cast<unsigned>(NativeOp::blx_reg)) {
            error = "native-jit: invalid opcode or discontinuous block";
            return nullptr;
        }
        expected_pc += op.size;
        const bool extra_integer = op.kind == NativeOp::integer
            && (op.integer.op == MicroOp::extend || op.integer.op == MicroOp::ubfx);
        if (op.kind == NativeOp::integer
            && (op.integer.pc != op.pc || op.integer.size != op.size
                || op.integer.rd >= 13U || op.integer.rn >= 13U || op.integer.rm >= 13U
                || op.integer.condition != Condition::al
                || (static_cast<unsigned>(op.integer.op) > static_cast<unsigned>(MicroOp::rrx)
                    && !extra_integer)
                || static_cast<unsigned>(op.integer.shift) > static_cast<unsigned>(ShiftType::rrx)
                || (op.integer.shift_immediate && op.integer.shift_amount > 32U))) {
            error = "native-jit: invalid integer operands or metadata";
            return nullptr;
        }
        if ((op.size != 2U && op.size != 4U) || (op.pc & 1U) != 0U
            || (op.kind == NativeOp::integer
                && (op.integer.op == MicroOp::b || op.integer.op == MicroOp::bl
                    || op.integer.op == MicroOp::b_cond || op.integer.op == MicroOp::cbz
                    || op.integer.op == MicroOp::cbnz))) {
            error = "native-jit: branch or malformed op declined";
            return nullptr;
        }
        if (op.kind != NativeOp::integer && op.kind != NativeOp::bx_lr && op.kind != NativeOp::blx_reg
            && (op.rd >= 13U || (!op.absolute && op.rn > 13U))) {
            error = "native-jit: invalid memory register";
            return nullptr;
        }
    }
    std::lock_guard lock(impl_->mutex);
    for (const auto& entry : impl_->native_cache)
        if (sameNativeBlock(entry.block, block)) return entry.function;
    // Startup alone exceeds 128 shapes on the real network. Keep a bounded
    // cache large enough for steady-state kernels as well; executable
    // storage is independently bounded by the arena.
    constexpr std::size_t kNativeCacheEntries = 1024U;
    if (impl_->native_cache.size() >= kNativeCacheEntries) {
        error = "native-jit: bounded native cache full";
        return nullptr;
    }
    impl_->native_cache.emplace_back();
    auto& entry = impl_->native_cache.back();
    entry.block = block;
    std::vector<std::uint8_t> code;
#if defined(FIL_MICRO_JIT_X86_64) && !defined(_WIN32)
    using namespace x86detail;
    const Offsets cpu_offsets{static_cast<std::uint32_t>(offsetof(CpuState, r)),
        static_cast<std::uint32_t>(offsetof(CpuState, xpsr)),
        static_cast<std::uint32_t>(offsetof(CpuState, instruction_address))};
    const auto ctx_off = [](std::size_t value) { return static_cast<std::uint32_t>(value); };
    constexpr std::uint32_t kCtx = 13U, kCosts = 14U, kCpu = 12U;
    constexpr std::uint32_t kScratch = 0U, kAddr = 2U, kValue = 1U;
    constexpr auto kRetired = ctx_off(offsetof(NativeExecutionContext, retired_count));
    constexpr auto kLimit = ctx_off(offsetof(NativeExecutionContext, instruction_limit));
    constexpr auto kCycles = ctx_off(offsetof(NativeExecutionContext, retired_cycles));
    constexpr auto kCycleLimit = ctx_off(offsetof(NativeExecutionContext, cycle_limit));
    constexpr auto kCurrent = ctx_off(offsetof(NativeExecutionContext, current_instruction_pc));
    constexpr auto kFailing = ctx_off(offsetof(NativeExecutionContext, failing_instruction_pc));
    constexpr auto kReason = ctx_off(offsetof(NativeExecutionContext, exit_reason));
    constexpr auto kChargedCount = ctx_off(offsetof(NativeExecutionContext, charged_cycles_count));
    constexpr auto kAdmittedMode = ctx_off(offsetof(NativeExecutionContext, admitted_cycle_mode));
    constexpr auto kBoundaryStop = ctx_off(offsetof(NativeExecutionContext, boundary_stop));
    X86Emitter e;
    const auto instruction_exit = e.makeLabel();
    const auto cycle_exit = e.makeLabel();
    const auto helper_exit = e.makeLabel();
    const auto unsupported_exit = e.makeLabel();
    const auto finish = e.makeLabel();
    // Save all used callee-saved registers. Five pushes align RSP to 16 bytes
    // at call sites; the 16-byte scratch slot holds nativeRead32's output.
    e.pushReg(3U); e.pushReg(12U); e.pushReg(13U); e.pushReg(14U); e.pushReg(15U);
    e.subRsp16();
    e.movR64M(kCpu, 7U, ctx_off(offsetof(NativeExecutionContext, cpu_state)));
    e.movRR64(kCtx, 7U);
    e.movR64M(kCosts, 7U, ctx_off(offsetof(NativeExecutionContext, charged_cycles)));
    // Null-hint callers jump directly to the unchanged checked path. Only a
    // supplied hint pays the one-time robust validator call.
    const auto checked_cost_table = e.makeLabel();
    e.movM32Imm(kCtx, kAdmittedMode, 0U);
    e.movR64M(kScratch, kCtx, ctx_off(offsetof(NativeExecutionContext, admitted_cycle_prefix)));
    e.testRR(kScratch, kScratch); e.jcc(4U, checked_cost_table);
    e.movRR64(7U, kCtx);
    e.movR64Imm(kEax, reinterpret_cast<std::uintptr_t>(&nativeAdmittedCyclePrefixValid));
    e.callReg(kEax);
    e.movM32R(kCtx, kAdmittedMode, kEax);
    e.testRR(kEax, kEax); e.jcc(4U, checked_cost_table);
    e.movR64M(kCosts, kCtx, ctx_off(offsetof(NativeExecutionContext, admitted_cycle_prefix)));
    e.bindLabel(checked_cost_table);
    e.movRR64(7U, kCpu);
    e.movM32Imm(kCtx, kRetired, 0U);
    e.movM32Imm(kCtx, kReason, static_cast<std::uint32_t>(NativeExitReason::none));
    e.testRR(kCosts, kCosts);
    e.jcc(4U, unsupported_exit);
    e.movR32M(kRetain, 7U, cpu_offsets.xpsr);
    e.movRR(kNzcv, kRetain);
    e.aluRImm(4U, kRetain, ~(kFlagN | kFlagZ | kFlagC | kFlagV));
    e.aluRImm(4U, kNzcv, kFlagN | kFlagZ | kFlagC | kFlagV);
    for (std::size_t i = 0; i < block.ops.size(); ++i) {
        const auto& op = block.ops[i];
        e.movM32Imm(kCtx, kCurrent, op.pc);
        e.movR32M(kScratch, kCtx, kRetired);
        e.aluRM(7U, kScratch, kCtx, kLimit);
        e.jcc(3U, instruction_exit); // retired >= limit
        // A committed-span MMIO that must stop exits here before running the
        // next op, so the retired prefix already includes that instruction.
        e.cmpM8Imm8(kCtx, kBoundaryStop, 0U);
        e.jcc(5U, instruction_exit);
        e.movR32M(kScratch, kCtx, kChargedCount);
        e.cmpR32Imm(kScratch, static_cast<std::uint32_t>(i + 1U));
        e.jcc(2U, unsupported_exit);
        // Aggregate admission already certified that this op is affordable, so
        // skip the per-op cost probe and cycle accumulation entirely.
        const auto skip_cycle_probe = e.makeLabel();
        e.movR32M(kScratch, kCtx, kAdmittedMode);
        e.testRR(kScratch, kScratch); e.jcc(5U, skip_cycle_probe);
        e.movR32M(kAddr, kCosts, static_cast<std::uint32_t>(i * sizeof(std::uint32_t)));
        e.movR64M(kScratch, kCtx, kCycles);
        e.addR64R64(kScratch, kAddr);
        e.jcc(2U, cycle_exit); // carry: cycle sum overflow
        e.cmpR64M(kScratch, kCtx, kCycleLimit);
        e.jcc(7U, cycle_exit);
        e.bindLabel(skip_cycle_probe);
        if (op.kind == NativeOp::integer) {
            if (!emitX86Op(e, cpu_offsets, op.integer, error)) {
                impl_->native_cache.pop_back();
                return nullptr;
            }
        } else {
            if (op.absolute) e.movR32Imm(kAddr, op.absolute_address);
            else {
                if (op.rn == 13U) {
                    // Architectural SP selects MSP/PSP; do not assume the
                    // public r13 mirror was synchronized by an API caller.
                    const auto use_msp = e.makeLabel();
                    const auto have_sp = e.makeLabel();
                    e.movR32M(kScratch, kCpu, cpu_offsets.xpsr);
                    e.aluRImm(4U, kScratch, xpsr_ipsr_mask);
                    e.jcc(5U, use_msp);
                    e.movR32M(kScratch, kCpu, ctx_off(offsetof(CpuState, control)));
                    e.aluRImm(4U, kScratch, 2U);
                    e.jcc(4U, use_msp);
                    e.movR32M(kScratch, kCpu, ctx_off(offsetof(CpuState, psp)));
                    e.jmp(have_sp);
                    e.bindLabel(use_msp);
                    e.movR32M(kScratch, kCpu, ctx_off(offsetof(CpuState, msp)));
                    e.bindLabel(have_sp);
                } else {
                    loadGuest(e, kScratch, cpu_offsets, op.rn);
                }
                if (op.offset != 0U) e.aluRImm(0U, kScratch, op.offset);
                e.movRR(kAddr, kScratch);
            }
            const bool mem_load = nativeOpIsLoad(op.kind);
            const std::uint32_t mem_width = nativeOpWidth(op.kind);
            if (!mem_load) loadGuest(e, kValue, cpu_offsets, op.rd);
            // A committed span's MMIO helper needs the exact prefix cycle count.
            // Checked mode already accumulated it; admitted mode reads the
            // certified cumulative table at this op index.
            const auto skip_prefix_store = e.makeLabel();
            e.movR32M(kScratch, kCtx, kAdmittedMode);
            e.testRR(kScratch, kScratch); e.jcc(4U, skip_prefix_store);
            e.movR32M(kScratch, kCosts, static_cast<std::uint32_t>(i * sizeof(std::uint32_t)));
            e.movM64R(kCtx, kCycles, kScratch);
            e.bindLabel(skip_prefix_store);
            // Preserve guest NZCV/retained xPSR across the C ABI helper.
            e.movRR(3U, kNzcv); e.movRR(15U, kRetain);
            e.movRR64(7U, kCtx); // RDI = helper context
            e.movR32Imm(6U, op.pc); // RSI = instruction PC
            e.movRR(kEdx, kAddr);
            if (mem_load) e.leaR64(1U, 4U, 0U); // RCX = [RSP]
            const auto helper = mem_width == 1U
                ? (mem_load ? reinterpret_cast<std::uintptr_t>(&nativeRead8)
                            : reinterpret_cast<std::uintptr_t>(&nativeWrite8))
                : mem_width == 2U
                ? (mem_load ? reinterpret_cast<std::uintptr_t>(&nativeRead16)
                            : reinterpret_cast<std::uintptr_t>(&nativeWrite16))
                : (mem_load ? reinterpret_cast<std::uintptr_t>(&nativeRead32)
                            : reinterpret_cast<std::uintptr_t>(&nativeWrite32));
            e.movR64Imm(kEax, static_cast<std::uint64_t>(helper));
            e.callReg(kEax);
            e.movRR(kNzcv, 3U); e.movRR(kRetain, 15U);
            e.movRR64(7U, kCpu);
            // SysV guarantees only AL for a bool return, not the upper EAX bits.
            e.u8(0x84U); e.u8(0xc0U); // test al, al
            e.jcc(4U, helper_exit);
            if (mem_load) {
                e.movR32M(kScratch, 4U, 0U);
                // LDRSB/LDRSH sign-extend the loaded field before committing.
                if (op.kind == NativeOp::load_byte_signed) {
                    e.shImm(4U, kScratch, 24U); e.shImm(7U, kScratch, 24U);
                } else if (op.kind == NativeOp::load_half_signed) {
                    e.shImm(4U, kScratch, 16U); e.shImm(7U, kScratch, 16U);
                }
                storeGuest(e, cpu_offsets, op.rd, kScratch);
            }
        }
        // Per-op architectural commit: no write-back of any unrelated state.
        e.movRR(kScratch, kRetain);
        e.aluRR(1U, kScratch, kNzcv);
        e.movM32R(kCpu, cpu_offsets.xpsr, kScratch);
        e.movM32Imm(kCpu, cpu_offsets.r + 60U, op.pc + op.size);
        e.movM32Imm(kCpu, cpu_offsets.addr, op.pc);
        // Checked mode accumulates per-op charges; admitted mode publishes the
        // certified cumulative prefix once at exit instead of accumulating.
        const auto skip_cycle_commit = e.makeLabel();
        e.movR32M(kScratch, kCtx, kAdmittedMode);
        e.testRR(kScratch, kScratch); e.jcc(5U, skip_cycle_commit);
        e.movR64M(kScratch, kCtx, kCycles);
        e.movR32M(kAddr, kCosts, static_cast<std::uint32_t>(i * sizeof(std::uint32_t)));
        e.addR64R64(kScratch, kAddr);
        e.movM64R(kCtx, kCycles, kScratch);
        e.bindLabel(skip_cycle_commit);
        e.movM32Imm(kCtx, kRetired, static_cast<std::uint32_t>(i + 1U));
    }
    e.jmp(finish);
    const auto emitExit = [&](std::size_t label, NativeExitReason reason) {
        e.bindLabel(label);
        e.movM32Imm(kCtx, kReason, static_cast<std::uint32_t>(reason));
        if (reason == NativeExitReason::helper_declined
            || reason == NativeExitReason::unsupported_instruction) {
            e.movR32M(kScratch, kCtx, kCurrent);
            e.movM32R(kCtx, kFailing, kScratch);
        }
        e.jmp(finish);
    };
    emitExit(instruction_exit, NativeExitReason::instruction_limit);
    emitExit(cycle_exit, NativeExitReason::cycle_limit);
    emitExit(helper_exit, NativeExitReason::helper_declined);
    emitExit(unsupported_exit, NativeExitReason::unsupported_instruction);
    e.bindLabel(finish);
    // Publish the committed prefix: admitted_mode uses the certified cumulative
    // table indexed by retired_count; checked mode already accumulated it.
    const auto skip_prefix_publish = e.makeLabel();
    e.movR32M(kScratch, kCtx, kAdmittedMode);
    e.testRR(kScratch, kScratch); e.jcc(4U, skip_prefix_publish);
    // kAddr = kCosts + retired*4, then publish the zero-extended entry.
    e.movR32M(kAddr, kCtx, kRetired);
    e.movRR64(kScratch, kAddr);
    e.addR64R64(kAddr, kScratch); // retired*2
    e.addR64R64(kAddr, kScratch); // retired*3
    e.addR64R64(kAddr, kScratch); // retired*4
    e.addR64R64(kAddr, kCosts);
    e.movR32M(kScratch, kAddr, 0U);
    e.movM32R(kCtx, kCycles, kScratch);
    e.movM32Imm(kCtx, kCycles + 4U, 0U);
    e.bindLabel(skip_prefix_publish);
    e.addRsp16(); e.popReg(15U); e.popReg(14U); e.popReg(13U); e.popReg(12U); e.popReg(3U); e.ret();
    if (!e.patchBranches(error)) { impl_->native_cache.pop_back(); return nullptr; }
    code = std::move(e.code);
#elif defined(FIL_MICRO_JIT_AARCH64)
    if (!aarch64detail::emitBlock(entry.block, code, error)) {
        impl_->native_cache.pop_back(); return nullptr;
    }
#else
    error = "native-jit: host ABI unsupported";
    impl_->native_cache.pop_back();
    return nullptr;
#endif
    if (!impl_->makeWritable(error)) {
        impl_->native_cache.pop_back();
        return nullptr;
    }
    std::size_t bytes = 0U;
    void* slot = impl_->append(code.data(), code.size(), error, bytes);
    if (slot == nullptr || !impl_->seal(slot, bytes, error)) {
        impl_->native_cache.pop_back();
        return nullptr;
    }
    std::memcpy(&entry.function, &slot, sizeof(entry.function));
    ++compilations_;
    bytes_emitted_ += bytes;
    return entry.function;
#else
    (void)block;
    error = "native-jit unsupported on this host";
    return nullptr;
#endif
}

NativeFunction MicroJit::compileNativeTrace(const NativeTracePlan& input, std::string& error) {
#if defined(FIL_ENABLE_MICRO_JIT) && FIL_ENABLE_MICRO_JIT && defined(FIL_MICRO_JIT_NATIVE)
    NativeTracePlan plan;
    std::vector<NativeTraceOp> sources = input.ops;
    if (!buildNativeTrace(sources, plan, error)) return nullptr;
    plan.generation = input.generation;
    std::lock_guard lock(impl_->mutex);
    const auto equal = [](const NativeTracePlan& a, const NativeTracePlan& b) {
        if (a.generation != b.generation || a.ops.size() != b.ops.size()) return false;
        for (std::size_t i = 0; i < a.ops.size(); ++i) {
            const auto& x = a.ops[i]; const auto& y = b.ops[i];
            if (x.raw != y.raw || x.pc != y.pc || x.size != y.size
                || x.decoded.kind != y.decoded.kind || x.decoded.register_list != y.decoded.register_list
                || x.expected_next_pc != y.expected_next_pc || x.expected_taken != y.expected_taken
                || x.execution_cycles != y.execution_cycles
                || x.native.kind != y.native.kind || x.native.rd != y.native.rd
                || x.native.rn != y.native.rn || x.native.offset != y.native.offset
                || x.native.absolute != y.native.absolute
                || x.native.absolute_address != y.native.absolute_address
                || !sameMicroInst(x.native.integer, y.native.integer)) return false;
        }
        return true;
    };
    for (const auto& entry : impl_->trace_cache) if (equal(entry.plan, plan)) return entry.function;
    if (impl_->trace_cache.size() >= 1024U) {
        error = "native-trace: permanent descriptor cache full";
        return nullptr;
    }
    impl_->trace_cache.emplace_back();
    auto& entry = impl_->trace_cache.back();
    entry.plan = std::move(plan);
    std::vector<std::uint8_t> code;
#if defined(FIL_MICRO_JIT_X86_64) && !defined(_WIN32)
    using namespace x86detail;
    const Offsets offsets{static_cast<std::uint32_t>(offsetof(CpuState, r)),
        static_cast<std::uint32_t>(offsetof(CpuState, xpsr)),
        static_cast<std::uint32_t>(offsetof(CpuState, instruction_address))};
    constexpr std::uint32_t ctx = 13U, costs = 14U, cpu = 12U;
    // The prologue already saves RBX/R15. Keeping guest flags there avoids
    // four moves (or stack spills) around every RAM/stack helper invocation.
    constexpr std::uint32_t kNzcv = 3U, kRetain = 15U;
    const auto off = [](std::size_t n) { return static_cast<std::uint32_t>(n); };
    const auto retired = off(offsetof(NativeExecutionContext, retired_count));
    const auto limit = off(offsetof(NativeExecutionContext, instruction_limit));
    const auto cycle_count = off(offsetof(NativeExecutionContext, retired_cycles));
    const auto cycle_limit = off(offsetof(NativeExecutionContext, cycle_limit));
    const auto current = off(offsetof(NativeExecutionContext, current_instruction_pc));
    const auto failing = off(offsetof(NativeExecutionContext, failing_instruction_pc));
    const auto reason = off(offsetof(NativeExecutionContext, exit_reason));
    const auto charged_count = off(offsetof(NativeExecutionContext, charged_cycles_count));
    const auto kAdmittedMode = off(offsetof(NativeExecutionContext, admitted_cycle_mode));
    const auto boundary_stop_off = off(offsetof(NativeExecutionContext, boundary_stop));
    const auto fast_read_base_off = off(offsetof(NativeExecutionContext, fast_read_base));
    const auto fast_read_begin_off = off(offsetof(NativeExecutionContext, fast_read_begin));
    const auto fast_read_size_off = off(offsetof(NativeExecutionContext, fast_read_size));
    X86Emitter e;
    e.nzcv = kNzcv;
    const auto add_flags = e.add_flags = e.makeLabel();
    const auto sub_flags = e.sub_flags = e.makeLabel();
    const auto logic_flags = e.logic_flags = e.makeLabel();
    const auto shift_flags = e.shift_flags = e.makeLabel();
    const auto done = e.makeLabel(), early = e.makeLabel();
    e.pushReg(3U); e.pushReg(12U); e.pushReg(13U); e.pushReg(14U); e.pushReg(15U);
    e.pushReg(5U); e.pushReg(10U); e.subRsp16();
    e.movRR64(ctx, 7U);
    e.cmpR64Imm8(ctx, 0U); e.jcc(4U, early);
    e.movR64M(cpu, ctx, off(offsetof(NativeExecutionContext, cpu_state)));
    e.cmpR64Imm8(cpu, 0U); e.jcc(4U, early);
    e.movR64M(costs, ctx, off(offsetof(NativeExecutionContext, charged_cycles)));
    // Null-hint callers jump directly to the unchanged checked path. Only a
    // supplied hint pays the one-time robust validator call, which also admits
    // the cumulative table (retaining per-op load/staged checks on decline).
    const auto checked_cost_table = e.makeLabel();
    e.movM32Imm(ctx, kAdmittedMode, 0U);
    e.movR64M(3U, ctx, off(offsetof(NativeExecutionContext, admitted_cycle_prefix)));
    e.testRR(3U, 3U); e.jcc(4U, checked_cost_table);
    e.movRR64(7U, ctx);
    e.movR64Imm(kEax, reinterpret_cast<std::uintptr_t>(&nativeAdmittedCyclePrefixValid));
    e.callReg(kEax);
    e.movM32R(ctx, kAdmittedMode, kEax);
    e.testRR(kEax, kEax); e.jcc(4U, checked_cost_table);
    e.movR64M(costs, ctx, off(offsetof(NativeExecutionContext, admitted_cycle_prefix)));
    e.bindLabel(checked_cost_table);
    e.movRR64(7U, cpu);
    e.movM32Imm(ctx, retired, 0U); e.movM32Imm(ctx, reason, 0U);
    e.movR64M(5U, ctx, cycle_count); // RBP is live retired-cycle accumulator.
    e.movR32M(kRetain, cpu, offsets.xpsr); e.movRR(kNzcv, kRetain);
    e.aluRImm(4U, kRetain, ~(kFlagN | kFlagZ | kFlagC | kFlagV));
    e.aluRImm(4U, kNzcv, kFlagN | kFlagZ | kFlagC | kFlagV);
    const std::size_t n = entry.plan.ops.size();
    entry.prefix_pcs.reserve(n + 1U);
    for (std::size_t i = 0; i <= n; ++i) {
        const auto& current_op = entry.plan.ops[i == n ? n - 1U : i];
        const auto& previous_op = entry.plan.ops[i == 0U ? 0U : i - 1U];
        entry.prefix_pcs.push_back({current_op.pc, previous_op.pc, previous_op.expected_next_pc, 0U});
    }
    struct ExitLabels { std::size_t instruction, cycle, unsupported, mismatch, helper, dynamic_return; };
    std::vector<ExitLabels> exits; exits.reserve(n);
    for (std::size_t i=0; i<n; ++i)
        exits.push_back({e.makeLabel(),e.makeLabel(),e.makeLabel(),e.makeLabel(),e.makeLabel(),e.makeLabel()});
    const auto normal = e.makeLabel();
    const auto instruction_exit = e.makeLabel(), cycle_exit = e.makeLabel();
    const auto unsupported_exit = e.makeLabel(), mismatch_exit = e.makeLabel();
    const auto helper_exit = e.makeLabel(), dynamic_exit = e.makeLabel();
    const auto checked_admission = e.makeLabel();
    const auto checked_kernel = e.makeLabel();
    e.movR32M(kEax, ctx, kAdmittedMode);
    e.testRR(kEax, kEax); e.jcc(4U, checked_kernel);
    // Select cycle admission once per invocation, not three times per guest
    // instruction. Both kernels share exact-prefix exit publication and retain
    // instruction-limit, control-flow and RAM-helper guards.
    for (const bool admitted_kernel : {true, false}) {
        if (!admitted_kernel) e.bindLabel(checked_kernel);
        for (std::size_t i = 0; i < n; ++i) {
            const auto& op = entry.plan.ops[i];
            const auto& inst = op.native.integer;
            const bool bx_lr = op.native.kind == NativeOp::bx_lr;
            const bool compare_branch = inst.op == MicroOp::cbz || inst.op == MicroOp::cbnz;
            const bool conditional = inst.op == MicroOp::b_cond || compare_branch;
            // Guarded branches retain the original unsupported -> mismatch ->
            // cycle ordering. Other ops can share one checked-admission body.
            const bool shared_admission = !bx_lr && !conditional && op.native.kind != NativeOp::blx_reg;
            const bool integer_body = op.native.kind == NativeOp::integer
                && op.decoded.kind != InstrKind::push && op.decoded.kind != InstrKind::pop;
            e.cmpM32Imm(ctx, limit, static_cast<std::uint32_t>(i));
            e.jcc(6U, exits[i].instruction); // i >= instruction_limit before all other admissions
            // Only helpers can request an MMIO boundary stop. An integer op
            // cannot change the flag, so testing it again after one is redundant.
            const auto* previous = i == 0U ? nullptr : &entry.plan.ops[i - 1U];
            if (previous == nullptr || previous->native.kind != NativeOp::integer
                || previous->decoded.kind == InstrKind::push || previous->decoded.kind == InstrKind::pop) {
                e.cmpM8Imm8(ctx, boundary_stop_off, 0U);
                e.jcc(5U, exits[i].instruction);
            }
            if (!admitted_kernel && !shared_admission) {
                e.cmpM32Imm(ctx, charged_count, static_cast<std::uint32_t>(i + 1U));
                e.jcc(2U, exits[i].unsupported);
                e.cmpR64Imm8(costs, 0U); e.jcc(4U, exits[i].unsupported);
            }
            if (bx_lr) {
                e.movR32M(kEax, cpu, offsets.r + 56U);
                e.cmpR32Imm(kEax, op.expected_next_pc | 1U);
                e.jcc(5U, exits[i].mismatch);
            } else if (op.native.kind == NativeOp::blx_reg) {
                e.movR32M(kEax, cpu, offsets.r + op.native.rn * 4U);
                e.cmpR32Imm(kEax, op.expected_next_pc | 1U); e.jcc(5U, exits[i].mismatch);
                e.movRR(kEcx, kEax); e.aluRImm(4U, kEcx, 0xfffffff0U);
                e.cmpR32Imm(kEcx, 0xfffffff0U); e.jcc(4U, exits[i].mismatch);
            }
            if (conditional) {
                const auto actually_taken = e.makeLabel(), guard_done = e.makeLabel();
                if (compare_branch) {
                    loadGuest(e, kEax, offsets, inst.rn);
                    e.testRR(kEax, kEax);
                    e.jcc(inst.op == MicroOp::cbz ? 4U : 5U, actually_taken);
                } else if (!emitX86CondTaken(e, inst.condition, 0U, actually_taken, error)) {
                    impl_->trace_cache.pop_back(); return nullptr;
                }
                if (op.expected_taken) e.jmp(exits[i].mismatch); else e.jmp(guard_done);
                e.bindLabel(actually_taken);
                if (op.expected_taken) e.jmp(guard_done); else e.jmp(exits[i].mismatch);
                e.bindLabel(guard_done);
            }
            if (!admitted_kernel) {
                if (shared_admission) {
                    e.movR32Imm(kEcx, static_cast<std::uint32_t>(i));
                    e.call(checked_admission);
                    // Pure integer ops cannot decline after this admission.
                    // Commit the computed cycle total instead of reloading cost.
                    if (integer_body) e.movRR64(5U, kEdx);
                } else {
                    e.movR32M(kEax, costs, static_cast<std::uint32_t>(i * sizeof(std::uint32_t)));
                    e.movRR64(kEdx, 5U); e.addR64R64(kEdx, kEax);
                    e.jcc(2U, exits[i].cycle); e.cmpR64M(kEdx, ctx, cycle_limit); e.jcc(7U, exits[i].cycle);
                }
            }
            if (inst.op == MicroOp::bl || op.native.kind == NativeOp::blx_reg)
                e.movM32Imm(kState, offsets.r + 56U, (op.pc + op.size) | 1U);
            if (op.decoded.kind == InstrKind::push || op.decoded.kind == InstrKind::pop) {
                const auto stack_done = e.makeLabel();
                if (op.decoded.kind == InstrKind::pop) {
                    const auto slow_pop = e.makeLabel(), use_msp = e.makeLabel();
                    const auto bytes = op.stack_plan.count() * 4U;
                    // An immutable register list lets the emitter scatter the
                    // admitted frame directly, without a helper's decode loop.
                    // R8 holds the selected bank address, R9 the resulting SP.
                    e.leaR64(8U, cpu, off(offsetof(CpuState, msp)));
                    e.movR32M(kEax, cpu, offsets.xpsr);
                    e.aluRImm(4U, kEax, xpsr_ipsr_mask); e.jcc(5U, use_msp);
                    e.movR32M(kEax, cpu, off(offsetof(CpuState, control)));
                    e.aluRImm(4U, kEax, 2U); e.jcc(4U, use_msp);
                    e.leaR64(8U, cpu, off(offsetof(CpuState, psp)));
                    e.bindLabel(use_msp);
                    e.movR32M(9U, 8U, 0U);
                    e.movRR(kEdx, 9U); e.aluRM(5U, kEdx, ctx, fast_read_begin_off);
                    e.movR32M(kEax, ctx, fast_read_size_off);
                    e.aluRImm(5U, kEax, bytes); e.jcc(2U, slow_pop);
                    e.aluRR(7U, kEdx, kEax); e.jcc(7U, slow_pop);
                    e.movR64M(kEax, ctx, fast_read_base_off); e.addR64R64(kEdx, kEax);
                    if (op.stack_plan.loadsPc()) {
                        e.movR32M(kEax, kEdx, bytes - 4U);
                        // Only predicted ordinary Thumb returns take this fast
                        // path. The checked helper owns alternate/exceptional
                        // targets and their exact decline/commit semantics.
                        if ((op.expected_next_pc & 0xfffffff0U) == 0xfffffff0U) e.jmp(slow_pop);
                        e.cmpR32Imm(kEax, op.expected_next_pc | 1U); e.jcc(5U, slow_pop);
                    }
                    e.aluRImm(0U, 9U, bytes);
                    for (std::uint32_t n = 0U; n < op.stack_plan.count(); ++n) {
                        const auto reg = op.stack_plan.reg(n);
                        if (reg == 15U) continue;
                        e.movR32M(kEax, kEdx, n * 4U);
                        e.movM32R(cpu, offsets.r + reg * 4U, kEax);
                    }
                    e.movM32R(8U, 0U, 9U); e.movM32R(cpu, offsets.r + 52U, 9U);
                    if (op.stack_plan.loadsPc()) e.movM32Imm(cpu, offsets.r + 60U, op.expected_next_pc);
                    e.jmp(stack_done);
                    e.bindLabel(slow_pop);
                }
                e.movRR64(7U, ctx); e.movR32Imm(6U, op.pc);
                // The retained cache entry owns this immutable plan for the
                // generated function's entire lifetime (vector moves retain storage).
                e.movR64Imm(kEdx, reinterpret_cast<std::uintptr_t>(&op.stack_plan));
                e.movR32Imm(1U, op.expected_next_pc);
                e.movR64Imm(kEax, reinterpret_cast<std::uintptr_t>(&nativeStackTransferPrepared)); e.callReg(kEax);
                e.movRR64(7U, cpu);
                if (op.decoded.kind == InstrKind::pop && (op.decoded.register_list & 0x8000U)) {
                    e.cmpR32Imm(kEax, static_cast<std::uint32_t>(NativeStackResult::dynamic_return));
                    e.jcc(4U, exits[i].dynamic_return);
                }
                e.cmpR32Imm(kEax, static_cast<std::uint32_t>(NativeStackResult::branch_mismatch));
                e.jcc(4U, exits[i].mismatch);
                e.testRR(kEax, kEax); e.jcc(5U, exits[i].helper);
                e.bindLabel(stack_done);
            } else if (!(bx_lr || op.native.kind == NativeOp::blx_reg || conditional || inst.op == MicroOp::b || inst.op == MicroOp::bl)
                       && op.native.kind == NativeOp::integer) {
                if (!emitX86Op(e, offsets, inst, error)) { impl_->trace_cache.pop_back(); return nullptr; }
            } else if (!(bx_lr || op.native.kind == NativeOp::blx_reg || conditional || inst.op == MicroOp::b || inst.op == MicroOp::bl)) {
                const auto& m = op.native; constexpr std::uint32_t addrreg = 2U, valuereg = 1U;
                if (m.absolute) e.movR32Imm(addrreg, m.absolute_address);
                else {
                    if (m.rn == 13U) {
                        const auto use_msp=e.makeLabel(), have_sp=e.makeLabel();
                        e.movR32M(kEax,cpu,offsets.xpsr); e.aluRImm(4U,kEax,xpsr_ipsr_mask); e.jcc(5U,use_msp);
                        e.movR32M(kEax,cpu,off(offsetof(CpuState,control))); e.aluRImm(4U,kEax,2U); e.jcc(4U,use_msp);
                        e.movR32M(kEax,cpu,off(offsetof(CpuState,psp))); e.jmp(have_sp); e.bindLabel(use_msp);
                        e.movR32M(kEax,cpu,off(offsetof(CpuState,msp))); e.bindLabel(have_sp);
                    } else loadGuest(e,kEax,offsets,m.rn);
                    if (m.offset != 0U) e.aluRImm(0U,kEax,m.offset);
                    e.movRR(addrreg,kEax);
                }
                const bool t_load = nativeOpIsLoad(m.kind);
                const std::uint32_t t_width = nativeOpWidth(m.kind);
                if (!t_load) loadGuest(e,valuereg,offsets,m.rd);
                // Inline backed-RAM word loads: the host publishes a direct
                // window only when footprint tracking is off, so a range check
                // plus a plain load replaces the checked helper on the common
                // path. Any address outside the window falls through untouched.
                const bool fast_load = t_load;
                std::size_t fast_word_done = 0U;
                if (fast_load) {
                    const auto slow_load = e.makeLabel();
                    fast_word_done = e.makeLabel();
                    const auto second_window = e.makeLabel();
                    // Absolute literal loads prefer the code window; other
                    // loads prefer the stack/data window. Either may legally
                    // address the other backed region through a pointer.
                    for (const bool primary : {true, false}) {
                        const bool literal_window = primary == m.absolute;
                        const auto miss = primary ? second_window : slow_load;
                        const auto begin_off = literal_window
                            ? off(offsetof(NativeExecutionContext, literal_read_begin)) : fast_read_begin_off;
                        const auto size_off = literal_window
                            ? off(offsetof(NativeExecutionContext, literal_read_size)) : fast_read_size_off;
                        const auto base_off = literal_window
                            ? off(offsetof(NativeExecutionContext, literal_read_base)) : fast_read_base_off;
                        e.movRR(kEax, addrreg);
                        e.aluRM(5U, kEax, ctx, begin_off);
                        // Guard the entire access, including tiny windows.
                        e.movR32M(kEcx, ctx, size_off);
                        e.aluRImm(5U, kEcx, t_width); e.jcc(2U, miss);
                        e.aluRR(7U, kEax, kEcx); e.jcc(7U, miss);
                        e.movR64M(addrreg, ctx, base_off);
                        e.addR64R64(addrreg, kEax);
                        switch (m.kind) {
                        case NativeOp::load_byte: e.movzxR32M8(kEax, addrreg, 0U); break;
                        case NativeOp::load_half: e.movzxR32M16(kEax, addrreg, 0U); break;
                        case NativeOp::load_byte_signed: e.movsxR32M8(kEax, addrreg, 0U); break;
                        case NativeOp::load_half_signed: e.movsxR32M16(kEax, addrreg, 0U); break;
                        default: e.movR32M(kEax, addrreg, 0U); break;
                        }
                        storeGuest(e, offsets, m.rd, kEax);
                        e.jmp(fast_word_done);
                        if (primary) e.bindLabel(second_window);
                    }
                    e.bindLabel(slow_load);
                }
                // Publish the exact prefix cycle count for a committed-span MMIO
                // helper: checked mode already accumulated RBP; admitted mode
                // reads the certified cumulative table at this op index.
                if (admitted_kernel) {
                    e.movR32M(kEax, costs, static_cast<std::uint32_t>(i * sizeof(std::uint32_t)));
                    e.movM64R(ctx, cycle_count, kEax);
                } else {
                    e.movM64R(ctx, cycle_count, 5U);
                }
                e.movRR64(7U,ctx); e.movR32Imm(6U,op.pc); e.movRR(kEdx,addrreg);
                if (t_load) e.leaR64(1U,4U,0U);
                const auto helper = t_width == 1U
                    ? (t_load ? reinterpret_cast<std::uintptr_t>(&nativeRead8) : reinterpret_cast<std::uintptr_t>(&nativeWrite8))
                    : t_width == 2U
                    ? (t_load ? reinterpret_cast<std::uintptr_t>(&nativeRead16) : reinterpret_cast<std::uintptr_t>(&nativeWrite16))
                    : (t_load ? reinterpret_cast<std::uintptr_t>(&nativeRead32) : reinterpret_cast<std::uintptr_t>(&nativeWrite32));
                e.movR64Imm(kEax,helper); e.callReg(kEax);
                e.movRR64(7U,cpu);
                e.u8(0x84U); e.u8(0xc0U); e.jcc(4U,exits[i].helper);
                if (t_load) {
                    e.movR32M(kEax,4U,0U);
                    if (m.kind == NativeOp::load_byte_signed) {
                        e.shImm(4U, kEax, 24U); e.shImm(7U, kEax, 24U);
                    } else if (m.kind == NativeOp::load_half_signed) {
                        e.shImm(4U, kEax, 16U); e.shImm(7U, kEax, 16U);
                    }
                    storeGuest(e,offsets,m.rd,kEax);
                }
                if (fast_load) e.bindLabel(fast_word_done);
            }
            // Only the live cycle accumulator advances here. RAM-only helpers
            // consume explicit PCs and no pending NZCV/PC/progress metadata; the
            // selected exit stub publishes the exact committed prefix once.
            if (!admitted_kernel && !(shared_admission && integer_body)) {
                e.movR32M(kEax,costs,static_cast<std::uint32_t>(i*4U)); e.addR64R64(5U,kEax);
            }
        }
        e.jmp(normal);
    }
    // Cold stubs carry only a prefix index. Each reason has one trampoline,
    // and every prefix/reason shares the same native publication code.
    const auto publish = e.makeLabel(), failed_exit = e.makeLabel();
    std::vector<bool> referenced(e.labels.size(), false);
    for (const auto& fixup : e.fixups) referenced[fixup.label] = true;
    const auto exit_stub = [&](std::size_t label, std::size_t target, std::size_t prefix) {
        if (!referenced[label]) return;
        e.bindLabel(label);
        e.movR32Imm(kEcx, static_cast<std::uint32_t>(prefix));
        e.jmp(target);
    };
    for (std::size_t i = 0; i < n; ++i) {
        exit_stub(exits[i].instruction, instruction_exit, i);
        exit_stub(exits[i].cycle, cycle_exit, i);
        exit_stub(exits[i].unsupported, unsupported_exit, i);
        exit_stub(exits[i].mismatch, mismatch_exit, i);
        exit_stub(exits[i].helper, helper_exit, i);
        exit_stub(exits[i].dynamic_return, dynamic_exit, i + 1U);
    }
    e.bindLabel(normal);
    e.movR32Imm(kEcx, static_cast<std::uint32_t>(n));
    e.movR32Imm(kEax, static_cast<std::uint32_t>(NativeExitReason::none));
    e.jmp(publish);
    const auto reason_stub = [&](std::size_t label, NativeExitReason why, bool failed) {
        e.bindLabel(label);
        e.movR32Imm(kEax, static_cast<std::uint32_t>(why));
        e.jmp(failed ? failed_exit : publish);
    };
    reason_stub(instruction_exit, NativeExitReason::instruction_limit, false);
    reason_stub(cycle_exit, NativeExitReason::cycle_limit, false);
    reason_stub(unsupported_exit, NativeExitReason::unsupported_instruction, true);
    reason_stub(mismatch_exit, NativeExitReason::branch_mismatch, true);
    reason_stub(helper_exit, NativeExitReason::helper_declined, true);
    reason_stub(dynamic_exit, NativeExitReason::dynamic_return, false);
    e.bindLabel(failed_exit);
    // Failure PC is the current instruction, not the last retired instruction.
    e.movRR(kEdx, kEcx); e.shImm(4U, kEdx, 4U);
    e.movR64Imm(kR10, reinterpret_cast<std::uintptr_t>(entry.prefix_pcs.data()));
    e.addR64R64(kR10, kEdx);
    e.movR32M(kEdx, kR10, 0U); e.movM32R(ctx, failing, kEdx);
    e.bindLabel(publish);
    e.movM32R(ctx, reason, kEax);
    e.movM32R(ctx, retired, kEcx);
    e.movRR(kEdx, kEcx); e.shImm(4U, kEdx, 4U);
    e.movR64Imm(kR10, reinterpret_cast<std::uintptr_t>(entry.prefix_pcs.data()));
    e.addR64R64(kR10, kEdx);
    const auto ordinary_current = e.makeLabel(), current_ready = e.makeLabel();
    e.cmpR32Imm(kEax, static_cast<std::uint32_t>(NativeExitReason::dynamic_return));
    e.jcc(5U, ordinary_current);
    e.movR32M(kEdx, kR10, 4U); e.jmp(current_ready);
    e.bindLabel(ordinary_current); e.movR32M(kEdx, kR10, 0U);
    e.bindLabel(current_ready); e.movM32R(ctx, current, kEdx);
    const auto checked_cycles = e.makeLabel(), cycles_ready = e.makeLabel();
    e.cmpMImm8(ctx, kAdmittedMode, 0U); e.jcc(4U, checked_cycles);
    e.movRR(kEdx, kEcx); e.shImm(4U, kEdx, 2U); e.addR64R64(kEdx, costs);
    e.movR32M(kEdx, kEdx, 0U); e.movM64R(ctx, cycle_count, kEdx);
    e.jmp(cycles_ready);
    e.bindLabel(checked_cycles);
    const auto checked_cycles_ready = e.makeLabel();
    e.cmpR32Imm(kEax, static_cast<std::uint32_t>(NativeExitReason::dynamic_return));
    e.jcc(5U, checked_cycles_ready);
    // A dynamic POP return commits this op without reaching the normal
    // per-op accumulator update. Include its cost exactly once.
    e.movRR(kEdx, kEcx); e.shImm(4U, kEdx, 2U); e.addR64R64(kEdx, costs);
    e.movR32M(kEdx, kEdx, static_cast<std::uint32_t>(-4)); e.addR64R64(5U, kEdx);
    e.bindLabel(checked_cycles_ready); e.movM64R(ctx, cycle_count, 5U);
    e.bindLabel(cycles_ready);
    e.testRR(kEcx, kEcx); e.jcc(4U, done); // Empty prefix leaves CPU state untouched.
    const auto keep_dynamic_pc = e.makeLabel();
    e.cmpR32Imm(kEax, static_cast<std::uint32_t>(NativeExitReason::dynamic_return));
    e.jcc(4U, keep_dynamic_pc);
    e.movR32M(kEdx, kR10, 8U); e.movM32R(kState, offsets.r + 60U, kEdx);
    e.bindLabel(keep_dynamic_pc);
    e.movR32M(kEdx, kR10, 4U); e.movM32R(kState, offsets.addr, kEdx);
    e.movRR(kEax, kRetain); e.aluRR(1U, kEax, kNzcv); e.movM32R(kState, offsets.xpsr, kEax);
    e.bindLabel(done);
    e.movM32Imm(ctx, kAdmittedMode, 0U);
    e.addRsp16(); e.popReg(10U); e.popReg(5U); e.popReg(15U); e.popReg(14U); e.popReg(13U); e.popReg(12U); e.popReg(3U); e.ret();
    e.bindLabel(early);
    e.addRsp16(); e.popReg(10U); e.popReg(5U); e.popReg(15U); e.popReg(14U); e.popReg(13U); e.popReg(12U); e.popReg(3U); e.ret();
    // Calls preserve the arithmetic host flags and return without touching the
    // helper scratch slot. Count these bodies once, including both cycle modes.
    e.add_flags = e.sub_flags = e.logic_flags = e.shift_flags = X86Emitter::no_label;
    const auto used = [&](std::size_t label) {
        return std::any_of(e.fixups.begin(), e.fixups.end(),
            [label](const auto& fixup) { return fixup.label == label; });
    };
    if (used(checked_admission)) {
        e.bindLabel(checked_admission);
        const auto unsupported = e.makeLabel(), cycle = e.makeLabel();
        e.aluRM(7U, kEcx, ctx, charged_count); e.jcc(3U, unsupported);
        e.cmpR64Imm8(costs, 0U); e.jcc(4U, unsupported);
        e.movRR(kEax, kEcx); e.shImm(4U, kEax, 2U); e.addR64R64(kEax, costs);
        e.movR32M(kEax, kEax, 0U);
        e.movRR64(kEdx, 5U); e.addR64R64(kEdx, kEax);
        e.jcc(2U, cycle); e.cmpR64M(kEdx, ctx, cycle_limit); e.jcc(7U, cycle);
        e.ret();
        // The shared epilogue expects the main kernel's stack frame. Drop this
        // native subroutine's return address before transferring an early exit.
        e.bindLabel(unsupported); e.addR64Imm8(4U, 8U); e.jmp(unsupported_exit);
        e.bindLabel(cycle); e.addR64Imm8(4U, 8U); e.jmp(cycle_exit);
    }
    if (used(add_flags)) {
        e.bindLabel(add_flags); endAddSub(e, offsets, false, 0U, false); e.ret();
    }
    if (used(sub_flags)) {
        e.bindLabel(sub_flags); endAddSub(e, offsets, true, 0U, false); e.ret();
    }
    if (used(logic_flags)) {
        e.bindLabel(logic_flags); endLogic(e, offsets, false, 0U, false); e.ret();
    }
    if (used(shift_flags)) {
        e.bindLabel(shift_flags); endLogic(e, offsets, true, 0U, false); e.ret();
    }
    if (!e.patchBranches(error)) { impl_->trace_cache.pop_back(); return nullptr; }
    code = std::move(e.code);
#elif defined(FIL_MICRO_JIT_AARCH64)
    if (!aarch64detail::emitTrace(entry.plan, code, error)) {
        impl_->trace_cache.pop_back(); return nullptr;
    }
#else
    impl_->trace_cache.pop_back(); error = "native-trace: unsupported host"; return nullptr;
#endif
    if (!impl_->makeWritable(error)) { impl_->trace_cache.pop_back(); return nullptr; }
    std::size_t bytes = 0U;
    void* slot = impl_->append(code.data(), code.size(), error, bytes);
    if (slot == nullptr || !impl_->seal(slot, bytes, error)) { impl_->trace_cache.pop_back(); return nullptr; }
    std::memcpy(&entry.function, &slot, sizeof(entry.function));
    ++compilations_; bytes_emitted_ += bytes;
    return entry.function;
#else
    (void)input; error = "native-trace unsupported on this host"; return nullptr;
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
