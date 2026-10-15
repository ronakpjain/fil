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
#include <mutex>
#include <vector>

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
    void csel(std::uint32_t rd, std::uint32_t rn, std::uint32_t rm, std::uint32_t cond) {
        emit(0x1A800000U | ((rm & 31U) << 16U) | ((cond & 15U) << 12U) | ((rn & 31U) << 5U) | (rd & 31U));
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
        // Terminal PC/LR effects are compile-time constants in the epilogue.
        return true;
    }
    error = "unhandled micro op";
    return false;
}

#endif // FIL_MICRO_JIT_AARCH64

struct MicroJit::Impl {
#ifdef FIL_MICRO_JIT_AARCH64
    std::mutex mutex;
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

    Emitter e;
    // Prologue: seed host flags from guest xPSR, retain non-NZCV bits,
    // load r0-r12 + LR.
    e.ldrW(16U, xpsr_off);
    e.msrNzcv(16U);
    e.clearTopNibble(16U); // retained xPSR bits (~N/Z/C/V)
    for (std::uint32_t i = 0U; i < 6U; ++i) e.ldpW(2U + 2U * i, 3U + 2U * i, r_off + 8U * i);
    e.ldrW(14U, r_off + 48U);
    e.ldrW(15U, r_off + 56U);
    for (const auto& op : block.ops) {
        if (!emitOp(e, op, error)) return nullptr;
    }
    // Epilogue.
    if (have_lr) e.movImm(15U, lr_value);
    e.mrsNzcv(17U);
    e.logicReg(2U, 16U, 16U, 17U); // orr w16, w16, w17
    e.strW(16U, xpsr_off);
    for (std::uint32_t i = 0U; i < 6U; ++i) e.stpW(2U + 2U * i, 3U + 2U * i, r_off + 8U * i);
    e.strW(14U, r_off + 48U);
    e.strW(15U, r_off + 56U);
    e.movImm(17U, final_pc);
    e.strW(17U, r_off + 60U);
    e.movImm(17U, last_pc);
    e.strW(17U, addr_off);
    e.ret();

    if (std::getenv("FIL_MICRO_JIT_DUMP") != nullptr) {
        for (std::size_t i = 0; i < e.words.size(); ++i)
            std::fprintf(stderr, "  micro[%zu] = %08x\n", i, e.words[i]);
    }
    std::lock_guard lock(impl_->mutex);
    if (!impl_->makeWritable(error)) return nullptr;
    std::size_t bytes = 0U;
    void* slot = impl_->append(e.words, error, bytes);
    if (!slot) return nullptr;
    if (!impl_->seal(slot, bytes, error)) return nullptr;
    ++compilations_;
    bytes_emitted_ += bytes;
    MicroFunction fn = nullptr;
    std::memcpy(&fn, &slot, sizeof(fn));
    return fn;
#else
    (void)block;
    error = "micro-jit unsupported on this host";
    return nullptr;
#endif
}

} // namespace fil::cpu
