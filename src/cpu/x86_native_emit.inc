// Included inside fil::cpu on x86-64 hosts.
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
            if ((base & 7U) == 4U) u8(0x24U); // SIB: no index, base=rsp
            u8(static_cast<std::uint8_t>(disp32));
        } else {
            u8(modrm(2U, reg, base));
            if ((base & 7U) == 4U) u8(0x24U); // SIB: no index, base=rsp
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
    void callReg(std::uint32_t r) { prefixRex(false, 2U, r); u8(0xFFU); u8(modrm(3U, 2U, r)); }
    void movR64Imm(std::uint32_t dst, std::uint64_t imm) {
        u8(rex(true, false, (dst & 8U) != 0U));
        u8(static_cast<std::uint8_t>(0xB8U | (dst & 7U)));
        for (unsigned i = 0; i < 8U; ++i) u8(static_cast<std::uint8_t>(imm >> (i * 8U)));
    }
    void leaR64(std::uint32_t dst, std::uint32_t base, std::uint32_t disp) {
        u8(rex(true, (dst & 8U) != 0U, (base & 8U) != 0U)); u8(0x8DU); mem(dst, base & 7U, disp);
    }
    void movR64M(std::uint32_t dst, std::uint32_t base, std::uint32_t disp) {
        u8(rex(true, (dst & 8U) != 0U, (base & 8U) != 0U)); u8(0x8BU); mem(dst, base & 7U, disp);
    }
    void movM64R(std::uint32_t base, std::uint32_t disp, std::uint32_t src) {
        u8(rex(true, (src & 8U) != 0U, (base & 8U) != 0U)); u8(0x89U); mem(src, base & 7U, disp);
    }
    void cmpR64M(std::uint32_t reg, std::uint32_t base, std::uint32_t disp) {
        u8(rex(true, (reg & 8U) != 0U, (base & 8U) != 0U)); u8(0x3BU); mem(reg, base & 7U, disp);
    }
    void addR64R64(std::uint32_t dst, std::uint32_t src) {
        u8(rex(true, (src & 8U) != 0U, (dst & 8U) != 0U)); u8(0x01U); u8(modrm(3U, src, dst));
    }
    void addR64Imm8(std::uint32_t reg, std::uint8_t imm) {
        u8(rex(true, false, (reg & 8U) != 0U)); u8(0x83U); u8(modrm(3U, 0U, reg)); u8(imm);
    }
    void subRsp16() { u8(0x48U); u8(0x83U); u8(0xECU); u8(16U); }
    void addRsp16() { u8(0x48U); u8(0x83U); u8(0xC4U); u8(16U); }
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
    case MicroOp::mov_reg:
        loadGuest(e, kEax, o, op.rm);
        if (op.set_flags) endLogic(e, o, false, op.rd); // N/Z updated; C/V retained
        else storeGuest(e, o, op.rd, kEax);
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
    case MicroOp::cmp_reg:
    case MicroOp::cmn_reg: {
        const bool is_sub = op.op == MicroOp::cmp_reg;
        loadGuest(e, kEax, o, op.rn);
        beginFlags(e);
        e.aluRM(is_sub ? 5U : 0U, kEax, kState, guestDisp(o, op.rm));
        endAddSub(e, o, is_sub, 0U, false);
        return true;
    }
    case MicroOp::extend: {
        const bool byte = op.extend_kind == ExtendKind::uxtb
            || op.extend_kind == ExtendKind::sxtb;
        const bool sign = op.extend_kind == ExtendKind::sxtb
            || op.extend_kind == ExtendKind::sxth;
        loadGuest(e, kEax, o, op.rm);
        if (op.imm != 0U) e.shImm(1U, kEax, static_cast<std::uint8_t>(op.imm & 31U)); // ror
        if (sign) {
            // Shift the target field to the top, then arithmetic-shift back to
            // sign-extend (byte: 24, halfword: 16).
            const std::uint8_t amount = byte ? 24U : 16U;
            e.shImm(4U, kEax, amount); // shl
            e.shImm(7U, kEax, amount); // sar
        } else {
            e.aluRImm(4U, kEax, byte ? 0xffU : 0xffffU); // and
        }
        storeGuest(e, o, op.rd, kEax);
        return true;
    }
    case MicroOp::ubfx: {
        loadGuest(e, kEax, o, op.rn);
        if (op.shift_amount != 0U) e.shImm(5U, kEax, op.shift_amount); // shr
        if (op.imm != 32U) {
            e.aluRImm(4U, kEax, (std::uint32_t{1U} << op.imm) - 1U); // and mask
        }
        storeGuest(e, o, op.rd, kEax);
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
