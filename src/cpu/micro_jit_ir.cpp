#include "fil/cpu/micro_jit_ir.hpp"

namespace fil::cpu {
namespace {

constexpr std::uint32_t kFlagN = 1U << 31U;
constexpr std::uint32_t kFlagZ = 1U << 30U;
constexpr std::uint32_t kFlagC = 1U << 29U;
constexpr std::uint32_t kFlagV = 1U << 28U;

bool immediateShape(const DecodedInstruction& d) noexcept {
    return d.form == OperandForm::immediate && d.rm == 0U && d.ra == 0U
        && d.shift_amount == 0U && d.shift_type == ShiftType::lsl
        && !d.writeback && d.register_list == 0U && !d.fp_double;
}

bool registerShape(const DecodedInstruction& d) noexcept {
    return d.form == OperandForm::register_value && d.ra == 0U && d.imm == 0U
        && d.branch_offset == 0 && !d.writeback && d.register_list == 0U
        && !d.immediate_carry_valid && !d.immediate_carry && !d.fp_double
        && d.index && d.add;
}

bool coreReg(std::uint8_t reg) noexcept { return reg <= 12U; }

void applyNzcv(MicroCpu& cpu, const AddResult& r) noexcept {
    cpu.xpsr &= ~(kFlagN | kFlagZ | kFlagC | kFlagV);
    if (r.n) cpu.xpsr |= kFlagN;
    if (r.z) cpu.xpsr |= kFlagZ;
    if (r.c) cpu.xpsr |= kFlagC;
    if (r.v) cpu.xpsr |= kFlagV;
}

void applyNz(MicroCpu& cpu, std::uint32_t value, bool carry_kept) noexcept {
    cpu.xpsr &= ~(kFlagN | kFlagZ);
    if ((value & kFlagN) != 0U) cpu.xpsr |= kFlagN;
    if (value == 0U) cpu.xpsr |= kFlagZ;
    (void)carry_kept; // logical ops retain C; shift ops set it explicitly.
}

std::uint32_t readOp(const MicroCpu& cpu, std::uint8_t reg) noexcept {
    return reg == 14U ? cpu.lr : cpu.r[reg];
}

void writeOp(MicroCpu& cpu, std::uint8_t reg, std::uint32_t value) noexcept {
    if (reg == 14U) cpu.lr = value;
    else cpu.r[reg] = value;
}

} // namespace

const char* toString(MicroOp op) noexcept {
    switch (op) {
    case MicroOp::nop: return "nop";
    case MicroOp::mov_imm: return "mov_imm";
    case MicroOp::movw: return "movw";
    case MicroOp::movt: return "movt";
    case MicroOp::add_imm: return "add_imm";
    case MicroOp::sub_imm: return "sub_imm";
    case MicroOp::cmp_imm: return "cmp_imm";
    case MicroOp::cmn_imm: return "cmn_imm";
    case MicroOp::add_reg: return "add_reg";
    case MicroOp::sub_reg: return "sub_reg";
    case MicroOp::and_reg: return "and_reg";
    case MicroOp::orr_reg: return "orr_reg";
    case MicroOp::eor_reg: return "eor_reg";
    case MicroOp::bic_reg: return "bic_reg";
    case MicroOp::add_shift: return "add_shift";
    case MicroOp::sub_shift: return "sub_shift";
    case MicroOp::lsl: return "lsl";
    case MicroOp::lsr: return "lsr";
    case MicroOp::asr: return "asr";
    case MicroOp::ror: return "ror";
    case MicroOp::rrx: return "rrx";
    case MicroOp::b: return "b";
    case MicroOp::bl: return "bl";
    }
    return "unknown";
}

const char* toString(MicroLowerReason reason) noexcept {
    switch (reason) {
    case MicroLowerReason::ok: return "ok";
    case MicroLowerReason::bad_condition: return "bad_condition";
    case MicroLowerReason::bad_size: return "bad_size";
    case MicroLowerReason::bad_pc: return "bad_pc";
    case MicroLowerReason::unsupported_kind: return "unsupported_kind";
    case MicroLowerReason::bad_shape: return "bad_shape";
    case MicroLowerReason::bad_register: return "bad_register";
    case MicroLowerReason::bad_immediate: return "bad_immediate";
    case MicroLowerReason::branch_not_terminal: return "branch_not_terminal";
    }
    return "unknown";
}

MicroLowerResult tryLowerMicroInst(const MicroSource& source, MicroInst& out) noexcept {
    const auto& d = source.decoded;
    MicroInst inst;
    inst.pc = source.pc;
    inst.size = source.size;
    inst.set_flags = d.set_flags;
    inst.branch_offset = d.branch_offset;

    if (d.condition != Condition::al) {
        return {false, MicroLowerReason::bad_condition, "only AL is lowered"};
    }
    if (source.size != 2U && source.size != 4U) {
        return {false, MicroLowerReason::bad_size, "width must be 2 or 4"};
    }
    if (d.is_32bit != (source.size == 4U)) {
        return {false, MicroLowerReason::bad_size, "is_32bit disagrees with width"};
    }
    if ((source.pc & 1U) != 0U) {
        return {false, MicroLowerReason::bad_pc, "thumb pc must be even"};
    }

    switch (d.kind) {
    case InstrKind::nop:
        if (d.form != OperandForm::none || d.set_flags || d.rd != 0U || d.rn != 0U
            || d.rm != 0U || d.imm != 0U || d.branch_offset != 0) {
            return {false, MicroLowerReason::bad_shape, "nop shape"};
        }
        inst.op = MicroOp::nop;
        out = inst;
        return {true, MicroLowerReason::ok, {}};
    case InstrKind::mov:
        if (!immediateShape(d) || !coreReg(d.rd)) {
            return {false, !coreReg(d.rd) ? MicroLowerReason::bad_register : MicroLowerReason::bad_shape, "mov imm"};
        }
        inst.op = MicroOp::mov_imm;
        inst.rd = d.rd;
        inst.imm = d.imm;
        inst.carry_valid = d.immediate_carry_valid;
        inst.carry = d.immediate_carry;
        out = inst;
        return {true, MicroLowerReason::ok, {}};
    case InstrKind::movw:
        if (!immediateShape(d) || d.rn != 0U || d.imm > 0xffffU || d.set_flags
            || d.immediate_carry_valid || d.immediate_carry || !coreReg(d.rd)) {
            return {false, MicroLowerReason::bad_shape, "movw shape"};
        }
        inst.op = MicroOp::movw;
        inst.rd = d.rd;
        inst.imm = d.imm;
        out = inst;
        return {true, MicroLowerReason::ok, {}};
    case InstrKind::movt:
        if (!immediateShape(d) || d.rn != 0U || d.imm > 0xffffU || d.set_flags
            || d.immediate_carry_valid || d.immediate_carry || !coreReg(d.rd)) {
            return {false, MicroLowerReason::bad_shape, "movt shape"};
        }
        inst.op = MicroOp::movt;
        inst.rd = d.rd;
        inst.imm = d.imm;
        out = inst;
        return {true, MicroLowerReason::ok, {}};
    case InstrKind::add:
    case InstrKind::sub: {
        const bool is_add = d.kind == InstrKind::add;
        if (d.form == OperandForm::immediate) {
            if (!immediateShape(d) || !coreReg(d.rd) || !coreReg(d.rn)
                || d.immediate_carry_valid || d.immediate_carry) {
                return {false, MicroLowerReason::bad_shape, "alu imm shape"};
            }
            inst.op = is_add ? MicroOp::add_imm : MicroOp::sub_imm;
            inst.rd = d.rd;
            inst.rn = d.rn;
            inst.imm = d.imm;
            out = inst;
            return {true, MicroLowerReason::ok, {}};
        }
        if (!registerShape(d) || !coreReg(d.rd) || !coreReg(d.rn) || !coreReg(d.rm)) {
            return {false, MicroLowerReason::bad_shape, "alu reg shape"};
        }
        if (d.shift_amount == 0U && d.shift_type == ShiftType::lsl) {
            inst.op = is_add ? MicroOp::add_reg : MicroOp::sub_reg;
        } else {
            if (d.shift_type == ShiftType::rrx && d.shift_amount != 0U) {
                return {false, MicroLowerReason::bad_shape, "rrx amount must be 0/1"};
            }
            inst.op = is_add ? MicroOp::add_shift : MicroOp::sub_shift;
            inst.shift = d.shift_type;
            inst.shift_amount = d.shift_amount;
        }
        inst.rd = d.rd;
        inst.rn = d.rn;
        inst.rm = d.rm;
        out = inst;
        return {true, MicroLowerReason::ok, {}};
    }
    case InstrKind::cmp:
    case InstrKind::cmn: {
        const bool is_cmp = d.kind == InstrKind::cmp;
        if (!immediateShape(d) || !coreReg(d.rn) || !d.set_flags
            || d.immediate_carry_valid || d.immediate_carry) {
            return {false, MicroLowerReason::bad_shape, "cmp/cmn shape"};
        }
        inst.op = is_cmp ? MicroOp::cmp_imm : MicroOp::cmn_imm;
        inst.rd = d.rd;
        inst.rn = d.rn;
        inst.imm = d.imm;
        out = inst;
        return {true, MicroLowerReason::ok, {}};
    }
    case InstrKind::and_:
    case InstrKind::orr:
    case InstrKind::eor:
    case InstrKind::bic: {
        if (!registerShape(d) || d.shift_amount != 0U || d.shift_type != ShiftType::lsl
            || !coreReg(d.rd) || !coreReg(d.rn) || !coreReg(d.rm)) {
            return {false, MicroLowerReason::bad_shape, "logical shape"};
        }
        switch (d.kind) {
        case InstrKind::and_: inst.op = MicroOp::and_reg; break;
        case InstrKind::orr: inst.op = MicroOp::orr_reg; break;
        case InstrKind::eor: inst.op = MicroOp::eor_reg; break;
        default: inst.op = MicroOp::bic_reg; break;
        }
        inst.rd = d.rd;
        inst.rn = d.rn;
        inst.rm = d.rm;
        out = inst;
        return {true, MicroLowerReason::ok, {}};
    }
    case InstrKind::lsl:
    case InstrKind::lsr:
    case InstrKind::asr:
    case InstrKind::ror:
    case InstrKind::rrx: {
        // Executor: immediate form reads the value from rm with a compiled
        // amount; register form reads the value from rn with amount=rm&0xff.
        const bool immediate = d.form == OperandForm::immediate;
        if (immediate) {
            if (!coreReg(d.rd) || !coreReg(d.rm)) {
                return {false, MicroLowerReason::bad_register, "shift reg"};
            }
        } else {
            if (d.form != OperandForm::register_value || !coreReg(d.rd)
                || !coreReg(d.rn) || !coreReg(d.rm)) {
                return {false, MicroLowerReason::bad_shape, "shift reg form"};
            }
            // Register-specified amount with flags needs a data-dependent
            // carry sequence the stencil emitter does not cover yet; the
            // block stays on the cached fast path instead.
            if (d.set_flags) {
                return {false, MicroLowerReason::bad_shape, "register shift with flags"};
            }
        }
        switch (d.kind) {
        case InstrKind::lsl: inst.op = MicroOp::lsl; break;
        case InstrKind::lsr: inst.op = MicroOp::lsr; break;
        case InstrKind::asr: inst.op = MicroOp::asr; break;
        case InstrKind::ror: inst.op = MicroOp::ror; break;
        default: inst.op = MicroOp::rrx; break;
        }
        inst.rd = d.rd;
        inst.rn = d.rn;
        inst.rm = d.rm; // imm form reads value from rm; reg form value=rn, amount=rm&0xff.
        inst.shift_amount = d.shift_amount;
        inst.shift = d.shift_type;
        inst.shift_immediate = immediate;
        inst.imm = d.imm;
        out = inst;
        return {true, MicroLowerReason::ok, {}};
    }
    case InstrKind::b:
    case InstrKind::bl: {
        if ((d.form != OperandForm::none && d.form != OperandForm::immediate)
            || d.set_flags || d.rd != 0U || d.rn != 0U || d.rm != 0U || d.imm != 0U
            || d.immediate_carry_valid || d.immediate_carry) {
            return {false, MicroLowerReason::bad_shape, "branch shape"};
        }
        inst.op = d.kind == InstrKind::b ? MicroOp::b : MicroOp::bl;
        out = inst;
        return {true, MicroLowerReason::ok, {}};
    }
    default:
        return {false, MicroLowerReason::unsupported_kind, "kind not in FilIR subset"};
    }
}

MicroLowerResult buildMicroBlock(std::span<const MicroSource> sources, MicroBlock& out) noexcept {
    MicroBlock block;
    if (sources.empty()) {
        return {false, MicroLowerReason::bad_shape, "empty block"};
    }
    if (sources.size() > MicroBlock::max_ops) {
        return {false, MicroLowerReason::bad_shape, "block exceeds 16 ops"};
    }
    block.entry_pc = sources.front().pc;
    for (std::size_t i = 0; i < sources.size(); ++i) {
        MicroInst inst;
        MicroLowerResult r = tryLowerMicroInst(sources[i], inst);
        if (!r.ok) return r;
        const bool is_branch = inst.op == MicroOp::b || inst.op == MicroOp::bl;
        if (is_branch && i + 1U != sources.size()) {
            return {false, MicroLowerReason::branch_not_terminal, "branch must terminate"};
        }
        if (is_branch) block.ends_with_branch = true;
        block.ops.push_back(inst);
    }
    out = std::move(block);
    return {true, MicroLowerReason::ok, {}};
}

std::uint64_t microBlockCost(const MicroBlock& block) noexcept {
    std::uint64_t cost = 0;
    for (const auto& op : block.ops) {
        cost += (op.op == MicroOp::b || op.op == MicroOp::bl) ? 2U : 1U;
    }
    return cost;
}

std::size_t interpretMicroBlock(const MicroBlock& block, MicroCpu& cpu, std::size_t max_instructions) noexcept {
    if (max_instructions == 0U) return 0U;
    std::size_t retired = 0U;
    for (const auto& op : block.ops) {
        if (retired >= max_instructions) break;
        const bool old_c = (cpu.xpsr & kFlagC) != 0U;
        switch (op.op) {
        case MicroOp::nop: break;
        case MicroOp::mov_imm:
            writeOp(cpu, op.rd, op.imm);
            if (op.set_flags) {
                // Matches setNzc(value, immediate_carry_valid ? carry : old C).
                applyNz(cpu, op.imm, true);
                cpu.xpsr &= ~kFlagC;
                const bool c = op.carry_valid ? op.carry : old_c;
                if (c) cpu.xpsr |= kFlagC;
            }
            break;
        case MicroOp::movw: writeOp(cpu, op.rd, op.imm & 0xffffU); break;
        case MicroOp::movt:
            writeOp(cpu, op.rd, (readOp(cpu, op.rd) & 0xffffU) | ((op.imm & 0xffffU) << 16U));
            break;
        case MicroOp::add_imm: {
            AddResult r = addWithCarry(readOp(cpu, op.rn), op.imm, false);
            writeOp(cpu, op.rd, r.value);
            if (op.set_flags) applyNzcv(cpu, r);
            break;
        }
        case MicroOp::sub_imm: {
            AddResult r = addWithCarry(readOp(cpu, op.rn), ~op.imm, true);
            writeOp(cpu, op.rd, r.value);
            if (op.set_flags) applyNzcv(cpu, r);
            break;
        }
        case MicroOp::cmp_imm: {
            AddResult r = addWithCarry(readOp(cpu, op.rn), ~op.imm, true);
            applyNzcv(cpu, r);
            break;
        }
        case MicroOp::cmn_imm: {
            AddResult r = addWithCarry(readOp(cpu, op.rn), op.imm, false);
            applyNzcv(cpu, r);
            break;
        }
        case MicroOp::add_reg:
        case MicroOp::sub_reg:
        case MicroOp::and_reg:
        case MicroOp::orr_reg:
        case MicroOp::eor_reg:
        case MicroOp::bic_reg: {
            const std::uint32_t lhs = readOp(cpu, op.rn);
            const std::uint32_t rhs = readOp(cpu, op.rm);
            if (op.op == MicroOp::add_reg) {
                AddResult r = addWithCarry(lhs, rhs, false);
                writeOp(cpu, op.rd, r.value);
                if (op.set_flags) applyNzcv(cpu, r);
            } else if (op.op == MicroOp::sub_reg) {
                AddResult r = addWithCarry(lhs, ~rhs, true);
                writeOp(cpu, op.rd, r.value);
                if (op.set_flags) applyNzcv(cpu, r);
            } else {
                std::uint32_t v = rhs;
                if (op.op == MicroOp::and_reg) v = lhs & rhs;
                else if (op.op == MicroOp::orr_reg) v = lhs | rhs;
                else if (op.op == MicroOp::eor_reg) v = lhs ^ rhs;
                else v = lhs & ~rhs;
                writeOp(cpu, op.rd, v);
                if (op.set_flags) applyNz(cpu, v, true);
            }
            break;
        }
        case MicroOp::add_shift:
        case MicroOp::sub_shift: {
            ShiftResult s = shiftC(readOp(cpu, op.rm), op.shift, op.shift_amount, old_c);
            const std::uint32_t lhs = readOp(cpu, op.rn);
            AddResult r = op.op == MicroOp::add_shift
                ? addWithCarry(lhs, s.value, false)
                : addWithCarry(lhs, ~s.value, true);
            writeOp(cpu, op.rd, r.value);
            if (op.set_flags) applyNzcv(cpu, r);
            break;
        }
        case MicroOp::lsl:
        case MicroOp::lsr:
        case MicroOp::asr:
        case MicroOp::ror:
        case MicroOp::rrx: {
            ShiftType t = op.shift;
            std::uint32_t amount = op.shift_amount;
            std::uint32_t value = op.shift_immediate ? readOp(cpu, op.rm) : readOp(cpu, op.rn);
            if (op.op == MicroOp::lsl) { t = ShiftType::lsl; }
            else if (op.op == MicroOp::lsr) { t = ShiftType::lsr; }
            else if (op.op == MicroOp::asr) { t = ShiftType::asr; }
            else if (op.op == MicroOp::ror) { t = ShiftType::ror; }
            else { t = ShiftType::rrx; amount = 1U; }
            // Register-specified shift: amount from rm low byte (matches ARM).
            if (!op.shift_immediate) {
                amount = readOp(cpu, op.rm) & 0xffU;
            }
            ShiftResult s = shiftC(value, t, amount, old_c);
            writeOp(cpu, op.rd, s.value);
            if (op.set_flags) {
                applyNz(cpu, s.value, true);
                if (s.carry) cpu.xpsr |= kFlagC;
                else cpu.xpsr &= ~kFlagC;
            }
            break;
        }
        case MicroOp::b:
            // Matches branchTarget(): (instrAddr + 4 + offset) & ~1.
            cpu.pc = static_cast<std::uint32_t>(
                static_cast<std::int64_t>(op.pc) + 4 + op.branch_offset) & ~1U;
            ++retired;
            return retired;
        case MicroOp::bl:
            // Matches execute(): LR = (r15 at dispatch) | 1 = (pc + size) | 1.
            cpu.lr = (op.pc + op.size) | 1U;
            cpu.pc = static_cast<std::uint32_t>(
                static_cast<std::int64_t>(op.pc) + 4 + op.branch_offset) & ~1U;
            ++retired;
            return retired;
        }
        ++retired;
        cpu.pc = op.pc + op.size;
    }
    return retired;
}

} // namespace fil::cpu
