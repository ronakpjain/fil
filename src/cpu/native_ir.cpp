#include "fil/cpu/native_ir.hpp"

#include <utility>

namespace fil::cpu {

MicroLowerResult tryLowerNativeInst(const MicroSource& source, NativeInst& out) noexcept {
    NativeInst inst;
    inst.pc = source.pc;
    inst.size = source.size;
    const auto& d = source.decoded;
    if (d.kind != InstrKind::ldr && d.kind != InstrKind::str) {
        const auto result = tryLowerMicroInst(source, inst.integer);
        if (!result.ok) return result;
        out = inst;
        return result;
    }
    if (d.condition != Condition::al)
        return {false, MicroLowerReason::bad_condition, "memory predication is not admitted"};
    if ((source.size != 2U && source.size != 4U) || d.is_32bit != (source.size == 4U))
        return {false, MicroLowerReason::bad_size, "invalid memory instruction width"};
    if ((source.pc & 1U) != 0U)
        return {false, MicroLowerReason::bad_pc, "thumb pc must be even"};
    if (d.rd >= 13U || d.rn >= 13U)
        return {false, MicroLowerReason::bad_register, "memory operands must be r0-r12"};
    if (d.form != OperandForm::immediate || !d.index || !d.add || d.writeback
        || d.set_flags || d.rm != 0U || d.ra != 0U || d.register_list != 0U
        || d.shift_amount != 0U || d.shift_type != ShiftType::lsl || d.fp_double
        || d.branch_offset != 0 || d.immediate_carry_valid || d.immediate_carry)
        return {false, MicroLowerReason::bad_shape, "unsupported memory addressing shape"};
    inst.kind = d.kind == InstrKind::ldr ? NativeOp::load_word : NativeOp::store_word;
    inst.rd = d.rd;
    inst.rn = d.rn;
    inst.offset = d.imm;
    out = inst;
    return {true, MicroLowerReason::ok, {}};
}

MicroLowerResult buildNativeBlock(std::span<const MicroSource> sources, NativeBlock& out) noexcept {
    if (sources.empty() || sources.size() > NativeBlock::max_ops)
        return {false, MicroLowerReason::bad_shape, "invalid native block length"};
    NativeBlock block;
    block.entry_pc = sources.front().pc;
    auto expected_pc = block.entry_pc;
    for (std::size_t i = 0U; i < sources.size(); ++i) {
        if (sources[i].pc != expected_pc)
            return {false, MicroLowerReason::bad_pc, "native block is not contiguous"};
        NativeInst inst;
        const auto result = tryLowerNativeInst(sources[i], inst);
        if (!result.ok) return result;
        const bool terminal = inst.kind == NativeOp::integer
            && (inst.integer.op == MicroOp::b || inst.integer.op == MicroOp::bl
                || inst.integer.op == MicroOp::b_cond || inst.integer.op == MicroOp::cbz
                || inst.integer.op == MicroOp::cbnz);
        if (terminal && i + 1U != sources.size())
            return {false, MicroLowerReason::branch_not_terminal, "branch must terminate"};
        block.ends_with_branch = terminal;
        block.has_memory |= inst.kind != NativeOp::integer;
        block.ops.push_back(inst);
        expected_pc += inst.size;
    }
    out = std::move(block);
    return {true, MicroLowerReason::ok, {}};
}

} // namespace fil::cpu
