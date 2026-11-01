#include "fil/cpu/native_trace.hpp"
#include "fil/cpu/decoder.hpp"

#include <utility>

namespace fil::cpu {

bool buildNativeTrace(std::span<const NativeTraceOp> sources, NativeTracePlan& out,
                      std::string& error) {
    if (sources.empty() || sources.size() > NativeTracePlan::max_ops) {
        error = "native-trace: invalid trace length";
        return false;
    }
    NativeTracePlan candidate;
    candidate.ops.reserve(sources.size());
    for (std::size_t i = 0; i < sources.size(); ++i) {
        const auto& src = sources[i];
        if ((src.size != 2U && src.size != 4U) || (src.pc & 1U) != 0U
            || src.decoded.is_32bit != (src.size == 4U)
            || src.decoded.raw != src.raw) {
            error = "native-trace: malformed width, alignment, or cycle cost";
            return false;
        }
        if (src.size == 2U && (src.raw & 0xffff0000U) != 0U) {
            error = "native-trace: malformed 16-bit raw encoding";
            return false;
        }
        MicroSource source{src.decoded, src.pc, src.size};
        NativeInst lowered;
        const bool stack = src.decoded.kind == InstrKind::push || src.decoded.kind == InstrKind::pop;
        if (stack) {
            const auto canonical = src.size == 2U ? decode16(static_cast<std::uint16_t>(src.raw))
                                                    : std::optional<DecodedInstruction>{};
            if (!canonical || canonical->kind != src.decoded.kind
                || canonical->form != src.decoded.form
                || canonical->register_list != src.decoded.register_list
                || canonical->condition != src.decoded.condition
                || canonical->rd != src.decoded.rd || canonical->rn != src.decoded.rn
                || canonical->rm != src.decoded.rm || canonical->ra != src.decoded.ra
                || canonical->imm != src.decoded.imm || canonical->branch_offset != src.decoded.branch_offset
                || canonical->shift_type != src.decoded.shift_type || canonical->shift_amount != src.decoded.shift_amount
                || canonical->set_flags != src.decoded.set_flags || canonical->index != src.decoded.index
                || canonical->add != src.decoded.add || canonical->writeback != src.decoded.writeback
                || canonical->immediate_carry_valid != src.decoded.immediate_carry_valid
                || canonical->immediate_carry != src.decoded.immediate_carry
                || canonical->fp_double != src.decoded.fp_double
                || src.size != 2U || src.decoded.form != OperandForm::register_list
                || src.decoded.register_list == 0U || (src.decoded.register_list & (1U << 13U))
                || (src.decoded.kind == InstrKind::push && (src.decoded.register_list & (1U << 15U)))
                || (src.decoded.kind == InstrKind::pop && (src.decoded.register_list & 0xc000U) == 0xc000U)) {
                error = "native-trace: invalid stack register list"; return false;
            }
            lowered = NativeInst{};
            lowered.kind = NativeOp::integer;
        } else {
            auto result = tryLowerNativeInst(source, lowered);
            if (!result.ok) { error = "native-trace: " + result.detail; return false; }
        }
        const auto op = lowered.integer.op;
        const bool bx_lr = lowered.kind == NativeOp::bx_lr;
        const bool blx_reg = lowered.kind == NativeOp::blx_reg;
        const bool compare_branch = lowered.kind == NativeOp::integer
            && (op == MicroOp::cbz || op == MicroOp::cbnz);
        const bool branch = bx_lr || blx_reg || compare_branch || (lowered.kind == NativeOp::integer
            && (op == MicroOp::b || op == MicroOp::b_cond || op == MicroOp::bl));
        // Extend and bitfield-extract sit after the contiguous shift/branch
        // range in the enum but are fully emitted; admit them explicitly.
        const bool extra_integer = lowered.kind == NativeOp::integer
            && (op == MicroOp::extend || op == MicroOp::ubfx);
        if (!blx_reg && !compare_branch && !(op <= MicroOp::bl || op == MicroOp::b_cond || extra_integer)) {
            error = "native-trace: unsupported operation";
            return false;
        }
        const std::uint32_t sequential = src.pc + src.size;
        const std::uint32_t target = static_cast<std::uint32_t>(
            static_cast<std::int64_t>(src.pc) + 4
            + (compare_branch ? static_cast<std::int64_t>(lowered.integer.imm)
                              : lowered.integer.branch_offset)) & ~1U;
        if (bx_lr) {
            const auto canonical = src.size == 2U ? decode16(static_cast<std::uint16_t>(src.raw))
                                                   : std::optional<DecodedInstruction>{};
            if (!canonical || canonical->kind != InstrKind::bx || canonical->rm != 14U
                || !src.expected_taken || (src.expected_next_pc & 1U) != 0U
                || (src.expected_next_pc & 0xfffffff0U) == 0xfffffff0U) {
                error = "native-trace: invalid BX LR edge";
                return false;
            }
        } else if (blx_reg) {
            if (src.size != 2U || src.decoded.kind != InstrKind::blx
                || src.decoded.condition != Condition::al || !src.expected_taken
                || (src.expected_next_pc & 1U) != 0U
                || (src.expected_next_pc & 0xfffffff0U) == 0xfffffff0U) {
                error = "native-trace: invalid BLX register edge";
                return false;
            }
        } else if (op == MicroOp::bl) {
            // Direct BL is permitted, and always takes its direct target.
            if (src.expected_next_pc != target || !src.expected_taken) {
                error = "native-trace: BL edge mismatch";
                return false;
            }
        } else if (op == MicroOp::b) {
            if (src.decoded.condition != Condition::al || src.expected_next_pc != target
                || !src.expected_taken) {
                error = "native-trace: invalid unconditional branch edge";
                return false;
            }
        } else if (compare_branch) {
            if (src.expected_next_pc != (src.expected_taken ? target : sequential)) {
                error = "native-trace: compare branch edge mismatch";
                return false;
            }
        } else if (op == MicroOp::b_cond) {
            const auto wanted = src.expected_taken ? target : sequential;
            if (src.expected_next_pc != wanted || src.decoded.condition == Condition::al
                || src.decoded.condition == Condition::nv) {
                error = "native-trace: conditional branch edge mismatch";
                return false;
            }
        } else if (stack && src.decoded.kind == InstrKind::pop
                   && (src.decoded.register_list & 0x8000U)) {
            if (!src.expected_taken || (src.expected_next_pc & 1U) != 0U) {
                error = "native-trace: invalid POP PC edge"; return false;
            }
        } else if (src.expected_taken || src.expected_next_pc != sequential) {
            error = "native-trace: sequential edge mismatch";
            return false;
        }
        if (i + 1U < sources.size() && src.expected_next_pc != sources[i + 1U].pc) {
            error = "native-trace: trace edge does not reach next operation";
            return false;
        }
        // Extend and bitfield-extract ops sit after the contiguous
        // shift/branch range in the enum but are fully emitted, so admit them
        // explicitly rather than treating enum order as the supported set.
        if (!branch && op != MicroOp::bl && lowered.integer.op > MicroOp::rrx
            && !extra_integer) {
            error = "native-trace: unsupported operation";
            return false;
        }
        candidate.ops.push_back(src);
        candidate.ops.back().native = lowered;
    }
    out = std::move(candidate);
    return true;
}

} // namespace fil::cpu
