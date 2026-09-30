#include "fil/cpu/jit.hpp"

#include <algorithm>

namespace fil::cpu {

JitBoundary classifyJitBoundary(const DecodedInstruction& instruction) noexcept {
    switch (instruction.kind) {
    case InstrKind::mov:
        return instruction.rd == 15U ? JitBoundary::control_flow : JitBoundary::none;
    case InstrKind::add:
    case InstrKind::adc:
    case InstrKind::sub:
    case InstrKind::sbc:
        return instruction.rd == 15U ? JitBoundary::control_flow : JitBoundary::none;
    case InstrKind::movw:
    case InstrKind::movt:
        return instruction.rd == 15U ? JitBoundary::control_flow : JitBoundary::none;
    case InstrKind::rsb:
    case InstrKind::cmp:
    case InstrKind::cmn:
    case InstrKind::tst:
    case InstrKind::and_:
    case InstrKind::orr:
    case InstrKind::eor:
    case InstrKind::bic:
    case InstrKind::mvn:
    case InstrKind::orn:
    case InstrKind::mul:
    case InstrKind::mla:
    case InstrKind::mls:
    case InstrKind::umull:
    case InstrKind::smull:
    case InstrKind::udiv:
    case InstrKind::sdiv:
    case InstrKind::clz:
    case InstrKind::bfc:
    case InstrKind::ubfx:
    case InstrKind::sxtb:
    case InstrKind::sxth:
    case InstrKind::uxtb:
    case InstrKind::uxth:
    case InstrKind::rev:
    case InstrKind::rev16:
    case InstrKind::revsh:
    case InstrKind::uadd8:
    case InstrKind::sel:
    case InstrKind::lsl:
    case InstrKind::lsr:
    case InstrKind::asr:
    case InstrKind::ror:
    case InstrKind::rrx:
    case InstrKind::nop:
        return JitBoundary::none;

    case InstrKind::ldr:
    case InstrKind::str:
    case InstrKind::ldrb:
    case InstrKind::strb:
    case InstrKind::ldrh:
    case InstrKind::strh:
    case InstrKind::ldrsb:
    case InstrKind::ldrsh:
    case InstrKind::ldrd:
    case InstrKind::strd:
    case InstrKind::ldm:
    case InstrKind::stm:
    case InstrKind::push:
    case InstrKind::pop:
        // Loads/stores run inside a block with a pre-instruction checkpoint:
        // backed RAM stays in the block, MMIO trapping aborts precisely.
        // PC-writing forms terminate the block as control flow.
        if (instruction.rd == 15U) return JitBoundary::control_flow;
        if (instruction.kind == InstrKind::ldm || instruction.kind == InstrKind::pop) {
            if ((instruction.register_list & (std::uint16_t{1U} << 15U)) != 0U) {
                return JitBoundary::control_flow;
            }
        }
        return JitBoundary::memory_may_trap;

    case InstrKind::b:
    case InstrKind::bl:
    case InstrKind::bx:
    case InstrKind::blx:
    case InstrKind::cbz:
    case InstrKind::cbnz:
        return JitBoundary::control_flow;

    case InstrKind::it:
    case InstrKind::dmb:
    case InstrKind::dsb:
    case InstrKind::isb:
    case InstrKind::svc:
    case InstrKind::mrs:
    case InstrKind::msr:
    case InstrKind::cps:
    case InstrKind::wfi:
    case InstrKind::wfe:
    case InstrKind::sev:
    case InstrKind::bkpt:
        return JitBoundary::exception_or_system;

    case InstrKind::vstm:
    case InstrKind::vldm:
    case InstrKind::vldr:
    case InstrKind::vstr:
    case InstrKind::vmov_core_to_single:
    case InstrKind::vmov_single_to_core:
    case InstrKind::vmov_single:
    case InstrKind::vmov_immediate:
    case InstrKind::vcvt_f32_s32:
    case InstrKind::vcvt_f32_u32:
    case InstrKind::vcvt_s32_f32:
    case InstrKind::vcvt_u32_f32:
    case InstrKind::vadd:
    case InstrKind::vsub:
    case InstrKind::vmul:
    case InstrKind::vnmul:
    case InstrKind::vdiv:
    case InstrKind::vfma:
    case InstrKind::vfms:
    case InstrKind::vfnms:
    case InstrKind::vcmp:
    case InstrKind::vneg:
    case InstrKind::vabs:
    case InstrKind::vsqrt:
    case InstrKind::vmrs:
        return JitBoundary::floating_point;

    case InstrKind::undefined:
        return JitBoundary::unsupported;
    }
    return JitBoundary::unsupported;
}

JitBlockPlan planJitBlock(
    const std::span<const DecodedInstruction> instructions,
    const std::size_t maximum_instructions
) noexcept {
    JitBlockPlan plan;
    const std::size_t limit = std::min(instructions.size(), maximum_instructions);
    while (plan.translated_instructions < limit) {
        const JitBoundary boundary = classifyJitBoundary(
            instructions[plan.translated_instructions]
        );
        if (boundary == JitBoundary::control_flow) {
            // Terminating branch is included, then the block ends.
            ++plan.translated_instructions;
            plan.ends_with_branch = true;
            plan.boundary = JitBoundary::control_flow;
            return plan;
        }
        if (boundary == JitBoundary::memory_may_trap
            || boundary == JitBoundary::none) {
            ++plan.translated_instructions;
            continue;
        }
        plan.boundary = boundary;
        return plan;
    }
    return plan;
}

} // namespace fil::cpu
