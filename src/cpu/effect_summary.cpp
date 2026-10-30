#include "fil/cpu/effect_summary.hpp"

#include <bit>
#include <cstdint>
#include <limits>

namespace fil::cpu {

bool PrivateMemoryFacts::contains(
    const std::uint32_t address, const std::uint32_t size
) const noexcept {
    if (size == 0U) return true;
    const std::uint64_t end = static_cast<std::uint64_t>(address) + size;
    if (end > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()) + 1U) {
        return false;
    }
    for (const auto& region : regions) {
        if (region.end <= region.begin) continue;
        if (address >= region.begin && end <= region.end) return true;
    }
    return false;
}

bool PrivateMemoryFacts::containsStack(
    const std::uint32_t address, const std::uint32_t size
) const noexcept {
    return contains(address, size);
}

bool isControlFlowMicroOp(const MicroOp op) noexcept {
    switch (op) {
    case MicroOp::b:
    case MicroOp::bl:
    case MicroOp::b_cond:
    case MicroOp::cbz:
    case MicroOp::cbnz:
        return true;
    default:
        return false;
    }
}

namespace {

[[nodiscard]] std::uint32_t stackWordCount(const std::uint16_t register_list) noexcept {
    return static_cast<std::uint32_t>(std::popcount(register_list));
}

} // namespace

EffectSummary summarizeNativeEffects(
    const std::span<const NativeTraceOp> ops,
    const PrivateMemoryFacts& facts
) noexcept {
    EffectSummary summary;
    std::uint64_t cycles = 0U;
    std::optional<std::uint32_t> stack = facts.stack_pointer;

    for (std::size_t i = 0; i < ops.size(); ++i) {
        const NativeTraceOp& op = ops[i];
        const std::uint64_t cost = op.execution_cycles == 0U
            ? 1U : op.execution_cycles;
        bool local = true;
        bool control_flow = false;

        const InstrKind decoded_kind = op.decoded.kind;
        const bool stack_op = decoded_kind == InstrKind::push
            || decoded_kind == InstrKind::pop;
        if (stack_op) {
            const std::uint32_t words = stackWordCount(op.decoded.register_list);
            const std::uint32_t bytes = words * 4U;
            if (!stack || bytes == 0U) {
                local = false;
            } else if (decoded_kind == InstrKind::push) {
                const std::uint32_t start = *stack - bytes;
                local = *stack >= bytes && facts.containsStack(start, bytes);
                if (local) *stack = start;
            } else {
                local = facts.containsStack(*stack, bytes);
                if (local) *stack = *stack + bytes;
            }
            control_flow = decoded_kind == InstrKind::pop
                && (op.decoded.register_list & 0x8000U) != 0U;
        } else if (op.native.kind == NativeOp::integer) {
            control_flow = isControlFlowMicroOp(op.native.integer.op);
        } else if (op.native.kind == NativeOp::load_word
                   || op.native.kind == NativeOp::store_word) {
            std::uint32_t address = op.native.absolute_address;
            if (!op.native.absolute) {
                if (op.native.rn == 13U && stack) {
                    address = *stack + op.native.offset;
                } else {
                    local = false;
                }
            }
            if (local) local = facts.contains(address, 4U);
        } else {
            local = false;
        }

        if (!local) {
            summary.reaches_shared = true;
            break;
        }
        if (control_flow && i + 1U < ops.size()) {
            // A branch may leave the summarized span. The branch itself has
            // no shared effect, but the operation after it is not proved to be
            // reachable only through the local prefix, so the provable window
            // ends before the branch.
            summary.reaches_shared = true;
            summary.unknown_control_flow = true;
            break;
        }
        cycles += cost;
        ++summary.local_ops;
        if (control_flow) summary.unknown_control_flow = true;
    }
    summary.minimum_cycles = cycles;
    return summary;
}

} // namespace fil::cpu
