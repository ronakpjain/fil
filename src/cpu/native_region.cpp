#include "fil/cpu/native_region.hpp"

#include "fil/cpu/cortex_m4.hpp"
#include "fil/cpu/decoder.hpp"

#include <utility>

namespace fil::cpu {

namespace {
bool buildRegion(const std::span<const std::uint8_t> bytes,
    const std::uint32_t base, const std::uint32_t entry_pc,
    NativeRegionPlan& out, std::string& error, const std::uint32_t window_bytes) {
    constexpr auto address_space = std::uint64_t{1U} << 32U;
    if (bytes.size() > address_space - base) {
        error = "native-region: executable span wraps target address space";
        return false;
    }
    const auto end = static_cast<std::uint64_t>(base) + bytes.size();
    const auto contains = [&](std::uint32_t pc, std::uint32_t width) {
        return pc >= base && static_cast<std::uint64_t>(pc) + width <= end;
    };
    if ((entry_pc & 1U) != 0U || !contains(entry_pc, 2U)) {
        error = "native-region: invalid executable span or entry";
        return false;
    }
    if (window_bytes != 0U && ((window_bytes & 1U) != 0U
        || window_bytes > NativeRegionPlan::max_ops * 2U || !contains(entry_pc, window_bytes))) {
        error = "native-region: invalid code window";
        return false;
    }
    NativeRegionPlan plan;
    plan.entry_pc = entry_pc;
    plan.ops[0].pc = entry_pc;
    plan.count = 1U;
    if (window_bytes != 0U) {
        plan.count = static_cast<std::uint16_t>(window_bytes / 2U);
        for (std::uint16_t i = 0U; i < plan.count; ++i)
            plan.ops[i].pc = entry_pc + i * 2U;
    }
    const auto link = [&](std::uint32_t pc) -> std::uint16_t {
        if ((pc & 1U) != 0U || !contains(pc, 2U)) return NativeRegionOp::external;
        if (window_bytes != 0U) {
            if (pc < entry_pc || pc - entry_pc >= window_bytes) return NativeRegionOp::external;
            return static_cast<std::uint16_t>((pc - entry_pc) / 2U);
        }
        for (std::uint16_t i = 0U; i < plan.count; ++i)
            if (plan.ops[i].pc == pc) return i;
        if (plan.count == plan.ops.size()) {
            plan.capacity_limited = true;
            return NativeRegionOp::external;
        }
        const auto index = plan.count++;
        plan.ops[index].pc = pc;
        return index;
    };
    const auto halfword = [&](std::uint32_t pc) {
        const auto offset = static_cast<std::size_t>(pc - base);
        return static_cast<std::uint16_t>(bytes[offset] | (bytes[offset + 1U] << 8U));
    };
    for (std::uint16_t i = 0U; i < plan.count; ++i) {
        auto& op = plan.ops[i];
        const auto first = halfword(op.pc);
        const bool wide = is32BitThumbPrefix(first);
        op.size = wide ? 4U : 2U;
        op.raw = wide ? static_cast<std::uint32_t>(first) << 16U : first;
        if (!contains(op.pc, op.size)) continue; // checked wide-fetch fault
        const auto second = wide ? halfword(op.pc + 2U) : std::uint16_t{0U};
        if (wide) op.raw |= second;
        const auto decoded = wide ? decode32(first, second) : decode16(first);
        if (!decoded) continue;
        const bool stack = decoded->kind == InstrKind::push || decoded->kind == InstrKind::pop;
        if (stack) {
            op.stack = NativeStackPlan(decoded->register_list, decoded->kind == InstrKind::push);
            if (op.stack.count() == 0U) continue;
        } else if (!tryLowerNativeInst({*decoded, op.pc, op.size}, op.instruction).ok) continue;
        op.fallthrough = op.pc + op.size;
        op.fallthrough_cycles = basePipelineCycles(*decoded);
        op.flow = RegionFlow::next;
        const bool compare = decoded->kind == InstrKind::cbz || decoded->kind == InstrKind::cbnz;
        if (compare || decoded->kind == InstrKind::b || decoded->kind == InstrKind::bl) {
            op.target = static_cast<std::uint32_t>(static_cast<std::int64_t>(op.pc) + 4
                + (compare ? static_cast<std::int64_t>(decoded->imm) : decoded->branch_offset)) & ~1U;
            op.taken_cycles = static_cast<std::uint16_t>(basePipelineCycles(*decoded)
                + (op.target == op.fallthrough ? 0U : takenBranchPenalty(decoded->kind)));
            if (compare || (decoded->kind == InstrKind::b && decoded->condition != Condition::al)) {
                op.flow = RegionFlow::conditional;
                op.fallthrough_cycles = 1U;
                op.edges[0] = link(op.fallthrough);
                op.edges[1] = link(op.target);
            } else {
                op.flow = decoded->kind == InstrKind::bl ? RegionFlow::call : RegionFlow::jump;
                op.edges[0] = link(op.target);
                if (op.flow == RegionFlow::call) op.edges[1] = link(op.fallthrough);
            }
        } else if (op.stack.loadsPc() || op.instruction.kind == NativeOp::bx_lr
                   || op.instruction.kind == NativeOp::blx_reg) {
            op.flow = RegionFlow::indirect;
            op.taken_cycles = static_cast<std::uint16_t>(op.fallthrough_cycles
                + takenBranchPenalty(decoded->kind));
            if (op.instruction.kind == NativeOp::blx_reg) op.edges[1] = link(op.fallthrough);
        } else {
            op.edges[0] = link(op.fallthrough);
        }
    }
    out = std::move(plan);
    error.clear();
    return true;
}

} // namespace

bool buildNativeRegion(const std::span<const std::uint8_t> bytes,
    const std::uint32_t base, const std::uint32_t entry_pc,
    NativeRegionPlan& out, std::string& error) {
    return buildRegion(bytes, base, entry_pc, out, error, 0U);
}

bool buildNativeRegionWindow(const std::span<const std::uint8_t> bytes,
    const std::uint32_t base, const std::uint32_t window_pc,
    const std::uint32_t window_bytes, NativeRegionPlan& out, std::string& error) {
    if (window_bytes == 0U) {
        error = "native-region: empty code window";
        return false;
    }
    return buildRegion(bytes, base, window_pc, out, error, window_bytes);
}

bool nativeRegionEntryValid(NativeRegionContext* context) noexcept {
    if (context == nullptr) return false;
    context->retired_count = 0U;
    context->retired_cycles = 0U;
    const auto* state = context->cpu_state;
    const auto read_valid = [](const mem::MemoryBus::DirectReadWindow& view) {
        return view.size == 0U || view.bytes != nullptr;
    };
    const auto& write = context->ram_write;
    const bool bindings_valid = read_valid(context->ram_read) && read_valid(context->code_read)
        && (write.size == 0U || (write.bytes != nullptr && write.mutation_sequence != nullptr
            && write.side_effect_generation != nullptr));
    const bool valid = bindings_valid && state != nullptr && (state->r[15] & 1U) == 0U && !state->halted
        && state->thumb && (state->xpsr & xpsr_t) != 0U && state->it_state == 0U
        && !state->pending_exception && !state->pending_exc_return;
    context->exit_reason = valid ? NativeRegionExit::none : NativeRegionExit::invalid_state;
    return valid;
}

} // namespace fil::cpu
