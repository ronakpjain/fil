#include "fil/cpu/native_stack.hpp"
#include <array>
#include <cstring>
#include <optional>

namespace fil::cpu {
NativeStackResult nativeStackTransfer(NativeExecutionContext* c, std::uint32_t pc,
    std::uint32_t mask, std::uint32_t push, std::uint32_t expected_pc) noexcept {
    const NativeStackPlan plan(mask, push != 0U);
    return nativeStackTransferPrepared(c, pc, plan, expected_pc);
}

NativeStackResult nativePushTransferPrepared(NativeExecutionContext* c, std::uint32_t pc,
    const NativeStackPlan& plan, std::uint32_t) noexcept {
    if (!c || !c->cpu_state || !c->memory_bus || !plan.push() || plan.count() == 0U)
        return NativeStackResult::memory_declined;
    CpuState& s = *c->cpu_state;
    auto& banked_sp = ((s.xpsr & xpsr_ipsr_mask) != 0U || (s.control & 2U) == 0U) ? s.msp : s.psp;
    const auto count = plan.count();
    const auto bytes = count * 4U;
    if (banked_sp < bytes) return NativeStackResult::memory_declined;
    const auto start = banked_sp - bytes;
    // Only the transferred entries are initialized; checked backing still
    // owns full-span permissions, exact byte journaling and store footprints.
    std::array<std::uint32_t, 16> values;
    for (std::uint32_t n = 0U; n < count; ++n) values[n] = s.r[plan.reg(n)];
    if (!c->memory_bus->tryFastWriteWords(start, count, values.data(), {mem::AccessType::data_write, pc}, true))
        return NativeStackResult::memory_declined;
    banked_sp = start; s.r[13] = start;
    return NativeStackResult::success;
}

NativeStackResult nativeStackTransferPrepared(NativeExecutionContext* c, std::uint32_t pc,
    const NativeStackPlan& plan, std::uint32_t expected_pc) noexcept {
    if (plan.push()) return nativePushTransferPrepared(c, pc, plan, expected_pc);
    if (!c || !c->cpu_state || !c->memory_bus || plan.count() == 0U)
        return NativeStackResult::memory_declined;
    CpuState& s = *c->cpu_state;
    const bool loads_pc = plan.loadsPc();
    const std::uint32_t count = plan.count();
    const std::uint64_t bytes = static_cast<std::uint64_t>(count) * 4U;
    // Select the bank once for this RAM-only operation. Neither the transfer
    // nor its checked backing helpers can change handler mode or CONTROL.
    auto& banked_sp = ((s.xpsr & xpsr_ipsr_mask) != 0U || (s.control & 2U) == 0U) ? s.msp : s.psp;
    const std::uint32_t old_sp = banked_sp;
    const std::uint32_t start = old_sp;
    // PC, when present, is always the last bit in the register list, so its
    // slot is the final transferred word. Resolving it arithmetically keeps
    // the pop path to a single walk over the mask.
    std::uint32_t target = 0;
    bool alternate_return = false;
    std::uint32_t pc_slot = 0U;
    if (loads_pc) {
        pc_slot = count - 1U;
    }
    // The pre-read footprint copy exists only to undo the speculative read on
    // the two rejection paths below. Snapshotting it up front charged every
    // successful pop a 32-byte copy even though the snapshot is discarded on
    // success, so it is only taken for pops that load the PC.
    std::optional<mem::MemoryBus::ReadFootprint> footprint;
    if (loads_pc && c->memory_bus->readFootprintTracking())
        footprint = c->memory_bus->readFootprint();
    std::array<std::uint32_t, 16> values;
    // Reuse the trace's direct-read window when it covers the whole frame: the
    // window is only published with read-footprint tracking off. Validate the
    // return target before committing any registers, then load directly from
    // the window instead of copying the frame into a temporary and back.
    const std::uint8_t* frame = nullptr;
    if (c->fast_read_base != nullptr) {
        const std::uint64_t off = static_cast<std::uint64_t>(start) - c->fast_read_begin;
        if (off + bytes <= c->fast_read_size) {
            frame = c->fast_read_base + off;
        }
    }
    if (frame == nullptr) {
        if (!c->memory_bus->tryFastReadWords(start, count, values.data(), {mem::AccessType::data_read, pc}))
            return NativeStackResult::memory_declined;
        frame = reinterpret_cast<const std::uint8_t*>(values.data());
    }
    if (loads_pc) {
        std::memcpy(&target, frame + pc_slot * 4U, sizeof(target));
        if ((target & 0xfffffff0U) == 0xfffffff0U || (target & 1U) == 0U) {
            if (footprint) c->memory_bus->restoreReadFootprint(*footprint);
            return NativeStackResult::memory_declined;
        }
        alternate_return = (target & ~1U) != expected_pc;
        if (alternate_return && !c->allow_dynamic_returns) {
            if (footprint) c->memory_bus->restoreReadFootprint(*footprint);
            return NativeStackResult::branch_mismatch;
        }
    }
    const auto ordinary_count = count - (loads_pc ? 1U : 0U);
    for (std::uint32_t n = 0U; n < ordinary_count; ++n) {
        std::uint32_t value;
        std::memcpy(&value, frame + n * 4U, sizeof(value));
        s.r[plan.reg(n)] = value;
    }
    banked_sp = old_sp + static_cast<std::uint32_t>(bytes); s.r[13] = banked_sp;
    if (loads_pc) s.r[15] = target & ~1U;
    return alternate_return ? NativeStackResult::dynamic_return : NativeStackResult::success;
}
}
