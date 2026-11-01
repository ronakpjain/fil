#include "fil/cpu/native_stack.hpp"
#include <array>
#include <bit>
#include <optional>

namespace fil::cpu {
NativeStackResult nativeStackTransfer(NativeExecutionContext* c, std::uint32_t pc,
    std::uint32_t mask, std::uint32_t push, std::uint32_t expected_pc) noexcept {
    if (!c || !c->cpu_state || !c->memory_bus || mask == 0
        || (mask & 0xffff0000U) != 0U || (mask & (1U << 13U)))
        return NativeStackResult::memory_declined;
    CpuState& s = *c->cpu_state;
    const bool is_push = push != 0;
    const bool loads_pc = !is_push && (mask & 0x8000U);
    if (is_push && (mask & 0x8000U)) return NativeStackResult::memory_declined;
    if (loads_pc && (mask & 0x4000U)) return NativeStackResult::memory_declined;
    const std::uint32_t count = static_cast<std::uint32_t>(std::popcount(mask));
    const std::uint64_t bytes = static_cast<std::uint64_t>(count) * 4U;
    const std::uint32_t old_sp = s.readRegister(13U);
    const std::uint32_t start = is_push ? old_sp - static_cast<std::uint32_t>(bytes) : old_sp;
    if (is_push) {
        if (old_sp < bytes) return NativeStackResult::memory_declined;
        // Only the first `count` entries are written and read; clearing all
        // sixteen would cost a 64-byte store per transferred stack frame.
        std::array<std::uint32_t, 16> values;
        std::uint32_t n = 0;
        for (std::uint32_t bits = mask; bits != 0U; bits &= bits - 1U) {
            const auto r = static_cast<std::uint32_t>(std::countr_zero(bits));
            values[n++] = s.r[r];
        }
        if (!c->memory_bus->tryFastWriteWords(start, count, values.data(), {mem::AccessType::data_write, pc}, true))
            return NativeStackResult::memory_declined;
        s.writeRegister(13U, start);
        return NativeStackResult::success;
    }
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
    if (loads_pc) footprint = c->memory_bus->readFootprint();
    std::array<std::uint32_t, 16> values;
    if (!c->memory_bus->tryFastReadWords(start, count, values.data(), {mem::AccessType::data_read, pc}))
        return NativeStackResult::memory_declined;
    if (loads_pc) {
        target = values[pc_slot];
        if ((target & 0xfffffff0U) == 0xfffffff0U || (target & 1U) == 0U) {
            c->memory_bus->restoreReadFootprint(*footprint);
            return NativeStackResult::memory_declined;
        }
        alternate_return = (target & ~1U) != expected_pc;
        if (alternate_return && !c->allow_dynamic_returns) {
            c->memory_bus->restoreReadFootprint(*footprint);
            return NativeStackResult::branch_mismatch;
        }
    }
    std::uint32_t n = 0;
    for (std::uint32_t bits = mask; bits != 0U; bits &= bits - 1U) {
        const auto r = static_cast<std::uint32_t>(std::countr_zero(bits));
        if (r != 15U) s.r[r] = values[n];
        ++n;
    }
    s.writeRegister(13U, old_sp + static_cast<std::uint32_t>(bytes));
    if (loads_pc) s.r[15] = target & ~1U;
    return alternate_return ? NativeStackResult::dynamic_return : NativeStackResult::success;
}
}
