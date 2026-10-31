#include "fil/cpu/native_stack.hpp"
#include <array>

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
    std::uint32_t count = 0;
    for (std::uint32_t bits = mask; bits != 0; bits &= bits - 1U) ++count;
    const std::uint64_t bytes = static_cast<std::uint64_t>(count) * 4U;
    const std::uint32_t old_sp = s.readRegister(13U);
    const std::uint32_t start = is_push ? old_sp - static_cast<std::uint32_t>(bytes) : old_sp;
    if (is_push) {
        if (old_sp < bytes || static_cast<std::uint64_t>(start) + bytes > (std::uint64_t{1} << 32U)
            || !c->memory_bus->isReversibleRamTarget(start)
            || !c->memory_bus->containsWritableRamRange(start, bytes)) return NativeStackResult::memory_declined;
        // Only the first `count` entries are written and read; clearing all
        // sixteen would cost a 64-byte store per transferred stack frame.
        std::array<std::uint32_t, 16> values;
        std::uint32_t n = 0;
        for (std::uint32_t r = 0; r < 16; ++r) if (mask & (1U << r)) values[n++] = s.readRegister(static_cast<std::uint8_t>(r));
        if (!c->memory_bus->tryFastWriteWords(start, count, values.data(), {mem::AccessType::data_write, pc}))
            return NativeStackResult::memory_declined;
        s.writeRegister(13U, start);
        return NativeStackResult::success;
    }
    const auto footprint = c->memory_bus->readFootprint();
    std::array<std::uint32_t, 16> values;
    if (!c->memory_bus->tryFastReadWords(start, count, values.data(), {mem::AccessType::data_read, pc}))
        return NativeStackResult::memory_declined;
    std::uint32_t n = 0;
    std::uint32_t target = 0;
    for (std::uint32_t r = 0; r < 16; ++r) if (mask & (1U << r)) {
        if (r == 15U) target = values[n];
        ++n;
    }
    if (loads_pc) {
        if ((target & 0xfffffff0U) == 0xfffffff0U || (target & 1U) == 0U) {
            c->memory_bus->restoreReadFootprint(footprint);
            return NativeStackResult::memory_declined;
        }
        if ((target & ~1U) != expected_pc) {
            c->memory_bus->restoreReadFootprint(footprint);
            return NativeStackResult::branch_mismatch;
        }
    }
    n = 0;
    for (std::uint32_t r = 0; r < 16; ++r) if (mask & (1U << r)) {
        if (r != 15U) s.writeRegister(static_cast<std::uint8_t>(r), values[n]);
        ++n;
    }
    s.writeRegister(13U, old_sp + static_cast<std::uint32_t>(bytes));
    if (loads_pc) s.r[15] = expected_pc;
    return NativeStackResult::success;
}
}
