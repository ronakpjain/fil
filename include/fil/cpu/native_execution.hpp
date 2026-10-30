#pragma once

#include "fil/cpu/cortex_m4.hpp"
#include "fil/mem/memory_bus.hpp"

#include <cstdint>

namespace fil::cpu {

/** @brief Stable exit classifications for a native reversible execution window. */
enum class NativeExitReason : std::uint32_t {
    none = 0,
    instruction_limit,
    cycle_limit,
    helper_declined,
    branch_mismatch,
    control_flow,
    unsupported_instruction,
};

/**
 * @brief Fixed-layout state shared with native reversible code.
 *
 * This is deliberately separate from MicroFunction's CpuState-only ABI. Native
 * emitters receive a pointer to this context; pointers are borrowed for the
 * duration of execution. Limits are admission ceilings, retired_count is the
 * committed prefix, and current/failing_instruction_pc identify the guest
 * instruction associated with progress/decline. Cycle fields are CPU cycles
 * only (no peripheral timing). Emitters must publish exit_reason and progress
 * before returning; helpers never update CPU state or context.
 */
struct NativeExecutionContext {
    CpuState* cpu_state{nullptr};
    mem::MemoryBus* memory_bus{nullptr};
    std::uint32_t instruction_limit{0};
    std::uint64_t cycle_limit{0};
    std::uint32_t retired_count{0};
    std::uint32_t current_instruction_pc{0};
    std::uint32_t failing_instruction_pc{0};
    NativeExitReason exit_reason{NativeExitReason::none};
    std::uint64_t fetch_cycles{0};
    std::uint64_t retired_cycles{0};
    /** Caller-precomputed total CPU cycles per candidate op, including fetch timing. */
    const std::uint32_t* charged_cycles{nullptr};
    std::uint32_t charged_cycles_count{0};
};

/** @brief Nonthrowing scalar ABI: false declines without changing output/state. */
[[nodiscard]] bool nativeRead32(
    NativeExecutionContext* context, std::uint32_t instruction_pc,
    std::uint32_t address, std::uint32_t* value_out) noexcept;
/**
 * @brief Reversible-only nonthrowing word store; only non-executable RAM is
 * accepted, irrespective of the bus's ambient reversible-only setting.
 */
[[nodiscard]] bool nativeWrite32(
    NativeExecutionContext* context, std::uint32_t instruction_pc,
    std::uint32_t address, std::uint32_t value) noexcept;
/** @brief Nonthrowing byte load ABI; false declines without changing output/state. */
[[nodiscard]] bool nativeRead8(
    NativeExecutionContext* context, std::uint32_t instruction_pc,
    std::uint32_t address, std::uint32_t* value_out) noexcept;
/** @brief Reversible-only nonthrowing byte store; only non-executable RAM is accepted. */
[[nodiscard]] bool nativeWrite8(
    NativeExecutionContext* context, std::uint32_t instruction_pc,
    std::uint32_t address, std::uint32_t value) noexcept;
/** @brief Nonthrowing halfword load ABI; false declines without changing output/state. */
[[nodiscard]] bool nativeRead16(
    NativeExecutionContext* context, std::uint32_t instruction_pc,
    std::uint32_t address, std::uint32_t* value_out) noexcept;
/** @brief Reversible-only nonthrowing halfword store; only non-executable RAM is accepted. */
[[nodiscard]] bool nativeWrite16(
    NativeExecutionContext* context, std::uint32_t instruction_pc,
    std::uint32_t address, std::uint32_t value) noexcept;

} // namespace fil::cpu
