#pragma once

/** @file jit.hpp
 *  @brief Conservative hot-path block classification for the CPU JIT.
 *
 *  The JIT auto-compiles hot straight-line paths: pure ALU plus RAM-backed
 *  loads/stores plus a single terminating branch. Memory/MMIO dispatch,
 *  exception/system, and floating-point boundaries stay exact by falling
 *  back to the interpreter at runtime (MMIO trapping, faults, pending
 *  exceptions abort the block with precise partial counts).
 */

#include "fil/cpu/instruction.hpp"

#include <cstddef>
#include <span>

namespace fil::cpu {

/** Reason a decoded instruction terminates or rejects a cached block. */
enum class JitBoundary {
    none,
    memory_may_trap,
    control_flow,
    exception_or_system,
    floating_point,
    unsupported,
};

/**
 * @brief Classifies whether an instruction can execute inside a JIT block.
 *
 * `none` means the instruction touches only integer CPU state. Memory and
 * control-flow instructions are allowed inside a block but force block
 * structure rules (memory needs a pre-instruction checkpoint for precise
 * MMIO restart; control flow must terminate the block). Exception/system
 * and floating-point instructions terminate compilation and stay interpreted
 * until dedicated equivalence tests exist.
 */
[[nodiscard]] JitBoundary classifyJitBoundary(const DecodedInstruction& instruction) noexcept;

/** Conservative straight-line prefix selected for JIT compilation. */
struct JitBlockPlan {
    std::size_t translated_instructions{0};
    JitBoundary boundary{JitBoundary::none};
    bool ends_with_branch{false};
};

/** @brief Selects the maximal JIT-able prefix before a mandatory exit. */
[[nodiscard]] JitBlockPlan planJitBlock(
    std::span<const DecodedInstruction> instructions,
    std::size_t maximum_instructions = 16U
) noexcept;

} // namespace fil::cpu
