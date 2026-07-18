#pragma once

/** @file jit.hpp
 *  @brief Conservative instruction classification for native block translation.
 */

#include "fil/cpu/instruction.hpp"

#include <cstddef>
#include <span>

namespace fil::cpu {

/** Reason a decoded instruction terminates or rejects a native straight-line block. */
enum class JitBoundary {
    none,
    memory,
    control_flow,
    exception_or_system,
    floating_point,
    unsupported,
};

/**
 * @brief Classifies whether an instruction can execute inside a native pure-CPU block.
 *
 * `none` means the instruction touches only integer CPU state and can be translated
 * without calling memory, peripherals, exception machinery, or host FP. Every other
 * value is a mandatory block exit in the initial JIT tier.
 */
[[nodiscard]] JitBoundary classifyJitBoundary(const DecodedInstruction& instruction) noexcept;

/** Conservative straight-line prefix selected for initial native translation. */
struct JitBlockPlan {
    std::size_t translated_instructions{0};
    JitBoundary boundary{JitBoundary::none};
};

/** @brief Selects the maximal pure-integer prefix before a mandatory JIT exit. */
[[nodiscard]] JitBlockPlan planJitBlock(
    std::span<const DecodedInstruction> instructions,
    std::size_t maximum_instructions = 64U
) noexcept;

} // namespace fil::cpu
