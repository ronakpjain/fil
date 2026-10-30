#pragma once

/** @file effect_summary.hpp
 *  @brief Conservative, allocation-free effect summaries for compiled regions.
 *
 * A lane horizon is only useful when the compiler can prove that a region
 * cannot create or observe a shared effect before an announced boundary. The
 * summary below is deliberately conservative: a memory access is local only
 * when its complete byte range is backed by an explicitly supplied private
 * region (and, for SP-relative accesses, the supplied stack pointer). Unknown
 * bases, indirect control flow beyond the summarized span, and MMIO are
 * reported as potentially shared at the earliest possible operation.
 *
 * Summaries contain no rollback snapshot, journal, or replay certificate. They
 * describe lower bounds over all paths represented by the validated span; they
 * are not a proof that the guest will take a particular path.
 */

#include "fil/cpu/native_trace.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace fil::cpu {

/** @brief Half-open private byte range, usually directly backed RAM or ROM. */
struct PrivateRegion {
    std::uint32_t begin{0};
    std::uint32_t end{0};
};

/**
 * @brief Runtime facts that a caller has proved before summarizing a region.
 *
 * `regions` must contain every range that may be accessed without a shared
 * effect. `stack_pointer` is the architectural SP at the region entry and is
 * used only for SP-relative word and stack-multiple operations. Supplying a
 * stale or speculative SP makes the summary unsound; callers must establish it
 * from the same synchronized state that admits the region.
 */
struct PrivateMemoryFacts {
    std::span<const PrivateRegion> regions{};
    std::optional<std::uint32_t> stack_pointer{};

    [[nodiscard]] bool contains(std::uint32_t address, std::uint32_t size) const noexcept;
    [[nodiscard]] bool containsStack(std::uint32_t address, std::uint32_t size) const noexcept;
};

/**
 * @brief Conservative effects of a validated native span.
 *
 * `local_ops` is the length of the entry-anchored prefix whose effects are
 * proved private and whose control flow stays inside the summarized span.
 * `first_external_op()` is the first operation that may create or observe a
 * shared effect; it equals `local_ops`. `minimum_cycles` is a lower bound on
 * the CPU cycles charged before that operation. A branch that might leave the
 * span ends the local prefix even when the branch itself has no shared effect.
 */
struct EffectSummary {
    std::uint64_t minimum_cycles{0};
    std::size_t local_ops{0};
    bool reaches_shared{false};
    bool unknown_control_flow{false};

    [[nodiscard]] std::size_t firstExternalOp() const noexcept { return local_ops; }
};

/** @brief True for lowered integer ops that transfer control outside the span. */
[[nodiscard]] bool isControlFlowMicroOp(MicroOp op) noexcept;

/**
 * @brief Summarizes one validated native span against private memory facts.
 *
 * The span must already have passed the trace/block validator: sequential
 * edges, branch targets, widths, and register shapes are assumed valid. This
 * function performs no memory access, allocation, or mutation.
 */
[[nodiscard]] EffectSummary summarizeNativeEffects(
    std::span<const NativeTraceOp> ops,
    const PrivateMemoryFacts& facts = {}
) noexcept;

} // namespace fil::cpu
