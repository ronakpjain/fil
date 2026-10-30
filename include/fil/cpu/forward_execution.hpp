#pragma once

/** @file forward_execution.hpp
 *  @brief Forward-only instruction admission with one explicit in-flight tail.
 *
 * The checked scheduler applies instruction effects at the reference issue
 * time and publishes completion later. A compiled lane must decide whether an
 * instruction fits its instruction, cycle, deadline, and horizon credit
 * *before* the instruction can create any effect. This header defines that
 * decision as a small result type plus one exact-cost helper.
 *
 * The helper is deliberately conservative: when the next instruction's exact
 * CPU-cycle cost is not known from prepared metadata, it reports
 * `checked_fallback` with no state change. Variable-cost or fault-sensitive
 * forms therefore stay on checked execution instead of being partially
 * executed and unwound. Helium: no rollback journals, snapshots, or replay
 * certificates participate.
 */

#include "fil/cpu/cortex_m4.hpp"

#include <cstdint>
#include <optional>

namespace fil::cpu {

/** @brief Why a forward admission stopped. */
enum class ForwardExitReason : std::uint8_t {
    completed = 0,     ///< Exactly one instruction retired.
    issued_pending_completion, ///< Effects applied; completion still pending.
    instruction_limit, ///< No instruction credit remains.
    cycle_limit,       ///< Exact cost exceeds remaining cycle credit.
    deadline,          ///< Absolute time deadline reached before issue.
    local_boundary,    ///< Lane-local event/exception boundary reached.
    shared_frontier,   ///< Proved shared-action frontier reached.
    checked_fallback,  ///< Exact cost unknown; execute on the checked path.
    declined_shared,   ///< Shared MMIO would change cross-lane state.
    fault,             ///< Architectural/bus fault raised by the instruction.
    unsupported,       ///< State cannot be executed forward safely.
};

[[nodiscard]] const char* forwardExitReasonName(ForwardExitReason reason) noexcept;

/**
 * @brief Per-admission limits; no ambiguous single "budget" field.
 *
 * `elapsedNs` converts charged CPU cycles to elapsed nanoseconds without
 * losing a caller-owned fractional remainder. It is called at most once per
 * admitted instruction and must be the same conversion the reference
 * scheduler uses, otherwise published ready times drift.
 */
struct ForwardExecutionLimit {
    std::uint64_t instruction_credit{0};
    std::uint64_t cycle_credit{0};
    std::optional<std::uint64_t> deadline_ns{};
    std::optional<std::uint64_t> local_boundary_ns{};
    std::optional<std::uint64_t> shared_frontier_ns{};
    std::uint64_t (*elapsedNs)(void* context, std::uint64_t cycles){nullptr};
    void* elapsed_context{nullptr};
};

/**
 * @brief Result of one forward admission.
 *
 * `in_flight` is set only when an instruction was issued but its completion
 * was not published by this admission. The current helper publishes completion
 * synchronously, so callers seed the field when they defer materialization;
 * the invariant is that at most one in-flight instruction exists per lane and
 * its issue effects are already applied.
 */
struct ForwardRunExit {
    ForwardExitReason reason{ForwardExitReason::checked_fallback};
    bool declined_before_effect{true};
    std::uint32_t started_pc{0};
    std::uint32_t next_pc{0};
    std::uint32_t completed_instructions{0};
    std::uint64_t completed_cycles{0};
    std::uint64_t issue_time_ns{0};
    std::uint64_t ready_time_ns{0};
    bool has_in_flight{false};
    std::uint32_t in_flight_pc{0};
    std::uint64_t in_flight_ready_time_ns{0};
};

/**
 * @brief Admits and executes at most one predictable instruction.
 *
 * All credit/deadline/frontier checks run before any effect. When the exact
 * cost is unavailable or exceeds credit, the CPU and memory are untouched and
 * the caller keeps the instruction on checked execution. Shared-MMIO declines
 * and faults are reported distinctly. With `defer_completion` the issue-time
 * effects are applied but the completion stays pending in the returned exit,
 * ready for `publishInFlightCompletion()` after intervening callbacks.
 */
[[nodiscard]] ForwardRunExit forwardCheckedStep(
    CortexM4& cpu, std::uint64_t now_ns, const ForwardExecutionLimit& limit,
    bool defer_completion = false
) noexcept;

/**
 * @brief Publishes the pending completion of one in-flight instruction.
 *
 * Returns false when the exit has no pending completion. Effects remain as
 * issued; only the completion boundary is advanced to the recorded ready time.
 */
[[nodiscard]] bool publishInFlightCompletion(ForwardRunExit& exit) noexcept;

} // namespace fil::cpu
