#include "fil/cpu/forward_execution.hpp"

#include <limits>

namespace fil::cpu {

const char* forwardExitReasonName(const ForwardExitReason reason) noexcept {
    switch (reason) {
    case ForwardExitReason::completed: return "completed";
    case ForwardExitReason::issued_pending_completion: return "issued-pending-completion";
    case ForwardExitReason::instruction_limit: return "instruction-limit";
    case ForwardExitReason::cycle_limit: return "cycle-limit";
    case ForwardExitReason::deadline: return "deadline";
    case ForwardExitReason::local_boundary: return "local-boundary";
    case ForwardExitReason::shared_frontier: return "shared-frontier";
    case ForwardExitReason::checked_fallback: return "checked-fallback";
    case ForwardExitReason::declined_shared: return "declined-shared";
    case ForwardExitReason::fault: return "fault";
    case ForwardExitReason::unsupported: return "unsupported";
    }
    return "unknown";
}

namespace {

[[nodiscard]] bool reached(
    const std::uint64_t now, const std::optional<std::uint64_t> boundary
) noexcept {
    return boundary && now >= *boundary;
}

[[nodiscard]] std::uint64_t saturatingAdd(
    const std::uint64_t left, const std::uint64_t right
) noexcept {
    return right > std::numeric_limits<std::uint64_t>::max() - left
        ? std::numeric_limits<std::uint64_t>::max() : left + right;
}

} // namespace

ForwardRunExit forwardCheckedStep(
    CortexM4& cpu, const std::uint64_t now_ns,
    const ForwardExecutionLimit& limit, const bool defer_completion
) noexcept {
    ForwardRunExit exit;
    exit.issue_time_ns = now_ns;
    exit.ready_time_ns = now_ns;
    exit.started_pc = cpu.state().r[15] & ~1U;
    exit.next_pc = exit.started_pc;

    if (limit.instruction_credit == 0U) {
        exit.reason = ForwardExitReason::instruction_limit;
        return exit;
    }
    if (reached(now_ns, limit.deadline_ns)) {
        exit.reason = ForwardExitReason::deadline;
        return exit;
    }
    if (reached(now_ns, limit.local_boundary_ns)) {
        exit.reason = ForwardExitReason::local_boundary;
        return exit;
    }
    if (reached(now_ns, limit.shared_frontier_ns)) {
        exit.reason = ForwardExitReason::shared_frontier;
        return exit;
    }

    // Exact cost must be known before the first effect. Prepared metadata is
    // allocation-free and generation-checked by the CPU; an uncertain form
    // stays on checked execution without touching architecturally visible
    // state.
    std::uint16_t predicted_cycles = 0;
    if (!cpu.peekPredictableCycles(predicted_cycles)) {
        exit.reason = ForwardExitReason::checked_fallback;
        return exit;
    }
    if (predicted_cycles > limit.cycle_credit) {
        exit.reason = ForwardExitReason::cycle_limit;
        return exit;
    }

    const FastStepResult result = cpu.stepFast();
    if (result.reason != StopReason::step_complete) {
        // A shared-MMIO trap is the only failure defined as a pre-effect
        // decline. Faults are precise architectural outcomes, not rollbacks.
        exit.declined_before_effect =
            result.reason == StopReason::synchronization_required;
        exit.reason = exit.declined_before_effect
            ? ForwardExitReason::declined_shared : ForwardExitReason::fault;
        return exit;
    }
    if (result.cycles != predicted_cycles) {
        // The prediction contract was violated; effects are already applied,
        // so the caller must materialize and stop this lane rather than
        // pretending the instruction was declined.
        exit.declined_before_effect = false;
        exit.reason = ForwardExitReason::unsupported;
        exit.completed_cycles = result.cycles;
        return exit;
    }

    exit.completed_cycles = result.cycles;
    exit.next_pc = cpu.state().r[15] & ~1U;
    const std::uint64_t elapsed = limit.elapsedNs != nullptr
        ? limit.elapsedNs(limit.elapsed_context, result.cycles)
        : result.cycles;
    exit.ready_time_ns = saturatingAdd(now_ns, elapsed);
    if (defer_completion) {
        exit.declined_before_effect = false;
        exit.reason = ForwardExitReason::issued_pending_completion;
        exit.has_in_flight = true;
        exit.in_flight_pc = exit.started_pc;
        exit.in_flight_ready_time_ns = exit.ready_time_ns;
        return exit;
    }
    exit.declined_before_effect = false;
    exit.reason = ForwardExitReason::completed;
    exit.completed_instructions = 1U;
    return exit;
}

bool publishInFlightCompletion(ForwardRunExit& exit) noexcept {
    if (!exit.has_in_flight) return false;
    exit.has_in_flight = false;
    exit.ready_time_ns = exit.in_flight_ready_time_ns;
    exit.completed_instructions = 1U;
    exit.reason = ForwardExitReason::completed;
    return true;
}

} // namespace fil::cpu
