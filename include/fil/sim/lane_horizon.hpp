#pragma once

/** @file lane_horizon.hpp
 *  @brief Conservative serial/parallel frontiers from lane-local proofs.
 *
 * A horizon is a lower bound on when a lane can next create or observe a
 * shared action. It is not a prediction, a snapshot, or a rollback point:
 * proofs are announced from one synchronized state and must not be revoked
 * after another lane relies on them. An absent `earliest_shared_action` means
 * the lane cannot create a shared action at all within the proof's scope; a
 * finite value means no shared action can occur before it.
 *
 * The coordinator may advance only to the earliest finite bound. The frontier
 * is exclusive: an action exactly at the frontier is not admitted until its
 * canonical issue/event phase is processed. If no finite bound exists, callers
 * must still stop at their own local deadline or event boundary.
 */

#include "fil/sim/event_loop.hpp"

#include <cstdint>
#include <optional>
#include <span>

namespace fil::sim {

/** @brief One lane's conservative proof, tagged with the state it assumed. */
struct HorizonProof {
    std::optional<SimTimeNs> earliest_shared_action{};
    std::uint64_t generation{0};
};

/**
 * @brief Returns the earliest finite shared-action bound across lanes and inputs.
 *
 * `pending_global` is the earliest already scheduled global observation or
 * shared event. `external_input` is the earliest host/network/deadline boundary
 * that can change lane state. The result is the minimum of every finite value;
 * nullopt means no supplied source proved a finite shared action.
 */
[[nodiscard]] std::optional<SimTimeNs> conservativeFrontier(
    std::span<const HorizonProof> lanes,
    std::optional<SimTimeNs> pending_global = std::nullopt,
    std::optional<SimTimeNs> external_input = std::nullopt
) noexcept;

/**
 * @brief Converts a cycle lower bound at `now` into an absolute horizon.
 *
 * Returns nullopt when the lane cannot create a shared action. Saturates at
 * the maximum representable simulation time instead of wrapping.
 */
[[nodiscard]] std::optional<SimTimeNs> horizonFromCycles(
    SimTimeNs now, std::uint64_t cycles, std::uint64_t clock_hz
) noexcept;

} // namespace fil::sim
