#include "fil/sim/lane_horizon.hpp"

#include <limits>

namespace fil::sim {

std::optional<SimTimeNs> conservativeFrontier(
    const std::span<const HorizonProof> lanes,
    const std::optional<SimTimeNs> pending_global,
    const std::optional<SimTimeNs> external_input
) noexcept {
    std::optional<SimTimeNs> frontier = pending_global;
    const auto fold = [&frontier](const std::optional<SimTimeNs> value) {
        if (!value) return;
        if (!frontier || *value < *frontier) frontier = value;
    };
    fold(external_input);
    for (const auto& lane : lanes) fold(lane.earliest_shared_action);
    return frontier;
}

std::optional<SimTimeNs> horizonFromCycles(
    const SimTimeNs now, const std::uint64_t cycles, const std::uint64_t clock_hz
) noexcept {
    if (clock_hz == 0U) return std::nullopt;
    constexpr std::uint64_t nanoseconds_per_second = 1'000'000'000ULL;
    const std::uint64_t numerator = cycles
            > std::numeric_limits<std::uint64_t>::max() / nanoseconds_per_second
        ? std::numeric_limits<std::uint64_t>::max()
        : cycles * nanoseconds_per_second;
    const std::uint64_t delta = numerator / clock_hz;
    if (delta > std::numeric_limits<SimTimeNs>::max() - now) {
        return std::numeric_limits<SimTimeNs>::max();
    }
    return now + delta;
}

} // namespace fil::sim
