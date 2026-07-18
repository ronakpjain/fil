#pragma once

/** @file event_loop.hpp
 *  @brief Deterministic simulated-time event scheduling.
 */

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

namespace fil::sim {

/** @brief Nanoseconds on the monotonically increasing simulation clock. */
using SimTimeNs = std::uint64_t;

/** @brief Stable identifier assigned to a scheduled event. */
using EventId = std::uint64_t;

/** @brief Callback executed by the deterministic event loop. */
using EventCallback = std::function<void()>;

/** @brief Outcome of one run through due events. */
struct EventRunResult {
    std::size_t events_executed{0}; ///< Number of callbacks completed.
    bool same_time_limit_hit{false}; ///< True when zero-delay livelock protection stopped the run.
    SimTimeNs stopped_at{0}; ///< Simulation time reached by the run.
};

/**
 * @brief Single-threaded deterministic simulated-time event loop.
 *
 * Events are ordered by timestamp and then insertion sequence. Cancellation is
 * idempotent, and callbacks may safely schedule more work at the current time.
 */
class EventLoop {
public:
    /** @brief Constructs an empty loop with a bounded same-time callback count. */
    explicit EventLoop(std::size_t maximum_same_time_events = 100000);
    ~EventLoop();

    EventLoop(const EventLoop&) = delete;
    EventLoop& operator=(const EventLoop&) = delete;
    EventLoop(EventLoop&&) noexcept;
    EventLoop& operator=(EventLoop&&) noexcept;

    /** @brief Schedules a callback relative to the current simulation time. */
    [[nodiscard]] EventId scheduleAfter(SimTimeNs delta, EventCallback callback);

    /** @brief Schedules a callback at an absolute time no earlier than now. */
    [[nodiscard]] EventId scheduleAt(SimTimeNs at, EventCallback callback);

    /** @brief Cancels an event if it is still pending. */
    [[nodiscard]] bool cancel(EventId id) noexcept;

    /** @brief Runs all events due at or before the supplied deadline. */
    [[nodiscard]] EventRunResult runDueEvents(SimTimeNs deadline);

    /** @brief Advances by a relative duration and runs all newly due events. */
    [[nodiscard]] EventRunResult advanceBy(SimTimeNs delta);

    /** @brief Removes every pending event without changing current time. */
    void clear() noexcept;

    /** @brief Gets the current monotonically increasing simulation time. */
    [[nodiscard]] SimTimeNs now() const noexcept;

    /** @brief Gets the number of live, non-cancelled events. */
    [[nodiscard]] std::size_t pending() const noexcept;

    /** @brief Gets the earliest live event timestamp, ignoring cancelled entries. */
    [[nodiscard]] std::optional<SimTimeNs> nextScheduledTime();

    /** @brief Changes zero-delay livelock protection; zero disables callbacks. */
    void setMaximumSameTimeEvents(std::size_t maximum) noexcept;

    /** @brief Gets the current zero-delay callback limit. */
    [[nodiscard]] std::size_t maximumSameTimeEvents() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace fil::sim
