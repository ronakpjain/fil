#pragma once

/** @file event_loop.hpp
 *  @brief Deterministic simulated-time event scheduling.
 */

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <limits>

namespace fil::sim {

/** @brief Nanoseconds on the monotonically increasing simulation clock. */
using SimTimeNs = std::uint64_t;

/** @brief Stable identifier assigned to a scheduled event. */
using EventId = std::uint64_t;

/** @brief Callback executed by the deterministic event loop. */
using EventCallback = std::function<void()>;

/** @brief Board lane responsible for an event, or the shared simulation domain. */
using EventOwner = std::uint32_t;

/** @brief Owner used by CAN and other cross-board events. */
inline constexpr EventOwner shared_event_owner = std::numeric_limits<EventOwner>::max();

/** @brief Outcome of one run through due events. */
struct EventRunResult {
    std::size_t events_executed{0}; ///< Number of callbacks completed.
    bool same_time_limit_hit{false}; ///< True when zero-delay livelock protection stopped the run.
    SimTimeNs stopped_at{0}; ///< Simulation time reached by the run.
    std::uint64_t local_owner_mask{0}; ///< Board owners below 64 whose callbacks ran.
    bool shared_event_executed{false}; ///< Whether a shared or unrepresentable owner ran.
};

/**
 * @brief Single-threaded deterministic simulated-time event loop.
 *
 * Events are ordered by timestamp and then insertion sequence. Cancellation is
 * idempotent, and callbacks may safely schedule more work at the current time.
 */
class EventLoop {
public:
    struct OwnerCheckpoint {
        EventOwner owner{shared_event_owner};
        SimTimeNs time_ns{0};
        std::uint64_t event_generation{0};
    };

    /** @brief RAII scope inherited by events scheduled within one board lane. */
    class OwnerScope {
    public:
        ~OwnerScope();
        OwnerScope(const OwnerScope&) = delete;
        OwnerScope& operator=(const OwnerScope&) = delete;
        OwnerScope(OwnerScope&& other) noexcept;
        OwnerScope& operator=(OwnerScope&&) = delete;

    private:
        friend class EventLoop;
        OwnerScope(EventLoop& loop, EventOwner owner) noexcept;
        EventLoop* loop_{nullptr};
        EventOwner previous_{shared_event_owner};
    };

    /** @brief Constructs an empty loop with a bounded same-time callback count. */
    explicit EventLoop(std::size_t maximum_same_time_events = 100000);
    ~EventLoop();

    EventLoop(const EventLoop&) = delete;
    EventLoop& operator=(const EventLoop&) = delete;
    EventLoop(EventLoop&&) noexcept;
    EventLoop& operator=(EventLoop&&) noexcept;

    /** @brief Makes subsequently scheduled events inherit a board or shared owner. */
    [[nodiscard]] OwnerScope useOwner(EventOwner owner) noexcept;

    /** @brief Gets the owner inherited by newly scheduled events. */
    [[nodiscard]] EventOwner activeOwner() const noexcept;

    /** @brief Schedules a callback relative to the current simulation time. */
    [[nodiscard]] EventId scheduleAfter(SimTimeNs delta, EventCallback callback);

    /** @brief Schedules a callback at an absolute time no earlier than now. */
    [[nodiscard]] EventId scheduleAt(SimTimeNs at, EventCallback callback);

    /** @brief Cancels an event if it is still pending. */
    [[nodiscard]] bool cancel(EventId id) noexcept;

    /** @brief Runs all events due at or before the supplied global deadline. */
    [[nodiscard]] EventRunResult runDueEvents(SimTimeNs deadline);

    /** @brief Runs only one owner's callbacks and advances that owner's local clock. */
    [[nodiscard]] EventRunResult runOwnedEvents(EventOwner owner, SimTimeNs deadline);

    /** @brief Advances by a relative duration and runs all newly due events. */
    [[nodiscard]] EventRunResult advanceBy(SimTimeNs delta);

    /** @brief Removes every pending event without changing current time. */
    void clear() noexcept;

    /** @brief Gets the active owner's clock, or the shared clock outside an owner scope. */
    [[nodiscard]] SimTimeNs now() const noexcept;

    /** @brief Gets one owner's local clock without changing scheduling context. */
    [[nodiscard]] SimTimeNs now(EventOwner owner) const noexcept;

    /** @brief Captures an owner clock plus its queue-mutation generation. */
    [[nodiscard]] OwnerCheckpoint ownerCheckpoint(EventOwner owner) const noexcept;

    /** @brief Whether an owner clock can be rewound without undoing callbacks. */
    [[nodiscard]] bool canRestoreOwnerCheckpoint(
        const OwnerCheckpoint& checkpoint
    ) const noexcept;

    /** @brief Rewinds an owner clock only when its event queue was untouched. */
    [[nodiscard]] bool restoreOwnerCheckpoint(const OwnerCheckpoint& checkpoint) noexcept;

    /** @brief Gets the number of live, non-cancelled events. */
    [[nodiscard]] std::size_t pending() const noexcept;

    /** @brief Gets the earliest live event timestamp across every owner queue. */
    [[nodiscard]] std::optional<SimTimeNs> nextScheduledTime();

    /** @brief Gets the earliest live timestamp for one board or the shared queue. */
    [[nodiscard]] std::optional<SimTimeNs> nextScheduledTime(EventOwner owner);

    /** @brief Enables locking for concurrent owner-lane access. Set before workers start. */
    void setConcurrentAccess(bool enabled) noexcept;

    /** @brief Changes zero-delay livelock protection; zero disables callbacks. */
    void setMaximumSameTimeEvents(std::size_t maximum) noexcept;

    /** @brief Gets the current zero-delay callback limit. */
    [[nodiscard]] std::size_t maximumSameTimeEvents() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace fil::sim
