#include "fil/sim/event_loop.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fil::sim {
namespace {
thread_local EventOwner active_event_owner = shared_event_owner;
}

struct EventLoop::Impl {
    struct Event {
        SimTimeNs at{0};
        EventId id{0};
        std::uint64_t sequence{0};
        EventOwner owner{shared_event_owner};
        EventCallback callback;
        bool live{true};
    };

    using EventPtr = std::shared_ptr<Event>;

    struct Later {
        bool operator()(const EventPtr& left, const EventPtr& right) const noexcept {
            if (left->at != right->at) return left->at > right->at;
            return left->sequence > right->sequence;
        }
    };

    using Queue = std::priority_queue<EventPtr, std::vector<EventPtr>, Later>;

    SimTimeNs shared_now{0};
    EventId next_id{1};
    std::uint64_t next_sequence{0};
    std::size_t maximum_same_time_events{100000};
    mutable std::recursive_mutex mutex;
    bool concurrent_access{false};
    EventOwner serial_active_owner{shared_event_owner};
    Queue events;
    std::unordered_map<EventOwner, Queue> owner_events;
    std::unordered_map<EventId, EventPtr> live_events;
    std::unordered_map<EventOwner, SimTimeNs> owner_now;
    // Completed global frontier applied lazily to materialized owner clocks.
    SimTimeNs owner_time_floor{0};
    std::unordered_map<EventOwner, std::uint64_t> owner_generation;
    bool realtime_pacing{false};
    SimTimeNs realtime_origin_ns{0};
    SimTimeNs next_realtime_sync_ns{0};
    SimTimeNs realtime_refresh_ns{10'000'000U};
    std::chrono::steady_clock::time_point realtime_wall_origin{};

    struct Lock {
        explicit Lock(Impl& impl)
            : lock(impl.mutex, std::defer_lock) {
            if (impl.concurrent_access) lock.lock();
        }
        std::unique_lock<std::recursive_mutex> lock;
    };

    [[nodiscard]] EventOwner currentOwner() const noexcept {
        return concurrent_access ? active_event_owner : serial_active_owner;
    }

    void setCurrentOwner(const EventOwner owner) noexcept {
        if (concurrent_access) active_event_owner = owner;
        else serial_active_owner = owner;
    }

    [[nodiscard]] SimTimeNs timeFor(const EventOwner owner) const noexcept {
        if (owner == shared_event_owner) return shared_now;
        const auto found = owner_now.find(owner);
        return found == owner_now.end()
            ? shared_now : std::max(found->second, owner_time_floor);
    }

    void setTime(const EventOwner owner, const SimTimeNs time) {
        if (owner == shared_event_owner) shared_now = time;
        else owner_now[owner] = time;
    }

    static void discardDeadFront(Queue& queue) {
        while (!queue.empty() && !queue.top()->live) queue.pop();
    }

    void discardDeadGlobalFront() {
        discardDeadFront(events);
    }

    [[nodiscard]] Queue* ownerQueue(const EventOwner owner) {
        const auto found = owner_events.find(owner);
        if (found == owner_events.end()) return nullptr;
        discardDeadFront(found->second);
        return &found->second;
    }

    void discardDeadOwnerFront(const EventOwner owner) {
        const auto found = owner_events.find(owner);
        if (found == owner_events.end()) return;
        discardDeadFront(found->second);
        if (found->second.empty()) owner_events.erase(found);
    }

    void retire(const EventPtr& event) {
        event->live = false;
        live_events.erase(event->id);
        discardDeadOwnerFront(event->owner);
        ++owner_generation[event->owner];
    }

    void markOwner(EventRunResult& result, const EventOwner owner) const noexcept {
        if (owner < 64U) result.local_owner_mask |= std::uint64_t{1U} << owner;
        else result.shared_event_executed = true;
    }

    void invoke(const EventPtr& event, EventRunResult& result) {
        setTime(event->owner, event->at);
        markOwner(result, event->owner);
        ++result.events_executed;
        retire(event);

        const EventOwner previous_owner = currentOwner();
        setCurrentOwner(event->owner);
        try {
            event->callback();
        } catch (...) {
            setCurrentOwner(previous_owner);
            throw;
        }
        setCurrentOwner(previous_owner);
    }

    void pace(const SimTimeNs time_ns) {
        if (!realtime_pacing || time_ns < next_realtime_sync_ns) return;
        const SimTimeNs elapsed = time_ns - realtime_origin_ns;
        const auto target = realtime_wall_origin + std::chrono::nanoseconds(elapsed);
        std::this_thread::sleep_until(target);
        next_realtime_sync_ns = time_ns > std::numeric_limits<SimTimeNs>::max() - realtime_refresh_ns
            ? std::numeric_limits<SimTimeNs>::max() : time_ns + realtime_refresh_ns;
    }
};

EventLoop::EventLoop(const std::size_t maximum_same_time_events)
    : impl_(std::make_unique<Impl>()) {
    impl_->maximum_same_time_events = maximum_same_time_events;
}

EventLoop::~EventLoop() = default;
EventLoop::EventLoop(EventLoop&&) noexcept = default;
EventLoop& EventLoop::operator=(EventLoop&&) noexcept = default;

EventLoop::OwnerScope::OwnerScope(EventLoop& loop, const EventOwner owner) noexcept
    : loop_(&loop), previous_(loop.impl_->currentOwner()) {
    loop.impl_->setCurrentOwner(owner);
}

EventLoop::OwnerScope::~OwnerScope() {
    if (loop_ != nullptr) loop_->impl_->setCurrentOwner(previous_);
}

EventLoop::OwnerScope::OwnerScope(OwnerScope&& other) noexcept
    : loop_(std::exchange(other.loop_, nullptr)), previous_(other.previous_) {}

EventLoop::OwnerScope EventLoop::useOwner(const EventOwner owner) noexcept {
    return OwnerScope(*this, owner);
}

EventOwner EventLoop::activeOwner() const noexcept {
    return impl_->currentOwner();
}

EventId EventLoop::scheduleAfter(const SimTimeNs delta, EventCallback callback) {
    const SimTimeNs current = now();
    if (delta > std::numeric_limits<SimTimeNs>::max() - current) {
        throw std::overflow_error("simulation event time overflow");
    }
    return scheduleAt(current + delta, std::move(callback));
}

EventId EventLoop::scheduleAt(const SimTimeNs at, EventCallback callback) {
    Impl::Lock lock(*impl_);
    if (!callback) throw std::invalid_argument("simulation event callback is empty");
    if (at < now()) {
        throw std::invalid_argument("cannot schedule a simulation event in the past");
    }
    if (impl_->next_id == 0U) {
        throw std::overflow_error("simulation event identifier space exhausted");
    }

    const EventId id = impl_->next_id++;
    const EventOwner owner = impl_->currentOwner();
    auto event = std::make_shared<Impl::Event>(Impl::Event{
        at, id, impl_->next_sequence++, owner, std::move(callback), true,
    });
    impl_->events.push(event);
    impl_->owner_events[owner].push(event);
    impl_->live_events.emplace(id, std::move(event));
    ++impl_->owner_generation[owner];
    return id;
}

bool EventLoop::cancel(const EventId id) noexcept {
    Impl::Lock lock(*impl_);
    if (id == 0U) return false;
    const auto found = impl_->live_events.find(id);
    if (found == impl_->live_events.end()) return false;
    found->second->live = false;
    const EventOwner owner = found->second->owner;
    ++impl_->owner_generation[owner];
    impl_->live_events.erase(found);
    impl_->discardDeadOwnerFront(owner);
    return true;
}

EventRunResult EventLoop::runDueEvents(const SimTimeNs deadline) {
    Impl::Lock lock(*impl_);
    if (deadline < impl_->shared_now) {
        throw std::invalid_argument("cannot run the simulation clock backwards");
    }

    EventRunResult result{0, false, impl_->shared_now};
    SimTimeNs counted_time = 0U;
    std::size_t events_at_counted_time = 0U;

    for (;;) {
        impl_->discardDeadGlobalFront();
        if (impl_->events.empty() || impl_->events.top()->at > deadline) {
            impl_->shared_now = deadline;
            impl_->owner_time_floor = std::max(impl_->owner_time_floor, deadline);
            impl_->pace(deadline);
            result.stopped_at = deadline;
            return result;
        }

        const Impl::EventPtr event = impl_->events.top();
        const SimTimeNs next_time = event->at;
        if (events_at_counted_time == 0U || next_time != counted_time) {
            counted_time = next_time;
            events_at_counted_time = 0U;
        }
        if (events_at_counted_time >= impl_->maximum_same_time_events) {
            impl_->shared_now = next_time;
            result.same_time_limit_hit = true;
            result.stopped_at = next_time;
            return result;
        }

        impl_->events.pop();
        impl_->shared_now = event->at;
        impl_->pace(event->at);
        ++events_at_counted_time;
        impl_->invoke(event, result);
    }
}

EventRunResult EventLoop::runOwnedEvents(
    const EventOwner owner, const SimTimeNs deadline
) {
    Impl::Lock lock(*impl_);
    const SimTimeNs current = impl_->timeFor(owner);
    if (deadline < current) {
        throw std::invalid_argument("cannot run an owner clock backwards");
    }

    EventRunResult result{0, false, current};
    SimTimeNs counted_time = 0U;
    std::size_t events_at_counted_time = 0U;
    for (;;) {
        Impl::Queue* const queue = impl_->ownerQueue(owner);
        if (queue == nullptr || queue->empty() || queue->top()->at > deadline) {
            impl_->setTime(owner, deadline);
            result.stopped_at = deadline;
            return result;
        }

        const Impl::EventPtr event = queue->top();
        const SimTimeNs next_time = event->at;
        if (events_at_counted_time == 0U || next_time != counted_time) {
            counted_time = next_time;
            events_at_counted_time = 0U;
        }
        if (events_at_counted_time >= impl_->maximum_same_time_events) {
            impl_->setTime(owner, next_time);
            result.same_time_limit_hit = true;
            result.stopped_at = next_time;
            return result;
        }

        queue->pop();
        ++events_at_counted_time;
        impl_->invoke(event, result);
    }
}

EventRunResult EventLoop::advanceBy(const SimTimeNs delta) {
    Impl::Lock lock(*impl_);
    if (delta > std::numeric_limits<SimTimeNs>::max() - impl_->shared_now) {
        throw std::overflow_error("simulation clock overflow");
    }
    return runDueEvents(impl_->shared_now + delta);
}

void EventLoop::setRealtimePacing(const bool enabled, const SimTimeNs refresh_interval_ns) {
    Impl::Lock lock(*impl_);
    if (enabled && refresh_interval_ns == 0U) {
        throw std::invalid_argument("real-time refresh interval must be nonzero");
    }
    impl_->realtime_pacing = enabled;
    impl_->realtime_refresh_ns = refresh_interval_ns;
    impl_->realtime_origin_ns = impl_->shared_now;
    impl_->next_realtime_sync_ns = impl_->shared_now;
    impl_->realtime_wall_origin = std::chrono::steady_clock::now();
}

void EventLoop::clear() noexcept {
    Impl::Lock lock(*impl_);
    impl_->events = {};
    impl_->owner_events.clear();
    impl_->live_events.clear();
    impl_->owner_generation.clear();
}

SimTimeNs EventLoop::now() const noexcept {
    Impl::Lock lock(*impl_);
    return impl_->timeFor(impl_->currentOwner());
}

SimTimeNs EventLoop::now(const EventOwner owner) const noexcept {
    Impl::Lock lock(*impl_);
    return impl_->timeFor(owner);
}

EventLoop::OwnerCheckpoint EventLoop::ownerCheckpoint(
    const EventOwner owner
) const noexcept {
    Impl::Lock lock(*impl_);
    const auto generation = impl_->owner_generation.find(owner);
    return OwnerCheckpoint{
        owner,
        impl_->timeFor(owner),
        generation == impl_->owner_generation.end() ? 0U : generation->second,
    };
}

bool EventLoop::canRestoreOwnerCheckpoint(
    const OwnerCheckpoint& checkpoint
) const noexcept {
    Impl::Lock lock(*impl_);
    const auto generation = impl_->owner_generation.find(checkpoint.owner);
    const std::uint64_t current_generation = generation == impl_->owner_generation.end()
        ? 0U : generation->second;
    return current_generation == checkpoint.event_generation
        && checkpoint.time_ns <= impl_->timeFor(checkpoint.owner);
}

bool EventLoop::restoreOwnerCheckpoint(const OwnerCheckpoint& checkpoint) noexcept {
    Impl::Lock lock(*impl_);
    if (!canRestoreOwnerCheckpoint(checkpoint)) return false;
    impl_->setTime(checkpoint.owner, checkpoint.time_ns);
    return true;
}

std::size_t EventLoop::pending() const noexcept {
    Impl::Lock lock(*impl_);
    return impl_->live_events.size();
}

std::optional<SimTimeNs> EventLoop::nextScheduledTime() {
    Impl::Lock lock(*impl_);
    impl_->discardDeadGlobalFront();
    if (impl_->events.empty()) return std::nullopt;
    return impl_->events.top()->at;
}

std::optional<SimTimeNs> EventLoop::nextScheduledTime(const EventOwner owner) {
    Impl::Lock lock(*impl_);
    Impl::Queue* const queue = impl_->ownerQueue(owner);
    if (queue == nullptr || queue->empty()) return std::nullopt;
    return queue->top()->at;
}

void EventLoop::setConcurrentAccess(const bool enabled) noexcept {
    std::lock_guard lock(impl_->mutex);
    impl_->concurrent_access = enabled;
}

void EventLoop::setMaximumSameTimeEvents(const std::size_t maximum) noexcept {
    Impl::Lock lock(*impl_);
    impl_->maximum_same_time_events = maximum;
}

std::size_t EventLoop::maximumSameTimeEvents() const noexcept {
    Impl::Lock lock(*impl_);
    return impl_->maximum_same_time_events;
}

} // namespace fil::sim
