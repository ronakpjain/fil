#include "fil/sim/event_loop.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
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
        std::function<bool()> locality_guard;
        bool globally_observed{true};
        bool live{true};
    };

    using EventPtr = std::shared_ptr<Event>;

    struct Later {
        bool operator()(const EventPtr& left, const EventPtr& right) const noexcept {
            if (left->at != right->at) return left->at > right->at;
            return left->sequence > right->sequence;
        }
    };

    using QueueBase = std::priority_queue<EventPtr, std::vector<EventPtr>, Later>;

    /// Exposes the underlying container so checkpoints can enumerate it.
    struct Queue : QueueBase {
        const std::vector<EventPtr>& container() const { return this->c; }
    };

    SimTimeNs shared_now{0};
    EventId next_id{1};
    std::uint64_t next_sequence{0};
    std::size_t maximum_same_time_events{100000};
    mutable std::recursive_mutex mutex;
    bool concurrent_access{false};
    EventOwner serial_active_owner{shared_event_owner};
    Queue events;
    Queue globally_observed_events;
    std::unordered_map<EventOwner, Queue> owner_events;
    std::shared_ptr<const ObservationBarrier> observation_barrier;
    ObservationBarrier scheduler_observation_barrier;
    std::unordered_map<EventId, EventPtr> live_events;
    std::unordered_map<EventOwner, SimTimeNs> owner_now;
    std::unordered_map<EventOwner, std::uint64_t> owner_generation;

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
        if (found == owner_now.end()) return shared_now;
        // Owner clocks never read behind the shared clock: runDueEvents used
        // to force every owner forward on each call, which cost a map walk
        // per scheduler frontier. Clamping here returns the same value that
        // eager walk would have produced (no owner read can intervene between
        // a rewind and the next frontier advance without passing through a
        // runDueEvents that re-syncs first), while frontiers with no owner
        // activity skip the walk entirely.
        return std::max(found->second, shared_now);
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

    void retire(const EventPtr& event) {
        event->live = false;
        live_events.erase(event->id);
        discardDeadFront(globally_observed_events);
        ++owner_generation[event->owner];
    }

    void markOwner(EventRunResult& result, const EventOwner owner) const noexcept {
        if (owner < 64U) result.local_owner_mask |= std::uint64_t{1U} << owner;
        else result.shared_event_executed = true;
    }

    EventId schedule(
        const SimTimeNs at, EventCallback callback,
        std::function<bool()> locality_guard = {}
    ) {
        if (!callback) throw std::invalid_argument("simulation event callback is empty");
        if (at < timeFor(currentOwner())) {
            throw std::invalid_argument("cannot schedule a simulation event in the past");
        }
        if (next_id == 0U) throw std::overflow_error("simulation event identifier space exhausted");
        const EventId id = next_id++;
        const EventOwner owner = currentOwner();
        const bool globally_observed = !locality_guard || owner == shared_event_owner;
        auto event = std::make_shared<Event>(Event{
            at, id, next_sequence++, owner, std::move(callback),
            std::move(locality_guard), globally_observed, true,
        });
        events.push(event);
        if (globally_observed) globally_observed_events.push(event);
        auto& owner_queue = owner_events[owner];
        discardDeadFront(owner_queue);
        owner_queue.push(event);
        live_events.emplace(id, event);
        ++owner_generation[owner];
        return id;
    }

    void invoke(const EventPtr& event, EventRunResult& result) {
        setTime(event->owner, event->at);
        markOwner(result, event->owner);
        ++result.events_executed;
        retire(event);

        const EventOwner previous_owner = currentOwner();
        setCurrentOwner(event->owner);
        try {
            const std::shared_ptr<const ObservationBarrier> observer = observation_barrier;
            const bool owner_local = event->owner != shared_event_owner &&
                event->locality_guard && event->locality_guard();
            const EventObservation classification = owner_local
                ? EventObservation::owner_local : EventObservation::global;

            if (scheduler_observation_barrier) {
                scheduler_observation_barrier(
                    event->at, event->owner,
                    observer ? EventObservation::global : classification
                );
            }
            if (observer && *observer) (*observer)(event->at, event->owner, classification);

            // Observers can alter ADC hooks. Preserve the invocation snapshot so
            // downgrade notification still reaches an observer that clears its slot.
            if (owner_local && !event->locality_guard()) {
                if (scheduler_observation_barrier) {
                    scheduler_observation_barrier(
                        event->at, event->owner, EventObservation::global
                    );
                }
                if (observer && *observer) {
                    (*observer)(event->at, event->owner, EventObservation::global);
                }
            }
            event->callback();
        } catch (...) {
            setCurrentOwner(previous_owner);
            throw;
        }
        setCurrentOwner(previous_owner);
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
    return impl_->schedule(at, std::move(callback));
}

EventId EventLoop::scheduleOwnerLocalAt(
    const SimTimeNs at, EventCallback callback, std::function<bool()> locality_guard
) {
    Impl::Lock lock(*impl_);
    if (!locality_guard) throw std::invalid_argument("owner-local event requires a locality guard");
    return impl_->schedule(at, std::move(callback), std::move(locality_guard));
}

bool EventLoop::cancel(const EventId id) noexcept {
    Impl::Lock lock(*impl_);
    if (id == 0U) return false;
    const auto found = impl_->live_events.find(id);
    if (found == impl_->live_events.end()) return false;
    found->second->live = false;
    Impl::discardDeadFront(impl_->globally_observed_events);
    ++impl_->owner_generation[found->second->owner];
    impl_->live_events.erase(found);
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
            // Owner clocks are clamped to the shared clock on read
            // (see timeFor), so no per-frontier map walk is needed here.
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

void EventLoop::clear() noexcept {
    Impl::Lock lock(*impl_);
    impl_->events = {};
    impl_->globally_observed_events = {};
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
    // Snapshot the owner's live events so a rollback can undo scheduling and
    // retirement performed by a transactional worker slice.
    std::vector<Impl::EventPtr> snapshot;
    if (const auto* queue = impl_->ownerQueue(owner)) {
        for (const auto& event : queue->container()) {
            if (event->live) snapshot.push_back(event);
        }
    }
    return OwnerCheckpoint{
        owner,
        impl_->timeFor(owner),
        generation == impl_->owner_generation.end() ? 0U : generation->second,
        std::make_shared<const std::vector<Impl::EventPtr>>(std::move(snapshot)),
    };
}

bool EventLoop::canRestoreOwnerCheckpoint(
    const OwnerCheckpoint& checkpoint
) const noexcept {
    Impl::Lock lock(*impl_);
    // The shared clock never runs backwards: a shared-owner checkpoint is
    // restorable only while the clock is untouched. Owner-owned queues can
    // rewind to their captured time.
    if (checkpoint.owner == shared_event_owner) {
        if (checkpoint.time_ns != impl_->timeFor(checkpoint.owner)) return false;
    } else if (checkpoint.time_ns > impl_->timeFor(checkpoint.owner)) {
        return false;
    }
    // Fired event callbacks are observable effects: an epoch in which any
    // captured event fired cannot be rolled back. Scheduling alone is
    // undoable, so pending additions/removals are fine.
    const auto* const snapshot = static_cast<const std::vector<Impl::EventPtr>*>(
        checkpoint.captured_events.get()
    );
    if (snapshot == nullptr) return false;
    for (const auto& event : *snapshot) {
        if (!event->live) return false;
    }
    return true;
}

bool EventLoop::restoreOwnerCheckpoint(const OwnerCheckpoint& checkpoint) noexcept {
    Impl::Lock lock(*impl_);
    if (!canRestoreOwnerCheckpoint(checkpoint)) return false;
    const auto* const snapshot = static_cast<const std::vector<Impl::EventPtr>*>(
        checkpoint.captured_events.get()
    );
    if (snapshot == nullptr) return false;
    auto* const queue = impl_->ownerQueue(checkpoint.owner);
    if (queue == nullptr && !snapshot->empty()) return false;

    // Undo events scheduled after the capture: mark them dead and drop them
    // from the live index. Their queue entries become lazy garbage.
    if (queue != nullptr) {
        std::unordered_set<EventId> captured;
        captured.reserve(snapshot->size());
        for (const auto& event : *snapshot) captured.insert(event->id);
        for (const auto& event : queue->container()) {
            if (event->live && captured.find(event->id) == captured.end()) {
                event->live = false;
                impl_->live_events.erase(event->id);
            }
        }
    }
    impl_->setTime(checkpoint.owner, checkpoint.time_ns);
    impl_->owner_generation[checkpoint.owner] = checkpoint.event_generation;
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

ObservationBarrier EventLoop::exchangeObservationBarrier(ObservationBarrier barrier) {
    Impl::Lock lock(*impl_);
    ObservationBarrier previous;
    if (impl_->observation_barrier) previous = *impl_->observation_barrier;
    impl_->observation_barrier = barrier
        ? std::make_shared<const ObservationBarrier>(std::move(barrier)) : nullptr;
    return previous;
}

ObservationBarrier EventLoop::exchangeSchedulerObservationBarrier(
    ObservationBarrier barrier
) {
    Impl::Lock lock(*impl_);
    std::swap(impl_->scheduler_observation_barrier, barrier);
    return barrier;
}

std::optional<SimTimeNs> EventLoop::nextObservationTime(const EventOwner owner) {
    Impl::Lock lock(*impl_);
    Impl::Queue* const owner_queue = impl_->ownerQueue(owner);
    Impl::discardDeadFront(impl_->globally_observed_events);
    std::optional<SimTimeNs> result;
    if (owner_queue != nullptr && !owner_queue->empty()) result = owner_queue->top()->at;
    if (!impl_->globally_observed_events.empty()) {
        const SimTimeNs global_time = impl_->globally_observed_events.top()->at;
        if (!result || global_time < *result) result = global_time;
    }
    return result;
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

ScheduledEvent::~ScheduledEvent() {
    cancel();
}

EventCallback ScheduledEvent::retireBefore(EventCallback callback) {
    return [this, callback = std::move(callback)]() mutable {
        id_ = 0U;
        callback();
    };
}

EventId ScheduledEvent::scheduleAfter(
    const SimTimeNs delta,
    EventCallback callback
) {
    if (!callback) throw std::invalid_argument("scheduled event callback is empty");
    cancel();
    if (loop_ == nullptr) return 0U;
    id_ = loop_->scheduleAfter(delta, retireBefore(std::move(callback)));
    return id_;
}

EventId ScheduledEvent::scheduleAt(
    const SimTimeNs at,
    EventCallback callback
) {
    if (!callback) throw std::invalid_argument("scheduled event callback is empty");
    cancel();
    if (loop_ == nullptr) return 0U;
    id_ = loop_->scheduleAt(at, retireBefore(std::move(callback)));
    return id_;
}

EventId ScheduledEvent::scheduleOwnerLocalAt(
    const SimTimeNs at, EventCallback callback, std::function<bool()> locality_guard
) {
    if (!callback) throw std::invalid_argument("scheduled event callback is empty");
    cancel();
    if (loop_ == nullptr) return 0U;
    id_ = loop_->scheduleOwnerLocalAt(
        at, retireBefore(std::move(callback)), std::move(locality_guard)
    );
    return id_;
}

void ScheduledEvent::cancel() noexcept {
    if (id_ != 0U && loop_ != nullptr) {
        static_cast<void>(loop_->cancel(id_));
    }
    id_ = 0U;
}

} // namespace fil::sim
