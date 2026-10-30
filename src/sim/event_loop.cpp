#include "fil/sim/event_loop.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <mutex>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fil::sim {
struct EventLoop::Impl {
    /**
     * @brief Callback storage for one scheduled event.
     *
     * Slots are reused after cancellation or execution. The monotonically
     * increasing EventId is the slot's generation: stale heap entries compare
     * their stored id against the slot and are discarded without an ABA risk.
     */
    struct Slot {
        SimTimeNs at{0};
        EventId id{0};
        std::uint64_t sequence{0};
        EventOwner owner{shared_event_owner};
        EventCallback callback;
        std::function<bool()> locality_guard;
        bool globally_observed{true};
        bool live{false};
    };

    /** Key and slot reference carried by every pending queue entry. */
    struct Entry {
        SimTimeNs at{0};
        std::uint64_t sequence{0};
        std::uint32_t slot{0};
        EventId id{0};
    };

    struct SlotRef {
        std::uint32_t slot{0};
        EventId id{0};
    };

    struct Later {
        bool operator()(const Entry& left, const Entry& right) const noexcept {
            if (left.at != right.at) return left.at > right.at;
            return left.sequence > right.sequence;
        }
    };

    using Queue = std::priority_queue<Entry, std::vector<Entry>, Later>;

    SimTimeNs shared_now{0};
    EventId next_id{1};
    std::uint64_t next_sequence{0};
    std::size_t maximum_same_time_events{100000};
    mutable std::recursive_mutex mutex;
    EventOwner serial_active_owner{shared_event_owner};
    std::vector<Slot> slots;
    std::vector<std::uint32_t> free_slots;
    Queue events;
    Queue globally_observed_events;
    std::unordered_map<EventOwner, Queue> owner_events;
    std::shared_ptr<const ObservationBarrier> observation_barrier;
    ObservationBarrier scheduler_observation_barrier;
    std::unordered_map<EventId, SlotRef> live_events;
    std::unordered_map<EventOwner, SimTimeNs> owner_now;

    static Impl*& exclusiveImpl() noexcept {
        static thread_local Impl* current = nullptr;
        return current;
    }

    struct Lock {
        explicit Lock(Impl& impl) : lock(impl.mutex, std::defer_lock) {
            if (exclusiveImpl() != &impl) lock.lock();
        }
        std::unique_lock<std::recursive_mutex> lock;
    };

    [[nodiscard]] EventOwner currentOwner() const noexcept {
        return serial_active_owner;
    }

    void setCurrentOwner(const EventOwner owner) noexcept {
        serial_active_owner = owner;
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

    [[nodiscard]] bool entryLive(const Entry& entry) const noexcept {
        return entry.slot < slots.size()
            && slots[entry.slot].live
            && slots[entry.slot].id == entry.id;
    }

    static void discardDeadFront(Queue& queue, const std::vector<Slot>& slots) {
        while (!queue.empty()) {
            const Entry& entry = queue.top();
            if (entry.slot < slots.size() && slots[entry.slot].live
                && slots[entry.slot].id == entry.id) return;
            queue.pop();
        }
    }

    void discardDeadFront(Queue& queue) const {
        discardDeadFront(queue, slots);
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

    void releaseSlot(const std::uint32_t index) noexcept {
        Slot& slot = slots[index];
        slot.live = false;
        slot.callback = {};
        slot.locality_guard = {};
        free_slots.push_back(index);
    }

    void retire(const Entry& entry) {
        Slot& slot = slots[entry.slot];
        slot.live = false;
        live_events.erase(slot.id);
        free_slots.push_back(entry.slot);
        discardDeadFront(globally_observed_events);
    }

    void markOwner(EventRunResult& result, const EventOwner owner) const noexcept {
        if (owner < 64U) result.local_owner_mask |= std::uint64_t{1U} << owner;
        else result.shared_event_executed = true;
    }

    std::uint32_t acquireSlot() {
        std::uint32_t index = 0;
        if (!free_slots.empty()) {
            index = free_slots.back();
            free_slots.pop_back();
        } else {
            if (slots.size() >= std::numeric_limits<std::uint32_t>::max()) {
                throw std::length_error("simulation event slot space exhausted");
            }
            index = static_cast<std::uint32_t>(slots.size());
            slots.emplace_back();
        }
        slots[index].live = true;
        return index;
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
        const std::uint32_t index = acquireSlot();
        Slot& slot = slots[index];
        slot.at = at;
        slot.id = id;
        slot.sequence = next_sequence++;
        slot.owner = owner;
        slot.callback = std::move(callback);
        slot.locality_guard = std::move(locality_guard);
        slot.globally_observed = globally_observed;

        const Entry entry{slot.at, slot.sequence, index, id};
        events.push(entry);
        if (globally_observed) globally_observed_events.push(entry);
        auto& owner_queue = owner_events[owner];
        discardDeadFront(owner_queue);
        owner_queue.push(entry);
        live_events.emplace(id, SlotRef{index, id});
        return id;
    }

    EventRunResult runDueEventsLocked(const SimTimeNs deadline) {
        if (deadline < shared_now) {
            throw std::invalid_argument("cannot run the simulation clock backwards");
        }

        EventRunResult result{0, false, shared_now};
        SimTimeNs counted_time = 0U;
        std::size_t events_at_counted_time = 0U;

        for (;;) {
            discardDeadGlobalFront();
            if (events.empty() || events.top().at > deadline) {
                shared_now = deadline;
                // Owner clocks are clamped to the shared clock on read
                // (see timeFor), so no per-frontier map walk is needed here.
                result.stopped_at = deadline;
                return result;
            }

            const Entry entry = events.top();
            const SimTimeNs next_time = entry.at;
            if (events_at_counted_time == 0U || next_time != counted_time) {
                counted_time = next_time;
                events_at_counted_time = 0U;
            }
            if (events_at_counted_time >= maximum_same_time_events) {
                shared_now = next_time;
                result.same_time_limit_hit = true;
                result.stopped_at = next_time;
                return result;
            }

            events.pop();
            shared_now = entry.at;
            ++events_at_counted_time;
            invoke(entry, result);
        }
    }

    void invoke(const Entry& entry, EventRunResult& result) {
        Slot& slot = slots[entry.slot];
        const SimTimeNs at = slot.at;
        const EventOwner owner = slot.owner;
        const EventId id = slot.id;
        // Move the callables out before the slot can be reused by a nested
        // schedule or by the observer itself.
        EventCallback callback = std::move(slot.callback);
        std::function<bool()> locality_guard = std::move(slot.locality_guard);
        slot.live = false;
        live_events.erase(id);
        free_slots.push_back(entry.slot);
        discardDeadFront(globally_observed_events);

        setTime(owner, at);
        markOwner(result, owner);
        ++result.events_executed;

        const EventOwner previous_owner = currentOwner();
        setCurrentOwner(owner);
        try {
            const std::shared_ptr<const ObservationBarrier> observer = observation_barrier;
            const bool owner_local = owner != shared_event_owner
                && locality_guard && locality_guard();
            const EventObservation classification = owner_local
                ? EventObservation::owner_local : EventObservation::global;

            if (scheduler_observation_barrier) {
                scheduler_observation_barrier(
                    at, owner,
                    observer ? EventObservation::global : classification
                );
            }
            if (observer && *observer) (*observer)(at, owner, classification);

            // Observers can alter ADC hooks. Preserve the invocation snapshot so
            // downgrade notification still reaches an observer that clears its slot.
            if (owner_local && !locality_guard()) {
                if (scheduler_observation_barrier) {
                    scheduler_observation_barrier(at, owner, EventObservation::global);
                }
                if (observer && *observer) {
                    (*observer)(at, owner, EventObservation::global);
                }
            }
            callback();
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

EventLoop::ExclusiveScope::ExclusiveScope(EventLoop& loop)
    : mutex_(&loop.impl_->mutex), impl_(loop.impl_.get()),
      previous_(Impl::exclusiveImpl()) {
    mutex_->lock();
    Impl::exclusiveImpl() = loop.impl_.get();
}

EventLoop::ExclusiveScope::~ExclusiveScope() noexcept {
    if (impl_ == nullptr) return;
    Impl::exclusiveImpl() = static_cast<Impl*>(previous_);
    mutex_->unlock();
}

EventLoop::ExclusiveScope EventLoop::useExclusiveExecution() {
    return ExclusiveScope(*this);
}

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
    Impl::Lock lock(*impl_);
    const SimTimeNs current = impl_->timeFor(impl_->currentOwner());
    if (delta > std::numeric_limits<SimTimeNs>::max() - current) {
        throw std::overflow_error("simulation event time overflow");
    }
    return impl_->schedule(current + delta, std::move(callback));
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
    const Impl::SlotRef ref = found->second;
    if (ref.slot >= impl_->slots.size() || impl_->slots[ref.slot].id != ref.id) {
        impl_->live_events.erase(found);
        return false;
    }
    impl_->releaseSlot(ref.slot);
    impl_->live_events.erase(found);
    impl_->discardDeadFront(impl_->globally_observed_events);
    return true;
}

EventRunResult EventLoop::runDueEvents(const SimTimeNs deadline) {
    Impl::Lock lock(*impl_);
    return impl_->runDueEventsLocked(deadline);
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
        if (queue == nullptr || queue->empty() || queue->top().at > deadline) {
            impl_->setTime(owner, deadline);
            result.stopped_at = deadline;
            return result;
        }

        const Impl::Entry entry = queue->top();
        const SimTimeNs next_time = entry.at;
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
        impl_->invoke(entry, result);
    }
}

EventRunResult EventLoop::advanceBy(const SimTimeNs delta) {
    Impl::Lock lock(*impl_);
    if (delta > std::numeric_limits<SimTimeNs>::max() - impl_->shared_now) {
        throw std::overflow_error("simulation clock overflow");
    }
    return impl_->runDueEventsLocked(impl_->shared_now + delta);
}

void EventLoop::clear() noexcept {
    Impl::Lock lock(*impl_);
    impl_->events = {};
    impl_->globally_observed_events = {};
    impl_->owner_events.clear();
    impl_->live_events.clear();
    for (auto& slot : impl_->slots) {
        slot.live = false;
        slot.callback = {};
        slot.locality_guard = {};
    }
    impl_->free_slots.resize(impl_->slots.size());
    std::iota(impl_->free_slots.begin(), impl_->free_slots.end(), 0U);
}

SimTimeNs EventLoop::now() const noexcept {
    Impl::Lock lock(*impl_);
    return impl_->timeFor(impl_->currentOwner());
}

SimTimeNs EventLoop::now(const EventOwner owner) const noexcept {
    Impl::Lock lock(*impl_);
    return impl_->timeFor(owner);
}

std::size_t EventLoop::pending() const noexcept {
    Impl::Lock lock(*impl_);
    return impl_->live_events.size();
}

std::optional<SimTimeNs> EventLoop::nextScheduledTime() {
    Impl::Lock lock(*impl_);
    impl_->discardDeadGlobalFront();
    if (impl_->events.empty()) return std::nullopt;
    return impl_->events.top().at;
}

std::optional<SimTimeNs> EventLoop::nextScheduledTime(const EventOwner owner) {
    Impl::Lock lock(*impl_);
    Impl::Queue* const queue = impl_->ownerQueue(owner);
    if (queue == nullptr || queue->empty()) return std::nullopt;
    return queue->top().at;
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
    impl_->discardDeadFront(impl_->globally_observed_events);
    std::optional<SimTimeNs> result;
    if (owner_queue != nullptr && !owner_queue->empty()) result = owner_queue->top().at;
    if (!impl_->globally_observed_events.empty()) {
        const SimTimeNs global_time = impl_->globally_observed_events.top().at;
        if (!result || global_time < *result) result = global_time;
    }
    return result;
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
