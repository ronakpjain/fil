#include "fil/sim/event_loop.hpp"

#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace fil::sim {

struct EventLoop::Impl {
    struct Event {
        SimTimeNs at{0};
        EventId id{0};
        std::uint64_t sequence{0};
        EventOwner owner{shared_event_owner};
        EventCallback callback;
    };

    struct EventRef {
        SimTimeNs at{0};
        EventId id{0};
        std::uint64_t sequence{0};
    };

    struct Later {
        bool operator()(const Event& left, const Event& right) const noexcept {
            if (left.at != right.at) return left.at > right.at;
            return left.sequence > right.sequence;
        }
    };

    struct RefLater {
        bool operator()(const EventRef& left, const EventRef& right) const noexcept {
            if (left.at != right.at) return left.at > right.at;
            return left.sequence > right.sequence;
        }
    };

    using Queue = std::priority_queue<Event, std::vector<Event>, Later>;
    using RefQueue = std::priority_queue<EventRef, std::vector<EventRef>, RefLater>;

    SimTimeNs now{0};
    EventId next_id{1};
    std::uint64_t next_sequence{0};
    std::size_t maximum_same_time_events{100000};
    EventOwner active_owner{shared_event_owner};
    Queue events;
    std::unordered_map<EventOwner, RefQueue> owner_events;
    std::unordered_map<EventId, EventOwner> live_owners;
    std::unordered_set<EventId> cancelled;

    void discardCancelledFront() {
        while (!events.empty() && cancelled.erase(events.top().id) != 0U) events.pop();
    }

    [[nodiscard]] RefQueue* ownerQueue(const EventOwner owner) {
        const auto found = owner_events.find(owner);
        if (found == owner_events.end()) return nullptr;
        RefQueue& queue = found->second;
        while (!queue.empty() && !live_owners.contains(queue.top().id)) queue.pop();
        return &queue;
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
    : loop_(&loop), previous_(loop.impl_->active_owner) {
    loop.impl_->active_owner = owner;
}

EventLoop::OwnerScope::~OwnerScope() {
    if (loop_ != nullptr) loop_->impl_->active_owner = previous_;
}

EventLoop::OwnerScope::OwnerScope(OwnerScope&& other) noexcept
    : loop_(std::exchange(other.loop_, nullptr)), previous_(other.previous_) {}

EventLoop::OwnerScope EventLoop::useOwner(const EventOwner owner) noexcept {
    return OwnerScope(*this, owner);
}

EventOwner EventLoop::activeOwner() const noexcept {
    return impl_->active_owner;
}

EventId EventLoop::scheduleAfter(const SimTimeNs delta, EventCallback callback) {
    if (delta > std::numeric_limits<SimTimeNs>::max() - impl_->now) {
        throw std::overflow_error("simulation event time overflow");
    }
    return scheduleAt(impl_->now + delta, std::move(callback));
}

EventId EventLoop::scheduleAt(const SimTimeNs at, EventCallback callback) {
    if (!callback) throw std::invalid_argument("simulation event callback is empty");
    if (at < impl_->now) {
        throw std::invalid_argument("cannot schedule a simulation event in the past");
    }
    if (impl_->next_id == 0U) {
        throw std::overflow_error("simulation event identifier space exhausted");
    }

    const EventId id = impl_->next_id++;
    const std::uint64_t sequence = impl_->next_sequence++;
    const EventOwner owner = impl_->active_owner;
    impl_->events.push(Impl::Event{at, id, sequence, owner, std::move(callback)});
    impl_->owner_events[owner].push(Impl::EventRef{at, id, sequence});
    impl_->live_owners.emplace(id, owner);
    return id;
}

bool EventLoop::cancel(const EventId id) noexcept {
    if (id == 0U) return false;
    const auto found = impl_->live_owners.find(id);
    if (found == impl_->live_owners.end()) return false;
    impl_->live_owners.erase(found);
    impl_->cancelled.insert(id);
    return true;
}

EventRunResult EventLoop::runDueEvents(const SimTimeNs deadline) {
    if (deadline < impl_->now) {
        throw std::invalid_argument("cannot run the simulation clock backwards");
    }

    EventRunResult result{0, false, impl_->now};
    SimTimeNs counted_time = 0U;
    std::size_t events_at_counted_time = 0U;

    for (;;) {
        impl_->discardCancelledFront();
        if (impl_->events.empty() || impl_->events.top().at > deadline) {
            impl_->now = deadline;
            result.stopped_at = impl_->now;
            return result;
        }

        const SimTimeNs next_time = impl_->events.top().at;
        if (events_at_counted_time == 0U || next_time != counted_time) {
            counted_time = next_time;
            events_at_counted_time = 0U;
        }
        if (events_at_counted_time >= impl_->maximum_same_time_events) {
            impl_->now = next_time;
            result.same_time_limit_hit = true;
            result.stopped_at = impl_->now;
            return result;
        }

        Impl::Event event = impl_->events.top();
        impl_->events.pop();
        impl_->live_owners.erase(event.id);
        impl_->now = event.at;
        ++events_at_counted_time;
        ++result.events_executed;
        if (event.owner < 64U) {
            result.local_owner_mask |= std::uint64_t{1U} << event.owner;
        } else {
            result.shared_event_executed = true;
        }

        const EventOwner previous_owner = impl_->active_owner;
        impl_->active_owner = event.owner;
        try {
            event.callback();
        } catch (...) {
            impl_->active_owner = previous_owner;
            throw;
        }
        impl_->active_owner = previous_owner;
    }
}

EventRunResult EventLoop::advanceBy(const SimTimeNs delta) {
    if (delta > std::numeric_limits<SimTimeNs>::max() - impl_->now) {
        throw std::overflow_error("simulation clock overflow");
    }
    return runDueEvents(impl_->now + delta);
}

void EventLoop::clear() noexcept {
    impl_->events = {};
    impl_->owner_events.clear();
    impl_->live_owners.clear();
    impl_->cancelled.clear();
}

SimTimeNs EventLoop::now() const noexcept {
    return impl_->now;
}

std::size_t EventLoop::pending() const noexcept {
    return impl_->live_owners.size();
}

std::optional<SimTimeNs> EventLoop::nextScheduledTime() {
    impl_->discardCancelledFront();
    if (impl_->events.empty()) return std::nullopt;
    return impl_->events.top().at;
}

std::optional<SimTimeNs> EventLoop::nextScheduledTime(const EventOwner owner) {
    Impl::RefQueue* const queue = impl_->ownerQueue(owner);
    if (queue == nullptr || queue->empty()) return std::nullopt;
    return queue->top().at;
}

void EventLoop::setMaximumSameTimeEvents(const std::size_t maximum) noexcept {
    impl_->maximum_same_time_events = maximum;
}

std::size_t EventLoop::maximumSameTimeEvents() const noexcept {
    return impl_->maximum_same_time_events;
}

} // namespace fil::sim
