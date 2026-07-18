#include "fil/sim/event_loop.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
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
    EventOwner active_owner{shared_event_owner};
    Queue events;
    std::unordered_map<EventOwner, Queue> owner_events;
    std::unordered_map<EventId, EventPtr> live_events;
    std::unordered_map<EventOwner, SimTimeNs> owner_now;

    [[nodiscard]] SimTimeNs timeFor(const EventOwner owner) const noexcept {
        if (owner == shared_event_owner) return shared_now;
        const auto found = owner_now.find(owner);
        return found == owner_now.end() ? shared_now : found->second;
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

        const EventOwner previous_owner = active_owner;
        active_owner = event->owner;
        try {
            event->callback();
        } catch (...) {
            active_owner = previous_owner;
            throw;
        }
        active_owner = previous_owner;
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
    if (owner != shared_event_owner && !loop.impl_->owner_now.contains(owner)) {
        loop.impl_->owner_now.emplace(owner, loop.impl_->shared_now);
    }
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
    const SimTimeNs current = now();
    if (delta > std::numeric_limits<SimTimeNs>::max() - current) {
        throw std::overflow_error("simulation event time overflow");
    }
    return scheduleAt(current + delta, std::move(callback));
}

EventId EventLoop::scheduleAt(const SimTimeNs at, EventCallback callback) {
    if (!callback) throw std::invalid_argument("simulation event callback is empty");
    if (at < now()) {
        throw std::invalid_argument("cannot schedule a simulation event in the past");
    }
    if (impl_->next_id == 0U) {
        throw std::overflow_error("simulation event identifier space exhausted");
    }

    const EventId id = impl_->next_id++;
    const EventOwner owner = impl_->active_owner;
    auto event = std::make_shared<Impl::Event>(Impl::Event{
        at, id, impl_->next_sequence++, owner, std::move(callback), true,
    });
    impl_->events.push(event);
    impl_->owner_events[owner].push(event);
    impl_->live_events.emplace(id, std::move(event));
    return id;
}

bool EventLoop::cancel(const EventId id) noexcept {
    if (id == 0U) return false;
    const auto found = impl_->live_events.find(id);
    if (found == impl_->live_events.end()) return false;
    found->second->live = false;
    impl_->live_events.erase(found);
    return true;
}

EventRunResult EventLoop::runDueEvents(const SimTimeNs deadline) {
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
            for (auto& [owner, time] : impl_->owner_now) {
                static_cast<void>(owner);
                time = std::max(time, deadline);
            }
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
    if (delta > std::numeric_limits<SimTimeNs>::max() - impl_->shared_now) {
        throw std::overflow_error("simulation clock overflow");
    }
    return runDueEvents(impl_->shared_now + delta);
}

void EventLoop::clear() noexcept {
    impl_->events = {};
    impl_->owner_events.clear();
    impl_->live_events.clear();
}

SimTimeNs EventLoop::now() const noexcept {
    return impl_->timeFor(impl_->active_owner);
}

SimTimeNs EventLoop::now(const EventOwner owner) const noexcept {
    return impl_->timeFor(owner);
}

std::size_t EventLoop::pending() const noexcept {
    return impl_->live_events.size();
}

std::optional<SimTimeNs> EventLoop::nextScheduledTime() {
    impl_->discardDeadGlobalFront();
    if (impl_->events.empty()) return std::nullopt;
    return impl_->events.top()->at;
}

std::optional<SimTimeNs> EventLoop::nextScheduledTime(const EventOwner owner) {
    Impl::Queue* const queue = impl_->ownerQueue(owner);
    if (queue == nullptr || queue->empty()) return std::nullopt;
    return queue->top()->at;
}

void EventLoop::setMaximumSameTimeEvents(const std::size_t maximum) noexcept {
    impl_->maximum_same_time_events = maximum;
}

std::size_t EventLoop::maximumSameTimeEvents() const noexcept {
    return impl_->maximum_same_time_events;
}

} // namespace fil::sim
