#include "fil/sim/event_loop.hpp"

#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace fil::sim {

struct EventLoop::Impl {
    struct Event {
        SimTimeNs at{0};
        EventId id{0};
        std::uint64_t sequence{0};
        EventCallback callback;
    };

    struct Later {
        bool operator()(const Event& left, const Event& right) const noexcept {
            if (left.at != right.at) {
                return left.at > right.at;
            }
            return left.sequence > right.sequence;
        }
    };

    SimTimeNs now{0};
    EventId next_id{1};
    std::uint64_t next_sequence{0};
    std::size_t maximum_same_time_events{100000};
    std::size_t live_events{0};
    std::priority_queue<Event, std::vector<Event>, Later> events;
    std::unordered_set<EventId> cancelled;

    void discardCancelledFront() {
        while (!events.empty() && cancelled.erase(events.top().id) != 0U) {
            events.pop();
        }
    }
};

EventLoop::EventLoop(const std::size_t maximum_same_time_events)
    : impl_(std::make_unique<Impl>()) {
    impl_->maximum_same_time_events = maximum_same_time_events;
}

EventLoop::~EventLoop() = default;
EventLoop::EventLoop(EventLoop&&) noexcept = default;
EventLoop& EventLoop::operator=(EventLoop&&) noexcept = default;

EventId EventLoop::scheduleAfter(const SimTimeNs delta, EventCallback callback) {
    if (delta > std::numeric_limits<SimTimeNs>::max() - impl_->now) {
        throw std::overflow_error("simulation event time overflow");
    }
    return scheduleAt(impl_->now + delta, std::move(callback));
}

EventId EventLoop::scheduleAt(const SimTimeNs at, EventCallback callback) {
    if (!callback) {
        throw std::invalid_argument("simulation event callback is empty");
    }
    if (at < impl_->now) {
        throw std::invalid_argument("cannot schedule a simulation event in the past");
    }
    if (impl_->next_id == 0) {
        throw std::overflow_error("simulation event identifier space exhausted");
    }

    const EventId id = impl_->next_id++;
    impl_->events.push(Impl::Event{at, id, impl_->next_sequence++, std::move(callback)});
    ++impl_->live_events;
    return id;
}

bool EventLoop::cancel(const EventId id) noexcept {
    if (id == 0 || impl_->cancelled.contains(id)) {
        return false;
    }

    // The queue is intentionally not searched. A stale identifier is harmless;
    // live_events is decremented only if the id is found while compacting below.
    bool found = false;
    auto copy = impl_->events;
    while (!copy.empty()) {
        if (copy.top().id == id) {
            found = true;
            break;
        }
        copy.pop();
    }
    if (!found) {
        return false;
    }
    impl_->cancelled.insert(id);
    --impl_->live_events;
    return true;
}

EventRunResult EventLoop::runDueEvents(const SimTimeNs deadline) {
    if (deadline < impl_->now) {
        throw std::invalid_argument("cannot run the simulation clock backwards");
    }

    EventRunResult result{0, false, impl_->now};
    SimTimeNs counted_time = 0;
    std::size_t events_at_counted_time = 0;

    for (;;) {
        impl_->discardCancelledFront();
        if (impl_->events.empty() || impl_->events.top().at > deadline) {
            impl_->now = deadline;
            result.stopped_at = impl_->now;
            return result;
        }

        const SimTimeNs next_time = impl_->events.top().at;
        if (events_at_counted_time == 0 || next_time != counted_time) {
            counted_time = next_time;
            events_at_counted_time = 0;
        }
        if (events_at_counted_time >= impl_->maximum_same_time_events) {
            impl_->now = next_time;
            result.same_time_limit_hit = true;
            result.stopped_at = impl_->now;
            return result;
        }

        Impl::Event event = impl_->events.top();
        impl_->events.pop();
        --impl_->live_events;
        impl_->now = event.at;
        ++events_at_counted_time;
        ++result.events_executed;
        event.callback();
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
    impl_->cancelled.clear();
    impl_->live_events = 0;
}

SimTimeNs EventLoop::now() const noexcept {
    return impl_->now;
}

std::size_t EventLoop::pending() const noexcept {
    return impl_->live_events;
}

std::optional<SimTimeNs> EventLoop::nextScheduledTime() {
    impl_->discardCancelledFront();
    if (impl_->events.empty()) return std::nullopt;
    return impl_->events.top().at;
}

void EventLoop::setMaximumSameTimeEvents(const std::size_t maximum) noexcept {
    impl_->maximum_same_time_events = maximum;
}

std::size_t EventLoop::maximumSameTimeEvents() const noexcept {
    return impl_->maximum_same_time_events;
}

} // namespace fil::sim
