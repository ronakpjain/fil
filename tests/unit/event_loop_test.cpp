#include "fil/sim/event_loop.hpp"
#include "fil/sim/trace.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <limits>
#include <optional>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <thread>
#include <vector>

namespace {

TEST(EventLoopTest, OrdersEventsDeterministically) {
    fil::sim::EventLoop loop;
    std::vector<std::int32_t> order;
    static_cast<void>(loop.scheduleAt(20, [&]() { order.push_back(3); }));
    static_cast<void>(loop.scheduleAt(10, [&]() {
        order.push_back(1);
        static_cast<void>(loop.scheduleAfter(0, [&]() { order.push_back(4); }));
    }));
    static_cast<void>(loop.scheduleAt(10, [&]() { order.push_back(2); }));

    const auto first = loop.runDueEvents(10);
    EXPECT_TRUE(first.events_executed == 3) << "event loop runs nested same-time events";
    EXPECT_TRUE(order == std::vector<std::int32_t>({1, 2, 4}))
        << "event loop orders by time then insertion sequence";
    EXPECT_TRUE(loop.now() == 10 && loop.pending() == 1) << "event loop retains future events";

    const auto second = loop.advanceBy(10);
    EXPECT_TRUE(second.events_executed == 1 && order.back() == 3)
        << "event loop advances relative time";
}

TEST(EventLoopTest, ExclusiveExecutionSupportsNestedPublicCallsAndPreservesBehavior) {
    fil::sim::EventLoop loop;
    std::vector<int> order;
    std::uint32_t callbacks = 0U;
    const auto canceled = loop.scheduleAt(8U, [&] { ++callbacks; });
    {
        auto exclusive = loop.useExclusiveExecution();
        EXPECT_EQ(loop.now(), 0U);
        EXPECT_TRUE(loop.cancel(canceled));
        static_cast<void>(loop.scheduleAt(3U, [&] {
            ++callbacks;
            order.push_back(1);
            EXPECT_EQ(loop.now(), 3U);
            static_cast<void>(loop.scheduleAfter(0U, [&] {
                ++callbacks;
                order.push_back(2);
            }));
        }));
        const auto run = loop.runDueEvents(3U);
        EXPECT_EQ(run.events_executed, 2U);
        EXPECT_EQ(loop.now(), 3U);
        EXPECT_EQ(loop.pending(), 0U);
    }
    EXPECT_EQ(callbacks, 2U);
    EXPECT_EQ(order, (std::vector<int>{1, 2}));
    EXPECT_EQ(loop.now(), 3U);
}

TEST(EventLoopTest, ExclusiveExecutionNestedScopesRestorePriorLoop) {
    fil::sim::EventLoop first;
    fil::sim::EventLoop second;
    {
        auto outer = first.useExclusiveExecution();
        {
            auto same = first.useExclusiveExecution();
            static_cast<void>(first.scheduleAfter(1U, [] {}));
        }
        {
            auto other = second.useExclusiveExecution();
            static_cast<void>(second.scheduleAfter(1U, [] {}));
            // Calls on the first loop must relock while the second loop is current.
            EXPECT_EQ(first.pending(), 1U);
        }
        static_cast<void>(first.scheduleAfter(1U, [] {}));
        EXPECT_EQ(first.pending(), 2U);
    }
    EXPECT_EQ(first.pending(), 2U);
    EXPECT_EQ(second.pending(), 1U);
}

TEST(EventLoopTest, ExclusiveExecutionFollowsAnImplementationMovedToAnotherLoop) {
    fil::sim::EventLoop original;
    std::optional<fil::sim::EventLoop> destination;
    {
        auto exclusive = original.useExclusiveExecution();
        destination.emplace(std::move(original));
        static_cast<void>(destination->scheduleAfter(1U, [] {}));
        EXPECT_EQ(destination->runDueEvents(1U).events_executed, 1U);
    }
    // The implementation survives its lease even though its original wrapper
    // no longer owns it. Queries after release must use ordinary locking again.
    EXPECT_EQ(destination->now(), 1U);
}

TEST(EventLoopTest, ExclusiveExecutionExceptionUnwindReleasesLease) {
    fil::sim::EventLoop loop;
    EXPECT_THROW(([&] {
        auto exclusive = loop.useExclusiveExecution();
        static_cast<void>(loop.scheduleAfter(0U, [] {}));
        throw std::runtime_error("unwind");
    }()), std::runtime_error);
    EXPECT_EQ(loop.pending(), 1U);
}

TEST(EventLoopTest, StandaloneExecutionStillSerializesOtherThreadScheduling) {
    fil::sim::EventLoop loop;
    std::atomic<bool> callback_entered{false};
    std::atomic<bool> release_callback{false};
    std::atomic<bool> attempted{false};
    std::atomic<bool> completed{false};
    static_cast<void>(loop.scheduleAt(0U, [&] {
        callback_entered.store(true, std::memory_order_release);
        while (!release_callback.load(std::memory_order_acquire)) std::this_thread::yield();
    }));
    std::thread runner([&] { static_cast<void>(loop.runDueEvents(0U)); });
    while (!callback_entered.load(std::memory_order_acquire)) std::this_thread::yield();
    std::thread scheduler([&] {
        attempted.store(true, std::memory_order_release);
        static_cast<void>(loop.scheduleAfter(0U, [] {}));
        completed.store(true, std::memory_order_release);
    });
    while (!attempted.load(std::memory_order_acquire)) std::this_thread::yield();
    EXPECT_FALSE(completed.load(std::memory_order_acquire));
    release_callback.store(true, std::memory_order_release);
    runner.join();
    scheduler.join();
    EXPECT_TRUE(completed.load(std::memory_order_acquire));
    EXPECT_EQ(loop.pending(), 1U);
}

TEST(EventLoopTest, ExclusiveExecutionBlocksOtherThreadsUntilReleased) {
    fil::sim::EventLoop loop;
    std::atomic<bool> attempted{false};
    std::atomic<bool> completed{false};
    std::thread worker;
    {
        auto exclusive = loop.useExclusiveExecution();
        worker = std::thread([&] {
            attempted.store(true, std::memory_order_release);
            static_cast<void>(loop.scheduleAfter(0U, [] {}));
            completed.store(true, std::memory_order_release);
        });
        while (!attempted.load(std::memory_order_acquire)) std::this_thread::yield();
        // The worker has announced its attempt while this scope owns the mutex.
        EXPECT_FALSE(completed.load(std::memory_order_acquire));
        EXPECT_EQ(loop.pending(), 0U);
    }
    worker.join();
    EXPECT_TRUE(completed.load(std::memory_order_acquire));
    EXPECT_EQ(loop.pending(), 1U);
}

TEST(EventLoopTest, AdvanceByMatchesRunningDueEventsAtTheEquivalentDeadline) {
    fil::sim::EventLoop advanced;
    fil::sim::EventLoop absolute;
    std::vector<std::int32_t> advanced_order;
    std::vector<std::int32_t> absolute_order;
    for (auto* loop : {&advanced, &absolute}) {
        auto& order = loop == &advanced ? advanced_order : absolute_order;
        static_cast<void>(loop->scheduleAt(3U, [&order] { order.push_back(1); }));
        static_cast<void>(loop->scheduleAt(8U, [&order] { order.push_back(2); }));
    }
    static_cast<void>(advanced.runDueEvents(5U));
    static_cast<void>(absolute.runDueEvents(5U));
    const auto relative_result = advanced.advanceBy(4U);
    const auto absolute_result = absolute.runDueEvents(9U);
    EXPECT_EQ(relative_result.events_executed, absolute_result.events_executed);
    EXPECT_EQ(relative_result.same_time_limit_hit, absolute_result.same_time_limit_hit);
    EXPECT_EQ(relative_result.stopped_at, absolute_result.stopped_at);
    EXPECT_EQ(relative_result.local_owner_mask, absolute_result.local_owner_mask);
    EXPECT_EQ(relative_result.shared_event_executed, absolute_result.shared_event_executed);
    EXPECT_EQ(advanced_order, absolute_order);
    EXPECT_EQ(advanced.pending(), absolute.pending());
}

TEST(EventLoopTest, AdvanceByRejectsClockOverflow) {
    fil::sim::EventLoop loop;
    static_cast<void>(loop.runDueEvents(std::numeric_limits<fil::sim::SimTimeNs>::max()));
    EXPECT_THROW(static_cast<void>(loop.advanceBy(1U)), std::overflow_error);
    EXPECT_EQ(loop.now(), std::numeric_limits<fil::sim::SimTimeNs>::max());
}

TEST(EventLoopTest, AdvanceByCallbackCanScheduleZeroDelayEvents) {
    fil::sim::EventLoop loop;
    std::vector<std::int32_t> order;
    static_cast<void>(loop.scheduleAt(4U, [&] {
        order.push_back(1);
        static_cast<void>(loop.scheduleAfter(0U, [&] { order.push_back(2); }));
    }));
    const auto result = loop.advanceBy(4U);
    EXPECT_EQ(result.events_executed, 2U);
    EXPECT_EQ(order, (std::vector<std::int32_t>{1, 2}));
    EXPECT_EQ(loop.now(), 4U);
}

struct BuiltinProbe {
    int calls{0};
    int tag{0};
    std::vector<int>* order{nullptr};
};

void builtinProbe(void* context) {
    auto* probe = static_cast<BuiltinProbe*>(context);
    ++probe->calls;
    probe->order->push_back(probe->tag);
}

TEST(EventLoopTest, BuiltinCallbacksDispatchDirectlyAndKeepReferenceOrder) {
    fil::sim::EventLoop loop;
    std::vector<int> order;
    BuiltinProbe first{0, 1, &order};
    BuiltinProbe second{0, 2, &order};
    static_cast<void>(loop.scheduleBuiltinAt(4U, {&builtinProbe, &first}));
    static_cast<void>(loop.scheduleAt(4U, [&] { order.push_back(3); }));
    static_cast<void>(loop.scheduleBuiltinAt(4U, {&builtinProbe, &second}));
    const auto result = loop.runDueEvents(4U);
    EXPECT_EQ(result.events_executed, 3U);
    EXPECT_EQ(order, (std::vector<int>{1, 3, 2}));
    EXPECT_EQ(first.calls, 1);
    EXPECT_EQ(second.calls, 1);
    EXPECT_EQ(loop.pending(), 0U);
}

TEST(EventLoopTest, ScheduledBuiltinReplacementRetiresIdentityBeforeCallback) {
    fil::sim::EventLoop loop;
    std::vector<int> order;
    BuiltinProbe first{0, 1, &order};
    BuiltinProbe second{0, 2, &order};
    fil::sim::ScheduledEvent handle(&loop);
    static_cast<void>(handle.scheduleBuiltinAt(9U, {&builtinProbe, &first}));
    EXPECT_TRUE(handle.pending());
    static_cast<void>(handle.scheduleBuiltinAt(9U, {&builtinProbe, &second}));
    EXPECT_TRUE(handle.pending());
    static_cast<void>(loop.runDueEvents(9U));
    EXPECT_EQ(order, (std::vector<int>{2}));
    EXPECT_EQ(first.calls, 0);
    EXPECT_EQ(second.calls, 1);
    EXPECT_FALSE(handle.pending());
}

TEST(EventLoopTest, CancelledSlotsAreReusedWithoutResurrectingOldEvents) {
    fil::sim::EventLoop loop;
    std::vector<int> order;
    const auto cancelled = loop.scheduleAt(5U, [&] { order.push_back(1); });
    EXPECT_TRUE(loop.cancel(cancelled));
    // The replacement may reuse the slot of the cancelled event; the stale
    // heap entry must not run it again or reorder the new event.
    static_cast<void>(loop.scheduleAt(4U, [&] { order.push_back(2); }));
    static_cast<void>(loop.scheduleAt(5U, [&] { order.push_back(3); }));
    EXPECT_FALSE(loop.cancel(cancelled));
    const auto result = loop.runDueEvents(5U);
    EXPECT_EQ(result.events_executed, 2U);
    EXPECT_EQ(order, (std::vector<int>{2, 3}));
    EXPECT_EQ(loop.pending(), 0U);
}

TEST(EventLoopTest, ClearReturnsSlotsForReuseAndKeepsTheClock) {
    fil::sim::EventLoop loop;
    static_cast<void>(loop.scheduleAt(2U, [] {}));
    static_cast<void>(loop.runDueEvents(2U));
    EXPECT_EQ(loop.now(), 2U);
    loop.clear();
    EXPECT_EQ(loop.pending(), 0U);
    EXPECT_FALSE(loop.nextScheduledTime().has_value());
    std::vector<int> order;
    static_cast<void>(loop.scheduleAt(2U, [&] { order.push_back(1); }));
    static_cast<void>(loop.scheduleAt(3U, [&] { order.push_back(2); }));
    EXPECT_EQ(loop.runDueEvents(3U).events_executed, 2U);
    EXPECT_EQ(order, (std::vector<int>{1, 2}));
}

TEST(EventLoopTest, SameTimeReplacementKeepsQueueOrder) {
    fil::sim::EventLoop loop;
    std::vector<int> order;
    static_cast<void>(loop.scheduleAt(7U, [&] { order.push_back(1); }));
    fil::sim::ScheduledEvent handle(&loop);
    static_cast<void>(handle.scheduleAt(7U, [&] { order.push_back(2); }));
    // Replacing the handle's callback must not move an already scheduled
    // earlier same-time event, even though the replacement reuses a slot.
    static_cast<void>(handle.scheduleAt(7U, [&] { order.push_back(3); }));
    EXPECT_EQ(loop.runDueEvents(7U).events_executed, 2U);
    EXPECT_EQ(order, (std::vector<int>{1, 3}));
}

TEST(EventLoopTest, TracksLocalAndSharedEventOwnership) {
    fil::sim::EventLoop loop;
    std::vector<fil::sim::EventOwner> observed;
    {
        auto owner = loop.useOwner(3U);
        static_cast<void>(loop.scheduleAt(5U, [&]() {
            observed.push_back(loop.activeOwner());
            static_cast<void>(loop.scheduleAfter(0U, [&]() {
                observed.push_back(loop.activeOwner());
            }));
        }));
    }
    EXPECT_TRUE(loop.activeOwner() == fil::sim::shared_event_owner)
        << "event owner scope restores the shared domain";
    static_cast<void>(loop.scheduleAt(5U, [&]() {
        observed.push_back(loop.activeOwner());
    }));
    EXPECT_TRUE(loop.nextScheduledTime(3U) == 5U &&
                loop.nextScheduledTime(fil::sim::shared_event_owner) == 5U &&
                !loop.nextScheduledTime(4U))
        << "owner queues expose independent event horizons";

    const auto result = loop.runDueEvents(5U);
    EXPECT_TRUE(result.events_executed == 3U &&
                result.local_owner_mask == (std::uint64_t{1U} << 3U) &&
                result.shared_event_executed)
        << "event runs report local and shared ownership";
    EXPECT_TRUE((observed ==
                 std::vector<fil::sim::EventOwner>{
                     3U,
                     fil::sim::shared_event_owner,
                     3U,
                 }))
        << "nested events inherit their callback owner deterministically";
}

TEST(EventLoopTest, ObservationBarrierReportsDefaultGlobalEventsAndNestedOrder) {
    fil::sim::EventLoop loop;
    using Observation = fil::sim::EventObservation;
    std::vector<std::tuple<fil::sim::SimTimeNs, fil::sim::EventOwner, Observation>> observed;
    static_cast<void>(loop.exchangeObservationBarrier([&](const auto time, const auto owner, const auto kind) {
        observed.emplace_back(time, owner, kind);
    }));
    std::vector<std::int32_t> callbacks;
    {
        auto scope = loop.useOwner(7U);
        static_cast<void>(loop.scheduleAt(4U, [&] {
            callbacks.push_back(1);
            static_cast<void>(loop.scheduleAfter(0U, [&] { callbacks.push_back(3); }));
        }));
        static_cast<void>(loop.scheduleAt(4U, [&] { callbacks.push_back(2); }));
    }
    EXPECT_EQ(loop.runDueEvents(4U).events_executed, 3U);
    EXPECT_EQ(callbacks, (std::vector<std::int32_t>{1, 2, 3}));
    EXPECT_EQ(observed.size(), 3U);
    EXPECT_EQ(observed[0], (std::tuple{4U, 7U, Observation::global}));
    EXPECT_EQ(observed[1], (std::tuple{4U, 7U, Observation::global}));
    EXPECT_EQ(observed[2], (std::tuple{4U, 7U, Observation::global}));
}

TEST(EventLoopTest, SelfClearingObserverSnapshotSurvivesNestedEventScheduling) {
    fil::sim::EventLoop loop;
    std::uint32_t observations = 0U;
    std::vector<std::int32_t> callbacks;
    static_cast<void>(loop.exchangeObservationBarrier([&](auto, auto, auto) {
        ++observations;
        const auto previous = loop.exchangeObservationBarrier({});
        EXPECT_TRUE(static_cast<bool>(previous));
        static_cast<void>(loop.scheduleAfter(0U, [&] { callbacks.push_back(2); }));
    }));
    static_cast<void>(loop.scheduleAt(1U, [&] { callbacks.push_back(1); }));

    EXPECT_EQ(loop.runDueEvents(1U).events_executed, 2U);
    EXPECT_EQ(observations, 1U);
    EXPECT_EQ(callbacks, (std::vector<std::int32_t>{1, 2}));
}

TEST(EventLoopTest, ObservationBarrierExceptionRestoresInvocationOwner) {
    fil::sim::EventLoop loop;
    static_cast<void>(loop.exchangeObservationBarrier([](auto, auto, auto) {
        throw std::runtime_error("observer failed");
    }));
    {
        auto scope = loop.useOwner(9U);
        static_cast<void>(loop.scheduleAt(3U, [] {}));
    }
    EXPECT_THROW(static_cast<void>(loop.runDueEvents(3U)), std::runtime_error);
    EXPECT_EQ(loop.activeOwner(), fil::sim::shared_event_owner);
}

TEST(EventLoopTest, OwnedInvocationObservesTimestampAndOwnerBeforeCallback) {
    fil::sim::EventLoop loop;
    std::vector<std::int32_t> sequence;
    static_cast<void>(loop.exchangeObservationBarrier([&](const auto time, const auto owner, const auto kind) {
        EXPECT_EQ(time, 6U);
        EXPECT_EQ(owner, 5U);
        EXPECT_EQ(kind, fil::sim::EventObservation::global);
        sequence.push_back(1);
    }));
    {
        auto scope = loop.useOwner(5U);
        static_cast<void>(loop.scheduleAt(6U, [&] {
            EXPECT_EQ(loop.activeOwner(), 5U);
            sequence.push_back(2);
        }));
    }
    EXPECT_EQ(loop.runOwnedEvents(5U, 6U).events_executed, 1U);
    EXPECT_EQ(sequence, (std::vector<std::int32_t>{1, 2}));
    EXPECT_EQ(loop.activeOwner(), fil::sim::shared_event_owner);
}

TEST(EventLoopTest, ObservationHorizonsIncludeOwnerAndGlobalEventsAndPruneCancellation) {
    fil::sim::EventLoop loop;
    fil::sim::EventId canceled = 0U;
    {
        auto scope = loop.useOwner(2U);
        canceled = loop.scheduleAt(1U, [] {});
        static_cast<void>(loop.scheduleAt(8U, [] {}));
    }
    EXPECT_TRUE(loop.cancel(canceled));
    EXPECT_EQ(loop.nextObservationTime(2U), 8U);
    EXPECT_EQ(loop.nextObservationTime(3U), 8U);
    static_cast<void>(loop.scheduleAt(5U, [] {}));
    EXPECT_EQ(loop.nextObservationTime(3U), 5U);
    EXPECT_EQ(loop.nextObservationTime(fil::sim::shared_event_owner), 5U);
}

TEST(EventLoopTest, InsertionPrunesGloballyRetiredOwnerCallbacks) {
    for (const fil::sim::EventOwner owner : {0U, 63U, 64U, 1000U}) {
        SCOPED_TRACE(owner);
        fil::sim::EventLoop loop;
        std::vector<std::int32_t> order;
        auto payload = std::make_shared<std::int32_t>(1);
        const std::weak_ptr<std::int32_t> lifetime = payload;
        {
            auto scope = loop.useOwner(owner);
            static_cast<void>(loop.scheduleAt(5U, [payload, &order] {
                order.push_back(*payload);
            }));
        }
        payload.reset();
        EXPECT_EQ(loop.runDueEvents(5U).events_executed, 1U);
        {
            auto scope = loop.useOwner(owner);
            static_cast<void>(loop.scheduleAt(10U, [&order] { order.push_back(2); }));
            static_cast<void>(loop.scheduleAt(10U, [&order] { order.push_back(3); }));
        }
        EXPECT_TRUE(lifetime.expired()) << "retired callback captures must not accumulate";
        EXPECT_EQ(loop.pending(), 2U);
        EXPECT_EQ(loop.nextScheduledTime(owner), 10U);
        EXPECT_EQ(loop.runOwnedEvents(owner, 10U).events_executed, 2U);
        EXPECT_EQ(order, (std::vector<std::int32_t>{1, 2, 3}));
        EXPECT_EQ(loop.runDueEvents(10U).events_executed, 0U);
    }
}

TEST(EventLoopTest, SharedRendezvousLeavesOwnerClockAndCallbacksUntouched) {
    fil::sim::EventLoop loop(2U);
    constexpr auto shared = fil::sim::shared_event_owner;
    std::vector<int> order;
    {
        auto owner = loop.useOwner(4U);
        static_cast<void>(loop.scheduleAt(21U, [&] { order.push_back(4); }));
    }
    static_cast<void>(loop.runOwnedEvents(4U, 20U));
    const auto canceled = loop.scheduleAt(3U, [&] { order.push_back(99); });
    EXPECT_TRUE(loop.cancel(canceled));
    static_cast<void>(loop.scheduleAt(5U, [&] {
        order.push_back(1);
        EXPECT_EQ(loop.now(4U), 20U);
        {
            auto owner = loop.useOwner(4U);
            // Relative owner work is clamped to the owner's already-ahead clock.
            static_cast<void>(loop.scheduleAfter(0U, [&] { order.push_back(3); }));
            EXPECT_THROW(static_cast<void>(loop.scheduleAt(5U, [] {})), std::invalid_argument);
        }
        static_cast<void>(loop.scheduleAt(5U, [&] { order.push_back(2); }));
    }));

    const auto result = loop.runSharedEvents(5U);
    EXPECT_EQ(result.events_executed, 2U);
    EXPECT_TRUE(result.shared_event_executed);
    EXPECT_EQ(order, (std::vector<int>{1, 2}));
    EXPECT_EQ(loop.now(shared), 5U);
    EXPECT_EQ(loop.now(4U), 20U);
    EXPECT_EQ(loop.nextScheduledTime(4U), 20U);
    EXPECT_EQ(loop.pending(), 2U);
    EXPECT_EQ(loop.runOwnedEvents(4U, 21U).events_executed, 2U);
    EXPECT_EQ(order, (std::vector<int>{1, 2, 3, 4}));
    EXPECT_EQ(loop.now(4U), 21U);
}

TEST(EventLoopTest, SharedRendezvousReportsSameTimeLivelockWithoutRunningOwnerWork) {
    fil::sim::EventLoop loop(2U);
    std::uint32_t calls = 0U;
    {
        auto owner = loop.useOwner(1U);
        static_cast<void>(loop.scheduleAt(0U, [&] { calls += 100U; }));
    }
    std::function<void()> repeat;
    repeat = [&] {
        ++calls;
        static_cast<void>(loop.scheduleAfter(0U, repeat));
    };
    static_cast<void>(loop.scheduleAt(0U, repeat));
    const auto result = loop.runSharedEvents(0U);
    EXPECT_TRUE(result.same_time_limit_hit);
    EXPECT_EQ(result.events_executed, 2U);
    EXPECT_EQ(calls, 2U);
    EXPECT_EQ(loop.now(1U), 0U);
    EXPECT_EQ(loop.pending(), 2U);
    loop.clear();
}

TEST(EventLoopTest, CountsOnlySuccessfullyDispatchedCallbacksAcrossDenseAndSparseOwners) {
    for (const fil::sim::EventOwner owner : {0U, 63U, 64U, 1000U}) {
        SCOPED_TRACE(owner);
        fil::sim::EventLoop loop;
        std::vector<int> order;
        EXPECT_EQ(loop.executedCallbacks(), 0U);
        const auto canceled = [&] {
            auto scope = loop.useOwner(owner);
            return loop.scheduleAt(4U, [&] { order.push_back(99); });
        }();
        EXPECT_TRUE(loop.cancel(canceled));
        {
            auto scope = loop.useOwner(owner);
            static_cast<void>(loop.scheduleAt(5U, [&] {
                order.push_back(1);
                static_cast<void>(loop.scheduleAfter(0U, [&] { order.push_back(3); }));
            }));
            static_cast<void>(loop.scheduleAt(5U, [&] { order.push_back(2); }));
        }
        static_cast<void>(loop.scheduleAt(5U, [&] { order.push_back(4); }));

        EXPECT_EQ(loop.runOwnedEvents(owner, 5U).events_executed, 3U);
        EXPECT_EQ(loop.executedCallbacks(), 3U);
        EXPECT_EQ(order, (std::vector<int>{1, 2, 3}));
        EXPECT_EQ(loop.runSharedEvents(5U).events_executed, 1U);
        EXPECT_EQ(loop.executedCallbacks(), 4U);
        EXPECT_EQ(order, (std::vector<int>{1, 2, 3, 4}));
    }

    fil::sim::EventLoop throwing_loop;
    static_cast<void>(throwing_loop.scheduleAt(1U, [] {
        throw std::runtime_error("callback failed");
    }));
    EXPECT_THROW(static_cast<void>(throwing_loop.runDueEvents(1U)), std::runtime_error);
    EXPECT_EQ(throwing_loop.executedCallbacks(), 0U);
}

TEST(EventLoopTest, OwnerRunRejectsReentrantDispatchButAllowsScheduling) {
    fil::sim::EventLoop loop;
    constexpr fil::sim::EventOwner owner_id = 2U;
    std::vector<int> order;
    {
        auto owner = loop.useOwner(owner_id);
        static_cast<void>(loop.scheduleAt(3U, [&] {
            EXPECT_THROW(static_cast<void>(loop.runOwnedEvents(owner_id, 12U)), std::logic_error);
            EXPECT_THROW(static_cast<void>(loop.runOwnedEvents(9U, 12U)), std::logic_error);
            EXPECT_THROW(static_cast<void>(loop.runSharedEvents(12U)), std::logic_error);
            EXPECT_THROW(static_cast<void>(loop.runDueEvents(12U)), std::logic_error);
            EXPECT_THROW(static_cast<void>(loop.advanceBy(12U)), std::logic_error);
            EXPECT_EQ(loop.now(owner_id), 3U);
            const auto canceled = loop.scheduleAfter(20U, [&] { order.push_back(99); });
            EXPECT_TRUE(loop.cancel(canceled));
            static_cast<void>(loop.scheduleAfter(10U, [&] { order.push_back(1); }));
        }));
    }

    const auto result = loop.runOwnedEvents(owner_id, 6U);
    EXPECT_EQ(result.events_executed, 1U);
    EXPECT_EQ(order, (std::vector<int>{}));
    EXPECT_EQ(loop.now(owner_id), 6U);
    EXPECT_EQ(loop.runOwnedEvents(owner_id, 13U).events_executed, 1U);
    EXPECT_EQ(order, (std::vector<int>{1}));
    EXPECT_EQ(loop.now(owner_id), 13U);
}

TEST(EventLoopTest, OwnerDispatchGuardUnwindsAfterCallbackException) {
    fil::sim::EventLoop loop;
    constexpr fil::sim::EventOwner owner_id = 5U;
    {
        auto owner = loop.useOwner(owner_id);
        static_cast<void>(loop.scheduleAt(2U, [] {
            throw std::runtime_error("owner callback failed");
        }));
    }
    EXPECT_THROW(static_cast<void>(loop.runOwnedEvents(owner_id, 8U)), std::runtime_error);
    EXPECT_EQ(loop.runDueEvents(0U).events_executed, 0U);
    EXPECT_EQ(loop.runOwnedEvents(owner_id, 2U).events_executed, 0U);
    EXPECT_EQ(loop.now(owner_id), 2U);
}

TEST(EventLoopTest, LaneDomainsRunConcurrentlyPreserveHandlesAndReturnToExactMode) {
    fil::sim::EventLoop loop;
    ASSERT_EQ(loop.runOwnedEvents(0U, 10U).stopped_at, 10U);
    ASSERT_EQ(loop.runOwnedEvents(1U, 15U).stopped_at, 15U);

    std::vector<int> first_order;
    std::vector<int> second_order;
    std::vector<int> exact_order;
    fil::sim::ScheduledEvent canceled_handle(&loop);
    fil::sim::ScheduledEvent builtin_handle(&loop);
    BuiltinProbe builtin{0, 2, &second_order};
    {
        auto owner = loop.useOwner(0U);
        static_cast<void>(canceled_handle.scheduleAt(14U, [&] { first_order.push_back(99); }));
    }
    {
        auto owner = loop.useOwner(1U);
        static_cast<void>(builtin_handle.scheduleBuiltinAt(18U, {&builtinProbe, &builtin}));
    }
    const auto migrated_id = canceled_handle.id();

    {
        auto domains = loop.useLaneDomains(2U);
        std::atomic<unsigned> ready{0U};
        std::atomic<bool> start{false};
        auto root_exclusive = loop.useExclusiveExecution();
        std::thread first([&] {
            auto lane = loop.useLaneDomain(0U);
            auto exclusive = loop.useExclusiveExecution();
            ready.fetch_add(1U, std::memory_order_release);
            while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
            EXPECT_EQ(loop.now(0U), 10U);
            EXPECT_THROW(static_cast<void>(loop.now(1U)), std::logic_error);
            canceled_handle.cancel();
            EXPECT_EQ(canceled_handle.id(), 0U);
            {
                auto owner = loop.useOwner(0U);
                static_cast<void>(loop.scheduleAt(12U, [&] { first_order.push_back(1); }));
            }
            EXPECT_EQ(loop.runOwnedEvents(0U, 12U).events_executed, 1U);
            EXPECT_EQ(loop.now(0U), 12U);
        });
        std::thread second([&] {
            auto lane = loop.useLaneDomain(1U);
            auto exclusive = loop.useExclusiveExecution();
            ready.fetch_add(1U, std::memory_order_release);
            while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
            EXPECT_EQ(loop.now(1U), 15U);
            EXPECT_THROW(static_cast<void>(loop.runDueEvents(100U)), std::logic_error);
            EXPECT_THROW(static_cast<void>(loop.runSharedEvents(100U)), std::logic_error);
            EXPECT_THROW(static_cast<void>(loop.scheduleAfter(1U, [] {})), std::logic_error);
            EXPECT_EQ(loop.runOwnedEvents(1U, 18U).events_executed, 1U);
            EXPECT_EQ(loop.now(1U), 18U);
        });
        while (ready.load(std::memory_order_acquire) != 2U) std::this_thread::yield();
        // The coordinator holds the root lease while workers use only their
        // private lane mutexes and still make progress.
        start.store(true, std::memory_order_release);
        first.join();
        second.join();
        EXPECT_EQ(loop.pending(), 0U);
        EXPECT_EQ(loop.executedCallbacks(), 2U);
        domains.close();
    }

    EXPECT_FALSE(canceled_handle.pending());
    EXPECT_FALSE(builtin_handle.pending());
    EXPECT_EQ(builtin.calls, 1);
    EXPECT_EQ(first_order, (std::vector<int>{1}));
    EXPECT_EQ(second_order, (std::vector<int>{2}));
    EXPECT_EQ(loop.now(0U), 12U);
    EXPECT_EQ(loop.now(1U), 18U);
    EXPECT_TRUE(loop.cancel(migrated_id) == false);
    {
        auto owner = loop.useOwner(0U);
        static_cast<void>(loop.scheduleAt(13U, [&] { exact_order.push_back(3); }));
    }
    EXPECT_EQ(loop.runOwnedEvents(0U, 13U).events_executed, 1U);
    EXPECT_EQ(exact_order, (std::vector<int>{3}));

    // A subsequent session gets fresh sequence/ID ranges and migrates the
    // newly scheduled event back to the ordinary serial queues on exit.
    {
        auto owner = loop.useOwner(0U);
        static_cast<void>(loop.scheduleAt(14U, [&] {
            exact_order.push_back(4);
            EXPECT_EQ(loop.now(), 14U);
        }));
    }
    {
        auto domains = loop.useLaneDomains(2U);
        EXPECT_EQ(loop.runOwnedEvents(0U, 14U).events_executed, 1U);
        std::thread worker([&] {
            auto lane = loop.useLaneDomain(0U);
            EXPECT_EQ(loop.runOwnedEvents(0U, 14U).events_executed, 0U);
        });
        worker.join();
        domains.close();
    }
    EXPECT_EQ(loop.now(0U), 14U);
    EXPECT_EQ(exact_order, (std::vector<int>{3, 4}));
}

TEST(EventLoopTest, OrdinaryConcurrentSchedulingSerializesOwnerContextSafely) {
    fil::sim::EventLoop loop;
    std::atomic<unsigned> ready{0U};
    std::atomic<bool> start{false};
    std::atomic<unsigned> calls{0U};
    auto worker = [&] {
        ready.fetch_add(1U, std::memory_order_release);
        while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
        for (unsigned i = 0U; i < 100U; ++i) {
            static_cast<void>(loop.scheduleAfter(0U, [&] {
                calls.fetch_add(1U, std::memory_order_relaxed);
            }));
        }
    };
    std::thread first(worker);
    std::thread second(worker);
    while (ready.load(std::memory_order_acquire) != 2U) std::this_thread::yield();
    start.store(true, std::memory_order_release);
    first.join();
    second.join();
    EXPECT_EQ(loop.pending(), 200U);
    EXPECT_EQ(loop.runDueEvents(0U).events_executed, 200U);
    EXPECT_EQ(calls.load(std::memory_order_relaxed), 200U);
}

TEST(EventLoopTest, RepeatedDomainCloseKeepsPendingCallbackAndIdentity) {
    fil::sim::EventLoop loop;
    std::uint32_t calls = 0U;
    fil::sim::ScheduledEvent handle(&loop);
    {
        auto owner = loop.useOwner(0U);
        static_cast<void>(handle.scheduleAt(50U, [&] { ++calls; }));
    }
    const auto original_id = handle.id();
    for (unsigned iteration = 0U; iteration < 100U; ++iteration) {
        auto domains = loop.useLaneDomains(1U);
        EXPECT_EQ(loop.pending(), 1U);
        EXPECT_EQ(handle.id(), original_id);
        EXPECT_EQ(loop.nextScheduledTime(0U), 50U);
        domains.close();
        EXPECT_EQ(loop.pending(), 1U);
        EXPECT_EQ(handle.id(), original_id);
        EXPECT_EQ(loop.nextScheduledTime(0U), 50U);
    }
    EXPECT_EQ(loop.runOwnedEvents(0U, 50U).events_executed, 1U);
    EXPECT_EQ(calls, 1U);
    EXPECT_FALSE(handle.pending());
}

TEST(EventLoopTest, LaneDomainCloseFailureIsRetryableAndWorkerExceptionUnwinds) {
    fil::sim::EventLoop loop;
    std::uint32_t calls = 0U;
    {
        auto owner = loop.useOwner(0U);
        static_cast<void>(loop.scheduleAt(5U, [&] { ++calls; }));
    }
    {
        auto domains = loop.useLaneDomains(1U);
        std::atomic<bool> entered{false};
        std::atomic<bool> leave{false};
        std::thread worker([&] {
            auto lane = loop.useLaneDomain(0U);
            entered.store(true, std::memory_order_release);
            while (!leave.load(std::memory_order_acquire)) std::this_thread::yield();
        });
        while (!entered.load(std::memory_order_acquire)) std::this_thread::yield();
        EXPECT_THROW(domains.close(), std::logic_error);
        leave.store(true, std::memory_order_release);
        worker.join();
        EXPECT_EQ(loop.pending(), 1U) << "failed close leaves the pending callback intact";
        domains.close();
    }
    EXPECT_EQ(loop.runOwnedEvents(0U, 5U).events_executed, 1U);
    EXPECT_EQ(calls, 1U);

    fil::sim::EventLoop throwing_loop;
    {
        auto owner = throwing_loop.useOwner(0U);
        static_cast<void>(throwing_loop.scheduleAt(7U, [] {
            throw std::runtime_error("lane callback failed");
        }));
    }
    std::atomic<bool> caught{false};
    {
        auto domains = throwing_loop.useLaneDomains(1U);
        std::thread worker([&] {
            auto lane = throwing_loop.useLaneDomain(0U);
            try {
                static_cast<void>(throwing_loop.runOwnedEvents(0U, 7U));
            } catch (const std::runtime_error&) {
                caught.store(true, std::memory_order_release);
            }
        });
        worker.join();
        domains.close();
    }
    EXPECT_TRUE(caught.load(std::memory_order_acquire));
    EXPECT_EQ(throwing_loop.now(0U), 7U);
    EXPECT_EQ(throwing_loop.executedCallbacks(), 0U);
}

TEST(EventLoopTest, LaneDomainDestructorOrphansAndLaterRecoversPendingEvents) {
    fil::sim::EventLoop loop;
    std::uint32_t calls = 0U;
    {
        auto owner = loop.useOwner(0U);
        static_cast<void>(loop.scheduleAt(9U, [&] { ++calls; }));
    }
    std::atomic<bool> entered{false};
    std::atomic<bool> leave{false};
    std::thread worker;
    {
        auto domains = loop.useLaneDomains(1U);
        worker = std::thread([&] {
            auto lane = loop.useLaneDomain(0U);
            entered.store(true, std::memory_order_release);
            while (!leave.load(std::memory_order_acquire)) std::this_thread::yield();
        });
        while (!entered.load(std::memory_order_acquire)) std::this_thread::yield();
        // Destruction cannot merge while a worker is active; it leaves the
        // domain state orphaned rather than discarding callbacks or terminating.
    }
    leave.store(true, std::memory_order_release);
    worker.join();
    EXPECT_EQ(loop.pending(), 1U);
    EXPECT_EQ(loop.runOwnedEvents(0U, 9U).events_executed, 1U);
    EXPECT_EQ(calls, 1U);
}

TEST(EventLoopTest, LaneDomainsRejectCrossOwnerAndUnboundWorkerOperations) {
    fil::sim::EventLoop loop;
    std::atomic<bool> rejected{false};
    {
        auto domains = loop.useLaneDomains(2U);
        std::thread worker([&] {
            try {
                static_cast<void>(loop.pending());
            } catch (const std::logic_error&) {
                rejected.store(true, std::memory_order_release);
            }
            auto lane = loop.useLaneDomain(0U);
            EXPECT_THROW(static_cast<void>(loop.runOwnedEvents(1U, 0U)), std::logic_error);
            EXPECT_THROW(static_cast<void>(loop.nextScheduledTime(1U)), std::logic_error);
            {
                auto shared = loop.useOwner(fil::sim::shared_event_owner);
                EXPECT_THROW(static_cast<void>(loop.scheduleAt(0U, [] {})), std::logic_error);
            }
        });
        worker.join();
    }
    EXPECT_TRUE(rejected.load(std::memory_order_acquire));
    EXPECT_EQ(loop.pending(), 0U);
}

TEST(EventLoopTest, AdvancesOneOwnerIndependently) {
    fil::sim::EventLoop loop;
    std::vector<std::uint32_t> calls;
    {
        auto owner = loop.useOwner(1U);
        static_cast<void>(loop.scheduleAt(7U, [&]() { calls.push_back(1U); }));
    }
    {
        auto owner = loop.useOwner(2U);
        static_cast<void>(loop.scheduleAt(7U, [&]() { calls.push_back(2U); }));
    }
    static_cast<void>(loop.scheduleAt(7U, [&]() { calls.push_back(3U); }));

    const auto local = loop.runOwnedEvents(1U, 7U);
    EXPECT_TRUE(local.events_executed == 1U && calls == std::vector<std::uint32_t>{1U})
        << "owner-local execution drains only the selected lane";
    EXPECT_TRUE(loop.now() == 0U && loop.now(1U) == 7U && loop.pending() == 2U)
        << "owner-local execution leaves the shared clock and other lanes untouched";
    const auto remaining = loop.runDueEvents(7U);
    EXPECT_TRUE((remaining.events_executed == 2U &&
                 calls == std::vector<std::uint32_t>{1U, 2U, 3U} && loop.pending() == 0U))
        << "global execution skips an owner event already committed locally";
}

TEST(EventLoopTest, DenseAndSparseOwnerClocksPreserveSharedClamp) {
    for (const fil::sim::EventOwner owner : {0U, 63U, 64U, 1000U}) {
        fil::sim::EventLoop loop;
        static_cast<void>(loop.runOwnedEvents(owner, 20U));
        EXPECT_EQ(loop.now(owner), 20U);
        static_cast<void>(loop.runDueEvents(10U));
        EXPECT_EQ(loop.now(owner), 20U);
        static_cast<void>(loop.runOwnedEvents(owner, 30U));
        EXPECT_EQ(loop.now(owner), 30U);
        EXPECT_EQ(loop.now(), 10U);
        static_cast<void>(loop.runDueEvents(40U));
        EXPECT_EQ(loop.now(owner), 40U);
    }
}

TEST(EventLoopTest, DenseAndSparseQueuesPreserveCancellationAndClear) {
    for (const fil::sim::EventOwner owner : {0U, 63U, 64U, 1000U}) {
        fil::sim::EventLoop loop;
        std::uint32_t calls = 0U;
        {
            auto scope = loop.useOwner(owner);
            const auto canceled = loop.scheduleAt(5U, [&] { ++calls; });
            static_cast<void>(loop.scheduleAt(10U, [&] { ++calls; }));
            EXPECT_TRUE(loop.cancel(canceled));
        }
        EXPECT_EQ(loop.nextScheduledTime(owner), 10U);
        loop.clear();
        EXPECT_FALSE(loop.nextScheduledTime(owner).has_value());
        EXPECT_EQ(loop.runOwnedEvents(owner, 20U).events_executed, 0U);
        EXPECT_EQ(calls, 0U);
    }
}

TEST(EventLoopTest, CancelsAndRejectsInvalidTime) {
    fil::sim::EventLoop loop;
    std::int32_t calls = 0;
    const fil::sim::EventId event = loop.scheduleAfter(5, [&]() { ++calls; });
    EXPECT_TRUE(loop.cancel(event)) << "event cancellation succeeds once";
    EXPECT_TRUE(!loop.cancel(event)) << "event cancellation is idempotent";
    EXPECT_TRUE(loop.pending() == 0) << "cancelled event is not pending";
    EXPECT_TRUE(!loop.nextScheduledTime().has_value())
        << "cancelled front event cannot cap a batching horizon";
    EXPECT_TRUE(loop.runDueEvents(10).events_executed == 0 && calls == 0)
        << "cancelled callback never runs";

    bool rejected = false;
    try {
        static_cast<void>(loop.runDueEvents(9));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    EXPECT_TRUE(rejected) << "event loop rejects backwards time";
}

TEST(EventLoopTest, ManagesReplaceableScheduledEvents) {
    fil::sim::EventLoop loop;
    std::int32_t calls = 0;
    {
        fil::sim::ScheduledEvent event(&loop);
        const auto first = event.scheduleAt(5U, [&]() { calls += 10; });
        const auto second = event.scheduleAt(6U, [&]() { ++calls; });
        EXPECT_TRUE(first != second && event.pending() && loop.pending() == 1U)
            << "scheduled event replacement cancels the prior callback";
        static_cast<void>(loop.runDueEvents(5U));
        EXPECT_TRUE(calls == 0 && event.pending())
            << "replacement callback keeps its later deadline";
        static_cast<void>(loop.runDueEvents(6U));
        EXPECT_TRUE(calls == 1 && !event.pending())
            << "scheduled event retires its handle before running";
        static_cast<void>(event.scheduleAfter(1U, [&]() { ++calls; }));
    }
    static_cast<void>(loop.runDueEvents(7U));
    EXPECT_TRUE(calls == 1 && loop.pending() == 0U)
        << "scheduled event destruction cancels pending work";
}

TEST(EventLoopTest, DetectsZeroDelayLivelock) {
    fil::sim::EventLoop loop(3);
    std::int32_t calls = 0;
    std::function<void()> reschedule;
    reschedule = [&]() {
        ++calls;
        static_cast<void>(loop.scheduleAfter(0, reschedule));
    };
    static_cast<void>(loop.scheduleAfter(0, reschedule));

    const auto result = loop.runDueEvents(0);
    EXPECT_TRUE(result.same_time_limit_hit) << "event loop reports the same-time event limit";
    EXPECT_TRUE(result.events_executed == 3 && calls == 3)
        << "same-time limit is exact and deterministic";
    EXPECT_TRUE(loop.pending() == 1) << "livelock protection leaves the next event inspectable";
    loop.clear();
}

TEST(EventLoopTest, SerializesStableTraceRecords) {
    fil::sim::TraceRecorder trace;
    static_cast<void>(trace.record(7, "board\"one", "gpio", {{"pin", "13"}}));
    fil::sim::CanTraceFrame frame;
    frame.id = 0x123;
    frame.fd = true;
    frame.dlc = 9U;
    frame.data = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
                  0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b};
    static_cast<void>(trace.recordCanFrame(8, "vehicle", true, frame));

    const std::string json = trace.jsonLines();
    EXPECT_TRUE(json.find("board\\\"one") != std::string::npos) << "trace JSON escapes strings";
    EXPECT_TRUE(json.find("\"sequence\":0") != std::string::npos)
        << "trace starts stable sequence numbering";
    EXPECT_TRUE(json.find("\"type\":\"can_tx\"") != std::string::npos)
        << "trace records CAN direction";
    EXPECT_TRUE(json.find("\"dlc\":\"9\",\"length\":\"12\"") != std::string::npos)
        << "CAN-FD trace distinguishes encoded DLC from payload length";
    EXPECT_TRUE(json.find("\"data\":\"000102030405060708090a0b\"") != std::string::npos)
        << "trace encodes every CAN-FD payload byte";
}

TEST(EventLoopTest, NotifiesTraceObserverAfterAppendingRecord) {
    fil::sim::TraceRecorder trace;
    bool notified = false;
    fil::sim::TraceRecord observed;
    trace.setObserver([&](const fil::sim::TraceRecord& record) {
        EXPECT_FALSE(trace.records().empty());
        observed = record;
        notified = true;
    });

    static_cast<void>(trace.record(4, "board", "can_tx"));
    ASSERT_TRUE(notified);
    EXPECT_EQ(observed.sequence, trace.records().back().sequence);
    EXPECT_EQ(observed.time_ns, 4U);

    trace.setEnabled(false);
    notified = false;
    static_cast<void>(trace.record(5, "board", "hidden"));
    EXPECT_FALSE(notified);
}

TEST(EventLoopTest, DisablesTraceCollectionWithoutDisturbingSequence) {
    fil::sim::TraceRecorder trace;
    EXPECT_TRUE(trace.enabled()) << "trace collection defaults to enabled";
    trace.setEnabled(false);
    static_cast<void>(trace.record(1, "board", "hidden"));
    fil::sim::CanTraceFrame frame;
    frame.id = 1;
    static_cast<void>(trace.recordCanFrame(2, "bus", true, frame));
    EXPECT_TRUE(!trace.enabled() && trace.records().empty())
        << "disabled trace recorder discards generic and CAN records";

    trace.setEnabled(true);
    static_cast<void>(trace.record(3, "board", "visible"));
    EXPECT_TRUE(trace.records().size() == 1 && trace.records().front().sequence == 0)
        << "reenabling trace preserves contiguous sequence numbering";
}

} // namespace

/** @brief Runs deterministic event-loop and trace unit tests. */
