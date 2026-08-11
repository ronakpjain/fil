#include "fil/sim/event_loop.hpp"
#include "fil/sim/trace.hpp"
#include "fil/sim/worker_pool.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

TEST(EventLoopTest, OrdersEventsDeterministically) {
    fil::sim::EventLoop loop;
    std::vector<int> order;
    static_cast<void>(loop.scheduleAt(20, [&]() { order.push_back(3); }));
    static_cast<void>(loop.scheduleAt(10, [&]() {
        order.push_back(1);
        static_cast<void>(loop.scheduleAfter(0, [&]() { order.push_back(4); }));
    }));
    static_cast<void>(loop.scheduleAt(10, [&]() { order.push_back(2); }));

    const auto first = loop.runDueEvents(10);
    EXPECT_TRUE(first.events_executed == 3) << "event loop runs nested same-time events";
    EXPECT_TRUE(order == std::vector<int>({1, 2, 4}))
        << "event loop orders by time then insertion sequence";
    EXPECT_TRUE(loop.now() == 10 && loop.pending() == 1) << "event loop retains future events";

    const auto second = loop.advanceBy(10);
    EXPECT_TRUE(second.events_executed == 1 && order.back() == 3)
        << "event loop advances relative time";
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

TEST(EventLoopTest, AdvancesOneOwnerIndependently) {
    fil::sim::EventLoop rewindable;
    const auto checkpoint = rewindable.ownerCheckpoint(4U);
    static_cast<void>(rewindable.runOwnedEvents(4U, 10U));
    EXPECT_TRUE(rewindable.restoreOwnerCheckpoint(checkpoint) && rewindable.now(4U) == 0U)
        << "untouched owner queues permit transactional clock rewind";

    fil::sim::EventLoop loop;
    std::vector<unsigned int> calls;
    {
        auto owner = loop.useOwner(1U);
        static_cast<void>(loop.scheduleAt(7U, [&]() { calls.push_back(1U); }));
    }
    {
        auto owner = loop.useOwner(2U);
        static_cast<void>(loop.scheduleAt(7U, [&]() { calls.push_back(2U); }));
    }
    static_cast<void>(loop.scheduleAt(7U, [&]() { calls.push_back(3U); }));

    const auto event_checkpoint = loop.ownerCheckpoint(1U);
    const auto local = loop.runOwnedEvents(1U, 7U);
    EXPECT_TRUE(local.events_executed == 1U && calls == std::vector<unsigned int>{1U})
        << "owner-local execution drains only the selected lane";
    EXPECT_TRUE(loop.now() == 0U && loop.now(1U) == 7U && loop.pending() == 2U)
        << "owner-local execution leaves the shared clock and other lanes untouched";
    EXPECT_TRUE(!loop.restoreOwnerCheckpoint(event_checkpoint))
        << "executed owner callbacks reject unsafe clock-only rollback";

    const auto remaining = loop.runDueEvents(7U);
    EXPECT_TRUE((remaining.events_executed == 2U &&
                 calls == std::vector<unsigned int>{1U, 2U, 3U} && loop.pending() == 0U))
        << "global execution skips an owner event already committed locally";
}

TEST(EventLoopTest, ReusesPersistentLaneWorkers) {
    fil::sim::LaneWorkerPool workers(4U);
    std::array<std::atomic<unsigned int>, 4> calls{};
    const auto task = [&](const std::size_t lane) {
        calls[lane].fetch_add(1U, std::memory_order_relaxed);
    };
    workers.run(task);
    workers.run(task);
    bool exact = workers.size() == 4U;
    for (const auto& count : calls) {
        exact = exact && count.load(std::memory_order_relaxed) == 2U;
    }
    EXPECT_TRUE(exact) << "persistent worker pool executes every lane once per epoch";
}

TEST(EventLoopTest, ValidatesWorkerPoolFailuresAndRecovery) {
    bool rejected_zero_lanes = false;
    try {
        fil::sim::LaneWorkerPool invalid(0U);
    } catch (const std::invalid_argument&) {
        rejected_zero_lanes = true;
    }
    EXPECT_TRUE(rejected_zero_lanes) << "worker pool rejects an empty lane set";

    fil::sim::LaneWorkerPool workers(2U);
    bool rejected_empty_task = false;
    try {
        workers.run({});
    } catch (const std::invalid_argument&) {
        rejected_empty_task = true;
    }
    EXPECT_TRUE(rejected_empty_task) << "worker pool rejects an empty task";

    bool propagated = false;
    try {
        workers.run([](const std::size_t lane) {
            if (lane == 1U) throw std::runtime_error("lane failed");
        });
    } catch (const std::runtime_error&) {
        propagated = true;
    }
    std::atomic<unsigned int> recovered{0U};
    workers.run([&](std::size_t) {
        recovered.fetch_add(1U, std::memory_order_relaxed);
    });
    EXPECT_TRUE(propagated && recovered.load(std::memory_order_relaxed) == 2U)
        << "worker pool propagates failures and remains reusable";
}

TEST(EventLoopTest, SupportsConcurrentOwnerLanes) {
    fil::sim::EventLoop loop;
    loop.setConcurrentAccess(true);
    std::atomic<unsigned int> calls{0U};
    const auto worker = [&](const fil::sim::EventOwner owner) {
        auto scope = loop.useOwner(owner);
        for (unsigned int index = 0U; index < 100U; ++index) {
            static_cast<void>(loop.scheduleAt(0U, [&]() {
                calls.fetch_add(1U, std::memory_order_relaxed);
            }));
        }
        const auto result = loop.runOwnedEvents(owner, 0U);
        EXPECT_TRUE(result.events_executed == 100U)
            << "worker drains its concurrently scheduled owner queue";
    };
    std::thread first(worker, 1U);
    std::thread second(worker, 2U);
    first.join();
    second.join();
    EXPECT_TRUE(
        calls.load(std::memory_order_relaxed) == 200U && loop.pending() == 0U && loop.now() == 0U)
        << "concurrent owner lanes preserve exactly-once callback execution";
}

TEST(EventLoopTest, CancelsAndRejectsInvalidTime) {
    fil::sim::EventLoop loop;
    int calls = 0;
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
    int calls = 0;
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
    int calls = 0;
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
    const fil::sim::TraceRecord* observed = nullptr;
    trace.setObserver([&observed](const fil::sim::TraceRecord& record) { observed = &record; });

    static_cast<void>(trace.record(4, "board", "can_tx"));
    ASSERT_NE(observed, nullptr);
    EXPECT_EQ(observed, &trace.records().back());
    EXPECT_EQ(observed->time_ns, 4U);

    trace.setEnabled(false);
    observed = nullptr;
    static_cast<void>(trace.record(5, "board", "hidden"));
    EXPECT_EQ(observed, nullptr);
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
