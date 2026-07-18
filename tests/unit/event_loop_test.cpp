#include "fil/sim/event_loop.hpp"
#include "fil/sim/trace.hpp"
#include "fil/sim/worker_pool.hpp"
#include "../test_support.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

void ordersEventsDeterministically() {
    fil::sim::EventLoop loop;
    std::vector<int> order;
    static_cast<void>(loop.scheduleAt(20, [&]() { order.push_back(3); }));
    static_cast<void>(loop.scheduleAt(10, [&]() {
        order.push_back(1);
        static_cast<void>(loop.scheduleAfter(0, [&]() { order.push_back(4); }));
    }));
    static_cast<void>(loop.scheduleAt(10, [&]() { order.push_back(2); }));

    const auto first = loop.runDueEvents(10);
    fil::test::check(first.events_executed == 3, "event loop runs nested same-time events");
    fil::test::check(order == std::vector<int>({1, 2, 4}), "event loop orders by time then insertion sequence");
    fil::test::check(loop.now() == 10 && loop.pending() == 1, "event loop retains future events");

    const auto second = loop.advanceBy(10);
    fil::test::check(second.events_executed == 1 && order.back() == 3, "event loop advances relative time");
}

void tracksLocalAndSharedEventOwnership() {
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
    fil::test::check(loop.activeOwner() == fil::sim::shared_event_owner,
                     "event owner scope restores the shared domain");
    static_cast<void>(loop.scheduleAt(5U, [&]() {
        observed.push_back(loop.activeOwner());
    }));
    fil::test::check(loop.nextScheduledTime(3U) == 5U
                         && loop.nextScheduledTime(fil::sim::shared_event_owner) == 5U
                         && !loop.nextScheduledTime(4U),
                     "owner queues expose independent event horizons");

    const auto result = loop.runDueEvents(5U);
    fil::test::check(result.events_executed == 3U
                         && result.local_owner_mask == (std::uint64_t{1U} << 3U)
                         && result.shared_event_executed,
                     "event runs report local and shared ownership");
    fil::test::check(observed == std::vector<fil::sim::EventOwner>{
                         3U, fil::sim::shared_event_owner, 3U,
                     },
                     "nested events inherit their callback owner deterministically");
}

void advancesOneOwnerIndependently() {
    fil::sim::EventLoop rewindable;
    const auto checkpoint = rewindable.ownerCheckpoint(4U);
    static_cast<void>(rewindable.runOwnedEvents(4U, 10U));
    fil::test::check(rewindable.restoreOwnerCheckpoint(checkpoint)
                         && rewindable.now(4U) == 0U,
                     "untouched owner queues permit transactional clock rewind");

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
    fil::test::check(local.events_executed == 1U && calls == std::vector<unsigned int>{1U},
                     "owner-local execution drains only the selected lane");
    fil::test::check(loop.now() == 0U && loop.now(1U) == 7U && loop.pending() == 2U,
                     "owner-local execution leaves the shared clock and other lanes untouched");
    fil::test::check(!loop.restoreOwnerCheckpoint(event_checkpoint),
                     "executed owner callbacks reject unsafe clock-only rollback");

    const auto remaining = loop.runDueEvents(7U);
    fil::test::check(remaining.events_executed == 2U
                         && calls == std::vector<unsigned int>{1U, 2U, 3U}
                         && loop.pending() == 0U,
                     "global execution skips an owner event already committed locally");
}

void reusesPersistentLaneWorkers() {
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
    fil::test::check(exact, "persistent worker pool executes every lane once per epoch");
}

void supportsConcurrentOwnerLanes() {
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
        fil::test::check(result.events_executed == 100U,
                         "worker drains its concurrently scheduled owner queue");
    };
    std::thread first(worker, 1U);
    std::thread second(worker, 2U);
    first.join();
    second.join();
    fil::test::check(calls.load(std::memory_order_relaxed) == 200U
                         && loop.pending() == 0U && loop.now() == 0U,
                     "concurrent owner lanes preserve exactly-once callback execution");
}

void cancelsAndRejectsInvalidTime() {
    fil::sim::EventLoop loop;
    int calls = 0;
    const fil::sim::EventId event = loop.scheduleAfter(5, [&]() { ++calls; });
    fil::test::check(loop.cancel(event), "event cancellation succeeds once");
    fil::test::check(!loop.cancel(event), "event cancellation is idempotent");
    fil::test::check(loop.pending() == 0, "cancelled event is not pending");
    fil::test::check(!loop.nextScheduledTime().has_value(),
                     "cancelled front event cannot cap a batching horizon");
    fil::test::check(loop.runDueEvents(10).events_executed == 0 && calls == 0, "cancelled callback never runs");

    bool rejected = false;
    try {
        static_cast<void>(loop.runDueEvents(9));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    fil::test::check(rejected, "event loop rejects backwards time");
}

void detectsZeroDelayLivelock() {
    fil::sim::EventLoop loop(3);
    int calls = 0;
    std::function<void()> reschedule;
    reschedule = [&]() {
        ++calls;
        static_cast<void>(loop.scheduleAfter(0, reschedule));
    };
    static_cast<void>(loop.scheduleAfter(0, reschedule));

    const auto result = loop.runDueEvents(0);
    fil::test::check(result.same_time_limit_hit, "event loop reports the same-time event limit");
    fil::test::check(result.events_executed == 3 && calls == 3, "same-time limit is exact and deterministic");
    fil::test::check(loop.pending() == 1, "livelock protection leaves the next event inspectable");
    loop.clear();
}

void serializesStableTraceRecords() {
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
    fil::test::check(json.find("board\\\"one") != std::string::npos, "trace JSON escapes strings");
    fil::test::check(json.find("\"sequence\":0") != std::string::npos, "trace starts stable sequence numbering");
    fil::test::check(json.find("\"type\":\"can_tx\"") != std::string::npos, "trace records CAN direction");
    fil::test::check(json.find("\"dlc\":\"9\",\"length\":\"12\"") != std::string::npos,
                     "CAN-FD trace distinguishes encoded DLC from payload length");
    fil::test::check(json.find("\"data\":\"000102030405060708090a0b\"") != std::string::npos,
                     "trace encodes every CAN-FD payload byte");
}

void disablesTraceCollectionWithoutDisturbingSequence() {
    fil::sim::TraceRecorder trace;
    fil::test::check(trace.enabled(), "trace collection defaults to enabled");
    trace.setEnabled(false);
    static_cast<void>(trace.record(1, "board", "hidden"));
    fil::sim::CanTraceFrame frame;
    frame.id = 1;
    static_cast<void>(trace.recordCanFrame(2, "bus", true, frame));
    fil::test::check(!trace.enabled() && trace.records().empty(),
                     "disabled trace recorder discards generic and CAN records");

    trace.setEnabled(true);
    static_cast<void>(trace.record(3, "board", "visible"));
    fil::test::check(trace.records().size() == 1 && trace.records().front().sequence == 0,
                     "reenabling trace preserves contiguous sequence numbering");
}

} // namespace

/** @brief Runs deterministic event-loop and trace unit tests. */
void runEventLoopTests() {
    ordersEventsDeterministically();
    tracksLocalAndSharedEventOwnership();
    advancesOneOwnerIndependently();
    reusesPersistentLaneWorkers();
    supportsConcurrentOwnerLanes();
    cancelsAndRejectsInvalidTime();
    detectsZeroDelayLivelock();
    serializesStableTraceRecords();
    disablesTraceCollectionWithoutDisturbingSequence();
}
