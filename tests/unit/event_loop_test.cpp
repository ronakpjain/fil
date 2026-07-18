#include "fil/sim/event_loop.hpp"
#include "fil/sim/trace.hpp"
#include "../test_support.hpp"

#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
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
    cancelsAndRejectsInvalidTime();
    detectsZeroDelayLivelock();
    serializesStableTraceRecords();
    disablesTraceCollectionWithoutDisturbingSequence();
}
