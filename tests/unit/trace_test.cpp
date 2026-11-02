#include "fil/sim/trace.hpp"

#include <cstdint>
#include <gtest/gtest.h>

#include <string>
#include <thread>
#include <vector>

namespace {

/** @brief Observer input remains valid when its callback recursively appends records. */
TEST(TraceRecorderTest, ObserverCanStreamWithoutRetainingHistory) {
    fil::sim::TraceRecorder trace;
    trace.setRetainRecords(false);
    trace.setTypeAllowlist({"can_tx"});
    std::vector<std::uint64_t> observed_sequences;
    trace.setObserver([&](const fil::sim::TraceRecord& record) {
        observed_sequences.push_back(record.sequence);
    });

    EXPECT_EQ(trace.record(1U, "bus", "can_tx"), 0U);
    EXPECT_EQ(trace.record(2U, "cpu", "instr"), 1U);
    EXPECT_EQ(trace.record(3U, "bus", "can_tx"), 2U);

    EXPECT_EQ(observed_sequences, (std::vector<std::uint64_t>{0U, 2U}));
    EXPECT_TRUE(trace.records().empty());
}

TEST(TraceRecorderTest, LaneCaptureMergesWorkerRecordsInTimeOrder) {
    fil::sim::TraceRecorder trace;
    trace.setTypeAllowlist({"lane"});
    std::vector<fil::sim::TraceRecord> observed;
    trace.setObserver([&](const fil::sim::TraceRecord& record) {
        observed.push_back(record);
    });
    trace.beginLaneCapture(3U);
    const auto emit = [&](const std::size_t lane) {
        fil::sim::TraceRecorder::setCurrentLane(lane);
        for (std::uint64_t index = 0U; index < 100U; ++index) {
            static_cast<void>(trace.record(1'000U - index, "lane" + std::to_string(lane), "lane"));
            // Filtered types must not consume the shared sequence counter while
            // worker threads are capturing.
            static_cast<void>(trace.record(1'000U - index, "hidden", "other"));
        }
        fil::sim::TraceRecorder::clearCurrentLane();
    };
    std::thread first(emit, 0U);
    std::thread third(emit, 2U);
    first.join();
    third.join();
    EXPECT_TRUE(observed.empty()) << "workers must not invoke the observer";

    trace.flushLaneCapture();
    ASSERT_EQ(trace.records().size(), 200U);
    ASSERT_EQ(observed.size(), trace.records().size());
    for (std::size_t index = 0U; index < trace.records().size(); ++index) {
        EXPECT_EQ(trace.records()[index].sequence, index);
        if (index != 0U) {
            EXPECT_LE(trace.records()[index - 1U].time_ns, trace.records()[index].time_ns);
        }
    }
    // Equal timestamps keep lane order (lane 0 before lane 2), deterministically.
    EXPECT_EQ(trace.records().front().source, "lane0");
    EXPECT_EQ(observed.front().source, "lane0");
    EXPECT_EQ(trace.jsonLines().find("\"source\":\"hidden\""), std::string::npos);

    // Reused buffers must not replay old records, including after lane shrink.
    trace.beginLaneCapture(1U);
    trace.flushLaneCapture();
    EXPECT_EQ(observed.size(), 200U);
    trace.beginLaneCapture(3U);
    fil::sim::TraceRecorder::setCurrentLane(2U);
    static_cast<void>(trace.record(2'000U, "new-window", "lane"));
    fil::sim::TraceRecorder::clearCurrentLane();
    trace.flushLaneCapture();
    ASSERT_EQ(observed.size(), 201U);
    EXPECT_EQ(observed.back().sequence, 200U);
    EXPECT_EQ(observed.back().source, "new-window");
}

TEST(TraceRecorderTest, ObserverRecordSurvivesRecursiveAppend) {
    fil::sim::TraceRecorder trace;
    bool expanding = false;
    bool checked_outer_record = false;
    trace.setObserver([&](const fil::sim::TraceRecord& record) {
        if (expanding) return;
        expanding = true;

        for (std::int32_t index = 0; index < 1024; ++index) {
            static_cast<void>(trace.record(
                static_cast<fil::sim::SimTimeNs>(index + 1), "nested", "event"));
        }

        EXPECT_EQ(record.time_ns, 7U);
        EXPECT_EQ(record.sequence, 0U);
        EXPECT_EQ(record.source, "outer");
        EXPECT_EQ(record.type, "start");
        checked_outer_record = true;
    });

    static_cast<void>(trace.record(7, "outer", "start"));

    EXPECT_TRUE(checked_outer_record);
    ASSERT_EQ(trace.records().size(), 1025U);
    EXPECT_EQ(trace.records().front().source, "outer");
    EXPECT_EQ(trace.records().front().type, "start");
}

} // namespace
