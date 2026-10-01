#include "fil/sim/trace.hpp"

#include <gtest/gtest.h>

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

TEST(TraceRecorderTest, ObserverRecordSurvivesRecursiveAppend) {
    fil::sim::TraceRecorder trace;
    bool expanding = false;
    bool checked_outer_record = false;
    trace.setObserver([&](const fil::sim::TraceRecord& record) {
        if (expanding) return;
        expanding = true;

        for (int index = 0; index < 1024; ++index) {
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
