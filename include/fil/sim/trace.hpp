#pragma once

/** @file trace.hpp
 *  @brief Stable in-memory and JSON-lines simulation trace records.
 */

#include "fil/sim/event_loop.hpp"

#include <cstdint>
#include <functional>
#include <iosfwd>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace fil::sim {

/** @brief One key/value pair attached to a trace record. */
using TraceField = std::pair<std::string, std::string>;

/** @brief Ordered trace record suitable for deterministic comparison. */
struct TraceRecord {
    SimTimeNs time_ns{0};
    std::uint64_t sequence{0};
    std::string source;
    std::string type;
    std::vector<TraceField> fields;
};

/** @brief Minimal CAN frame metadata used by generic bus-facing traces. */
struct CanTraceFrame {
    std::uint32_t id{0};
    bool extended{false};
    bool fd{false};
    bool brs{false};
    std::uint8_t dlc{0}; ///< Encoded CAN data-length code, not decoded byte count.
    std::vector<std::uint8_t> data;
};

/** @brief Collects trace records in insertion order and serializes stable JSONL. */
class TraceRecorder {
public:
    using Observer = std::function<void(const TraceRecord&)>;

    /** @brief Enables or disables future record collection without clearing existing records. */
    void setEnabled(bool enabled) noexcept { enabled_ = enabled; }

    /** @brief Enables/disables history retention while preserving observers and sequence IDs. */
    void setRetainRecords(bool retain) noexcept { retain_records_ = retain; }

    /** @brief Reports whether future trace records are currently observable. */
    [[nodiscard]] bool enabled() const noexcept { return enabled_; }

    /** @brief Observes newly appended records without changing stored trace output. */
    void setObserver(Observer observer) { observer_ = std::move(observer); }

    /** @brief Whether trace emission can invoke an arbitrary external observer. */
    [[nodiscard]] bool hasObserver() const noexcept { return static_cast<bool>(observer_); }

    /** @brief Sentinel used for records emitted outside any parallel lane. */
    static constexpr std::size_t coordinator_lane = static_cast<std::size_t>(-1);

    /** @brief Associates the calling worker thread with one parallel lane. */
    static void setCurrentLane(std::size_t lane) noexcept;

    /** @brief Clears the calling thread's parallel-lane association. */
    static void clearCurrentLane() noexcept;

    /** @brief Reports the calling thread's lane, or coordinator_lane outside a lane. */
    [[nodiscard]] static std::size_t currentLane() noexcept;

    /**
     * @brief Begins buffering lane-local records until flushLaneCapture().
     *
     * The recorder is not synchronized, so a parallel scheduler must keep worker
     * threads off the shared record vector. While capture is active, records
     * emitted from a thread marked with setCurrentLane() are buffered per lane
     * without touching the global sequence, history, or observer. The scheduler
     * calls flushLaneCapture() once every lane has parked; the coordinator then
     * merges the buffers in (time, lane) order and dispatches observers on its
     * own thread, so any record type, including lane-local GPIO or ADC records,
     * can be traced while lanes run in parallel.
     */
    void beginLaneCapture(std::size_t lane_count);

    /** @brief Merges captured lane records into the global stream in time order. */
    void flushLaneCapture();

    /** @brief Whether lane capture is currently buffering worker records. */
    [[nodiscard]] bool capturingLanes() const noexcept { return capturing_; }

    /**
     * @brief Restricts recorded types to an allowlist (empty allows all).
     *
     * Filtered-out types still consume sequence numbers but skip field
     * formatting, storage, and observer notification. Only monitoring
     * front ends that select a few types from a hot event stream should set
     * this; batch trace artifacts always need the full record set.
     */
    void setTypeAllowlist(std::vector<std::string> allowlist) {
        allowlist_ = std::move(allowlist);
    }

    /** @brief Reports whether a record type passes the allowlist (or none is set). */
    [[nodiscard]] bool passesFilter(std::string_view type) const noexcept;

    /**
     * @brief Appends one trace record and returns its assigned sequence.
     * @return Assigned sequence, or the unconsumed next sequence while disabled.
     */
    std::uint64_t record(
        SimTimeNs time_ns,
        std::string source,
        std::string type,
        std::vector<TraceField> fields = {}
    );

    /**
     * @brief Appends a normalized CAN transmit or receive record.
     * @return Assigned sequence, or the unconsumed next sequence while disabled.
     */
    std::uint64_t recordCanFrame(
        SimTimeNs time_ns,
        std::string source,
        bool transmit,
        const CanTraceFrame& frame,
        std::string bus = {},
        std::string origin = {}
    );

    /** @brief Gets immutable records in deterministic insertion order. */
    [[nodiscard]] const std::vector<TraceRecord>& records() const noexcept { return records_; }

    /** @brief Removes records and restarts sequence numbering at zero. */
    void clear() noexcept;

    /** @brief Writes records as one compact JSON object per line. */
    void writeJsonLines(std::ostream& output) const;

    /** @brief Returns the entire compact JSON-lines representation. */
    [[nodiscard]] std::string jsonLines() const;

    /** @brief Encodes bytes as stable lowercase hexadecimal without separators. */
    [[nodiscard]] static std::string hexBytes(std::span<const std::uint8_t> bytes);

private:
    void appendRecord(TraceRecord record);

    bool enabled_{true};
    bool retain_records_{true};
    bool capturing_{false};
    std::uint64_t next_sequence_{0};
    std::vector<TraceRecord> records_;
    std::vector<std::vector<TraceRecord>> lane_records_;
    std::vector<std::string> allowlist_;
    Observer observer_;
};

} // namespace fil::sim
