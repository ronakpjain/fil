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
    /** @brief Optional synchronous observer used by interactive front ends. */
    using Observer = std::function<void(const TraceRecord&)>;

    /** @brief Enables or disables future record collection without clearing existing records. */
    void setEnabled(bool enabled) noexcept { enabled_ = enabled; }

    /** @brief Reports whether future trace records are currently observable. */
    [[nodiscard]] bool enabled() const noexcept { return enabled_; }

    /** @brief Observes each newly appended record without changing stored JSONL output. */
    void setObserver(Observer observer) { observer_ = std::move(observer); }

    /** @brief Controls in-memory retention while keeping observers active. */
    void setRetainRecords(bool retain) noexcept { retain_records_ = retain; }

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
        const CanTraceFrame& frame
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
    bool enabled_{true};
    bool retain_records_{true};
    std::uint64_t next_sequence_{0};
    std::vector<TraceRecord> records_;
    Observer observer_;
};

} // namespace fil::sim
