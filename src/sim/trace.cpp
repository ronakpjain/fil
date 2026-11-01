#include "fil/sim/trace.hpp"

#include <algorithm>
#include <cstdint>
#include <ostream>
#include <sstream>
#include <utility>

namespace fil::sim {
namespace {

void writeEscaped(std::ostream& output, const std::string_view text) {
    static constexpr char hexadecimal[] = "0123456789abcdef";
    for (const char character : text) {
        const auto byte = static_cast<std::uint8_t>(character);
        switch (byte) {
        case '"': output << "\\\""; break;
        case '\\': output << "\\\\"; break;
        case '\b': output << "\\b"; break;
        case '\f': output << "\\f"; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default:
            if (byte < 0x20U) {
                output << "\\u00" << hexadecimal[byte >> 4U] << hexadecimal[byte & 0x0fU];
            } else {
                output << static_cast<char>(byte);
            }
            break;
        }
    }
}

std::string booleanText(const bool value) {
    return value ? "true" : "false";
}

} // namespace

namespace {

// Worker threads announce which lane they are running so record() can route
// lane-local events into a per-lane buffer instead of the shared record vector.
thread_local std::size_t g_current_lane = TraceRecorder::coordinator_lane;

} // namespace

void TraceRecorder::setCurrentLane(const std::size_t lane) noexcept {
    g_current_lane = lane;
}

void TraceRecorder::clearCurrentLane() noexcept {
    g_current_lane = coordinator_lane;
}

std::size_t TraceRecorder::currentLane() noexcept {
    return g_current_lane;
}

bool TraceRecorder::passesFilter(const std::string_view type) const noexcept {
    if (allowlist_.empty()) return true;
    return std::find(allowlist_.begin(), allowlist_.end(), type) != allowlist_.end();
}

void TraceRecorder::beginLaneCapture(const std::size_t lane_count) {
    capturing_ = true;
    lane_records_.assign(lane_count, {});
}

void TraceRecorder::flushLaneCapture() {
    capturing_ = false;
    std::size_t captured = 0U;
    for (const auto& lane : lane_records_) captured += lane.size();
    std::vector<TraceRecord> pending;
    pending.reserve(captured);
    // lane_records_ is indexed by lane, so appending lanes in order and using a
    // stable sort keeps (lane, per-lane insertion) order for equal timestamps.
    for (auto& lane : lane_records_) {
        for (auto& record : lane) pending.push_back(std::move(record));
        lane.clear();
    }
    lane_records_.clear();
    std::stable_sort(pending.begin(), pending.end(), [](const TraceRecord& a, const TraceRecord& b) {
        return a.time_ns < b.time_ns;
    });
    for (auto& record : pending) {
        record.sequence = next_sequence_++;
        appendRecord(std::move(record));
    }
}

void TraceRecorder::appendRecord(TraceRecord record) {
    if (retain_records_) {
        records_.push_back(std::move(record));
        if (observer_) {
            // Copy so an observer that recursively appends cannot observe a
            // moved-from element after records_ reallocates.
            const TraceRecord observed = records_.back();
            observer_(observed);
        }
    } else if (observer_) {
        observer_(record);
    }
}

std::uint64_t TraceRecorder::record(
    const SimTimeNs time_ns,
    std::string source,
    std::string type,
    std::vector<TraceField> fields
) {
    if (!enabled_) return next_sequence_;
    if (!passesFilter(type)) {
        // Capture keeps worker threads off the shared sequence counter.
        if (capturing_) return next_sequence_;
        return next_sequence_++;
    }
    if (capturing_ && currentLane() != coordinator_lane
        && currentLane() < lane_records_.size()) {
        lane_records_[currentLane()].push_back(TraceRecord{
            time_ns, 0U, std::move(source), std::move(type), std::move(fields),
        });
        return next_sequence_;
    }
    const std::uint64_t sequence = next_sequence_++;
    appendRecord(TraceRecord{
        time_ns, sequence, std::move(source), std::move(type), std::move(fields),
    });
    return sequence;
}

std::uint64_t TraceRecorder::recordCanFrame(
    const SimTimeNs time_ns,
    std::string source,
    const bool transmit,
    const CanTraceFrame& frame,
    std::string bus,
    std::string origin
) {
    if (!enabled_) return next_sequence_;
    // Skip field formatting for types no consumer selected; record() would
    // discard the formatted fields below, but only after paying for them.
    if (!passesFilter(transmit ? "can_tx" : "can_rx")) {
        if (capturing_) return next_sequence_;
        return next_sequence_++;
    }
    std::ostringstream id;
    id << "0x" << std::hex << frame.id;
    std::vector<TraceField> fields{
        {"id", id.str()},
        {"extended", booleanText(frame.extended)},
        {"fd", booleanText(frame.fd)},
        {"brs", booleanText(frame.brs)},
        {"dlc", std::to_string(frame.dlc)},
        {"length", std::to_string(frame.data.size())},
        {"data", hexBytes(frame.data)},
    };
    if (!bus.empty()) fields.emplace_back("bus", std::move(bus));
    if (!origin.empty()) fields.emplace_back("origin", std::move(origin));
    return record(time_ns, std::move(source), transmit ? "can_tx" : "can_rx", std::move(fields));
}

void TraceRecorder::clear() noexcept {
    records_.clear();
    lane_records_.clear();
    capturing_ = false;
    next_sequence_ = 0;
}

void TraceRecorder::writeJsonLines(std::ostream& output) const {
    for (const TraceRecord& record : records_) {
        output << "{\"time_ns\":" << record.time_ns
               << ",\"sequence\":" << record.sequence
               << ",\"source\":\"";
        writeEscaped(output, record.source);
        output << "\",\"type\":\"";
        writeEscaped(output, record.type);
        output << '"';
        for (const auto& [key, value] : record.fields) {
            output << ",\"";
            writeEscaped(output, key);
            output << "\":\"";
            writeEscaped(output, value);
            output << '"';
        }
        output << "}\n";
    }
}

std::string TraceRecorder::jsonLines() const {
    std::ostringstream output;
    writeJsonLines(output);
    return output.str();
}

std::string TraceRecorder::hexBytes(const std::span<const std::uint8_t> bytes) {
    static constexpr char hexadecimal[] = "0123456789abcdef";
    std::string encoded;
    encoded.reserve(bytes.size() * 2U);
    for (const std::uint8_t byte : bytes) {
        encoded.push_back(hexadecimal[byte >> 4U]);
        encoded.push_back(hexadecimal[byte & 0x0fU]);
    }
    return encoded;
}

} // namespace fil::sim
