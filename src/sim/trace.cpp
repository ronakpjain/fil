#include "fil/sim/trace.hpp"

#include <algorithm>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <utility>

namespace fil::sim {
namespace {

void writeEscaped(std::ostream& output, const std::string_view text) {
    static constexpr char hexadecimal[] = "0123456789abcdef";
    for (const char character : text) {
        const auto byte = static_cast<unsigned char>(character);
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

bool TraceRecorder::passesFilter(const std::string_view type) const noexcept {
    if (allowlist_.empty()) return true;
    return std::find(allowlist_.begin(), allowlist_.end(), type) != allowlist_.end();
}

std::uint64_t TraceRecorder::record(
    const SimTimeNs time_ns,
    std::string source,
    std::string type,
    std::vector<TraceField> fields
) {
    if (!enabled_) return next_sequence_;
    if (!passesFilter(type)) return next_sequence_++;
    const std::uint64_t sequence = next_sequence_++;
    records_.push_back(TraceRecord{
        time_ns, sequence, std::move(source), std::move(type), std::move(fields),
    });
    if (observer_) {
        const TraceRecord observed = records_.back();
        observer_(observed);
    }
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
    if (!passesFilter(transmit ? "can_tx" : "can_rx")) return next_sequence_++;
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
