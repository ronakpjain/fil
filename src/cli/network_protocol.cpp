#include "fil/cli/network_protocol.hpp"

#include "fil/common/error.hpp"

#include <algorithm>
#include <array>
#include <ostream>
#include <string>
#include <utility>

namespace fil::cli::network_protocol {
namespace {

constexpr std::array<std::uint8_t, 4U> magic{'F', 'I', 'L', 'N'};

std::uint16_t readU16(const std::span<const std::uint8_t> bytes, const std::size_t offset) {
    return static_cast<std::uint16_t>(bytes[offset])
        | static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[offset + 1U]) << 8U);
}

std::uint32_t readU32(const std::span<const std::uint8_t> bytes, const std::size_t offset) {
    return static_cast<std::uint32_t>(bytes[offset])
        | (static_cast<std::uint32_t>(bytes[offset + 1U]) << 8U)
        | (static_cast<std::uint32_t>(bytes[offset + 2U]) << 16U)
        | (static_cast<std::uint32_t>(bytes[offset + 3U]) << 24U);
}

void writeU16(std::span<std::uint8_t> bytes, const std::size_t offset, const std::uint16_t value) {
    bytes[offset] = static_cast<std::uint8_t>(value & 0xffU);
    bytes[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

void writeU32(std::span<std::uint8_t> bytes, const std::size_t offset, const std::uint32_t value) {
    for (unsigned int index = 0U; index < 4U; ++index) {
        bytes[offset + index] = static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU);
    }
}

Error parseError(std::string message) {
    return Error{ErrorCategory::parse, std::move(message), std::nullopt};
}

} // namespace

Result<void> FrameDecoder::append(const std::span<const std::uint8_t> bytes) {
    if (bytes.size() > maximum_buffered_size - bytes_.size()) {
        return parseError("stdio protocol input buffer exceeds its limit");
    }
    bytes_.insert(bytes_.end(), bytes.begin(), bytes.end());
    return {};
}

Result<std::optional<Frame>> FrameDecoder::next() {
    if (bytes_.size() < header_size) return std::optional<Frame>{};

    const std::span<const std::uint8_t> view{bytes_};
    for (std::size_t index = 0U; index < magic.size(); ++index) {
        if (view[index] != magic[index]) return parseError("invalid stdio protocol magic");
    }
    if (view[4U] != protocol_version) return parseError("unsupported stdio protocol version");
    if (readU16(view, 6U) != 0U) return parseError("stdio protocol flags must be zero in version 1");

    const std::uint32_t payload_size = readU32(view, 8U);
    if (payload_size > maximum_payload_size) {
        return parseError("stdio protocol payload exceeds the 64 KiB limit");
    }
    const std::size_t frame_size = header_size + static_cast<std::size_t>(payload_size);
    if (bytes_.size() < frame_size) return std::optional<Frame>{};

    Frame frame;
    frame.kind = static_cast<MessageKind>(view[5U]);
    frame.request_id = readU32(view, 12U);
    frame.payload.assign(
        bytes_.begin() + static_cast<std::ptrdiff_t>(header_size),
        bytes_.begin() + static_cast<std::ptrdiff_t>(frame_size)
    );
    bytes_.erase(bytes_.begin(), bytes_.begin() + static_cast<std::ptrdiff_t>(frame_size));
    return std::optional<Frame>{std::move(frame)};
}

Result<void> FrameDecoder::finish() const {
    if (!bytes_.empty()) return parseError("truncated stdio protocol frame at end of input");
    return {};
}

Result<void> writeFrame(std::ostream& output, const Frame& frame) {
    if (frame.payload.size() > maximum_payload_size) {
        return Error{ErrorCategory::invalid_argument,
                     "stdio protocol payload exceeds the 64 KiB limit", std::nullopt};
    }

    std::array<std::uint8_t, header_size> header{};
    std::copy(magic.begin(), magic.end(), header.begin());
    header[4U] = protocol_version;
    header[5U] = static_cast<std::uint8_t>(frame.kind);
    writeU16(header, 6U, 0U);
    writeU32(header, 8U, static_cast<std::uint32_t>(frame.payload.size()));
    writeU32(header, 12U, frame.request_id);

    output.write(reinterpret_cast<const char*>(header.data()),
                 static_cast<std::streamsize>(header.size()));
    if (!frame.payload.empty()) {
        output.write(reinterpret_cast<const char*>(frame.payload.data()),
                     static_cast<std::streamsize>(frame.payload.size()));
    }
    output.flush();
    if (!output) {
        return Error{ErrorCategory::io, "failed to write stdio protocol frame", std::nullopt};
    }
    return {};
}

} // namespace fil::cli::network_protocol
