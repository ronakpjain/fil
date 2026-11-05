#include "rsp.hpp"

#include <algorithm>
#include <limits>

namespace fil::debug::detail {

constexpr std::size_t maximum_packet_bytes = 64U * 1024U;

int hexDigitValue(const char character) noexcept {
    if (character >= '0' && character <= '9') return character - '0';
    if (character >= 'a' && character <= 'f') return character - 'a' + 10;
    if (character >= 'A' && character <= 'F') return character - 'A' + 10;
    return -1;
}

std::string hexValue(const std::uint64_t value, const std::size_t bytes) {
    std::string text(bytes * 2U, '0');
    for (std::size_t index = 0U; index < bytes; ++index) {
        const auto digit = static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU);
        text[index * 2U] = hex_digits[digit >> 4U];
        text[index * 2U + 1U] = hex_digits[digit & 0x0fU];
    }
    return text;
}

std::optional<std::uint64_t> parseHex(const std::string_view text) {
    if (text.empty() || text.size() > 16U) return std::nullopt;
    std::uint64_t value = 0U;
    for (const char character : text) {
        const int digit = hexDigitValue(character);
        if (digit < 0) return std::nullopt;
        value = (value << 4U) | static_cast<std::uint64_t>(digit);
    }
    return value;
}

std::optional<std::uint32_t> parseHex32(const std::string_view text) {
    const auto value = parseHex(text);
    if (!value || *value > std::numeric_limits<std::uint32_t>::max()) return std::nullopt;
    return static_cast<std::uint32_t>(*value);
}

std::optional<std::uint64_t> parseLittleEndianHex(const std::string_view text) {
    if (text.empty() || text.size() % 2U != 0U || text.size() > 16U) return std::nullopt;
    std::uint64_t value = 0U;
    for (std::size_t position = 0U; position < text.size(); position += 2U) {
        const int high = hexDigitValue(text[position]);
        const int low = hexDigitValue(text[position + 1U]);
        if (high < 0 || low < 0) return std::nullopt;
        value |= static_cast<std::uint64_t>((high << 4) | low) << (position * 4U);
    }
    return value;
}

std::optional<std::vector<std::uint8_t>> decodeHexBytes(const std::string_view text) {
    if (text.size() % 2U != 0U) return std::nullopt;
    std::vector<std::uint8_t> bytes;
    bytes.reserve(text.size() / 2U);
    for (std::size_t index = 0U; index < text.size(); index += 2U) {
        const int high = hexDigitValue(text[index]);
        const int low = hexDigitValue(text[index + 1U]);
        if (high < 0 || low < 0) return std::nullopt;
        bytes.push_back(static_cast<std::uint8_t>((high << 4) | low));
    }
    return bytes;
}

std::optional<std::vector<std::uint8_t>> decodeBinaryBytes(const std::string_view text) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(text.size());
    for (std::size_t index = 0U; index < text.size(); ++index) {
        auto byte = static_cast<std::uint8_t>(text[index]);
        if (byte == static_cast<std::uint8_t>('}')) {
            if (++index >= text.size()) return std::nullopt;
            byte = static_cast<std::uint8_t>(
                static_cast<std::uint8_t>(text[index]) ^ 0x20U
            );
        }
        bytes.push_back(byte);
    }
    return bytes;
}

std::optional<std::string> RspConnection::readPacket() {
    for (;;) {
        const auto byte = transport_->readByte();
        if (!byte) return std::nullopt;
        if (*byte == static_cast<std::uint8_t>('+')) continue;
        if (*byte == static_cast<std::uint8_t>('-')) {
            // The peer rejected the previous frame; resend it.
            if (have_last_frame_ && !transport_->write(last_frame_)) return std::nullopt;
            continue;
        }
        if (*byte != static_cast<std::uint8_t>('$')) continue; // Stray interrupt or noise.
        std::string payload;
        for (;;) {
            const auto next = transport_->readByte();
            if (!next) return std::nullopt;
            if (*next == static_cast<std::uint8_t>('#')) break;
            if (payload.size() >= maximum_packet_bytes) return std::nullopt;
            payload.push_back(static_cast<char>(*next));
        }
        const auto high = transport_->readByte();
        const auto low = transport_->readByte();
        if (!high || !low) return std::nullopt;
        std::uint8_t checksum = 0U;
        for (const char character : payload) {
            checksum = static_cast<std::uint8_t>(
                checksum + static_cast<std::uint8_t>(character)
            );
        }
        const int high_digit = hexDigitValue(static_cast<char>(*high));
        const int low_digit = hexDigitValue(static_cast<char>(*low));
        const auto expected = static_cast<std::uint8_t>(
            ((high_digit < 0 ? 0 : high_digit) << 4) | (low_digit < 0 ? 0 : low_digit)
        );
        if (checksum != expected) {
            if (!no_ack_ && !transport_->write("-")) return std::nullopt;
            continue;
        }
        if (!no_ack_ && !transport_->write("+")) return std::nullopt;
        return payload;
    }
}

bool RspConnection::sendPacket(const std::string_view payload) {
    std::string frame;
    frame.reserve(payload.size() + 4U);
    frame.push_back('$');
    frame.append(payload);
    frame.push_back('#');
    std::uint8_t checksum = 0U;
    for (const char character : payload) {
        checksum = static_cast<std::uint8_t>(
            checksum + static_cast<std::uint8_t>(character)
        );
    }
    frame += hexValue(checksum, 1U);
    last_frame_ = frame;
    have_last_frame_ = true;
    if (!transport_->write(frame)) return false;
    if (no_ack_) return true;
    for (;;) {
        const auto byte = transport_->readByte();
        if (!byte) return false;
        if (*byte == static_cast<std::uint8_t>('+')) return true;
        if (*byte == static_cast<std::uint8_t>('-')) {
            if (!transport_->write(frame)) return false;
        }
    }
}

bool RspConnection::sendConsole(const std::string_view text) {
    std::string payload = "O";
    for (const char character : text) {
        payload += hexValue(static_cast<std::uint8_t>(character), 1U);
    }
    return sendPacket(payload);
}

} // namespace fil::debug::detail
