#pragma once

/** @file rsp.hpp
 *  @brief Internal RSP framing, acknowledgement, and value codecs.
 */

#include "fil/debug/gdb_server.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace fil::debug::detail {

inline constexpr char hex_digits[] = "0123456789abcdef";

/** @brief Value of one hexadecimal digit, or -1 when the character is not one. */
int hexDigitValue(char character) noexcept;

/** @brief Renders a little-endian value as exactly `bytes` hexadecimal digits. */
std::string hexValue(std::uint64_t value, std::size_t bytes);

/** @brief Parses up to sixteen hexadecimal digits as a plain value. */
std::optional<std::uint64_t> parseHex(std::string_view text);

/** @brief Parses a hexadecimal value that must fit the 32-bit target address space. */
std::optional<std::uint32_t> parseHex32(std::string_view text);

/**
 * @brief Parses a little-endian hexadecimal byte string into one value.
 *
 * RSP `g`, `G`, `p`, and `P` payloads carry register bytes in target order,
 * so `deadbeef` is the value 0xefbeadde.
 */
std::optional<std::uint64_t> parseLittleEndianHex(std::string_view text);

/** @brief Decodes an even-length hexadecimal byte string into raw bytes. */
std::optional<std::vector<std::uint8_t>> decodeHexBytes(std::string_view text);

/** @brief Decodes the `X` packet's `}`-escaped binary payload. */
std::optional<std::vector<std::uint8_t>> decodeBinaryBytes(std::string_view text);

/** @brief Framed RSP packet I/O over one transport, including ack mode. */
class RspConnection {
public:
    explicit RspConnection(RspTransport& transport) : transport_(&transport) {}
    ~RspConnection() = default;
    RspConnection(const RspConnection&) = delete;
    RspConnection& operator=(const RspConnection&) = delete;

    [[nodiscard]] RspTransport& transport() noexcept { return *transport_; }
    [[nodiscard]] bool noAck() const noexcept { return no_ack_; }
    void setNoAck(const bool value) noexcept { no_ack_ = value; }

    /** @brief Reads one complete packet; empty on end of input. */
    [[nodiscard]] std::optional<std::string> readPacket();

    /** @brief Writes one packet and, in ack mode, waits for acknowledgement. */
    [[nodiscard]] bool sendPacket(std::string_view payload);

    /** @brief Writes one `O` console-output packet. */
    [[nodiscard]] bool sendConsole(std::string_view text);

private:
    RspTransport* transport_;
    std::string last_frame_;
    bool have_last_frame_{false};
    bool no_ack_{false};
};

} // namespace fil::debug::detail
