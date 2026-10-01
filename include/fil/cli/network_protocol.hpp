#pragma once

/** @file network_protocol.hpp
 *  @brief Framing primitives for the serve-network stdio protocol.
 */

#include "fil/common/result.hpp"

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <span>
#include <vector>

namespace fil::cli::network_protocol {

inline constexpr std::size_t header_size = 16U;
inline constexpr std::uint32_t maximum_payload_size = 64U * 1024U;
inline constexpr std::size_t maximum_buffered_size =
    static_cast<std::size_t>(maximum_payload_size) + header_size + 4096U;
inline constexpr std::uint8_t protocol_version = 1U;

enum class MessageKind : std::uint8_t {
    can_inject = 0x01U,
    adc_set = 0x02U,
    gpio_set = 0x03U,
    stop = 0x04U,
    hello = 0x80U,
    reply = 0x81U,
    trace = 0x82U,
    end = 0x83U,
};

/** @brief One decoded protocol frame. Unknown message kinds are preserved. */
struct Frame {
    MessageKind kind{MessageKind::stop};
    std::uint32_t request_id{0U};
    std::vector<std::uint8_t> payload;
};

/**
 * @brief Incremental bounded decoder for the fixed-header stdio wire format.
 *
 * Call append() with available bytes, then next() until it returns an empty
 * optional. finish() distinguishes clean EOF from a truncated frame.
 */
class FrameDecoder {
public:
    [[nodiscard]] Result<void> append(std::span<const std::uint8_t> bytes);
    [[nodiscard]] Result<std::optional<Frame>> next();
    [[nodiscard]] Result<void> finish() const;
    [[nodiscard]] std::size_t bufferedBytes() const noexcept { return bytes_.size(); }

private:
    std::vector<std::uint8_t> bytes_;
};

/** @brief Writes one complete v1 frame and flushes the stream. */
[[nodiscard]] Result<void> writeFrame(std::ostream& output, const Frame& frame);

} // namespace fil::cli::network_protocol
