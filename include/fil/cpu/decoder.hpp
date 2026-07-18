#pragma once

/** @file decoder.hpp
 *  @brief Table-driven Thumb and Thumb-2 instruction decoder.
 */

#include "fil/cpu/instruction.hpp"

#include <cstdint>
#include <optional>
#include <span>

namespace fil::cpu {

using DecodeFn16 = std::optional<DecodedInstruction> (*)(std::uint16_t);
using DecodeFn32 = std::optional<DecodedInstruction> (*)(std::uint16_t, std::uint16_t);

/** @brief One non-overlapping 16-bit decoder table row. */
struct DecodePattern16 {
    std::uint16_t mask{0};
    std::uint16_t value{0};
    DecodeFn16 decode{nullptr};
};

/** @brief One non-overlapping 32-bit decoder table row. */
struct DecodePattern32 {
    std::uint32_t mask{0};
    std::uint32_t value{0};
    DecodeFn32 decode{nullptr};
};

/** @brief Tests whether a first halfword introduces a 32-bit Thumb instruction. */
[[nodiscard]] constexpr bool is32BitThumbPrefix(const std::uint16_t halfword) noexcept {
    return (halfword & 0xf800U) >= 0xe800U;
}

/** @brief Decodes one 16-bit Thumb encoding, rejecting reserved forms. */
[[nodiscard]] std::optional<DecodedInstruction> decode16(std::uint16_t halfword) noexcept;

/** @brief Decodes two halfwords of one Thumb-2 encoding, in memory order. */
[[nodiscard]] std::optional<DecodedInstruction> decode32(
    std::uint16_t first,
    std::uint16_t second
) noexcept;

/** @brief Exposes the immutable 16-bit mask/value table for validation/tests. */
[[nodiscard]] std::span<const DecodePattern16> decodePatterns16() noexcept;

/** @brief Exposes the immutable 32-bit mask/value table for validation/tests. */
[[nodiscard]] std::span<const DecodePattern32> decodePatterns32() noexcept;

/** @brief Proves that no two rows in either decoder table can match one encoding. */
[[nodiscard]] bool decoderTablesHaveNoOverlaps() noexcept;

} // namespace fil::cpu
