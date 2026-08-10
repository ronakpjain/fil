#pragma once

/** @file format.hpp
 *  @brief Stable formatting helpers shared by diagnostics and traces.
 */

#include <cstdint>
#include <string>

namespace fil {

/** @brief Formats an unsigned value as lowercase hexadecimal with a `0x` prefix. */
[[nodiscard]] std::string hexValue(
    std::uint64_t value,
    unsigned int minimum_digits = 0U
);

/** @brief Formats a 32-bit target value as eight lowercase hexadecimal digits. */
[[nodiscard]] std::string hex32(std::uint32_t value);

} // namespace fil
