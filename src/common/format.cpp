#include "fil/common/format.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <system_error>

namespace fil {

std::string hexValue(
    const std::uint64_t value,
    const unsigned int minimum_digits
) {
    std::array<char, 16> digits{};
    const auto [end, error] = std::to_chars(
        digits.data(), digits.data() + digits.size(), value, 16
    );
    if (error != std::errc{}) return "0x0";

    const std::size_t digit_count = static_cast<std::size_t>(end - digits.data());
    const std::size_t width = std::min<std::size_t>(minimum_digits, digits.size());
    std::string result;
    result.reserve(2U + std::max(width, digit_count));
    result = "0x";
    if (width > digit_count) result.append(width - digit_count, '0');
    result.append(digits.data(), digit_count);
    return result;
}

std::string hex32(const std::uint32_t value) {
    return hexValue(value, 8U);
}

} // namespace fil
