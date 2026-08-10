#pragma once

/** @file numeric.hpp
 *  @brief Small checked-integer helpers shared by emulator subsystems.
 */

#include <concepts>
#include <limits>
#include <type_traits>

namespace fil {

/** @brief Adds unsigned values and clamps overflow to the type's maximum. */
template <std::unsigned_integral T, std::unsigned_integral U>
    requires (std::numeric_limits<U>::digits <= std::numeric_limits<T>::digits)
[[nodiscard]] constexpr T saturatingAdd(const T left, const U right) noexcept {
    const T converted_right = static_cast<T>(right);
    return converted_right > std::numeric_limits<T>::max() - left
        ? std::numeric_limits<T>::max()
        : left + converted_right;
}

/** @brief Tests whether `[offset, offset + size)` fits inside `total`. */
template <std::integral Offset, std::integral Size, std::integral Total>
[[nodiscard]] constexpr bool rangeFits(
    const Offset offset,
    const Size size,
    const Total total
) noexcept {
    const auto is_nonnegative = []<std::integral T>(const T value) {
        if constexpr (std::signed_integral<T>) return value >= 0;
        return true;
    };
    if (!is_nonnegative(offset) || !is_nonnegative(size)
        || !is_nonnegative(total)) {
        return false;
    }

    using Common = std::common_type_t<
        std::make_unsigned_t<Offset>,
        std::make_unsigned_t<Size>,
        std::make_unsigned_t<Total>
    >;
    const Common common_offset = static_cast<Common>(offset);
    const Common common_size = static_cast<Common>(size);
    const Common common_total = static_cast<Common>(total);
    return common_offset <= common_total
        && common_size <= common_total - common_offset;
}

} // namespace fil
