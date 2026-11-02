#pragma once

#include <array>
#include <bit>
#include <cstdint>

namespace fil::cpu {

/** Immutable ISA-only register-list decoding for native stack transfers. */
class NativeStackPlan {
public:
    constexpr NativeStackPlan() noexcept = default;
    constexpr NativeStackPlan(std::uint32_t mask, const bool push) noexcept : push_(push) {
        if (mask == 0U || (mask & 0xffff0000U) != 0U || (mask & (1U << 13U)) != 0U
            || (push && (mask & 0x8000U) != 0U)
            || (!push && (mask & 0xc000U) == 0xc000U)) return;
        while (mask != 0U) {
            registers_[count_++] = static_cast<std::uint8_t>(std::countr_zero(mask));
            mask &= mask - 1U;
        }
    }
    [[nodiscard]] constexpr std::uint32_t count() const noexcept { return count_; }
    [[nodiscard]] constexpr bool push() const noexcept { return push_; }
    [[nodiscard]] constexpr bool loadsPc() const noexcept {
        return !push_ && count_ != 0U && registers_[count_ - 1U] == 15U;
    }
    [[nodiscard]] constexpr std::uint32_t reg(const std::uint32_t index) const noexcept {
        return registers_[index];
    }
private:
    std::array<std::uint8_t, 16> registers_{};
    std::uint8_t count_{0U};
    bool push_{false};
};

} // namespace fil::cpu
