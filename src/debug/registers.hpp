#pragma once

/** @file registers.hpp
 *  @brief Internal target-description and register-layout helpers.
 */

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace fil::sim {
class Board;
}

namespace fil::debug::detail {

// Register order shared by target.xml, `g`, `p`, `P`, and the live sessions.
inline constexpr std::size_t sp_register_index = 13U;
inline constexpr std::size_t lr_register_index = 14U;
inline constexpr std::size_t pc_register_index = 15U;
inline constexpr std::size_t xpsr_register_index = 16U;
inline constexpr std::size_t msp_register_index = 17U;
inline constexpr std::size_t psp_register_index = 18U;
inline constexpr std::size_t control_register_index = 19U;
inline constexpr std::size_t primask_register_index = 20U;
inline constexpr std::size_t basepri_register_index = 21U;
inline constexpr std::size_t faultmask_register_index = 22U;
inline constexpr std::size_t double_register_base = 23U;
inline constexpr std::size_t double_register_count = 16U;
inline constexpr std::size_t fpscr_register_index = 39U;
inline constexpr std::size_t register_count = 40U;
inline constexpr std::size_t double_register_nibbles = 16U;
inline constexpr std::size_t single_register_nibbles = 8U;

/** @brief GDB target description: core integer, system, and FPv4-SP registers. */
[[nodiscard]] std::string_view targetDescription() noexcept;

/**
 * @brief Answers one `qXfer:features:read:` packet for the target description.
 * @return Nothing when the annex is not `target.xml` (unsupported).
 */
[[nodiscard]] std::optional<std::string> readFeaturePacket(std::string_view packet);

/** @brief Renders the complete `g`-packet register payload. */
[[nodiscard]] std::string readAllRegisters(const sim::Board& board);

/** @brief Renders one `p`-packet register value, or nothing for a bad index. */
[[nodiscard]] std::optional<std::string> readSingleRegister(
    const sim::Board& board, std::uint32_t index);

/** @brief Applies a complete `G`-packet register payload. */
[[nodiscard]] bool writeAllRegisters(sim::Board& board, std::string_view text);

/** @brief Applies one `P`-packet register value. */
[[nodiscard]] bool writeSingleRegister(
    sim::Board& board, std::uint32_t index, std::uint64_t value);

} // namespace fil::debug::detail
