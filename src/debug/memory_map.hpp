#pragma once

/** @file memory_map.hpp
 *  @brief Internal GDB memory-map description of the emulated target.
 */

#include <optional>
#include <string>
#include <string_view>

namespace fil::sim {
class Board;
}

namespace fil::debug::detail {

/**
 * @brief Answers one `qXfer:memory-map:read` packet.
 * @return Nothing when the packet is not a memory-map request.
 */
[[nodiscard]] std::optional<std::string> readMemoryMapPacket(
    const sim::Board& board, std::string_view packet);

} // namespace fil::debug::detail
