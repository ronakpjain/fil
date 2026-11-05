#pragma once

/** @file monitor.hpp
 *  @brief Telnet monitor endpoint for live board inspection without halting.
 */

#include "fil/common/result.hpp"
#include "fil/debug/endpoint.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

namespace fil::sim {
class Board;
}

namespace fil::debug {

/** @brief Reply text and connection effect of one monitor command. */
struct MonitorReply {
    std::string output; ///< LF-terminated response text; may be empty.
    bool close{false};  ///< True when the client asked to disconnect.
};

/**
 * @brief Executes one monitor command line against a board.
 *
 * Commands are serviced from the debug session at instruction boundaries, so
 * the target is never halted to answer them. While `running` is true only
 * backed RAM/ROM/flash reads are permitted: those never change
 * firmware-visible state, so live inspection cannot perturb execution.
 * Registers and board information remain readable while running.
 *
 * @param board Target board.
 * @param line Command line without trailing line ending.
 * @param running True while a continue request is executing.
 * @return Response text and connection effect.
 */
[[nodiscard]] MonitorReply executeMonitorCommand(
    sim::Board& board,
    std::string_view line,
    bool running
);

/** @brief Telnet monitor endpoint serviced by the debug session. */
class MonitorEndpoint : public DebugEndpoint {};

/**
 * @brief Creates and binds the TCP telnet monitor endpoint.
 *
 * The endpoint accepts several concurrent clients, strips telnet command
 * sequences, and answers complete command lines during `service()`.
 * @param board Board exposed by monitor commands.
 * @param bind_address Local address; empty selects every interface.
 * @param port Requested port; zero selects an ephemeral port.
 * @param bound_port Receives the actual bound port.
 * @return Endpoint or a structured bind/resolve failure.
 */
[[nodiscard]] Result<std::unique_ptr<MonitorEndpoint>> createTcpMonitorEndpoint(
    sim::Board& board,
    const std::string& bind_address,
    std::uint16_t port,
    std::uint16_t& bound_port
);

} // namespace fil::debug
