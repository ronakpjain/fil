#pragma once

/** @file gdb_server.hpp
 *  @brief GDB Remote Serial Protocol server for one emulated board.
 */

#include "fil/common/result.hpp"

#include <chrono>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace fil::sim {
class Board;
}

namespace fil::debug {

class MonitorEndpoint;

/** @brief TCP endpoints and execution-slice controls for one debug session. */
struct GdbServerOptions {
    std::string bind_address{"127.0.0.1"};         ///< Local address to listen on.
    std::uint16_t port{3333};                      ///< GDB TCP port; zero selects an ephemeral port.
    bool telnet_enabled{true};                     ///< Whether to serve the telnet monitor endpoint.
    std::uint16_t telnet_port{4444};               ///< Telnet port; zero selects an ephemeral port.
    bool once{false};                              ///< Exit after the first GDB session ends.
    std::uint64_t slice_instructions{262'144};     ///< Instructions per interruptible run slice.
    std::uint64_t slice_time_ns{1'000'000};        ///< Simulated time per run slice; zero disables.
};

/** @brief Outcome of waiting for debugger input. */
enum class TransportWait : std::uint8_t {
    ready,   ///< At least one byte is readable.
    timeout, ///< No input arrived before the timeout.
    closed,  ///< The peer closed the transport.
};

/** @brief Stable reason one RSP session ended. */
enum class GdbSessionEnd : std::uint8_t {
    detached,     ///< GDB detached or released the target.
    killed,       ///< GDB killed the target.
    disconnected, ///< The transport reached end of input.
};

/** @brief Stable lowercase name for a session end reason. */
[[nodiscard]] std::string_view gdbSessionEndName(GdbSessionEnd end) noexcept;

/**
 * @brief Abstract byte transport for one RSP session.
 *
 * Production sessions use a TCP socket; tests supply scripted transports so
 * the protocol logic is exercised without sockets or threads.
 */
class RspTransport {
public:
    virtual ~RspTransport() = default;

    /** @brief Reads the next byte, blocking; empty signals orderly end of input. */
    [[nodiscard]] virtual std::optional<std::uint8_t> readByte() = 0;
    /** @brief Reports whether unread peer input is ready without consuming it. */
    [[nodiscard]] virtual bool inputPending() = 0;
    /** @brief Blocks until input is readable, the timeout elapses, or the peer closes. */
    [[nodiscard]] virtual TransportWait waitForInput(std::chrono::milliseconds timeout) = 0;
    /** @brief Writes one complete frame. @return False on transport failure. */
    [[nodiscard]] virtual bool write(std::string_view bytes) = 0;
};

/**
 * @brief One GDB Remote Serial Protocol session against a board.
 *
 * The session owns no thread and no socket; it drives the referenced board
 * directly. Continue and step requests execute through bounded board slices so
 * an interrupting `0x03` byte is observed promptly.
 */
class GdbSession {
public:
    /**
     * @brief Creates a session over an already-connected transport.
     * @param board Board controlled by the debugger.
     * @param transport Byte transport that stays alive for the session.
     * @param options Endpoint and slice controls.
     * @param monitor Optional monitor endpoint serviced while idle and running.
     */
    GdbSession(
        sim::Board& board,
        RspTransport& transport,
        GdbServerOptions options = {},
        MonitorEndpoint* monitor = nullptr
    );
    ~GdbSession();
    GdbSession(const GdbSession&) = delete;
    GdbSession& operator=(const GdbSession&) = delete;
    GdbSession(GdbSession&&) = delete;
    GdbSession& operator=(GdbSession&&) = delete;

    /** @brief Serves packets until detach, kill, or end of input. */
    [[nodiscard]] Result<GdbSessionEnd> run();

private:
    struct Implementation;
    std::unique_ptr<Implementation> implementation_;
};

/**
 * @brief Listens on a TCP endpoint and serves GDB sessions.
 *
 * Prints the listening endpoint before blocking for a connection and reports
 * each connection and outcome on the supplied streams. Unless
 * `options.once` is set, another debugger may attach after a session ends;
 * the telnet monitor stays available throughout.
 * @param board Board controlled by the debugger.
 * @param options Endpoint and slice controls.
 * @param out Human-readable progress stream.
 * @return Session end reason or a structured bind/accept failure.
 */
[[nodiscard]] Result<GdbSessionEnd> serveGdb(
    sim::Board& board,
    const GdbServerOptions& options,
    std::ostream& out
);

} // namespace fil::debug
