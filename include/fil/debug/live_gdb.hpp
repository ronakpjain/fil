#pragma once

/** @file live_gdb.hpp
 *  @brief Concurrent read-only GDB sessions for live variable inspection.
 */

#include "fil/debug/endpoint.hpp"

#include <cstdint>
#include <memory>

namespace fil::sim {
class Board;
}

namespace fil::debug {

class RspTransport;

/**
 * @brief Read-only RSP session served alongside the primary debugger.
 *
 * Live sessions answer identification, register, and memory requests at
 * instruction boundaries while the primary debugger keeps the target running.
 * Every target-control or write request is refused, and while the target runs
 * only RAM/ROM/flash reads are allowed, so live inspection can never perturb
 * execution.
 *
 * This is the second connection cortex-debug's Live Watch opens; it is not a
 * general-purpose debug session.
 */
class LiveGdbSession {
public:
    LiveGdbSession(sim::Board& board, RspTransport& transport);
    ~LiveGdbSession();
    LiveGdbSession(const LiveGdbSession&) = delete;
    LiveGdbSession& operator=(const LiveGdbSession&) = delete;
    LiveGdbSession(LiveGdbSession&&) = delete;
    LiveGdbSession& operator=(LiveGdbSession&&) = delete;

    /** @brief Reads and answers every pending packet without blocking. */
    void service(bool running);

    /** @brief Whether the peer detached or the transport ended. */
    [[nodiscard]] bool closed() const noexcept;

private:
    struct Implementation;
    std::unique_ptr<Implementation> implementation_;
};

/**
 * @brief Accepts live read-only GDB clients from an already-listening socket.
 *
 * The listener stays owned by the caller; accepted clients are served when the
 * endpoint is serviced by the primary session.
 * @param board Board exposed to live sessions.
 * @param listener_descriptor Borrowed non-blocking listening descriptor.
 * @return Endpoint owned by the caller.
 */
[[nodiscard]] std::unique_ptr<DebugEndpoint> createLiveGdbServer(
    sim::Board& board,
    int listener_descriptor
);

} // namespace fil::debug
