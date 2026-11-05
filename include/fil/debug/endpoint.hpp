#pragma once

/** @file endpoint.hpp
 *  @brief Auxiliary debug endpoints serviced at instruction boundaries.
 */

namespace fil::debug {

/**
 * @brief Secondary request endpoint driven by the primary debug session.
 *
 * The telnet monitor and the live read-only GDB sessions implement this seam.
 * They are serviced by the primary session between execution slices, so they
 * never block or halt the emulated target.
 */
class DebugEndpoint {
public:
    virtual ~DebugEndpoint() = default;

    /**
     * @brief Accepts clients and answers every pending request without blocking.
     * @param running Whether the primary debugger currently keeps the target running.
     */
    virtual void service(bool running) = 0;

    /** @brief Whether a listener or connected client needs periodic servicing. */
    [[nodiscard]] virtual bool active() const noexcept = 0;
};

} // namespace fil::debug
