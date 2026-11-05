#pragma once

/** @file tcp_listener.hpp
 *  @brief Internal listening-socket helper shared by the debug endpoints.
 */

#include "fil/common/result.hpp"
#include "fil/debug/gdb_server.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace fil::debug::detail {

/** @brief Movable owner of one non-blocking listening TCP socket. */
class TcpListener {
public:
    TcpListener() = default;
    ~TcpListener();
    TcpListener(const TcpListener&) = delete;
    TcpListener& operator=(const TcpListener&) = delete;
    TcpListener(TcpListener&& other) noexcept;
    TcpListener& operator=(TcpListener&& other) noexcept;

    [[nodiscard]] int descriptor() const noexcept { return descriptor_; }
    [[nodiscard]] bool valid() const noexcept { return descriptor_ >= 0; }

private:
    friend Result<TcpListener> openTcpListener(
        const std::string& bind_address, std::uint16_t port, std::uint16_t& bound_port);

    explicit TcpListener(const int descriptor) : descriptor_(descriptor) {}

    int descriptor_{-1};
};

/**
 * @brief Binds and listens on one local TCP endpoint.
 * @param bind_address Local address; empty selects every interface.
 * @param port Requested port; zero selects an ephemeral port.
 * @param bound_port Receives the actual bound port.
 * @return Listener or a structured bind/resolve failure.
 */
[[nodiscard]] Result<TcpListener> openTcpListener(
    const std::string& bind_address,
    std::uint16_t port,
    std::uint16_t& bound_port
);

/** @brief Enables non-blocking mode on one descriptor. */
[[nodiscard]] bool setNonBlocking(int descriptor) noexcept;

/** @brief Restores blocking mode on one descriptor. */
[[nodiscard]] bool setBlocking(int descriptor) noexcept;

/** @brief Suppresses SIGPIPE for one socket where the platform supports it. */
void setNoSigPipe(int descriptor) noexcept;

/** @brief Suppresses SIGPIPE on an outgoing send where the platform supports it. */
[[nodiscard]] constexpr int socketSendFlags() noexcept {
#if defined(MSG_NOSIGNAL)
    return MSG_NOSIGNAL;
#else
    return 0;
#endif
}

/** @brief Blocking TCP byte transport owned by one accepted connection. */
class SocketTransport final : public RspTransport {
public:
    explicit SocketTransport(int descriptor);
    ~SocketTransport() override;
    SocketTransport(const SocketTransport&) = delete;
    SocketTransport& operator=(const SocketTransport&) = delete;

    /** @brief Reads the next byte, blocking; empty signals orderly end of input. */
    [[nodiscard]] std::optional<std::uint8_t> readByte() override;
    /** @brief Reports whether unread peer input is ready without consuming it. */
    [[nodiscard]] bool inputPending() override;
    /** @brief Blocks until input is readable, the timeout elapses, or the peer closes. */
    [[nodiscard]] TransportWait waitForInput(std::chrono::milliseconds timeout) override;
    /** @brief Writes one complete frame. @return False on transport failure. */
    [[nodiscard]] bool write(std::string_view bytes) override;

private:
    int descriptor_{-1};
};

} // namespace fil::debug::detail
