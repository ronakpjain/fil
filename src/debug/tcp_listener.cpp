#include "tcp_listener.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace fil::debug::detail {

TcpListener::~TcpListener() {
    if (descriptor_ >= 0) static_cast<void>(::close(descriptor_));
}

TcpListener::TcpListener(TcpListener&& other) noexcept : descriptor_(other.descriptor_) {
    other.descriptor_ = -1;
}

TcpListener& TcpListener::operator=(TcpListener&& other) noexcept {
    if (this == &other) return *this;
    if (descriptor_ >= 0) static_cast<void>(::close(descriptor_));
    descriptor_ = other.descriptor_;
    other.descriptor_ = -1;
    return *this;
}

bool setNonBlocking(const int descriptor) noexcept {
    const int current = ::fcntl(descriptor, F_GETFL, 0);
    if (current < 0) return false;
    return ::fcntl(descriptor, F_SETFL, current | O_NONBLOCK) == 0;
}

bool setBlocking(const int descriptor) noexcept {
    const int current = ::fcntl(descriptor, F_GETFL, 0);
    if (current < 0) return false;
    if ((current & O_NONBLOCK) == 0) return true;
    return ::fcntl(descriptor, F_SETFL, current & ~O_NONBLOCK) == 0;
}

void setNoSigPipe(const int descriptor) noexcept {
#if defined(SO_NOSIGPIPE)
    const int enabled = 1;
    static_cast<void>(::setsockopt(
        descriptor, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled)
    ));
#else
    static_cast<void>(descriptor);
#endif
}

Result<TcpListener> openTcpListener(
    const std::string& bind_address,
    const std::uint16_t port,
    std::uint16_t& bound_port
) {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    addrinfo* resolved = nullptr;
    const std::string service = std::to_string(port);
    const int status = ::getaddrinfo(
        bind_address.empty() ? nullptr : bind_address.c_str(),
        service.c_str(), &hints, &resolved
    );
    if (status != 0) {
        return Error{
            ErrorCategory::io,
            "unable to resolve listen address '" + bind_address + "': " + ::gai_strerror(status),
            std::nullopt
        };
    }
    std::unique_ptr<addrinfo, void (*)(addrinfo*)> guard(resolved, ::freeaddrinfo);
    std::string last_error = "no usable address";
    for (addrinfo* candidate = resolved; candidate != nullptr; candidate = candidate->ai_next) {
        const int descriptor = ::socket(
            candidate->ai_family, candidate->ai_socktype, candidate->ai_protocol
        );
        if (descriptor < 0) {
            last_error = std::strerror(errno);
            continue;
        }
        const int enabled = 1;
        static_cast<void>(::setsockopt(
            descriptor, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled)
        ));
        setNoSigPipe(descriptor);
        if (::bind(descriptor, candidate->ai_addr, candidate->ai_addrlen) == 0
            && ::listen(descriptor, 4) == 0) {
            if (!setNonBlocking(descriptor)) {
                last_error = std::strerror(errno);
                static_cast<void>(::close(descriptor));
                continue;
            }
            bound_port = port;
            if (port == 0U) {
                sockaddr_storage local{};
                socklen_t length = sizeof(local);
                if (::getsockname(
                        descriptor, reinterpret_cast<sockaddr*>(&local), &length) == 0) {
                    if (local.ss_family == AF_INET) {
                        bound_port = ntohs(reinterpret_cast<sockaddr_in*>(&local)->sin_port);
                    } else if (local.ss_family == AF_INET6) {
                        bound_port = ntohs(reinterpret_cast<sockaddr_in6*>(&local)->sin6_port);
                    }
                }
            }
            return TcpListener(descriptor);
        }
        last_error = std::strerror(errno);
        static_cast<void>(::close(descriptor));
    }
    return Error{
        ErrorCategory::io,
        "unable to listen on " + bind_address + ":" + service + ": " + last_error,
        std::nullopt
    };
}

} // namespace fil::debug::detail
