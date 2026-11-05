#include "fil/debug/live_gdb.hpp"

#include "fil/debug/gdb_server.hpp"
#include "fil/mem/memory_bus.hpp"
#include "fil/sim/board.hpp"
#include "registers.hpp"
#include "rsp.hpp"
#include "tcp_listener.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace fil::debug {
namespace {

using detail::parseHex;
using detail::parseHex32;

constexpr std::size_t maximum_memory_request = 32U * 1024U;

/** @brief Reads one memory byte, refusing side-effecting MMIO while running. */
bool readItem(
    sim::Board& board,
    const std::uint32_t address,
    const bool running,
    std::uint8_t& value
) {
    if (running && !board.memory().isBackedRange(address, 1U, false)) return false;
    const auto byte = board.memory().read8(
        address, mem::AccessContext{mem::AccessType::debug, 0U}
    );
    if (!byte) return false;
    value = byte.value();
    return true;
}

} // namespace

struct LiveGdbSession::Implementation {
    Implementation(sim::Board& board_value, RspTransport& transport_value)
        : board(board_value), connection(transport_value) {}

    sim::Board& board;
    detail::RspConnection connection;
    bool closed{false};

    void service(const bool running) {
        RspTransport& transport = connection.transport();
        while (!closed && transport.inputPending()) {
            const auto packet = connection.readPacket();
            if (!packet) {
                closed = true;
                return;
            }
            if (!handle(*packet, running)) closed = true;
        }
    }

    [[nodiscard]] bool reply(const std::string_view payload) {
        return connection.sendPacket(payload);
    }

    [[nodiscard]] bool handle(std::string_view packet, bool running);
    [[nodiscard]] bool handleQuery(std::string_view packet);
    [[nodiscard]] bool handleSet(std::string_view packet);
    [[nodiscard]] bool handleV(std::string_view packet);
    [[nodiscard]] bool handleMemoryRead(std::string_view packet, bool running);
};

bool LiveGdbSession::Implementation::handle(
    const std::string_view packet,
    const bool running
) {
    if (packet.empty()) return reply("");
    switch (packet.front()) {
    case '?': return reply("S05");
    case 'g': return reply(detail::readAllRegisters(board));
    case 'p': {
        const auto index = parseHex(packet.substr(1U));
        if (!index || *index >= detail::register_count) return reply("E01");
        const auto value = detail::readSingleRegister(
            board, static_cast<std::uint32_t>(*index)
        );
        if (!value) return reply("E01");
        return reply(*value);
    }
    case 'm': return handleMemoryRead(packet, running);
    case 'H':
    case 'T':
    case '!': return reply("OK");
    case 'q': return handleQuery(packet);
    case 'Q': return handleSet(packet);
    case 'v': return handleV(packet);
    case 'D':
        if (!reply("OK")) return false;
        closed = true;
        return true;
    default:
        // Writes, resumes, breakpoints, resets, and monitor commands are
        // refused: a live session may only observe.
        return reply("E01");
    }
}

bool LiveGdbSession::Implementation::handleQuery(const std::string_view packet) {
    if (packet == "qSupported" || packet.starts_with("qSupported:")) {
        return reply("PacketSize=4000;qXfer:features:read+;QStartNoAckMode+");
    }
    if (packet.starts_with("qXfer:features:read:")) {
        const auto feature = detail::readFeaturePacket(packet);
        return reply(feature ? *feature : "");
    }
    if (packet == "qfThreadInfo") return reply("m1");
    if (packet == "qsThreadInfo") return reply("l");
    if (packet == "qC" || packet.starts_with("qC")) return reply("QC1");
    if (packet == "qAttached" || packet.starts_with("qAttached:")) return reply("1");
    if (packet == "qOffsets") return reply("Text=0;Data=0;Bss=0");
    return reply("");
}

bool LiveGdbSession::Implementation::handleSet(const std::string_view packet) {
    if (packet == "QStartNoAckMode") {
        if (!reply("OK")) return false;
        connection.setNoAck(true);
        return true;
    }
    return reply("");
}

bool LiveGdbSession::Implementation::handleV(const std::string_view packet) {
    // No resume actions: a live session cannot continue, step, or run.
    if (packet.starts_with("vCont;")) return reply("E01");
    return reply("");
}

bool LiveGdbSession::Implementation::handleMemoryRead(
    const std::string_view packet,
    const bool running
) {
    const auto comma = packet.find(',');
    if (comma == std::string_view::npos) return reply("E01");
    const auto address = parseHex32(packet.substr(1U, comma - 1U));
    const auto length = parseHex(packet.substr(comma + 1U));
    if (!address || !length || *length > maximum_memory_request) return reply("E01");
    if (*length > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())
            - *address) {
        return reply("E01");
    }
    std::string text;
    text.reserve(static_cast<std::size_t>(*length) * 2U);
    for (std::uint64_t offset = 0U; offset < *length; ++offset) {
        std::uint8_t byte = 0U;
        if (!readItem(board, static_cast<std::uint32_t>(*address + offset), running, byte)) {
            return reply("E01");
        }
        text += detail::hexValue(byte, 1U);
    }
    return reply(text);
}

namespace {

using detail::setNoSigPipe;
using detail::setNonBlocking;
using detail::SocketTransport;

/** @brief Accepts and serves read-only GDB clients on a borrowed listener. */
class LiveGdbServer final : public DebugEndpoint {
public:
    LiveGdbServer(sim::Board& board, const int listener_descriptor)
        : board_(board), listener_(listener_descriptor) {}

    void service(const bool running) override {
        acceptClients();
        for (Client& client : clients_) {
            if (client.session->closed()) continue;
            client.session->service(running);
        }
        clients_.erase(
            std::remove_if(
                clients_.begin(), clients_.end(),
                [](const Client& client) { return client.session->closed(); }
            ),
            clients_.end()
        );
    }

    [[nodiscard]] bool active() const noexcept override { return true; }

private:
    struct Client {
        std::unique_ptr<SocketTransport> transport;
        std::unique_ptr<LiveGdbSession> session;
    };

    void acceptClients() {
        for (;;) {
            const int descriptor = ::accept(listener_, nullptr, nullptr);
            if (descriptor < 0) return;
            if (!setNonBlocking(descriptor)) {
                static_cast<void>(::close(descriptor));
                continue;
            }
            setNoSigPipe(descriptor);
            auto transport = std::make_unique<SocketTransport>(descriptor);
            auto session = std::make_unique<LiveGdbSession>(board_, *transport);
            clients_.push_back(Client{std::move(transport), std::move(session)});
        }
    }

    sim::Board& board_;
    int listener_{-1};
    std::vector<Client> clients_;
};

} // namespace

LiveGdbSession::LiveGdbSession(sim::Board& board, RspTransport& transport)
    : implementation_(std::make_unique<Implementation>(board, transport)) {}

LiveGdbSession::~LiveGdbSession() = default;

void LiveGdbSession::service(const bool running) {
    implementation_->service(running);
}

bool LiveGdbSession::closed() const noexcept {
    return implementation_->closed;
}

std::unique_ptr<DebugEndpoint> createLiveGdbServer(
    sim::Board& board,
    const int listener_descriptor
) {
    return std::make_unique<LiveGdbServer>(board, listener_descriptor);
}

} // namespace fil::debug
