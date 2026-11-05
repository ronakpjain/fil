#include "fil/debug/monitor.hpp"

#include "fil/common/format.hpp"
#include "fil/config/config.hpp"
#include "fil/cpu/cortex_m4.hpp"
#include "tcp_listener.hpp"
#include "fil/mem/memory_bus.hpp"
#include "fil/sim/board.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <cerrno>
#include <charconv>
#include <cstddef>
#include <cstring>
#include <limits>
#include <optional>
#include <sstream>
#include <span>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace fil::debug {
namespace {

constexpr std::uint8_t telnet_iac = 255U;
constexpr std::uint8_t telnet_dont = 254U;
constexpr std::uint8_t telnet_do = 253U;
constexpr std::uint8_t telnet_wont = 252U;
constexpr std::uint8_t telnet_will = 251U;
constexpr std::uint8_t telnet_echo = 1U;

constexpr std::uint64_t default_memory_count = 16U;
constexpr std::uint64_t maximum_memory_count = 1024U;
constexpr std::uint32_t bytes_per_row = 16U;

constexpr std::string_view help_text =
    "Monitor commands:\n"
    "  help                 Show this help.\n"
    "  info                 Board name, run state, PC, simulated time, and clock.\n"
    "  regs                 Dump core and system registers.\n"
    "  reg NAME             Read one register (r0-r15, sp, lr, pc, xpsr, msp, psp,\n"
    "                       control, primask, basepri, faultmask, fpscr, s0-s31, d0-d15).\n"
    "  mdw ADDR [COUNT]     Read COUNT 32-bit words (default 16, max 1024).\n"
    "  mdh ADDR [COUNT]     Read COUNT 16-bit halfwords (default 16, max 1024).\n"
    "  mdb ADDR [COUNT]     Read COUNT bytes (default 16, max 1024).\n"
    "  reset                Reset the board; refused while the target is running.\n"
    "  quit | exit          Close this connection.\n"
    "Addresses accept 0x-prefixed hex. While the target is running, only\n"
    "RAM/ROM/flash reads are allowed: inspecting them never changes state.\n";

/** @brief Splits one command line into whitespace-separated tokens. */
std::vector<std::string_view> tokenize(const std::string_view line) {
    std::vector<std::string_view> tokens;
    std::size_t index = 0U;
    while (index < line.size()) {
        while (index < line.size()
            && std::isspace(static_cast<unsigned char>(line[index])) != 0) {
            ++index;
        }
        const std::size_t start = index;
        while (index < line.size()
            && std::isspace(static_cast<unsigned char>(line[index])) == 0) {
            ++index;
        }
        if (index > start) tokens.push_back(line.substr(start, index - start));
    }
    return tokens;
}

/** @brief Renders one fixed-width `name = 0x........` register cell. */
std::string registerCell(const std::string_view name, const std::uint32_t value) {
    std::string cell(name);
    cell.resize(std::max<std::size_t>(cell.size(), 9U), ' ');
    cell += "= ";
    cell += hex32(value);
    return cell;
}

/** @brief Renders an unknown-register diagnostic. */
std::string unknownRegister(const std::string_view name) {
    return "error: unknown register '" + std::string(name) + "'\n";
}

std::string infoText(sim::Board& board, const bool running) {
    std::string text;
    text += "board = " + board.config().name + "\n";
    text += std::string("state = ") + (running ? "running" : "stopped") + "\n";
    text += "pc    = " + hex32(board.cpu().state().r[15] & ~std::uint32_t{1U}) + "\n";
    text += "time  = " + std::to_string(board.eventLoop().now()) + " ns\n";
    text += "clock = " + std::to_string(board.peripherals().rcc().systemClockHz()) + " Hz\n";
    return text;
}

std::string registerText(const sim::Board& board) {
    const cpu::CpuState& state = board.cpu().state();
    const std::array<std::uint32_t, 24> values{
        state.r[0], state.r[1], state.r[2], state.r[3],
        state.r[4], state.r[5], state.r[6], state.r[7],
        state.r[8], state.r[9], state.r[10], state.r[11],
        state.r[12], state.readRegister(13U), state.r[14],
        state.r[15] & ~std::uint32_t{1U}, state.xpsr, state.msp, state.psp,
        state.control, state.primask, state.basepri, state.faultmask, state.fpscr,
    };
    constexpr std::array<std::string_view, 24> names{
        "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11",
        "r12", "sp", "lr", "pc", "xpsr", "msp", "psp", "control", "primask",
        "basepri", "faultmask", "fpscr",
    };
    constexpr std::size_t cells_per_row = 4U;
    std::string text;
    for (std::size_t index = 0U; index < values.size(); ++index) {
        if (index % cells_per_row != 0U) text += "  ";
        text += registerCell(names[index], values[index]);
        if (index % cells_per_row == cells_per_row - 1U) text += "\n";
    }
    return text;
}

/** @brief Resolves one named register for the `reg` command. */
std::optional<std::uint64_t> registerByName(
    const cpu::CpuState& state,
    const std::string_view name
) {
    const auto decimalIndex = [&name](const std::size_t prefix, const std::uint32_t limit)
        -> std::optional<std::uint32_t> {
        if (name.size() <= prefix) return std::nullopt;
        std::uint32_t value = 0U;
        const auto conversion = std::from_chars(
            name.data() + static_cast<std::ptrdiff_t>(prefix),
            name.data() + static_cast<std::ptrdiff_t>(name.size()), value
        );
        if (conversion.ec != std::errc{} || conversion.ptr != name.data() + name.size()) {
            return std::nullopt;
        }
        if (value >= limit) return std::nullopt;
        return value;
    };
    if (name.size() >= 2U && name[0] == 'r') {
        const auto index = decimalIndex(1U, 16U);
        if (!index) return std::nullopt;
        if (*index < 13U) return state.r[*index];
        return state.readRegister(static_cast<std::uint8_t>(*index));
    }
    if (name == "sp") return state.readRegister(13U);
    if (name == "lr") return state.r[14];
    if (name == "pc") return state.r[15] & ~std::uint32_t{1U};
    if (name == "xpsr") return state.xpsr;
    if (name == "msp") return state.msp;
    if (name == "psp") return state.psp;
    if (name == "control") return state.control;
    if (name == "primask") return state.primask;
    if (name == "basepri") return state.basepri;
    if (name == "faultmask") return state.faultmask;
    if (name == "fpscr") return state.fpscr;
    if (name.size() >= 2U && name[0] == 's') {
        const auto index = decimalIndex(1U, 32U);
        if (!index) return std::nullopt;
        return std::bit_cast<std::uint32_t>(state.s[*index]);
    }
    if (name.size() >= 2U && name[0] == 'd') {
        const auto index = decimalIndex(1U, 16U);
        if (!index) return std::nullopt;
        const std::uint32_t low = std::bit_cast<std::uint32_t>(state.s[*index * 2U]);
        const std::uint32_t high = std::bit_cast<std::uint32_t>(state.s[*index * 2U + 1U]);
        return static_cast<std::uint64_t>(low) | (static_cast<std::uint64_t>(high) << 32U);
    }
    return std::nullopt;
}

/** @brief Reads one memory item, refusing side-effecting MMIO while running. */
bool readItem(
    sim::Board& board,
    const std::uint32_t address,
    const std::uint32_t width,
    const bool running,
    std::uint32_t& value,
    std::string& error
) {
    if (running && !board.memory().isBackedRange(address, width, false)) {
        error = "error: target is running; only RAM/ROM/flash reads are allowed ("
            + hex32(address) + ")\n";
        return false;
    }
    const mem::AccessContext context{mem::AccessType::debug, 0U};
    if (width == 4U) {
        const auto result = board.memory().read32(address, context);
        if (!result) {
            error = "error: read failed at " + hex32(address) + ": " + result.fault().message + "\n";
            return false;
        }
        value = result.value();
        return true;
    }
    if (width == 2U) {
        const auto result = board.memory().read16(address, context);
        if (!result) {
            error = "error: read failed at " + hex32(address) + ": " + result.fault().message + "\n";
            return false;
        }
        value = result.value();
        return true;
    }
    const auto result = board.memory().read8(address, context);
    if (!result) {
        error = "error: read failed at " + hex32(address) + ": " + result.fault().message + "\n";
        return false;
    }
    value = result.value();
    return true;
}

std::string memoryText(
    sim::Board& board,
    const std::string_view command,
    const std::span<const std::string_view> tokens,
    const bool running
) {
    const std::uint32_t width = command == "mdw" ? 4U : command == "mdh" ? 2U : 1U;
    if (tokens.size() < 2U) {
        return "error: " + std::string(command) + " requires an address\n";
    }
    const auto parsed_address = config::parseUnsigned(tokens[1]);
    if (!parsed_address || parsed_address.value() > std::numeric_limits<std::uint32_t>::max()) {
        return "error: invalid address '" + std::string(tokens[1]) + "'\n";
    }
    std::uint64_t count = default_memory_count;
    if (tokens.size() >= 3U) {
        const auto parsed_count = config::parseUnsigned(tokens[2]);
        if (!parsed_count || parsed_count.value() == 0U
            || parsed_count.value() > maximum_memory_count) {
            return "error: count must be in [1, " + std::to_string(maximum_memory_count) + "]\n";
        }
        count = parsed_count.value();
    }
    const auto address = static_cast<std::uint32_t>(parsed_address.value());
    const std::uint64_t total_bytes = count * width;
    if (total_bytes > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())
            - address + 1U) {
        return "error: address range overflows the 32-bit target address space\n";
    }

    const std::uint32_t items_per_row = bytes_per_row / width;
    std::string text;
    std::uint64_t index = 0U;
    while (index < count) {
        const auto row_address = static_cast<std::uint32_t>(address + index * width);
        const auto items = static_cast<std::uint32_t>(
            std::min<std::uint64_t>(count - index, items_per_row)
        );
        std::string hex_part;
        std::string ascii;
        for (std::uint32_t item = 0U; item < items; ++item) {
            const auto item_address = static_cast<std::uint32_t>(row_address + item * width);
            std::uint32_t value = 0U;
            std::string error;
            if (!readItem(board, item_address, width, running, value, error)) {
                text += error;
                return text;
            }
            hex_part += " " + hexValue(value, width * 2U).substr(2U);
            for (std::uint32_t byte = 0U; byte < width; ++byte) {
                const auto character = static_cast<std::uint8_t>(
                    (value >> (byte * 8U)) & 0xffU
                );
                ascii.push_back(character >= 0x20U && character < 0x7fU
                    ? static_cast<char>(character) : '.');
            }
        }
        for (std::uint32_t item = items; item < items_per_row; ++item) {
            hex_part += std::string(width * 2U + 1U, ' ');
        }
        while (ascii.size() < bytes_per_row) ascii.push_back(' ');
        text += hex32(row_address) + ":" + hex_part + "  " + ascii + "\n";
        index += items;
    }
    return text;
}

std::string toCrlf(const std::string_view text) {
    std::string converted;
    converted.reserve(text.size() + text.size() / 16U + 2U);
    for (const char character : text) {
        if (character == '\n') converted += "\r\n";
        else converted.push_back(character);
    }
    return converted;
}

/** @brief Telnet monitor endpoint over one non-blocking listener. */
class TcpMonitorEndpoint final : public MonitorEndpoint {
public:
    TcpMonitorEndpoint(sim::Board& board, detail::TcpListener listener)
        : board_(board), listener_(std::move(listener)) {}

    void service(const bool running) override {
        acceptClients();
        for (Client& client : clients_) {
            if (client.closed) continue;
            readClient(client, running);
        }
        for (Client& client : clients_) {
            if (!client.closed) flushClient(client);
        }
        for (std::size_t index = 0U; index < clients_.size();) {
            if (!clients_[index].closed) {
                ++index;
                continue;
            }
            if (clients_[index].descriptor >= 0) {
                static_cast<void>(::close(clients_[index].descriptor));
            }
            clients_.erase(clients_.begin() + static_cast<std::ptrdiff_t>(index));
        }
    }

    [[nodiscard]] bool active() const noexcept override {
        return listener_.valid() || !clients_.empty();
    }

private:
    struct Client {
        int descriptor{-1};
        std::string line;
        std::string outbox;
        std::uint8_t iac_state{0U};
        std::uint8_t iac_command{0U};
        bool echo{false};
        bool skip_line_feed{false};
        bool closed{false};
    };

    void acceptClients() {
        if (!listener_.valid()) return;
        for (;;) {
            const int descriptor = ::accept(listener_.descriptor(), nullptr, nullptr);
            if (descriptor < 0) return;
            if (!detail::setNonBlocking(descriptor)) {
                static_cast<void>(::close(descriptor));
                continue;
            }
            detail::setNoSigPipe(descriptor);
            Client client;
            client.descriptor = descriptor;
            queueText(client, "fil monitor\r\n> ");
            clients_.push_back(std::move(client));
        }
    }

    void readClient(Client& client, const bool running) {
        std::array<char, 512U> buffer{};
        for (;;) {
            const auto received = ::recv(
                client.descriptor, buffer.data(), buffer.size(), 0
            );
            if (received > 0) {
                processBytes(
                    client,
                    std::string_view(buffer.data(), static_cast<std::size_t>(received)),
                    running
                );
                if (client.closed) return;
                continue;
            }
            if (received == 0) {
                client.closed = true;
                return;
            }
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return;
            client.closed = true;
            return;
        }
    }

    void processBytes(Client& client, const std::string_view bytes, const bool running) {
        for (std::size_t index = 0U; index < bytes.size() && !client.closed; ++index) {
            const auto byte = static_cast<std::uint8_t>(bytes[index]);
            if (client.iac_state == 0U) {
                if (byte == telnet_iac) {
                    client.iac_state = 1U;
                    continue;
                }
                handleDataByte(client, byte, running);
                continue;
            }
            if (client.iac_state == 1U) {
                if (byte == telnet_iac) {
                    client.iac_state = 0U;
                    continue;
                }
                if (byte == telnet_will || byte == telnet_wont
                    || byte == telnet_do || byte == telnet_dont) {
                    client.iac_command = byte;
                    client.iac_state = 2U;
                    continue;
                }
                client.iac_state = 0U;
                continue;
            }
            client.iac_state = 0U;
            negotiate(client, client.iac_command, byte);
        }
    }

    void negotiate(
        Client& client,
        const std::uint8_t command,
        const std::uint8_t option
    ) {
        if (option == telnet_echo && command == telnet_do) {
            client.echo = true;
            queueNegotiation(client, telnet_will, telnet_echo);
            return;
        }
        if (option == telnet_echo && command == telnet_dont) {
            client.echo = false;
            queueNegotiation(client, telnet_wont, telnet_echo);
            return;
        }
        if (command == telnet_do) {
            queueNegotiation(client, telnet_wont, option);
            return;
        }
        if (command == telnet_will || command == telnet_wont) {
            queueNegotiation(client, telnet_dont, option);
        }
    }

    void handleDataByte(Client& client, const std::uint8_t byte, const bool running) {
        if (byte == static_cast<std::uint8_t>('\r')) {
            executeLine(client, running);
            client.skip_line_feed = true;
            return;
        }
        if (byte == static_cast<std::uint8_t>('\n')) {
            if (client.skip_line_feed) {
                client.skip_line_feed = false;
                return;
            }
            executeLine(client, running);
            return;
        }
        client.skip_line_feed = false;
        if (byte == 0x08U || byte == 0x7fU) {
            if (!client.line.empty()) {
                client.line.pop_back();
                if (client.echo) queueText(client, "\b \b");
            }
            return;
        }
        if (byte == 0x03U) {
            client.line.clear();
            if (client.echo) queueText(client, "^C\r\n");
            return;
        }
        if (byte >= 0x20U && byte < 0x7fU) {
            if (client.line.size() >= 512U) client.line.clear();
            client.line.push_back(static_cast<char>(byte));
            if (client.echo) {
                const char character = static_cast<char>(byte);
                queueText(client, std::string_view(&character, 1U));
            }
        }
    }

    void executeLine(Client& client, const bool running) {
        const std::string line = client.line;
        client.line.clear();
        if (client.echo) queueText(client, "\r\n");
        const MonitorReply reply = executeMonitorCommand(board_, line, running);
        if (!reply.output.empty()) queueText(client, toCrlf(reply.output));
        if (reply.close) {
            client.closed = true;
            return;
        }
        queueText(client, "> ");
    }

    void queueText(Client& client, const std::string_view text) {
        client.outbox.append(text);
    }

    void queueNegotiation(
        Client& client,
        const std::uint8_t command,
        const std::uint8_t option
    ) {
        const char bytes[]{
            static_cast<char>(telnet_iac),
            static_cast<char>(command),
            static_cast<char>(option),
        };
        queueText(client, std::string_view(bytes, sizeof(bytes)));
    }

    void flushClient(Client& client) {
        while (!client.outbox.empty()) {
            const auto sent = ::send(
                client.descriptor, client.outbox.data(), client.outbox.size(),
                detail::socketSendFlags()
            );
            if (sent > 0) {
                client.outbox.erase(0U, static_cast<std::size_t>(sent));
                continue;
            }
            if (sent < 0 && errno == EINTR) continue;
            if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
            client.closed = true;
            return;
        }
    }

    sim::Board& board_;
    detail::TcpListener listener_;
    std::vector<Client> clients_;
};

} // namespace

MonitorReply executeMonitorCommand(
    sim::Board& board,
    const std::string_view line,
    const bool running
) {
    const std::vector<std::string_view> tokens = tokenize(line);
    if (tokens.empty()) return {};
    const std::string_view command = tokens[0U];
    if (command == "help" || command == "?") return {std::string(help_text)};
    if (command == "quit" || command == "exit") return {"", true};
    if (command == "info") return {infoText(board, running)};
    if (command == "regs") return {registerText(board)};
    if (command == "reg") {
        if (tokens.size() < 2U) return {"error: reg requires a register name\n"};
        const auto value = registerByName(board.cpu().state(), tokens[1]);
        if (!value) return {unknownRegister(tokens[1])};
        const bool double_register = tokens[1].size() >= 2U && tokens[1].front() == 'd';
        if (double_register || value.value() > std::numeric_limits<std::uint32_t>::max()) {
            return {std::string(tokens[1]) + " = " + hexValue(value.value(), 16U) + "\n"};
        }
        std::string text = std::string(tokens[1]) + " = "
            + hex32(static_cast<std::uint32_t>(value.value()));
        if (tokens[1].size() >= 2U && tokens[1].front() == 's') {
            std::ostringstream stream;
            stream << std::bit_cast<float>(static_cast<std::uint32_t>(value.value()));
            text += " (" + stream.str() + ")";
        }
        return {text + "\n"};
    }
    if (command == "mdw" || command == "mdh" || command == "mdb") {
        return {memoryText(board, command, tokens, running)};
    }
    if (command == "reset") {
        if (running) return {"error: target is running; reset requires a stopped target\n"};
        const auto result = board.reset();
        if (!result) {
            return {"error: reset failed: " + formatError(result.error()) + "\n"};
        }
        return {"reset complete\n"};
    }
    return {"error: unknown command '" + std::string(command) + "'; type 'help'\n"};
}

Result<std::unique_ptr<MonitorEndpoint>> createTcpMonitorEndpoint(
    sim::Board& board,
    const std::string& bind_address,
    const std::uint16_t port,
    std::uint16_t& bound_port
) {
    auto listener = detail::openTcpListener(bind_address, port, bound_port);
    if (!listener) return listener.error();
    return std::unique_ptr<MonitorEndpoint>(
        std::make_unique<TcpMonitorEndpoint>(board, std::move(listener.value()))
    );
}

} // namespace fil::debug
