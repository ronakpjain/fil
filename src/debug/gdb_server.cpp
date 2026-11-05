#include "fil/debug/gdb_server.hpp"

#include "fil/cpu/cortex_m4.hpp"
#include "fil/debug/monitor.hpp"
#include "tcp_listener.hpp"
#include "fil/mem/memory_bus.hpp"
#include "fil/sim/board.hpp"

#include <algorithm>
#include <bit>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <poll.h>
#include <span>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace fil::debug {
namespace {

constexpr std::uint8_t signal_interrupt = 2U;
constexpr std::uint8_t signal_illegal_instruction = 4U;
constexpr std::uint8_t signal_trap = 5U;
constexpr std::uint8_t signal_abort = 6U;
constexpr std::uint8_t signal_segmentation = 11U;

constexpr std::size_t maximum_packet_bytes = 64U * 1024U;
// GDB's binary-download path (used by `load`) writes roughly 16 KiB per
// packet even when a smaller PacketSize was advertised, so accept generously
// more than the advertised packet size.
constexpr std::size_t maximum_memory_request = 32U * 1024U;

// Register order shared by target.xml, `g`, `p`, and `P`.
constexpr std::size_t sp_register_index = 13U;
constexpr std::size_t lr_register_index = 14U;
constexpr std::size_t pc_register_index = 15U;
constexpr std::size_t xpsr_register_index = 16U;
constexpr std::size_t msp_register_index = 17U;
constexpr std::size_t psp_register_index = 18U;
constexpr std::size_t control_register_index = 19U;
constexpr std::size_t primask_register_index = 20U;
constexpr std::size_t basepri_register_index = 21U;
constexpr std::size_t faultmask_register_index = 22U;
constexpr std::size_t double_register_base = 23U;
constexpr std::size_t double_register_count = 16U;
constexpr std::size_t fpscr_register_index = 39U;
constexpr std::size_t register_count = 40U;
constexpr std::size_t double_register_nibbles = 16U;
constexpr std::size_t single_register_nibbles = 8U;

/** @brief GDB target description: core integer, system, and FPv4-SP registers. */
constexpr std::string_view target_description = R"XML(<?xml version="1.0"?>
<!DOCTYPE target SYSTEM "gdb-target.dtd">
<target>
  <architecture>arm</architecture>
  <feature name="org.gnu.gdb.arm.m-profile">
    <reg name="r0" bitsize="32" type="uint32"/>
    <reg name="r1" bitsize="32" type="uint32"/>
    <reg name="r2" bitsize="32" type="uint32"/>
    <reg name="r3" bitsize="32" type="uint32"/>
    <reg name="r4" bitsize="32" type="uint32"/>
    <reg name="r5" bitsize="32" type="uint32"/>
    <reg name="r6" bitsize="32" type="uint32"/>
    <reg name="r7" bitsize="32" type="uint32"/>
    <reg name="r8" bitsize="32" type="uint32"/>
    <reg name="r9" bitsize="32" type="uint32"/>
    <reg name="r10" bitsize="32" type="uint32"/>
    <reg name="r11" bitsize="32" type="uint32"/>
    <reg name="r12" bitsize="32" type="uint32"/>
    <reg name="sp" bitsize="32" type="data_ptr"/>
    <reg name="lr" bitsize="32" type="uint32"/>
    <reg name="pc" bitsize="32" type="code_ptr"/>
    <reg name="xpsr" bitsize="32" type="uint32"/>
    <reg name="msp" bitsize="32" type="data_ptr"/>
    <reg name="psp" bitsize="32" type="data_ptr"/>
    <reg name="control" bitsize="32" type="uint32"/>
    <reg name="primask" bitsize="32" type="uint32"/>
    <reg name="basepri" bitsize="32" type="uint32"/>
    <reg name="faultmask" bitsize="32" type="uint32"/>
  </feature>
  <feature name="org.gnu.gdb.arm.vfp">
    <reg name="d0" bitsize="64" type="ieee_double"/>
    <reg name="d1" bitsize="64" type="ieee_double"/>
    <reg name="d2" bitsize="64" type="ieee_double"/>
    <reg name="d3" bitsize="64" type="ieee_double"/>
    <reg name="d4" bitsize="64" type="ieee_double"/>
    <reg name="d5" bitsize="64" type="ieee_double"/>
    <reg name="d6" bitsize="64" type="ieee_double"/>
    <reg name="d7" bitsize="64" type="ieee_double"/>
    <reg name="d8" bitsize="64" type="ieee_double"/>
    <reg name="d9" bitsize="64" type="ieee_double"/>
    <reg name="d10" bitsize="64" type="ieee_double"/>
    <reg name="d11" bitsize="64" type="ieee_double"/>
    <reg name="d12" bitsize="64" type="ieee_double"/>
    <reg name="d13" bitsize="64" type="ieee_double"/>
    <reg name="d14" bitsize="64" type="ieee_double"/>
    <reg name="d15" bitsize="64" type="ieee_double"/>
    <reg name="fpscr" bitsize="32" type="int"/>
  </feature>
</target>
)XML";

/** @brief Value of one hexadecimal digit, or -1 when the character is not one. */
int hexDigitValue(const char character) noexcept {
    if (character >= '0' && character <= '9') return character - '0';
    if (character >= 'a' && character <= 'f') return character - 'a' + 10;
    if (character >= 'A' && character <= 'F') return character - 'A' + 10;
    return -1;
}

/** @brief Renders a little-endian value as `bytes` hexadecimal digits. */
std::string hexValue(const std::uint64_t value, const std::size_t bytes) {
    std::string text(bytes * 2U, '0');
    for (std::size_t index = 0U; index < bytes; ++index) {
        const auto digit = static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU);
        text[index * 2U] = "0123456789abcdef"[digit >> 4U];
        text[index * 2U + 1U] = "0123456789abcdef"[digit & 0x0fU];
    }
    return text;
}

/** @brief Parses up to sixteen hexadecimal digits without sign or prefixes. */
std::optional<std::uint64_t> parseHex(const std::string_view text) {
    if (text.empty() || text.size() > 16U) return std::nullopt;
    std::uint64_t value = 0U;
    for (const char character : text) {
        const int digit = hexDigitValue(character);
        if (digit < 0) return std::nullopt;
        value = (value << 4U) | static_cast<std::uint64_t>(digit);
    }
    return value;
}

/**
 * @brief Parses a little-endian hexadecimal byte string into one value.
 *
 * RSP `g`, `G`, `p`, and `P` payloads carry register bytes in target order,
 * so `deadbeef` is the value 0xefbeadde.
 */
std::optional<std::uint64_t> parseLittleEndianHex(const std::string_view text) {
    if (text.empty() || text.size() % 2U != 0U || text.size() > 16U) return std::nullopt;
    std::uint64_t value = 0U;
    for (std::size_t position = 0U; position < text.size(); position += 2U) {
        const int high = hexDigitValue(text[position]);
        const int low = hexDigitValue(text[position + 1U]);
        if (high < 0 || low < 0) return std::nullopt;
        value |= static_cast<std::uint64_t>((high << 4) | low) << (position * 4U);
    }
    return value;
}

/** @brief Parses a hexadecimal value that must fit the 32-bit target address space. */
std::optional<std::uint32_t> parseHex32(const std::string_view text) {
    const auto value = parseHex(text);
    if (!value || *value > std::numeric_limits<std::uint32_t>::max()) return std::nullopt;
    return static_cast<std::uint32_t>(*value);
}

/** @brief Decodes an even-length hexadecimal byte string into raw bytes. */
std::optional<std::vector<std::uint8_t>> decodeHexBytes(const std::string_view text) {
    if (text.size() % 2U != 0U) return std::nullopt;
    std::vector<std::uint8_t> bytes;
    bytes.reserve(text.size() / 2U);
    for (std::size_t index = 0U; index < text.size(); index += 2U) {
        const int high = hexDigitValue(text[index]);
        const int low = hexDigitValue(text[index + 1U]);
        if (high < 0 || low < 0) return std::nullopt;
        bytes.push_back(static_cast<std::uint8_t>((high << 4) | low));
    }
    return bytes;
}

/** @brief Decodes the `X` packet's `}`-escaped binary payload. */
std::optional<std::vector<std::uint8_t>> decodeBinaryBytes(const std::string_view text) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(text.size());
    for (std::size_t index = 0U; index < text.size(); ++index) {
        auto byte = static_cast<std::uint8_t>(text[index]);
        if (byte == static_cast<std::uint8_t>('}')) {
            if (++index >= text.size()) return std::nullopt;
            byte = static_cast<std::uint8_t>(
                static_cast<std::uint8_t>(text[index]) ^ 0x20U
            );
        }
        bytes.push_back(byte);
    }
    return bytes;
}

/** @brief Blocking TCP byte transport owned by one accepted connection. */
class SocketTransport final : public RspTransport {
public:
    explicit SocketTransport(const int descriptor) : descriptor_(descriptor) {
        detail::setNoSigPipe(descriptor);
    }

    ~SocketTransport() override {
        if (descriptor_ >= 0) static_cast<void>(::close(descriptor_));
    }

    SocketTransport(const SocketTransport&) = delete;
    SocketTransport& operator=(const SocketTransport&) = delete;

    [[nodiscard]] std::optional<std::uint8_t> readByte() override {
        std::uint8_t byte = 0U;
        for (;;) {
            const auto received = ::recv(descriptor_, &byte, 1U, 0);
            if (received == 1) return byte;
            if (received == 0) return std::nullopt;
            if (errno == EINTR) continue;
            // A non-blocking descriptor (Darwin inherits the listener's mode)
            // is polled instead of being reported as end of input.
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                pollfd pending{descriptor_, POLLIN, 0};
                if (::poll(&pending, 1, 1000) > 0) continue;
                return std::nullopt;
            }
            return std::nullopt;
        }
    }

    [[nodiscard]] bool inputPending() override {
        pollfd pending{descriptor_, POLLIN, 0};
        const int ready = ::poll(&pending, 1, 0);
        return ready > 0
            && (pending.revents & (POLLIN | POLLHUP | POLLERR | POLLNVAL)) != 0;
    }

    [[nodiscard]] TransportWait waitForInput(
        const std::chrono::milliseconds timeout
    ) override {
        pollfd pending{descriptor_, POLLIN, 0};
        const int ready = ::poll(&pending, 1, static_cast<int>(timeout.count()));
        if (ready == 0) return TransportWait::timeout;
        if (ready < 0) {
            return errno == EINTR ? TransportWait::timeout : TransportWait::closed;
        }
        if ((pending.revents & POLLIN) != 0) return TransportWait::ready;
        return TransportWait::closed;
    }

    [[nodiscard]] bool write(const std::string_view bytes) override {
        std::size_t written = 0U;
        while (written < bytes.size()) {
            const auto sent = ::send(
                descriptor_, bytes.data() + written, bytes.size() - written,
                detail::socketSendFlags()
            );
            if (sent > 0) {
                written += static_cast<std::size_t>(sent);
                continue;
            }
            if (sent < 0 && errno == EINTR) continue;
            return false;
        }
        return true;
    }

private:
    int descriptor_{-1};
};

} // namespace

std::string_view gdbSessionEndName(const GdbSessionEnd end) noexcept {
    switch (end) {
    case GdbSessionEnd::detached: return "detached";
    case GdbSessionEnd::killed: return "killed";
    case GdbSessionEnd::disconnected: return "disconnected";
    }
    return "disconnected";
}

struct GdbSession::Implementation {
    Implementation(
        sim::Board& board_value,
        RspTransport& transport_value,
        GdbServerOptions options_value,
        MonitorEndpoint* monitor_value
    ) : board(board_value), transport(transport_value),
        options(std::move(options_value)), monitor(monitor_value) {}

    sim::Board& board;
    RspTransport& transport;
    GdbServerOptions options;
    MonitorEndpoint* monitor{nullptr};
    std::vector<std::uint32_t> breakpoints;
    std::vector<std::uint32_t> hardware_breakpoints;
    std::vector<std::uint32_t> combined_breakpoints;
    std::string last_stop{"S05"};
    std::string last_frame;
    std::optional<GdbSessionEnd> end;
    bool have_last_frame{false};
    bool no_ack{false};

    [[nodiscard]] std::optional<std::string> readPacket();
    [[nodiscard]] bool sendPacket(std::string_view payload);
    [[nodiscard]] bool sendStopReply(std::uint8_t signal, std::string_view reason = {});
    [[nodiscard]] bool sendMonitor(std::string_view text);
    [[nodiscard]] bool replyError() { return sendPacket("E01"); }
    [[nodiscard]] bool replyUnsupported() { return sendPacket(""); }

    [[nodiscard]] bool dispatch(std::string_view packet);
    [[nodiscard]] bool handleQuery(std::string_view packet);
    [[nodiscard]] bool handleSet(std::string_view packet);
    [[nodiscard]] bool handleV(std::string_view packet);
    [[nodiscard]] bool handleVCont(std::string_view packet);
    [[nodiscard]] bool handleMonitor(std::string_view hex);
    [[nodiscard]] bool handleBreakpoint(std::string_view packet, bool insert);
    [[nodiscard]] bool handleMemoryRead(std::string_view packet);
    [[nodiscard]] bool handleMemoryWrite(std::string_view packet, bool binary);
    [[nodiscard]] bool handleRegisterRead(std::string_view text);
    [[nodiscard]] bool handleRegisterWrite(std::string_view text);
    [[nodiscard]] bool resume(std::string_view packet);
    [[nodiscard]] bool resumeTarget(bool step);

    [[nodiscard]] std::string readAllRegisters() const;
    [[nodiscard]] std::optional<std::string> readSingleRegister(std::uint32_t index) const;
    [[nodiscard]] bool writeAllRegisters(std::string_view text);
    [[nodiscard]] bool writeSingleRegister(std::uint32_t index, std::uint64_t value);
    void refreshActiveSpMirror();
    void refreshCombinedBreakpoints();
};

std::optional<std::string> GdbSession::Implementation::readPacket() {
    for (;;) {
        const auto byte = transport.readByte();
        if (!byte) return std::nullopt;
        if (*byte == static_cast<std::uint8_t>('+')) continue;
        if (*byte == static_cast<std::uint8_t>('-')) {
            // The peer rejected the previous frame; resend it once.
            if (have_last_frame && !transport.write(last_frame)) return std::nullopt;
            continue;
        }
        if (*byte != static_cast<std::uint8_t>('$')) continue; // Stray interrupt or noise.
        std::string payload;
        for (;;) {
            const auto next = transport.readByte();
            if (!next) return std::nullopt;
            if (*next == static_cast<std::uint8_t>('#')) break;
            if (payload.size() >= maximum_packet_bytes) return std::nullopt;
            payload.push_back(static_cast<char>(*next));
        }
        const auto high = transport.readByte();
        const auto low = transport.readByte();
        if (!high || !low) return std::nullopt;
        std::uint8_t checksum = 0U;
        for (const char character : payload) {
            checksum = static_cast<std::uint8_t>(
                checksum + static_cast<std::uint8_t>(character)
            );
        }
        const int high_digit = hexDigitValue(static_cast<char>(*high));
        const int low_digit = hexDigitValue(static_cast<char>(*low));
        const auto expected = static_cast<std::uint8_t>(
            ((high_digit < 0 ? 0 : high_digit) << 4) | (low_digit < 0 ? 0 : low_digit)
        );
        if (checksum != expected) {
            if (!no_ack && !transport.write("-")) return std::nullopt;
            continue;
        }
        if (!no_ack && !transport.write("+")) return std::nullopt;
        return payload;
    }
}

bool GdbSession::Implementation::sendPacket(const std::string_view payload) {
    std::string frame;
    frame.reserve(payload.size() + 4U);
    frame.push_back('$');
    frame.append(payload);
    frame.push_back('#');
    std::uint8_t checksum = 0U;
    for (const char character : payload) {
        checksum = static_cast<std::uint8_t>(
            checksum + static_cast<std::uint8_t>(character)
        );
    }
    frame += hexValue(checksum, 1U);
    last_frame = frame;
    have_last_frame = true;
    if (!transport.write(frame)) return false;
    if (no_ack) return true;
    for (;;) {
        const auto byte = transport.readByte();
        if (!byte) return false;
        if (*byte == static_cast<std::uint8_t>('+')) return true;
        if (*byte == static_cast<std::uint8_t>('-')) {
            if (!transport.write(frame)) return false;
        }
    }
}

bool GdbSession::Implementation::sendStopReply(
    const std::uint8_t signal,
    const std::string_view reason
) {
    std::string payload = "T";
    payload += hexValue(signal, 1U);
    if (!reason.empty()) {
        payload.append(reason);
        payload += ":;";
    }
    last_stop = payload;
    return sendPacket(payload);
}

bool GdbSession::Implementation::sendMonitor(const std::string_view text) {
    std::string payload = "O";
    for (const char character : text) {
        payload += hexValue(static_cast<std::uint8_t>(character), 1U);
    }
    return sendPacket(payload);
}

void GdbSession::Implementation::refreshActiveSpMirror() {
    cpu::CpuState& state = board.cpu().state();
    state.r[13] = state.activeSp();
}

void GdbSession::Implementation::refreshCombinedBreakpoints() {
    combined_breakpoints.clear();
    combined_breakpoints.reserve(breakpoints.size() + hardware_breakpoints.size());
    combined_breakpoints.insert(
        combined_breakpoints.end(), breakpoints.begin(), breakpoints.end()
    );
    combined_breakpoints.insert(
        combined_breakpoints.end(), hardware_breakpoints.begin(), hardware_breakpoints.end()
    );
    std::sort(combined_breakpoints.begin(), combined_breakpoints.end());
    combined_breakpoints.erase(
        std::unique(combined_breakpoints.begin(), combined_breakpoints.end()),
        combined_breakpoints.end()
    );
}

std::string GdbSession::Implementation::readAllRegisters() const {
    const cpu::CpuState& state = board.cpu().state();
    std::string text;
    text.reserve(register_count * single_register_nibbles);
    for (std::size_t index = 0U; index < sp_register_index; ++index) {
        text += hexValue(state.r[index], 4U);
    }
    text += hexValue(state.readRegister(static_cast<std::uint8_t>(sp_register_index)), 4U);
    text += hexValue(state.r[lr_register_index], 4U);
    text += hexValue(state.r[pc_register_index] & ~std::uint32_t{1U}, 4U);
    text += hexValue(state.xpsr, 4U);
    text += hexValue(state.msp, 4U);
    text += hexValue(state.psp, 4U);
    text += hexValue(state.control, 4U);
    text += hexValue(state.primask, 4U);
    text += hexValue(state.basepri, 4U);
    text += hexValue(state.faultmask, 4U);
    for (std::size_t index = 0U; index < double_register_count; ++index) {
        const std::uint32_t low = std::bit_cast<std::uint32_t>(state.s[index * 2U]);
        const std::uint32_t high = std::bit_cast<std::uint32_t>(state.s[index * 2U + 1U]);
        text += hexValue(
            static_cast<std::uint64_t>(low) | (static_cast<std::uint64_t>(high) << 32U), 8U
        );
    }
    text += hexValue(state.fpscr, 4U);
    return text;
}

std::optional<std::string> GdbSession::Implementation::readSingleRegister(
    const std::uint32_t index
) const {
    const cpu::CpuState& state = board.cpu().state();
    if (index < sp_register_index) return hexValue(state.r[index], 4U);
    switch (index) {
    case sp_register_index:
        return hexValue(state.readRegister(static_cast<std::uint8_t>(index)), 4U);
    case lr_register_index: return hexValue(state.r[lr_register_index], 4U);
    case pc_register_index:
        return hexValue(state.r[pc_register_index] & ~std::uint32_t{1U}, 4U);
    case xpsr_register_index: return hexValue(state.xpsr, 4U);
    case msp_register_index: return hexValue(state.msp, 4U);
    case psp_register_index: return hexValue(state.psp, 4U);
    case control_register_index: return hexValue(state.control, 4U);
    case primask_register_index: return hexValue(state.primask, 4U);
    case basepri_register_index: return hexValue(state.basepri, 4U);
    case faultmask_register_index: return hexValue(state.faultmask, 4U);
    case fpscr_register_index: return hexValue(state.fpscr, 4U);
    default: break;
    }
    if (index >= double_register_base && index < double_register_base + double_register_count) {
        const std::size_t pair = index - double_register_base;
        const std::uint32_t low = std::bit_cast<std::uint32_t>(state.s[pair * 2U]);
        const std::uint32_t high = std::bit_cast<std::uint32_t>(state.s[pair * 2U + 1U]);
        return hexValue(
            static_cast<std::uint64_t>(low) | (static_cast<std::uint64_t>(high) << 32U), 8U
        );
    }
    return std::nullopt;
}

bool GdbSession::Implementation::writeSingleRegister(
    const std::uint32_t index,
    const std::uint64_t value
) {
    cpu::CpuState& state = board.cpu().state();
    const auto word = static_cast<std::uint32_t>(value);
    if (index < sp_register_index) {
        state.r[index] = word;
        return true;
    }
    switch (index) {
    case sp_register_index:
        state.writeRegister(static_cast<std::uint8_t>(index), word);
        return true;
    case lr_register_index:
        state.r[lr_register_index] = word;
        return true;
    case pc_register_index:
        state.r[pc_register_index] = word & ~std::uint32_t{1U};
        return true;
    case xpsr_register_index:
        state.xpsr = word;
        state.thumb = (word & cpu::xpsr_t) != 0U;
        state.it_state = static_cast<std::uint8_t>(
            ((word >> 25U) & 0x3U) | (((word >> 10U) & 0x3fU) << 2U)
        );
        refreshActiveSpMirror();
        return true;
    case msp_register_index:
        state.msp = word;
        refreshActiveSpMirror();
        return true;
    case psp_register_index:
        state.psp = word;
        refreshActiveSpMirror();
        return true;
    case control_register_index:
        state.control = word;
        refreshActiveSpMirror();
        return true;
    case primask_register_index:
        state.primask = word;
        return true;
    case basepri_register_index:
        state.basepri = word;
        return true;
    case faultmask_register_index:
        state.faultmask = word;
        return true;
    case fpscr_register_index:
        state.fpscr = word;
        return true;
    default: break;
    }
    if (index >= double_register_base && index < double_register_base + double_register_count) {
        const std::size_t pair = index - double_register_base;
        state.s[pair * 2U] = std::bit_cast<float>(word);
        state.s[pair * 2U + 1U] = std::bit_cast<float>(static_cast<std::uint32_t>(value >> 32U));
        return true;
    }
    return false;
}

bool GdbSession::Implementation::writeAllRegisters(const std::string_view text) {
    std::size_t position = 0U;
    for (std::uint32_t index = 0U; index < register_count; ++index) {
        const bool wide = index >= double_register_base
            && index < double_register_base + double_register_count;
        const std::size_t nibbles = wide ? double_register_nibbles : single_register_nibbles;
        if (position + nibbles > text.size()) return false;
        const auto value = parseLittleEndianHex(text.substr(position, nibbles));
        if (!value || !writeSingleRegister(index, *value)) return false;
        position += nibbles;
    }
    return position == text.size();
}

bool GdbSession::Implementation::handleRegisterRead(const std::string_view text) {
    const auto index = parseHex(text);
    if (!index || *index >= register_count) return replyError();
    const auto value = readSingleRegister(static_cast<std::uint32_t>(*index));
    if (!value) return replyError();
    return sendPacket(*value);
}

bool GdbSession::Implementation::handleRegisterWrite(const std::string_view text) {
    const auto equals = text.find('=');
    if (equals == std::string_view::npos) return replyError();
    const auto index = parseHex(text.substr(0U, equals));
    const auto value = parseLittleEndianHex(text.substr(equals + 1U));
    if (!index || !value || *index >= register_count) return replyError();
    if (!writeSingleRegister(static_cast<std::uint32_t>(*index), *value)) return replyError();
    return sendPacket("OK");
}

bool GdbSession::Implementation::handleMemoryRead(const std::string_view packet) {
    const auto comma = packet.find(',');
    if (comma == std::string_view::npos) return replyError();
    const auto address = parseHex32(packet.substr(1U, comma - 1U));
    const auto length = parseHex(packet.substr(comma + 1U));
    if (!address || !length || *length > maximum_memory_request) return replyError();
    if (*length > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())
            - *address) {
        return replyError();
    }
    std::string text;
    text.reserve(static_cast<std::size_t>(*length) * 2U);
    for (std::uint64_t offset = 0U; offset < *length; ++offset) {
        const auto byte = board.memory().read8(
            static_cast<std::uint32_t>(*address + offset),
            mem::AccessContext{mem::AccessType::debug, 0U}
        );
        if (!byte) return replyError();
        text += hexValue(byte.value(), 1U);
    }
    return sendPacket(text);
}

bool GdbSession::Implementation::handleMemoryWrite(
    const std::string_view packet,
    const bool binary
) {
    const auto comma = packet.find(',');
    if (comma == std::string_view::npos) return replyError();
    const auto colon = packet.find(':', comma + 1U);
    if (colon == std::string_view::npos) return replyError();
    const auto address = parseHex32(packet.substr(1U, comma - 1U));
    const auto length = parseHex(packet.substr(comma + 1U, colon - comma - 1U));
    if (!address || !length || *length > maximum_memory_request) return replyError();
    if (*length > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())
            - *address) {
        return replyError();
    }
    const auto bytes = binary
        ? decodeBinaryBytes(packet.substr(colon + 1U))
        : decodeHexBytes(packet.substr(colon + 1U));
    if (!bytes || bytes->size() != static_cast<std::size_t>(*length)) return replyError();
    for (std::size_t offset = 0U; offset < bytes->size(); ++offset) {
        const auto result = board.memory().write8(
            static_cast<std::uint32_t>(*address + offset),
            (*bytes)[offset],
            mem::AccessContext{mem::AccessType::debug, 0U}
        );
        if (!result) return replyError();
    }
    return sendPacket("OK");
}

bool GdbSession::Implementation::handleBreakpoint(
    const std::string_view packet,
    const bool insert
) {
    if (packet.size() < 2U) return replyError();
    const char type = packet[1U];
    if (type < '0' || type > '4') return replyError();
    if (type >= '2') return replyUnsupported(); // Watchpoints are not modeled yet.
    const auto comma = packet.find(',');
    if (comma == std::string_view::npos) return replyError();
    const auto kind = packet.find(',', comma + 1U);
    const auto address = parseHex32(kind == std::string_view::npos
        ? packet.substr(comma + 1U)
        : packet.substr(comma + 1U, kind - comma - 1U));
    if (!address) return replyError();
    std::vector<std::uint32_t>& list = type == '1' ? hardware_breakpoints : breakpoints;
    const std::uint32_t target = *address & ~std::uint32_t{1U};
    const auto position = std::find(list.begin(), list.end(), target);
    if (insert) {
        if (position == list.end()) {
            list.push_back(target);
            std::sort(list.begin(), list.end());
        }
    } else if (position != list.end()) {
        list.erase(position);
    }
    refreshCombinedBreakpoints();
    return sendPacket("OK");
}

bool GdbSession::Implementation::resume(const std::string_view packet) {
    const char lead = packet.front();
    const bool step = lead == 's' || lead == 'S';
    std::string_view rest = packet.substr(1U);
    if (lead == 'C' || lead == 'S') {
        const auto semicolon = rest.find(';');
        const auto signal = parseHex(rest.substr(0U, semicolon));
        if (!signal) return replyError();
        rest = semicolon == std::string_view::npos ? std::string_view{} : rest.substr(semicolon + 1U);
    }
    if (!rest.empty()) {
        const auto address = parseHex32(rest);
        if (!address) return replyError();
        board.cpu().state().r[pc_register_index] = *address & ~std::uint32_t{1U};
    }
    return resumeTarget(step);
}

bool GdbSession::Implementation::resumeTarget(const bool step) {
    cpu::CpuState& state = board.cpu().state();
    // A resume request clears the modeled halt left by BKPT/faults.
    state.halted = false;

    sim::BoardRunOptions run;
    run.trace_instructions = false;
    run.detect_spin = false;
    run.enable_jit = true;
    run.adc_decimation = 1U;
    if (step) {
        run.max_instructions = 1U;
        run.duration_ns = 0U;
    } else {
        run.max_instructions = options.slice_instructions == 0U
            ? std::numeric_limits<std::uint64_t>::max() : options.slice_instructions;
        run.duration_ns = options.slice_time_ns;
    }
    const std::span<const std::uint32_t> stops = step
        ? std::span<const std::uint32_t>{} : std::span<const std::uint32_t>{combined_breakpoints};

    for (;;) {
        // Service monitor clients at instruction boundaries; reads never halt
        // the running target and side-effecting MMIO stays refused.
        if (monitor != nullptr) monitor->service(true);
        if (!step && transport.inputPending()) {
            return sendStopReply(signal_interrupt);
        }
        run.breakpoints = stops;
        const sim::BoardRunResult result = board.run(run);
        switch (result.reason) {
        case sim::BoardStopReason::target_reached: {
            const std::uint32_t pc = board.cpu().state().r[pc_register_index];
            const bool hardware = std::binary_search(
                hardware_breakpoints.begin(), hardware_breakpoints.end(), pc
            );
            return sendStopReply(signal_trap, hardware ? "hwbreak" : "swbreak");
        }
        case sim::BoardStopReason::instruction_budget:
        case sim::BoardStopReason::time_budget:
            if (step) return sendStopReply(signal_trap);
            break;
        case sim::BoardStopReason::breakpoint:
        case sim::BoardStopReason::halted:
            return sendStopReply(signal_trap);
        case sim::BoardStopReason::unimplemented_instruction:
            return sendStopReply(signal_illegal_instruction);
        case sim::BoardStopReason::architectural_fault:
        case sim::BoardStopReason::synchronization_required:
            return sendStopReply(signal_segmentation);
        case sim::BoardStopReason::spin_detected:
        case sim::BoardStopReason::reset_requested:
        case sim::BoardStopReason::host_error:
            return sendStopReply(signal_abort);
        }
    }
}

bool GdbSession::Implementation::handleVCont(const std::string_view packet) {
    std::string_view rest = packet.substr(std::string_view{"vCont;"}.size());
    if (rest.empty()) return replyError();
    const auto semicolon = rest.find(';');
    std::string_view action = rest.substr(0U, semicolon);
    const auto colon = action.find(':');
    if (colon != std::string_view::npos) action = action.substr(0U, colon);
    if (action.empty()) return replyError();
    const char lead = action.front();
    if (lead != 'c' && lead != 'C' && lead != 's' && lead != 'S') return replyError();
    if ((lead == 'C' || lead == 'S') && action.size() > 1U) {
        const auto signal = parseHex(action.substr(1U));
        if (!signal) return replyError();
    }
    return resumeTarget(lead == 's' || lead == 'S');
}

bool GdbSession::Implementation::handleMonitor(const std::string_view hex) {
    const auto bytes = decodeHexBytes(hex);
    if (!bytes) return replyError();
    // Share the telnet monitor command layer so `monitor reset halt`,
    // `monitor halt`, `monitor mdw ...`, and friends behave identically.
    const MonitorReply reply = executeMonitorCommand(
        board, std::string_view(reinterpret_cast<const char*>(bytes->data()), bytes->size()),
        false
    );
    if (!reply.output.empty() && !sendMonitor(reply.output)) return false;
    return sendPacket("OK");
}

bool GdbSession::Implementation::handleQuery(const std::string_view packet) {
    if (packet == "qSupported" || packet.starts_with("qSupported:")) {
        return sendPacket(
            "PacketSize=4000;qXfer:features:read+;QStartNoAckMode+;"
            "swbreak+;hwbreak+;vContSupported+"
        );
    }
    constexpr std::string_view feature_prefix = "qXfer:features:read:";
    if (packet.starts_with(feature_prefix)) {
        const std::string_view spec = packet.substr(feature_prefix.size());
        const auto colon = spec.rfind(':');
        if (colon == std::string_view::npos) return replyError();
        const std::string_view annex = spec.substr(0U, colon);
        const std::string_view range = spec.substr(colon + 1U);
        const auto comma = range.find(',');
        if (annex != "target.xml" || comma == std::string_view::npos) return replyUnsupported();
        const auto offset = parseHex(range.substr(0U, comma));
        const auto length = parseHex(range.substr(comma + 1U));
        if (!offset || !length) return replyError();
        if (*offset >= target_description.size()) return sendPacket("l");
        const std::size_t available = target_description.size() - static_cast<std::size_t>(*offset);
        const std::size_t count = std::min<std::size_t>(
            static_cast<std::size_t>(*length), available
        );
        std::string reply;
        reply.reserve(count + 1U);
        reply.push_back(count < available ? 'm' : 'l');
        reply.append(target_description.substr(static_cast<std::size_t>(*offset), count));
        return sendPacket(reply);
    }
    if (packet == "qfThreadInfo") return sendPacket("m1");
    if (packet == "qsThreadInfo") return sendPacket("l");
    if (packet == "qC" || packet.starts_with("qC")) return sendPacket("QC1");
    if (packet == "qAttached" || packet.starts_with("qAttached:")) return sendPacket("1");
    if (packet == "qOffsets") return sendPacket("Text=0;Data=0;Bss=0");
    if (packet.starts_with("qRcmd,")) return handleMonitor(packet.substr(6U));
    return replyUnsupported();
}

bool GdbSession::Implementation::handleSet(const std::string_view packet) {
    if (packet == "QStartNoAckMode") {
        if (!sendPacket("OK")) return false;
        no_ack = true;
        return true;
    }
    return replyUnsupported();
}

bool GdbSession::Implementation::handleV(const std::string_view packet) {
    if (packet == "vCont?") return sendPacket("vCont;c;C;s;S");
    if (packet.starts_with("vCont;")) return handleVCont(packet);
    // Extended-mode process management: killing an inferior keeps the debug
    // connection, and starting one restores reset state and reports a stop.
    if (packet.starts_with("vKill")) return sendPacket("OK");
    if (packet.starts_with("vRun")) {
        const auto result = board.reset();
        if (!result) return replyError();
        return sendStopReply(signal_trap);
    }
    if (packet.starts_with("vAttach")) return sendStopReply(signal_trap);
    return replyUnsupported();
}

bool GdbSession::Implementation::dispatch(const std::string_view packet) {
    if (packet.empty()) return replyUnsupported();
    switch (packet.front()) {
    case '?': return sendPacket(last_stop);
    case 'g': return sendPacket(readAllRegisters());
    case 'G': return writeAllRegisters(packet.substr(1U)) ? sendPacket("OK") : replyError();
    case 'p': return handleRegisterRead(packet.substr(1U));
    case 'P': return handleRegisterWrite(packet.substr(1U));
    case 'm': return handleMemoryRead(packet);
    case 'M': return handleMemoryWrite(packet, false);
    case 'X': return handleMemoryWrite(packet, true);
    case 'c':
    case 'C':
    case 's':
    case 'S': return resume(packet);
    case 'Z': return handleBreakpoint(packet, true);
    case 'z': return handleBreakpoint(packet, false);
    case 'H':
    case 'T':
    case '!': return sendPacket("OK");
    case 'R': {
        const auto result = board.reset();
        return result ? sendPacket("OK") : replyError();
    }
    case 'D':
        if (!sendPacket("OK")) return false;
        end = GdbSessionEnd::detached;
        return false;
    case 'k':
        end = GdbSessionEnd::killed;
        return false;
    case 'v': return handleV(packet);
    case 'q': return handleQuery(packet);
    case 'Q': return handleSet(packet);
    default: return replyUnsupported();
    }
}

GdbSession::GdbSession(
    sim::Board& board,
    RspTransport& transport,
    GdbServerOptions options,
    MonitorEndpoint* monitor
) : implementation_(std::make_unique<Implementation>(
        board, transport, std::move(options), monitor
    )) {}

GdbSession::~GdbSession() = default;

Result<GdbSessionEnd> GdbSession::run() {
    Implementation& self = *implementation_;
    while (!self.end) {
        if (self.monitor != nullptr) self.monitor->service(false);
        const auto wait = self.transport.waitForInput(std::chrono::milliseconds(20));
        if (wait == TransportWait::timeout) continue;
        if (wait == TransportWait::closed) {
            if (self.end) break;
            return GdbSessionEnd::disconnected;
        }
        const auto packet = self.readPacket();
        if (!packet) {
            if (self.end) break;
            return GdbSessionEnd::disconnected;
        }
        if (!self.dispatch(*packet)) break;
    }
    return self.end.value_or(GdbSessionEnd::disconnected);
}

namespace {

/** @brief Waits briefly for one acceptable pending GDB connection. */
bool waitForConnection(const int listener, const bool monitor_active) {
    pollfd pending{listener, POLLIN, 0};
    const int timeout_ms = monitor_active ? 20 : 200;
    const int ready = ::poll(&pending, 1, timeout_ms);
    return ready > 0 && (pending.revents & POLLIN) != 0;
}

} // namespace

Result<GdbSessionEnd> serveGdb(
    sim::Board& board,
    const GdbServerOptions& options,
    std::ostream& out
) {
    std::uint16_t gdb_port = options.port;
    auto listener = detail::openTcpListener(options.bind_address, options.port, gdb_port);
    if (!listener) return listener.error();
    out << "fil debug: board=" << board.config().name
        << " GDB on " << options.bind_address << ':' << gdb_port << '\n' << std::flush;

    std::unique_ptr<MonitorEndpoint> monitor;
    if (options.telnet_enabled) {
        std::uint16_t telnet_port = options.telnet_port;
        auto endpoint = createTcpMonitorEndpoint(
            board, options.bind_address, options.telnet_port, telnet_port
        );
        if (!endpoint) return endpoint.error();
        monitor = std::move(endpoint.value());
        out << "fil debug: telnet monitor on " << options.bind_address << ':' << telnet_port
            << '\n' << std::flush;
    }

    for (;;) {
        int client = -1;
        while (client < 0) {
            if (monitor != nullptr) monitor->service(false);
            if (!waitForConnection(listener.value().descriptor(), monitor != nullptr)) {
                continue;
            }
            client = ::accept(listener.value().descriptor(), nullptr, nullptr);
            if (client >= 0) break;
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK
                || errno == ECONNABORTED) {
                client = -1;
                continue;
            }
            return Error{
                ErrorCategory::io,
                "GDB connection accept failed: " + std::string(std::strerror(errno)),
                std::nullopt
            };
        }
        static_cast<void>(detail::setBlocking(client));
        out << "fil debug: GDB connected" << '\n' << std::flush;

        SocketTransport transport(client);
        GdbSession session(board, transport, options, monitor.get());
        const auto result = session.run();
        if (!result) return result.error();
        out << "fil debug: GDB session ended (" << gdbSessionEndName(result.value()) << ")\n"
            << std::flush;
        if (options.once) return result.value();
        out << "fil debug: waiting for the next GDB connection\n" << std::flush;
    }
}

} // namespace fil::debug
