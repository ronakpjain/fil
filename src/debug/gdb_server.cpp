#include "fil/debug/gdb_server.hpp"

#include "fil/cpu/cortex_m4.hpp"
#include "fil/debug/live_gdb.hpp"
#include "fil/debug/monitor.hpp"
#include "fil/mem/memory_bus.hpp"
#include "fil/sim/board.hpp"
#include "memory_map.hpp"
#include "registers.hpp"
#include "rsp.hpp"
#include "tcp_listener.hpp"

#include <algorithm>
#include <bit>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <optional>
#include <poll.h>
#include <span>
#include <string>
#include <string_view>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace fil::debug {
namespace {

using detail::RspConnection;
using detail::SocketTransport;
using detail::double_register_nibbles;
using detail::double_register_count;
using detail::decodeBinaryBytes;
using detail::decodeHexBytes;
using detail::parseHex;
using detail::parseHex32;
using detail::parseLittleEndianHex;
using detail::register_count;
using detail::pc_register_index;
using detail::single_register_nibbles;
using detail::sp_register_index;

constexpr std::uint8_t signal_interrupt = 2U;
constexpr std::uint8_t signal_illegal_instruction = 4U;
constexpr std::uint8_t signal_trap = 5U;
constexpr std::uint8_t signal_abort = 6U;
constexpr std::uint8_t signal_segmentation = 11U;

// GDB's binary-download path (used by `load`) writes roughly 16 KiB per
// packet even when a smaller PacketSize was advertised, so accept generously
// more than the advertised packet size.
constexpr std::size_t maximum_memory_request = 32U * 1024U;

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
        DebugEndpoint* monitor_value,
        DebugEndpoint* live_value
    ) : board(board_value), transport(transport_value), connection(transport_value),
        options(std::move(options_value)), monitor(monitor_value), live(live_value) {}

    sim::Board& board;
    RspTransport& transport;
    RspConnection connection;
    GdbServerOptions options;
    DebugEndpoint* monitor{nullptr};
    DebugEndpoint* live{nullptr};
    std::vector<std::uint32_t> breakpoints;
    std::vector<std::uint32_t> hardware_breakpoints;
    std::vector<std::uint32_t> combined_breakpoints;
    std::vector<sim::Watchpoint> watchpoints;
    std::string last_stop{"S05"};
    bool semihosting{false};
    std::optional<GdbSessionEnd> end;

    /** @brief Services the telnet monitor and live read-only sessions. */
    void serviceEndpoints(const bool running) {
        if (monitor != nullptr) monitor->service(running);
        if (live != nullptr) live->service(running);
    }

    [[nodiscard]] std::optional<std::string> readPacket() { return connection.readPacket(); }
    [[nodiscard]] bool sendPacket(const std::string_view payload) {
        return connection.sendPacket(payload);
    }
    [[nodiscard]] bool sendMonitor(const std::string_view text) {
        return connection.sendConsole(text);
    }
    [[nodiscard]] bool sendStopReply(
        const std::uint8_t signal,
        const std::string_view reason = {}
    ) {
        std::string payload = "T";
        payload += detail::hexValue(signal, 1U);
        if (!reason.empty()) {
            payload.append(reason);
            payload += ":;";
        }
        last_stop = payload;
        return sendPacket(payload);
    }
    [[nodiscard]] bool sendStopReply(
        const std::uint8_t signal,
        const std::string_view key,
        const std::uint32_t value
    ) {
        std::string payload = "T";
        payload += detail::hexValue(signal, 1U);
        payload.append(key);
        payload += ':';
        payload += detail::hexNumber(value, 8U);
        payload += ';';
        last_stop = payload;
        return sendPacket(payload);
    }
    [[nodiscard]] bool replyError() { return sendPacket("E01"); }
    [[nodiscard]] bool replyUnsupported() { return sendPacket(""); }

    [[nodiscard]] bool dispatch(std::string_view packet);
    [[nodiscard]] bool handleQuery(std::string_view packet);
    [[nodiscard]] bool handleSet(std::string_view packet);
    [[nodiscard]] bool handleV(std::string_view packet);
    [[nodiscard]] bool handleVCont(std::string_view packet);
    [[nodiscard]] bool handleMonitor(std::string_view hex);
    [[nodiscard]] bool handleBreakpoint(std::string_view packet, bool insert);
    [[nodiscard]] bool handleFlashErase(std::string_view packet);
    [[nodiscard]] bool handleFlashWrite(std::string_view packet);
    [[nodiscard]] bool handleMemoryRead(std::string_view packet);
    [[nodiscard]] bool handleMemoryWrite(std::string_view packet, bool binary);
    [[nodiscard]] bool handleRegisterRead(std::string_view text);
    [[nodiscard]] bool handleRegisterWrite(std::string_view text);
    [[nodiscard]] bool resume(std::string_view packet);
    [[nodiscard]] bool resumeTarget(bool step);
    [[nodiscard]] bool serviceSemihosting(std::string& console);
    void refreshCombinedBreakpoints();
};

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

bool GdbSession::Implementation::handleRegisterRead(const std::string_view text) {
    const auto index = parseHex(text);
    if (!index || *index >= register_count) return replyError();
    const auto value = detail::readSingleRegister(
        board, static_cast<std::uint32_t>(*index)
    );
    if (!value) return replyError();
    return sendPacket(*value);
}

bool GdbSession::Implementation::handleRegisterWrite(const std::string_view text) {
    const auto equals = text.find('=');
    if (equals == std::string_view::npos) return replyError();
    const auto index = parseHex(text.substr(0U, equals));
    const auto value = parseLittleEndianHex(text.substr(equals + 1U));
    if (!index || !value || *index >= register_count) return replyError();
    if (!detail::writeSingleRegister(
            board, static_cast<std::uint32_t>(*index), *value)) return replyError();
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
        text += detail::hexValue(byte.value(), 1U);
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
    const auto comma = packet.find(',');
    if (comma == std::string_view::npos) return replyError();
    const auto kind = packet.find(',', comma + 1U);
    const auto address = parseHex32(kind == std::string_view::npos
        ? packet.substr(comma + 1U)
        : packet.substr(comma + 1U, kind - comma - 1U));
    if (!address) return replyError();
    std::uint32_t kind_value = 4U;
    if (kind != std::string_view::npos) {
        const auto parsed = parseHex(packet.substr(kind + 1U));
        if (!parsed || *parsed == 0U || *parsed > 8U) return replyError();
        kind_value = static_cast<std::uint32_t>(*parsed);
    }
    if (type <= '1') {
        // Instruction breakpoints compare even Thumb addresses.
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
    // Data watchpoints: Z2 write, Z3 read, Z4 access.
    const sim::Watchpoint watchpoint{
        *address,
        kind_value,
        type == '2' ? sim::Watchpoint::Access::write
            : type == '3' ? sim::Watchpoint::Access::read
                          : sim::Watchpoint::Access::access
    };
    const auto same = [&watchpoint](const sim::Watchpoint& candidate) {
        return candidate.address == watchpoint.address
            && candidate.length == watchpoint.length
            && candidate.access == watchpoint.access;
    };
    const auto position = std::find_if(watchpoints.begin(), watchpoints.end(), same);
    if (insert) {
        if (position == watchpoints.end()) watchpoints.push_back(watchpoint);
    } else if (position != watchpoints.end()) {
        watchpoints.erase(position);
    }
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
        // Service monitor and live-session clients at instruction boundaries;
        // reads never halt the running target and side-effecting MMIO stays
        // refused.
        serviceEndpoints(true);
        if (!step && transport.inputPending()) {
            return sendStopReply(signal_interrupt);
        }
        run.breakpoints = stops;
        run.watchpoints = std::span<const sim::Watchpoint>{watchpoints};
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
        case sim::BoardStopReason::breakpoint: {
            if (semihosting && result.diagnostic.raw == 0xbeabU
                && result.diagnostic.instruction_size == 2U) {
                cpu::CpuState& state = board.cpu().state();
                state.halted = false;
                const bool exit_requested = state.r[0] == 0x18U;
                std::string console;
                if (!serviceSemihosting(console)) return sendStopReply(signal_trap);
                if (!console.empty() && !sendMonitor(console)) return false;
                // On continue, keep running after a serviced call; SYS_EXIT
                // leaves the target stopped for inspection.
                if (!step && !exit_requested) continue;
                return sendStopReply(signal_trap);
            }
            return sendStopReply(signal_trap);
        }
        case sim::BoardStopReason::halted:
            return sendStopReply(signal_trap);
        case sim::BoardStopReason::watchpoint:
            // GDB identifies the triggering watchpoint from this address and
            // its own breakpoint table; the protocol key is always `watch`.
            if (!result.watchpoint) return sendStopReply(signal_trap);
            return sendStopReply(signal_trap, "watch", result.watchpoint->address);
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

bool GdbSession::Implementation::serviceSemihosting(std::string& console) {
    cpu::CpuState& state = board.cpu().state();
    const auto readByte = [this](const std::uint32_t address) {
        return board.memory().read8(
            address, mem::AccessContext{mem::AccessType::debug, 0U}
        );
    };
    const std::uint32_t operation = state.r[0];
    switch (operation) {
    case 0x04U: { // SYS_WRITE0: NUL-terminated string at r1.
        console.clear();
        for (std::uint32_t index = 0U; index < 8192U; ++index) {
            const std::uint32_t address = state.r[1] + index;
            if (address < state.r[1]) break;
            const auto byte = readByte(address);
            if (!byte || byte.value() == 0U) break;
            console.push_back(static_cast<char>(byte.value()));
        }
        state.r[0] = 0U;
        return true;
    }
    case 0x05U: { // SYS_WRITE: handle, buffer, length.
        const std::uint32_t buffer = state.r[2];
        const std::uint32_t length = state.r[3];
        const std::uint32_t capped = std::min<std::uint32_t>(length, 8192U);
        console.clear();
        console.reserve(capped);
        for (std::uint32_t index = 0U; index < capped; ++index) {
            const std::uint32_t address = buffer + index;
            if (address < buffer) break;
            const auto byte = readByte(address);
            if (!byte) break;
            console.push_back(static_cast<char>(byte.value()));
        }
        state.r[0] = length;
        return true;
    }
    case 0x07U: // SYS_READC: no interactive console input.
        state.r[0] = 0U;
        return true;
    case 0x09U: // SYS_ISTTY: stdout/stderr handles are terminals.
        state.r[0] = state.r[1] == 1U || state.r[1] == 2U ? 1U : 0U;
        return true;
    case 0x18U: // SYS_EXIT.
        console = "semihosting exit: reason " + std::to_string(state.r[1]) + "\n";
        state.r[0] = 0U;
        return true;
    default:
        state.r[0] = std::numeric_limits<std::uint32_t>::max(); // -1: unsupported
        return true;
    }
}

bool GdbSession::Implementation::handleMonitor(const std::string_view hex) {
    const auto bytes = decodeHexBytes(hex);
    if (!bytes) return replyError();
    const std::string_view command(
        reinterpret_cast<const char*>(bytes->data()), bytes->size()
    );
    if (command == "arm semihosting enable") {
        semihosting = true;
        return sendPacket("OK");
    }
    if (command == "arm semihosting disable") {
        semihosting = false;
        return sendPacket("OK");
    }
    if (command == "arm semihosting") {
        if (!sendMonitor(semihosting ? "semihosting is enabled\n"
                                     : "semihosting is disabled\n")) {
            return false;
        }
        return sendPacket("OK");
    }
    // Share the telnet monitor command layer so `monitor reset halt`,
    // `monitor halt`, `monitor mdw ...`, and friends behave identically.
    const MonitorReply reply = executeMonitorCommand(board, command, false);
    if (!reply.output.empty() && !sendMonitor(reply.output)) return false;
    return sendPacket("OK");
}

bool GdbSession::Implementation::handleQuery(const std::string_view packet) {
    if (packet == "qSupported" || packet.starts_with("qSupported:")) {
        return sendPacket(
            "PacketSize=4000;qXfer:features:read+;qXfer:memory-map:read+;"
            "QStartNoAckMode+;swbreak+;hwbreak+;vContSupported+"
        );
    }
    constexpr std::string_view feature_prefix = "qXfer:features:read:";
    if (packet.starts_with(feature_prefix)) {
        const auto feature = detail::readFeaturePacket(packet);
        return feature ? sendPacket(*feature) : replyUnsupported();
    }
    if (const auto memory_map = detail::readMemoryMapPacket(board, packet)) {
        return sendPacket(*memory_map);
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
        connection.setNoAck(true);
        return true;
    }
    return replyUnsupported();
}

bool GdbSession::Implementation::handleFlashErase(const std::string_view packet) {
    const std::string_view args = packet.substr(std::string_view{"vFlashErase:"}.size());
    const auto comma = args.find(',');
    if (comma == std::string_view::npos) return replyError();
    const auto address = parseHex32(args.substr(0U, comma));
    const auto length = parseHex(args.substr(comma + 1U));
    if (!address || !length || *length == 0U) return replyError();
    if (*length > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())
            - *address + 1U) {
        return replyError();
    }
    // Modeled flash erases to 0xFF, like the real part.
    const auto base = static_cast<std::uint32_t>(*address);
    std::uint64_t offset = 0U;
    while (offset < *length && ((base + offset) & 3U) != 0U) {
        const auto written = board.memory().write8(
            static_cast<std::uint32_t>(base + offset), 0xffU,
            mem::AccessContext{mem::AccessType::debug, 0U}
        );
        if (!written) return replyError();
        ++offset;
    }
    for (; offset + 4U <= *length; offset += 4U) {
        const auto written = board.memory().write32(
            static_cast<std::uint32_t>(base + offset), 0xffffffffU,
            mem::AccessContext{mem::AccessType::debug, 0U}
        );
        if (!written) return replyError();
    }
    for (; offset < *length; ++offset) {
        const auto written = board.memory().write8(
            static_cast<std::uint32_t>(base + offset), 0xffU,
            mem::AccessContext{mem::AccessType::debug, 0U}
        );
        if (!written) return replyError();
    }
    return sendPacket("OK");
}

bool GdbSession::Implementation::handleFlashWrite(const std::string_view packet) {
    const std::string_view args = packet.substr(std::string_view{"vFlashWrite:"}.size());
    const auto colon = args.find(':');
    if (colon == std::string_view::npos) return replyError();
    const auto address = parseHex32(args.substr(0U, colon));
    if (!address) return replyError();
    const auto bytes = decodeBinaryBytes(args.substr(colon + 1U));
    if (!bytes || bytes->size() > maximum_memory_request) return replyError();
    if (bytes->size() > static_cast<std::size_t>(
            std::numeric_limits<std::uint32_t>::max() - *address) + 1U) {
        return replyError();
    }
    for (std::size_t index = 0U; index < bytes->size(); ++index) {
        const auto written = board.memory().write8(
            static_cast<std::uint32_t>(*address + index), (*bytes)[index],
            mem::AccessContext{mem::AccessType::debug, 0U}
        );
        if (!written) return replyError();
    }
    return sendPacket("OK");
}

bool GdbSession::Implementation::handleV(const std::string_view packet) {
    if (packet == "vCont?") return sendPacket("vCont;c;C;s;S");
    if (packet.starts_with("vCont;")) return handleVCont(packet);
    if (packet.starts_with("vFlashErase:")) return handleFlashErase(packet);
    if (packet.starts_with("vFlashWrite:")) return handleFlashWrite(packet);
    if (packet == "vFlashDone") return sendPacket("OK");
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
    case 'g': return sendPacket(detail::readAllRegisters(board));
    case 'G': return detail::writeAllRegisters(board, packet.substr(1U))
        ? sendPacket("OK") : replyError();
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
    DebugEndpoint* monitor,
    DebugEndpoint* live
) : implementation_(std::make_unique<Implementation>(
        board, transport, std::move(options), monitor, live
    )) {}

GdbSession::~GdbSession() = default;

Result<GdbSessionEnd> GdbSession::run() {
    Implementation& self = *implementation_;
    while (!self.end) {
        self.serviceEndpoints(false);
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

        // Live watch (cortex-debug) opens a second read-only connection on
        // this same port; the primary session services it at slice boundaries.
        auto live = createLiveGdbServer(board, listener.value().descriptor());
        SocketTransport transport(client);
        GdbSession session(board, transport, options, monitor.get(), live.get());
        const auto result = session.run();
        if (!result) return result.error();
        out << "fil debug: GDB session ended (" << gdbSessionEndName(result.value()) << ")\n"
            << std::flush;
        if (options.once) return result.value();
        out << "fil debug: waiting for the next GDB connection\n" << std::flush;
    }
}

} // namespace fil::debug
