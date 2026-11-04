#include "fil/cli/cli.hpp"
#include "fil/cli/network_protocol.hpp"

#include "fil/common/error.hpp"
#include "fil/config/config.hpp"
#include "fil/devices/can_bus.hpp"
#include "fil/cpu/decoder.hpp"
#include "fil/elf/elf_loader.hpp"
#include "fil/hardware/comparison.hpp"
#include "fil/sim/board.hpp"
#include "fil/sim/stimulus.hpp"
#include "fil/sim/world.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <ostream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include <poll.h>
#include <thread>
#include <unistd.h>

namespace fil::cli {

namespace {

constexpr std::string_view version = "0.1.0";

std::optional<std::string_view> nextOptionValue(
    const std::span<const std::string_view> args,
    std::size_t& index
) {
    if (index + 1U >= args.size()) return std::nullopt;
    return args[++index];
}

/**
 * @brief Parses a simulated-time limit into nanoseconds.
 *
 * A bare number is seconds; `ms`, `us`, `s`, and `m` suffixes select the unit.
 * Zero means unlimited. The value is independent of the emulated clock.
 *
 * @param option Option name used in diagnostics.
 * @param text User-supplied value.
 * @return Nanoseconds, or an invalid-argument error.
 */
Result<std::uint64_t> parseDurationNs(
    const std::string_view option,
    const std::string_view text
) {
    std::string_view number = text;
    std::uint64_t multiplier = 1'000'000'000ULL; // Bare numbers are seconds.
    if (text.size() >= 2U && text.ends_with("ms")) {
        number = text.substr(0U, text.size() - 2U);
        multiplier = 1'000'000ULL;
    } else if (text.size() >= 2U && text.ends_with("us")) {
        number = text.substr(0U, text.size() - 2U);
        multiplier = 1'000ULL;
    } else if (text.size() >= 1U && text.ends_with('s')) {
        number = text.substr(0U, text.size() - 1U);
        multiplier = 1'000'000'000ULL;
    } else if (text.size() >= 1U && text.ends_with('m')) {
        number = text.substr(0U, text.size() - 1U);
        multiplier = 60'000'000'000ULL;
    }
    const auto parsed = config::parseUnsigned(number);
    if (!parsed) {
        return Error{ErrorCategory::invalid_argument,
                     std::string(option) + " requires a number with an optional us, ms, s, or m suffix",
                     std::nullopt};
    }
    if (parsed.value() != 0U
        && parsed.value() > std::numeric_limits<std::uint64_t>::max() / multiplier) {
        return Error{ErrorCategory::invalid_argument, std::string(option) + " is too large", std::nullopt};
    }
    return parsed.value() * multiplier;
}

struct PendingCanInjection {
    std::string bus;
    std::uint64_t at_ms{0};
    devices::CanFrame frame;
};

std::optional<std::uint8_t> hexNibble(const char value) noexcept {
    if (value >= '0' && value <= '9') return static_cast<std::uint8_t>(value - '0');
    if (value >= 'a' && value <= 'f') return static_cast<std::uint8_t>(value - 'a' + 10);
    if (value >= 'A' && value <= 'F') return static_cast<std::uint8_t>(value - 'A' + 10);
    return std::nullopt;
}

Result<PendingCanInjection> parseCanInjection(const std::string_view text) {
    const std::size_t first_colon = text.find(':');
    const std::size_t second_colon = first_colon == std::string_view::npos
        ? std::string_view::npos : text.find(':', first_colon + 1U);
    if (first_colon == 0U || first_colon == std::string_view::npos
        || second_colon == std::string_view::npos) {
        return Error{ErrorCategory::invalid_argument,
                     "CAN injection must be BUS[@TIME_MS]:ID:HEXDATA", std::nullopt};
    }
    PendingCanInjection injection;
    std::string_view endpoint = text.substr(0U, first_colon);
    const std::size_t at = endpoint.find('@');
    if (at != std::string_view::npos) {
        if (at == 0U || at + 1U == endpoint.size()) {
            return Error{ErrorCategory::invalid_argument, "invalid CAN injection time", std::nullopt};
        }
        auto parsed_time = config::parseUnsigned(endpoint.substr(at + 1U));
        if (!parsed_time) return parsed_time.error();
        injection.at_ms = parsed_time.value();
        endpoint = endpoint.substr(0U, at);
    }
    injection.bus = std::string(endpoint);

    auto parsed_id = config::parseUnsigned(
        text.substr(first_colon + 1U, second_colon - first_colon - 1U)
    );
    if (!parsed_id || parsed_id.value() > 0x1fffffffU) {
        return Error{ErrorCategory::invalid_argument, "invalid CAN injection identifier", std::nullopt};
    }
    injection.frame.id = static_cast<std::uint32_t>(parsed_id.value());
    injection.frame.extended = injection.frame.id > 0x7ffU;

    const std::string_view payload = text.substr(second_colon + 1U);
    if ((payload.size() & 1U) != 0U || payload.size() > injection.frame.data.size() * 2U) {
        return Error{ErrorCategory::invalid_argument,
                     "CAN injection payload must contain 0..64 complete hex bytes", std::nullopt};
    }
    const std::size_t byte_count = payload.size() / 2U;
    const auto dlc = devices::lengthToDlc(byte_count);
    if (!dlc || devices::dlcToLength(*dlc) != byte_count) {
        return Error{ErrorCategory::invalid_argument,
                     "CAN-FD payload length must be 0..8, 12, 16, 20, 24, 32, 48, or 64", std::nullopt};
    }
    for (std::size_t index = 0; index < byte_count; ++index) {
        const auto high = hexNibble(payload[index * 2U]);
        const auto low = hexNibble(payload[index * 2U + 1U]);
        if (!high || !low) {
            return Error{ErrorCategory::invalid_argument, "CAN injection payload is not hexadecimal", std::nullopt};
        }
        injection.frame.data[index] = static_cast<std::uint8_t>((*high << 4U) | *low);
    }
    injection.frame.dlc = *dlc;
    injection.frame.fd = byte_count > 8U;
    return injection;
}

struct PendingAdcInjection {
    std::string board;
    std::string instance;
    std::uint8_t channel{0};
    std::uint16_t value{0};
};

struct PendingGpioInjection {
    std::string board;
    std::string port;
    std::uint8_t pin{0};
    std::optional<bool> level; ///< nullopt releases the externally driven pin.
};

std::vector<std::string> splitTokens(const std::string_view text) {
    std::istringstream stream{std::string(text)};
    std::vector<std::string> tokens;
    std::string token;
    while (stream >> token) tokens.push_back(token);
    return tokens;
}

Result<PendingAdcInjection> parseAdcInjection(const std::string_view text) {
    const auto tokens = splitTokens(text);
    if (tokens.size() != 5U) {
        return Error{ErrorCategory::invalid_argument,
                     "ADC injection must be adc BOARD INSTANCE CHANNEL VALUE", std::nullopt};
    }
    PendingAdcInjection injection;
    injection.board = tokens[1];
    injection.instance = tokens[2];
    const auto channel = config::parseUnsigned(tokens[3]);
    if (!channel || channel.value() > 19U) {
        return Error{ErrorCategory::invalid_argument,
                     "ADC injection channel must be an integer from 0 through 19", std::nullopt};
    }
    const auto value = config::parseUnsigned(tokens[4]);
    if (!value || value.value() > 0x0fffU) {
        return Error{ErrorCategory::invalid_argument,
                     "ADC injection value must be an integer from 0 through 4095", std::nullopt};
    }
    injection.channel = static_cast<std::uint8_t>(channel.value());
    injection.value = static_cast<std::uint16_t>(value.value());
    return injection;
}

Result<PendingGpioInjection> parseGpioInjection(const std::string_view text) {
    const auto tokens = splitTokens(text);
    if (tokens.size() != 5U) {
        return Error{ErrorCategory::invalid_argument,
                     "GPIO injection must be gpio BOARD PORT PIN VALUE", std::nullopt};
    }
    PendingGpioInjection injection;
    injection.board = tokens[1];
    injection.port = tokens[2];
    const auto pin = config::parseUnsigned(tokens[3]);
    if (!pin || pin.value() > 15U) {
        return Error{ErrorCategory::invalid_argument,
                     "GPIO injection pin must be an integer from 0 through 15", std::nullopt};
    }
    if (tokens[4] == "release") {
        injection.level = std::nullopt;
    } else if (tokens[4] == "0") {
        injection.level = false;
    } else if (tokens[4] == "1") {
        injection.level = true;
    } else {
        return Error{ErrorCategory::invalid_argument,
                     "GPIO injection value must be 0, 1, or release", std::nullopt};
    }
    injection.pin = static_cast<std::uint8_t>(pin.value());
    return injection;
}

bool validUtf8(const std::string_view text) noexcept {
    std::size_t index = 0U;
    while (index < text.size()) {
        const auto first = static_cast<std::uint8_t>(text[index]);
        if (first <= 0x7fU) {
            ++index;
            continue;
        }
        if (first >= 0xc2U && first <= 0xdfU) {
            if (index + 1U >= text.size()) return false;
            const auto second = static_cast<std::uint8_t>(text[index + 1U]);
            if (second < 0x80U || second > 0xbfU) return false;
            index += 2U;
            continue;
        }
        if (first >= 0xe0U && first <= 0xefU) {
            if (index + 2U >= text.size()) return false;
            const auto second = static_cast<std::uint8_t>(text[index + 1U]);
            const auto third = static_cast<std::uint8_t>(text[index + 2U]);
            if (third < 0x80U || third > 0xbfU) return false;
            if ((first == 0xe0U && (second < 0xa0U || second > 0xbfU))
                || (first == 0xedU && (second < 0x80U || second > 0x9fU))
                || (first != 0xe0U && first != 0xedU
                    && (second < 0x80U || second > 0xbfU))) return false;
            index += 3U;
            continue;
        }
        if (first >= 0xf0U && first <= 0xf4U) {
            if (index + 3U >= text.size()) return false;
            const auto second = static_cast<std::uint8_t>(text[index + 1U]);
            const auto third = static_cast<std::uint8_t>(text[index + 2U]);
            const auto fourth = static_cast<std::uint8_t>(text[index + 3U]);
            if (third < 0x80U || third > 0xbfU || fourth < 0x80U || fourth > 0xbfU) return false;
            if ((first == 0xf0U && (second < 0x90U || second > 0xbfU))
                || (first == 0xf4U && (second < 0x80U || second > 0x8fU))
                || (first != 0xf0U && first != 0xf4U
                    && (second < 0x80U || second > 0xbfU))) return false;
            index += 4U;
            continue;
        }
        return false;
    }
    return true;
}

class BinaryPayloadWriter {
public:
    void u8(const std::uint8_t value) { bytes_.push_back(value); }

    void u16(const std::uint16_t value) {
        u8(static_cast<std::uint8_t>(value & 0xffU));
        u8(static_cast<std::uint8_t>((value >> 8U) & 0xffU));
    }

    void u32(const std::uint32_t value) {
        for (std::uint32_t index = 0U; index < 4U; ++index) {
            u8(static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU));
        }
    }

    void u64(const std::uint64_t value) {
        for (std::uint32_t index = 0U; index < 8U; ++index) {
            u8(static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU));
        }
    }

    void string(const std::string_view value) {
        if (value.size() > std::numeric_limits<std::uint16_t>::max() || !validUtf8(value)) {
            valid_ = false;
            return;
        }
        u16(static_cast<std::uint16_t>(value.size()));
        bytes_.insert(bytes_.end(), value.begin(), value.end());
    }

    void raw(const std::span<const std::uint8_t> value) {
        bytes_.insert(bytes_.end(), value.begin(), value.end());
    }

    [[nodiscard]] bool valid() const noexcept {
        return valid_ && bytes_.size() <= network_protocol::maximum_payload_size;
    }

    [[nodiscard]] std::vector<std::uint8_t> take() && { return std::move(bytes_); }

private:
    bool valid_{true};
    std::vector<std::uint8_t> bytes_;
};

class BinaryPayloadReader {
public:
    explicit BinaryPayloadReader(const std::span<const std::uint8_t> bytes) : bytes_(bytes) {}

    bool u8(std::uint8_t& value) {
        if (remaining() < 1U) return false;
        value = bytes_[position_++];
        return true;
    }

    bool u16(std::uint16_t& value) {
        std::uint8_t low = 0U;
        std::uint8_t high = 0U;
        if (!u8(low) || !u8(high)) return false;
        value = static_cast<std::uint16_t>(low)
            | static_cast<std::uint16_t>(static_cast<std::uint16_t>(high) << 8U);
        return true;
    }

    bool u32(std::uint32_t& value) {
        if (remaining() < 4U) return false;
        value = 0U;
        for (std::uint32_t index = 0U; index < 4U; ++index) {
            value |= static_cast<std::uint32_t>(bytes_[position_++]) << (index * 8U);
        }
        return true;
    }

    bool u64(std::uint64_t& value) {
        if (remaining() < 8U) return false;
        value = 0U;
        for (std::uint32_t index = 0U; index < 8U; ++index) {
            value |= static_cast<std::uint64_t>(bytes_[position_++]) << (index * 8U);
        }
        return true;
    }

    bool string(std::string& value) {
        std::uint16_t length = 0U;
        if (!u16(length) || remaining() < length) return false;
        const auto* const data = reinterpret_cast<const char*>(bytes_.data() + position_);
        value.assign(data, length);
        position_ += length;
        return validUtf8(value);
    }

    bool raw(const std::size_t length, std::span<const std::uint8_t>& value) {
        if (remaining() < length) return false;
        value = bytes_.subspan(position_, length);
        position_ += length;
        return true;
    }

    [[nodiscard]] bool finished() const noexcept { return position_ == bytes_.size(); }

private:
    [[nodiscard]] std::size_t remaining() const noexcept { return bytes_.size() - position_; }

    std::span<const std::uint8_t> bytes_;
    std::size_t position_{0U};
};

enum class ProtocolReplyStatus : std::uint16_t {
    ok = 0U,
    invalid_request = 1U,
    unknown_target = 2U,
    unsupported_message = 3U,
    queue_full = 4U,
};

enum class ProtocolEndReason : std::uint8_t {
    stop_request = 0U,
    input_eof = 1U,
    duration_reached = 2U,
    all_boards_stopped = 3U,
    runtime_error = 4U,
};

bool writeProtocolFrame(
    std::ostream& out,
    const network_protocol::MessageKind kind,
    const std::uint32_t request_id,
    BinaryPayloadWriter payload
) {
    if (!payload.valid()) return false;
    const network_protocol::Frame frame{kind, request_id, std::move(payload).take()};
    return network_protocol::writeFrame(out, frame).hasValue();
}

bool writeProtocolReply(
    std::ostream& out,
    const std::uint32_t request_id,
    const ProtocolReplyStatus status,
    const std::uint64_t time_ns,
    const std::string_view message
) {
    BinaryPayloadWriter payload;
    payload.u16(static_cast<std::uint16_t>(status));
    payload.u64(time_ns);
    payload.string(message);
    return writeProtocolFrame(out, network_protocol::MessageKind::reply, request_id,
                              std::move(payload));
}

bool writeProtocolTrace(std::ostream& out, const sim::TraceRecord& record) {
    if (record.fields.size() > std::numeric_limits<std::uint16_t>::max()) return false;
    BinaryPayloadWriter payload;
    payload.u64(record.time_ns);
    payload.u64(record.sequence);
    payload.string(record.source);
    payload.string(record.type);
    payload.u16(static_cast<std::uint16_t>(record.fields.size()));
    for (const auto& [key, value] : record.fields) {
        payload.string(key);
        payload.string(value);
    }
    return writeProtocolFrame(out, network_protocol::MessageKind::trace, 0U,
                              std::move(payload));
}

bool writeProtocolEnd(
    std::ostream& out,
    const std::uint64_t time_ns,
    const ProtocolEndReason reason,
    const ExitCode exit_code,
    const std::string_view message = {}
) {
    BinaryPayloadWriter payload;
    payload.u64(time_ns);
    payload.u8(static_cast<std::uint8_t>(reason));
    payload.u8(static_cast<std::uint8_t>(exit_code));
    payload.string(message);
    return writeProtocolFrame(out, network_protocol::MessageKind::end, 0U,
                              std::move(payload));
}

void printLiveTraceRecord(std::ostream& out, const sim::TraceRecord& record) {
    out << '[' << std::fixed << std::setprecision(3)
        << static_cast<double>(record.time_ns) / 1'000'000.0
        << " ms] " << record.source << ' ' << record.type;
    for (const auto& [key, value] : record.fields) {
        out << ' ' << key << '=' << value;
    }
    out << '\n' << std::flush;
}

std::string_view instructionName(const cpu::InstrKind kind) noexcept {
    using cpu::InstrKind;
    switch (kind) {
    case InstrKind::undefined: return "undefined";
    case InstrKind::mov: return "mov";
    case InstrKind::movw: return "movw";
    case InstrKind::movt: return "movt";
    case InstrKind::add: return "add";
    case InstrKind::adc: return "adc";
    case InstrKind::sub: return "sub";
    case InstrKind::sbc: return "sbc";
    case InstrKind::rsb: return "rsb";
    case InstrKind::cmp: return "cmp";
    case InstrKind::cmn: return "cmn";
    case InstrKind::tst: return "tst";
    case InstrKind::and_: return "and";
    case InstrKind::orr: return "orr";
    case InstrKind::eor: return "eor";
    case InstrKind::bic: return "bic";
    case InstrKind::mvn: return "mvn";
    case InstrKind::orn: return "orn";
    case InstrKind::mul: return "mul";
    case InstrKind::mla: return "mla";
    case InstrKind::mls: return "mls";
    case InstrKind::umull: return "umull";
    case InstrKind::udiv: return "udiv";
    case InstrKind::clz: return "clz";
    case InstrKind::bfc: return "bfc";
    case InstrKind::ubfx: return "ubfx";
    case InstrKind::sxtb: return "sxtb";
    case InstrKind::sxth: return "sxth";
    case InstrKind::uxtb: return "uxtb";
    case InstrKind::uxth: return "uxth";
    case InstrKind::rev: return "rev";
    case InstrKind::rev16: return "rev16";
    case InstrKind::revsh: return "revsh";
    case InstrKind::sdiv: return "sdiv";
    case InstrKind::smull: return "smull";
    case InstrKind::uadd8: return "uadd8";
    case InstrKind::sel: return "sel";
    case InstrKind::lsl: return "lsl";
    case InstrKind::lsr: return "lsr";
    case InstrKind::asr: return "asr";
    case InstrKind::ror: return "ror";
    case InstrKind::rrx: return "rrx";
    case InstrKind::ldr: return "ldr";
    case InstrKind::str: return "str";
    case InstrKind::ldrb: return "ldrb";
    case InstrKind::strb: return "strb";
    case InstrKind::ldrh: return "ldrh";
    case InstrKind::strh: return "strh";
    case InstrKind::ldrsb: return "ldrsb";
    case InstrKind::ldrsh: return "ldrsh";
    case InstrKind::ldrd: return "ldrd";
    case InstrKind::strd: return "strd";
    case InstrKind::ldm: return "ldm";
    case InstrKind::stm: return "stm";
    case InstrKind::push: return "push";
    case InstrKind::pop: return "pop";
    case InstrKind::b: return "b";
    case InstrKind::bl: return "bl";
    case InstrKind::bx: return "bx";
    case InstrKind::blx: return "blx";
    case InstrKind::tbb: return "tbb";
    case InstrKind::tbh: return "tbh";
    case InstrKind::cbz: return "cbz";
    case InstrKind::cbnz: return "cbnz";
    case InstrKind::it: return "it";
    case InstrKind::nop: return "nop";
    case InstrKind::dmb: return "dmb";
    case InstrKind::dsb: return "dsb";
    case InstrKind::isb: return "isb";
    case InstrKind::svc: return "svc";
    case InstrKind::mrs: return "mrs";
    case InstrKind::msr: return "msr";
    case InstrKind::cps: return "cps";
    case InstrKind::wfi: return "wfi";
    case InstrKind::wfe: return "wfe";
    case InstrKind::sev: return "sev";
    case InstrKind::vstm: return "vstm";
    case InstrKind::vldm: return "vldm";
    case InstrKind::vldr: return "vldr";
    case InstrKind::vstr: return "vstr";
    case InstrKind::vmov_core_to_single:
    case InstrKind::vmov_single_to_core:
    case InstrKind::vmov_single:
    case InstrKind::vmov_immediate: return "vmov";
    case InstrKind::vcvt_f32_s32: return "vcvt.f32.s32";
    case InstrKind::vcvt_f32_u32: return "vcvt.f32.u32";
    case InstrKind::vcvt_s32_f32: return "vcvt.s32.f32";
    case InstrKind::vcvt_u32_f32: return "vcvt.u32.f32";
    case InstrKind::vadd: return "vadd.f32";
    case InstrKind::vsub: return "vsub.f32";
    case InstrKind::vmul: return "vmul.f32";
    case InstrKind::vnmul: return "vnmul.f32";
    case InstrKind::vdiv: return "vdiv.f32";
    case InstrKind::vfma: return "vfma.f32";
    case InstrKind::vfms: return "vfms.f32";
    case InstrKind::vfnms: return "vfnms.f32";
    case InstrKind::vcmp: return "vcmp.f32";
    case InstrKind::vneg: return "vneg.f32";
    case InstrKind::vabs: return "vabs.f32";
    case InstrKind::vsqrt: return "vsqrt.f32";
    case InstrKind::vmrs: return "vmrs";
    case InstrKind::bkpt: return "bkpt";
    }
    return "unknown";
}

ExitCode disassembleWindowCommand(
    const std::span<const std::string_view> args,
    std::ostream& out,
    std::ostream& err
) {
    if (args.size() < 2U) {
        err << "fil: disasm-window requires a firmware path\n";
        return ExitCode::usage_error;
    }
    std::optional<std::uint32_t> address;
    std::uint64_t count = 32U;
    for (std::size_t index = 2U; index < args.size(); ++index) {
        const std::string_view option = args[index];
        if (option != "--addr" && option != "--count") {
            err << "fil: unknown disasm-window option: " << option << '\n';
            return ExitCode::usage_error;
        }
        if (++index >= args.size()) {
            err << "fil: " << option << " requires a value\n";
            return ExitCode::usage_error;
        }
        auto parsed = config::parseUnsigned(args[index]);
        if (!parsed) {
            err << "fil: invalid value for " << option << ": " << parsed.error().message << '\n';
            return ExitCode::usage_error;
        }
        if (option == "--addr") {
            if (parsed.value() > std::numeric_limits<std::uint32_t>::max()) {
                err << "fil: disassembly address exceeds target address space\n";
                return ExitCode::usage_error;
            }
            address = static_cast<std::uint32_t>(parsed.value());
        } else {
            count = parsed.value();
        }
    }
    auto image = elf::load(args[1]);
    if (!image) {
        err << "fil: " << formatError(image.error()) << '\n';
        return ExitCode::runtime_error;
    }
    std::uint32_t pc = address.value_or(image.value().entryPoint() & ~1U);
    if ((pc & 1U) != 0U) {
        err << "fil: disassembly address must be halfword aligned\n";
        return ExitCode::usage_error;
    }
    for (std::uint64_t index = 0U; index < count; ++index) {
        auto first_bytes = image.value().readLoadImage(pc, 2U);
        if (!first_bytes) {
            err << "fil: " << formatError(first_bytes.error()) << '\n';
            return ExitCode::runtime_error;
        }
        const std::uint16_t first = static_cast<std::uint16_t>(first_bytes.value()[0])
            | static_cast<std::uint16_t>(static_cast<std::uint16_t>(first_bytes.value()[1]) << 8U);
        const bool wide = cpu::is32BitThumbPrefix(first);
        std::uint16_t second = 0U;
        if (wide) {
            if (pc > std::numeric_limits<std::uint32_t>::max() - 2U) {
                err << "fil: wide instruction wraps the target address space\n";
                return ExitCode::runtime_error;
            }
            auto second_bytes = image.value().readLoadImage(pc + 2U, 2U);
            if (!second_bytes) {
                err << "fil: " << formatError(second_bytes.error()) << '\n';
                return ExitCode::runtime_error;
            }
            second = static_cast<std::uint16_t>(second_bytes.value()[0])
                | static_cast<std::uint16_t>(static_cast<std::uint16_t>(second_bytes.value()[1]) << 8U);
        }
        const auto decoded = wide ? cpu::decode32(first, second) : cpu::decode16(first);
        out << "0x" << std::hex << std::setfill('0') << std::setw(8) << pc << ": "
            << std::setw(4) << first;
        if (wide) out << ' ' << std::setw(4) << second;
        else out << "     ";
        out << "  " << (decoded ? instructionName(decoded->kind) : "unsupported");
        if (const elf::ElfSymbol* symbol = image.value().symbolAtOrBefore(pc)) {
            out << "  ; " << symbol->name << "+0x" << (pc - symbol->address);
        }
        out << std::dec << '\n';
        const std::uint32_t width = wide ? 4U : 2U;
        if (pc > std::numeric_limits<std::uint32_t>::max() - width) break;
        pc += width;
    }
    return ExitCode::success;
}

/** @brief One report field; an empty value leaves the key bare as a section head. */
struct ReportLine {
    std::string key;
    std::string value;
};

/**
 * @brief Prints report fields aligned under their longest key.
 *
 * @param out Destination stream.
 * @param indent Leading spaces on every line.
 * @param lines Fields in print order.
 */
void printReportLines(
    std::ostream& out,
    const std::size_t indent,
    const std::vector<ReportLine>& lines
) {
    std::size_t width = 0U;
    for (const ReportLine& line : lines) width = std::max(width, line.key.size());
    for (const ReportLine& line : lines) {
        out << std::string(indent, ' ') << line.key << ':';
        if (!line.value.empty()) {
            out << std::string(width - line.key.size() + 1U, ' ') << line.value;
        }
        out << '\n';
    }
}

/** @brief Formats simulated-time with a readable unit and exact nanoseconds. */
std::string formatSimTime(const std::uint64_t time_ns) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(3);
    if (time_ns >= 1'000'000'000ULL) text << static_cast<double>(time_ns) / 1e9 << " s";
    else if (time_ns >= 1'000'000ULL) text << static_cast<double>(time_ns) / 1e6 << " ms";
    else if (time_ns >= 1'000ULL) text << static_cast<double>(time_ns) / 1e3 << " us";
    else return std::to_string(time_ns) + " ns";
    return text.str() + " (" + std::to_string(time_ns) + " ns)";
}

/** @brief Formats a target word as a fixed-width hexadecimal address. */
std::string formatHexWord(const std::uint32_t value) {
    std::ostringstream text;
    text << "0x" << std::hex << std::setfill('0') << std::setw(8) << value;
    return text.str();
}

/** @brief One named internal JIT counter, reported only when nonzero. */
struct JitCounter {
    std::string_view name;
    std::uint64_t value;
};

/**
 * @brief Groups nonzero JIT counters for the opt-in `--jit-stats` report.
 *
 * @param cpu CPU whose counters are read.
 * @return Named group lines in stable order; groups without nonzero counters
 *         are omitted so short runs report almost nothing.
 */
std::vector<ReportLine> jitReportLines(const cpu::CortexM4& cpu) {
    const auto stats = cpu.jitStats();
    std::vector<ReportLine> groups;
    const auto add = [&groups](
        const std::string_view group,
        const std::span<const JitCounter> counters
    ) {
        std::string value;
        for (const JitCounter& counter : counters) {
            if (counter.value == 0U) continue;
            if (!value.empty()) value += ' ';
            value += counter.name;
            value += '=';
            value += std::to_string(counter.value);
        }
        if (!value.empty()) {
            groups.push_back(ReportLine{std::string(group), std::move(value)});
        }
    };
    add("micro", std::array{
        JitCounter{"compilations", stats.micro_compilations},
        JitCounter{"executions", stats.micro_executions},
        JitCounter{"instructions", stats.micro_instructions},
        JitCounter{"failures", stats.micro_compilation_failures},
        JitCounter{"cost_ns", stats.micro_compile_cost_ns},
        JitCounter{"admitted", stats.micro_admitted},
        JitCounter{"rejected", stats.micro_rejected},
        JitCounter{"admission_cost_ns", stats.micro_admission_cost_ns},
    });
    add("native", std::array{
        JitCounter{"compilations", stats.native_compilations},
        JitCounter{"executions", stats.native_executions},
        JitCounter{"instructions", stats.native_instructions},
        JitCounter{"declines", stats.native_declines},
    });
    add("trace", std::array{
        JitCounter{"compilations", stats.trace_compilations},
        JitCounter{"compiled_instructions", stats.trace_compiled_instructions},
        JitCounter{"executions", stats.trace_executions},
        JitCounter{"instructions", stats.trace_instructions},
        JitCounter{"guard_mismatches", stats.trace_guard_mismatches},
        JitCounter{"memory_declines", stats.trace_memory_declines},
    });
    add("horizon", std::array{
        JitCounter{"summaries", stats.trace_horizon_summaries},
        JitCounter{"instructions", stats.trace_horizon_instructions},
        JitCounter{"local_ops", stats.trace_horizon_local_ops},
        JitCounter{"shared", stats.trace_horizon_shared},
        JitCounter{"control_flow", stats.trace_horizon_control_flow},
        JitCounter{"zero_prefix", stats.trace_horizon_zero_prefix},
        JitCounter{"stack_local_ops", stats.trace_horizon_stack_local_ops},
        JitCounter{"stack_shared", stats.trace_horizon_stack_shared},
    });
    add("blocks", std::array{
        JitCounter{"executions", stats.block_executions},
        JitCounter{"instructions", stats.block_instructions},
        JitCounter{"compilations", stats.compilations},
        JitCounter{"fallbacks", stats.fallbacks},
        JitCounter{"decline_guard", stats.block_decline_guard},
        JitCounter{"decline_empty", stats.block_decline_empty},
        JitCounter{"prepare_failed", stats.block_prepare_failed},
        JitCounter{"span_ok", stats.block_native_span_ok},
    });
    add("spans", std::array{
        JitCounter{"skip_boundary", stats.span_skip_boundary},
        JitCounter{"skip_clock", stats.span_skip_clock},
        JitCounter{"skip_credit", stats.span_skip_credit},
        JitCounter{"skip_env", stats.span_skip_env},
        JitCounter{"skip_retire", stats.span_skip_retire},
    });
    add("budget", std::array{
        JitCounter{"fetch", stats.tb_fetch},
        JitCounter{"lower", stats.tb_lower},
        JitCounter{"profile", stats.tb_profile},
        JitCounter{"limit", stats.tb_limit},
        JitCounter{"ok", stats.tb_ok},
        JitCounter{"ops", stats.tb_ops},
    });
    add("single", std::array{
        JitCounter{"fast", stats.single_fast},
        JitCounter{"generic", stats.single_generic},
        JitCounter{"block_generic", stats.block_generic},
        JitCounter{"decline", stats.single_decline},
    });
    return groups;
}

ExitCode runBoardCommand(
    const std::span<const std::string_view> args,
    std::ostream& out,
    std::ostream& err
) {
    if (args.size() < 2) {
        err << "fil: run requires a config path\n";
        return ExitCode::usage_error;
    }
    std::optional<std::uint64_t> duration_ns;
    std::optional<std::uint64_t> max_instructions;
    std::optional<std::uint32_t> stop_address;
    std::optional<std::string> stop_symbol;
    std::optional<std::filesystem::path> trace_path;
    bool strict_mmio = false;
    bool trace_instructions = false;
    bool detect_spin = false;
    bool enable_jit = true;
    bool show_jit_stats = false;
    bool allow_breakpoint = false;
    std::uint32_t adc_decimation = 1U;

    for (std::size_t index = 2; index < args.size(); ++index) {
        const std::string_view option = args[index];
        if (option == "--duration" || option == "--instruction-budget"
            || option == "--stop-address" || option == "--adc-decimation") {
            const auto value = nextOptionValue(args, index);
            if (!value) {
                err << "fil: " << option << " requires a value\n";
                return ExitCode::usage_error;
            }
            if (option == "--duration") {
                auto parsed = parseDurationNs(option, *value);
                if (!parsed) {
                    err << "fil: " << parsed.error().message << '\n';
                    return ExitCode::usage_error;
                }
                duration_ns = parsed.value();
                continue;
            }
            auto parsed = config::parseUnsigned(*value);
            if (!parsed) {
                err << "fil: invalid value for " << option << ": " << parsed.error().message << '\n';
                return ExitCode::usage_error;
            }
            if (option == "--instruction-budget") max_instructions = parsed.value();
            else if (option == "--adc-decimation") {
                if (parsed.value() == 0U || parsed.value() > 1024U) {
                    err << "fil: --adc-decimation requires a factor in [1, 1024]\n";
                    return ExitCode::usage_error;
                }
                adc_decimation = static_cast<std::uint32_t>(parsed.value());
            }
            else {
                if (parsed.value() > std::numeric_limits<std::uint32_t>::max()) {
                    err << "fil: --stop-address exceeds 32-bit target address space\n";
                    return ExitCode::usage_error;
                }
                stop_address = static_cast<std::uint32_t>(parsed.value());
            }
        } else if (option == "--stop-at-symbol" || option == "--trace") {
            const auto value = nextOptionValue(args, index);
            if (!value) {
                err << "fil: " << option << " requires a value\n";
                return ExitCode::usage_error;
            }
            if (option == "--stop-at-symbol") stop_symbol = std::string(*value);
            else trace_path = std::filesystem::path(*value);
        } else if (option == "--strict-mmio") strict_mmio = true;
        else if (option == "--lenient-mmio") strict_mmio = false;
        else if (option == "--trace-instr") trace_instructions = true;
        else if (option == "--detect-spin") detect_spin = true;
        else if (option == "--no-detect-spin") detect_spin = false;
        else if (option == "--jit-stats") show_jit_stats = true;
        else if (option == "--allow-breakpoint") allow_breakpoint = true;
        else {
            err << "fil: unknown run option: " << option << '\n';
            return ExitCode::usage_error;
        }
    }

    auto board_config = config::loadBoardConfig(args[1]);
    if (!board_config) {
        err << "fil: " << formatError(board_config.error()) << '\n';
        return ExitCode::config_error;
    }
    auto board = sim::Board::load(board_config.value(), strict_mmio);
    if (!board) {
        err << "fil: " << formatError(board.error()) << '\n';
        return board.error().category == ErrorCategory::config ? ExitCode::config_error : ExitCode::runtime_error;
    }
    const bool diagnostics_enabled = trace_path.has_value();
    board.value()->trace().setEnabled(diagnostics_enabled);
    board.value()->peripherals().setAdcDiagnosticsEnabled(diagnostics_enabled);

    sim::BoardRunOptions options;
    options.max_instructions = max_instructions.value_or(board_config.value().run.max_instructions);
    if (options.max_instructions == 0U) {
        options.max_instructions = std::numeric_limits<std::uint64_t>::max();
    }
    std::uint64_t resolved_duration_ns = 0U;
    if (duration_ns) {
        resolved_duration_ns = duration_ns.value();
    } else if (board_config.value().run.default_duration_ms != 0U) {
        if (board_config.value().run.default_duration_ms
            > std::numeric_limits<std::uint64_t>::max() / 1'000'000ULL) {
            err << "fil: configured duration is too large\n";
            return ExitCode::usage_error;
        }
        resolved_duration_ns = board_config.value().run.default_duration_ms * 1'000'000ULL;
    }
    options.duration_ns = resolved_duration_ns;
    options.trace_instructions = trace_instructions;
    options.detect_spin = detect_spin;
    options.enable_jit = enable_jit;
    options.adc_decimation = adc_decimation;
    options.stop_address = stop_address;
    if (stop_symbol) {
        const auto& symbols = board.value()->image().symbols();
        const auto match = std::find_if(
            symbols.begin(), symbols.end(),
            [&](const elf::ElfSymbol& symbol) { return symbol.name == *stop_symbol; }
        );
        if (match == symbols.end()) {
            err << "fil: symbol not found: " << *stop_symbol << '\n';
            return ExitCode::usage_error;
        }
        options.stop_address = match->address;
    }

    const sim::BoardRunResult result = board.value()->run(options);
    const auto unknown = board.value()->peripherals().router().unknownAccesses();
    const std::vector<ReportLine> jit_groups = show_jit_stats
        ? jitReportLines(board.value()->cpu()) : std::vector<ReportLine>{};
    std::vector<ReportLine> summary;
    summary.push_back({"stop", std::string(sim::boardStopReasonName(result.reason))});
    if (!result.message.empty()) summary.push_back({"message", result.message});
    summary.push_back({"instructions", std::to_string(result.instructions)});
    summary.push_back({"cycles", std::to_string(result.cycles)});
    summary.push_back({"time", formatSimTime(result.time_ns)});
    summary.push_back({"pc", formatHexWord(result.diagnostic.next_pc)});
    if (!result.succeeded()) {
        const auto& state = board.value()->cpu().state();
        summary.push_back({"msp", formatHexWord(state.msp)});
        summary.push_back({"psp", formatHexWord(state.psp)});
        summary.push_back({"lr", formatHexWord(state.r[14])});
        summary.push_back({"xpsr", formatHexWord(state.xpsr)});
    }
    summary.push_back({"unknown_mmio", std::to_string(unknown.size())});
    if (show_jit_stats) {
        summary.push_back({"jit", jit_groups.empty() ? "no recorded counters" : ""});
    }
    out << "board: " << board_config.value().name << '\n';
    printReportLines(out, 2U, summary);
    if (!jit_groups.empty()) printReportLines(out, 4U, jit_groups);
    for (const auto& access : unknown) {
        out << "    " << formatHexWord(access.address)
            << "  reads=" << access.reads << " writes=" << access.writes << '\n';
    }
    if (trace_path) {
        std::ofstream trace(*trace_path, std::ios::binary | std::ios::trunc);
        if (!trace) {
            err << "fil: unable to open trace output: " << trace_path->string() << '\n';
            return ExitCode::runtime_error;
        }
        board.value()->trace().writeJsonLines(trace);
        if (!trace) {
            err << "fil: failed while writing trace output\n";
            return ExitCode::runtime_error;
        }
    }
    const bool unexpected_breakpoint =
        result.reason == sim::BoardStopReason::breakpoint && !allow_breakpoint;
    if (!result.succeeded() || unexpected_breakpoint) {
        err << "fil: run stopped with " << sim::boardStopReasonName(result.reason) << '\n';
        return ExitCode::runtime_error;
    }
    return ExitCode::success;
}

ExitCode compareStlinkCommand(
    const std::span<const std::string_view> args,
    std::ostream& out,
    std::ostream& err
) {
    if (args.size() < 2U) {
        err << "fil: compare-stlink requires a board config path\n";
        return ExitCode::usage_error;
    }

    std::optional<std::uint64_t> max_instructions;
    std::optional<std::uint32_t> stop_address;
    std::optional<std::string> stop_symbol;
    std::optional<std::string> serial;
    std::optional<std::filesystem::path> artifacts;
    std::filesystem::path openocd{"openocd"};
    std::chrono::milliseconds timeout{10'000};
    std::vector<hardware::MemoryRange> memory_ranges;
    std::vector<std::string> explicit_registers;
    std::vector<std::string> ignored_registers;
    bool strict_mmio = false;
    bool flash = false;

    for (std::size_t index = 2U; index < args.size(); ++index) {
        const std::string_view option = args[index];
        if (option == "--flash") {
            flash = true;
        } else if (option == "--strict-mmio") {
            strict_mmio = true;
        } else if (option == "--lenient-mmio") {
            strict_mmio = false;
        } else if (option == "--max-instructions" || option == "--stop-address"
                   || option == "--timeout-ms") {
            const auto value = nextOptionValue(args, index);
            if (!value) {
                err << "fil: " << option << " requires a value\n";
                return ExitCode::usage_error;
            }
            const auto parsed = config::parseUnsigned(*value);
            if (!parsed) {
                err << "fil: invalid value for " << option << ": "
                    << parsed.error().message << '\n';
                return ExitCode::usage_error;
            }
            if (option == "--max-instructions") {
                max_instructions = parsed.value();
            } else if (option == "--stop-address") {
                if (parsed.value() > std::numeric_limits<std::uint32_t>::max()) {
                    err << "fil: --stop-address exceeds 32-bit target address space\n";
                    return ExitCode::usage_error;
                }
                stop_address = static_cast<std::uint32_t>(parsed.value());
                if ((*stop_address & 1U) != 0U) {
                    err << "fil: --stop-address must be halfword aligned\n";
                    return ExitCode::usage_error;
                }
            } else {
                if (parsed.value() == 0U
                    || parsed.value() > static_cast<std::uint64_t>(
                        std::numeric_limits<std::chrono::milliseconds::rep>::max()
                    )) {
                    err << "fil: --timeout-ms is outside the supported range\n";
                    return ExitCode::usage_error;
                }
                timeout = std::chrono::milliseconds(
                    static_cast<std::chrono::milliseconds::rep>(parsed.value())
                );
            }
        } else if (option == "--memory") {
            const auto value = nextOptionValue(args, index);
            if (!value) {
                err << "fil: --memory requires ADDRESS:LENGTH\n";
                return ExitCode::usage_error;
            }
            auto parsed = hardware::parseMemoryRange(*value);
            if (!parsed) {
                err << "fil: invalid --memory value: " << parsed.error().message << '\n';
                return ExitCode::usage_error;
            }
            memory_ranges.push_back(parsed.value());
        } else if (option == "--stop-at-symbol" || option == "--serial"
                   || option == "--openocd" || option == "--artifacts"
                   || option == "--register" || option == "--ignore-register") {
            const auto value = nextOptionValue(args, index);
            if (!value) {
                err << "fil: " << option << " requires a value\n";
                return ExitCode::usage_error;
            }
            if (option == "--stop-at-symbol") stop_symbol = std::string(*value);
            else if (option == "--serial") serial = std::string(*value);
            else if (option == "--openocd") openocd = std::filesystem::path(*value);
            else if (option == "--artifacts") artifacts = std::filesystem::path(*value);
            else if (option == "--register") explicit_registers.emplace_back(*value);
            else ignored_registers.emplace_back(*value);
        } else {
            err << "fil: unknown compare-stlink option: " << option << '\n';
            return ExitCode::usage_error;
        }
    }
    if (stop_address && stop_symbol) {
        err << "fil: --stop-address and --stop-at-symbol are mutually exclusive\n";
        return ExitCode::usage_error;
    }

    auto board_config = config::loadBoardConfig(args[1]);
    if (!board_config) {
        err << "fil: " << formatError(board_config.error()) << '\n';
        return ExitCode::config_error;
    }
    auto board = sim::Board::load(board_config.value(), strict_mmio);
    if (!board) {
        err << "fil: " << formatError(board.error()) << '\n';
        return board.error().category == ErrorCategory::config
            ? ExitCode::config_error : ExitCode::runtime_error;
    }

    if (stop_symbol) {
        const auto match = std::find_if(
            board.value()->image().symbols().begin(), board.value()->image().symbols().end(),
            [&](const elf::ElfSymbol& symbol) { return symbol.name == *stop_symbol; }
        );
        if (match == board.value()->image().symbols().end()) {
            err << "fil: symbol not found: " << *stop_symbol << '\n';
            return ExitCode::usage_error;
        }
        stop_address = match->address & ~1U;
    }

    std::vector<std::string> selected_registers;
    if (explicit_registers.empty()) {
        for (const std::string_view name : hardware::comparableRegisterNames()) {
            selected_registers.emplace_back(name);
        }
    } else {
        selected_registers = explicit_registers;
    }
    const auto knownRegister = [](const std::string_view name) {
        const auto names = hardware::comparableRegisterNames();
        return std::find(names.begin(), names.end(), name) != names.end();
    };
    for (const std::string& name : selected_registers) {
        if (!knownRegister(name)) {
            err << "fil: unknown comparison register: " << name << '\n';
            return ExitCode::usage_error;
        }
    }
    for (const std::string& name : ignored_registers) {
        if (!knownRegister(name)) {
            err << "fil: unknown ignored register: " << name << '\n';
            return ExitCode::usage_error;
        }
        std::erase(selected_registers, name);
    }
    std::vector<std::string> unique_registers;
    unique_registers.reserve(selected_registers.size());
    for (const std::string& name : selected_registers) {
        if (std::find(unique_registers.begin(), unique_registers.end(), name)
            == unique_registers.end()) {
            unique_registers.push_back(name);
        }
    }
    selected_registers = std::move(unique_registers);

    sim::BoardRunOptions run_options;
    run_options.max_instructions = max_instructions.value_or(
        board_config.value().run.max_instructions
    );
    if (run_options.max_instructions == 0U) {
        run_options.max_instructions = std::numeric_limits<std::uint64_t>::max();
    }
    run_options.duration_ns = 0U;
    run_options.stop_address = stop_address;
    run_options.detect_spin = false;
    const sim::BoardRunResult run_result = board.value()->run(run_options);
    const sim::BoardStopReason expected_stop = stop_address
        ? sim::BoardStopReason::target_reached : sim::BoardStopReason::breakpoint;
    if (run_result.reason != expected_stop) {
        err << "fil: emulator stopped with " << sim::boardStopReasonName(run_result.reason)
            << " before the comparison boundary\n";
        return ExitCode::runtime_error;
    }
    auto emulator = hardware::captureEmulator(*board.value(), run_result, memory_ranges);
    if (!emulator) {
        err << "fil: " << formatError(emulator.error()) << '\n';
        return ExitCode::runtime_error;
    }

    hardware::StlinkCaptureOptions capture_options;
    capture_options.openocd = openocd;
    capture_options.serial = serial;
    capture_options.stop_address = stop_address;
    capture_options.timeout = timeout;
    capture_options.flash = flash;
    auto target = hardware::captureStlink(
        board_config.value().elf_path, memory_ranges, capture_options
    );
    if (!target) {
        err << "fil: " << formatError(target.error()) << '\n';
        return ExitCode::runtime_error;
    }

    const hardware::Comparison comparison = hardware::compare(
        emulator.value(), target.value(), selected_registers
    );
    hardware::writeJson(comparison, out);

    if (artifacts) {
        std::error_code directory_error;
        std::filesystem::create_directories(*artifacts, directory_error);
        if (directory_error) {
            err << "fil: unable to create artifact directory: "
                << directory_error.message() << '\n';
            return ExitCode::runtime_error;
        }
        const auto writeArtifact = [&](
            const std::string_view name,
            const auto& value
        ) -> bool {
            std::ofstream file(*artifacts / std::string(name), std::ios::binary | std::ios::trunc);
            if (!file) return false;
            hardware::writeJson(value, file);
            return static_cast<bool>(file);
        };
        if (!writeArtifact("emulator.json", emulator.value())
            || !writeArtifact("hardware.json", target.value())
            || !writeArtifact("comparison.json", comparison)) {
            err << "fil: unable to write comparison artifacts\n";
            return ExitCode::runtime_error;
        }
    }

    if (!comparison.matches()) {
        err << "fil: hardware state differs from emulator state\n";
        return ExitCode::runtime_error;
    }
    return ExitCode::success;
}

class ScopedTraceObserverReset {
public:
    explicit ScopedTraceObserverReset(sim::TraceRecorder& trace) : trace_(&trace) {}
    ~ScopedTraceObserverReset() { trace_->setObserver({}); }
    ScopedTraceObserverReset(const ScopedTraceObserverReset&) = delete;
    ScopedTraceObserverReset& operator=(const ScopedTraceObserverReset&) = delete;

private:
    sim::TraceRecorder* trace_;
};

/** @brief Types that must be recorded to evaluate the given expectation checks. */
std::vector<std::string> expectationRecordTypes(
    const std::vector<sim::ExpectationCheck>& expectations
) {
    std::vector<std::string> types{
        "expectation_pending", "expectation_pass", "expectation_fail", "expectation_incomplete",
    };
    const auto add = [&](const std::string_view type) {
        if (std::find(types.begin(), types.end(), type) == types.end()) types.emplace_back(type);
    };
    for (const auto& check : expectations) {
        std::visit([&](const auto& specific) {
            using Specific = std::decay_t<decltype(specific)>;
            add(std::is_same_v<Specific, sim::CanExpectation> ? "can_tx" : "gpio_output");
        }, check.expectation);
    }
    return types;
}

/** @brief One-line human-readable name for an expectation check's target. */
std::string describeExpectation(const sim::Expectation& expectation) {
    return std::visit([](const auto& specific) -> std::string {
        using Specific = std::decay_t<decltype(specific)>;
        std::ostringstream text;
        if constexpr (std::is_same_v<Specific, sim::CanExpectation>) {
            text << "CAN " << specific.bus << " id=0x" << std::hex << specific.id;
            if (!specific.data.empty()) {
                text << " data=";
                for (const std::uint8_t byte : specific.data) {
                    text << std::hex << std::setfill('0') << std::setw(2)
                        << static_cast<unsigned>(byte);
                }
            }
        } else {
            text << "GPIO " << specific.board << '.' << specific.port << " pin "
                 << static_cast<unsigned>(specific.pin) << " level "
                 << (specific.level ? "1" : "0");
        }
        return text.str();
    }, expectation);
}

/** @brief One end-of-run expectation line, ready for aligned printing. */
struct ExpectationRow {
    std::string marker;      ///< Status marker such as `[FAIL]`.
    std::string status;      ///< Raw evaluator status used for ordering/headers.
    std::string script;      ///< Stimulus script name.
    std::string description; ///< Expected CAN frame or GPIO transition.
    std::string window;      ///< Inclusive window as `[start, end] ms`.
};

/** @brief Ranks statuses so failures print first in the expectation report. */
int expectationRank(const std::string_view status) noexcept {
    if (status == "fail") return 0;
    if (status == "incomplete") return 1;
    if (status == "pass") return 2;
    return 3;
}

/** @brief Builds the expectation report rows, most severe status first. */
std::vector<ExpectationRow> buildExpectationRows(
    const std::vector<sim::ExpectationCheck>& expectations,
    const sim::ExpectationEvaluator& evaluator
) {
    std::vector<ExpectationRow> rows;
    rows.reserve(expectations.size());
    for (std::size_t index = 0; index < expectations.size(); ++index) {
        const sim::ExpectationCheck& check = expectations[index];
        std::string status(evaluator.status(index));
        std::ostringstream window;
        const std::uint64_t at_ms = sim::expectationStartMs(check.expectation);
        window << '[' << at_ms << ", "
               << (at_ms + sim::expectationWindowMs(check.expectation)) << "] ms";
        std::string marker;
        if (status == "fail") marker = "[FAIL]";
        else if (status == "incomplete") marker = "[INCOMPLETE]";
        else if (status == "pass") marker = "[pass]";
        else marker = "[" + status + "]";
        rows.push_back(ExpectationRow{
            std::move(marker), std::move(status), check.script,
            describeExpectation(check.expectation), window.str()
        });
    }
    std::stable_sort(rows.begin(), rows.end(),
        [](const ExpectationRow& left, const ExpectationRow& right) {
            return expectationRank(left.status) < expectationRank(right.status);
        });
    return rows;
}

/** @brief Counts expectation outcomes, capitalizing any failures. */
std::string summarizeExpectationRows(const std::vector<ExpectationRow>& rows) {
    std::size_t failed = 0U, incomplete = 0U, passed = 0U, other = 0U;
    for (const ExpectationRow& row : rows) {
        if (row.status == "fail") ++failed;
        else if (row.status == "incomplete") ++incomplete;
        else if (row.status == "pass") ++passed;
        else ++other;
    }
    std::vector<std::string> parts;
    if (failed != 0U) parts.push_back(std::to_string(failed) + " FAILED");
    if (incomplete != 0U) parts.push_back(std::to_string(incomplete) + " incomplete");
    if (passed != 0U) parts.push_back(std::to_string(passed) + " passed");
    if (other != 0U) parts.push_back(std::to_string(other) + " pending");
    std::string summary;
    for (const std::string& part : parts) {
        if (!summary.empty()) summary += ", ";
        summary += part;
    }
    return summary;
}

/** @brief Prints expectation rows in aligned columns, failures first. */
void printExpectationRows(std::ostream& out, const std::vector<ExpectationRow>& rows) {
    std::size_t marker_width = 0U, script_width = 0U, description_width = 0U;
    for (const ExpectationRow& row : rows) {
        marker_width = std::max(marker_width, row.marker.size());
        script_width = std::max(script_width, row.script.size());
        description_width = std::max(description_width, row.description.size());
    }
    const auto padded = [](const std::string& text, const std::size_t width) {
        return text + std::string(width - text.size(), ' ');
    };
    for (const ExpectationRow& row : rows) {
        out << "    " << padded(row.marker, marker_width) << "  "
            << padded(row.script, script_width) << "  "
            << padded(row.description, description_width) << "  "
            << row.window << '\n';
    }
}

Result<void> scheduleConfiguredStimuli(
    const config::NetworkConfig& network,
    sim::World& world,
    std::vector<sim::ExpectationCheck>& expectations
) {
    for (std::size_t stimulus_index = 0; stimulus_index < network.stimulus_paths.size(); ++stimulus_index) {
        const std::filesystem::path& path = network.stimulus_paths[stimulus_index];
        auto script = sim::loadStimulusScript(path);
        if (!script) return script.error();
        auto scheduled = sim::scheduleStimulusScript(script.value(), world);
        if (!scheduled) return scheduled.error();
        for (std::size_t index = 0; index < script.value().expect.size(); ++index) {
            expectations.push_back(sim::ExpectationCheck{
                script.value().name, index, script.value().expect[index], stimulus_index
            });
        }
    }
    return {};
}

/**
 * @brief Loads a network configuration, accepting one board as a one-board network.
 *
 * `watch` and `serve` operate on a network. Passing a board config wraps it in
 * an anonymous network that reuses that board's declared CAN buses.
 *
 * @param path Network or board configuration path.
 * @return Loaded network configuration, or a source-aware error.
 */
Result<config::NetworkConfig> loadNetworkView(const std::filesystem::path& path) {
    auto network = config::loadNetworkConfig(path);
    if (network) return network;

    auto board = config::loadBoardConfig(path);
    if (!board) return network.error();

    config::NetworkConfig synthesized;
    synthesized.name = board.value().name;
    synthesized.source_path = board.value().source_path;
    for (const config::CanControllerConfig& controller : board.value().can) {
        const bool seen = std::any_of(
            synthesized.buses.begin(), synthesized.buses.end(),
            [&](const config::CanBusConfig& bus) { return bus.name == controller.bus; }
        );
        if (seen) continue;
        config::CanBusConfig bus;
        bus.name = controller.bus;
        synthesized.buses.push_back(std::move(bus));
    }
    synthesized.board_paths.push_back(board.value().source_path);
    return synthesized;
}

ExitCode runNetworkCommand(
    const std::span<const std::string_view> args,
    std::ostream& out,
    std::ostream& err
) {
    if (args.size() < 2) {
        err << "fil: run requires a config path\n";
        return ExitCode::usage_error;
    }

    std::optional<std::uint64_t> duration_ns;
    std::uint64_t max_instructions = std::numeric_limits<std::uint64_t>::max();
    std::uint64_t quantum = 1'024U;
    sim::SimTimeNs cross_board_window_ns = sim::WorldRunOptions{}.cross_board_window_ns;
    bool enable_parallel_lanes = sim::WorldRunOptions{}.enable_parallel_lanes;
    std::optional<std::filesystem::path> trace_path;
    bool strict_mmio = false;
    bool trace_instructions = false;
    bool detect_spin = false;
    bool enable_jit = true;
    bool show_jit_stats = false;
    bool enable_ram_capsules = true;
    bool allow_breakpoint = false;
    std::uint32_t adc_decimation = 1U;
    std::vector<PendingCanInjection> injections;

    for (std::size_t index = 2; index < args.size(); ++index) {
        const std::string_view option = args[index];
        if (option == "--duration") {
            const auto value = nextOptionValue(args, index);
            if (!value) {
                err << "fil: " << option << " requires a value\n";
                return ExitCode::usage_error;
            }
            auto parsed = parseDurationNs(option, *value);
            if (!parsed) {
                err << "fil: " << parsed.error().message << '\n';
                return ExitCode::usage_error;
            }
            duration_ns = parsed.value();
        } else if (option == "--instruction-budget" || option == "--quantum"
                   || option == "--adc-decimation" || option == "--cross-board-window-ns") {
            const auto value = nextOptionValue(args, index);
            if (!value) {
                err << "fil: " << option << " requires a value\n";
                return ExitCode::usage_error;
            }
            auto parsed = config::parseUnsigned(*value);
            if (!parsed) {
                err << "fil: invalid value for " << option << ": " << parsed.error().message << '\n';
                return ExitCode::usage_error;
            }
            if (option == "--instruction-budget") max_instructions = parsed.value();
            else if (option == "--adc-decimation") {
                if (parsed.value() == 0U || parsed.value() > 1024U) {
                    err << "fil: --adc-decimation requires a factor in [1, 1024]\n";
                    return ExitCode::usage_error;
                }
                adc_decimation = static_cast<std::uint32_t>(parsed.value());
            }
            else if (option == "--cross-board-window-ns") cross_board_window_ns = parsed.value();
            else quantum = parsed.value();
        } else if (option == "--no-parallel-lanes") {
            enable_parallel_lanes = false;
        } else if (option == "--parallel-lanes") {
            enable_parallel_lanes = true;
        } else if (option == "--trace" || option == "--inject-can") {
            const auto value = nextOptionValue(args, index);
            if (!value) {
                err << "fil: " << option << " requires a value\n";
                return ExitCode::usage_error;
            }
            if (option == "--trace") {
                trace_path = std::filesystem::path(*value);
            } else {
                auto injection = parseCanInjection(*value);
                if (!injection) {
                    err << "fil: invalid --inject-can value: " << injection.error().message << '\n';
                    return ExitCode::usage_error;
                }
                injections.push_back(std::move(injection).value());
            }
        } else if (option == "--strict-mmio") strict_mmio = true;
        else if (option == "--lenient-mmio") strict_mmio = false;
        else if (option == "--trace-instr") trace_instructions = true;
        else if (option == "--detect-spin") detect_spin = true;
        else if (option == "--no-detect-spin") detect_spin = false;
        else if (option == "--jit-stats") show_jit_stats = true;
        else if (option == "--allow-breakpoint") allow_breakpoint = true;
        else {
            err << "fil: unknown run option: " << option << '\n';
            return ExitCode::usage_error;
        }
    }

    auto network_config = loadNetworkView(args[1]);
    if (!network_config) {
        err << "fil: " << formatError(network_config.error()) << '\n';
        return ExitCode::config_error;
    }
    auto world = sim::World::load(network_config.value(), strict_mmio);
    if (!world) {
        err << "fil: " << formatError(world.error()) << '\n';
        return world.error().category == ErrorCategory::config
            ? ExitCode::config_error : ExitCode::runtime_error;
    }
    std::vector<sim::ExpectationCheck> expectations;
    auto scheduled_stimuli = scheduleConfiguredStimuli(network_config.value(), *world.value(), expectations);
    if (!scheduled_stimuli) {
        err << "fil: " << formatError(scheduled_stimuli.error()) << '\n';
        return ExitCode::config_error;
    }
    // Expectations need trace recording even without a requested trace artifact.
    // An expectation-only run selects just the records the evaluator consumes;
    // lane-local records (for example gpio_output) are buffered per lane and
    // merged on the coordinator, so the parallel scheduler stays eligible.
    const bool expectation_only_trace = !expectations.empty() && !trace_path.has_value();
    world.value()->setDiagnosticsEnabled(trace_path.has_value());
    world.value()->trace().setEnabled(trace_path.has_value() || !expectations.empty());
    if (expectation_only_trace) {
        world.value()->trace().setTypeAllowlist(expectationRecordTypes(expectations));
        world.value()->trace().setRetainRecords(false);
    }
    sim::ExpectationEvaluator expectation_evaluator(world.value()->trace(), expectations);
    ScopedTraceObserverReset observer_reset(world.value()->trace());
    if (!expectations.empty()) {
        world.value()->trace().setObserver([&expectation_evaluator](const sim::TraceRecord& record) {
            expectation_evaluator.observe(record);
        });
    }

    for (const PendingCanInjection& injection : injections) {
        devices::VirtualCanBus* bus = world.value()->canBus(injection.bus);
        if (bus == nullptr) {
            err << "fil: CAN injection names undeclared bus: " << injection.bus << '\n';
            return ExitCode::usage_error;
        }
        if (injection.at_ms > std::numeric_limits<std::uint64_t>::max() / 1'000'000ULL) {
            err << "fil: CAN injection time is too large\n";
            return ExitCode::usage_error;
        }
        sim::EventLoop* loop = &world.value()->eventLoop();
        static_cast<void>(loop->scheduleAfter(
            injection.at_ms * 1'000'000ULL,
            [bus, loop, frame = injection.frame]() {
                static_cast<void>(bus->inject(frame, loop->now()));
            }
        ));
    }

    if (!expectations.empty()) expectation_evaluator.begin(
        world.value()->eventLoop().now(), world.value()->eventLoop()
    );

    sim::WorldRunOptions options;
    options.duration_ns = duration_ns.value_or(0U);
    if (max_instructions == 0U) max_instructions = std::numeric_limits<std::uint64_t>::max();
    options.max_instructions_per_board = max_instructions;
    options.instruction_quantum = quantum;
    options.cross_board_window_ns = cross_board_window_ns;
    options.enable_parallel_lanes = enable_parallel_lanes;
    options.trace_instructions = trace_instructions;
    options.detect_spin = detect_spin;
    options.enable_jit = enable_jit;
    options.enable_ram_capsules = enable_ram_capsules;
    options.adc_decimation = adc_decimation;
    auto result = world.value()->run(options);
    if (!result) {
        if (!expectations.empty()) expectation_evaluator.finish(world.value()->eventLoop().now());
        err << "fil: " << formatError(result.error()) << '\n';
        return result.error().category == ErrorCategory::invalid_argument
            ? ExitCode::usage_error : ExitCode::runtime_error;
    }

    if (!expectations.empty()) expectation_evaluator.finish(result.value().end_time_ns);

    std::vector<ReportLine> summary;
    summary.push_back({"stop", std::string(sim::worldStopReasonName(result.value().reason))});
    if (!result.value().message.empty()) summary.push_back({"message", result.value().message});
    summary.push_back({"boards", std::to_string(result.value().boards.size())});
    summary.push_back({"instructions", std::to_string(result.value().instructions)});
    summary.push_back({"cycles", std::to_string(result.value().cycles)});
    summary.push_back({"time", formatSimTime(result.value().end_time_ns)});
    summary.push_back({"rounds", std::to_string(result.value().rounds)});
    summary.push_back({"parallel_lanes", std::to_string(result.value().parallel_lanes)});
    summary.push_back({"cross_board_window_ns", std::to_string(trace_instructions || detect_spin
        ? 0U : cross_board_window_ns)});
    summary.push_back({"dispatches", std::to_string(result.value().dispatches)});
    summary.push_back({"exact_dispatches", std::to_string(result.value().exact_dispatches)});
    summary.push_back({"event_callbacks", std::to_string(result.value().event_callbacks)});
    summary.push_back({"lockstep_bursts", std::to_string(result.value().lockstep_bursts)});
    if (enable_ram_capsules
        && (result.value().deferred_prefixes != 0U || result.value().memoized_prefixes != 0U
            || result.value().memoized_instructions != 0U
            || result.value().deferred_instructions != 0U
            || result.value().deferred_truncations != 0U)) {
        summary.push_back({"deferred_prefixes", std::to_string(result.value().deferred_prefixes)});
        summary.push_back({"memoized_prefixes", std::to_string(result.value().memoized_prefixes)});
        summary.push_back({"memoized_instructions", std::to_string(result.value().memoized_instructions)});
        summary.push_back({"deferred_instructions", std::to_string(result.value().deferred_instructions)});
        summary.push_back({"deferred_truncations", std::to_string(result.value().deferred_truncations)});
    }
    std::vector<ExpectationRow> expectation_rows = buildExpectationRows(
        expectations, expectation_evaluator
    );
    if (!expectation_rows.empty()) {
        summary.push_back({"expectations", summarizeExpectationRows(expectation_rows)});
    }
    out << "network: " << network_config.value().name << '\n';
    printReportLines(out, 2U, summary);

    const bool unmet_expectation = expectation_evaluator.hasFailures();
    for (std::size_t index = 0; index < expectations.size(); ++index) {
        const auto& check = expectations[index];
        if (expectation_evaluator.status(index) == "fail"
            || expectation_evaluator.status(index) == "incomplete") {
            const std::string_view kind = std::holds_alternative<sim::CanExpectation>(check.expectation)
                ? "CAN" : "GPIO";
            err << "fil: unmet " << kind << " output expectation in stimulus '"
                << check.script << "': " << describeExpectation(check.expectation) << '\n';
        }
    }
    if (!expectation_rows.empty()) {
        out << '\n';
        printExpectationRows(out, expectation_rows);
    }
    for (const auto& board : result.value().boards) {
        out << '\n' << "  board: " << board.name << '\n';
        std::vector<ReportLine> fields;
        fields.push_back({"stop", std::string(sim::boardStopReasonName(board.result.reason))});
        if (!board.result.message.empty() && !board.result.succeeded()) {
            fields.push_back({"message", board.result.message});
        }
        fields.push_back({"instructions", std::to_string(board.result.instructions)});
        fields.push_back({"pc", formatHexWord(board.result.diagnostic.next_pc)});
        if (enable_ram_capsules
            && (board.memoized_prefixes != 0U || board.memoized_instructions != 0U)) {
            fields.push_back({"memoized_ram", "prefixes=" + std::to_string(board.memoized_prefixes)
                + " admitted_instructions=" + std::to_string(board.memoized_instructions)});
        }
        sim::Board* instance = world.value()->board(board.name);
        const std::vector<ReportLine> jit_groups = show_jit_stats && instance != nullptr
            ? jitReportLines(instance->cpu()) : std::vector<ReportLine>{};
        if (instance != nullptr) {
            fields.push_back({"clock", std::to_string(
                instance->peripherals().rcc().systemClockHz()) + " Hz"});
        }
        if (show_jit_stats && instance != nullptr) {
            fields.push_back({"jit", jit_groups.empty() ? "no recorded counters" : ""});
        }
        printReportLines(out, 4U, fields);
        if (!jit_groups.empty()) printReportLines(out, 6U, jit_groups);
        if (instance != nullptr) {
            const auto unknown = instance->peripherals().router().unknownAccesses();
            if (!unknown.empty()) {
                out << "    unknown_mmio: " << unknown.size() << '\n';
                for (const auto& access : unknown) {
                    out << "      " << formatHexWord(access.address)
                        << "  reads=" << access.reads << " writes=" << access.writes << '\n';
                }
            }
        }
    }

    if (trace_path) {
        std::ofstream trace(*trace_path, std::ios::binary | std::ios::trunc);
        if (!trace) {
            err << "fil: unable to open trace output: " << trace_path->string() << '\n';
            return ExitCode::runtime_error;
        }
        world.value()->trace().writeJsonLines(trace);
        if (!trace) {
            err << "fil: failed while writing trace output\n";
            return ExitCode::runtime_error;
        }
    }
    bool unexpected_breakpoint = false;
    if (!allow_breakpoint) {
        for (const auto& board : result.value().boards) {
            if (board.result.reason == sim::BoardStopReason::breakpoint) {
                unexpected_breakpoint = true;
                break;
            }
        }
    }
    if (!result.value().succeeded() || unexpected_breakpoint || unmet_expectation) {
        err << "fil: network stopped with " << sim::worldStopReasonName(result.value().reason) << '\n';
        if (unexpected_breakpoint) err << "fil: a board reached an unrequested breakpoint\n";
        return ExitCode::runtime_error;
    }
    return ExitCode::success;
}

ExitCode networkMonitorCommand(
    const std::span<const std::string_view> args,
    std::ostream& out,
    std::ostream& err,
    const bool binary_protocol
) {
    const std::string_view command_name = binary_protocol ? "serve" : "watch";
    if (args.size() < 2U) {
        err << "fil: " << command_name << " requires a network config path\n";
        return ExitCode::usage_error;
    }

    std::uint64_t max_instructions = std::numeric_limits<std::uint64_t>::max();
    std::uint64_t quantum = 1'024U;
    sim::SimTimeNs cross_board_window_ns = sim::WorldRunOptions{}.cross_board_window_ns;
    bool enable_parallel_lanes = sim::WorldRunOptions{}.enable_parallel_lanes;
    std::optional<std::uint64_t> duration_ns;
    sim::SimTimeNs refresh_ns = 100'000U;
    std::uint32_t adc_decimation = 1U;
    bool strict_mmio = false;
    bool enable_jit = true;
    bool enable_ram_capsules = true;
    bool trace_instructions = false;
    bool detect_spin = false;
    bool wall_pacing = true;
    bool custom_live_filters = false;
    std::vector<std::string> live_filters{"can_tx"};

    for (std::size_t index = 2U; index < args.size(); ++index) {
        const std::string_view option = args[index];
        if (option == "--duration") {
            const auto value = nextOptionValue(args, index);
            if (!value) {
                err << "fil: " << option << " requires a value\n";
                return ExitCode::usage_error;
            }
            auto parsed = parseDurationNs(option, *value);
            if (!parsed) {
                err << "fil: " << parsed.error().message << '\n';
                return ExitCode::usage_error;
            }
            // Zero selects an unlimited simulated-time run.
            duration_ns = parsed.value() == 0U
                ? std::nullopt
                : std::optional<std::uint64_t>{parsed.value()};
        } else if (option == "--instruction-budget" || option == "--quantum"
            || option == "--refresh-ms" || option == "--refresh-us"
            || option == "--adc-decimation" || option == "--cross-board-window-ns") {
            const auto value = nextOptionValue(args, index);
            if (!value) {
                err << "fil: " << option << " requires a value\n";
                return ExitCode::usage_error;
            }
            const auto parsed = config::parseUnsigned(*value);
            if (!parsed) {
                err << "fil: invalid value for " << option << ": "
                    << parsed.error().message << '\n';
                return ExitCode::usage_error;
            }
            if (option == "--instruction-budget") max_instructions = parsed.value();
            else if (option == "--quantum") quantum = parsed.value();
            else if (option == "--cross-board-window-ns") cross_board_window_ns = parsed.value();
            else if (option == "--adc-decimation") {
                if (parsed.value() < 1U || parsed.value() > 1024U) {
                    err << "fil: --adc-decimation requires a factor in [1, 1024]\n";
                    return ExitCode::usage_error;
                }
                adc_decimation = static_cast<std::uint32_t>(parsed.value());
            } else {
                if (parsed.value() == 0U
                    || parsed.value() > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())) {
                    err << "fil: " << option << " must be between 1 and "
                        << std::numeric_limits<std::int32_t>::max() << '\n';
                    return ExitCode::usage_error;
                }
                refresh_ns = parsed.value() * (option == "--refresh-us" ? 1'000ULL : 1'000'000ULL);
            }
        } else if (option == "--live-filter" || option == "--trace-type") {
            const auto value = nextOptionValue(args, index);
            if (!value || value->empty()) {
                err << "fil: --live-filter requires a value\n";
                return ExitCode::usage_error;
            }
            if (!custom_live_filters) {
                live_filters.clear();
                custom_live_filters = true;
            }
            live_filters.emplace_back(*value);
        } else if (option == "--strict-mmio") strict_mmio = true;
        else if (option == "--lenient-mmio") strict_mmio = false;
        else if (option == "--no-wall-pacing") wall_pacing = false;
        else if (option == "--parallel-lanes") enable_parallel_lanes = true;
        else if (option == "--no-parallel-lanes") enable_parallel_lanes = false;
        else if (option == "--trace-instr") trace_instructions = true;
        else if (option == "--detect-spin") detect_spin = true;
        else if (option != "--control-stdin") {
            err << "fil: unknown " << command_name << " option: " << option << '\n';
            return ExitCode::usage_error;
        }
    }
    if (max_instructions == 0U) {
        max_instructions = std::numeric_limits<std::uint64_t>::max();
    }

    auto network_config = loadNetworkView(args[1]);
    if (!network_config) {
        err << "fil: " << formatError(network_config.error()) << '\n';
        return ExitCode::config_error;
    }
    auto world = sim::World::load(network_config.value(), strict_mmio);
    if (!world) {
        err << "fil: " << formatError(world.error()) << '\n';
        return world.error().category == ErrorCategory::config
            ? ExitCode::config_error : ExitCode::runtime_error;
    }

    std::vector<sim::ExpectationCheck> expectations;
    auto scheduled_stimuli = scheduleConfiguredStimuli(
        network_config.value(), *world.value(), expectations
    );
    if (!scheduled_stimuli) {
        err << "fil: " << formatError(scheduled_stimuli.error()) << '\n';
        return ExitCode::config_error;
    }
    world.value()->setDiagnosticsEnabled(true);
    world.value()->trace().setRetainRecords(!binary_protocol);
    sim::ExpectationEvaluator expectation_evaluator(world.value()->trace(), expectations);
    ScopedTraceObserverReset observer_reset(world.value()->trace());
    std::vector<std::string> recorded_types = live_filters;
    if (!expectations.empty()) {
        // Expectation input records are always recorded even when the live
        // display filter excludes them; lifecycle records stay selectable.
        for (const std::string& type : expectationRecordTypes(expectations)) {
            if (std::find(recorded_types.begin(), recorded_types.end(), type) == recorded_types.end()) {
                recorded_types.push_back(type);
            }
        }
    }
    world.value()->trace().setTypeAllowlist(std::move(recorded_types));
    bool protocol_output_failed = false;
    world.value()->trace().setObserver([&out, &live_filters, &expectation_evaluator,
                                         binary_protocol, &protocol_output_failed](
                                            const sim::TraceRecord& record) {
        if (std::find(live_filters.begin(), live_filters.end(), record.type) != live_filters.end()) {
            if (binary_protocol) protocol_output_failed |= !writeProtocolTrace(out, record);
            else printLiveTraceRecord(out, record);
        }
        expectation_evaluator.observe(record);
    });
    if (binary_protocol) {
        BinaryPayloadWriter hello;
        hello.string(network_config.value().name);
        hello.u32(static_cast<std::uint32_t>(world.value()->boardCount()));
        hello.u32(static_cast<std::uint32_t>(world.value()->canBusCount()));
        if (!writeProtocolFrame(out, network_protocol::MessageKind::hello, 0U,
                                std::move(hello))) {
            err << "fil: serve failed to write protocol hello\n";
            return ExitCode::runtime_error;
        }
        if (!expectations.empty()) expectation_evaluator.begin(
            world.value()->eventLoop().now(), world.value()->eventLoop()
        );
    } else {
        if (!expectations.empty()) expectation_evaluator.begin(
            world.value()->eventLoop().now(), world.value()->eventLoop()
        );
        out << "watching network " << network_config.value().name << '\n' << std::flush;
    }

    const sim::SimTimeNs started_at = world.value()->eventLoop().now();
    const auto wall_start = std::chrono::steady_clock::now();
    const std::uint64_t slice_ns = refresh_ns;
    bool stdin_eof = false;
    bool stop_requested = false;
    auto wall_elapsed_ns = [&]() -> std::uint64_t {
        return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - wall_start
        ).count());
    };
    bool duration_reached = false;
    ProtocolEndReason end_reason = ProtocolEndReason::stop_request;
    network_protocol::FrameDecoder protocol_decoder;
    const auto handleProtocolRequest = [&](const network_protocol::Frame& request) -> bool {
        ProtocolReplyStatus status = ProtocolReplyStatus::ok;
        std::string message;
        bool stop_after_reply = false;
        BinaryPayloadReader payload(request.payload);

        if (request.request_id == 0U) {
            status = ProtocolReplyStatus::invalid_request;
            message = "request ID zero is reserved for server messages";
        } else if (request.kind == network_protocol::MessageKind::can_inject) {
            std::string bus_name;
            std::uint32_t id = 0U;
            std::uint8_t flags = 0U;
            std::uint8_t data_length = 0U;
            std::span<const std::uint8_t> data;
            if (!payload.string(bus_name) || !payload.u32(id) || !payload.u8(flags)
                || !payload.u8(data_length) || !payload.raw(data_length, data)
                || !payload.finished() || (flags & 0xf8U) != 0U) {
                status = ProtocolReplyStatus::invalid_request;
                message = "malformed CAN_INJECT payload";
            } else {
                const auto dlc = devices::lengthToDlc(data_length);
                if (!dlc || devices::dlcToLength(*dlc) != data_length) {
                    status = ProtocolReplyStatus::invalid_request;
                    message = "CAN data length must be 0..8, 12, 16, 20, 24, 32, 48, or 64";
                } else {
                    devices::CanFrame frame;
                    frame.id = id;
                    frame.extended = (flags & 0x01U) != 0U;
                    frame.fd = (flags & 0x02U) != 0U;
                    frame.brs = (flags & 0x04U) != 0U;
                    frame.dlc = *dlc;
                    std::copy(data.begin(), data.end(), frame.data.begin());
                    const auto valid = devices::validate(frame);
                    if (!valid) {
                        status = ProtocolReplyStatus::invalid_request;
                        message = valid.error().message;
                    } else if (devices::VirtualCanBus* bus = world.value()->canBus(bus_name);
                               bus == nullptr) {
                        status = ProtocolReplyStatus::unknown_target;
                        message = "CAN_INJECT names an undeclared bus";
                    } else {
                        const auto injected = bus->inject(frame, world.value()->eventLoop().now());
                        if (!injected) {
                            status = injected.error().message == "CAN transmit queue full"
                                ? ProtocolReplyStatus::queue_full
                                : ProtocolReplyStatus::invalid_request;
                            message = injected.error().message;
                        }
                    }
                }
            }
        } else if (request.kind == network_protocol::MessageKind::adc_set) {
            std::string board_name;
            std::string instance_name;
            std::uint8_t channel = 0U;
            std::uint16_t value = 0U;
            if (!payload.string(board_name) || !payload.string(instance_name)
                || !payload.u8(channel) || !payload.u16(value) || !payload.finished()
                || channel > 19U || value > 4095U) {
                status = ProtocolReplyStatus::invalid_request;
                message = "malformed ADC_SET payload or value out of range";
            } else if (sim::Board* board = world.value()->board(board_name); board == nullptr) {
                status = ProtocolReplyStatus::unknown_target;
                message = "ADC_SET names an unknown board";
            } else if (stm32g4::AdcPeripheral* adc = board->peripherals().adc(instance_name);
                       adc == nullptr) {
                status = ProtocolReplyStatus::unknown_target;
                message = "ADC_SET names an unknown ADC instance";
            } else {
                adc->setChannelValue(channel, value);
            }
        } else if (request.kind == network_protocol::MessageKind::gpio_set) {
            std::string board_name;
            std::string port_name;
            std::uint8_t pin = 0U;
            std::uint8_t action = 0U;
            if (!payload.string(board_name) || !payload.string(port_name)
                || !payload.u8(pin) || !payload.u8(action) || !payload.finished()
                || pin > 15U || action > 2U) {
                status = ProtocolReplyStatus::invalid_request;
                message = "malformed GPIO_SET payload or value out of range";
            } else if (sim::Board* board = world.value()->board(board_name); board == nullptr) {
                status = ProtocolReplyStatus::unknown_target;
                message = "GPIO_SET names an unknown board";
            } else if (stm32g4::GpioPeripheral* port = board->peripherals().gpio(port_name);
                       port == nullptr) {
                status = ProtocolReplyStatus::unknown_target;
                message = "GPIO_SET names an unknown GPIO port";
            } else if (action == 2U) {
                port->releaseInput(pin);
            } else {
                port->setInput(pin, action == 1U);
            }
        } else if (request.kind == network_protocol::MessageKind::stop) {
            if (!payload.finished()) {
                status = ProtocolReplyStatus::invalid_request;
                message = "STOP payload must be empty";
            } else {
                stop_after_reply = true;
            }
        } else {
            status = ProtocolReplyStatus::unsupported_message;
            message = "unsupported client message kind";
        }

        if (status != ProtocolReplyStatus::ok) {
            err << "fil: serve request " << request.request_id << ": " << message << '\n';
        }
        if (protocol_output_failed || !writeProtocolReply(
                out, request.request_id, status, world.value()->eventLoop().now(), message)) {
            return false;
        }
        if (stop_after_reply) {
            stop_requested = true;
            end_reason = ProtocolEndReason::stop_request;
        }
        return true;
    };
    const auto finishProtocol = [&](const ProtocolEndReason reason, const ExitCode code,
                                    const std::string_view message = {}) {
        if (!binary_protocol) return code;
        if (!writeProtocolEnd(out, world.value()->eventLoop().now(), reason, code, message)) {
            err << "fil: serve failed to write protocol END\n";
            return ExitCode::runtime_error;
        }
        return code;
    };
    const auto closeProtocolInput = [&]() -> bool {
        const auto complete = protocol_decoder.finish();
        if (!complete) {
            err << "fil: " << complete.error().message << '\n';
            return false;
        }
        stdin_eof = true;
        if (!duration_ns) {
            end_reason = ProtocolEndReason::input_eof;
            stop_requested = true;
        }
        return true;
    };
    const auto readProtocolInput = [&]() -> bool {
        std::array<std::uint8_t, 4096U> input_bytes{};
        ssize_t count = -1;
        do {
            count = ::read(STDIN_FILENO, input_bytes.data(), input_bytes.size());
        } while (count < 0 && errno == EINTR);
        if (count < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return true;
            err << "fil: failed to read serve stdio input\n";
            return false;
        }
        if (count == 0) return closeProtocolInput();
        const auto appended = protocol_decoder.append(std::span<const std::uint8_t>{
            input_bytes.data(), static_cast<std::size_t>(count)
        });
        if (!appended) {
            err << "fil: " << appended.error().message << '\n';
            return false;
        }
        while (!stop_requested) {
            auto frame = protocol_decoder.next();
            if (!frame) {
                err << "fil: " << frame.error().message << '\n';
                return false;
            }
            if (!frame.value()) break;
            if (!handleProtocolRequest(*frame.value())) {
                err << "fil: serve failed to write protocol reply\n";
                return false;
            }
        }
        return true;
    };
    if (binary_protocol) {
        pollfd input{STDIN_FILENO, POLLIN, 0};
        const std::int32_t ready = ::poll(&input, 1, 0);
        if (ready < 0 && errno != EINTR) {
            err << "fil: failed to poll serve stdio input\n";
            return ExitCode::runtime_error;
        }
        if (ready > 0 && (input.revents & (POLLIN | POLLHUP | POLLERR | POLLNVAL)) != 0) {
            if (!readProtocolInput()) return ExitCode::runtime_error;
        }
    }
    const bool window_controls = binary_protocol && !wall_pacing
        && cross_board_window_ns != 0U && !trace_instructions && !detect_spin
        && max_instructions >= 50'000'000U
        && (slice_ns % cross_board_window_ns == 0U || cross_board_window_ns % slice_ns == 0U);
    bool window_controls_ok = true;
    auto next_control_at = started_at + slice_ns;
    const auto pollWindowControls = [&](const sim::SimTimeNs now) {
        if (protocol_output_failed) return false;
        if (now < next_control_at) return true;
        next_control_at = now + slice_ns;
        if (stdin_eof) return !stop_requested;
        pollfd input{STDIN_FILENO, POLLIN, 0};
        const auto ready = ::poll(&input, 1, 0);
        if (ready < 0) {
            if (errno == EINTR) return true;
            err << "fil: failed to poll serve stdio input\n";
            window_controls_ok = false;
        } else if (ready > 0) {
            if ((input.revents & (POLLERR | POLLNVAL)) != 0
                && (input.revents & (POLLIN | POLLHUP)) == 0) {
                window_controls_ok = closeProtocolInput();
            } else if ((input.revents & (POLLIN | POLLHUP)) != 0) {
                window_controls_ok = readProtocolInput();
            }
        }
        return window_controls_ok && !stop_requested;
    };
    while (!stop_requested) {
        const sim::SimTimeNs elapsed = world.value()->eventLoop().now() - started_at;
        if (duration_ns && elapsed >= *duration_ns) {
            duration_reached = true;
            break;
        }

        sim::WorldRunOptions options;
        // Control checks stay at refresh boundaries inside the independent
        // scheduler, amortizing domain setup over a longer host invocation.
        const auto invocation_ns = window_controls ? std::max<sim::SimTimeNs>(slice_ns, 10'000'000U) : slice_ns;
        options.duration_ns = duration_ns ? std::min(invocation_ns, *duration_ns - elapsed) : invocation_ns;
        options.max_instructions_per_board = max_instructions;
        options.instruction_quantum = quantum;
        options.cross_board_window_ns = cross_board_window_ns;
        options.enable_parallel_lanes = enable_parallel_lanes;
            options.enable_jit = enable_jit;
        options.enable_ram_capsules = enable_ram_capsules;
        options.adc_decimation = adc_decimation;
        options.trace_instructions = trace_instructions;
        options.detect_spin = detect_spin;
        if (window_controls) {
            options.cross_board_window_ns = std::min(cross_board_window_ns, slice_ns);
            options.window_control = pollWindowControls;
        }
        auto result = world.value()->run(options);
        if (!window_controls_ok) {
            if (!expectations.empty()) expectation_evaluator.finish(world.value()->eventLoop().now());
            return ExitCode::runtime_error;
        }
        if (protocol_output_failed) {
            if (!expectations.empty()) expectation_evaluator.finish(world.value()->eventLoop().now());
            err << "fil: serve failed to write protocol trace\n";
            return ExitCode::runtime_error;
        }
        if (!result || !result.value().succeeded()) {
            if (!expectations.empty()) expectation_evaluator.finish(world.value()->eventLoop().now());
            const std::string failure = !result
                ? formatError(result.error())
                : "network stopped with " + std::string(sim::worldStopReasonName(result.value().reason));
            err << "fil: " << command_name << " stopped: " << failure << '\n';
            if (protocol_output_failed) return ExitCode::runtime_error;
            return finishProtocol(ProtocolEndReason::runtime_error, ExitCode::runtime_error, failure);
        }
        if (result.value().reason == sim::WorldStopReason::all_boards_stopped) {
            end_reason = ProtocolEndReason::all_boards_stopped;
            break;
        }

        // Pace simulation time to wall-clock time so live consumers observe
        // real rates. Each iteration re-syncs to the absolute sim-vs-wall
        // delta, so the poll below never accumulates drift. When the model
        // runs slower than real time there is nothing to wait for; slices
        // run back-to-back and trace order is unchanged either way.
        if (wall_pacing) {
            const sim::SimTimeNs sim_elapsed = world.value()->eventLoop().now() - started_at;
            const std::uint64_t wall_elapsed = wall_elapsed_ns();
            if (sim_elapsed > wall_elapsed) {
                std::this_thread::sleep_for(
                    std::chrono::nanoseconds(sim_elapsed - wall_elapsed)
                );
            }
        }

        if (stdin_eof) continue;
        pollfd input{STDIN_FILENO, POLLIN, 0};
        const bool buffered = !binary_protocol && std::cin.rdbuf()->in_avail() > 0;
        // Pacing is handled above. Input polling must never add a wait to
        // either paced or back-to-back slices.
        const std::int32_t ready = buffered
            ? 1
            : ::poll(&input, 1, 0);
        if (ready < 0) {
            if (binary_protocol && errno == EINTR) continue;
            if (!expectations.empty()) expectation_evaluator.finish(world.value()->eventLoop().now());
            err << "fil: failed to poll stdin\n";
            return ExitCode::runtime_error;
        }
        if (ready == 0) continue;
        if (binary_protocol) {
            if ((input.revents & (POLLERR | POLLNVAL)) != 0
                && (input.revents & (POLLIN | POLLHUP)) == 0) {
                if (!closeProtocolInput()) return ExitCode::runtime_error;
            } else if ((input.revents & (POLLIN | POLLHUP)) != 0
                       && !readProtocolInput()) {
                if (!expectations.empty()) expectation_evaluator.finish(world.value()->eventLoop().now());
                return ExitCode::runtime_error;
            }
            continue;
        }
        if (!buffered && (input.revents & (POLLERR | POLLNVAL)) != 0) {
            // An unusable control channel behaves like end-of-input.
            stdin_eof = true;
            if (!duration_ns) break;
            continue;
        }

        std::string line;
        if (!std::getline(std::cin, line)) {
            stdin_eof = true;
            if (!duration_ns) break;
            continue;
        }
        if (line == "quit" || line == "exit") {
            stop_requested = true;
            continue;
        }

        const std::size_t command_end = line.find_first_of(" \t");
        const std::string_view command = std::string_view{line}.substr(0U, command_end);
        if (command == "adc") {
            auto injection = parseAdcInjection(line);
            if (!injection) {
                err << "fil: ignored stdin ADC command: " << injection.error().message << '\n';
                continue;
            }
            sim::Board* board = world.value()->board(injection.value().board);
            if (board == nullptr) {
                err << "fil: ignored stdin ADC command for unknown board: "
                    << injection.value().board << '\n';
                continue;
            }
            stm32g4::AdcPeripheral* adc = board->peripherals().adc(injection.value().instance);
            if (adc == nullptr) {
                err << "fil: ignored stdin ADC command for unknown ADC instance: "
                    << injection.value().instance << '\n';
                continue;
            }
            adc->setChannelValue(injection.value().channel, injection.value().value);
        } else if (command == "gpio") {
            auto injection = parseGpioInjection(line);
            if (!injection) {
                err << "fil: ignored stdin GPIO command: " << injection.error().message << '\n';
                continue;
            }
            sim::Board* board = world.value()->board(injection.value().board);
            if (board == nullptr) {
                err << "fil: ignored stdin GPIO command for unknown board: "
                    << injection.value().board << '\n';
                continue;
            }
            stm32g4::GpioPeripheral* port = board->peripherals().gpio(injection.value().port);
            if (port == nullptr) {
                err << "fil: ignored stdin GPIO command for unknown port: "
                    << injection.value().port << '\n';
                continue;
            }
            if (injection.value().level.has_value()) {
                port->setInput(injection.value().pin, injection.value().level.value());
            } else {
                port->releaseInput(injection.value().pin);
            }
        } else {
            auto injection = parseCanInjection(line);
            if (!injection) {
                err << "fil: ignored stdin CAN command: " << injection.error().message << '\n';
                continue;
            }
            devices::VirtualCanBus* bus = world.value()->canBus(injection.value().bus);
            if (bus == nullptr) {
                err << "fil: ignored stdin CAN command for undeclared bus: "
                    << injection.value().bus << '\n';
                continue;
            }
            static_cast<void>(
                bus->inject(injection.value().frame, world.value()->eventLoop().now())
            );
        }
    }
    if (!expectations.empty()) expectation_evaluator.finish(world.value()->eventLoop().now());
    if (duration_reached) end_reason = ProtocolEndReason::duration_reached;
    else if (stdin_eof && !stop_requested) end_reason = ProtocolEndReason::input_eof;
    if (protocol_output_failed) {
        err << "fil: serve failed to write protocol trace\n";
        return ExitCode::runtime_error;
    }
    // Total-duration exactness: the per-slice sleeps keep phase within a
    // fraction of a slice, but the last sleep's wake jitter is never
    // corrected inside the loop. Sync once to the absolute deadline so the
    // run's wall time equals its simulated time. Early exits (quit, halted
    // boards, EOF without a duration) return immediately.
    if (wall_pacing && duration_reached && !stop_requested) {
        std::this_thread::sleep_until(wall_start + std::chrono::nanoseconds(*duration_ns));
    }
    return finishProtocol(end_reason, ExitCode::success);
}

ExitCode watchNetworkCommand(
    const std::span<const std::string_view> args,
    std::ostream& out,
    std::ostream& err
) {
    return networkMonitorCommand(args, out, err, false);
}

class ScopedSigpipeIgnore {
public:
    ScopedSigpipeIgnore() : previous_(std::signal(SIGPIPE, SIG_IGN)) {}
    ~ScopedSigpipeIgnore() {
        if (previous_ != SIG_ERR) static_cast<void>(std::signal(SIGPIPE, previous_));
    }
    ScopedSigpipeIgnore(const ScopedSigpipeIgnore&) = delete;
    ScopedSigpipeIgnore& operator=(const ScopedSigpipeIgnore&) = delete;

private:
    using SignalHandler = void (*)(std::int32_t);
    SignalHandler previous_{SIG_ERR};
};

ExitCode serveNetworkCommand(
    const std::span<const std::string_view> args,
    std::ostream& out,
    std::ostream& err
) {
    if (args.size() < 2U) {
        err << "fil: serve requires a network config path\n";
        return ExitCode::usage_error;
    }

    std::vector<std::string_view> monitor_args{"serve", args[1]};
    for (std::size_t index = 2U; index < args.size(); ++index) {
        const std::string_view option = args[index];
        if (option == "--control-stdin") {
            err << "fil: serve uses stdio protocol controls; --control-stdin is not applicable\n";
            return ExitCode::usage_error;
        }
        if (option == "--transport") {
            err << "fil: serve always uses the stdio transport; --transport is not needed\n";
            return ExitCode::usage_error;
        }
        monitor_args.push_back(option);
    }

    if (&out == &std::cout && ::isatty(STDOUT_FILENO)) {
        err << "fil: serve cannot write binary protocol frames to a terminal; "
               "pipe stdout to a protocol client or redirect it to a file. "
               "Use watch for terminal output.\n";
        return ExitCode::usage_error;
    }

    ScopedSigpipeIgnore ignore_sigpipe;
    return networkMonitorCommand(monitor_args, out, err, true);
}

} // namespace

ExitCode runCommand(
    const std::span<const std::string_view> args,
    std::ostream& out,
    std::ostream& err
) {
    if (args.size() < 2U) {
        err << "fil: run requires a config path\n";
        return ExitCode::usage_error;
    }
    if (config::loadBoardConfig(args[1])) {
        return runBoardCommand(args, out, err);
    }
    if (config::loadNetworkConfig(args[1])) {
        return runNetworkCommand(args, out, err);
    }
    // Neither kind loads. Route through the board parser so option errors keep
    // precedence over the configuration diagnostic.
    return runBoardCommand(args, out, err);
}

void printHelp(std::ostream& out) {
    out << "fil - deterministic STM32G4 firmware emulator\n\n"
        << "Usage:\n"
        << "  fil --help\n"
        << "  fil --version\n\n"
        << "Commands:\n"
        << "  inspect-config <config.json>  Validate and normalize an MCU, board, network, or stimulus input\n"
        << "  inspect-elf <firmware.elf>     Inspect an ELF32 ARM firmware image\n"
        << "  disasm-window <firmware.elf>   Decode a bounded Thumb instruction window\n"
        << "  run <config.json> [options]    Execute a board or network deterministically\n"
        << "  watch <config.json> [options]  Run a human-facing live network monitor\n"
        << "  serve <config.json> [options]  Serve binary stdio control and traces\n"
        << "  compare-stlink <board.json>    Compare emulator state with STM32G4 hardware\n\n"
        << "Defaults: unlimited simulated time and instruction budget (runs to completion),\n"
        << "JIT on, network RAM capsules on, ADC decimation 1.\n"
        << "A bare --duration number is seconds; us, ms, s, and m suffixes select other units.\n"
        << "ADC decimation above 1 trades fidelity for speed; diagnostics retain safe fallbacks.\n\n"
        << "Run options:\n"
        << "  --duration D --instruction-budget N --trace FILE --trace-instr\n"
        << "  --strict-mmio --stop-address ADDR --stop-at-symbol NAME --allow-breakpoint\n"
        << "  --detect-spin --adc-decimation N --jit-stats\n\n"
        << "ST-Link comparison options:\n"
        << "  --flash --memory ADDR:LENGTH --register NAME --ignore-register NAME\n"
        << "  --stop-address ADDR --stop-at-symbol NAME --max-instructions N\n"
        << "  --serial ID --openocd PATH --timeout-ms N --artifacts DIRECTORY\n\n"
        << "Network options:\n"
        << "  --duration D --instruction-budget N --quantum N --trace FILE\n"
        << "  --cross-board-window-ns N (network commands: default 100000; 0 = exact)\n"
        << "  --parallel-lanes (default) | --no-parallel-lanes (validation reference)\n"
        << "  --strict-mmio --trace-instr --detect-spin --allow-breakpoint\n"
        << "  --adc-decimation N (keep 1 of N continuous ADC scans) --jit-stats\n"
        << "  --inject-can BUS[@TIME_MS]:ID:HEXDATA\n\n"
        << "Watch options:\n"
        << "  --duration D --refresh-ms N --refresh-us N --instruction-budget N --quantum N\n"
        << "  --adc-decimation N (keep 1 of N continuous ADC scans)\n"
        << "  --trace-type TYPE (alias: --live-filter; repeatable) --strict-mmio --trace-instr --detect-spin\n"
        << "  --control-stdin\n"
        << "  --no-wall-pacing (watch runs slices back-to-back instead of real time)\n"
        << "  watch also stops early when all boards reach terminal CPU boundaries\n"
        << "  stdin: BUS:ID:HEXDATA, adc BOARD INSTANCE CHANNEL VALUE,\n"
        << "  gpio BOARD PORT PIN 0|1|release, quit, or exit\n"
        << "\nServe options:\n"
        << "  Binary protocol on stdin/stdout; stdio is the only transport (no --transport).\n"
        << "  --duration D --refresh-ms N --refresh-us N --instruction-budget N --quantum N\n"
        << "  --live-filter TYPE (repeatable), --trace-instr, --no-wall-pacing\n"
        << "  --adc-decimation N --strict-mmio --lenient-mmio --detect-spin\n"
        << "  See docs/serve_network.md for the versioned binary protocol.\n";
}

ExitCode run(
    const std::span<const std::string_view> args,
    std::ostream& out,
    std::ostream& err
) {
    if (args.empty() || args.front() == "--help" || args.front() == "-h") {
        printHelp(out);
        return ExitCode::success;
    }

    if (args.front() == "--version") {
        out << "fil " << version << '\n';
        return ExitCode::success;
    }

    if (args.front() == "inspect-elf") {
        if (args.size() != 2) {
            err << "fil: inspect-elf requires exactly one firmware path\n";
            return ExitCode::usage_error;
        }
        auto image = elf::load(args[1]);
        if (!image) {
            err << "fil: " << formatError(image.error()) << '\n';
            return ExitCode::runtime_error;
        }
        out << elf::inspect(image.value(), args[1]);
        return ExitCode::success;
    }

    if (args.front() == "disasm-window") {
        return disassembleWindowCommand(args, out, err);
    }

    if (args.front() == "inspect-config") {
        if (args.size() != 2) {
            err << "fil: inspect-config requires exactly one config path\n";
            return ExitCode::usage_error;
        }

        auto board = config::loadBoardConfig(args[1]);
        if (board) {
            out << config::normalize(board.value());
            return ExitCode::success;
        }

        auto mcu = config::loadMcuConfig(args[1]);
        if (mcu) {
            out << config::normalize(mcu.value());
            return ExitCode::success;
        }

        auto network = config::loadNetworkConfig(args[1]);
        if (network) {
            out << config::normalize(network.value());
            return ExitCode::success;
        }

        auto stimulus = sim::loadStimulusScript(args[1]);
        if (stimulus) {
            out << sim::normalizeStimulusScript(stimulus.value());
            return ExitCode::success;
        }

        err << "fil: input is not a valid board, MCU, network, or stimulus script\n"
            << "  board: " << formatError(board.error()) << '\n'
            << "  mcu: " << formatError(mcu.error()) << '\n'
            << "  network: " << formatError(network.error()) << '\n'
            << "  stimulus: " << formatError(stimulus.error()) << '\n';
        return ExitCode::config_error;
    }

    if (args.front() == "run") {
        return runCommand(args, out, err);
    }

    if (args.front() == "watch") {
        return watchNetworkCommand(args, out, err);
    }

    if (args.front() == "serve") {
        return serveNetworkCommand(args, out, err);
    }

    if (args.front() == "compare-stlink") {
        return compareStlinkCommand(args, out, err);
    }

    err << "fil: unknown command or option: " << args.front() << '\n'
        << "Try 'fil --help' for usage.\n";
    return ExitCode::usage_error;
}

} // namespace fil::cli
