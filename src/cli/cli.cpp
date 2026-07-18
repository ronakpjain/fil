#include "fil/cli/cli.hpp"

#include "fil/common/error.hpp"
#include "fil/config/config.hpp"
#include "fil/devices/can_bus.hpp"
#include "fil/cpu/decoder.hpp"
#include "fil/elf/elf_loader.hpp"
#include "fil/sim/board.hpp"
#include "fil/sim/world.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <ostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fil::cli {

namespace {

constexpr std::string_view version = "0.1.0";

void printLiveRecord(std::ostream& out, const sim::TraceRecord& record) {
    const double milliseconds = static_cast<double>(record.time_ns) / 1'000'000.0;
    out << '[' << std::fixed << std::setprecision(3) << milliseconds << " ms] "
        << record.source << "  " << record.type;
    for (const auto& [key, value] : record.fields) out << ' ' << key << '=' << value;
    out << std::defaultfloat << '\n';
    out.flush();
}

bool liveTypeSelected(
    const sim::TraceRecord& record,
    const std::vector<std::string>& filters
) {
    if (filters.empty()) return true;
    for (const std::string& filter : filters) {
        if (record.type == filter) return true;
    }
    return false;
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

ExitCode runBoardCommand(
    const std::span<const std::string_view> args,
    std::ostream& out,
    std::ostream& err
) {
    if (args.size() < 2) {
        err << "fil: run requires a board config path\n";
        return ExitCode::usage_error;
    }
    std::optional<std::uint64_t> duration_ms;
    std::optional<std::uint64_t> max_instructions;
    std::optional<std::uint32_t> stop_address;
    std::optional<std::string> stop_symbol;
    std::optional<std::filesystem::path> trace_path;
    bool strict_mmio = false;
    bool trace_instructions = false;
    bool detect_spin = false;
    bool enable_loop_batching = true;
    bool allow_breakpoint = false;
    bool realtime = args.front() == "watch";
    bool live = realtime;
    std::uint64_t refresh_ms = 10U;
    std::vector<std::string> live_filters;

    for (std::size_t index = 2; index < args.size(); ++index) {
        const std::string_view option = args[index];
        const auto valueAfter = [&]() -> std::optional<std::string_view> {
            if (index + 1 >= args.size()) return std::nullopt;
            return args[++index];
        };
        if (option == "--duration-ms" || option == "--max-instructions"
            || option == "--stop-address" || option == "--refresh-ms") {
            const auto value = valueAfter();
            if (!value) {
                err << "fil: " << option << " requires a value\n";
                return ExitCode::usage_error;
            }
            auto parsed = config::parseUnsigned(*value);
            if (!parsed) {
                err << "fil: invalid value for " << option << ": " << parsed.error().message << '\n';
                return ExitCode::usage_error;
            }
            if (option == "--duration-ms") duration_ms = parsed.value();
            else if (option == "--max-instructions") max_instructions = parsed.value();
            else if (option == "--refresh-ms") refresh_ms = parsed.value();
            else {
                if (parsed.value() > std::numeric_limits<std::uint32_t>::max()) {
                    err << "fil: --stop-address exceeds 32-bit target address space\n";
                    return ExitCode::usage_error;
                }
                stop_address = static_cast<std::uint32_t>(parsed.value());
            }
        } else if (option == "--stop-at-symbol" || option == "--trace"
                   || option == "--live-filter") {
            const auto value = valueAfter();
            if (!value) {
                err << "fil: " << option << " requires a value\n";
                return ExitCode::usage_error;
            }
            if (option == "--stop-at-symbol") stop_symbol = std::string(*value);
            else if (option == "--trace") trace_path = std::filesystem::path(*value);
            else live_filters.emplace_back(*value);
        } else if (option == "--strict-mmio") strict_mmio = true;
        else if (option == "--lenient-mmio") strict_mmio = false;
        else if (option == "--trace-instr") trace_instructions = true;
        else if (option == "--detect-spin") detect_spin = true;
        else if (option == "--no-detect-spin") detect_spin = false;
        else if (option == "--loop-batching") enable_loop_batching = true;
        else if (option == "--no-loop-batching") enable_loop_batching = false;
        else if (option == "--allow-breakpoint") allow_breakpoint = true;
        else if (option == "--realtime") realtime = true;
        else if (option == "--no-realtime") realtime = false;
        else if (option == "--live") live = true;
        else if (option == "--no-live") live = false;
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
    if (refresh_ms == 0U || refresh_ms > std::numeric_limits<std::uint64_t>::max() / 1'000'000ULL) {
        err << "fil: --refresh-ms must be a positive representable duration\n";
        return ExitCode::usage_error;
    }
    const bool diagnostics_enabled = trace_path.has_value() || live;
    board.value()->trace().setEnabled(diagnostics_enabled);
    board.value()->peripherals().setAdcDiagnosticsEnabled(diagnostics_enabled);
    if (live) {
        out << "watching board " << board_config.value().name << " (Ctrl-C to stop)\n";
        board.value()->trace().setObserver([&out, &live_filters](const sim::TraceRecord& record) {
            if (liveTypeSelected(record, live_filters)) printLiveRecord(out, record);
        });
    }
    board.value()->eventLoop().setRealtimePacing(realtime, refresh_ms * 1'000'000ULL);

    sim::BoardRunOptions options;
    options.max_instructions = max_instructions.value_or(board_config.value().run.max_instructions);
    const std::uint64_t configured_duration = duration_ms.value_or(board_config.value().run.default_duration_ms);
    if (configured_duration > std::numeric_limits<std::uint64_t>::max() / 1'000'000ULL) {
        err << "fil: duration is too large\n";
        return ExitCode::usage_error;
    }
    options.duration_ns = configured_duration * 1'000'000ULL;
    options.trace_instructions = trace_instructions;
    options.detect_spin = detect_spin;
    options.enable_loop_batching = enable_loop_batching;
    options.stop_address = stop_address;
    if (stop_symbol) {
        const elf::ElfSymbol* match = nullptr;
        for (const auto& symbol : board.value()->image().symbols()) {
            if (symbol.name == *stop_symbol) {
                match = &symbol;
                break;
            }
        }
        if (match == nullptr) {
            err << "fil: symbol not found: " << *stop_symbol << '\n';
            return ExitCode::usage_error;
        }
        options.stop_address = match->address;
    }

    const sim::BoardRunResult result = board.value()->run(options);
    out << "board: " << board_config.value().name << '\n'
        << "stop: " << sim::boardStopReasonName(result.reason) << '\n'
        << "instructions: " << result.instructions << '\n'
        << "cycles: " << result.cycles << '\n'
        << "time_ns: " << result.time_ns << '\n'
        << "pc: 0x" << std::hex << result.diagnostic.next_pc << std::dec << '\n';
    if (!result.succeeded()) {
        const auto& state = board.value()->cpu().state();
        out << "msp: 0x" << std::hex << state.msp << '\n'
            << "psp: 0x" << state.psp << '\n'
            << "lr: 0x" << state.r[14] << '\n'
            << "xpsr: 0x" << state.xpsr << std::dec << '\n';
    }
    if (!result.message.empty()) out << "message: " << result.message << '\n';
    const auto unknown = board.value()->peripherals().router().unknownAccesses();
    out << "unknown_mmio_addresses: " << unknown.size() << '\n';
    for (const auto& access : unknown) {
        out << "unknown_mmio: 0x" << std::hex << access.address << std::dec
            << " reads=" << access.reads << " writes=" << access.writes << '\n';
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

ExitCode runNetworkCommand(
    const std::span<const std::string_view> args,
    std::ostream& out,
    std::ostream& err
) {
    if (args.size() < 2) {
        err << "fil: run-network requires a network config path\n";
        return ExitCode::usage_error;
    }

    std::uint64_t duration_ms = 1'000U;
    std::uint64_t max_instructions = 50'000'000U;
    std::uint64_t quantum = 1'024U;
    std::optional<std::filesystem::path> trace_path;
    bool strict_mmio = false;
    bool trace_instructions = false;
    bool detect_spin = false;
    bool enable_loop_batching = true;
    bool enable_transactional_slices = false;
    bool allow_breakpoint = false;
    bool control_stdin = false;
    bool realtime = args.front() == "watch-network";
    bool live = realtime;
    std::uint64_t refresh_ms = 10U;
    std::vector<std::string> live_filters;
    std::vector<PendingCanInjection> injections;

    for (std::size_t index = 2; index < args.size(); ++index) {
        const std::string_view option = args[index];
        const auto valueAfter = [&]() -> std::optional<std::string_view> {
            if (index + 1 >= args.size()) return std::nullopt;
            return args[++index];
        };
        if (option == "--duration-ms" || option == "--max-instructions"
            || option == "--quantum" || option == "--refresh-ms") {
            const auto value = valueAfter();
            if (!value) {
                err << "fil: " << option << " requires a value\n";
                return ExitCode::usage_error;
            }
            auto parsed = config::parseUnsigned(*value);
            if (!parsed) {
                err << "fil: invalid value for " << option << ": " << parsed.error().message << '\n';
                return ExitCode::usage_error;
            }
            if (option == "--duration-ms") duration_ms = parsed.value();
            else if (option == "--max-instructions") max_instructions = parsed.value();
            else if (option == "--refresh-ms") refresh_ms = parsed.value();
            else quantum = parsed.value();
        } else if (option == "--trace" || option == "--inject-can"
                   || option == "--live-filter") {
            const auto value = valueAfter();
            if (!value) {
                err << "fil: " << option << " requires a value\n";
                return ExitCode::usage_error;
            }
            if (option == "--trace") {
                trace_path = std::filesystem::path(*value);
            } else if (option == "--live-filter") {
                live_filters.emplace_back(*value);
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
        else if (option == "--loop-batching") enable_loop_batching = true;
        else if (option == "--no-loop-batching") enable_loop_batching = false;
        else if (option == "--transactional-slices") enable_transactional_slices = true;
        else if (option == "--no-transactional-slices") enable_transactional_slices = false;
        else if (option == "--allow-breakpoint") allow_breakpoint = true;
        else if (option == "--realtime") realtime = true;
        else if (option == "--no-realtime") realtime = false;
        else if (option == "--live") live = true;
        else if (option == "--no-live") live = false;
        else if (option == "--control-stdin") control_stdin = true;
        else {
            err << "fil: unknown run-network option: " << option << '\n';
            return ExitCode::usage_error;
        }
    }
    if (duration_ms > std::numeric_limits<std::uint64_t>::max() / 1'000'000ULL) {
        err << "fil: duration is too large\n";
        return ExitCode::usage_error;
    }
    if (refresh_ms == 0U || refresh_ms > std::numeric_limits<std::uint64_t>::max() / 1'000'000ULL) {
        err << "fil: --refresh-ms must be a positive representable duration\n";
        return ExitCode::usage_error;
    }

    auto network_config = config::loadNetworkConfig(args[1]);
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
    world.value()->setDiagnosticsEnabled(trace_path.has_value() || live);
    if (live) {
        out << "watching network " << network_config.value().name << " (Ctrl-C to stop)\n";
        world.value()->trace().setObserver([&out, &live_filters](const sim::TraceRecord& record) {
            if (liveTypeSelected(record, live_filters)) printLiveRecord(out, record);
        });
    }
    world.value()->eventLoop().setRealtimePacing(realtime, refresh_ms * 1'000'000ULL);

    struct StdinControlState {
        std::mutex mutex;
        bool active{true};
    };
    std::shared_ptr<StdinControlState> stdin_control;
    if (control_stdin) {
        stdin_control = std::make_shared<StdinControlState>();
        world.value()->eventLoop().setConcurrentAccess(true);
        sim::World* const controlled_world = world.value().get();
        std::thread([controlled_world, state = stdin_control]() {
            std::string line;
            while (std::getline(std::cin, line)) {
                if (line.starts_with("adc ")) {
                    std::istringstream command(line);
                    std::string kind;
                    std::string board_name;
                    std::string adc_name;
                    std::string channel_text;
                    std::string value_text;
                    std::string trailing;
                    command >> kind >> board_name >> adc_name >> channel_text >> value_text;
                    if (kind != "adc" || board_name.empty() || adc_name.empty()
                        || channel_text.empty() || value_text.empty() || (command >> trailing)) {
                        std::cerr << "fil: ignored stdin ADC command; expected "
                                     "adc BOARD ADCx CHANNEL VALUE\n";
                        continue;
                    }
                    const auto channel = config::parseUnsigned(channel_text);
                    const auto value = config::parseUnsigned(value_text);
                    if (!channel || channel.value() > 19U || !value || value.value() > 4095U) {
                        std::cerr << "fil: ignored stdin ADC command; channel must be 0..19 "
                                     "and value must be 0..4095\n";
                        continue;
                    }
                    std::lock_guard lock(state->mutex);
                    if (!state->active) return;
                    sim::Board* const board = controlled_world->board(board_name);
                    stm32g4::AdcPeripheral* const adc = board == nullptr
                        ? nullptr : board->peripherals().adc(adc_name);
                    if (adc == nullptr) {
                        std::cerr << "fil: ignored stdin ADC command for unknown board/ADC: "
                                  << board_name << '/' << adc_name << '\n';
                        continue;
                    }
                    sim::EventLoop* const loop = &controlled_world->eventLoop();
                    sim::TraceRecorder* const trace = &controlled_world->trace();
                    static_cast<void>(loop->scheduleAfter(
                        0U,
                        [adc, loop, trace, board_name, adc_name,
                         channel = static_cast<unsigned int>(channel.value()),
                         value = static_cast<std::uint16_t>(value.value())]() {
                            adc->overrideChannelValue(channel, value);
                            trace->record(
                                loop->now(), board_name + "." + adc_name, "adc_input",
                                {{"channel", std::to_string(channel)},
                                 {"value", std::to_string(value)}}
                            );
                        }
                    ));
                    continue;
                }
                auto injection = parseCanInjection(line);
                if (!injection) {
                    std::cerr << "fil: ignored stdin CAN command: "
                              << injection.error().message << '\n';
                    continue;
                }
                std::lock_guard lock(state->mutex);
                if (!state->active) return;
                devices::VirtualCanBus* const bus = controlled_world->canBus(injection.value().bus);
                if (bus == nullptr) {
                    std::cerr << "fil: ignored stdin CAN command for undeclared bus: "
                              << injection.value().bus << '\n';
                    continue;
                }
                sim::EventLoop* const loop = &controlled_world->eventLoop();
                static_cast<void>(loop->scheduleAfter(
                    0U,
                    [bus, loop, frame = injection.value().frame]() {
                        static_cast<void>(bus->inject(frame, loop->now()));
                    }
                ));
            }
        }).detach();
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

    sim::WorldRunOptions options;
    options.duration_ns = duration_ms * 1'000'000ULL;
    options.max_instructions_per_board = max_instructions;
    options.instruction_quantum = quantum;
    options.trace_instructions = trace_instructions;
    options.detect_spin = detect_spin;
    options.enable_loop_batching = enable_loop_batching;
    options.enable_transactional_slices = enable_transactional_slices;
    auto result = world.value()->run(options);
    if (stdin_control) {
        std::lock_guard lock(stdin_control->mutex);
        stdin_control->active = false;
    }
    if (!result) {
        err << "fil: " << formatError(result.error()) << '\n';
        return result.error().category == ErrorCategory::invalid_argument
            ? ExitCode::usage_error : ExitCode::runtime_error;
    }

    out << "network: " << network_config.value().name << '\n'
        << "stop: " << sim::worldStopReasonName(result.value().reason) << '\n'
        << "boards: " << result.value().boards.size() << '\n'
        << "instructions: " << result.value().instructions << '\n'
        << "cycles: " << result.value().cycles << '\n'
        << "time_ns: " << result.value().end_time_ns << '\n'
        << "rounds: " << result.value().rounds << '\n'
        << "dispatches: " << result.value().dispatches << '\n'
        << "exact_dispatches: " << result.value().exact_dispatches << '\n'
        << "loop_batches: " << result.value().loop_batches << '\n'
        << "batched_instructions: " << result.value().batched_instructions << '\n'
        << "event_callbacks: " << result.value().event_callbacks << '\n'
        << "transactional_attempts: " << result.value().transactional_attempts << '\n'
        << "transactional_commits: " << result.value().transactional_commits << '\n'
        << "transactional_instructions: "
        << result.value().transactional_instructions << '\n'
        << "lockstep_bursts: " << result.value().lockstep_bursts << '\n';
    for (const auto& board : result.value().boards) {
        out << "board " << board.name
            << ": stop=" << sim::boardStopReasonName(board.result.reason)
            << " instructions=" << board.result.instructions
            << " pc=0x" << std::hex << board.result.diagnostic.next_pc << std::dec << '\n';
        if (!board.result.message.empty() && !board.result.succeeded()) {
            out << "  message: " << board.result.message << '\n';
        }
        if (sim::Board* instance = world.value()->board(board.name)) {
            const auto unknown = instance->peripherals().router().unknownAccesses();
            for (const auto& access : unknown) {
                out << "  unknown_mmio: 0x" << std::hex << access.address << std::dec
                    << " reads=" << access.reads << " writes=" << access.writes << '\n';
            }
        }
    }
    if (!result.value().message.empty()) out << "message: " << result.value().message << '\n';

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
    if (!result.value().succeeded() || unexpected_breakpoint) {
        err << "fil: network stopped with " << sim::worldStopReasonName(result.value().reason) << '\n';
        if (unexpected_breakpoint) err << "fil: a board reached an unrequested breakpoint\n";
        return ExitCode::runtime_error;
    }
    return ExitCode::success;
}

} // namespace

void printHelp(std::ostream& out) {
    out << "fil - deterministic STM32G4 firmware emulator\n\n"
        << "Usage:\n"
        << "  fil --help\n"
        << "  fil --version\n\n"
        << "Commands:\n"
        << "  inspect-config <config.json>  Validate and normalize an MCU or board config\n"
        << "  inspect-elf <firmware.elf>     Inspect an ELF32 ARM firmware image\n"
        << "  disasm-window <firmware.elf>   Decode a bounded Thumb instruction window\n"
        << "  run <board.json> [options]     Execute one firmware board deterministically\n"
        << "  watch <board.json> [options]   Run in real time and print events as they occur\n"
        << "  run-network <network.json>     Execute a deterministic multi-board CAN network\n"
        << "  watch-network <network.json>   Watch a CAN network in real time\n\n"
        << "Run options:\n"
        << "  --duration-ms N --max-instructions N --trace FILE --trace-instr\n"
        << "  --strict-mmio --stop-address ADDR --stop-at-symbol NAME --allow-breakpoint\n"
        << "  --detect-spin --no-loop-batching\n"
        << "  --realtime --live --refresh-ms N --live-filter TYPE (repeatable)\n\n"
        << "Network options:\n"
        << "  --duration-ms N --max-instructions N --quantum N --trace FILE\n"
        << "  --strict-mmio --trace-instr --detect-spin --no-loop-batching --allow-breakpoint\n"
        << "  --transactional-slices (experimental parallel lane epochs)\n"
        << "  --inject-can BUS[@TIME_MS]:ID:HEXDATA\n"
        << "  --control-stdin (accept CAN and ADC commands while running)\n";
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

        err << "fil: config is not a valid board, MCU, or network config\n"
            << "  board: " << formatError(board.error()) << '\n'
            << "  mcu: " << formatError(mcu.error()) << '\n'
            << "  network: " << formatError(network.error()) << '\n';
        return ExitCode::config_error;
    }

    if (args.front() == "run" || args.front() == "watch") {
        return runBoardCommand(args, out, err);
    }

    if (args.front() == "run-network" || args.front() == "watch-network") {
        return runNetworkCommand(args, out, err);
    }

    err << "fil: unknown command or option: " << args.front() << '\n'
        << "Try 'fil --help' for usage.\n";
    return ExitCode::usage_error;
}

} // namespace fil::cli
