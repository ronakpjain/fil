#include "fil/hardware/comparison.hpp"

#include "fil/common/format.hpp"
#include "fil/common/numeric.hpp"
#include "fil/config/config.hpp"
#include "fil/mem/region.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cctype>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <ostream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#if defined(__APPLE__) || defined(__unix__)
#include <fcntl.h>
#include <poll.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
#endif

namespace fil::hardware {
namespace {

constexpr std::array<std::string_view, 23> register_names{
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
    "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15",
    "xpsr", "msp", "psp", "primask", "basepri", "faultmask", "control",
};

Error error(const ErrorCategory category, std::string message) {
    return Error{category, std::move(message), std::nullopt};
}

std::string hex32(const std::uint32_t value) {
    return fil::hexValue(value, 8U);
}

void writeJsonString(std::ostream& output, const std::string_view text) {
    output << '"';
    for (const char value : text) {
        const auto byte = static_cast<unsigned char>(value);
        switch (byte) {
        case '"': output << "\\\""; break;
        case '\\': output << "\\\\"; break;
        case '\b': output << "\\b"; break;
        case '\f': output << "\\f"; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default:
            if (byte < 0x20U) {
                output << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
                       << static_cast<unsigned int>(byte) << std::dec;
            } else {
                output << static_cast<char>(byte);
            }
        }
    }
    output << '"';
}

std::string bytesToHex(const std::span<const std::uint8_t> bytes) {
    constexpr char digits[] = "0123456789abcdef";
    std::string result;
    result.resize(bytes.size() * 2U);
    for (std::size_t index = 0; index < bytes.size(); ++index) {
        result[index * 2U] = digits[bytes[index] >> 4U];
        result[index * 2U + 1U] = digits[bytes[index] & 0x0fU];
    }
    return result;
}

const RegisterValue* findRegister(const Snapshot& snapshot, const std::string_view name) {
    const auto match = std::find_if(
        snapshot.registers.begin(), snapshot.registers.end(),
        [name](const RegisterValue& value) { return value.name == name; }
    );
    return match == snapshot.registers.end() ? nullptr : &*match;
}

const MemoryValue* findMemory(const Snapshot& snapshot, const MemoryRange range) {
    const auto match = std::find_if(
        snapshot.memory.begin(), snapshot.memory.end(),
        [range](const MemoryValue& value) {
            return value.range.address == range.address && value.range.size == range.size;
        }
    );
    return match == snapshot.memory.end() ? nullptr : &*match;
}

bool rangeIsInspectable(
    const std::span<const mem::MemoryRegionInfo> regions,
    const MemoryRange range
) {
    if (range.size == 0U
        || !fil::rangeFits(range.address, range.size, std::uint64_t{1U} << 32U)) {
        return false;
    }
    const std::uint64_t end = static_cast<std::uint64_t>(range.address) + range.size;
    std::uint64_t cursor = range.address;
    while (cursor < end) {
        const auto match = std::find_if(
            regions.begin(), regions.end(),
            [cursor](const mem::MemoryRegionInfo& region) {
                const std::uint64_t region_end = static_cast<std::uint64_t>(region.base) + region.size;
                return cursor >= region.base && cursor < region_end;
            }
        );
        if (match == regions.end() || !match->readable
            || (match->kind != mem::RegionKind::ram && match->kind != mem::RegionKind::rom)) {
            return false;
        }
        cursor = std::min(
            end, static_cast<std::uint64_t>(match->base) + match->size
        );
    }
    return true;
}

struct ProcessOutput {
    int exit_code{0};
    bool timed_out{false};
    std::string text;
};

Result<ProcessOutput> runProcess(
    const std::vector<std::string>& arguments,
    const std::chrono::milliseconds timeout
) {
#if defined(__APPLE__) || defined(__unix__)
    if (arguments.empty()) {
        return error(ErrorCategory::invalid_argument, "process argument list is empty");
    }

    int output_pipe[2]{};
    if (::pipe(output_pipe) != 0) {
        return error(ErrorCategory::io, "unable to create process output pipe: "
            + std::string(std::strerror(errno)));
    }

    posix_spawn_file_actions_t actions;
    if (posix_spawn_file_actions_init(&actions) != 0) {
        ::close(output_pipe[0]);
        ::close(output_pipe[1]);
        return error(ErrorCategory::io, "unable to initialize process file actions");
    }
    static_cast<void>(posix_spawn_file_actions_adddup2(&actions, output_pipe[1], STDOUT_FILENO));
    static_cast<void>(posix_spawn_file_actions_adddup2(&actions, output_pipe[1], STDERR_FILENO));
    static_cast<void>(posix_spawn_file_actions_addclose(&actions, output_pipe[0]));
    static_cast<void>(posix_spawn_file_actions_addclose(&actions, output_pipe[1]));

    std::vector<char*> argv;
    argv.reserve(arguments.size() + 1U);
    for (const std::string& argument : arguments) {
        argv.push_back(const_cast<char*>(argument.c_str()));
    }
    argv.push_back(nullptr);

    pid_t pid = 0;
    const int spawn_error = posix_spawnp(
        &pid, argv.front(), &actions, nullptr, argv.data(), environ
    );
    static_cast<void>(posix_spawn_file_actions_destroy(&actions));
    ::close(output_pipe[1]);
    if (spawn_error != 0) {
        ::close(output_pipe[0]);
        return error(ErrorCategory::io, "unable to start " + arguments.front() + ": "
            + std::string(std::strerror(spawn_error)));
    }

    const int flags = ::fcntl(output_pipe[0], F_GETFL, 0);
    if (flags >= 0) static_cast<void>(::fcntl(output_pipe[0], F_SETFL, flags | O_NONBLOCK));

    ProcessOutput output;
    int status = 0;
    bool child_done = false;
    bool pipe_done = false;
    const auto started = std::chrono::steady_clock::now();
    std::array<char, 4096> buffer{};

    while (!child_done || !pipe_done) {
        pollfd descriptor{output_pipe[0], POLLIN | POLLHUP, 0};
        static_cast<void>(::poll(&descriptor, 1, 20));
        while (!pipe_done) {
            const ssize_t count = ::read(output_pipe[0], buffer.data(), buffer.size());
            if (count > 0) {
                output.text.append(buffer.data(), static_cast<std::size_t>(count));
                continue;
            }
            if (count == 0) pipe_done = true;
            else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) pipe_done = true;
            break;
        }
        if (!child_done) {
            const pid_t waited = ::waitpid(pid, &status, WNOHANG);
            if (waited == pid) child_done = true;
        }
        if (!child_done && std::chrono::steady_clock::now() - started >= timeout) {
            output.timed_out = true;
            static_cast<void>(::kill(pid, SIGKILL));
            static_cast<void>(::waitpid(pid, &status, 0));
            child_done = true;
        }
    }
    ::close(output_pipe[0]);

    if (output.timed_out) output.exit_code = 124;
    else if (WIFEXITED(status)) output.exit_code = WEXITSTATUS(status);
    else if (WIFSIGNALED(status)) output.exit_code = 128 + WTERMSIG(status);
    else output.exit_code = 1;
    return output;
#else
    static_cast<void>(arguments);
    static_cast<void>(timeout);
    return error(ErrorCategory::unsupported,
                 "OpenOCD process capture is currently supported on POSIX hosts only");
#endif
}

long long processId() noexcept {
#if defined(__APPLE__) || defined(__unix__)
    return static_cast<long long>(::getpid());
#else
    return 0LL;
#endif
}

class TemporaryDirectory {
public:
    TemporaryDirectory() {
        const auto suffix = std::to_string(processId()) + "-"
            + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
        path_ = std::filesystem::temp_directory_path() / ("fil-stlink-" + suffix);
        std::filesystem::create_directory(path_);
    }
    ~TemporaryDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }
    TemporaryDirectory(const TemporaryDirectory&) = delete;
    TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;

    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

bool safeOpenOcdToken(const std::string_view text, const bool allow_slash) {
    return !text.empty() && std::all_of(text.begin(), text.end(), [allow_slash](const char value) {
        const auto byte = static_cast<unsigned char>(value);
        return std::isalnum(byte) != 0 || value == '_' || value == '-' || value == '.'
            || (allow_slash && value == '/');
    });
}

Result<std::string> tclPath(const std::filesystem::path& path) {
    const std::string text = std::filesystem::absolute(path).string();
    if (text.find_first_of("{}\n\r") != std::string::npos) {
        return error(ErrorCategory::invalid_argument,
                     "OpenOCD paths cannot contain braces or newlines: " + text);
    }
    return "{" + text + "}";
}

std::string openOcdRegisterName(const std::string_view canonical) {
    if (canonical == "r13") return "sp";
    if (canonical == "r14") return "lr";
    if (canonical == "r15") return "pc";
    if (canonical == "xpsr") return "xPSR";
    return std::string(canonical);
}

Result<std::uint32_t> parseMarker(
    const std::string_view output,
    const std::string_view marker
) {
    const std::size_t marker_at = output.find(marker);
    if (marker_at == std::string_view::npos) {
        return error(ErrorCategory::parse, "OpenOCD output omitted marker " + std::string(marker));
    }
    const std::size_t line_end = output.find('\n', marker_at);
    const std::string_view line = output.substr(marker_at, line_end - marker_at);
    const std::size_t value_at = line.rfind("0x");
    if (value_at == std::string_view::npos) {
        return error(ErrorCategory::parse, "OpenOCD marker has no hexadecimal value: "
            + std::string(line));
    }
    const std::string_view digits = line.substr(value_at + 2U);
    std::uint32_t value = 0U;
    const auto parsed = std::from_chars(digits.data(), digits.data() + digits.size(), value, 16);
    if (parsed.ec != std::errc{}) {
        return error(ErrorCategory::parse, "invalid OpenOCD marker value: " + std::string(line));
    }
    return value;
}

Result<std::vector<std::uint8_t>> readBinaryFile(
    const std::filesystem::path& path,
    const std::uint32_t expected_size
) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return error(ErrorCategory::io, "OpenOCD did not create memory dump: " + path.string());
    }
    const std::vector<char> raw{
        std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()
    };
    std::vector<std::uint8_t> bytes;
    bytes.reserve(raw.size());
    std::transform(raw.begin(), raw.end(), std::back_inserter(bytes), [](const char byte) {
        return static_cast<std::uint8_t>(static_cast<unsigned char>(byte));
    });
    if (bytes.size() != expected_size) {
        return error(ErrorCategory::io, "OpenOCD memory dump has unexpected size: "
            + path.string());
    }
    return bytes;
}

} // namespace

std::span<const std::string_view> comparableRegisterNames() noexcept {
    return register_names;
}

Result<MemoryRange> parseMemoryRange(const std::string_view text) {
    const std::size_t separator = text.find(':');
    if (separator == 0U || separator == std::string_view::npos
        || separator + 1U == text.size() || text.find(':', separator + 1U) != std::string_view::npos) {
        return error(ErrorCategory::invalid_argument, "memory range must be ADDRESS:LENGTH");
    }
    auto address = config::parseUnsigned(text.substr(0U, separator));
    auto size = config::parseUnsigned(text.substr(separator + 1U));
    if (!address || !size || address.value() > std::numeric_limits<std::uint32_t>::max()
        || size.value() == 0U || size.value() > std::numeric_limits<std::uint32_t>::max()) {
        return error(ErrorCategory::invalid_argument, "memory range is outside the 32-bit target space");
    }
    const MemoryRange range{
        static_cast<std::uint32_t>(address.value()), static_cast<std::uint32_t>(size.value())
    };
    if (!fil::rangeFits(range.address, range.size, std::uint64_t{1U} << 32U)) {
        return error(ErrorCategory::invalid_argument, "memory range wraps the 32-bit target space");
    }
    return range;
}

Result<Snapshot> captureEmulator(
    const sim::Board& board,
    const sim::BoardRunResult& result,
    const std::span<const MemoryRange> ranges
) {
    Snapshot snapshot;
    snapshot.source = "emulator";
    snapshot.stop_reason = std::string(sim::boardStopReasonName(result.reason));
    snapshot.instructions = result.instructions;

    const cpu::CpuState& state = board.cpu().state();
    snapshot.registers.reserve(register_names.size());
    for (std::size_t index = 0U; index < state.r.size(); ++index) {
        std::uint32_t value = state.r[index];
        // OpenOCD reports PC at the BKPT instruction, while normal execution
        // state already points past it. Preserve the common halted-boundary view.
        if (index == 15U && result.reason == sim::BoardStopReason::breakpoint) {
            value = result.diagnostic.instruction_address;
        }
        snapshot.registers.push_back({"r" + std::to_string(index), value});
    }
    snapshot.registers.push_back({"xpsr", state.xpsr});
    snapshot.registers.push_back({"msp", state.msp});
    snapshot.registers.push_back({"psp", state.psp});
    snapshot.registers.push_back({"primask", state.primask});
    snapshot.registers.push_back({"basepri", state.basepri});
    snapshot.registers.push_back({"faultmask", state.faultmask});
    snapshot.registers.push_back({"control", state.control});

    const std::vector<mem::MemoryRegionInfo> regions = board.memory().regions();
    snapshot.memory.reserve(ranges.size());
    for (const MemoryRange range : ranges) {
        if (!rangeIsInspectable(regions, range)) {
            return error(ErrorCategory::invalid_argument,
                         "snapshot memory range is not wholly readable RAM or ROM: "
                         + hex32(range.address));
        }
        MemoryValue value;
        value.range = range;
        value.bytes.reserve(range.size);
        for (std::uint32_t offset = 0U; offset < range.size; ++offset) {
            const auto byte = board.memory().read8(range.address + offset);
            if (!byte) {
                return error(ErrorCategory::runtime,
                             "snapshot memory read fault at " + hex32(range.address + offset));
            }
            value.bytes.push_back(byte.value());
        }
        snapshot.memory.push_back(std::move(value));
    }
    return snapshot;
}

Result<Snapshot> captureStlink(
    const std::filesystem::path& elf_path,
    const std::span<const MemoryRange> ranges,
    const StlinkCaptureOptions& options
) {
    if (options.timeout <= std::chrono::milliseconds::zero()) {
        return error(ErrorCategory::invalid_argument, "ST-Link timeout must be positive");
    }
    if (!safeOpenOcdToken(options.interface_config, true)
        || !safeOpenOcdToken(options.target_config, true)
        || (options.serial && !safeOpenOcdToken(*options.serial, false))) {
        return error(ErrorCategory::invalid_argument,
                     "OpenOCD config names and probe serial must contain only safe token characters");
    }
    if (!std::filesystem::is_regular_file(elf_path)) {
        return error(ErrorCategory::io, "firmware ELF does not exist: " + elf_path.string());
    }

    TemporaryDirectory temporary;
    const auto elf_word = tclPath(elf_path);
    if (!elf_word) return elf_word.error();
    const auto script_path = temporary.path() / "capture.cfg";
    std::ofstream script(script_path, std::ios::binary | std::ios::trunc);
    if (!script) {
        return error(ErrorCategory::io, "unable to create temporary OpenOCD script");
    }
    script << "source [find " << options.interface_config << "]\n"
           << "source [find " << options.target_config << "]\n"
           << "gdb_port disabled\n"
           << "tcl_port disabled\n"
           << "telnet_port disabled\n";
    if (options.serial) script << "adapter serial " << *options.serial << "\n";
    script << "adapter speed 1800\n"
           << "init\n"
           << "reset halt\n"
           << "set fil_chip_id [mrw 0xe0042000]\n"
           << "echo FIL_CHIP_ID=$fil_chip_id\n"
           << "if {[expr {$fil_chip_id & 0x0fff}] != 0x0469} { error \"unexpected target chip\" }\n";
    if (options.flash) {
        script << "flash write_image erase " << elf_word.value() << "\n"
               << "verify_image " << elf_word.value() << "\n";
    }
    script << "reset halt\n";
    if (options.stop_address) {
        script << "bp " << hex32(*options.stop_address) << " 2 hw\n";
    }
    script << "resume\n"
           << "wait_halt " << options.timeout.count() << "\n";
    for (const std::string_view name : register_names) {
        script << "echo FIL_REG_" << name << "=[reg " << openOcdRegisterName(name) << "]\n";
    }
    for (std::size_t index = 0U; index < ranges.size(); ++index) {
        const auto dump_path = tclPath(temporary.path() / ("memory-" + std::to_string(index) + ".bin"));
        if (!dump_path) return dump_path.error();
        script << "dump_image " << dump_path.value() << ' ' << hex32(ranges[index].address)
               << ' ' << ranges[index].size << "\n";
    }
    script << "reset halt\nshutdown\n";
    script.close();
    if (!script) {
        return error(ErrorCategory::io, "failed while writing temporary OpenOCD script");
    }

    const auto process = runProcess(
        {options.openocd.string(), "-f", script_path.string()}, options.timeout + std::chrono::seconds(5)
    );
    if (!process) return process.error();
    if (process.value().timed_out) {
        return error(ErrorCategory::runtime, "OpenOCD timed out while waiting for the target");
    }
    if (process.value().exit_code != 0) {
        return error(ErrorCategory::runtime, "OpenOCD capture failed (exit "
            + std::to_string(process.value().exit_code) + "):\n" + process.value().text);
    }

    const auto chip_id = parseMarker(process.value().text, "FIL_CHIP_ID=");
    if (!chip_id) return chip_id.error();
    if ((chip_id.value() & 0x0fffU) != 0x0469U) {
        return error(ErrorCategory::unsupported, "attached target is not an STM32G47x/G48x (ID "
            + hex32(chip_id.value()) + ")");
    }

    Snapshot snapshot;
    snapshot.source = "stlink";
    snapshot.stop_reason = options.stop_address ? "target_reached" : "breakpoint";
    snapshot.registers.reserve(register_names.size());
    for (const std::string_view name : register_names) {
        const auto value = parseMarker(process.value().text, "FIL_REG_" + std::string(name) + "=");
        if (!value) return value.error();
        snapshot.registers.push_back({std::string(name), value.value()});
    }
    snapshot.memory.reserve(ranges.size());
    for (std::size_t index = 0U; index < ranges.size(); ++index) {
        auto bytes = readBinaryFile(
            temporary.path() / ("memory-" + std::to_string(index) + ".bin"), ranges[index].size
        );
        if (!bytes) return bytes.error();
        snapshot.memory.push_back({ranges[index], std::move(bytes.value())});
    }
    return snapshot;
}

Comparison compare(
    const Snapshot& emulator,
    const Snapshot& hardware,
    const std::span<const std::string> selected_registers
) {
    Comparison result;
    for (const std::string& name : selected_registers) {
        const RegisterValue* left = findRegister(emulator, name);
        const RegisterValue* right = findRegister(hardware, name);
        if (left == nullptr || right == nullptr || left->value != right->value) {
            result.register_differences.push_back({
                name,
                left == nullptr ? std::nullopt : std::optional<std::uint32_t>(left->value),
                right == nullptr ? std::nullopt : std::optional<std::uint32_t>(right->value),
            });
        }
    }
    for (const MemoryValue& left : emulator.memory) {
        const MemoryValue* right = findMemory(hardware, left.range);
        if (right == nullptr || left.bytes != right->bytes) {
            result.memory_differences.push_back({
                left.range, left.bytes, right == nullptr ? std::vector<std::uint8_t>{} : right->bytes,
            });
        }
    }
    for (const MemoryValue& right : hardware.memory) {
        if (findMemory(emulator, right.range) == nullptr) {
            result.memory_differences.push_back({right.range, {}, right.bytes});
        }
    }
    return result;
}

void writeJson(const Snapshot& snapshot, std::ostream& output) {
    output << "{\n  \"schema_version\": 1,\n  \"source\": ";
    writeJsonString(output, snapshot.source);
    output << ",\n  \"stop_reason\": ";
    writeJsonString(output, snapshot.stop_reason);
    output << ",\n  \"instructions\": " << snapshot.instructions << ",\n  \"registers\": {";
    for (std::size_t index = 0U; index < snapshot.registers.size(); ++index) {
        output << (index == 0U ? "\n    " : ",\n    ");
        writeJsonString(output, snapshot.registers[index].name);
        output << ": ";
        writeJsonString(output, hex32(snapshot.registers[index].value));
    }
    if (!snapshot.registers.empty()) output << '\n';
    output << "  },\n  \"memory\": [";
    for (std::size_t index = 0U; index < snapshot.memory.size(); ++index) {
        const MemoryValue& value = snapshot.memory[index];
        output << (index == 0U ? "\n    " : ",\n    ")
               << "{\"address\": ";
        writeJsonString(output, hex32(value.range.address));
        output << ", \"size\": " << value.range.size << ", \"bytes\": ";
        writeJsonString(output, bytesToHex(value.bytes));
        output << '}';
    }
    if (snapshot.memory.empty()) output << "]\n}\n";
    else output << "\n  ]\n}\n";
}

void writeJson(const Comparison& comparison, std::ostream& output) {
    output << "{\n  \"schema_version\": 1,\n  \"match\": "
           << (comparison.matches() ? "true" : "false")
           << ",\n  \"register_differences\": [";
    for (std::size_t index = 0U; index < comparison.register_differences.size(); ++index) {
        const RegisterDifference& difference = comparison.register_differences[index];
        output << (index == 0U ? "\n    " : ",\n    ") << "{\"register\": ";
        writeJsonString(output, difference.name);
        output << ", \"emulator\": ";
        if (difference.emulator) writeJsonString(output, hex32(*difference.emulator));
        else output << "null";
        output << ", \"hardware\": ";
        if (difference.hardware) writeJsonString(output, hex32(*difference.hardware));
        else output << "null";
        output << '}';
    }
    if (comparison.register_differences.empty()) {
        output << "],\n  \"memory_differences\": [";
    } else {
        output << "\n  ],\n  \"memory_differences\": [";
    }
    for (std::size_t index = 0U; index < comparison.memory_differences.size(); ++index) {
        const MemoryDifference& difference = comparison.memory_differences[index];
        output << (index == 0U ? "\n    " : ",\n    ") << "{\"address\": ";
        writeJsonString(output, hex32(difference.range.address));
        output << ", \"size\": " << difference.range.size << ", \"emulator\": ";
        writeJsonString(output, bytesToHex(difference.emulator));
        output << ", \"hardware\": ";
        writeJsonString(output, bytesToHex(difference.hardware));
        output << '}';
    }
    if (comparison.memory_differences.empty()) output << "]\n}\n";
    else output << "\n  ]\n}\n";
}

} // namespace fil::hardware
