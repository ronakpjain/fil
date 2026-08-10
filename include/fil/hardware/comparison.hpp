#pragma once

/** @file comparison.hpp
 *  @brief Deterministic emulator and STM32G4 hardware snapshot comparison.
 */

#include "fil/common/result.hpp"
#include "fil/sim/board.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace fil::hardware {

/** @brief One non-MMIO target memory interval included in a snapshot. */
struct MemoryRange {
    std::uint32_t address{0}; ///< First target byte.
    std::uint32_t size{0};    ///< Number of bytes; must be nonzero and non-wrapping.
};

/** @brief One named 32-bit architectural register value. */
struct RegisterValue {
    std::string name;         ///< Canonical lowercase register name.
    std::uint32_t value{0};   ///< Raw architectural bit pattern.
};

/** @brief Captured bytes for one requested target memory interval. */
struct MemoryValue {
    MemoryRange range;                 ///< Captured target interval.
    std::vector<std::uint8_t> bytes;   ///< Bytes in ascending address order.
};

/** @brief Machine-readable architectural state at a deterministic stop boundary. */
struct Snapshot {
    std::string source;                       ///< `emulator` or `stlink`.
    std::string stop_reason;                  ///< Stable boundary classification.
    std::uint64_t instructions{0};            ///< Emulator count; zero when unavailable.
    std::vector<RegisterValue> registers;     ///< Canonically ordered register values.
    std::vector<MemoryValue> memory;           ///< Requested memory intervals.
};

/** @brief Configuration for one synchronous OpenOCD/ST-Link capture. */
struct StlinkCaptureOptions {
    std::filesystem::path openocd{"openocd"}; ///< OpenOCD executable or absolute path.
    std::string interface_config{"interface/stlink.cfg"}; ///< OpenOCD probe configuration.
    std::string target_config{"target/stm32g4x.cfg"};     ///< OpenOCD target configuration.
    std::optional<std::string> serial;          ///< Optional ST-Link serial selector.
    std::optional<std::uint32_t> stop_address;  ///< Hardware breakpoint; BKPT when empty.
    std::chrono::milliseconds timeout{10'000};  ///< Target halt and process deadline.
    bool flash{false};                          ///< Erase, program, and verify before running.
};

/** @brief One differing register value. */
struct RegisterDifference {
    std::string name;                         ///< Canonical register name.
    std::optional<std::uint32_t> emulator;    ///< Emulator value, or empty when missing.
    std::optional<std::uint32_t> hardware;    ///< Hardware value, or empty when missing.
};

/** @brief One differing or missing memory interval. */
struct MemoryDifference {
    MemoryRange range;                       ///< Differing interval.
    std::vector<std::uint8_t> emulator;      ///< Emulator bytes, empty when missing.
    std::vector<std::uint8_t> hardware;      ///< Hardware bytes, empty when missing.
};

/** @brief Deterministic comparison result. */
struct Comparison {
    std::vector<RegisterDifference> register_differences; ///< Selected register mismatches.
    std::vector<MemoryDifference> memory_differences;     ///< Requested memory mismatches.

    /** @brief Whether every selected register and memory byte matched. */
    [[nodiscard]] bool matches() const noexcept {
        return register_differences.empty() && memory_differences.empty();
    }
};

/** @brief Canonical register names emitted by both capture backends. */
[[nodiscard]] std::span<const std::string_view> comparableRegisterNames() noexcept;

/** @brief Parses an `ADDRESS:LENGTH` memory-range option. */
[[nodiscard]] Result<MemoryRange> parseMemoryRange(std::string_view text);

/** @brief Captures CPU and requested non-MMIO memory from an emulator board. */
[[nodiscard]] Result<Snapshot> captureEmulator(
    const sim::Board& board,
    const sim::BoardRunResult& result,
    std::span<const MemoryRange> ranges
);

/**
 * @brief Optionally flashes an ELF, runs it through ST-Link, and captures state.
 *
 * This operation resets and controls the attached target. Flash is modified only
 * when options.flash is true.
 */
[[nodiscard]] Result<Snapshot> captureStlink(
    const std::filesystem::path& elf_path,
    std::span<const MemoryRange> ranges,
    const StlinkCaptureOptions& options
);

/** @brief Compares selected registers and all requested memory intervals. */
[[nodiscard]] Comparison compare(
    const Snapshot& emulator,
    const Snapshot& hardware,
    std::span<const std::string> register_names
);

/** @brief Writes one stable JSON snapshot object. */
void writeJson(const Snapshot& snapshot, std::ostream& output);

/** @brief Writes one stable JSON comparison object. */
void writeJson(const Comparison& comparison, std::ostream& output);

} // namespace fil::hardware
