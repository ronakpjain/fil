#pragma once

#include "fil/config/config.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace fil::test {

/** @brief Collision-safe temporary directory removed when a test scope exits. */
class TemporaryDirectory {
public:
    explicit TemporaryDirectory(const std::string_view prefix) {
        static std::atomic<std::uint64_t> sequence{0U};
        const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
        const auto base = std::filesystem::temp_directory_path();
        for (std::uint64_t attempt = 0U; attempt < 1024U; ++attempt) {
            static_cast<void>(attempt);
            root_ = base / (std::string(prefix) + "-"
                + std::to_string(tick) + "-"
                + std::to_string(sequence.fetch_add(1U, std::memory_order_relaxed)));
            std::error_code error;
            if (std::filesystem::create_directories(root_, error)) return;
            if (error) {
                throw std::runtime_error(
                    "unable to create temporary test directory: " + error.message()
                );
            }
        }
        throw std::runtime_error("unable to allocate a unique temporary test directory");
    }

    ~TemporaryDirectory() {
        std::error_code error;
        std::filesystem::remove_all(root_, error);
    }

    TemporaryDirectory(const TemporaryDirectory&) = delete;
    TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;
    TemporaryDirectory(TemporaryDirectory&&) = delete;
    TemporaryDirectory& operator=(TemporaryDirectory&&) = delete;

    [[nodiscard]] const std::filesystem::path& root() const noexcept {
        return root_;
    }

    [[nodiscard]] std::filesystem::path write(
        const std::filesystem::path& relative,
        const std::string_view contents
    ) const {
        const auto path = root_ / relative;
        std::error_code error;
        std::filesystem::create_directories(path.parent_path(), error);
        if (error) {
            throw std::runtime_error(
                "unable to create temporary test path: " + error.message()
            );
        }
        std::ofstream output(path, std::ios::binary | std::ios::trunc);
        if (!output) {
            throw std::runtime_error("unable to write temporary test file: " + path.string());
        }
        output << contents;
        return path;
    }

private:
    std::filesystem::path root_;
};

/** @brief Returns the repository's standard synthetic board configuration. */
inline config::BoardConfig fixtureBoardConfig(const std::string_view name = "fixture") {
    config::BoardConfig config;
    config.name = name;
    config.mcu_path = std::filesystem::path(FIL_SOURCE_DIR)
        / "configs/mcus/stm32g474retx.json";
    config.elf_path = std::filesystem::path(FIL_SOURCE_DIR)
        / "tests/fixtures/elf/split_image.elf";
    config.vector_base = 0x08000000U;
    return config;
}

/**
 * @brief Finds the trimmed value of the first `key:` line in a report.
 *
 * @param text Complete CLI output.
 * @param key Field name without the colon.
 * @return Trimmed value, or nothing when the key is absent.
 */
inline std::optional<std::string> reportValue(
    const std::string_view text,
    const std::string_view key
) {
    std::size_t line_start = 0U;
    while (line_start <= text.size()) {
        std::size_t line_end = text.find('\n', line_start);
        if (line_end == std::string_view::npos) line_end = text.size();
        const std::string_view line = text.substr(line_start, line_end - line_start);
        const std::size_t colon = line.find(':');
        if (colon != std::string_view::npos) {
            std::string_view name = line.substr(0U, colon);
            while (!name.empty() && name.front() == ' ') name.remove_prefix(1U);
            while (!name.empty() && name.back() == ' ') name.remove_suffix(1U);
            if (name == key) {
                std::string_view value = line.substr(colon + 1U);
                while (!value.empty() && value.front() == ' ') value.remove_prefix(1U);
                while (!value.empty() && value.back() == ' ') value.remove_suffix(1U);
                return std::string(value);
            }
        }
        if (line_end == text.size()) break;
        line_start = line_end + 1U;
    }
    return std::nullopt;
}

} // namespace fil::test

// Common isolated event/trace/system state for peripheral integration tests.
#include "fil/cortexm/system_control.hpp"
#include "fil/sim/event_loop.hpp"
#include "fil/sim/trace.hpp"

namespace fil::test {

struct SimulationRig {
    sim::EventLoop events;
    sim::TraceRecorder trace;
    cortexm::SystemControl system;
};

} // namespace fil::test
