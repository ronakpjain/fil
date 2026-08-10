#pragma once

#include "fil/config/config.hpp"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
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

} // namespace fil::test
