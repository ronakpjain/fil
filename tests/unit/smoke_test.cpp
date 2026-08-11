#include "fil/cli/cli.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>

namespace {

/// @brief Verifies help output and status.
TEST(SmokeTest, HelpIsSuccessful) {
    std::ostringstream out;
    std::ostringstream err;
    const std::string_view args[] = {"--help"};

    const auto result = fil::cli::run(args, out, err);

    EXPECT_TRUE(result == fil::cli::ExitCode::success) << "--help returns success";
    EXPECT_TRUE(out.str().find("Usage:") != std::string::npos) << "--help prints usage";
    EXPECT_TRUE(err.str().empty()) << "--help does not print an error";
}

/// @brief Verifies unknown commands produce a usage error.
TEST(SmokeTest, UnknownCommandIsAUsageError) {
    std::ostringstream out;
    std::ostringstream err;
    const std::string_view args[] = {"not-a-command"};

    const auto result = fil::cli::run(args, out, err);

    EXPECT_TRUE(result == fil::cli::ExitCode::usage_error) << "unknown command returns usage error";
    EXPECT_TRUE(out.str().empty()) << "unknown command does not print normal output";
    EXPECT_TRUE(err.str().find("unknown command") != std::string::npos)
        << "unknown command prints a diagnostic";
}

TEST(SmokeTest, RejectsInvalidWatchRefreshInterval) {
    const std::string_view args[]{"watch-network", "missing.json", "--refresh-ms", "0"};
    std::ostringstream out;
    std::ostringstream err;

    const auto result = fil::cli::run(args, out, err);

    EXPECT_EQ(result, fil::cli::ExitCode::usage_error);
    EXPECT_NE(err.str().find("--refresh-ms"), std::string::npos);
}

TEST(SmokeTest, DisassemblesSyntheticWindow) {
    const std::string path = (
        std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/startup_runtime.elf"
    ).string();
    const std::string_view args[]{
        "disasm-window", path, "--addr", "0x08000008", "--count", "2",
    };
    std::ostringstream out;
    std::ostringstream err;
    const auto result = fil::cli::run(args, out, err);
    EXPECT_TRUE(result == fil::cli::ExitCode::success)
        << "disasm-window decodes a synthetic firmware range";
    EXPECT_TRUE(out.str().find("0x08000008: 480c") != std::string::npos &&
                out.str().find("ldr") != std::string::npos)
        << "disasm-window prints addresses, raw encodings, and semantic names";
    EXPECT_TRUE(err.str().empty()) << "successful disasm-window has no error output";
}

TEST(SmokeTest, RequiresExplicitBreakpointAcceptance) {
    const std::filesystem::path config_path =
        std::filesystem::temp_directory_path() / "fil-cli-breakpoint-test.json";
    const std::filesystem::path mcu =
        std::filesystem::path(FIL_SOURCE_DIR) / "configs/mcus/stm32g474retx.json";
    const std::filesystem::path elf =
        std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";
    {
        std::ofstream config(config_path, std::ios::binary | std::ios::trunc);
        config << "{\n"
               << "  \"schema_version\": 1,\n"
               << "  \"name\": \"cli-breakpoint\",\n"
               << "  \"mcu\": \"" << mcu.string() << "\",\n"
               << "  \"elf\": \"" << elf.string() << "\"\n"
               << "}\n";
    }
    const std::string config_text = config_path.string();
    const std::string_view default_args[]{
        "run", config_text, "--duration-ms", "0", "--max-instructions", "20",
        "--no-detect-spin",
    };
    std::ostringstream default_out;
    std::ostringstream default_err;
    EXPECT_TRUE(
        fil::cli::run(default_args, default_out, default_err) == fil::cli::ExitCode::runtime_error)
        << "run rejects an unrequested firmware BKPT";
    EXPECT_TRUE(default_err.str().find("breakpoint") != std::string::npos)
        << "unrequested BKPT produces an explicit diagnostic";

    const std::string_view allowed_args[]{
        "run", config_text, "--duration-ms", "0", "--max-instructions", "20",
        "--no-detect-spin", "--allow-breakpoint",
    };
    std::ostringstream allowed_out;
    std::ostringstream allowed_err;
    EXPECT_TRUE(
        fil::cli::run(allowed_args, allowed_out, allowed_err) == fil::cli::ExitCode::success)
        << "--allow-breakpoint explicitly accepts a firmware BKPT";
    EXPECT_TRUE(allowed_err.str().empty()) << "an explicitly accepted BKPT has no error diagnostic";
    std::error_code remove_error;
    std::filesystem::remove(config_path, remove_error);
}

} // namespace
