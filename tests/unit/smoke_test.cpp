#include "fil/cli/cli.hpp"

#include "../fixture_support.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

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
    EXPECT_EQ(out.str().find("--no-ram-capsules"), std::string::npos);
    EXPECT_NE(out.str().find("--adc-decimation"), std::string::npos);
    EXPECT_EQ(out.str().find("--deferred-prefixes"), std::string::npos);
    EXPECT_NE(out.str().find("trades fidelity for speed"), std::string::npos);
    EXPECT_EQ(out.str().find("--no-jit"), std::string::npos);
    EXPECT_NE(out.str().find("--jit-stats"), std::string::npos);
    EXPECT_NE(out.str().find("debug <board.json>"), std::string::npos);
    EXPECT_NE(out.str().find("--telnet-port"), std::string::npos);
}

/// @brief Verifies debug command argument validation before any I/O starts.
TEST(SmokeTest, DebugCommandValidatesItsInputs) {
    {
        std::ostringstream out;
        std::ostringstream err;
        const std::string_view args[] = {"debug"};
        EXPECT_EQ(fil::cli::run(args, out, err), fil::cli::ExitCode::usage_error);
        EXPECT_NE(err.str().find("requires a board config"), std::string::npos);
    }
    {
        std::ostringstream out;
        std::ostringstream err;
        const std::string_view args[] = {"debug", "missing-board.json", "--bogus"};
        EXPECT_EQ(fil::cli::run(args, out, err), fil::cli::ExitCode::usage_error);
        EXPECT_NE(err.str().find("unknown debug option"), std::string::npos);
    }
    {
        std::ostringstream out;
        std::ostringstream err;
        const std::string_view args[] = {"debug", "missing-board.json", "--gdb-port", "70000"};
        EXPECT_EQ(fil::cli::run(args, out, err), fil::cli::ExitCode::usage_error);
        EXPECT_NE(err.str().find("--gdb-port requires a TCP port"), std::string::npos);
    }
    {
        std::ostringstream out;
        std::ostringstream err;
        const std::string_view args[] = {"debug", "missing-board.json", "--telnet-port", "70000"};
        EXPECT_EQ(fil::cli::run(args, out, err), fil::cli::ExitCode::usage_error);
        EXPECT_NE(err.str().find("--telnet-port requires a TCP port"), std::string::npos);
    }
    {
        std::ostringstream out;
        std::ostringstream err;
        const std::string_view args[] = {"debug", "missing-board.json", "--no-telnet"};
        EXPECT_EQ(fil::cli::run(args, out, err), fil::cli::ExitCode::config_error);
    }
    {
        const std::string network = (std::filesystem::path(FIL_SOURCE_DIR)
            / "configs/networks/per_vehicle.json").string();
        std::ostringstream out;
        std::ostringstream err;
        const std::string_view args[] = {"debug", network};
        EXPECT_EQ(fil::cli::run(args, out, err), fil::cli::ExitCode::usage_error);
        EXPECT_NE(err.str().find("network debugging is not implemented"), std::string::npos);
    }
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

TEST(SmokeTest, ExecutionSlowdownSwitchesAreRejected) {
    for (const std::string_view command : {"run", "watch", "serve"}) {
        for (const std::string_view option : {
                 "--no-ram-capsules", "--deferred-prefixes", "--ram-capsules"}) {
            if (command == "run" && option == "--no-ram-capsules") continue;
            std::vector<std::string_view> args{command, "missing-jit-config.json", option};
            std::ostringstream out;
            std::ostringstream err;
            const auto result = fil::cli::run(args, out, err);
            EXPECT_EQ(result, fil::cli::ExitCode::usage_error) << command << " " << option;
            EXPECT_NE(err.str().find("unknown"), std::string::npos) << command << " " << option;
        }
    }
}

TEST(SmokeTest, FidelityReducingAdcDecimationRemainsExplicitlyAvailable) {
    for (const std::string_view command : {"run", "watch", "serve"}) {
        std::vector<std::string_view> args{command, "missing-adc-config.json", "--adc-decimation", "4"};
        std::ostringstream out;
        std::ostringstream err;
        EXPECT_EQ(fil::cli::run(args, out, err), fil::cli::ExitCode::config_error) << command;
        EXPECT_EQ(err.str().find("unknown"), std::string::npos) << command;
    }
}

TEST(SmokeTest, RejectsInvalidWatchRefreshInterval) {
    const std::string_view args[]{"watch", "missing.json", "--refresh-ms", "0"};
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

/// @brief Verifies breakpoint acceptance and symbol-based stopping.
TEST(SmokeTest, HonorsRunStopControls) {
    fil::test::TemporaryDirectory directory{"fil-cli-breakpoint-test"};
    const auto mcu = std::filesystem::path(FIL_SOURCE_DIR)
        / "configs/mcus/stm32g474retx.json";
    const auto elf = std::filesystem::path(FIL_SOURCE_DIR)
        / "tests/fixtures/elf/split_image.elf";
    const std::string config_contents = "{\n"
        "  \"schema_version\": 1,\n"
        "  \"name\": \"cli-breakpoint\",\n"
        "  \"mcu\": \"" + mcu.string() + "\",\n"
        "  \"elf\": \"" + elf.string() + "\"\n"
        "}\n";
    const std::string config_text = directory.write("config.json", config_contents).string();
    const std::string_view default_args[]{
        "run", config_text, "--duration", "0", "--instruction-budget", "20",
        "--no-detect-spin",
    };
    std::ostringstream default_out;
    std::ostringstream default_err;
    EXPECT_TRUE(
        fil::cli::run(default_args, default_out, default_err) == fil::cli::ExitCode::runtime_error)
        << "run rejects an unrequested firmware BKPT";
    EXPECT_TRUE(default_err.str().find("breakpoint") != std::string::npos)
        << "unrequested BKPT produces an explicit diagnostic";
    EXPECT_TRUE(default_out.str().find("jit:") == std::string::npos)
        << "JIT diagnostics stay hidden without --jit-stats";

    const std::string_view allowed_args[]{
        "run", config_text, "--duration", "0", "--instruction-budget", "20",
        "--no-detect-spin", "--allow-breakpoint", "--jit-stats",
    };
    std::ostringstream allowed_out;
    std::ostringstream allowed_err;
    EXPECT_TRUE(
        fil::cli::run(allowed_args, allowed_out, allowed_err) == fil::cli::ExitCode::success)
        << "--allow-breakpoint explicitly accepts a firmware BKPT";
    EXPECT_TRUE(allowed_err.str().empty()) << "an explicitly accepted BKPT has no error diagnostic";
    EXPECT_NE(allowed_out.str().find("jit:"), std::string::npos);
    EXPECT_NE(allowed_out.str().find("single:"), std::string::npos);
    EXPECT_NE(allowed_out.str().find("fast="), std::string::npos);
    EXPECT_NE(allowed_out.str().find("generic="), std::string::npos);

    const std::string_view symbol_args[]{"run", config_text, "--stop-at-symbol", "Reset_Handler"};
    std::ostringstream symbol_out;
    std::ostringstream symbol_err;
    EXPECT_EQ(fil::cli::run(symbol_args, symbol_out, symbol_err), fil::cli::ExitCode::success);
    EXPECT_EQ(fil::test::reportValue(symbol_out.str(), "stop"), "target-reached");
    EXPECT_EQ(fil::test::reportValue(symbol_out.str(), "instructions"), "0");
    EXPECT_TRUE(symbol_err.str().empty());

    const std::string_view missing_symbol_args[]{
        "run", config_text, "--stop-at-symbol", "NoSuchSymbol",
    };
    std::ostringstream missing_out;
    std::ostringstream missing_err;
    EXPECT_EQ(
        fil::cli::run(missing_symbol_args, missing_out, missing_err),
        fil::cli::ExitCode::usage_error
    );
    EXPECT_NE(missing_err.str().find("symbol not found: NoSuchSymbol"), std::string::npos);
}

} // namespace
