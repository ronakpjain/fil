#include "fil/cli/cli.hpp"
#include "../test_support.hpp"

#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string_view>

/** @brief Runs configuration unit tests defined in config_test.cpp. */
void runConfigTests();
/** @brief Runs integrated board execution tests. */
void runBoardTests();
/** @brief Runs Cortex-M system-control unit tests. */
void runCortexMTests();
/** @brief Runs architectural exception entry/return unit tests. */
void runExceptionTests();
/** @brief Runs deterministic virtual CAN bus unit tests. */
void runCanBusTests();
/** @brief Runs ELF unit tests defined in elf_loader_test.cpp. */
void runElfLoaderTests();
/** @brief Runs deterministic event-loop and trace tests. */
void runEventLoopTests();
/** @brief Runs FDCAN controller and message-RAM tests. */
void runFdcanTests();
/** @brief Runs memory bus unit tests defined in memory_bus_test.cpp. */
void runMemoryBusTests();
/** @brief Runs STM32G4 peripheral model tests. */
void runPeripheralTests();
/** @brief Runs integrated STM32G4 MMIO routing tests. */
void runStm32G4Tests();
/** @brief Runs the synthetic C-runtime startup integration fixture. */
void runStartupRuntimeTests();
/** @brief Runs the hermetic SVC/PendSV/PSP/SysTick firmware fixture. */
void runSchedulerTickTests();
/** @brief Runs the compiled Cortex-M4F hard-float firmware fixture. */
void runHardFloatFirmwareTests();
/** @brief Runs deterministic multi-board world tests. */
void runWorldTests();
/** @brief Runs concurrent virtual-time scheduler regression tests. */
void runWorldTimeTests();

namespace {

/// @brief Verifies help output and status.
void helpIsSuccessful() {
    std::ostringstream out;
    std::ostringstream err;
    const std::string_view args[] = {"--help"};

    const auto result = fil::cli::run(args, out, err);

    fil::test::check(result == fil::cli::ExitCode::success, "--help returns success");
    fil::test::check(out.str().find("Usage:") != std::string::npos, "--help prints usage");
    fil::test::check(err.str().empty(), "--help does not print an error");
}

/// @brief Verifies unknown commands produce a usage error.
void unknownCommandIsAUsageError() {
    std::ostringstream out;
    std::ostringstream err;
    const std::string_view args[] = {"not-a-command"};

    const auto result = fil::cli::run(args, out, err);

    fil::test::check(result == fil::cli::ExitCode::usage_error, "unknown command returns usage error");
    fil::test::check(out.str().empty(), "unknown command does not print normal output");
    fil::test::check(err.str().find("unknown command") != std::string::npos, "unknown command prints a diagnostic");
}

void disassemblesSyntheticWindow() {
    const std::string path = (
        std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/startup_runtime.elf"
    ).string();
    const std::string_view args[]{
        "disasm-window", path, "--addr", "0x08000008", "--count", "2",
    };
    std::ostringstream out;
    std::ostringstream err;
    const auto result = fil::cli::run(args, out, err);
    fil::test::check(result == fil::cli::ExitCode::success,
                     "disasm-window decodes a synthetic firmware range");
    fil::test::check(out.str().find("0x08000008: 480c") != std::string::npos
                         && out.str().find("ldr") != std::string::npos,
                     "disasm-window prints addresses, raw encodings, and semantic names");
    fil::test::check(err.str().empty(), "successful disasm-window has no error output");
}

void requiresExplicitBreakpointAcceptance() {
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
    fil::test::check(
        fil::cli::run(default_args, default_out, default_err) == fil::cli::ExitCode::runtime_error,
        "run rejects an unrequested firmware BKPT"
    );
    fil::test::check(default_err.str().find("breakpoint") != std::string::npos,
                     "unrequested BKPT produces an explicit diagnostic");

    const std::string_view allowed_args[]{
        "run", config_text, "--duration-ms", "0", "--max-instructions", "20",
        "--no-detect-spin", "--allow-breakpoint",
    };
    std::ostringstream allowed_out;
    std::ostringstream allowed_err;
    fil::test::check(
        fil::cli::run(allowed_args, allowed_out, allowed_err) == fil::cli::ExitCode::success,
        "--allow-breakpoint explicitly accepts a firmware BKPT"
    );
    fil::test::check(allowed_err.str().empty(),
                     "an explicitly accepted BKPT has no error diagnostic");

    const std::string_view watch_args[]{
        "watch", config_text, "--duration-ms", "0", "--max-instructions", "20",
        "--trace-instr", "--no-realtime", "--allow-breakpoint",
    };
    std::ostringstream watch_out;
    std::ostringstream watch_err;
    fil::test::check(
        fil::cli::run(watch_args, watch_out, watch_err) == fil::cli::ExitCode::success,
        "watch executes a board through the regular CLI"
    );
    fil::test::check(
        watch_out.str().find("watching board cli-breakpoint") != std::string::npos
            && watch_out.str().find(" ms] cli-breakpoint  instr pc=") != std::string::npos,
        "watch prints live timestamped trace records"
    );
    fil::test::check(watch_err.str().empty(), "successful watch has no error output");
    std::error_code remove_error;
    std::filesystem::remove(config_path, remove_error);
}

} // namespace

/// @brief Runs the dependency-free unit-test executable.
int main() {
    helpIsSuccessful();
    unknownCommandIsAUsageError();
    disassemblesSyntheticWindow();
    requiresExplicitBreakpointAcceptance();
    runConfigTests();
    runBoardTests();
    runCortexMTests();
    runExceptionTests();
    runCanBusTests();
    runElfLoaderTests();
    runEventLoopTests();
    runFdcanTests();
    runMemoryBusTests();
    runPeripheralTests();
    runStm32G4Tests();
    runStartupRuntimeTests();
    runSchedulerTickTests();
    runHardFloatFirmwareTests();
    runWorldTests();
    runWorldTimeTests();

    if (fil::test::failures != 0) {
        std::cerr << fil::test::failures << " test(s) failed\n";
        return 1;
    }

    std::cout << "all tests passed\n";
    return 0;
}
