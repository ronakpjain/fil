#include "fil/cli/cli.hpp"
#include "fil/hardware/comparison.hpp"
#include "fil/sim/board.hpp"
#include "../fixture_support.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

std::filesystem::path writeFakeOpenOcd(
    const fil::test::TemporaryDirectory& directory,
    const std::uint32_t r0 = 0x20000000U
) {
    std::ostringstream script;
    script << "#!/bin/sh\n"
           << "capture=$2\n"
           << "while IFS= read -r line; do\n"
           << "  case \"$line\" in\n"
           << "    dump_image*)\n"
           << "      path=$(printf '%s\\n' \"$line\" | sed -E 's/^dump_image \\{([^}]*)\\}.*/\\1/')\n"
           << "      printf '\\000\\000\\000\\000' > \"$path\"\n"
           << "      ;;\n"
           << "  esac\n"
           << "done < \"$capture\"\n"
           << "echo FIL_CHIP_ID=0x10000469\n";
    for (const std::string_view name : fil::hardware::comparableRegisterNames()) {
        const std::uint32_t value = name == "r0" ? r0 : 0U;
        script << "echo FIL_REG_" << name << "='" << name << " (/32): 0x"
               << std::hex << value << std::dec << "'\n";
    }
    const auto path = directory.write("fake-openocd", script.str());
    std::filesystem::permissions(
        path,
        std::filesystem::perms::owner_read
            | std::filesystem::perms::owner_write
            | std::filesystem::perms::owner_exec
    );
    return path;
}

TEST(HardwareComparisonTest, ParsesBoundedMemoryRanges) {
    const auto valid = fil::hardware::parseMemoryRange("0x20000000:16");
    ASSERT_TRUE(valid.hasValue());
    EXPECT_EQ(valid.value().address, 0x20000000U);
    EXPECT_EQ(valid.value().size, 16U);
    EXPECT_FALSE(fil::hardware::parseMemoryRange("0x20000000").hasValue());
    EXPECT_FALSE(fil::hardware::parseMemoryRange("0xffffffff:2").hasValue());
    EXPECT_FALSE(fil::hardware::parseMemoryRange("0x20000000:0").hasValue());
}

TEST(HardwareComparisonTest, CapturesEmulatorRegistersAndMemory) {
    auto board = fil::sim::Board::load(fil::test::fixtureBoardConfig());
    ASSERT_TRUE(board.hasValue());

    fil::sim::BoardRunOptions options;
    options.max_instructions = 20U;
    options.duration_ns = 0U;
    options.enable_loop_batching = false;
    const auto result = board.value()->run(options);
    ASSERT_EQ(result.reason, fil::sim::BoardStopReason::breakpoint);

    const fil::hardware::MemoryRange range{0x20000000U, 4U};
    const auto snapshot = fil::hardware::captureEmulator(*board.value(), result, {&range, 1U});
    ASSERT_TRUE(snapshot.hasValue());
    EXPECT_EQ(snapshot.value().source, "emulator");
    EXPECT_EQ(snapshot.value().stop_reason, "breakpoint");
    EXPECT_EQ(snapshot.value().registers.size(), 23U);
    ASSERT_EQ(snapshot.value().memory.size(), 1U);
    EXPECT_EQ(snapshot.value().memory.front().bytes, std::vector<std::uint8_t>(4U, 0U));

    const fil::hardware::MemoryRange mmio{0x40000000U, 4U};
    EXPECT_FALSE(fil::hardware::captureEmulator(*board.value(), result, {&mmio, 1U}));
}

TEST(HardwareComparisonTest, ProbeFixtureInitializesTheCompleteComparisonState) {
    const auto config_path = std::filesystem::path(FIL_SOURCE_DIR)
        / "tests/fixtures/config/hardware_compare_board.json";
    const auto config = fil::config::loadBoardConfig(config_path);
    ASSERT_TRUE(config.hasValue());
    auto board = fil::sim::Board::load(config.value());
    ASSERT_TRUE(board.hasValue());

    fil::sim::BoardRunOptions options;
    options.max_instructions = 100U;
    options.duration_ns = 0U;
    options.enable_loop_batching = false;
    const auto result = board.value()->run(options);
    ASSERT_EQ(result.reason, fil::sim::BoardStopReason::breakpoint);
    EXPECT_EQ(result.instructions, 25U);

    const fil::hardware::MemoryRange range{0x20000000U, 16U};
    const auto snapshot = fil::hardware::captureEmulator(*board.value(), result, {&range, 1U});
    ASSERT_TRUE(snapshot.hasValue());
    for (std::uint32_t index = 0U; index <= 12U; ++index) {
        EXPECT_EQ(snapshot.value().registers[index].value, 0x10U + index);
    }
    EXPECT_EQ(snapshot.value().registers[13].value, 0x20001000U);
    EXPECT_EQ(snapshot.value().registers[14].value, 0x1eU);
    EXPECT_EQ(snapshot.value().registers[15].value, 0x08000044U);
    EXPECT_EQ(snapshot.value().registers[16].value, 0x61000000U);
    ASSERT_EQ(snapshot.value().memory.size(), 1U);
    EXPECT_EQ(
        snapshot.value().memory.front().bytes,
        (std::vector<std::uint8_t>{
            0x44U, 0x33U, 0x22U, 0x11U,
            0x5aU, 0x5aU, 0xa5U, 0xa5U,
            0xdeU, 0xc0U, 0xadU, 0x0bU,
            0xefU, 0xbeU, 0xedU, 0xfeU,
        })
    );
}

TEST(HardwareComparisonTest, ReportsOnlySelectedDifferences) {
    fil::hardware::Snapshot emulator;
    emulator.registers = {{"r0", 1U}, {"r1", 2U}};
    emulator.memory = {{{0x20000000U, 2U}, {0xaaU, 0xbbU}}};
    auto target = emulator;
    target.registers[0].value = 3U;
    target.memory[0].bytes[1] = 0xccU;

    const std::vector<std::string> only_r1{"r1"};
    EXPECT_FALSE(fil::hardware::compare(emulator, target, only_r1).matches());
    target.memory = emulator.memory;
    EXPECT_TRUE(fil::hardware::compare(emulator, target, only_r1).matches());

    const std::vector<std::string> only_r0{"r0"};
    const auto difference = fil::hardware::compare(emulator, target, only_r0);
    ASSERT_EQ(difference.register_differences.size(), 1U);
    EXPECT_EQ(difference.register_differences.front().emulator, 1U);
    EXPECT_EQ(difference.register_differences.front().hardware, 3U);
}

TEST(HardwareComparisonTest, CapturesParseableFakeOpenOcdOutput) {
    fil::test::TemporaryDirectory directory{"fil-openocd-test"};
    const auto fake = writeFakeOpenOcd(directory, 0x12345678U);
    const auto elf = std::filesystem::path(FIL_SOURCE_DIR)
        / "tests/fixtures/elf/split_image.elf";
    const fil::hardware::MemoryRange range{0x20000000U, 4U};
    fil::hardware::StlinkCaptureOptions options;
    options.openocd = fake;
    options.timeout = std::chrono::milliseconds(1'000);

    const auto snapshot = fil::hardware::captureStlink(elf, {&range, 1U}, options);
    ASSERT_TRUE(snapshot.hasValue()) << snapshot.error().message;
    EXPECT_EQ(snapshot.value().source, "stlink");
    EXPECT_EQ(snapshot.value().registers.front().name, "r0");
    EXPECT_EQ(snapshot.value().registers.front().value, 0x12345678U);
    ASSERT_EQ(snapshot.value().memory.size(), 1U);
    EXPECT_EQ(snapshot.value().memory.front().bytes, std::vector<std::uint8_t>(4U, 0U));

    options.serial = "unsafe;shutdown";
    EXPECT_FALSE(fil::hardware::captureStlink(elf, {&range, 1U}, options).hasValue());
}

TEST(HardwareComparisonTest, RunsHermeticCliComparison) {
    fil::test::TemporaryDirectory directory{"fil-compare-cli-test"};
    const auto fake = writeFakeOpenOcd(directory);
    const auto config = directory.write(
        "board.json",
        std::string("{\n")
            + "  \"schema_version\": 1,\n"
            + "  \"name\": \"comparison-fixture\",\n"
            + "  \"mcu\": \"" + (std::filesystem::path(FIL_SOURCE_DIR)
                / "configs/mcus/stm32g474retx.json").string() + "\",\n"
            + "  \"elf\": \"" + (std::filesystem::path(FIL_SOURCE_DIR)
                / "tests/fixtures/elf/split_image.elf").string() + "\",\n"
            + "  \"vector_base\": \"0x08000000\"\n"
            + "}\n"
    );
    const std::vector<std::string> owned{
        "compare-stlink", config.string(), "--openocd", fake.string(),
        "--max-instructions", "20", "--register", "r0",
        "--memory", "0x20000000:4",
    };
    std::vector<std::string_view> args;
    for (const std::string& value : owned) args.push_back(value);
    std::ostringstream output;
    std::ostringstream errors;
    const auto status = fil::cli::run(args, output, errors);
    EXPECT_EQ(status, fil::cli::ExitCode::success) << errors.str();
    EXPECT_NE(output.str().find("\"match\": true"), std::string::npos);
}

} // namespace
