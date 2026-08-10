#include "fil/common/error.hpp"
#include "fil/common/log.hpp"
#include "fil/common/numeric.hpp"
#include "fil/config/config.hpp"
#include "../fixture_support.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <sstream>
#include <string>
#include <string_view>

namespace {

class TempConfigs {
public:
    [[nodiscard]] std::filesystem::path write(
        const std::filesystem::path& relative,
        const std::string_view contents
    ) const {
        return directory_.write(relative, contents);
    }

    [[nodiscard]] const std::filesystem::path& root() const noexcept {
        return directory_.root();
    }

private:
    fil::test::TemporaryDirectory directory_{"fil-config-tests"};
};

TEST(ConfigTest, ChecksNumericHelpers) {
    EXPECT_TRUE(fil::saturatingAdd(std::uint8_t{250U}, std::uint8_t{10U}) == 255U)
        << "saturating addition clamps unsigned overflow";
    EXPECT_TRUE(fil::saturatingAdd(std::uint64_t{40U}, std::uint8_t{2U}) == 42U)
        << "saturating addition accepts narrower operands";
    EXPECT_TRUE(
        fil::rangeFits(4U, 4U, 8U) && !fil::rangeFits(5U, 4U, 8U) && !fil::rangeFits(-1, 1U, 8U))
        << "range checks handle boundaries and signed rejection";
}

/// @brief Verifies supported numeric formats and overflow handling.
TEST(ConfigTest, ParsesUnsignedValues) {
    const auto decimal = fil::config::parseUnsigned("1234");
    const auto hex = fil::config::parseUnsigned("0x08000000");
    const auto kibibytes = fil::config::parseUnsigned("512K");
    const auto mebibytes = fil::config::parseUnsigned("2M");
    const auto invalid = fil::config::parseUnsigned("12KB");
    const auto overflow = fil::config::parseUnsigned("18446744073709551615M");

    EXPECT_TRUE(decimal && decimal.value() == 1234) << "parses decimal integers";
    EXPECT_TRUE(hex && hex.value() == 0x08000000) << "parses hexadecimal integers";
    EXPECT_TRUE(kibibytes && kibibytes.value() == 512ULL * 1024ULL) << "parses K suffix";
    EXPECT_TRUE(mebibytes && mebibytes.value() == 2ULL * 1024ULL * 1024ULL) << "parses M suffix";
    EXPECT_TRUE(!invalid) << "rejects unsupported suffixes";
    EXPECT_TRUE(!overflow) << "rejects suffix multiplication overflow";
}

/// @brief Verifies valid MCU/board loading and relative path resolution.
TEST(ConfigTest, LoadsMcuAndBoardConfig) {
    TempConfigs files;
    const auto mcu_path = files.write(
        "mcus/g4.json",
        R"({
  "schema_version": 1,
  "name": "g4",
  "flash_base": "0x08000000",
  "flash_size": "512K",
  "sram_base": "0x20000000",
  "sram_size": "128K",
  "ccm_sram_base": "0x10000000",
  "ccm_sram_size": "32K",
  "hse_hz": 16000000
})"
    );
    const auto board_path = files.write(
        "boards/test.json",
        R"({
  "schema_version": 1,
  "name": "test-board",
  "mcu": "../mcus/g4.json",
  "elf": "../firmware/test.elf",
  "vector_base": "0x08000000",
  "run": {"default_duration_ms": 25, "max_instructions": 9000},
  "gpio": {"PA0":{"mode":"input","value":true},"PC13":{"mode":"output","trace":true}},
  "can": {"FDCAN1":{"bus":"vehicle","loopback":false}},
  "usart": {"USART1":{"tx_log":"../logs/uart.bin","rx":[1,2,255]}},
  "adc": {"ADC1":{"channels":{"1":2048,"3":{"type":"sine","min":1000,"max":3000,"period_ms":50}}}},
  "spi": {"SPI1":{"device":"echo"}}
})"
    );

    const auto mcu = fil::config::loadMcuConfig(mcu_path);
    const auto board = fil::config::loadBoardConfig(board_path);

    EXPECT_TRUE(mcu && mcu.value().flash_size == 512U * 1024U) << "loads MCU sizes";
    EXPECT_TRUE(mcu && mcu.value().ccm_sram_base == 0x10000000U) << "loads CCM SRAM address";
    EXPECT_TRUE(mcu && mcu.value().hse_hz == 16'000'000U) << "loads HSE frequency";
    EXPECT_TRUE(board && board.value().name == "test-board") << "loads board name";
    EXPECT_TRUE(board && board.value().mcu_path == mcu_path.lexically_normal())
        << "resolves MCU path against config directory";
    EXPECT_TRUE(
        board && board.value().elf_path == (files.root() / "firmware/test.elf").lexically_normal())
        << "resolves ELF path against config directory";
    EXPECT_TRUE(board && board.value().run.max_instructions == 9000) << "loads run limits";
    EXPECT_TRUE(board && board.value().gpio.size() == 2 && board.value().gpio[1].trace)
        << "loads GPIO pin definitions";
    EXPECT_TRUE(board && board.value().can.size() == 1 && board.value().can[0].bus == "vehicle")
        << "loads CAN attachment";
    EXPECT_TRUE(
        board && board.value().usart.size() == 1 && board.value().usart[0].scripted_rx.size() == 3)
        << "loads USART script";
    EXPECT_TRUE(board && board.value().adc[0].channels.at(3).period_ms == 50)
        << "loads ADC waveform";
    EXPECT_TRUE(board && board.value().spi[0].device == "echo") << "loads SPI attachment";
}

/// @brief Verifies network bus parsing and relative board path resolution.
TEST(ConfigTest, LoadsNetworkConfig) {
    TempConfigs files;
    const auto network_path = files.write(
        "network.json",
        R"({
  "schema_version":1,
  "name":"vehicle",
  "buses":{"vehicle":{"type":"can","bitrate":500000}},
  "boards":["boards/a.json","boards/b.json"]
})"
    );
    const auto network = fil::config::loadNetworkConfig(network_path);
    EXPECT_TRUE(network && network.value().buses.size() == 1) << "loads network CAN bus";
    EXPECT_TRUE(network && network.value().buses[0].bitrate == 500000U) << "loads network bitrate";
    EXPECT_TRUE(network && network.value().board_paths[1] ==
                               (files.root() / "boards/b.json").lexically_normal())
        << "resolves network board paths against network config";
}

/// @brief Verifies strict schema and JSON failures.
TEST(ConfigTest, RejectsMalformedConfigs) {
    TempConfigs files;
    const auto unknown_path = files.write(
        "unknown.json",
        R"({"schema_version":1,"name":"x","mcu":"m","elf":"e","surprise":true})"
    );
    const auto duplicate_path = files.write(
        "duplicate.json",
        R"({"schema_version":1,"name":"x","name":"y","mcu":"m","elf":"e"})"
    );
    const auto missing_path = files.write(
        "missing.json",
        R"({"schema_version":1,"name":"x","mcu":"m"})"
    );
    const auto bad_address_path = files.write(
        "bad-address.json",
        R"({"schema_version":1,"name":"x","mcu":"m","elf":"e","vector_base":"0xnope"})"
    );

    const auto unknown = fil::config::loadBoardConfig(unknown_path);
    const auto duplicate = fil::config::loadBoardConfig(duplicate_path);
    const auto missing = fil::config::loadBoardConfig(missing_path);
    const auto bad_address = fil::config::loadBoardConfig(bad_address_path);
    const auto absent = fil::config::loadBoardConfig(files.root() / "absent.json");

    EXPECT_TRUE(!unknown && unknown.error().message.find("unknown key") != std::string::npos)
        << "rejects unknown keys";
    EXPECT_TRUE(!duplicate && duplicate.error().category == fil::ErrorCategory::parse)
        << "rejects duplicate keys while parsing";
    EXPECT_TRUE(
        !missing && missing.error().message.find("missing required key 'elf'") != std::string::npos)
        << "rejects missing keys";
    EXPECT_TRUE(
        !bad_address && bad_address.error().message.find("invalid value") != std::string::npos)
        << "rejects malformed addresses";
    EXPECT_TRUE(!absent && absent.error().category == fil::ErrorCategory::io)
        << "reports missing config files as IO errors";
    EXPECT_TRUE(
        !duplicate && duplicate.error().source.has_value() && duplicate.error().source->line == 1)
        << "parse errors contain source locations";
}

/// @brief Verifies logger severity filtering.
TEST(ConfigTest, LoggingHonorsMinimumLevel) {
    std::ostringstream output;
    fil::Logger logger(output, fil::LogLevel::warning);
    logger.log(fil::LogLevel::info, "hidden");
    logger.log(fil::LogLevel::error, "visible");
    EXPECT_TRUE(output.str() == "[error] visible\n") << "logger filters and formats messages";
}

} // namespace

/// @brief Runs all configuration and common-utility unit tests.
