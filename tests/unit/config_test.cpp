#include "fil/common/error.hpp"
#include "fil/common/log.hpp"
#include "fil/config/config.hpp"
#include "../test_support.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>

namespace {

class TempConfigs {
public:
    /// @brief Creates an isolated temporary config tree.
    TempConfigs() : root_(std::filesystem::temp_directory_path() / "fil-config-tests") {
        std::error_code error;
        std::filesystem::remove_all(root_, error);
        std::filesystem::create_directories(root_ / "boards");
        std::filesystem::create_directories(root_ / "mcus");
    }

    /// @brief Removes the temporary config tree.
    ~TempConfigs() {
        std::error_code error;
        std::filesystem::remove_all(root_, error);
    }

    /// @brief Writes one temporary config file.
    std::filesystem::path write(const std::filesystem::path& relative, const std::string_view contents) const {
        const auto path = root_ / relative;
        std::ofstream output(path, std::ios::binary);
        output << contents;
        return path;
    }

    /** @brief Gets the temporary tree root. @return Root path. */
    [[nodiscard]] const std::filesystem::path& root() const { return root_; }

private:
    std::filesystem::path root_;
};

/// @brief Verifies supported numeric formats and overflow handling.
void parsesUnsignedValues() {
    const auto decimal = fil::config::parseUnsigned("1234");
    const auto hex = fil::config::parseUnsigned("0x08000000");
    const auto kibibytes = fil::config::parseUnsigned("512K");
    const auto mebibytes = fil::config::parseUnsigned("2M");
    const auto invalid = fil::config::parseUnsigned("12KB");
    const auto overflow = fil::config::parseUnsigned("18446744073709551615M");

    fil::test::check(decimal && decimal.value() == 1234, "parses decimal integers");
    fil::test::check(hex && hex.value() == 0x08000000, "parses hexadecimal integers");
    fil::test::check(kibibytes && kibibytes.value() == 512ULL * 1024ULL, "parses K suffix");
    fil::test::check(mebibytes && mebibytes.value() == 2ULL * 1024ULL * 1024ULL, "parses M suffix");
    fil::test::check(!invalid, "rejects unsupported suffixes");
    fil::test::check(!overflow, "rejects suffix multiplication overflow");
}

/// @brief Verifies valid MCU/board loading and relative path resolution.
void loadsMcuAndBoardConfig() {
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

    fil::test::check(mcu && mcu.value().flash_size == 512U * 1024U, "loads MCU sizes");
    fil::test::check(mcu && mcu.value().ccm_sram_base == 0x10000000U, "loads CCM SRAM address");
    fil::test::check(mcu && mcu.value().hse_hz == 16'000'000U, "loads HSE frequency");
    fil::test::check(board && board.value().name == "test-board", "loads board name");
    fil::test::check(board && board.value().mcu_path == mcu_path.lexically_normal(), "resolves MCU path against config directory");
    fil::test::check(board && board.value().elf_path == (files.root() / "firmware/test.elf").lexically_normal(), "resolves ELF path against config directory");
    fil::test::check(board && board.value().run.max_instructions == 9000, "loads run limits");
    fil::test::check(board && board.value().gpio.size() == 2 && board.value().gpio[1].trace, "loads GPIO pin definitions");
    fil::test::check(board && board.value().can.size() == 1 && board.value().can[0].bus == "vehicle", "loads CAN attachment");
    fil::test::check(board && board.value().usart.size() == 1 && board.value().usart[0].scripted_rx.size() == 3, "loads USART script");
    fil::test::check(board && board.value().adc[0].channels.at(3).period_ms == 50, "loads ADC waveform");
    fil::test::check(board && board.value().spi[0].device == "echo", "loads SPI attachment");
}

/// @brief Verifies network bus parsing and relative board path resolution.
void loadsNetworkConfig() {
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
    fil::test::check(network && network.value().buses.size() == 1, "loads network CAN bus");
    fil::test::check(network && network.value().buses[0].bitrate == 500000U, "loads network bitrate");
    fil::test::check(
        network && network.value().board_paths[1] == (files.root() / "boards/b.json").lexically_normal(),
        "resolves network board paths against network config"
    );
}

/// @brief Verifies strict schema and JSON failures.
void rejectsMalformedConfigs() {
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

    fil::test::check(!unknown && unknown.error().message.find("unknown key") != std::string::npos, "rejects unknown keys");
    fil::test::check(!duplicate && duplicate.error().category == fil::ErrorCategory::parse, "rejects duplicate keys while parsing");
    fil::test::check(!missing && missing.error().message.find("missing required key 'elf'") != std::string::npos, "rejects missing keys");
    fil::test::check(!bad_address && bad_address.error().message.find("invalid value") != std::string::npos, "rejects malformed addresses");
    fil::test::check(!absent && absent.error().category == fil::ErrorCategory::io, "reports missing config files as IO errors");
    fil::test::check(!duplicate && duplicate.error().source.has_value() && duplicate.error().source->line == 1, "parse errors contain source locations");
}

/// @brief Verifies logger severity filtering.
void loggingHonorsMinimumLevel() {
    std::ostringstream output;
    fil::Logger logger(output, fil::LogLevel::warning);
    logger.log(fil::LogLevel::info, "hidden");
    logger.log(fil::LogLevel::error, "visible");
    fil::test::check(output.str() == "[error] visible\n", "logger filters and formats messages");
}

} // namespace

/// @brief Runs all configuration and common-utility unit tests.
void runConfigTests() {
    parsesUnsignedValues();
    loadsMcuAndBoardConfig();
    loadsNetworkConfig();
    rejectsMalformedConfigs();
    loggingHonorsMinimumLevel();
}
