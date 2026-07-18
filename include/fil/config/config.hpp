#pragma once

/** @file config.hpp
 *  @brief Versioned MCU and board configuration models and loaders.
 */

#include "fil/common/result.hpp"

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace fil::config {

/** @brief Schema version accepted by the current configuration loaders. */
inline constexpr std::uint32_t current_schema_version = 1;

/**
 * @brief Memory geometry for one supported microcontroller variant.
 *
 * Values are loaded from a strict, versioned JSON file. Addresses and sizes are
 * target values and therefore remain fixed-width rather than host-sized.
 */
struct McuConfig {
    std::uint32_t schema_version{current_schema_version}; ///< Input schema version.
    std::string name;                                     ///< Stable MCU identifier.
    std::uint32_t flash_base{0};                          ///< First flash address.
    std::uint32_t flash_size{0};                          ///< Flash capacity in bytes.
    std::uint32_t sram_base{0};                           ///< First conventional SRAM address.
    std::uint32_t sram_size{0};                           ///< Conventional SRAM capacity in bytes.
    std::uint32_t ccm_sram_base{0};                       ///< First core-coupled SRAM address.
    std::uint32_t ccm_sram_size{0};                       ///< Core-coupled SRAM capacity in bytes.
    std::uint32_t hse_hz{8'000'000};                      ///< External high-speed oscillator frequency.
};

/** @brief Default stopping limits for a single emulator run. */
struct RunConfig {
    std::uint64_t default_duration_ms{1'000};   ///< Simulated duration limit in milliseconds.
    std::uint64_t max_instructions{50'000'000}; ///< Executed instruction limit.
};

/** @brief Initial value and tracing policy for one named GPIO pin. */
struct GpioPinConfig {
    std::string pin;             ///< Canonical pin name such as `PA0`.
    std::string mode{"input"};  ///< `input`, `output`, `alternate`, or `analog`.
    bool value{false};           ///< Initial externally observed logical level.
    bool trace{false};           ///< Whether output transitions are traced.
};

/** @brief Attachment of one FDCAN controller to a named virtual bus. */
struct CanControllerConfig {
    std::string instance; ///< Controller name such as `FDCAN1`.
    std::string bus;      ///< Network-local virtual bus name.
    bool loopback{false}; ///< Whether transmitted frames are delivered to the sender.
};

/** @brief Host-facing behavior of one USART instance. */
struct UsartConfig {
    std::string instance;                 ///< Peripheral name such as `USART1`.
    std::optional<std::filesystem::path> tx_log; ///< Optional resolved byte-log path.
    std::vector<std::uint8_t> scripted_rx;       ///< Deterministic bytes available at reset.
};

/** @brief Deterministic ADC channel waveform. */
struct AdcChannelConfig {
    enum class Kind { constant, sine } kind{Kind::constant}; ///< Configured waveform type.
    std::uint16_t value{0};     ///< Constant conversion value.
    std::uint16_t minimum{0};   ///< Sine-wave minimum value.
    std::uint16_t maximum{4095}; ///< Sine-wave maximum value.
    std::uint64_t period_ms{1000}; ///< Sine-wave period in simulated milliseconds.
};

/** @brief Per-channel input values for one ADC instance. */
struct AdcConfig {
    std::string instance;                         ///< Peripheral name such as `ADC1`.
    std::map<std::uint8_t, AdcChannelConfig> channels; ///< Channel number to source model.
};

/** @brief External device selection for one SPI controller. */
struct SpiConfig {
    std::string instance;    ///< Peripheral name such as `SPI1`.
    std::string device{"zero"}; ///< Built-in device model name.
};

/**
 * @brief Validated configuration for one emulated board instance.
 *
 * Paths are absolute and lexically normalized after loading. Device-specific
 * configuration will be added as peripheral models are implemented.
 */
struct BoardConfig {
    std::uint32_t schema_version{current_schema_version}; ///< Input schema version.
    std::string name;                                     ///< Board instance name used in traces.
    std::filesystem::path source_path;                    ///< Absolute path of the board JSON file.
    std::filesystem::path mcu_path;                       ///< Resolved MCU configuration path.
    std::filesystem::path elf_path;                       ///< Resolved firmware ELF path.
    std::optional<std::uint32_t> vector_base;             ///< Optional vector-table address override.
    RunConfig run;                                        ///< Default execution limits.
    std::vector<GpioPinConfig> gpio;                       ///< Named GPIO pin configuration.
    std::vector<CanControllerConfig> can;                  ///< FDCAN virtual-bus attachments.
    std::vector<UsartConfig> usart;                        ///< USART endpoint behavior.
    std::vector<AdcConfig> adc;                            ///< ADC channel inputs.
    std::vector<SpiConfig> spi;                            ///< SPI device attachments.
};

/** @brief One deterministic CAN bus declared by a network configuration. */
struct CanBusConfig {
    std::string name;              ///< Stable network-local bus name.
    std::uint32_t bitrate{500000}; ///< Informational nominal bit rate.
};

/** @brief Multiple boards and buses run in one shared simulation world. */
struct NetworkConfig {
    std::uint32_t schema_version{current_schema_version}; ///< Input schema version.
    std::string name;                                     ///< Stable network name.
    std::filesystem::path source_path;                    ///< Absolute source config path.
    std::vector<CanBusConfig> buses;                       ///< Declared virtual CAN buses.
    std::vector<std::filesystem::path> board_paths;        ///< Resolved board config paths.
};

/**
 * @brief Parses an unsigned decimal or hexadecimal value with optional K/M suffix.
 * @param text Value such as `512K` or `0x08000000`.
 * @return Parsed value, or an invalid-argument error.
 */
[[nodiscard]] Result<std::uint64_t> parseUnsigned(std::string_view text);

/**
 * @brief Loads and validates a versioned MCU JSON configuration.
 * @param path Path to the configuration file.
 * @return Validated MCU configuration, or a source-aware error.
 */
[[nodiscard]] Result<McuConfig> loadMcuConfig(const std::filesystem::path& path);

/**
 * @brief Loads and validates a versioned board JSON configuration.
 * @param path Path to the configuration file.
 * @return Board configuration with referenced paths resolved against the file.
 */
[[nodiscard]] Result<BoardConfig> loadBoardConfig(const std::filesystem::path& path);

/** @brief Loads and validates a multi-board network JSON configuration. */
[[nodiscard]] Result<NetworkConfig> loadNetworkConfig(const std::filesystem::path& path);

/**
 * @brief Produces stable human-readable output for an MCU configuration.
 * @param config Configuration to render.
 * @return Normalized multiline representation.
 */
[[nodiscard]] std::string normalize(const McuConfig& config);

/**
 * @brief Produces stable human-readable output for a board configuration.
 * @param config Configuration to render.
 * @return Normalized multiline representation.
 */
[[nodiscard]] std::string normalize(const BoardConfig& config);

/** @brief Produces stable human-readable output for a network configuration. */
[[nodiscard]] std::string normalize(const NetworkConfig& config);

} // namespace fil::config
