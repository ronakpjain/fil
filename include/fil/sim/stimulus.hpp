#pragma once

/** @file stimulus.hpp
 *  @brief Timed external inputs for deterministic network simulations.
 */

#include "fil/common/result.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace fil::sim {

class World;

/** @brief External CAN frame injected into a named virtual bus. */
struct CanStimulus {
    std::string bus;                 ///< Network-local CAN bus name.
    std::uint32_t id{0};             ///< Standard or extended identifier.
    bool extended{false};            ///< Whether the identifier uses 29-bit format.
    bool fd{false};                  ///< Whether this is a CAN-FD frame.
    bool brs{false};                 ///< Whether CAN-FD bit-rate switching is enabled.
    std::vector<std::uint8_t> data;  ///< Payload bytes.
};

/** @brief External level driven on one named board GPIO. */
struct GpioStimulus {
    std::string board;               ///< Target board instance name.
    std::string port;                ///< GPIO port such as `GPIOB`.
    std::uint8_t pin{0};             ///< Pin number from 0 through 15.
    std::optional<bool> level;       ///< Driven level, or empty to release the pin.
};

/** @brief External sample value applied to one ADC input. */
struct AdcStimulus {
    std::string board;               ///< Target board instance name.
    std::string instance;            ///< ADC instance such as `ADC1`.
    std::uint8_t channel{0};         ///< ADC channel from 0 through 19.
    std::uint16_t value{0};          ///< 12-bit conversion input.
};

/** @brief Bytes delivered to a board USART receive queue. */
struct UsartStimulus {
    std::string board;               ///< Target board instance name.
    std::string instance;            ///< USART instance such as `USART1`.
    std::vector<std::uint8_t> bytes; ///< Bytes made available to firmware.
};

/** @brief One supported external input variant in a stimulus script. */
using StimulusInput = std::variant<CanStimulus, GpioStimulus, AdcStimulus, UsartStimulus>;

/** @brief A timed input, optionally repeated a fixed number of times. */
struct StimulusEvent {
    std::uint64_t at_ms{0};              ///< First injection time relative to reset.
    std::uint64_t repeat_every_ms{0};    ///< Interval between injections; zero for a single event.
    std::uint32_t repeat_count{1};       ///< Total injections including the first.
    StimulusInput input;                 ///< CAN, GPIO, ADC, or USART input.
};

/** @brief A deterministic, timed input script attached to a network. */
struct StimulusScript {
    std::string name;                     ///< Stable script name.
    std::filesystem::path source_path;    ///< Absolute source file path.
    std::vector<StimulusEvent> events;    ///< Timed external inputs.
};

/** @brief Loads and validates a versioned timed input script. */
[[nodiscard]] Result<StimulusScript> loadStimulusScript(const std::filesystem::path& path);

/** @brief Validates targets and schedules every input in a loaded script. */
[[nodiscard]] Result<void> scheduleStimulusScript(const StimulusScript& script, World& world);

/** @brief Produces stable human-readable output for a stimulus script. */
[[nodiscard]] std::string normalizeStimulusScript(const StimulusScript& script);

} // namespace fil::sim
