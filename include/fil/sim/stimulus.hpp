#pragma once

/** @file stimulus.hpp
 *  @brief Timed external inputs for deterministic network simulations.
 */

#include "fil/common/result.hpp"
#include "fil/sim/event_loop.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace fil::sim {

class World;
struct TraceRecord;
class TraceRecorder;

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

/** @brief Expected firmware-originated CAN frame within an inclusive time interval. */
struct CanExpectation {
    std::string bus;                 ///< Network-local CAN bus name.
    std::uint32_t id{0};            ///< Standard or extended identifier.
    bool extended{false};           ///< Identifier format.
    std::vector<std::uint8_t> data; ///< Exact expected payload.
    std::uint64_t at_ms{0};          ///< Inclusive start relative to simulation time zero.
    std::uint64_t window_ms{0};      ///< Inclusive end offset from at_ms.
};

/** @brief A deterministic, timed input script attached to a network. */
struct StimulusScript {
    std::string name;                     ///< Stable script name.
    std::filesystem::path source_path;    ///< Absolute source file path.
    std::vector<StimulusEvent> events;    ///< Timed external inputs.
    std::vector<CanExpectation> expect;   ///< Optional firmware output requirements.
};

/** @brief Loads and validates a versioned timed input script. */
[[nodiscard]] Result<StimulusScript> loadStimulusScript(const std::filesystem::path& path);

/** @brief Validates targets and schedules every input in a loaded script. */
[[nodiscard]] Result<void> scheduleStimulusScript(const StimulusScript& script, World& world);

/** @brief Tests a CAN transmit trace record against exact bus/origin/frame/window criteria. */
[[nodiscard]] bool matchesCanExpectation(const CanExpectation& expectation, const TraceRecord& record);

/** @brief Stable script/check pair used by CLI expectation evaluation. */
struct CanExpectationCheck {
    std::string script;
    std::size_t index{0};
    CanExpectation expectation;
    std::size_t stimulus_index{0}; ///< Zero-based attachment position in the network config.
};

/**
 * @brief Emits and tracks live CAN expectation lifecycle trace records.
 *
 * The referenced TraceRecorder and EventLoop must outlive this evaluator.
 * The evaluator is non-copyable/non-movable because deadline callbacks refer to
 * its stable address; destruction cancels any outstanding callbacks.
 */
class CanExpectationEvaluator {
public:
    CanExpectationEvaluator(TraceRecorder& trace, std::vector<CanExpectationCheck> checks);
    ~CanExpectationEvaluator();
    CanExpectationEvaluator(const CanExpectationEvaluator&) = delete;
    CanExpectationEvaluator& operator=(const CanExpectationEvaluator&) = delete;
    CanExpectationEvaluator(CanExpectationEvaluator&&) = delete;
    CanExpectationEvaluator& operator=(CanExpectationEvaluator&&) = delete;

    void begin(std::uint64_t now_ns, EventLoop& loop);
    void observe(const TraceRecord& record);
    void advance(std::uint64_t now_ns);
    void finish(std::uint64_t now_ns);
    [[nodiscard]] bool hasFailures() const noexcept;
    [[nodiscard]] std::string_view status(std::size_t index) const noexcept;

private:
    struct CheckState {
        CanExpectationCheck check;
        std::string status{"pending"};
    };

    void emit(CheckState& state, std::string_view status, std::uint64_t time_ns,
        const TraceRecord* matched = nullptr);
    void cancelDeadlines() noexcept;
    TraceRecorder* trace_{nullptr};
    EventLoop* loop_{nullptr};
    std::vector<EventId> deadline_events_;
    std::vector<CheckState> checks_;
    bool started_{false};
    bool finished_{false};
};

/** @brief Produces stable human-readable output for a stimulus script. */
[[nodiscard]] std::string normalizeStimulusScript(const StimulusScript& script);

} // namespace fil::sim
