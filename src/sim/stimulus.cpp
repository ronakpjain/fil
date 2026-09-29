#include "fil/sim/stimulus.hpp"

#include "fil/config/config.hpp"
#include "fil/devices/can_bus.hpp"
#include "fil/sim/board.hpp"
#include "fil/sim/world.hpp"
#include "fil/stm32g4/peripheral.hpp"
#include "fil/stm32g4/stm32g4.hpp"
#include "../config/json_internal.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <type_traits>
#include <utility>

namespace fil::sim {
namespace {

using config::detail::JsonValue;
using config::detail::absoluteNormalized;
using config::detail::booleanValue;
using config::detail::configError;
using config::detail::find;
using config::detail::loadJson;
using config::detail::numericValue;
using config::detail::rejectUnknown;
using config::detail::requireArray;
using config::detail::requireField;
using config::detail::requireObject;
using config::detail::requireString;
using config::detail::requireUnsigned;
using config::detail::validateSchema;

constexpr std::uint64_t ns_per_ms = 1'000'000U;
constexpr std::uint64_t maximum_repeat_count = 100'000U;
constexpr std::size_t maximum_usart_bytes = 65'536U;

/// @brief Parses the optional fixed-count repeat block on one event.
Result<std::pair<std::uint64_t, std::uint32_t>> parseRepeat(
    const JsonValue::Object& event,
    const std::filesystem::path& path
) {
    const JsonValue* repeat = find(event, "repeat");
    if (repeat == nullptr) return std::pair<std::uint64_t, std::uint32_t>{0U, 1U};

    auto object = requireObject(*repeat, path, "stimulus event repeat");
    if (!object) return object.error();
    auto unknown = rejectUnknown(*object.value(), path, {"every_ms", "count"});
    if (!unknown) return unknown.error();
    auto interval = requireUnsigned(*object.value(), path, "every_ms");
    auto count = requireUnsigned(*object.value(), path, "count");
    if (!interval) return interval.error();
    if (!count) return count.error();
    if (interval.value() == 0U || count.value() < 2U || count.value() > maximum_repeat_count) {
        return configError(path, "stimulus repeat requires every_ms > 0 and count in [2, 100000]");
    }
    return std::pair<std::uint64_t, std::uint32_t>{
        interval.value(), static_cast<std::uint32_t>(count.value())
    };
}

/// @brief Parses a canonical GPIO pin name such as `PB9`.
Result<std::pair<std::string, std::uint8_t>> parsePin(
    const std::string_view name,
    const std::filesystem::path& path
) {
    if (name.size() < 3U || name[0] != 'P' || name[1] < 'A' || name[1] > 'G') {
        return configError(path, "stimulus GPIO pin must use a name from PA0 through PG15");
    }
    auto pin = config::parseUnsigned(name.substr(2U));
    if (!pin || pin.value() > 15U) {
        return configError(path, "stimulus GPIO pin number must be from 0 through 15");
    }
    return std::pair<std::string, std::uint8_t>{
        "GPIO" + std::string(1U, name[1]), static_cast<std::uint8_t>(pin.value())
    };
}

/// @brief Parses a JSON byte array with a caller-selected size limit.
Result<std::vector<std::uint8_t>> parseBytes(
    const JsonValue& value,
    const std::filesystem::path& path,
    const std::string_view key,
    const std::size_t maximum
) {
    auto bytes = requireArray(value, path, "key '" + std::string(key) + "'");
    if (!bytes) return bytes.error();
    if (bytes.value()->size() > maximum) {
        return configError(path, "key '" + std::string(key) + "' contains too many bytes");
    }
    std::vector<std::uint8_t> result;
    result.reserve(bytes.value()->size());
    for (const JsonValue& byte : *bytes.value()) {
        auto parsed = numericValue(byte, path, key);
        if (!parsed) return parsed.error();
        if (parsed.value() > 0xffU) {
            return configError(path, "key '" + std::string(key) + "' byte exceeds 255");
        }
        result.push_back(static_cast<std::uint8_t>(parsed.value()));
    }
    return result;
}

/// @brief Parses one type-specific external stimulus and its schedule.
Result<StimulusEvent> parseEvent(const JsonValue& value, const std::filesystem::path& path) {
    auto object = requireObject(value, path, "stimulus event");
    if (!object) return object.error();
    auto type = requireString(*object.value(), path, "type");
    auto at_ms = requireUnsigned(*object.value(), path, "at_ms");
    if (!type) return type.error();
    if (!at_ms) return at_ms.error();

    auto repeat = parseRepeat(*object.value(), path);
    if (!repeat) return repeat.error();
    if (at_ms.value() > std::numeric_limits<std::uint64_t>::max() / ns_per_ms
        || (repeat.value().second > 1U
            && repeat.value().first
                > (std::numeric_limits<std::uint64_t>::max() / ns_per_ms - at_ms.value())
                    / static_cast<std::uint64_t>(repeat.value().second - 1U))) {
        return configError(path, "stimulus event time is too large");
    }

    StimulusEvent event;
    event.at_ms = at_ms.value();
    event.repeat_every_ms = repeat.value().first;
    event.repeat_count = repeat.value().second;

    if (type.value() == "can") {
        auto unknown = rejectUnknown(
            *object.value(), path,
            {"type", "at_ms", "repeat", "bus", "id", "data", "extended", "fd", "brs"}
        );
        if (!unknown) return unknown.error();
        auto bus = requireString(*object.value(), path, "bus");
        auto id = requireUnsigned(*object.value(), path, "id");
        auto data_field = requireField(*object.value(), path, "data");
        if (!bus) return bus.error();
        if (!id) return id.error();
        if (!data_field) return data_field.error();
        if (id.value() > 0x1fffffffU) {
            return configError(path, "CAN identifier exceeds 29-bit range");
        }
        auto data = parseBytes(*data_field.value(), path, "data", 64U);
        if (!data) return data.error();

        CanStimulus input;
        input.bus = std::move(bus).value();
        input.id = static_cast<std::uint32_t>(id.value());
        input.extended = input.id > 0x7ffU;
        input.fd = data.value().size() > 8U;
        if (const JsonValue* extended = find(*object.value(), "extended")) {
            auto parsed = booleanValue(*extended, path, "extended");
            if (!parsed) return parsed.error();
            input.extended = parsed.value();
        }
        if (!input.extended && input.id > 0x7ffU) {
            return configError(path, "standard CAN identifiers must fit in 11 bits");
        }
        if (const JsonValue* fd = find(*object.value(), "fd")) {
            auto parsed = booleanValue(*fd, path, "fd");
            if (!parsed) return parsed.error();
            input.fd = parsed.value();
        }
        const std::size_t length = data.value().size();
        const bool valid_fd_length = length <= 8U || length == 12U || length == 16U
            || length == 20U || length == 24U || length == 32U || length == 48U
            || length == 64U;
        if ((!input.fd && length > 8U) || (input.fd && !valid_fd_length)) {
            return configError(
                path,
                "CAN payload length must be classic 0..8 or CAN-FD 0..8, 12, 16, 20, 24, 32, 48, or 64"
            );
        }
        if (const JsonValue* brs = find(*object.value(), "brs")) {
            auto parsed = booleanValue(*brs, path, "brs");
            if (!parsed) return parsed.error();
            input.brs = parsed.value();
        }
        if (input.brs && !input.fd) {
            return configError(path, "CAN bit-rate switching requires fd: true");
        }
        input.data = std::move(data).value();
        event.input = std::move(input);
    } else if (type.value() == "gpio") {
        auto unknown = rejectUnknown(
            *object.value(), path, {"type", "at_ms", "repeat", "board", "pin", "value"}
        );
        if (!unknown) return unknown.error();
        auto board = requireString(*object.value(), path, "board");
        auto pin_name = requireString(*object.value(), path, "pin");
        auto value_field = requireField(*object.value(), path, "value");
        if (!board) return board.error();
        if (!pin_name) return pin_name.error();
        if (!value_field) return value_field.error();
        auto pin = parsePin(pin_name.value(), path);
        if (!pin) return pin.error();

        GpioStimulus input;
        input.board = std::move(board).value();
        input.port = std::move(pin.value().first);
        input.pin = pin.value().second;
        if (std::holds_alternative<bool>(value_field.value()->storage)) {
            auto level = booleanValue(*value_field.value(), path, "value");
            if (!level) return level.error();
            input.level = level.value();
        } else if (const auto* text = std::get_if<std::string>(&value_field.value()->storage)) {
            if (*text != "release") {
                return configError(path, "GPIO stimulus value must be true, false, or 'release'");
            }
        } else {
            return configError(path, "GPIO stimulus value must be a boolean or 'release'");
        }
        event.input = std::move(input);
    } else if (type.value() == "adc") {
        auto unknown = rejectUnknown(
            *object.value(), path,
            {"type", "at_ms", "repeat", "board", "instance", "channel", "value"}
        );
        if (!unknown) return unknown.error();
        auto board = requireString(*object.value(), path, "board");
        auto instance = requireString(*object.value(), path, "instance");
        auto channel = requireUnsigned(*object.value(), path, "channel");
        auto input_value = requireUnsigned(*object.value(), path, "value");
        if (!board) return board.error();
        if (!instance) return instance.error();
        if (!channel) return channel.error();
        if (!input_value) return input_value.error();
        if (channel.value() > 19U || input_value.value() > 4095U) {
            return configError(path, "ADC stimulus requires channel 0..19 and value 0..4095");
        }
        event.input = AdcStimulus{
            std::move(board).value(), std::move(instance).value(),
            static_cast<std::uint8_t>(channel.value()),
            static_cast<std::uint16_t>(input_value.value()),
        };
    } else if (type.value() == "usart_rx" || type.value() == "usart") {
        auto unknown = rejectUnknown(
            *object.value(), path,
            {"type", "at_ms", "repeat", "board", "instance", "bytes"}
        );
        if (!unknown) return unknown.error();
        auto board = requireString(*object.value(), path, "board");
        auto instance = requireString(*object.value(), path, "instance");
        auto bytes_field = requireField(*object.value(), path, "bytes");
        if (!board) return board.error();
        if (!instance) return instance.error();
        if (!bytes_field) return bytes_field.error();
        auto bytes = parseBytes(*bytes_field.value(), path, "bytes", maximum_usart_bytes);
        if (!bytes) return bytes.error();
        event.input = UsartStimulus{
            std::move(board).value(), std::move(instance).value(), std::move(bytes).value()
        };
    } else {
        return configError(path, "unsupported stimulus event type '" + type.value() + "'");
    }
    return event;
}

Error targetError(const StimulusScript& script, std::string message) {
    return Error{
        ErrorCategory::config,
        std::move(message),
        SourceContext{script.source_path, 0U, 0U},
    };
}

struct BoundEvent {
    SimTimeNs at_ns{0};
    SimTimeNs repeat_every_ns{0};
    std::uint32_t repeat_count{1};
    std::function<void()> apply;
};

/// @brief Resolves one input target and creates its lifetime-safe callback.
Result<std::function<void()>> bindInput(const StimulusScript& script, const StimulusInput& input, World& world) {
    return std::visit([&](const auto& stimulus) -> Result<std::function<void()>> {
        using Input = std::decay_t<decltype(stimulus)>;
        if constexpr (std::is_same_v<Input, CanStimulus>) {
            devices::VirtualCanBus* bus = world.canBus(stimulus.bus);
            if (bus == nullptr) {
                return targetError(script, "stimulus '" + script.name + "' names undeclared CAN bus '" + stimulus.bus + "'");
            }
            devices::CanFrame frame;
            frame.id = stimulus.id;
            frame.extended = stimulus.extended;
            frame.fd = stimulus.fd;
            frame.brs = stimulus.brs;
            const auto dlc = devices::lengthToDlc(stimulus.data.size());
            if (!dlc) return targetError(script, "stimulus '" + script.name + "' has invalid CAN payload length");
            frame.dlc = *dlc;
            std::copy(stimulus.data.begin(), stimulus.data.end(), frame.data.begin());
            auto valid = devices::validate(frame);
            if (!valid) return targetError(script, "stimulus '" + script.name + "' has invalid CAN frame: " + valid.error().message);
            return std::function<void()>{[bus, frame, loop = &world.eventLoop()] {
                static_cast<void>(bus->inject(frame, loop->now()));
            }};
        } else {
            Board* board = world.board(stimulus.board);
            if (board == nullptr) {
                return targetError(script, "stimulus '" + script.name + "' names unknown board '" + stimulus.board + "'");
            }
            if constexpr (std::is_same_v<Input, GpioStimulus>) {
                stm32g4::GpioPeripheral* gpio = board->peripherals().gpio(stimulus.port);
                if (gpio == nullptr) {
                    return targetError(script, "stimulus '" + script.name + "' names unknown GPIO port '" + stimulus.port + "' on '" + stimulus.board + "'");
                }
                const std::uint8_t pin = stimulus.pin;
                const std::optional<bool> level = stimulus.level;
                return std::function<void()>{[gpio, pin, level] {
                    if (level.has_value()) gpio->setInput(pin, level.value());
                    else gpio->releaseInput(pin);
                }};
            } else if constexpr (std::is_same_v<Input, AdcStimulus>) {
                stm32g4::AdcPeripheral* adc = board->peripherals().adc(stimulus.instance);
                if (adc == nullptr) {
                    return targetError(script, "stimulus '" + script.name + "' names unknown ADC '" + stimulus.instance + "' on '" + stimulus.board + "'");
                }
                const std::uint8_t channel = stimulus.channel;
                const std::uint16_t value = stimulus.value;
                return std::function<void()>{[adc, channel, value] {
                    adc->setChannelValue(channel, value);
                }};
            } else {
                stm32g4::UsartPeripheral* usart = board->peripherals().usart(stimulus.instance);
                if (usart == nullptr) {
                    return targetError(script, "stimulus '" + script.name + "' names unknown USART '" + stimulus.instance + "' on '" + stimulus.board + "'");
                }
                auto bytes = std::make_shared<const std::vector<std::uint8_t>>(stimulus.bytes);
                return std::function<void()>{[usart, bytes] { usart->injectRx(*bytes); }};
            }
        }
    }, input);
}

} // namespace

Result<StimulusScript> loadStimulusScript(const std::filesystem::path& path) {
    auto source_path = absoluteNormalized(path);
    if (!source_path) return source_path.error();
    auto json = loadJson(source_path.value());
    if (!json) return json.error();
    auto object = requireObject(json.value(), source_path.value(), "stimulus script root");
    if (!object) return object.error();
    auto unknown = rejectUnknown(*object.value(), source_path.value(), {"schema_version", "name", "events"});
    if (!unknown) return unknown.error();
    auto schema = validateSchema(*object.value(), source_path.value());
    if (!schema) return schema.error();
    auto name = requireString(*object.value(), source_path.value(), "name");
    auto events_field = requireField(*object.value(), source_path.value(), "events");
    if (!name) return name.error();
    if (!events_field) return events_field.error();
    auto events = requireArray(*events_field.value(), source_path.value(), "key 'events'");
    if (!events) return events.error();
    if (events.value()->empty()) {
        return configError(source_path.value(), "stimulus script must contain at least one event");
    }

    StimulusScript script;
    script.name = std::move(name).value();
    script.source_path = source_path.value();
    for (const JsonValue& event : *events.value()) {
        auto parsed = parseEvent(event, source_path.value());
        if (!parsed) return parsed.error();
        script.events.push_back(std::move(parsed).value());
    }
    return script;
}

Result<void> scheduleStimulusScript(const StimulusScript& script, World& world) {
    std::vector<BoundEvent> bound_events;
    bound_events.reserve(script.events.size());
    for (const StimulusEvent& event : script.events) {
        if (event.repeat_count == 0U || (event.repeat_count > 1U && event.repeat_every_ms == 0U)) {
            return targetError(script, "stimulus '" + script.name + "' has an invalid repeat schedule");
        }
        if (event.at_ms > std::numeric_limits<SimTimeNs>::max() / ns_per_ms
            || (event.repeat_count > 1U
                && event.repeat_every_ms
                    > (std::numeric_limits<SimTimeNs>::max() / ns_per_ms - event.at_ms)
                        / static_cast<std::uint64_t>(event.repeat_count - 1U))) {
            return targetError(script, "stimulus '" + script.name + "' event time is too large");
        }
        auto apply = bindInput(script, event.input, world);
        if (!apply) return apply.error();
        bound_events.push_back(BoundEvent{
            event.at_ms * ns_per_ms,
            event.repeat_every_ms * ns_per_ms,
            event.repeat_count,
            std::move(apply).value(),
        });
    }

    for (const BoundEvent& event : bound_events) {
        auto apply = std::make_shared<const std::function<void()>>(event.apply);
        for (std::uint32_t index = 0U; index < event.repeat_count; ++index) {
            const SimTimeNs at_ns = event.at_ns
                + static_cast<SimTimeNs>(index) * event.repeat_every_ns;
            static_cast<void>(world.eventLoop().scheduleAt(at_ns, [apply] { (*apply)(); }));
        }
    }
    return {};
}

std::string normalizeStimulusScript(const StimulusScript& script) {
    std::ostringstream output;
    output << "type: stimulus\n"
           << "name: " << script.name << '\n'
           << "source: " << script.source_path.string() << '\n'
           << "events: " << script.events.size() << '\n';
    for (const StimulusEvent& event : script.events) {
        output << "event: at_ms=" << event.at_ms;
        if (event.repeat_count > 1U) {
            output << " every_ms=" << event.repeat_every_ms << " count=" << event.repeat_count;
        }
        output << '\n';
    }
    return output.str();
}

} // namespace fil::sim
