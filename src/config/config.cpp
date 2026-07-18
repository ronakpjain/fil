#include "fil/config/config.hpp"

#include <charconv>
#include <cctype>
#include <fstream>
#include <limits>
#include <sstream>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

namespace fil::config {
namespace {

struct JsonValue {
    using Object = std::vector<std::pair<std::string, JsonValue>>;
    using Array = std::vector<JsonValue>;
    using Storage = std::variant<std::nullptr_t, bool, std::uint64_t, std::string, Object, Array>;

    Storage storage;
};

class JsonParser {
public:
    /// @brief Creates a parser over one config file's text.
    JsonParser(std::string_view input, std::filesystem::path path)
        : input_(input), path_(std::move(path)) {}

    /// @brief Parses one complete JSON document.
    Result<JsonValue> parse() {
        skipWhitespace();
        auto value = parseValue();
        if (!value) {
            return value.error();
        }
        skipWhitespace();
        if (position_ != input_.size()) {
            return makeError("unexpected characters after JSON value");
        }
        return std::move(value).value();
    }

private:
    /// @brief Parses the next JSON value based on its leading byte.
    Result<JsonValue> parseValue() {
        if (position_ == input_.size()) {
            return makeError("expected a JSON value");
        }

        switch (input_[position_]) {
        case '{':
            return parseObject();
        case '[':
            return parseArray();
        case '"': {
            auto string = parseString();
            if (!string) {
                return string.error();
            }
            return JsonValue{std::move(string).value()};
        }
        case 't':
            return parseLiteral("true", JsonValue{true});
        case 'f':
            return parseLiteral("false", JsonValue{false});
        case 'n':
            return parseLiteral("null", JsonValue{nullptr});
        default:
            if (std::isdigit(static_cast<unsigned char>(input_[position_])) != 0) {
                return parseNumber();
            }
            return makeError("expected an object, array, string, unsigned integer, boolean, or null");
        }
    }

    /// @brief Parses an object while rejecting duplicate keys.
    Result<JsonValue> parseObject() {
        ++position_;
        skipWhitespace();
        JsonValue::Object object;
        if (consume('}')) {
            return JsonValue{std::move(object)};
        }

        while (true) {
            if (position_ == input_.size() || input_[position_] != '"') {
                return makeError("expected a quoted object key");
            }
            auto key = parseString();
            if (!key) {
                return key.error();
            }
            for (const auto& [existing, value] : object) {
                static_cast<void>(value);
                if (existing == key.value()) {
                    return makeError("duplicate object key '" + key.value() + "'");
                }
            }

            skipWhitespace();
            if (!consume(':')) {
                return makeError("expected ':' after object key");
            }
            skipWhitespace();
            auto value = parseValue();
            if (!value) {
                return value.error();
            }
            object.emplace_back(std::move(key).value(), std::move(value).value());

            skipWhitespace();
            if (consume('}')) {
                return JsonValue{std::move(object)};
            }
            if (!consume(',')) {
                return makeError("expected ',' or '}' in object");
            }
            skipWhitespace();
        }
    }

    /// @brief Parses a JSON array.
    Result<JsonValue> parseArray() {
        ++position_;
        skipWhitespace();
        JsonValue::Array array;
        if (consume(']')) {
            return JsonValue{std::move(array)};
        }

        while (true) {
            auto value = parseValue();
            if (!value) {
                return value.error();
            }
            array.push_back(std::move(value).value());
            skipWhitespace();
            if (consume(']')) {
                return JsonValue{std::move(array)};
            }
            if (!consume(',')) {
                return makeError("expected ',' or ']' in array");
            }
            skipWhitespace();
        }
    }

    /// @brief Parses and unescapes a JSON string.
    Result<std::string> parseString() {
        ++position_;
        std::string result;
        while (position_ < input_.size()) {
            const char character = input_[position_++];
            if (character == '"') {
                return result;
            }
            if (static_cast<unsigned char>(character) < 0x20U) {
                return makeError("unescaped control character in string");
            }
            if (character != '\\') {
                result.push_back(character);
                continue;
            }
            if (position_ == input_.size()) {
                return makeError("unterminated string escape");
            }
            const char escaped = input_[position_++];
            switch (escaped) {
            case '"':
            case '\\':
            case '/':
                result.push_back(escaped);
                break;
            case 'b':
                result.push_back('\b');
                break;
            case 'f':
                result.push_back('\f');
                break;
            case 'n':
                result.push_back('\n');
                break;
            case 'r':
                result.push_back('\r');
                break;
            case 't':
                result.push_back('\t');
                break;
            case 'u': {
                auto codepoint = parseUnicodeEscape();
                if (!codepoint) {
                    return codepoint.error();
                }
                appendUtf8(result, codepoint.value());
                break;
            }
            default:
                return makeError("invalid string escape");
            }
        }
        return makeError("unterminated string");
    }

    /// @brief Parses one four-digit JSON Unicode escape.
    Result<std::uint32_t> parseUnicodeEscape() {
        if (input_.size() - position_ < 4) {
            return makeError("incomplete Unicode escape");
        }
        std::uint32_t value = 0;
        for (int index = 0; index < 4; ++index) {
            const char digit = input_[position_++];
            value <<= 4U;
            if (digit >= '0' && digit <= '9') {
                value |= static_cast<std::uint32_t>(digit - '0');
            } else if (digit >= 'a' && digit <= 'f') {
                value |= static_cast<std::uint32_t>(digit - 'a' + 10);
            } else if (digit >= 'A' && digit <= 'F') {
                value |= static_cast<std::uint32_t>(digit - 'A' + 10);
            } else {
                return makeError("invalid Unicode escape");
            }
        }
        if (value >= 0xd800U && value <= 0xdfffU) {
            return makeError("Unicode surrogate pairs are not supported in config files");
        }
        return value;
    }

    /// @brief Encodes a Unicode code point as UTF-8.
    static void appendUtf8(std::string& output, const std::uint32_t codepoint) {
        if (codepoint <= 0x7fU) {
            output.push_back(static_cast<char>(codepoint));
        } else if (codepoint <= 0x7ffU) {
            output.push_back(static_cast<char>(0xc0U | (codepoint >> 6U)));
            output.push_back(static_cast<char>(0x80U | (codepoint & 0x3fU)));
        } else {
            output.push_back(static_cast<char>(0xe0U | (codepoint >> 12U)));
            output.push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3fU)));
            output.push_back(static_cast<char>(0x80U | (codepoint & 0x3fU)));
        }
    }

    /// @brief Parses a config-supported unsigned JSON integer.
    Result<JsonValue> parseNumber() {
        const std::size_t begin = position_;
        while (position_ < input_.size()
               && std::isdigit(static_cast<unsigned char>(input_[position_])) != 0) {
            ++position_;
        }
        if (position_ < input_.size()
            && (input_[position_] == '.' || input_[position_] == 'e' || input_[position_] == 'E')) {
            return makeError("configuration numbers must be unsigned integers");
        }

        std::uint64_t value = 0;
        const std::string_view digits = input_.substr(begin, position_ - begin);
        const auto conversion = std::from_chars(digits.data(), digits.data() + digits.size(), value);
        if (conversion.ec != std::errc{} || conversion.ptr != digits.data() + digits.size()) {
            return makeError("unsigned integer is out of range");
        }
        return JsonValue{value};
    }

    /// @brief Parses a fixed JSON literal such as true, false, or null.
    Result<JsonValue> parseLiteral(const std::string_view literal, JsonValue value) {
        if (input_.substr(position_, literal.size()) != literal) {
            return makeError("invalid JSON literal");
        }
        position_ += literal.size();
        return value;
    }

    /// @brief Advances over JSON whitespace.
    void skipWhitespace() {
        while (position_ < input_.size()) {
            const char character = input_[position_];
            if (character != ' ' && character != '\t' && character != '\r' && character != '\n') {
                break;
            }
            ++position_;
        }
    }

    /// @brief Consumes an expected byte when present.
    bool consume(const char expected) {
        if (position_ < input_.size() && input_[position_] == expected) {
            ++position_;
            return true;
        }
        return false;
    }

    /// @brief Creates a parse error at the current line and column.
    Error makeError(std::string message) const {
        std::size_t line = 1;
        std::size_t column = 1;
        for (std::size_t index = 0; index < position_ && index < input_.size(); ++index) {
            if (input_[index] == '\n') {
                ++line;
                column = 1;
            } else {
                ++column;
            }
        }
        return Error{
            ErrorCategory::parse,
            std::move(message),
            SourceContext{path_, line, column},
        };
    }

    std::string_view input_;
    std::filesystem::path path_;
    std::size_t position_{0};
};

/// @brief Creates a semantic config error associated with a file.
Error configError(const std::filesystem::path& path, std::string message) {
    return Error{ErrorCategory::config, std::move(message), SourceContext{path, 0, 0}};
}

/// @brief Reads and parses one JSON configuration document.
Result<JsonValue> loadJson(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return Error{ErrorCategory::io, "unable to open config file", SourceContext{path, 0, 0}};
    }
    std::ostringstream contents;
    contents << input.rdbuf();
    if (input.bad()) {
        return Error{ErrorCategory::io, "failed while reading config file", SourceContext{path, 0, 0}};
    }
    const std::string text = contents.str();
    return JsonParser(text, path).parse();
}

/// @brief Finds an object member without modifying insertion order.
const JsonValue* find(const JsonValue::Object& object, const std::string_view key) {
    for (const auto& [candidate, value] : object) {
        if (candidate == key) {
            return &value;
        }
    }
    return nullptr;
}

/// @brief Validates that a JSON value is an object.
Result<const JsonValue::Object*> requireObject(
    const JsonValue& value,
    const std::filesystem::path& path,
    const std::string_view description
) {
    const auto* object = std::get_if<JsonValue::Object>(&value.storage);
    if (object == nullptr) {
        return configError(path, std::string(description) + " must be an object");
    }
    return object;
}

/// @brief Gets a required object field.
Result<const JsonValue*> requireField(
    const JsonValue::Object& object,
    const std::filesystem::path& path,
    const std::string_view key
) {
    const JsonValue* value = find(object, key);
    if (value == nullptr) {
        return configError(path, "missing required key '" + std::string(key) + "'");
    }
    return value;
}

/// @brief Rejects object keys outside an explicit schema allowlist.
Result<void> rejectUnknown(
    const JsonValue::Object& object,
    const std::filesystem::path& path,
    const std::initializer_list<std::string_view> allowed
) {
    for (const auto& [key, value] : object) {
        static_cast<void>(value);
        bool known = false;
        for (const std::string_view candidate : allowed) {
            if (key == candidate) {
                known = true;
                break;
            }
        }
        if (!known) {
            return configError(path, "unknown key '" + key + "'");
        }
    }
    return {};
}

/// @brief Gets a required string field.
Result<std::string> requireString(
    const JsonValue::Object& object,
    const std::filesystem::path& path,
    const std::string_view key
) {
    auto field = requireField(object, path, key);
    if (!field) {
        return field.error();
    }
    const auto* string = std::get_if<std::string>(&field.value()->storage);
    if (string == nullptr) {
        return configError(path, "key '" + std::string(key) + "' must be a string");
    }
    return *string;
}

/// @brief Converts a JSON value to a string with a field-oriented diagnostic.
Result<std::string> stringValue(
    const JsonValue& value,
    const std::filesystem::path& path,
    const std::string_view key
) {
    const auto* string = std::get_if<std::string>(&value.storage);
    if (string == nullptr) {
        return configError(path, "key '" + std::string(key) + "' must be a string");
    }
    return *string;
}

/// @brief Converts a JSON value to a Boolean with a field-oriented diagnostic.
Result<bool> booleanValue(
    const JsonValue& value,
    const std::filesystem::path& path,
    const std::string_view key
) {
    const auto* boolean = std::get_if<bool>(&value.storage);
    if (boolean == nullptr) {
        return configError(path, "key '" + std::string(key) + "' must be a boolean");
    }
    return *boolean;
}

/// @brief Validates that a JSON value is an array.
Result<const JsonValue::Array*> requireArray(
    const JsonValue& value,
    const std::filesystem::path& path,
    const std::string_view description
) {
    const auto* array = std::get_if<JsonValue::Array>(&value.storage);
    if (array == nullptr) {
        return configError(path, std::string(description) + " must be an array");
    }
    return array;
}

/// @brief Converts a JSON integer or numeric string to an unsigned value.
Result<std::uint64_t> numericValue(
    const JsonValue& value,
    const std::filesystem::path& path,
    const std::string_view key
) {
    if (const auto* number = std::get_if<std::uint64_t>(&value.storage)) {
        return *number;
    }
    if (const auto* string = std::get_if<std::string>(&value.storage)) {
        auto parsed = parseUnsigned(*string);
        if (parsed) {
            return parsed.value();
        }
        return configError(path, "invalid value for key '" + std::string(key) + "': " + parsed.error().message);
    }
    return configError(path, "key '" + std::string(key) + "' must be an integer or numeric string");
}

/// @brief Gets a required unsigned numeric field.
Result<std::uint64_t> requireUnsigned(
    const JsonValue::Object& object,
    const std::filesystem::path& path,
    const std::string_view key
) {
    auto field = requireField(object, path, key);
    if (!field) {
        return field.error();
    }
    return numericValue(*field.value(), path, key);
}

/// @brief Gets a required unsigned field constrained to 32 bits.
Result<std::uint32_t> requireUint32(
    const JsonValue::Object& object,
    const std::filesystem::path& path,
    const std::string_view key
) {
    auto value = requireUnsigned(object, path, key);
    if (!value) {
        return value.error();
    }
    if (value.value() > std::numeric_limits<std::uint32_t>::max()) {
        return configError(path, "key '" + std::string(key) + "' exceeds 32-bit range");
    }
    return static_cast<std::uint32_t>(value.value());
}

/// @brief Validates the required config schema version.
Result<void> validateSchema(const JsonValue::Object& object, const std::filesystem::path& path) {
    auto version = requireUint32(object, path, "schema_version");
    if (!version) {
        return version.error();
    }
    if (version.value() != current_schema_version) {
        return configError(
            path,
            "unsupported schema_version " + std::to_string(version.value())
                + "; expected " + std::to_string(current_schema_version)
        );
    }
    return {};
}

/// @brief Converts a path to an absolute, lexically normalized path.
Result<std::filesystem::path> absoluteNormalized(const std::filesystem::path& path) {
    std::error_code error;
    auto absolute = std::filesystem::absolute(path, error);
    if (error) {
        return Error{ErrorCategory::io, "unable to resolve absolute path: " + error.message(), SourceContext{path, 0, 0}};
    }
    return absolute.lexically_normal();
}

/// @brief Resolves a referenced path relative to its containing config file.
std::filesystem::path resolveRelative(
    const std::filesystem::path& config_path,
    const std::filesystem::path& referenced_path
) {
    if (referenced_path.is_absolute()) {
        return referenced_path.lexically_normal();
    }
    return (config_path.parent_path() / referenced_path).lexically_normal();
}

/// @brief Parses the board `gpio` object keyed by pin name.
Result<std::vector<GpioPinConfig>> parseGpio(
    const JsonValue& value,
    const std::filesystem::path& path
) {
    auto pins = requireObject(value, path, "key 'gpio'");
    if (!pins) return pins.error();
    std::vector<GpioPinConfig> result;
    for (const auto& [pin, definition] : *pins.value()) {
        auto object = requireObject(definition, path, "GPIO pin '" + pin + "'");
        if (!object) return object.error();
        auto unknown = rejectUnknown(*object.value(), path, {"mode", "value", "trace"});
        if (!unknown) return unknown.error();
        GpioPinConfig config;
        config.pin = pin;
        if (const JsonValue* mode = find(*object.value(), "mode")) {
            auto parsed = stringValue(*mode, path, pin + ".mode");
            if (!parsed) return parsed.error();
            config.mode = std::move(parsed).value();
            if (config.mode != "input" && config.mode != "output"
                && config.mode != "alternate" && config.mode != "analog") {
                return configError(path, "GPIO pin '" + pin + "' has unsupported mode '" + config.mode + "'");
            }
        }
        if (const JsonValue* initial = find(*object.value(), "value")) {
            auto parsed = booleanValue(*initial, path, pin + ".value");
            if (!parsed) return parsed.error();
            config.value = parsed.value();
        }
        if (const JsonValue* trace = find(*object.value(), "trace")) {
            auto parsed = booleanValue(*trace, path, pin + ".trace");
            if (!parsed) return parsed.error();
            config.trace = parsed.value();
        }
        result.push_back(std::move(config));
    }
    return result;
}

/// @brief Parses the board `can` object keyed by controller name.
Result<std::vector<CanControllerConfig>> parseCan(
    const JsonValue& value,
    const std::filesystem::path& path
) {
    auto controllers = requireObject(value, path, "key 'can'");
    if (!controllers) return controllers.error();
    std::vector<CanControllerConfig> result;
    for (const auto& [instance, definition] : *controllers.value()) {
        auto object = requireObject(definition, path, "CAN controller '" + instance + "'");
        if (!object) return object.error();
        auto unknown = rejectUnknown(*object.value(), path, {"bus", "loopback"});
        if (!unknown) return unknown.error();
        auto bus = requireString(*object.value(), path, "bus");
        if (!bus) return bus.error();
        CanControllerConfig config{instance, std::move(bus).value(), false};
        if (const JsonValue* loopback = find(*object.value(), "loopback")) {
            auto parsed = booleanValue(*loopback, path, instance + ".loopback");
            if (!parsed) return parsed.error();
            config.loopback = parsed.value();
        }
        result.push_back(std::move(config));
    }
    return result;
}

/// @brief Parses the board `usart` object keyed by peripheral name.
Result<std::vector<UsartConfig>> parseUsart(
    const JsonValue& value,
    const std::filesystem::path& path
) {
    auto controllers = requireObject(value, path, "key 'usart'");
    if (!controllers) return controllers.error();
    std::vector<UsartConfig> result;
    for (const auto& [instance, definition] : *controllers.value()) {
        auto object = requireObject(definition, path, "USART '" + instance + "'");
        if (!object) return object.error();
        auto unknown = rejectUnknown(*object.value(), path, {"tx_log", "rx"});
        if (!unknown) return unknown.error();
        UsartConfig config;
        config.instance = instance;
        if (const JsonValue* log = find(*object.value(), "tx_log")) {
            auto parsed = stringValue(*log, path, instance + ".tx_log");
            if (!parsed) return parsed.error();
            config.tx_log = resolveRelative(path, parsed.value());
        }
        if (const JsonValue* rx = find(*object.value(), "rx")) {
            auto bytes = requireArray(*rx, path, "key '" + instance + ".rx'");
            if (!bytes) return bytes.error();
            for (const JsonValue& byte : *bytes.value()) {
                auto parsed = numericValue(byte, path, instance + ".rx");
                if (!parsed) return parsed.error();
                if (parsed.value() > 0xffU) {
                    return configError(path, "scripted USART RX byte exceeds 255");
                }
                config.scripted_rx.push_back(static_cast<std::uint8_t>(parsed.value()));
            }
        }
        result.push_back(std::move(config));
    }
    return result;
}

/// @brief Parses one ADC channel scalar or waveform object.
Result<AdcChannelConfig> parseAdcChannel(
    const JsonValue& value,
    const std::filesystem::path& path,
    const std::string_view description
) {
    if (std::holds_alternative<std::uint64_t>(value.storage)
        || std::holds_alternative<std::string>(value.storage)) {
        auto parsed = numericValue(value, path, description);
        if (!parsed) return parsed.error();
        if (parsed.value() > 4095U) return configError(path, "ADC value exceeds 12-bit range");
        AdcChannelConfig config;
        config.value = static_cast<std::uint16_t>(parsed.value());
        return config;
    }
    auto object = requireObject(value, path, std::string(description));
    if (!object) return object.error();
    auto unknown = rejectUnknown(*object.value(), path, {"type", "value", "min", "max", "period_ms"});
    if (!unknown) return unknown.error();
    auto type = requireString(*object.value(), path, "type");
    if (!type) return type.error();
    AdcChannelConfig config;
    if (type.value() == "constant") {
        auto channel_value = requireUnsigned(*object.value(), path, "value");
        if (!channel_value) return channel_value.error();
        if (channel_value.value() > 4095U) return configError(path, "ADC value exceeds 12-bit range");
        config.value = static_cast<std::uint16_t>(channel_value.value());
    } else if (type.value() == "sine") {
        config.kind = AdcChannelConfig::Kind::sine;
        auto minimum = requireUnsigned(*object.value(), path, "min");
        auto maximum = requireUnsigned(*object.value(), path, "max");
        auto period = requireUnsigned(*object.value(), path, "period_ms");
        if (!minimum) return minimum.error();
        if (!maximum) return maximum.error();
        if (!period) return period.error();
        if (minimum.value() > maximum.value() || maximum.value() > 4095U || period.value() == 0) {
            return configError(path, "ADC sine input requires 0 <= min <= max <= 4095 and nonzero period_ms");
        }
        config.minimum = static_cast<std::uint16_t>(minimum.value());
        config.maximum = static_cast<std::uint16_t>(maximum.value());
        config.period_ms = period.value();
    } else {
        return configError(path, "unsupported ADC source type '" + type.value() + "'");
    }
    return config;
}

/// @brief Parses the board `adc` object keyed by peripheral name.
Result<std::vector<AdcConfig>> parseAdc(
    const JsonValue& value,
    const std::filesystem::path& path
) {
    auto controllers = requireObject(value, path, "key 'adc'");
    if (!controllers) return controllers.error();
    std::vector<AdcConfig> result;
    for (const auto& [instance, definition] : *controllers.value()) {
        auto object = requireObject(definition, path, "ADC '" + instance + "'");
        if (!object) return object.error();
        auto unknown = rejectUnknown(*object.value(), path, {"channels"});
        if (!unknown) return unknown.error();
        auto channels_field = requireField(*object.value(), path, "channels");
        if (!channels_field) return channels_field.error();
        auto channels = requireObject(*channels_field.value(), path, "ADC channels");
        if (!channels) return channels.error();
        AdcConfig config;
        config.instance = instance;
        for (const auto& [channel_name, source] : *channels.value()) {
            auto channel = parseUnsigned(channel_name);
            if (!channel || channel.value() > 31U) {
                return configError(path, "ADC channel name must be an integer from 0 through 31");
            }
            auto parsed = parseAdcChannel(source, path, instance + ".channels." + channel_name);
            if (!parsed) return parsed.error();
            config.channels.emplace(static_cast<std::uint8_t>(channel.value()), std::move(parsed).value());
        }
        result.push_back(std::move(config));
    }
    return result;
}

/// @brief Parses the board `spi` object keyed by peripheral name.
Result<std::vector<SpiConfig>> parseSpi(
    const JsonValue& value,
    const std::filesystem::path& path
) {
    auto controllers = requireObject(value, path, "key 'spi'");
    if (!controllers) return controllers.error();
    std::vector<SpiConfig> result;
    for (const auto& [instance, definition] : *controllers.value()) {
        auto object = requireObject(definition, path, "SPI '" + instance + "'");
        if (!object) return object.error();
        auto unknown = rejectUnknown(*object.value(), path, {"device"});
        if (!unknown) return unknown.error();
        SpiConfig config;
        config.instance = instance;
        if (const JsonValue* device = find(*object.value(), "device")) {
            auto parsed = stringValue(*device, path, instance + ".device");
            if (!parsed) return parsed.error();
            config.device = std::move(parsed).value();
        }
        result.push_back(std::move(config));
    }
    return result;
}

/// @brief Formats a 32-bit config value as fixed-width hexadecimal.
std::string hex32(const std::uint32_t value) {
    constexpr char digits[] = "0123456789abcdef";
    std::string result(10, '0');
    result[0] = '0';
    result[1] = 'x';
    for (std::size_t index = 0; index < 8; ++index) {
        const unsigned int shift = static_cast<unsigned int>((7U - index) * 4U);
        result[index + 2] = digits[(value >> shift) & 0xfU];
    }
    return result;
}

} // namespace

Result<std::uint64_t> parseUnsigned(const std::string_view text) {
    if (text.empty()) {
        return Error{ErrorCategory::invalid_argument, "numeric value is empty", std::nullopt};
    }

    std::uint64_t multiplier = 1;
    std::string_view digits = text;
    const char suffix = text.back();
    if (suffix == 'K' || suffix == 'k') {
        multiplier = 1024;
        digits.remove_suffix(1);
    } else if (suffix == 'M' || suffix == 'm') {
        multiplier = 1024 * 1024;
        digits.remove_suffix(1);
    }
    if (digits.empty()) {
        return Error{ErrorCategory::invalid_argument, "numeric value has no digits", std::nullopt};
    }

    int base = 10;
    if (digits.size() >= 2 && digits[0] == '0' && (digits[1] == 'x' || digits[1] == 'X')) {
        base = 16;
        digits.remove_prefix(2);
    }
    if (digits.empty()) {
        return Error{ErrorCategory::invalid_argument, "numeric value has no digits", std::nullopt};
    }

    std::uint64_t value = 0;
    const auto conversion = std::from_chars(digits.data(), digits.data() + digits.size(), value, base);
    if (conversion.ec != std::errc{} || conversion.ptr != digits.data() + digits.size()) {
        return Error{ErrorCategory::invalid_argument, "invalid or out-of-range unsigned integer", std::nullopt};
    }
    if (value > std::numeric_limits<std::uint64_t>::max() / multiplier) {
        return Error{ErrorCategory::invalid_argument, "numeric value overflows after size suffix", std::nullopt};
    }
    return value * multiplier;
}

Result<McuConfig> loadMcuConfig(const std::filesystem::path& path) {
    auto source_path = absoluteNormalized(path);
    if (!source_path) {
        return source_path.error();
    }
    auto json = loadJson(source_path.value());
    if (!json) {
        return json.error();
    }
    auto object = requireObject(json.value(), source_path.value(), "MCU config root");
    if (!object) {
        return object.error();
    }
    auto unknown = rejectUnknown(
        *object.value(), source_path.value(),
        {"schema_version", "name", "flash_base", "flash_size", "sram_base", "sram_size",
         "ccm_sram_base", "ccm_sram_size", "hse_hz"}
    );
    if (!unknown) {
        return unknown.error();
    }
    auto schema = validateSchema(*object.value(), source_path.value());
    if (!schema) {
        return schema.error();
    }

    auto name = requireString(*object.value(), source_path.value(), "name");
    auto flash_base = requireUint32(*object.value(), source_path.value(), "flash_base");
    auto flash_size = requireUint32(*object.value(), source_path.value(), "flash_size");
    auto sram_base = requireUint32(*object.value(), source_path.value(), "sram_base");
    auto sram_size = requireUint32(*object.value(), source_path.value(), "sram_size");
    auto ccm_base = requireUint32(*object.value(), source_path.value(), "ccm_sram_base");
    auto ccm_size = requireUint32(*object.value(), source_path.value(), "ccm_sram_size");
    std::uint32_t hse_hz = 8'000'000U;
    if (const JsonValue* configured_hse = find(*object.value(), "hse_hz")) {
        auto parsed = numericValue(*configured_hse, source_path.value(), "hse_hz");
        if (!parsed) return parsed.error();
        if (parsed.value() == 0U || parsed.value() > std::numeric_limits<std::uint32_t>::max()) {
            return configError(source_path.value(), "hse_hz must be a nonzero 32-bit frequency");
        }
        hse_hz = static_cast<std::uint32_t>(parsed.value());
    }
    if (!name) return name.error();
    if (!flash_base) return flash_base.error();
    if (!flash_size) return flash_size.error();
    if (!sram_base) return sram_base.error();
    if (!sram_size) return sram_size.error();
    if (!ccm_base) return ccm_base.error();
    if (!ccm_size) return ccm_size.error();

    return McuConfig{
        current_schema_version,
        std::move(name).value(),
        flash_base.value(), flash_size.value(),
        sram_base.value(), sram_size.value(),
        ccm_base.value(), ccm_size.value(), hse_hz,
    };
}

Result<BoardConfig> loadBoardConfig(const std::filesystem::path& path) {
    auto source_path = absoluteNormalized(path);
    if (!source_path) {
        return source_path.error();
    }
    auto json = loadJson(source_path.value());
    if (!json) {
        return json.error();
    }
    auto object = requireObject(json.value(), source_path.value(), "board config root");
    if (!object) {
        return object.error();
    }
    auto unknown = rejectUnknown(
        *object.value(), source_path.value(),
        {"schema_version", "name", "mcu", "elf", "vector_base", "run", "gpio", "can", "usart", "adc", "spi"}
    );
    if (!unknown) {
        return unknown.error();
    }
    auto schema = validateSchema(*object.value(), source_path.value());
    if (!schema) {
        return schema.error();
    }

    auto name = requireString(*object.value(), source_path.value(), "name");
    auto mcu = requireString(*object.value(), source_path.value(), "mcu");
    auto elf = requireString(*object.value(), source_path.value(), "elf");
    if (!name) return name.error();
    if (!mcu) return mcu.error();
    if (!elf) return elf.error();

    BoardConfig config;
    config.name = std::move(name).value();
    config.source_path = source_path.value();
    config.mcu_path = resolveRelative(config.source_path, mcu.value());
    config.elf_path = resolveRelative(config.source_path, elf.value());

    if (const JsonValue* vector = find(*object.value(), "vector_base")) {
        auto parsed = numericValue(*vector, source_path.value(), "vector_base");
        if (!parsed) return parsed.error();
        if (parsed.value() > std::numeric_limits<std::uint32_t>::max()) {
            return configError(source_path.value(), "key 'vector_base' exceeds 32-bit range");
        }
        config.vector_base = static_cast<std::uint32_t>(parsed.value());
    }

    if (const JsonValue* run = find(*object.value(), "run")) {
        auto run_object = requireObject(*run, source_path.value(), "key 'run'");
        if (!run_object) return run_object.error();
        auto run_unknown = rejectUnknown(
            *run_object.value(), source_path.value(), {"default_duration_ms", "max_instructions"}
        );
        if (!run_unknown) return run_unknown.error();
        if (const JsonValue* duration = find(*run_object.value(), "default_duration_ms")) {
            auto parsed = numericValue(*duration, source_path.value(), "default_duration_ms");
            if (!parsed) return parsed.error();
            config.run.default_duration_ms = parsed.value();
        }
        if (const JsonValue* instructions = find(*run_object.value(), "max_instructions")) {
            auto parsed = numericValue(*instructions, source_path.value(), "max_instructions");
            if (!parsed) return parsed.error();
            config.run.max_instructions = parsed.value();
        }
    }

    if (const JsonValue* gpio = find(*object.value(), "gpio")) {
        auto parsed = parseGpio(*gpio, source_path.value());
        if (!parsed) return parsed.error();
        config.gpio = std::move(parsed).value();
    }
    if (const JsonValue* can = find(*object.value(), "can")) {
        auto parsed = parseCan(*can, source_path.value());
        if (!parsed) return parsed.error();
        config.can = std::move(parsed).value();
    }
    if (const JsonValue* usart = find(*object.value(), "usart")) {
        auto parsed = parseUsart(*usart, source_path.value());
        if (!parsed) return parsed.error();
        config.usart = std::move(parsed).value();
    }
    if (const JsonValue* adc = find(*object.value(), "adc")) {
        auto parsed = parseAdc(*adc, source_path.value());
        if (!parsed) return parsed.error();
        config.adc = std::move(parsed).value();
    }
    if (const JsonValue* spi = find(*object.value(), "spi")) {
        auto parsed = parseSpi(*spi, source_path.value());
        if (!parsed) return parsed.error();
        config.spi = std::move(parsed).value();
    }

    return config;
}

Result<NetworkConfig> loadNetworkConfig(const std::filesystem::path& path) {
    auto source_path = absoluteNormalized(path);
    if (!source_path) return source_path.error();
    auto json = loadJson(source_path.value());
    if (!json) return json.error();
    auto object = requireObject(json.value(), source_path.value(), "network config root");
    if (!object) return object.error();
    auto unknown = rejectUnknown(*object.value(), source_path.value(), {"schema_version", "name", "buses", "boards"});
    if (!unknown) return unknown.error();
    auto schema = validateSchema(*object.value(), source_path.value());
    if (!schema) return schema.error();
    auto name = requireString(*object.value(), source_path.value(), "name");
    if (!name) return name.error();
    auto buses_field = requireField(*object.value(), source_path.value(), "buses");
    auto boards_field = requireField(*object.value(), source_path.value(), "boards");
    if (!buses_field) return buses_field.error();
    if (!boards_field) return boards_field.error();
    auto buses = requireObject(*buses_field.value(), source_path.value(), "key 'buses'");
    auto boards = requireArray(*boards_field.value(), source_path.value(), "key 'boards'");
    if (!buses) return buses.error();
    if (!boards) return boards.error();

    NetworkConfig config;
    config.name = std::move(name).value();
    config.source_path = source_path.value();
    for (const auto& [bus_name, definition] : *buses.value()) {
        auto bus = requireObject(definition, source_path.value(), "CAN bus '" + bus_name + "'");
        if (!bus) return bus.error();
        auto bus_unknown = rejectUnknown(*bus.value(), source_path.value(), {"type", "bitrate"});
        if (!bus_unknown) return bus_unknown.error();
        auto type = requireString(*bus.value(), source_path.value(), "type");
        if (!type) return type.error();
        if (type.value() != "can") return configError(source_path.value(), "only CAN network buses are supported");
        CanBusConfig bus_config;
        bus_config.name = bus_name;
        if (const JsonValue* bitrate = find(*bus.value(), "bitrate")) {
            auto parsed = numericValue(*bitrate, source_path.value(), bus_name + ".bitrate");
            if (!parsed) return parsed.error();
            if (parsed.value() == 0 || parsed.value() > std::numeric_limits<std::uint32_t>::max()) {
                return configError(source_path.value(), "CAN bus bitrate must be a nonzero 32-bit value");
            }
            bus_config.bitrate = static_cast<std::uint32_t>(parsed.value());
        }
        config.buses.push_back(std::move(bus_config));
    }
    if (boards.value()->empty()) {
        return configError(source_path.value(), "network must contain at least one board");
    }
    for (const JsonValue& board : *boards.value()) {
        auto board_path = stringValue(board, source_path.value(), "boards[]");
        if (!board_path) return board_path.error();
        config.board_paths.push_back(resolveRelative(source_path.value(), board_path.value()));
    }
    return config;
}

std::string normalize(const McuConfig& config) {
    std::ostringstream output;
    output << "schema_version: " << config.schema_version << '\n'
           << "type: mcu\n"
           << "name: " << config.name << '\n'
           << "flash: " << hex32(config.flash_base) << "+" << config.flash_size << '\n'
           << "sram: " << hex32(config.sram_base) << "+" << config.sram_size << '\n'
           << "ccm_sram: " << hex32(config.ccm_sram_base) << "+" << config.ccm_sram_size << '\n'
           << "hse_hz: " << config.hse_hz << '\n';
    return output.str();
}

std::string normalize(const BoardConfig& config) {
    std::ostringstream output;
    output << "schema_version: " << config.schema_version << '\n'
           << "type: board\n"
           << "name: " << config.name << '\n'
           << "source: " << config.source_path.string() << '\n'
           << "mcu: " << config.mcu_path.string() << '\n'
           << "elf: " << config.elf_path.string() << '\n';
    if (config.vector_base.has_value()) {
        output << "vector_base: " << hex32(*config.vector_base) << '\n';
    }
    output << "duration_ms: " << config.run.default_duration_ms << '\n'
           << "max_instructions: " << config.run.max_instructions << '\n'
           << "gpio_pins: " << config.gpio.size() << '\n'
           << "can_controllers: " << config.can.size() << '\n'
           << "usarts: " << config.usart.size() << '\n'
           << "adcs: " << config.adc.size() << '\n'
           << "spis: " << config.spi.size() << '\n';
    return output.str();
}

std::string normalize(const NetworkConfig& config) {
    std::ostringstream output;
    output << "schema_version: " << config.schema_version << '\n'
           << "type: network\n"
           << "name: " << config.name << '\n'
           << "source: " << config.source_path.string() << '\n'
           << "buses: " << config.buses.size() << '\n';
    for (const CanBusConfig& bus : config.buses) {
        output << "bus: " << bus.name << " bitrate=" << bus.bitrate << '\n';
    }
    output << "boards: " << config.board_paths.size() << '\n';
    for (const auto& board : config.board_paths) output << "board: " << board.string() << '\n';
    return output.str();
}

} // namespace fil::config
