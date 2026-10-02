#pragma once

#include "fil/common/result.hpp"

#include <cstdint>
#include <filesystem>
#include <initializer_list>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace fil::config::detail {

/** @brief Small JSON value tree shared by the versioned input loaders.
 *
 * The recursive member of an object is held by `std::unique_ptr` on purpose.
 * `JsonValue` is still incomplete while `Storage` is declared, and an object entry
 * type that embeds it by value (`std::pair<std::string, JsonValue>`) forces the
 * standard library to instantiate that pair's members with an incomplete argument,
 * which libstdc++ rejects outright (libc++ happens to defer it). `unique_ptr` is
 * explicitly allowed to be instantiated with an incomplete type, so the same header
 * compiles against every standard library.
 */
struct JsonValue {
    using Member = std::pair<std::string, std::unique_ptr<JsonValue>>;
    using Object = std::vector<Member>;
    using Array = std::vector<JsonValue>;
    using Storage = std::variant<std::nullptr_t, bool, std::uint64_t, std::string, Object, Array>;

    Storage storage;
};

[[nodiscard]] Error configError(const std::filesystem::path& path, std::string message);
[[nodiscard]] Result<JsonValue> loadJson(const std::filesystem::path& path);
[[nodiscard]] const JsonValue* find(const JsonValue::Object& object, std::string_view key);
[[nodiscard]] Result<const JsonValue::Object*> requireObject(
    const JsonValue& value,
    const std::filesystem::path& path,
    std::string_view description
);
[[nodiscard]] Result<const JsonValue*> requireField(
    const JsonValue::Object& object,
    const std::filesystem::path& path,
    std::string_view key
);
[[nodiscard]] Result<void> rejectUnknown(
    const JsonValue::Object& object,
    const std::filesystem::path& path,
    std::initializer_list<std::string_view> allowed
);
[[nodiscard]] Result<std::string> requireString(
    const JsonValue::Object& object,
    const std::filesystem::path& path,
    std::string_view key
);
[[nodiscard]] Result<std::string> stringValue(
    const JsonValue& value,
    const std::filesystem::path& path,
    std::string_view key
);
[[nodiscard]] Result<bool> booleanValue(
    const JsonValue& value,
    const std::filesystem::path& path,
    std::string_view key
);
[[nodiscard]] Result<const JsonValue::Array*> requireArray(
    const JsonValue& value,
    const std::filesystem::path& path,
    std::string_view description
);
[[nodiscard]] Result<std::uint64_t> numericValue(
    const JsonValue& value,
    const std::filesystem::path& path,
    std::string_view key
);
[[nodiscard]] Result<std::uint64_t> requireUnsigned(
    const JsonValue::Object& object,
    const std::filesystem::path& path,
    std::string_view key
);
[[nodiscard]] Result<std::uint32_t> requireUint32(
    const JsonValue::Object& object,
    const std::filesystem::path& path,
    std::string_view key
);
[[nodiscard]] Result<void> validateSchema(
    const JsonValue::Object& object,
    const std::filesystem::path& path
);
[[nodiscard]] Result<std::filesystem::path> absoluteNormalized(const std::filesystem::path& path);
[[nodiscard]] std::filesystem::path resolveRelative(
    const std::filesystem::path& config_path,
    const std::filesystem::path& referenced_path
);

} // namespace fil::config::detail
