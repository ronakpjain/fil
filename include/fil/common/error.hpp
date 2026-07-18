#pragma once

/** @file error.hpp
 *  @brief Typed error categories and source-aware diagnostics.
 */

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>

namespace fil {

/** @brief Broad failure category used for programmatic error handling. */
enum class ErrorCategory {
    invalid_argument, ///< A caller supplied an invalid value.
    io,               ///< A filesystem or stream operation failed.
    parse,            ///< Input bytes or text were structurally malformed.
    config,           ///< Parsed configuration violated its schema.
    unsupported,      ///< A valid feature is not implemented.
    runtime,          ///< Emulation or command execution failed.
    internal,         ///< An invariant inside fil was violated.
};

/** @brief Optional location identifying where an input-related error occurred. */
struct SourceContext {
    std::filesystem::path path; ///< Input file associated with the error.
    std::size_t line{0};        ///< One-based line, or zero when unavailable.
    std::size_t column{0};      ///< One-based column, or zero when unavailable.
};

/** @brief Structured failure propagated through Result without C++ exceptions. */
struct Error {
    ErrorCategory category{ErrorCategory::internal}; ///< Machine-readable failure category.
    std::string message;                             ///< Human-readable explanation.
    std::optional<SourceContext> source;             ///< Input location when known.
};

/**
 * @brief Formats an error with optional file, line, and column context.
 * @param error Error to format.
 * @return Human-readable single-line diagnostic.
 */
[[nodiscard]] std::string formatError(const Error& error);

} // namespace fil
