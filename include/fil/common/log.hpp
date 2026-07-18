#pragma once

/** @file log.hpp
 *  @brief Minimal stream-based logging with severity filtering.
 */

#include <iosfwd>
#include <string_view>

namespace fil {

/** @brief Severity assigned to a log record. */
enum class LogLevel {
    debug,   ///< Detailed development diagnostics.
    info,    ///< Normal lifecycle information.
    warning, ///< Recoverable or suspicious behavior.
    error,   ///< Operation-ending failure.
};

/**
 * @brief Lightweight synchronous logger writing stable line-oriented records.
 *
 * Logger does not own its output stream and performs no global registration,
 * buffering, timestamps, or thread synchronization. This keeps host output
 * deterministic and lets callers choose the destination.
 */
class Logger {
public:
    /**
     * @brief Creates a logger that writes messages at or above a minimum level.
     * @param output Destination stream, which must outlive the logger.
     * @param minimum_level Lowest severity that will be emitted.
     */
    explicit Logger(std::ostream& output, LogLevel minimum_level = LogLevel::info);

    /**
     * @brief Emits a message if its level meets the configured threshold.
     * @param level Message severity.
     * @param message Message text without a trailing newline.
     */
    void log(LogLevel level, std::string_view message);

private:
    std::ostream* output_;
    LogLevel minimum_level_;
};

/**
 * @brief Gets the stable lowercase name of a log level.
 * @param level Log level to describe.
 * @return Static level name.
 */
[[nodiscard]] std::string_view logLevelName(LogLevel level) noexcept;

} // namespace fil
