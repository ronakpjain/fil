#pragma once

/** @file cli.hpp
 *  @brief Testable command-line dispatch and stable process exit codes.
 */

#include <iosfwd>
#include <span>
#include <string_view>

namespace fil::cli {

/** @brief Stable process status codes returned by fil commands. */
enum class ExitCode : int {
    success = 0,        ///< Command completed successfully.
    usage_error = 2,    ///< Command name or arguments were invalid.
    config_error = 3,   ///< Configuration parsing or validation failed.
    runtime_error = 4,  ///< Input loading or command execution failed.
    internal_error = 70, ///< An internal invariant failed (`EX_SOFTWARE` style).
};

/**
 * @brief Executes the command-line interface with caller-provided streams.
 * @param args Arguments excluding the executable name.
 * @param out Stream for normal command output.
 * @param err Stream for diagnostics.
 * @return Stable process exit code for the command result.
 */
[[nodiscard]] ExitCode run(
    std::span<const std::string_view> args,
    std::ostream& out,
    std::ostream& err
);

/**
 * @brief Writes command usage and command summaries.
 * @param out Destination stream.
 */
void printHelp(std::ostream& out);

} // namespace fil::cli
