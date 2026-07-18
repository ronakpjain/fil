#include "fil/common/log.hpp"

#include <ostream>

namespace fil {

Logger::Logger(std::ostream& output, const LogLevel minimum_level)
    : output_(&output), minimum_level_(minimum_level) {}

void Logger::log(const LogLevel level, const std::string_view message) {
    if (level < minimum_level_) {
        return;
    }
    *output_ << '[' << logLevelName(level) << "] " << message << '\n';
}

std::string_view logLevelName(const LogLevel level) noexcept {
    switch (level) {
    case LogLevel::debug:
        return "debug";
    case LogLevel::info:
        return "info";
    case LogLevel::warning:
        return "warning";
    case LogLevel::error:
        return "error";
    }
    return "unknown";
}

} // namespace fil
