#include "fil/cli/cli.hpp"

#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

/**
 * @brief Converts process arguments to string views and dispatches the CLI.
 * @param argc Number of process arguments.
 * @param argv Process argument array.
 * @return Stable CLI exit code.
 */
int main(const std::int32_t argc, const char* const argv[]) {
    std::vector<std::string_view> args;
    args.reserve(static_cast<std::size_t>(argc > 0 ? argc - 1 : 0));

    for (std::int32_t index = 1; index < argc; ++index) {
        args.emplace_back(argv[index]);
    }

    return static_cast<std::int32_t>(fil::cli::run(args, std::cout, std::cerr));
}
