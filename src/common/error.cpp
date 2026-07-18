#include "fil/common/error.hpp"

#include <sstream>

namespace fil {

std::string formatError(const Error& error) {
    std::ostringstream output;
    if (error.source.has_value()) {
        output << error.source->path.string();
        if (error.source->line != 0) {
            output << ':' << error.source->line;
            if (error.source->column != 0) {
                output << ':' << error.source->column;
            }
        }
        output << ": ";
    }
    output << error.message;
    return output.str();
}

} // namespace fil
