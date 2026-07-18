#include "fil/mem/address.hpp"

#include <iomanip>
#include <sstream>

namespace fil::mem {

std::string_view accessTypeName(const AccessType type) noexcept {
    switch (type) {
    case AccessType::instruction_fetch:
        return "instruction fetch";
    case AccessType::data_read:
        return "data read";
    case AccessType::data_write:
        return "data write";
    case AccessType::debug:
        return "debug access";
    }
    return "unknown access";
}

std::string formatBusFault(const BusFault& fault) {
    std::ostringstream output;
    output << fault.message
           << " (" << accessTypeName(fault.context.type)
           << ", address=0x" << std::hex << std::setfill('0') << std::setw(8) << fault.address
           << ", size=" << std::dec << byteCount(fault.size);
    if (fault.context.pc != 0) {
        output << ", pc=0x" << std::hex << std::setfill('0') << std::setw(8) << fault.context.pc;
    }
    if (!fault.region.empty()) {
        output << ", region=" << fault.region;
    }
    output << ')';
    return output.str();
}

} // namespace fil::mem
