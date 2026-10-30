#include "fil/cpu/native_execution.hpp"

namespace fil::cpu {

bool nativeRead32(
    NativeExecutionContext* const context, const std::uint32_t instruction_pc,
    const std::uint32_t address, std::uint32_t* const value_out
) noexcept {
    if (context == nullptr || context->memory_bus == nullptr || value_out == nullptr) {
        return false;
    }
    std::uint32_t value = 0;
    if (!context->memory_bus->tryFastRead32(
            address, value, {mem::AccessType::data_read, instruction_pc})) {
        return false;
    }
    *value_out = value;
    return true;
}

bool nativeWrite32(
    NativeExecutionContext* const context, const std::uint32_t instruction_pc,
    const std::uint32_t address, const std::uint32_t value
) noexcept {
    if (context == nullptr || context->memory_bus == nullptr
        || !context->memory_bus->isReversibleRamTarget(address)) {
        return false;
    }
    return context->memory_bus->tryFastWrite32(
        address, value, {mem::AccessType::data_write, instruction_pc});
}

bool nativeRead8(
    NativeExecutionContext* const context, const std::uint32_t instruction_pc,
    const std::uint32_t address, std::uint32_t* const value_out
) noexcept {
    if (context == nullptr || context->memory_bus == nullptr || value_out == nullptr) {
        return false;
    }
    std::uint64_t value = 0U;
    if (!context->memory_bus->tryFastRead(
            address, mem::AccessSize::byte, value,
            {mem::AccessType::data_read, instruction_pc})) {
        return false;
    }
    *value_out = static_cast<std::uint32_t>(value & 0xffU);
    return true;
}

bool nativeWrite8(
    NativeExecutionContext* const context, const std::uint32_t instruction_pc,
    const std::uint32_t address, const std::uint32_t value
) noexcept {
    if (context == nullptr || context->memory_bus == nullptr
        || !context->memory_bus->isReversibleRamTarget(address)) {
        return false;
    }
    return context->memory_bus->tryFastWrite(
        address, mem::AccessSize::byte, value & 0xffU,
        {mem::AccessType::data_write, instruction_pc});
}

} // namespace fil::cpu
