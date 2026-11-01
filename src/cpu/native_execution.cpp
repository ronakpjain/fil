#include "fil/cpu/native_execution.hpp"

namespace fil::cpu {
namespace {

constexpr std::uint32_t mmioWidth(const mem::AccessSize size) noexcept {
    switch (size) {
    case mem::AccessSize::byte: return 1U;
    case mem::AccessSize::halfword: return 2U;
    default: return 4U;
    }
}

// Committed-span MMIO fallback. Only reachable when the host installed a
// bridge; speculative/certificate spans keep the strict reversible-RAM ABI.
bool mmioRead(NativeExecutionContext* const context, const std::uint32_t instruction_pc,
    const std::uint32_t address, const mem::AccessSize size,
    std::uint32_t* const value_out) {
    if (context->mmio_bridge == nullptr || context->mmio_bridge->begin == nullptr
        || !context->memory_bus->isMmioRange(address, mmioWidth(size))) {
        return false;
    }
    // The host advances/validates the exact guest time before the access.
    if (!context->mmio_bridge->begin(
            context->mmio_bridge->context, context->retired_cycles, instruction_pc)) {
        return false;
    }
    const auto result = context->memory_bus->readMmioChecked(
        address, size, {mem::AccessType::data_read, instruction_pc});
    if (!result) return false;
    *value_out = static_cast<std::uint32_t>(result.value());
    if (context->mmio_bridge->must_stop != nullptr
        && context->mmio_bridge->must_stop(context->mmio_bridge->context)) {
        context->boundary_stop = true;
    }
    return true;
}

bool mmioWrite(NativeExecutionContext* const context, const std::uint32_t instruction_pc,
    const std::uint32_t address, const mem::AccessSize size, const std::uint32_t value) {
    if (context->mmio_bridge == nullptr || context->mmio_bridge->begin == nullptr
        || !context->memory_bus->isMmioRange(address, mmioWidth(size))) {
        return false;
    }
    if (!context->mmio_bridge->begin(
            context->mmio_bridge->context, context->retired_cycles, instruction_pc)) {
        return false;
    }
    const auto result = context->memory_bus->writeMmioChecked(
        address, size, value, {mem::AccessType::data_write, instruction_pc});
    if (!result) return false;
    if (context->mmio_bridge->must_stop != nullptr
        && context->mmio_bridge->must_stop(context->mmio_bridge->context)) {
        context->boundary_stop = true;
    }
    return true;
}

} // namespace

bool nativeAdmittedCyclePrefixValid(const NativeExecutionContext* const context) noexcept {
    if (context == nullptr || context->admitted_cycle_prefix == nullptr
        || context->charged_cycles == nullptr
        || context->instruction_limit > context->charged_cycles_count
        || context->instruction_limit >= context->admitted_cycle_prefix_count
        || context->retired_cycles != 0U) return false;
    const auto* const prefix = context->admitted_cycle_prefix;
    return prefix[0] == 0U && prefix[context->instruction_limit] <= context->cycle_limit;
}

bool nativeRead32(
    NativeExecutionContext* const context, const std::uint32_t instruction_pc,
    const std::uint32_t address, std::uint32_t* const value_out
) noexcept {
    if (context == nullptr || context->memory_bus == nullptr || value_out == nullptr) {
        return false;
    }
    // tryFastRead32 publishes its output only after every guard succeeds.
    // Forward the destination so the JIT helper needs no temporary stack slot
    // (and can tail-call the checked bus access).
    if (context->memory_bus->tryFastRead32(
            address, *value_out, {mem::AccessType::data_read, instruction_pc})) {
        return true;
    }
    return mmioRead(context, instruction_pc, address, mem::AccessSize::word, value_out);
}

bool nativeWrite32(
    NativeExecutionContext* const context, const std::uint32_t instruction_pc,
    const std::uint32_t address, const std::uint32_t value
) noexcept {
    if (context == nullptr || context->memory_bus == nullptr) return false;
    if (context->memory_bus->tryFastWrite32(
            address, value, {mem::AccessType::data_write, instruction_pc}, true)) {
        return true;
    }
    return mmioWrite(context, instruction_pc, address, mem::AccessSize::word, value);
}

bool nativeRead8(
    NativeExecutionContext* const context, const std::uint32_t instruction_pc,
    const std::uint32_t address, std::uint32_t* const value_out
) noexcept {
    if (context == nullptr || context->memory_bus == nullptr || value_out == nullptr) {
        return false;
    }
    std::uint64_t value = 0U;
    if (context->memory_bus->tryFastRead(
            address, mem::AccessSize::byte, value,
            {mem::AccessType::data_read, instruction_pc})) {
        *value_out = static_cast<std::uint32_t>(value & 0xffU);
        return true;
    }
    return mmioRead(context, instruction_pc, address, mem::AccessSize::byte, value_out);
}

bool nativeWrite8(
    NativeExecutionContext* const context, const std::uint32_t instruction_pc,
    const std::uint32_t address, const std::uint32_t value
) noexcept {
    if (context == nullptr || context->memory_bus == nullptr) return false;
    if (context->memory_bus->tryFastWrite(
            address, mem::AccessSize::byte, value & 0xffU,
            {mem::AccessType::data_write, instruction_pc}, true)) {
        return true;
    }
    return mmioWrite(context, instruction_pc, address, mem::AccessSize::byte, value);
}

bool nativeRead16(
    NativeExecutionContext* const context, const std::uint32_t instruction_pc,
    const std::uint32_t address, std::uint32_t* const value_out
) noexcept {
    if (context == nullptr || context->memory_bus == nullptr || value_out == nullptr) {
        return false;
    }
    std::uint64_t value = 0U;
    if (context->memory_bus->tryFastRead(
            address, mem::AccessSize::halfword, value,
            {mem::AccessType::data_read, instruction_pc})) {
        *value_out = static_cast<std::uint32_t>(value & 0xffffU);
        return true;
    }
    return mmioRead(context, instruction_pc, address, mem::AccessSize::halfword, value_out);
}

bool nativeWrite16(
    NativeExecutionContext* const context, const std::uint32_t instruction_pc,
    const std::uint32_t address, const std::uint32_t value
) noexcept {
    if (context == nullptr || context->memory_bus == nullptr) return false;
    if (context->memory_bus->tryFastWrite(
            address, mem::AccessSize::halfword, value & 0xffffU,
            {mem::AccessType::data_write, instruction_pc}, true)) {
        return true;
    }
    return mmioWrite(context, instruction_pc, address, mem::AccessSize::halfword, value);
}

} // namespace fil::cpu
