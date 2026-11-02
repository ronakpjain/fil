#include "fil/stm32g4/peripheral.hpp"

#include "fil/common/format.hpp"
#include "fil/common/numeric.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace fil::stm32g4 {
namespace {

std::string qualifiedTraceSource(
    const std::string_view prefix,
    const std::string_view device_name
) {
    if (prefix.empty()) return std::string(device_name);
    std::string source;
    source.reserve(prefix.size() + 1U + device_name.size());
    source.append(prefix);
    source.push_back('.');
    source.append(device_name);
    return source;
}

} // namespace

RegisterPeripheral::RegisterPeripheral(
    std::string name,
    const std::uint32_t register_block_size,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : name_(std::move(name)),
    trace_source_(name_),
    block_size_(register_block_size),
    registers_((static_cast<std::size_t>(register_block_size) + 3U) / 4U),
    reset_values_(registers_.size()),
    event_loop_(event_loop),
    trace_(trace) {
    if (block_size_ == 0) {
        throw std::invalid_argument("peripheral register block must not be empty");
    }
}

RegisterPeripheral::~RegisterPeripheral() = default;

void RegisterPeripheral::setTraceSourcePrefix(const std::string_view prefix) {
    trace_source_ = qualifiedTraceSource(prefix, name_);
}

void RegisterPeripheral::setInterruptLevelCallback(InterruptLevelCallback callback) {
    interrupt_level_callback_device_local_ = false;
    interrupt_level_callback_ = std::move(callback);
    ++interrupt_level_callback_generation_;
    if (interrupt_level_callback_) {
        for (std::uint32_t line = 0; line < interrupt_levels_.size(); ++line) {
            interrupt_level_callback_(line, interrupt_levels_[line]);
        }
    }
}

void RegisterPeripheral::setDeviceLocalInterruptLevelCallback(InterruptLevelCallback callback) {
    const auto generation = interrupt_level_callback_generation_ + 1U;
    setInterruptLevelCallback(std::move(callback));
    // Initial level publication may reenter and replace the callback.
    if (interrupt_level_callback_generation_ == generation)
        interrupt_level_callback_device_local_ = true;
}

mem::BusFault RegisterPeripheral::accessFault(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const mem::AccessContext& context,
    std::string message
) const {
    return mem::BusFault{
        mem::BusFaultReason::device_error,
        offset,
        size,
        context,
        name_,
        std::move(message),
    };
}

mem::MemoryResult<std::uint64_t> RegisterPeripheral::read(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const mem::AccessContext& context
) {
    const std::uint32_t width = mem::byteCount(size);
    if (!mem::validAccessSize(size) || !rangeFits(offset, width, block_size_)) {
        return accessFault(offset, size, context, "peripheral register read is outside the register block");
    }

    if (width == 4U && (offset & 3U) == 0U) return loadRegister(offset, context);
    std::uint64_t value = 0U;
    for (std::uint32_t consumed = 0U; consumed < width;) {
        const auto address = offset + consumed;
        const auto shift = (address & 3U) * 8U;
        const auto count = std::min(width - consumed, 4U - (address & 3U));
        const auto mask = (std::uint64_t{1U} << (count * 8U)) - 1U;
        // Walk contiguous words once, in architectural order. Read-to-clear
        // registers are never reloaded for subsequent byte lanes.
        const auto word = loadRegister(address & ~std::uint32_t{3U}, context);
        value |= ((static_cast<std::uint64_t>(word) >> shift) & mask) << (consumed * 8U);
        consumed += count;
    }
    return value;
}

mem::MemoryResult<std::uint64_t> RegisterPeripheral::write(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const std::uint64_t value,
    const mem::AccessContext& context
) {
    const std::uint32_t width = mem::byteCount(size);
    if (!mem::validAccessSize(size) || !rangeFits(offset, width, block_size_)) {
        return accessFault(offset, size, context, "peripheral register write is outside the register block");
    }

    const auto store = [&](const std::uint32_t word_offset, const std::uint32_t bits,
        const std::uint32_t mask) {
        const auto previous = registers_[word_offset / 4U];
        const auto merged = (previous & ~mask) | (bits & mask);
        registers_[word_offset / 4U] = merged;
        storeRegister(word_offset, previous, merged, mask, context);
    };
    if (width == 4U && (offset & 3U) == 0U) {
        store(offset, static_cast<std::uint32_t>(value), UINT32_MAX);
        return std::uint64_t{0U};
    }
    for (std::uint32_t consumed = 0U; consumed < width;) {
        const auto address = offset + consumed;
        const auto shift = (address & 3U) * 8U;
        const auto count = std::min(width - consumed, 4U - (address & 3U));
        const auto low_mask = (std::uint64_t{1U} << (count * 8U)) - 1U;
        const auto bits = static_cast<std::uint32_t>((value >> (consumed * 8U)) & low_mask) << shift;
        store(address & ~std::uint32_t{3U}, bits, static_cast<std::uint32_t>(low_mask) << shift);
        consumed += count;
    }
    return std::uint64_t{0U};
}

void RegisterPeripheral::reset() {
    // The authoritative register bank keeps its address across resets.
    std::copy(reset_values_.begin(), reset_values_.end(), registers_.begin());
    for (std::uint32_t line = 0; line < interrupt_levels_.size(); ++line) {
        setInterruptLevel(line, false);
    }
    onReset();
}

std::uint32_t RegisterPeripheral::peekRegister(const std::uint32_t word_offset) const noexcept {
    return registerValue(word_offset);
}

const std::uint32_t* RegisterPeripheral::registerStorage(const std::uint32_t word_offset) const noexcept {
    if ((word_offset & 3U) != 0U || word_offset >= block_size_) return nullptr;
    return registers_.data() + word_offset / 4U;
}

std::uint32_t RegisterPeripheral::loadRegister(
    const std::uint32_t word_offset,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    return registerValue(word_offset);
}

void RegisterPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(word_offset);
    static_cast<void>(previous);
    static_cast<void>(value);
    static_cast<void>(write_mask);
    static_cast<void>(context);
}

void RegisterPeripheral::onReset() {}

void RegisterPeripheral::setResetValue(
    const std::uint32_t word_offset,
    const std::uint32_t value
) noexcept {
    if ((word_offset & 3U) == 0U && word_offset < block_size_) {
        reset_values_[word_offset / 4U] = value;
    }
}

void RegisterPeripheral::setRegister(
    const std::uint32_t word_offset,
    const std::uint32_t value
) noexcept {
    if ((word_offset & 3U) == 0U && word_offset < block_size_) {
        registers_[word_offset / 4U] = value;
    }
}

std::uint32_t RegisterPeripheral::registerValue(const std::uint32_t word_offset) const noexcept {
    if ((word_offset & 3U) != 0U || word_offset >= block_size_) {
        return 0;
    }
    return registers_[word_offset / 4U];
}

sim::SimTimeNs RegisterPeripheral::currentTime() const noexcept {
    return event_loop_ == nullptr ? 0 : event_loop_->now();
}

void RegisterPeripheral::setInterruptLevel(const std::uint32_t line, const bool asserted) {
    if (line >= interrupt_levels_.size() || interrupt_levels_[line] == asserted) return;
    interrupt_levels_[line] = asserted;
    if (interrupt_level_callback_) interrupt_level_callback_(line, asserted);
}

void RegisterPeripheral::traceEvent(
    std::string type,
    std::vector<sim::TraceField> fields
) {
    if (trace_ != nullptr) {
        static_cast<void>(trace_->record(
            currentTime(), trace_source_, std::move(type), std::move(fields)
        ));
    }
}

UnknownMmioDevice::UnknownMmioDevice(
    std::string name,
    const std::uint32_t absolute_base,
    const bool strict,
    const std::uint64_t default_read_value,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : name_(std::move(name)),
    trace_source_(name_),
    absolute_base_(absolute_base),
    strict_(strict),
    default_read_value_(default_read_value),
    event_loop_(event_loop),
    trace_(trace) {}

void UnknownMmioDevice::setTraceSourcePrefix(const std::string_view prefix) {
    trace_source_ = qualifiedTraceSource(prefix, name_);
}

mem::BusFault UnknownMmioDevice::fault(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const mem::AccessContext& context,
    std::string message
) const {
    return mem::BusFault{
        mem::BusFaultReason::device_error,
        absolute_base_ + offset,
        size,
        context,
        name_,
        std::move(message),
    };
}

sim::SimTimeNs UnknownMmioDevice::currentTime() const noexcept {
    return event_loop_ == nullptr ? 0 : event_loop_->now();
}

void UnknownMmioDevice::traceAccess(const PeripheralAccess& access) {
    if (trace_ == nullptr) {
        return;
    }
    static_cast<void>(trace_->record(
        access.time_ns,
        trace_source_,
        access.write ? "unknown_mmio_write" : "unknown_mmio_read",
        {
            {"offset", hexValue(access.offset)},
            {"size", std::to_string(mem::byteCount(access.size))},
            {"value", hexValue(access.value)},
            {"pc", hexValue(access.pc)},
        }
    ));
}

mem::MemoryResult<std::uint64_t> UnknownMmioDevice::read(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const mem::AccessContext& context
) {
    if (!mem::validAccessSize(size)) {
        return fault(offset, size, context, "unknown MMIO read has an invalid width");
    }
    const std::uint32_t width = mem::byteCount(size);
    if (offset > std::numeric_limits<std::uint32_t>::max() - (width - 1U)) {
        return fault(offset, size, context, "unknown MMIO read wraps the address space");
    }

    std::uint64_t value = 0;
    for (std::uint32_t index = 0; index < width; ++index) {
        const auto stored = bytes_.find(offset + index);
        const std::uint8_t byte = stored == bytes_.end()
            ? static_cast<std::uint8_t>((default_read_value_ >> (index * 8U)) & 0xffU)
            : stored->second;
        value |= static_cast<std::uint64_t>(byte) << (index * 8U);
    }
    const PeripheralAccess access{currentTime(), false, offset, size, value, context.pc};
    accesses_.push_back(access);
    traceAccess(access);
    if (strict_) {
        return fault(offset, size, context, "read from unknown MMIO register");
    }
    return value;
}

mem::MemoryResult<std::uint64_t> UnknownMmioDevice::write(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const std::uint64_t value,
    const mem::AccessContext& context
) {
    if (!mem::validAccessSize(size)) {
        return fault(offset, size, context, "unknown MMIO write has an invalid width");
    }
    const std::uint32_t width = mem::byteCount(size);
    if (offset > std::numeric_limits<std::uint32_t>::max() - (width - 1U)) {
        return fault(offset, size, context, "unknown MMIO write wraps the address space");
    }

    const PeripheralAccess access{
        currentTime(), true, offset, size,
        value & mem::accessWidthMask(size), context.pc,
    };
    accesses_.push_back(access);
    traceAccess(access);
    if (strict_) {
        return fault(offset, size, context, "write to unknown MMIO register");
    }
    for (std::uint32_t index = 0; index < width; ++index) {
        bytes_[offset + index] = static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU);
    }
    return std::uint64_t{0};
}

void UnknownMmioDevice::clear() {
    bytes_.clear();
    accesses_.clear();
}

} // namespace fil::stm32g4
