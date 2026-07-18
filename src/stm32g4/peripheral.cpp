#include "fil/stm32g4/peripheral.hpp"

#include <array>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace fil::stm32g4 {
namespace {

bool validAccessSize(const mem::AccessSize size) noexcept {
    const std::uint32_t width = mem::byteCount(size);
    return width == 1U || width == 2U || width == 4U || width == 8U;
}

std::uint64_t widthMask(const mem::AccessSize size) noexcept {
    if (size == mem::AccessSize::doubleword) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return (std::uint64_t{1} << (mem::byteCount(size) * 8U)) - 1U;
}

std::string hexadecimal(const std::uint64_t value) {
    std::ostringstream output;
    output << "0x" << std::hex << value;
    return output.str();
}

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
    registers_((static_cast<std::size_t>(register_block_size) + 3U) / 4U, 0),
    reset_values_(registers_.size(), 0),
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
    if (!validAccessSize(size) || offset > block_size_ || width > block_size_ - offset) {
        return accessFault(offset, size, context, "peripheral register read is outside the register block");
    }

    std::array<std::uint32_t, 3> cached_offsets{};
    std::array<std::uint32_t, 3> cached_values{};
    std::size_t cached_count = 0;
    std::uint64_t value = 0;

    for (std::uint32_t byte_index = 0; byte_index < width; ++byte_index) {
        const std::uint32_t byte_offset = offset + byte_index;
        const std::uint32_t word_offset = byte_offset & ~std::uint32_t{3};
        std::size_t cache_index = 0;
        while (cache_index < cached_count && cached_offsets[cache_index] != word_offset) {
            ++cache_index;
        }
        if (cache_index == cached_count) {
            cached_offsets[cached_count] = word_offset;
            cached_values[cached_count] = loadRegister(word_offset, context);
            ++cached_count;
        }
        const std::uint32_t shift = (byte_offset & 3U) * 8U;
        const std::uint64_t byte = (cached_values[cache_index] >> shift) & 0xffU;
        value |= byte << (byte_index * 8U);
    }
    return value & widthMask(size);
}

mem::MemoryResult<std::uint64_t> RegisterPeripheral::write(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const std::uint64_t value,
    const mem::AccessContext& context
) {
    const std::uint32_t width = mem::byteCount(size);
    if (!validAccessSize(size) || offset > block_size_ || width > block_size_ - offset) {
        return accessFault(offset, size, context, "peripheral register write is outside the register block");
    }

    std::array<std::uint32_t, 3> touched_offsets{};
    std::array<std::uint32_t, 3> masks{};
    std::array<std::uint32_t, 3> bits{};
    std::size_t touched_count = 0;

    for (std::uint32_t byte_index = 0; byte_index < width; ++byte_index) {
        const std::uint32_t byte_offset = offset + byte_index;
        const std::uint32_t word_offset = byte_offset & ~std::uint32_t{3};
        std::size_t touched_index = 0;
        while (touched_index < touched_count && touched_offsets[touched_index] != word_offset) {
            ++touched_index;
        }
        if (touched_index == touched_count) {
            touched_offsets[touched_count] = word_offset;
            ++touched_count;
        }
        const std::uint32_t register_shift = (byte_offset & 3U) * 8U;
        const std::uint32_t source_shift = byte_index * 8U;
        masks[touched_index] |= 0xffU << register_shift;
        bits[touched_index] |= static_cast<std::uint32_t>((value >> source_shift) & 0xffU)
            << register_shift;
    }

    for (std::size_t index = 0; index < touched_count; ++index) {
        const std::uint32_t word_offset = touched_offsets[index];
        const std::size_t register_index = word_offset / 4U;
        const std::uint32_t previous = registers_[register_index];
        const std::uint32_t merged = (previous & ~masks[index]) | bits[index];
        journalRegister(register_index);
        registers_[register_index] = merged;
        storeRegister(word_offset, previous, merged, masks[index], context);
    }
    return std::uint64_t{0};
}

void RegisterPeripheral::beginTransaction() {
    transaction_journal_.clear();
    transaction_journal_.reserve(registers_.size());
    transaction_active_ = true;
}

void RegisterPeripheral::commitTransaction() noexcept {
    transaction_journal_.clear();
    transaction_active_ = false;
}

void RegisterPeripheral::rollbackTransaction() noexcept {
    for (auto mutation = transaction_journal_.rbegin();
         mutation != transaction_journal_.rend(); ++mutation) {
        registers_[mutation->index] = mutation->previous;
    }
    transaction_journal_.clear();
    transaction_active_ = false;
}

void RegisterPeripheral::reset() {
    commitTransaction();
    registers_ = reset_values_;
    onReset();
}

std::uint32_t RegisterPeripheral::peekRegister(const std::uint32_t word_offset) const noexcept {
    return registerValue(word_offset);
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
        const std::size_t index = word_offset / 4U;
        journalRegister(index);
        registers_[index] = value;
    }
}

void RegisterPeripheral::journalRegister(const std::size_t index) noexcept {
    if (!transaction_active_) return;
    for (const RegisterMutation& mutation : transaction_journal_) {
        if (mutation.index == index) return;
    }
    transaction_journal_.push_back(RegisterMutation{index, registers_[index]});
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
            {"offset", hexadecimal(access.offset)},
            {"size", std::to_string(mem::byteCount(access.size))},
            {"value", hexadecimal(access.value)},
            {"pc", hexadecimal(access.pc)},
        }
    ));
}

mem::MemoryResult<std::uint64_t> UnknownMmioDevice::read(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const mem::AccessContext& context
) {
    if (!validAccessSize(size)) {
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
    if (access_history_enabled_) accesses_.push_back(access);
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
    if (!validAccessSize(size)) {
        return fault(offset, size, context, "unknown MMIO write has an invalid width");
    }
    const std::uint32_t width = mem::byteCount(size);
    if (offset > std::numeric_limits<std::uint32_t>::max() - (width - 1U)) {
        return fault(offset, size, context, "unknown MMIO write wraps the address space");
    }

    const PeripheralAccess access{currentTime(), true, offset, size, value & widthMask(size), context.pc};
    if (access_history_enabled_) accesses_.push_back(access);
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
