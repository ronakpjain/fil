#include "fil/stm32g4/peripheral.hpp"

#include <algorithm>
#include <utility>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t dma_isr = 0x00;
constexpr std::uint32_t dma_ifcr = 0x04;
constexpr std::uint32_t first_channel = 0x08;
constexpr std::uint32_t channel_stride = 0x14;

unsigned int normalizedDmaChannels(const unsigned int count) noexcept {
    return std::clamp(count, 1U, 8U);
}

unsigned int normalizedMuxChannels(const unsigned int count) noexcept {
    return std::clamp(count, 1U, 16U);
}

std::uint32_t transferWidth(const std::uint32_t selector) noexcept {
    return selector <= 2U ? (1U << selector) : 4U;
}

} // namespace

DmaPeripheral::DmaPeripheral(
    std::string name,
    const unsigned int channel_count,
    mem::MemoryBus* const memory,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral(
        std::move(name),
        first_channel + normalizedDmaChannels(channel_count) * channel_stride,
        event_loop,
        trace
    ),
    channel_count_(normalizedDmaChannels(channel_count)),
    memory_(memory) {
    reset();
}

void DmaPeripheral::setInterruptCallback(InterruptCallback callback) {
    interrupt_callback_ = std::move(callback);
}

bool DmaPeripheral::request(const unsigned int channel) {
    if (channel == 0U || channel > channel_count_) {
        return false;
    }
    const std::uint32_t base = first_channel + (channel - 1U) * channel_stride;
    if ((registerValue(base) & 1U) == 0U
        || (registerValue(base + 0x04U) & 0xffffU) == 0U) {
        return false;
    }
    runChannelRequest(channel);
    return true;
}

void DmaPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(write_mask);
    static_cast<void>(context);
    if (word_offset == dma_isr) {
        setRegister(dma_isr, previous);
        return;
    }
    if (word_offset == dma_ifcr) {
        setRegister(dma_isr, registerValue(dma_isr) & ~value);
        setRegister(dma_ifcr, 0);
        return;
    }

    unsigned int channel = 0;
    if (channelForOffset(word_offset, channel)
        && ((previous & 1U) == 0U)
        && ((value & 1U) != 0U)) {
        initializeChannel(channel);
        if ((value & (1U << 14U)) != 0U) {
            const std::uint32_t count = reload_counts_[channel - 1U];
            for (std::uint32_t item = 0; item < count; ++item) {
                if (!request(channel)) break;
            }
        }
    }
}

bool DmaPeripheral::channelForOffset(
    const std::uint32_t offset,
    unsigned int& channel
) const noexcept {
    if (offset < first_channel) {
        return false;
    }
    const std::uint32_t relative = offset - first_channel;
    if ((relative % channel_stride) != 0U) {
        return false;
    }
    channel = relative / channel_stride + 1U;
    return channel <= channel_count_;
}

void DmaPeripheral::initializeChannel(const unsigned int channel) {
    const std::uint32_t base = first_channel + (channel - 1U) * channel_stride;
    reload_counts_[channel - 1U] = registerValue(base + 0x04U) & 0xffffU;
}

void DmaPeripheral::runChannelRequest(const unsigned int channel) {
    const std::uint32_t base = first_channel + (channel - 1U) * channel_stride;
    const std::uint32_t control = registerValue(base);
    const std::uint32_t remaining = registerValue(base + 0x04U) & 0xffffU;
    const std::uint32_t reload_count = reload_counts_[channel - 1U];
    if (remaining == 0U || reload_count == 0U) return;

    const std::uint32_t transferred = reload_count - remaining;
    std::uint32_t peripheral_address = registerValue(base + 0x08U);
    std::uint32_t memory_address = registerValue(base + 0x0cU);
    const bool memory_to_peripheral = (control & (1U << 4U)) != 0U;
    const bool peripheral_increment = (control & (1U << 6U)) != 0U;
    const bool memory_increment = (control & (1U << 7U)) != 0U;
    const std::uint32_t peripheral_width = transferWidth((control >> 8U) & 0x3U);
    const std::uint32_t memory_width = transferWidth((control >> 10U) & 0x3U);
    if (peripheral_increment) peripheral_address += transferred * peripheral_width;
    if (memory_increment) memory_address += transferred * memory_width;

    const std::uint32_t source_address = memory_to_peripheral ? memory_address : peripheral_address;
    const std::uint32_t destination_address = memory_to_peripheral ? peripheral_address : memory_address;
    const std::uint32_t source_width = memory_to_peripheral ? memory_width : peripheral_width;
    const std::uint32_t destination_width = memory_to_peripheral ? peripheral_width : memory_width;
    auto read = readMemory(source_address, source_width);
    const bool success = read && writeMemory(destination_address, destination_width, read.value());
    const std::uint32_t next_remaining = success ? remaining - 1U : remaining;
    setRegister(base + 0x04U, next_remaining);

    const bool complete = success && next_remaining == 0U;
    if (complete) {
        setChannelFlag(channel, 1U); // TCIF
        if ((control & (1U << 5U)) != 0U) {
            setRegister(base + 0x04U, reload_count);
        } else {
            setRegister(base, registerValue(base) & ~1U);
        }
    } else if (!success) {
        setChannelFlag(channel, 3U); // TEIF
        setRegister(base, registerValue(base) & ~1U);
    }

    if (transfer_history_enabled_) {
        transfers_.push_back(DmaTransfer{
            currentTime(), channel, 1U, memory_to_peripheral, success,
        });
    }
    if (traceEnabled()) {
        traceEvent("dma_transfer", {
            {"channel", std::to_string(channel)},
            {"items", "1"},
            {"direction", memory_to_peripheral ? "memory_to_peripheral" : "peripheral_to_memory"},
            {"success", success ? "true" : "false"},
        });
    }

    const bool interrupt_enabled = complete
        ? ((control & (1U << 1U)) != 0U)
        : (!success && ((control & (1U << 3U)) != 0U));
    if (interrupt_enabled && interrupt_callback_) interrupt_callback_(channel);
}

mem::MemoryResult<std::uint64_t> DmaPeripheral::readMemory(
    const std::uint32_t address,
    const std::uint32_t width
) {
    if (memory_ == nullptr) {
        return mem::BusFault{
            mem::BusFaultReason::device_error,
            address,
            mem::AccessSize::byte,
            {mem::AccessType::data_read, 0},
            std::string{name()},
            "DMA has no attached memory bus",
        };
    }
    if (width == 1U) {
        auto result = memory_->read8(address);
        return result ? mem::MemoryResult<std::uint64_t>{result.value()}
                      : mem::MemoryResult<std::uint64_t>{result.fault()};
    }
    if (width == 2U) {
        auto result = memory_->read16(address);
        return result ? mem::MemoryResult<std::uint64_t>{result.value()}
                      : mem::MemoryResult<std::uint64_t>{result.fault()};
    }
    auto result = memory_->read32(address);
    return result ? mem::MemoryResult<std::uint64_t>{result.value()}
                  : mem::MemoryResult<std::uint64_t>{result.fault()};
}

mem::MemoryResult<std::uint64_t> DmaPeripheral::writeMemory(
    const std::uint32_t address,
    const std::uint32_t width,
    const std::uint64_t value
) {
    if (memory_ == nullptr) {
        return mem::BusFault{
            mem::BusFaultReason::device_error,
            address,
            mem::AccessSize::byte,
            {mem::AccessType::data_write, 0},
            std::string{name()},
            "DMA has no attached memory bus",
        };
    }
    if (width == 1U) {
        return memory_->write8(address, static_cast<std::uint8_t>(value));
    }
    if (width == 2U) {
        return memory_->write16(address, static_cast<std::uint16_t>(value));
    }
    return memory_->write32(address, static_cast<std::uint32_t>(value));
}

void DmaPeripheral::setChannelFlag(
    const unsigned int channel,
    const unsigned int flag_bit
) {
    const unsigned int shift = (channel - 1U) * 4U;
    setRegister(dma_isr, registerValue(dma_isr) | (1U << shift) | (1U << (shift + flag_bit)));
}

DmamuxPeripheral::DmamuxPeripheral(
    std::string name,
    const unsigned int channel_count,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral(std::move(name), 0x100, event_loop, trace),
    channel_count_(normalizedMuxChannels(channel_count)) {
    reset();
}

std::uint8_t DmamuxPeripheral::requestForChannel(const unsigned int channel) const noexcept {
    if (channel >= channel_count_) {
        return 0;
    }
    return static_cast<std::uint8_t>(registerValue(channel * 4U) & 0x7fU);
}

void DmamuxPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(write_mask);
    static_cast<void>(context);
    if (word_offset < channel_count_ * 4U) {
        if ((previous & 0x7fU) != (value & 0x7fU)) ++routing_generation_;
        traceEvent("dmamux_request", {
            {"channel", std::to_string(word_offset / 4U)},
            {"request", std::to_string(value & 0x7fU)},
        });
    } else if (word_offset == 0x80U) {
        setRegister(word_offset, previous);
    } else if (word_offset == 0x84U) {
        setRegister(0x80U, registerValue(0x80U) & ~value);
        setRegister(0x84U, 0);
    }
}

} // namespace fil::stm32g4
