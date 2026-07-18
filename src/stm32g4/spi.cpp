#include "fil/stm32g4/peripheral.hpp"

#include <algorithm>
#include <utility>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t cr2 = 0x04;
constexpr std::uint32_t sr = 0x08;
constexpr std::uint32_t dr = 0x0c;
constexpr std::uint32_t rxne = 1U << 0U;
constexpr std::uint32_t txe = 1U << 1U;
constexpr std::uint32_t bsy = 1U << 7U;

} // namespace

SpiPeripheral::SpiPeripheral(
    std::string name,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral(std::move(name), 0x24, event_loop, trace) {
    setResetValue(sr, txe);
    reset();
}

mem::MemoryResult<std::uint64_t> SpiPeripheral::read(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const mem::AccessContext& context
) {
    if (offset != dr || mem::byteCount(size) > 4U) {
        return RegisterPeripheral::read(offset, size, context);
    }
    const std::uint32_t width = mem::byteCount(size);
    std::uint64_t value = 0;
    for (std::uint32_t index = 0; index < width && !receive_bytes_.empty(); ++index) {
        value |= static_cast<std::uint64_t>(receive_bytes_.front()) << (index * 8U);
        receive_bytes_.pop_front();
    }
    setRegister(dr, static_cast<std::uint32_t>(value));
    updateStatus();
    signalRequests();
    return value;
}

void SpiPeripheral::setTransferCallback(TransferCallback callback) {
    transfer_callback_ = std::move(callback);
}

void SpiPeripheral::setInterruptCallback(InterruptCallback callback) {
    interrupt_callback_ = std::move(callback);
    signalRequests();
}

void SpiPeripheral::setDmaRequestCallback(DmaRequestCallback callback) {
    dma_request_callback_ = std::move(callback);
    signalRequests();
}

std::uint32_t SpiPeripheral::loadRegister(
    const std::uint32_t word_offset,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    updateStatus();
    return registerValue(word_offset);
}

void SpiPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    if (word_offset == dr) {
        transferWord(value, write_mask);
    } else if (word_offset == sr) {
        setRegister(sr, previous);
    } else if (word_offset == cr2) {
        signalRequests();
    }
}

void SpiPeripheral::onReset() {
    receive_bytes_.clear();
    setRegister(sr, txe);
}

void SpiPeripheral::transferWord(const std::uint32_t value, const std::uint32_t write_mask) {
    std::size_t byte_count = 1;
    if ((write_mask & 0xffffff00U) != 0U) {
        byte_count = 2;
    }
    std::vector<std::uint8_t> transmitted(byte_count, 0);
    for (std::size_t index = 0; index < byte_count; ++index) {
        transmitted[index] = static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU);
    }

    setRegister(sr, (registerValue(sr) | bsy) & ~txe);
    std::vector<std::uint8_t> received;
    if (transfer_callback_) {
        received = transfer_callback_(transmitted, currentTime());
    } else if (echo_) {
        received = transmitted;
    }
    received.resize(byte_count, 0);
    if (received.size() > byte_count) {
        received.resize(byte_count);
    }
    for (const std::uint8_t byte : received) {
        receive_bytes_.push_back(byte);
    }

    transfers_.push_back(SpiTransfer{currentTime(), transmitted, received});
    traceEvent("spi_transfer", {
        {"tx", sim::TraceRecorder::hexBytes(transmitted)},
        {"rx", sim::TraceRecorder::hexBytes(received)},
    });
    setRegister(sr, (registerValue(sr) & ~bsy) | txe);
    updateStatus();
    signalRequests();
}

void SpiPeripheral::updateStatus() {
    std::uint32_t status = registerValue(sr) | txe;
    if (receive_bytes_.empty()) {
        status &= ~rxne;
    } else {
        status |= rxne;
    }
    setRegister(sr, status);
}

void SpiPeripheral::signalRequests() {
    updateStatus();
    const std::uint32_t control = registerValue(cr2);
    const std::uint32_t status = registerValue(sr);
    if (interrupt_callback_
        && ((((control & (1U << 6U)) != 0U) && ((status & rxne) != 0U))
            || (((control & (1U << 7U)) != 0U) && ((status & txe) != 0U)))) {
        interrupt_callback_();
    }
    if (dma_request_callback_) {
        if ((control & 1U) != 0U && (status & rxne) != 0U) {
            dma_request_callback_(false);
        }
        if ((control & (1U << 1U)) != 0U && (status & txe) != 0U) {
            dma_request_callback_(true);
        }
    }
}

} // namespace fil::stm32g4
