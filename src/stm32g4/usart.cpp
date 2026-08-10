#include "fil/stm32g4/peripheral.hpp"

#include <utility>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t cr1 = 0x00;
constexpr std::uint32_t rqr = 0x18;
constexpr std::uint32_t isr = 0x1c;
constexpr std::uint32_t icr = 0x20;
constexpr std::uint32_t rdr = 0x24;
constexpr std::uint32_t tdr = 0x28;

constexpr std::uint32_t idle_flag = 1U << 4U;
constexpr std::uint32_t rxne_flag = 1U << 5U;
constexpr std::uint32_t tc_flag = 1U << 6U;
constexpr std::uint32_t txe_flag = 1U << 7U;
constexpr std::uint32_t teack_flag = 1U << 21U;
constexpr std::uint32_t reack_flag = 1U << 22U;
constexpr std::uint32_t txfe_flag = 1U << 23U;

} // namespace

UsartPeripheral::UsartPeripheral(
    std::string name,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral(std::move(name), 0x30, event_loop, trace),
    idle_event_(event_loop) {
    setResetValue(isr, tc_flag | txe_flag | txfe_flag);
    reset();
}

UsartPeripheral::~UsartPeripheral() = default;

void UsartPeripheral::injectRx(const std::span<const std::uint8_t> bytes) {
    for (const std::uint8_t byte : bytes) {
        rx_queue_.push_back(byte);
        traceEvent("uart_rx", {{"byte", std::to_string(byte)}});
    }
    updateStatus();
    scheduleIdle();
    signalInterruptIfEnabled();
}

void UsartPeripheral::injectRx(const std::uint8_t byte) {
    const std::array<std::uint8_t, 1> bytes{byte};
    injectRx(bytes);
}

void UsartPeripheral::setTxCallback(TxCallback callback) {
    tx_callback_ = std::move(callback);
}

void UsartPeripheral::setRxProvider(RxProvider provider) {
    rx_provider_ = std::move(provider);
}

void UsartPeripheral::setInterruptCallback(InterruptCallback callback) {
    interrupt_callback_ = std::move(callback);
    signalInterruptIfEnabled();
}

std::uint32_t UsartPeripheral::loadRegister(
    const std::uint32_t word_offset,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    if (word_offset == isr) {
        refillRx();
        updateStatus();
        signalInterruptIfEnabled();
    } else if (word_offset == rdr) {
        refillRx();
        std::uint32_t value = 0;
        if (!rx_queue_.empty()) {
            value = rx_queue_.front();
            rx_queue_.pop_front();
        }
        setRegister(rdr, value);
        updateStatus();
        signalInterruptIfEnabled();
        return value;
    }
    return registerValue(word_offset);
}

void UsartPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(previous);
    static_cast<void>(write_mask);
    static_cast<void>(context);
    if (word_offset == tdr) {
        const std::uint8_t byte = static_cast<std::uint8_t>(value & 0xffU);
        const UsartTxByte output{currentTime(), byte};
        tx_log_.push_back(output);
        traceEvent("uart_tx", {{"byte", std::to_string(byte)}});
        if (tx_callback_) {
            tx_callback_(byte, output.time_ns);
        }
        updateStatus();
        signalInterruptIfEnabled();
    } else if (word_offset == icr) {
        setRegister(isr, registerValue(isr) & ~value);
        setRegister(icr, 0);
        updateStatus();
    } else if (word_offset == rqr) {
        if ((value & (1U << 3U)) != 0U) {
            rx_queue_.clear();
        }
        setRegister(rqr, 0);
        updateStatus();
    } else if (word_offset == cr1) {
        updateStatus();
        scheduleIdle();
        signalInterruptIfEnabled();
    } else if (word_offset == rdr || word_offset == isr) {
        setRegister(word_offset, previous);
    }
}

void UsartPeripheral::onReset() {
    cancelIdle();
    rx_queue_.clear();
    setRegister(isr, tc_flag | txe_flag | txfe_flag);
}

void UsartPeripheral::refillRx() {
    if (!rx_queue_.empty() || !rx_provider_) {
        return;
    }
    const std::optional<std::uint8_t> byte = rx_provider_(currentTime());
    if (byte.has_value()) {
        rx_queue_.push_back(*byte);
        traceEvent("uart_rx", {{"byte", std::to_string(*byte)}});
        scheduleIdle();
    }
}

void UsartPeripheral::updateStatus() {
    std::uint32_t status = registerValue(isr) | tc_flag | txe_flag | txfe_flag;
    if (rx_queue_.empty()) {
        status &= ~rxne_flag;
    } else {
        status |= rxne_flag;
    }
    const std::uint32_t control = registerValue(cr1);
    const bool enabled = (control & 1U) != 0U;
    status = (enabled && (control & (1U << 3U)) != 0U) ? (status | teack_flag) : (status & ~teack_flag);
    status = (enabled && (control & (1U << 2U)) != 0U) ? (status | reack_flag) : (status & ~reack_flag);
    setRegister(isr, status);
}

void UsartPeripheral::signalInterruptIfEnabled() {
    if (!interrupt_callback_) {
        return;
    }
    const std::uint32_t control = registerValue(cr1);
    const std::uint32_t status = registerValue(isr);
    const bool pending = (((control & (1U << 4U)) != 0U) && ((status & idle_flag) != 0U))
        || (((control & (1U << 5U)) != 0U) && ((status & rxne_flag) != 0U))
        || (((control & (1U << 6U)) != 0U) && ((status & tc_flag) != 0U))
        || (((control & (1U << 7U)) != 0U) && ((status & txe_flag) != 0U));
    if (pending) {
        interrupt_callback_();
    }
}

void UsartPeripheral::scheduleIdle() {
    cancelIdle();
    if (eventLoop() == nullptr || rx_queue_.empty()) {
        return;
    }
    static_cast<void>(idle_event_.scheduleAfter(idle_gap_ns_, [this]() {
        setRegister(isr, registerValue(isr) | idle_flag);
        traceEvent("uart_idle");
        signalInterruptIfEnabled();
    }));
}

void UsartPeripheral::cancelIdle() noexcept {
    idle_event_.cancel();
}

} // namespace fil::stm32g4
