#include "fil/stm32g4/peripheral.hpp"

#include <utility>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t moder = 0x00;
constexpr std::uint32_t pupdr = 0x0c;
constexpr std::uint32_t idr = 0x10;
constexpr std::uint32_t odr = 0x14;
constexpr std::uint32_t bsrr = 0x18;
constexpr std::uint32_t brr = 0x28;

} // namespace

GpioPeripheral::GpioPeripheral(
    std::string name,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral(std::move(name), 0x2c, event_loop, trace) {
    setResetValue(moder, 0xffffffffU);
    reset();
}

void GpioPeripheral::setInput(const unsigned int pin, const bool high) {
    if (pin >= 16U) {
        return;
    }
    const bool previous = (inputValue() & (1U << pin)) != 0U;
    const std::uint16_t mask = static_cast<std::uint16_t>(1U << pin);
    external_input_mask_ = static_cast<std::uint16_t>(external_input_mask_ | mask);
    if (high) {
        external_input_value_ = static_cast<std::uint16_t>(external_input_value_ | mask);
    } else {
        external_input_value_ = static_cast<std::uint16_t>(external_input_value_ & ~mask);
    }
    traceEvent("gpio_input", {
        {"pin", std::to_string(pin)},
        {"value", high ? "1" : "0"},
    });
    const bool next = (inputValue() & mask) != 0U;
    if (input_callback_ && previous != next) input_callback_(pin, previous, next);
}

void GpioPeripheral::releaseInput(const unsigned int pin) {
    if (pin < 16U) {
        const bool previous = (inputValue() & (1U << pin)) != 0U;
        external_input_mask_ = static_cast<std::uint16_t>(
            external_input_mask_ & static_cast<std::uint16_t>(~(1U << pin))
        );
        traceEvent("gpio_input", {
            {"pin", std::to_string(pin)},
            {"value", "release"},
        });
        const bool next = (inputValue() & (1U << pin)) != 0U;
        if (input_callback_ && previous != next) input_callback_(pin, previous, next);
    }
}

bool GpioPeripheral::output(const unsigned int pin) const noexcept {
    return pin < 16U && (registerValue(odr) & (1U << pin)) != 0U;
}

void GpioPeripheral::setOutputCallback(OutputCallback callback) {
    output_callback_ = std::move(callback);
}

void GpioPeripheral::setInputCallback(InputCallback callback) {
    input_callback_ = std::move(callback);
}

std::uint32_t GpioPeripheral::loadRegister(
    const std::uint32_t word_offset,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    if (word_offset == idr) {
        const std::uint32_t value = inputValue();
        setRegister(idr, value);
        return value;
    }
    if (word_offset == bsrr || word_offset == brr) {
        return 0;
    }
    return registerValue(word_offset);
}

void GpioPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    if (word_offset == odr) {
        setRegister(odr, previous & 0xffffU);
        applyOutput(value);
    } else if (word_offset == bsrr) {
        const std::uint32_t command = value & write_mask;
        const std::uint32_t set_bits = command & 0xffffU;
        const std::uint32_t reset_bits = (command >> 16U) & 0xffffU;
        const std::uint32_t next = (registerValue(odr) & ~reset_bits) | set_bits;
        setRegister(bsrr, 0);
        applyOutput(next);
    } else if (word_offset == brr) {
        const std::uint32_t command = value & write_mask & 0xffffU;
        setRegister(brr, 0);
        applyOutput(registerValue(odr) & ~command);
    } else if (word_offset == idr) {
        setRegister(idr, previous);
    }
}

void GpioPeripheral::applyOutput(const std::uint32_t new_output) {
    const std::uint32_t previous = registerValue(odr) & 0xffffU;
    const std::uint32_t next = new_output & 0xffffU;
    setRegister(odr, next);
    const std::uint32_t changed = previous ^ next;
    for (unsigned int pin = 0; pin < 16U; ++pin) {
        const std::uint32_t mask = 1U << pin;
        if ((changed & mask) == 0U) {
            continue;
        }
        const bool high = (next & mask) != 0U;
        const GpioTransition transition{currentTime(), pin, high};
        if (transition_history_enabled_) transitions_.push_back(transition);
        traceEvent("gpio_output", {
            {"pin", std::to_string(pin)},
            {"value", high ? "1" : "0"},
        });
        if (output_callback_) {
            output_callback_(pin, high, transition.time_ns);
        }
    }
}

std::uint32_t GpioPeripheral::inputValue() const noexcept {
    const std::uint32_t modes = registerValue(moder);
    const std::uint32_t outputs = registerValue(odr);
    std::uint32_t result = external_input_value_ & external_input_mask_;
    for (unsigned int pin = 0; pin < 16U; ++pin) {
        const std::uint32_t mask = 1U << pin;
        if ((external_input_mask_ & mask) != 0U) {
            continue;
        }
        const std::uint32_t mode = (modes >> (pin * 2U)) & 0x3U;
        if ((mode == 1U || mode == 2U) && (outputs & mask) != 0U) {
            result |= mask;
        } else if (mode == 0U && ((registerValue(pupdr) >> (pin * 2U)) & 0x3U) == 1U) {
            result |= mask;
        }
    }
    return result & 0xffffU;
}

namespace {
constexpr std::uint32_t exti_imr1 = 0x00;
constexpr std::uint32_t exti_rtsr1 = 0x08;
constexpr std::uint32_t exti_ftsr1 = 0x0c;
constexpr std::uint32_t exti_pr1 = 0x14;
constexpr std::uint32_t syscfg_exticr1 = 0x08;
}

ExtiPeripheral::ExtiPeripheral(
    RegisterPeripheral& syscfg,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral("EXTI", 0x400, event_loop, trace), syscfg_(syscfg) {
    reset();
}

void ExtiPeripheral::onGpioEdge(
    const unsigned int port,
    const unsigned int pin,
    const bool previous,
    const bool high
) {
    if (pin >= 16U || previous == high) return;
    const std::uint32_t exticr = syscfg_.peekRegister(syscfg_exticr1 + (pin / 4U) * 4U);
    const unsigned int selected_port = (exticr >> ((pin % 4U) * 4U)) & 0xfU;
    const std::uint32_t mask = 1U << pin;
    if (selected_port != port || (peekRegister(exti_imr1) & mask) == 0U) return;
    const bool enabled = high ? (peekRegister(exti_rtsr1) & mask) != 0U
                              : (peekRegister(exti_ftsr1) & mask) != 0U;
    if (!enabled) return;
    setRegister(exti_pr1, peekRegister(exti_pr1) | mask);
    traceEvent("exti_edge", {{"line", std::to_string(pin)}, {"edge", high ? "rising" : "falling"}});
    if (interrupt_callback_) interrupt_callback_(pin);
}

void ExtiPeripheral::setInterruptCallback(InterruptCallback callback) {
    interrupt_callback_ = std::move(callback);
}

void ExtiPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    if (word_offset == exti_pr1) {
        setRegister(exti_pr1, previous & ~(value & write_mask));
    }
}

} // namespace fil::stm32g4
