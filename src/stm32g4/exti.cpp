#include "fil/stm32g4/peripheral.hpp"

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t exticr1 = 0x08;
constexpr std::uint32_t exticr2 = 0x0c;
constexpr std::uint32_t exticr3 = 0x10;
constexpr std::uint32_t exticr4 = 0x14;

constexpr std::uint32_t imr1 = 0x00;
constexpr std::uint32_t emr1 = 0x04;
constexpr std::uint32_t rtsr1 = 0x08;
constexpr std::uint32_t ftsr1 = 0x0c;
constexpr std::uint32_t swier1 = 0x10;
constexpr std::uint32_t pr1 = 0x14;

} // namespace

SyscfgPeripheral::SyscfgPeripheral(
    std::string name,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral(std::move(name), 0x400, event_loop, trace) {
    reset();
}

int SyscfgPeripheral::portForLine(const unsigned int line) const noexcept {
    if (line > 15U) return -1;
    const unsigned int reg = line / 4U;
    const unsigned int field = line % 4U;
    const std::uint32_t offsets[4] = {exticr1, exticr2, exticr3, exticr4};
    const std::uint32_t value = peekRegister(offsets[reg]);
    const unsigned int port = (value >> (field * 4U)) & 0xFU;
    // RM0440: 0=PA,1=PB,2=PC,3=PD,4=PE,5=PF,6=PG. Values 7-15 are reserved;
    // treat them as unrouted so edges never pend.
    if (port > 6U) return -1;
    return static_cast<int>(port);
}

ExtiPeripheral::ExtiPeripheral(
    std::string name,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral(std::move(name), 0x400, event_loop, trace) {
    reset();
}

std::uint32_t ExtiPeripheral::loadRegister(
    const std::uint32_t word_offset,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    return registerValue(word_offset);
}

void ExtiPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    static_cast<void>(previous);
    if (word_offset == swier1) {
        // SWIER is W1S: writing 1 pends the corresponding PR bit.
        const std::uint32_t set_bits = value & write_mask;
        if (set_bits != 0U) {
            const std::uint32_t new_pr = registerValue(pr1) | set_bits;
            setRegister(pr1, new_pr);
            setRegister(swier1, 0);
            for (unsigned int line = 0; line < 32U; ++line) {
                if ((set_bits & (1U << line)) != 0U) {
                    traceEvent("exti_pending", {
                        {"line", std::to_string(line)},
                        {"source", "software"},
                    });
                }
            }
        } else {
            setRegister(swier1, 0);
        }
        updateInterruptLevels();
        return;
    }
    if (word_offset == pr1) {
        // PR is W1C (rc_w1): writing 1 clears, writing 0 has no effect.
        // Note: `value` is the merged register image (previous & ~mask | written),
        // so the written ones are `value & write_mask`, but the base must be
        // `previous` (pre-write PR), not the merged image which already lost
        // uncleared bits.
        const std::uint32_t clear_bits = value & write_mask;
        const std::uint32_t new_pr = previous & ~clear_bits;
        setRegister(pr1, new_pr);
        if (clear_bits != 0U) {
            traceEvent("exti_clear", {
                {"mask", std::to_string(clear_bits)},
            });
        }
        updateInterruptLevels();
        return;
    }
    if (word_offset == imr1 || word_offset == emr1 || word_offset == rtsr1 || word_offset == ftsr1) {
        updateInterruptLevels();
        return;
    }
    // Other offsets (IMR2 etc.) are generic storage with no modelled lines.
}

void ExtiPeripheral::onReset() {
    for (unsigned int line = 0; line < 8U; ++line) {
        setInterruptLevel(line, false);
    }
}

void ExtiPeripheral::notifyGpioEdge(
    const unsigned int port,
    const unsigned int pin,
    const bool high
) {
    if (pin > 15U || port > 6U) return;
    if (syscfg_ != nullptr) {
        const int routed = syscfg_->portForLine(pin);
        if (routed < 0 || static_cast<unsigned int>(routed) != port) return;
    }
    const std::uint32_t mask = 1U << pin;
    const std::uint32_t rtsr = registerValue(rtsr1);
    const std::uint32_t ftsr = registerValue(ftsr1);
    const bool rising_armed = (rtsr & mask) != 0U;
    const bool falling_armed = (ftsr & mask) != 0U;
    const bool triggered = (high && rising_armed) || (!high && falling_armed);
    if (!triggered) return;
    const std::uint32_t new_pr = registerValue(pr1) | mask;
    setRegister(pr1, new_pr);
    traceEvent("exti_pending", {
        {"line", std::to_string(pin)},
        {"source", high ? "rising" : "falling"},
    });
    updateInterruptLevels();
}

bool ExtiPeripheral::linePendingEnabled(const unsigned int line) const noexcept {
    if (line > 15U) return false;
    const std::uint32_t mask = 1U << line;
    return (registerValue(pr1) & mask) != 0U && (registerValue(imr1) & mask) != 0U;
}

void ExtiPeripheral::updateInterruptLevels() {
    // Lines 0-4 have dedicated NVIC IRQs; 5-9 share one; 10-15 share another.
    // RegisterPeripheral supports 8 lines; we use 0-6.
    for (unsigned int dedicated = 0; dedicated < 5U; ++dedicated) {
        setInterruptLevel(dedicated, linePendingEnabled(dedicated));
    }
    bool shared_5_9 = false;
    for (unsigned int line = 5U; line <= 9U; ++line) {
        if (linePendingEnabled(line)) {
            shared_5_9 = true;
            break;
        }
    }
    setInterruptLevel(5, shared_5_9);
    bool shared_10_15 = false;
    for (unsigned int line = 10U; line <= 15U; ++line) {
        if (linePendingEnabled(line)) {
            shared_10_15 = true;
            break;
        }
    }
    setInterruptLevel(6, shared_10_15);
    // Line 7 unused.
    setInterruptLevel(7, false);
}

} // namespace fil::stm32g4
