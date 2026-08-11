#include "fil/stm32g4/peripheral.hpp"

namespace fil::stm32g4 {

PwrPeripheral::PwrPeripheral(
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral("PWR", 0x40, event_loop, trace) {
    // STM32G4 resets in voltage-scaling range 1, which is required for flash
    // programming at the firmware's configured clock rate.
    setResetValue(0x00U, 1U << 9U);
    reset();
}

std::uint32_t PwrPeripheral::loadRegister(
    const std::uint32_t word_offset,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    std::uint32_t value = registerValue(word_offset);
    if (word_offset == 0x14U) {
        value &= ~(1U << 10U); // Voltage scaling transition is always complete.
        setRegister(word_offset, value);
    }
    return value;
}

} // namespace fil::stm32g4
