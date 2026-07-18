#include "fil/stm32g4/peripheral.hpp"

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t keyr = 0x08;
constexpr std::uint32_t optkeyr = 0x0c;
constexpr std::uint32_t sr = 0x10;
constexpr std::uint32_t cr = 0x14;
constexpr std::uint32_t busy = 1U << 16U;
constexpr std::uint32_t lock = 1U << 31U;
constexpr std::uint32_t option_lock = 1U << 30U;

} // namespace

FlashPeripheral::FlashPeripheral(
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral("FLASH", 0x40, event_loop, trace) {
    setResetValue(cr, lock | option_lock);
    reset();
}

std::uint32_t FlashPeripheral::loadRegister(
    const std::uint32_t word_offset,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    if (word_offset == keyr || word_offset == optkeyr) {
        return 0;
    }
    if (word_offset == sr) {
        setRegister(sr, registerValue(sr) & ~busy);
    }
    return registerValue(word_offset);
}

void FlashPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    if (word_offset == keyr) {
        if (value == 0x45670123U) {
            key_step_ = 1;
        } else if (key_step_ == 1U && value == 0xcdef89abU) {
            setRegister(cr, registerValue(cr) & ~lock);
            key_step_ = 0;
        } else {
            key_step_ = 0;
        }
        setRegister(keyr, 0);
    } else if (word_offset == optkeyr) {
        if (value == 0x08192a3bU) {
            option_key_step_ = 1;
        } else if (option_key_step_ == 1U && value == 0x4c5d6e7fU) {
            setRegister(cr, registerValue(cr) & ~option_lock);
            option_key_step_ = 0;
        } else {
            option_key_step_ = 0;
        }
        setRegister(optkeyr, 0);
    } else if (word_offset == sr) {
        setRegister(sr, (previous & ~(value & write_mask)) & ~busy);
    } else if (word_offset == cr && (previous & lock) != 0U) {
        setRegister(cr, previous);
    }
}

void FlashPeripheral::onReset() {
    key_step_ = 0;
    option_key_step_ = 0;
    setRegister(sr, registerValue(sr) & ~busy);
}

} // namespace fil::stm32g4
