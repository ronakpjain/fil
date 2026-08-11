#include "fil/stm32g4/peripheral.hpp"

#include <cstdint>
#include <utility>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t keyr = 0x08;
constexpr std::uint32_t optkeyr = 0x0c;
constexpr std::uint32_t sr = 0x10;
constexpr std::uint32_t cr = 0x14;
constexpr std::uint32_t busy = 1U << 16U;
constexpr std::uint32_t lock = 1U << 31U;
constexpr std::uint32_t option_lock = 1U << 30U;

// FLASH_CR operation bits (STM32G4 RM0440).
constexpr std::uint32_t per = 1U << 1U;    ///< Page erase enable.
constexpr std::uint32_t bker = 1U << 11U;  ///< Bank select for page erase.
constexpr std::uint32_t strt = 1U << 16U; ///< Start erase or programming.

// FLASH_SR status bits.
constexpr std::uint32_t eop = 1U << 0U; ///< End of operation.

/** @brief Reconstructs the flat page number from the contiguous CR PNB field. */
std::uint32_t pageNumberFromControl(const std::uint32_t control) {
    return (control >> 3U) & 0x7fU;
}

} // namespace

FlashPeripheral::FlashPeripheral(
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral("FLASH", 0x40, event_loop, trace) {
    setResetValue(cr, lock | option_lock);
    reset();
}

void FlashPeripheral::setPageEraseCallback(PageEraseCallback callback) {
    page_erase_callback_ = std::move(callback);
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
    } else if (word_offset == cr) {
        setRegister(cr, value);
        if ((value & per) != 0U && (value & strt) != 0U) {
            performPageErase(value);
        }
    }
}

void FlashPeripheral::performPageErase(const std::uint32_t control) {
    // Model the operation synchronously: raise BSY, erase the page through the
    // installed backing, then clear BSY and report EOP on success. Reading SR
    // already masks BSY away, so firmware polling sees a completed erase.
    setRegister(sr, registerValue(sr) | busy);

    const std::uint32_t bank = (control & bker) != 0U ? 1U : 0U;
    const std::uint32_t page = pageNumberFromControl(control);
    const std::uint32_t page_base =
        flash_base_ + bank * bank_size_ + page * page_size_;

    bool succeeded = true;
    if (page_erase_callback_) {
        auto result = page_erase_callback_(page_base, page_size_);
        succeeded = static_cast<bool>(result);
    }
    if (succeeded) {
        setRegister(sr, (registerValue(sr) & ~busy) | eop);
    } else {
        setRegister(sr, registerValue(sr) & ~busy);
    }
}

void FlashPeripheral::onReset() {
    key_step_ = 0;
    option_key_step_ = 0;
    setRegister(sr, registerValue(sr) & ~busy);
}

} // namespace fil::stm32g4
