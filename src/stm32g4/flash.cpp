#include "fil/stm32g4/peripheral.hpp"

#include <cstdint>
#include <utility>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t acr = 0x00;
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

// RM0440 unlock key sequences, written in order.
constexpr std::uint32_t flash_unlock_key1 = 0x45670123U;
constexpr std::uint32_t flash_unlock_key2 = 0xcdef89abU;
constexpr std::uint32_t option_unlock_key1 = 0x08192a3bU;
constexpr std::uint32_t option_unlock_key2 = 0x4c5d6e7fU;

/** @brief Reconstructs the flat page number from the contiguous CR PNB field. */
std::uint32_t pageNumberFromControl(const std::uint32_t control) {
    return (control >> 3U) & 0x7fU;
}

} // namespace

FlashPeripheral::FlashPeripheral(
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral("FLASH", 0x40, event_loop, trace) {
    // RM0440 §3.7.1 category-3 reset: 1 WS, prefetch off, I/D caches enabled.
    setResetValue(acr, 0x00040601U);
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
    if (word_offset == acr) {
        // RM0440 FLASH_ACR: LATENCY[2:0], PRFTEN(8), ICEN(9), DCEN(10),
        // ICRST(11), DCRST(12). Reset bits clear immediately (no cached
        // lines are modelled); LATENCY/PRFTEN/ICEN/DCEN changes bump the
        // generation so loop proofs fail closed across ACR rewrites.
        constexpr std::uint32_t writable =
            0x7U | (1U << 8U) | (1U << 9U) | (1U << 10U) | (1U << 11U) | (1U << 12U);
        const std::uint32_t masked = value & writable & write_mask;
        std::uint32_t next = (previous & ~(writable & write_mask))
            | (masked & ~((1U << 11U) | (1U << 12U)));
        setRegister(acr, next);
        acr_ = next;
        if (((previous ^ next) & (0x7U | (1U << 8U) | (1U << 9U) | (1U << 10U))) != 0U) {
            ++acr_generation_;
            if (acr_generation_ == 0U) acr_generation_ = 1U;
        }
        return;
    }
    if (word_offset == keyr) {
        if (advanceKeySequence(key_sequence_, value, flash_unlock_key1, flash_unlock_key2)) {
            setRegister(cr, registerValue(cr) & ~lock);
        }
        setRegister(keyr, 0);
    } else if (word_offset == optkeyr) {
        if (advanceKeySequence(
                option_key_sequence_, value, option_unlock_key1, option_unlock_key2)) {
            setRegister(cr, registerValue(cr) & ~option_lock);
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

bool FlashPeripheral::advanceKeySequence(
    KeySequence& sequence,
    const std::uint32_t value,
    const std::uint32_t first_key,
    const std::uint32_t second_key
) noexcept {
    switch (sequence) {
    case KeySequence::first_key_seen:
        if (value == second_key) {
            sequence = KeySequence::idle;
            return true;
        }
        break;
    case KeySequence::idle:
        break;
    }
    sequence = value == first_key ? KeySequence::first_key_seen : KeySequence::idle;
    return false;
}

void FlashPeripheral::onReset() {
    key_sequence_ = KeySequence::idle;
    option_key_sequence_ = KeySequence::idle;
    acr_ = registerValue(acr);
    ++acr_generation_;
    if (acr_generation_ == 0U) acr_generation_ = 1U;
    setRegister(sr, registerValue(sr) & ~busy);
}

} // namespace fil::stm32g4
