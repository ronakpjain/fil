#include "fil/stm32g4/peripheral.hpp"

#include <cstdint>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t dr = 0x00;
constexpr std::uint32_t cr = 0x08;
constexpr std::uint32_t init_register = 0x10;
constexpr std::uint32_t pol_register = 0x14;
constexpr std::uint32_t cr_reset = 1U << 0U;  ///< CR RESET bit.
constexpr std::uint32_t cr_init = 1U << 7U;   ///< CR INIT bit.

} // namespace

CrcPeripheral::CrcPeripheral(
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral("CRC", 0x18, event_loop, trace) {
    setResetValue(init_register, 0xffffffffU);
    setResetValue(pol_register, 0x04c11db7U);
    reset();
}

std::uint32_t CrcPeripheral::loadRegister(
    const std::uint32_t word_offset,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    if (word_offset == dr) {
        return crc_;
    }
    return registerValue(word_offset);
}

void CrcPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    if (word_offset == dr) {
        update(value);
    } else if (word_offset == cr) {
        // RESET and INIT both reload the INIT register into the calculator.
        if ((value & write_mask & (cr_reset | cr_init)) != 0U) {
            crc_ = init_;
        }
    } else if (word_offset == init_register) {
        init_ = value;
    } else if (word_offset == pol_register) {
        poly_ = value;
    }
    static_cast<void>(previous);
}

void CrcPeripheral::onReset() {
    init_ = registerValue(init_register);
    poly_ = registerValue(pol_register);
    crc_ = init_;
}

void CrcPeripheral::update(const std::uint32_t data_word) {
    // CRC-32/MPEG-2 word-wise update: process the four bytes most-significant
    // byte first with no bit reflection and no final XOR.
    for (unsigned int byte_index = 0U; byte_index < 4U; ++byte_index) {
        const std::uint32_t data_byte =
            (data_word >> (24U - byte_index * 8U)) & 0xffU;
        crc_ ^= data_byte << 24U;
        for (unsigned int bit = 0U; bit < 8U; ++bit) {
            crc_ = (crc_ & 0x80000000U) != 0U
                ? (crc_ << 1U) ^ poly_
                : (crc_ << 1U);
        }
    }
}

} // namespace fil::stm32g4
