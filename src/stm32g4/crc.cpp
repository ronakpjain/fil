#include "fil/stm32g4/peripheral.hpp"

#include <cstdint>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t dr = 0x00;
constexpr std::uint32_t idr = 0x04;
constexpr std::uint32_t cr = 0x08;
constexpr std::uint32_t init_register = 0x10;
constexpr std::uint32_t pol_register = 0x14;
constexpr std::uint32_t cr_reset = 1U << 0U; ///< CR RESET bit.

void updateCrc(std::uint32_t& crc, const std::uint32_t polynomial,
               const std::uint32_t data_word, const std::uint32_t byte_count) {
    // DR subword accesses are right-aligned; process the supplied bytes MSB first.
    for (std::uint32_t byte_index = 0U; byte_index < byte_count; ++byte_index) {
        const std::uint32_t shift = (byte_count - 1U - byte_index) * 8U;
        crc ^= ((data_word >> shift) & 0xffU) << 24U;
        for (std::uint32_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x80000000U) != 0U ? (crc << 1U) ^ polynomial : (crc << 1U);
        }
    }
}

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
        return crc_; // REV_OUT is intentionally outside this simplified model.
    }
    if (word_offset == idr) return registerValue(idr) & 0xffU;
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
        // RM0440 16.3.3: subword accesses feed only their right-aligned byte(s).
        const std::uint32_t byte_count = write_mask == 0x000000ffU ? 1U
            : write_mask == 0x0000ffffU ? 2U : 4U;
        if (byte_count == 4U) update(value);
        else updateCrc(crc_, poly_, value, byte_count);
    } else if (word_offset == idr) {
        setRegister(idr, value & 0xffU);
    } else if (word_offset == cr) {
        if ((value & write_mask & cr_reset) != 0U) {
            crc_ = init_;
        }
    } else if (word_offset == init_register) {
        init_ = value;
        crc_ = value;
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
    updateCrc(crc_, poly_, data_word, 4U);
}

} // namespace fil::stm32g4
