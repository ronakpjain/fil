#include "fil/stm32g4/peripheral.hpp"

#include <algorithm>
#include <utility>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t cr = 0x00;
constexpr std::uint32_t cfgr = 0x08;
constexpr std::uint32_t pllcfgr = 0x0c;
constexpr std::uint32_t cifr = 0x1c;
constexpr std::uint32_t cicr = 0x20;
constexpr std::uint32_t bdcr = 0x90;
constexpr std::uint32_t csr = 0x94;
constexpr std::uint32_t crrcr = 0x98;

constexpr std::uint32_t msion = 1U << 0U;
constexpr std::uint32_t msirdy = 1U << 1U;
constexpr std::uint32_t hsion = 1U << 8U;
constexpr std::uint32_t hsirdy = 1U << 10U;
constexpr std::uint32_t hseon = 1U << 16U;
constexpr std::uint32_t hserdy = 1U << 17U;
constexpr std::uint32_t pllon = 1U << 24U;
constexpr std::uint32_t pllrdy = 1U << 25U;

} // namespace

RccPeripheral::RccPeripheral(
    const bool hse_present,
    const std::uint64_t hse_hz,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral("RCC", 0xa0, event_loop, trace),
    hse_present_(hse_present),
    hse_hz_(hse_hz == 0 ? 8000000U : hse_hz) {
    setResetValue(cr, hsion | hsirdy);
    setResetValue(cfgr, 0x5U); // HSI selected and reflected in SWS.
    reset();
}

void RccPeripheral::setClockChangedCallback(std::function<void(std::uint64_t)> callback) {
    clock_changed_ = std::move(callback);
}

void RccPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(previous);
    static_cast<void>(write_mask);
    static_cast<void>(context);

    if (word_offset == cr || word_offset == bdcr || word_offset == csr || word_offset == crrcr) {
        updateClockReadyBits();
    } else if (word_offset == cfgr) {
        const std::uint32_t selected = value & 0x3U;
        setRegister(cfgr, (value & ~0x0cU) | (selected << 2U));
    } else if (word_offset == cicr) {
        setRegister(cifr, registerValue(cifr) & ~value);
        setRegister(cicr, 0);
    }

    if (word_offset == cr || word_offset == cfgr || word_offset == pllcfgr) {
        updateSystemClock();
    }
}

void RccPeripheral::onReset() {
    const std::uint64_t previous_clock = system_clock_hz_;
    system_clock_hz_ = 16000000;
    updateClockReadyBits();
    if (previous_clock != system_clock_hz_ && clock_changed_) {
        traceEvent("clock_change", {{"frequency_hz", std::to_string(system_clock_hz_)}});
        clock_changed_(system_clock_hz_);
    }
}

void RccPeripheral::updateClockReadyBits() {
    std::uint32_t value = registerValue(cr);
    value = (value & ~msirdy) | ((value & msion) != 0U ? msirdy : 0U);
    value = (value & ~hsirdy) | ((value & hsion) != 0U ? hsirdy : 0U);
    value = (value & ~hserdy) | (((value & hseon) != 0U && hse_present_) ? hserdy : 0U);
    value = (value & ~pllrdy) | ((value & pllon) != 0U ? pllrdy : 0U);
    setRegister(cr, value);

    std::uint32_t backup = registerValue(bdcr);
    backup = (backup & ~(1U << 1U)) | ((backup & 1U) != 0U ? (1U << 1U) : 0U);
    setRegister(bdcr, backup);

    std::uint32_t low_speed = registerValue(csr);
    low_speed = (low_speed & ~(1U << 1U)) | ((low_speed & 1U) != 0U ? (1U << 1U) : 0U);
    setRegister(csr, low_speed);

    std::uint32_t hsi48 = registerValue(crrcr);
    hsi48 = (hsi48 & ~(1U << 1U)) | ((hsi48 & 1U) != 0U ? (1U << 1U) : 0U);
    setRegister(crrcr, hsi48);
}

std::uint64_t RccPeripheral::pllClockHz() const noexcept {
    const std::uint32_t config = registerValue(pllcfgr);
    const std::uint32_t source_selector = config & 0x3U;
    std::uint64_t source_hz = 0;
    if (source_selector == 1U) {
        source_hz = 4000000; // Permissive fixed MSI estimate.
    } else if (source_selector == 2U) {
        source_hz = 16000000;
    } else if (source_selector == 3U) {
        source_hz = hse_hz_;
    }
    if (source_hz == 0) {
        return 16000000;
    }

    const std::uint64_t divider_m = ((config >> 4U) & 0x0fU) + 1U;
    const std::uint64_t multiplier_n = std::max<std::uint64_t>((config >> 8U) & 0x7fU, 1U);
    const std::uint64_t divider_r = 2U * (((config >> 25U) & 0x3U) + 1U);
    return (source_hz / divider_m) * multiplier_n / divider_r;
}

void RccPeripheral::updateSystemClock() {
    const std::uint32_t selected = registerValue(cfgr) & 0x3U;
    std::uint64_t next_clock = 16000000;
    if (selected == 0U) {
        next_clock = 4000000;
    } else if (selected == 2U && hse_present_) {
        next_clock = hse_hz_;
    } else if (selected == 3U) {
        next_clock = pllClockHz();
    }
    if (next_clock == 0 || next_clock == system_clock_hz_) {
        return;
    }
    system_clock_hz_ = next_clock;
    traceEvent("clock_change", {{"frequency_hz", std::to_string(system_clock_hz_)}});
    if (clock_changed_) {
        clock_changed_(system_clock_hz_);
    }
}

} // namespace fil::stm32g4
