#include "fil/stm32g4/peripheral.hpp"

#include <algorithm>
#include <cstdint>
#include <utility>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t cr = 0x00;
constexpr std::uint32_t cfgr = 0x08;
constexpr std::uint32_t pllcfgr = 0x0c;
constexpr std::uint32_t ahb2rstr = 0x2c;
constexpr std::uint32_t ahb2enr = 0x4c;
constexpr std::uint32_t ccipr = 0x88;
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

void RccPeripheral::setAdcClockChangedCallback(
    std::function<void(std::uint64_t, std::uint64_t, std::uint64_t, bool, bool, bool)> callback
) {
    adc_clock_changed_ = std::move(callback);
    if (adc_clock_changed_) {
        const std::uint32_t ahb2 = registerValue(ahb2enr);
        adc_clock_changed_(hclk_hz_, adc12_clock_hz_, adc345_clock_hz_,
            (ahb2 & (1U << 13U)) != 0U, (ahb2 & (1U << 14U)) != 0U,
            ((registerValue(cfgr) >> 4U) & 0x0fU) < 8U);
    }
}

void RccPeripheral::setAdcResetChangedCallback(std::function<void(bool, bool)> callback) {
    adc_reset_changed_ = std::move(callback);
    if (adc_reset_changed_) updateAdcResets();
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
    if (word_offset == cr || word_offset == cfgr || word_offset == pllcfgr
        || word_offset == ccipr || word_offset == ahb2enr) {
        updateAdcClocks();
    }
    if (word_offset == ahb2rstr) updateAdcResets();
}

void RccPeripheral::onReset() {
    const std::uint64_t previous_clock = system_clock_hz_;
    system_clock_hz_ = 16000000;
    hclk_hz_ = 16000000;
    adc12_clock_hz_ = 0U;
    adc345_clock_hz_ = 0U;
    updateClockReadyBits();
    updateAdcClocks(true);
    updateAdcResets();
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

std::uint64_t RccPeripheral::hclkClockHz() const noexcept {
    const std::uint32_t hpre = (registerValue(cfgr) >> 4U) & 0x0fU;
    static constexpr std::uint16_t divisors[16]{1,1,1,1,1,1,1,1,2,4,8,16,64,128,256,512};
    return system_clock_hz_ / divisors[hpre];
}

std::uint64_t RccPeripheral::adcKernelClockHz(const std::uint32_t selector) const noexcept {
    if (selector == 0U || selector == 3U) return 0U;
    if (selector == 2U) return system_clock_hz_;
    const std::uint32_t config = registerValue(pllcfgr);
    const std::uint32_t source_selector = config & 0x3U;
    const std::uint32_t oscillator_state = registerValue(cr);
    if ((config & (1U << 16U)) == 0U
        || (oscillator_state & (pllon | pllrdy)) != (pllon | pllrdy)) return 0U;
    std::uint64_t source_hz = 0U;
    if (source_selector == 2U && (oscillator_state & hsirdy) != 0U) source_hz = 16000000U;
    if (source_selector == 3U && (oscillator_state & hserdy) != 0U) source_hz = hse_hz_;
    if (source_hz == 0U) return 0U;
    const std::uint64_t divider_m = ((config >> 4U) & 0x0fU) + 1U;
    const std::uint64_t multiplier_n = std::max<std::uint64_t>((config >> 8U) & 0x7fU, 1U);
    std::uint64_t p_divider = (config >> 27U) & 0x1fU;
    if (p_divider == 1U) return 0U;
    if (p_divider == 0U) p_divider = (config & (1U << 17U)) != 0U ? 17U : 7U;
    return (source_hz * multiplier_n) / (divider_m * p_divider);
}

void RccPeripheral::updateAdcClocks(const bool force_callback) {
    const std::uint64_t previous_hclk = hclk_hz_;
    hclk_hz_ = hclkClockHz();
    const std::uint32_t ccipr_value = registerValue(ccipr);
    const std::uint32_t ahb2_value = registerValue(ahb2enr);
    const bool adc12_enabled = (ahb2_value & (1U << 13U)) != 0U;
    const bool adc345_enabled = (ahb2_value & (1U << 14U)) != 0U;
    const bool hpre_div1_allowed = ((registerValue(cfgr) >> 4U) & 0x0fU) < 8U;
    // AHB2ENR gates the bus-interface clock. The asynchronous conversion
    // kernel is a separate clock domain (RM0440 21.4.3, Figure 83).
    const std::uint64_t adc12 = adcKernelClockHz((ccipr_value >> 28U) & 0x3U);
    const std::uint64_t adc345 = adcKernelClockHz((ccipr_value >> 30U) & 0x3U);
    if (!force_callback && previous_hclk == hclk_hz_
        && adc12 == adc12_clock_hz_ && adc345 == adc345_clock_hz_
        && adc12_enabled == adc12_enabled_ && adc345_enabled == adc345_enabled_
        && hpre_div1_allowed == hpre_div1_allowed_) return;
    adc12_clock_hz_ = adc12;
    adc345_clock_hz_ = adc345;
    adc12_enabled_ = adc12_enabled;
    adc345_enabled_ = adc345_enabled;
    hpre_div1_allowed_ = hpre_div1_allowed;
    if (adc_clock_changed_) {
        adc_clock_changed_(hclk_hz_, adc12_clock_hz_, adc345_clock_hz_,
            adc12_enabled_, adc345_enabled_, hpre_div1_allowed_);
    }
}

void RccPeripheral::updateAdcResets() {
    if (!adc_reset_changed_) return;
    const std::uint32_t resets = registerValue(ahb2rstr);
    adc_reset_changed_((resets & (1U << 13U)) != 0U, (resets & (1U << 14U)) != 0U);
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
