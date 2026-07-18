#include "fil/stm32g4/peripheral.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace fil::stm32g4 {
namespace {

sim::SimTimeNs boundedDuration(const long double duration) noexcept {
    if (duration >= static_cast<long double>(std::numeric_limits<sim::SimTimeNs>::max())) {
        return std::numeric_limits<sim::SimTimeNs>::max();
    }
    return std::max<sim::SimTimeNs>(static_cast<sim::SimTimeNs>(std::ceil(duration)), 1U);
}

} // namespace

IwdgPeripheral::IwdgPeripheral(
    const bool reset_enabled,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral("IWDG", 0x14, event_loop, trace),
    reset_enabled_(reset_enabled) {
    setResetValue(0x08, 0x0fffU);
    setResetValue(0x10, 0x0fffU);
    reset();
}

IwdgPeripheral::~IwdgPeripheral() {
    cancelTimeout();
}

void IwdgPeripheral::setResetCallback(ResetCallback callback) {
    reset_callback_ = std::move(callback);
}

void IwdgPeripheral::setResetEnabled(const bool enabled) {
    reset_enabled_ = enabled;
    if (running_) {
        reload();
    }
}

std::uint32_t IwdgPeripheral::loadRegister(
    const std::uint32_t word_offset,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    if (word_offset == 0x00U || word_offset == 0x0cU) {
        return 0;
    }
    return registerValue(word_offset);
}

void IwdgPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(write_mask);
    static_cast<void>(context);
    if (word_offset == 0x00U) {
        const std::uint16_t key = static_cast<std::uint16_t>(value);
        if (key == 0x5555U) {
            registers_unlocked_ = true;
        } else if (key == 0xccccU) {
            running_ = true;
            reload();
        } else if (key == 0xaaaaU && running_) {
            reload();
        }
        setRegister(0x00, 0);
    } else if ((word_offset == 0x04U || word_offset == 0x08U || word_offset == 0x10U)
        && !registers_unlocked_) {
        setRegister(word_offset, previous);
    } else if (word_offset == 0x04U) {
        setRegister(word_offset, value & 0x7U);
    } else if (word_offset == 0x08U || word_offset == 0x10U) {
        setRegister(word_offset, value & 0x0fffU);
    } else if (word_offset == 0x0cU) {
        setRegister(word_offset, previous);
    }
}

void IwdgPeripheral::onReset() {
    cancelTimeout();
    running_ = false;
    registers_unlocked_ = false;
}

void IwdgPeripheral::reload() {
    cancelTimeout();
    traceEvent("watchdog_reload");
    if (!running_ || eventLoop() == nullptr) {
        return;
    }
    const std::uint32_t divider_code = std::min(registerValue(0x04) & 0x7U, 6U);
    const std::uint64_t prescaler = std::uint64_t{4} << divider_code;
    const std::uint64_t ticks = static_cast<std::uint64_t>(registerValue(0x08) & 0x0fffU) + 1U;
    const sim::SimTimeNs duration = boundedDuration(
        static_cast<long double>(ticks) * static_cast<long double>(prescaler) * 1000000000.0L / 32000.0L
    );
    timeout_event_ = eventLoop()->scheduleAfter(duration, [this]() {
        timeout_event_ = 0;
        traceEvent("watchdog_timeout");
        if (reset_enabled_ && reset_callback_) {
            reset_callback_();
        }
    });
}

void IwdgPeripheral::cancelTimeout() noexcept {
    if (timeout_event_ != 0 && eventLoop() != nullptr) {
        static_cast<void>(eventLoop()->cancel(timeout_event_));
    }
    timeout_event_ = 0;
}

WwdgPeripheral::WwdgPeripheral(
    const bool reset_enabled,
    const std::uint64_t peripheral_clock_hz,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral("WWDG", 0x0c, event_loop, trace),
    reset_enabled_(reset_enabled),
    peripheral_clock_hz_(peripheral_clock_hz == 0 ? 1U : peripheral_clock_hz) {
    setResetValue(0x00, 0x7fU);
    setResetValue(0x04, 0x7fU);
    reset();
}

WwdgPeripheral::~WwdgPeripheral() {
    cancelTimeout();
}

void WwdgPeripheral::setResetCallback(ResetCallback callback) {
    reset_callback_ = std::move(callback);
}

void WwdgPeripheral::setResetEnabled(const bool enabled) {
    reset_enabled_ = enabled;
    if ((registerValue(0x00) & (1U << 7U)) != 0U) {
        reload();
    }
}

void WwdgPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(write_mask);
    static_cast<void>(context);
    if (word_offset == 0x00U) {
        setRegister(0x00, value & 0xffU);
        if ((value & (1U << 7U)) != 0U) {
            reload();
        } else {
            cancelTimeout();
        }
    } else if (word_offset == 0x08U) {
        setRegister(0x08, previous & value);
    }
}

void WwdgPeripheral::onReset() {
    cancelTimeout();
}

void WwdgPeripheral::reload() {
    cancelTimeout();
    traceEvent("watchdog_reload");
    if (eventLoop() == nullptr) {
        return;
    }
    const std::uint64_t counter = registerValue(0x00) & 0x7fU;
    const std::uint64_t steps = counter > 0x3fU ? counter - 0x3fU : 1U;
    const std::uint64_t prescaler = std::uint64_t{1} << ((registerValue(0x04) >> 7U) & 0x3U);
    const sim::SimTimeNs duration = boundedDuration(
        static_cast<long double>(steps) * 4096.0L * static_cast<long double>(prescaler)
        * 1000000000.0L / static_cast<long double>(peripheral_clock_hz_)
    );
    timeout_event_ = eventLoop()->scheduleAfter(duration, [this]() {
        timeout_event_ = 0;
        traceEvent("watchdog_timeout");
        if (reset_enabled_ && reset_callback_) {
            reset_callback_();
        }
    });
}

void WwdgPeripheral::cancelTimeout() noexcept {
    if (timeout_event_ != 0 && eventLoop() != nullptr) {
        static_cast<void>(eventLoop()->cancel(timeout_event_));
    }
    timeout_event_ = 0;
}

} // namespace fil::stm32g4
