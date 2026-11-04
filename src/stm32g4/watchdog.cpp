#include "fil/stm32g4/peripheral.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace fil::stm32g4 {
namespace {

constexpr std::uint16_t key_unlock = 0x5555U;
constexpr std::uint16_t key_start = 0xccccU;
constexpr std::uint16_t key_reload = 0xaaaaU;

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
    reset_enabled_(reset_enabled),
    timeout_event_(event_loop) {
    setResetValue(0x08, 0x0fffU);
    setResetValue(0x10, 0x0fffU);
    reset();
}

void IwdgPeripheral::setResetEnabled(const bool enabled) {
    reset_enabled_ = enabled;
    reloadIfRunning();
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
        applyKey(static_cast<std::uint16_t>(value));
        setRegister(0x00, 0);
    } else if ((word_offset == 0x04U || word_offset == 0x08U || word_offset == 0x10U)
        && unlock_ == UnlockState::locked) {
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
    state_ = State::stopped;
    unlock_ = UnlockState::locked;
}

void IwdgPeripheral::applyKey(const std::uint16_t key) {
    switch (key) {
    case key_unlock:
        unlock_ = UnlockState::key1_seen;
        return;
    case key_start:
        enterRunning();
        return;
    case key_reload:
        reloadIfRunning();
        return;
    default:
        return;
    }
}

void IwdgPeripheral::enterRunning() {
    switch (state_) {
    case State::stopped:
        state_ = State::running;
        break;
    case State::running:
        break;
    }
    reload();
}

void IwdgPeripheral::reloadIfRunning() {
    switch (state_) {
    case State::stopped:
        return;
    case State::running:
        reload();
        return;
    }
}

void IwdgPeripheral::reload() {
    cancelTimeout();
    traceEvent("watchdog_reload");
    switch (state_) {
    case State::stopped:
        return;
    case State::running:
        break;
    }
    if (eventLoop() == nullptr) {
        return;
    }
    const std::uint32_t divider_code = std::min(registerValue(0x04) & 0x7U, 6U);
    const std::uint64_t prescaler = std::uint64_t{4} << divider_code;
    const std::uint64_t ticks = static_cast<std::uint64_t>(registerValue(0x08) & 0x0fffU) + 1U;
    const sim::SimTimeNs duration = boundedDuration(
        static_cast<long double>(ticks) * static_cast<long double>(prescaler) * 1000000000.0L / 32000.0L
    );
    static_cast<void>(timeout_event_.scheduleAfter(duration, [this]() {
        traceEvent("watchdog_timeout");
        if (reset_enabled_ && reset_callback_) {
            reset_callback_();
        }
    }));
}

WwdgPeripheral::WwdgPeripheral(
    const bool reset_enabled,
    const std::uint64_t peripheral_clock_hz,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral("WWDG", 0x0c, event_loop, trace),
    reset_enabled_(reset_enabled),
    peripheral_clock_hz_(peripheral_clock_hz == 0 ? 1U : peripheral_clock_hz),
    timeout_event_(event_loop) {
    setResetValue(0x00, 0x7fU);
    setResetValue(0x04, 0x7fU);
    reset();
}

void WwdgPeripheral::setResetEnabled(const bool enabled) {
    reset_enabled_ = enabled;
    switch (state_) {
    case State::stopped:
        return;
    case State::running:
        reload();
        return;
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
        applyControl(value);
    } else if (word_offset == 0x08U) {
        setRegister(0x08, previous & value);
    }
}

void WwdgPeripheral::onReset() {
    state_ = State::stopped;
    cancelTimeout();
}

void WwdgPeripheral::applyControl(const std::uint32_t value) {
    setRegister(0x00, value & 0xffU);
    const bool enabled = (value & (1U << 7U)) != 0U;
    switch (state_) {
    case State::stopped:
        if (!enabled) return;
        state_ = State::running;
        reload();
        return;
    case State::running:
        if (enabled) {
            reload();
            return;
        }
        state_ = State::stopped;
        cancelTimeout();
        return;
    }
}

void WwdgPeripheral::reload() {
    cancelTimeout();
    traceEvent("watchdog_reload");
    switch (state_) {
    case State::stopped:
        return;
    case State::running:
        break;
    }
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
    static_cast<void>(timeout_event_.scheduleAfter(duration, [this]() {
        traceEvent("watchdog_timeout");
        if (reset_enabled_ && reset_callback_) {
            reset_callback_();
        }
    }));
}

} // namespace fil::stm32g4
