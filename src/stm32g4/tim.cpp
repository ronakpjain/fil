#include "fil/stm32g4/peripheral.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t cr1 = 0x00;
constexpr std::uint32_t dier = 0x0c;
constexpr std::uint32_t sr = 0x10;
constexpr std::uint32_t egr = 0x14;
constexpr std::uint32_t cnt = 0x24;
constexpr std::uint32_t psc = 0x28;
constexpr std::uint32_t arr = 0x2c;

std::uint64_t timerTicks(
    const sim::SimTimeNs duration_ns,
    const std::uint64_t frequency_hz,
    const std::uint64_t prescaler
) noexcept {
    const long double ticks = static_cast<long double>(duration_ns)
        * static_cast<long double>(frequency_hz)
        / (1000000000.0L * static_cast<long double>(prescaler));
    if (ticks >= static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return static_cast<std::uint64_t>(ticks);
}

sim::SimTimeNs durationForTicks(
    const std::uint64_t ticks,
    const std::uint64_t frequency_hz,
    const std::uint64_t prescaler
) noexcept {
    if (frequency_hz == 0) {
        return std::numeric_limits<sim::SimTimeNs>::max();
    }
    const long double duration = std::ceil(
        static_cast<long double>(ticks)
        * static_cast<long double>(prescaler)
        * 1000000000.0L
        / static_cast<long double>(frequency_hz)
    );
    if (duration >= static_cast<long double>(std::numeric_limits<sim::SimTimeNs>::max())) {
        return std::numeric_limits<sim::SimTimeNs>::max();
    }
    return std::max<sim::SimTimeNs>(static_cast<sim::SimTimeNs>(duration), 1U);
}

} // namespace

TimerPeripheral::TimerPeripheral(
    std::string name,
    const std::uint64_t input_clock_hz,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral(std::move(name), 0x50, event_loop, trace),
    input_clock_hz_(input_clock_hz == 0 ? 1U : input_clock_hz) {
    setResetValue(arr, 0xffffU);
    reset();
}

TimerPeripheral::~TimerPeripheral() {
    cancelUpdate();
}

void TimerPeripheral::setInputClockHz(const std::uint64_t frequency_hz) {
    if (frequency_hz == 0) {
        throw std::invalid_argument("timer input clock must be nonzero");
    }
    captureCounter();
    input_clock_hz_ = frequency_hz;
    scheduleUpdate();
}

void TimerPeripheral::setInterruptCallback(InterruptCallback callback) {
    interrupt_callback_ = std::move(callback);
}

void TimerPeripheral::setUpdateCallback(UpdateCallback callback) {
    update_callback_ = std::move(callback);
}

std::uint32_t TimerPeripheral::loadRegister(
    const std::uint32_t word_offset,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    if (word_offset == cnt) {
        const std::uint32_t value = liveCounter();
        setRegister(cnt, value);
        return value;
    }
    return registerValue(word_offset);
}

void TimerPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    if (word_offset == cr1 || word_offset == psc || word_offset == arr || word_offset == cnt) {
        // Capture with the old timing register, then install the just-written value.
        setRegister(word_offset, previous);
        captureCounter();
        setRegister(word_offset, value);
        if (word_offset == cnt) {
            counter_epoch_value_ = value;
        }
        counter_epoch_ns_ = currentTime();
        scheduleUpdate();
    } else if (word_offset == sr) {
        setRegister(sr, previous & (value | ~write_mask));
    } else if (word_offset == egr) {
        const bool generate_update = (value & write_mask & 1U) != 0U;
        setRegister(egr, 0);
        if (generate_update) {
            fireUpdate(true);
        }
    }
}

void TimerPeripheral::onReset() {
    cancelUpdate();
    counter_epoch_ns_ = currentTime();
    counter_epoch_value_ = registerValue(cnt);
}

void TimerPeripheral::captureCounter() {
    counter_epoch_value_ = liveCounter();
    counter_epoch_ns_ = currentTime();
    setRegister(cnt, counter_epoch_value_);
}

std::uint32_t TimerPeripheral::liveCounter() const noexcept {
    if ((registerValue(cr1) & 1U) == 0U || currentTime() <= counter_epoch_ns_) {
        return counter_epoch_value_;
    }
    const std::uint64_t period = static_cast<std::uint64_t>(registerValue(arr)) + 1U;
    const std::uint64_t prescaler = static_cast<std::uint64_t>(registerValue(psc)) + 1U;
    const std::uint64_t ticks = timerTicks(currentTime() - counter_epoch_ns_, input_clock_hz_, prescaler);
    const std::uint64_t position = (static_cast<std::uint64_t>(counter_epoch_value_) + (ticks % period)) % period;
    return static_cast<std::uint32_t>(position);
}

void TimerPeripheral::scheduleUpdate() {
    cancelUpdate();
    if (eventLoop() == nullptr || (registerValue(cr1) & 1U) == 0U) {
        return;
    }
    const std::uint64_t period = static_cast<std::uint64_t>(registerValue(arr)) + 1U;
    const std::uint64_t current = liveCounter();
    const std::uint64_t remaining = period - std::min(current, period - 1U);
    const std::uint64_t prescaler = static_cast<std::uint64_t>(registerValue(psc)) + 1U;
    const sim::SimTimeNs delay = durationForTicks(remaining, input_clock_hz_, prescaler);
    update_event_ = eventLoop()->scheduleAfter(delay, [this]() {
        update_event_ = 0;
        fireUpdate(false);
    });
}

void TimerPeripheral::cancelUpdate() noexcept {
    if (update_event_ != 0 && eventLoop() != nullptr) {
        static_cast<void>(eventLoop()->cancel(update_event_));
    }
    update_event_ = 0;
}

void TimerPeripheral::fireUpdate(const bool forced) {
    const std::uint32_t before = forced ? liveCounter() : registerValue(arr);
    counter_epoch_ns_ = currentTime();
    counter_epoch_value_ = 0;
    setRegister(cnt, 0);
    setRegister(sr, registerValue(sr) | 1U);
    if (update_history_enabled_) updates_.push_back(TimerUpdate{currentTime(), before});
    traceEvent("timer_update", {{"forced", forced ? "true" : "false"}});
    if ((registerValue(dier) & 1U) != 0U && interrupt_callback_) {
        interrupt_callback_();
    }
    if (update_callback_) {
        update_callback_(currentTime());
    }
    if ((registerValue(cr1) & (1U << 3U)) != 0U) {
        setRegister(cr1, registerValue(cr1) & ~1U);
    }
    scheduleUpdate();
}

} // namespace fil::stm32g4
