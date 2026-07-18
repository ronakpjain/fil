#include "fil/stm32g4/peripheral.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t isr = 0x00;
constexpr std::uint32_t ier = 0x04;
constexpr std::uint32_t cr = 0x08;
constexpr std::uint32_t cfgr = 0x0c;
constexpr std::uint32_t smpr1 = 0x14;
constexpr std::uint32_t smpr2 = 0x18;
constexpr std::uint32_t sqr1 = 0x30;
constexpr std::uint32_t sqr2 = 0x34;
constexpr std::uint32_t sqr3 = 0x38;
constexpr std::uint32_t sqr4 = 0x3c;
constexpr std::uint32_t dr = 0x40;

constexpr std::uint32_t adrdy = 1U << 0U;
constexpr std::uint32_t eoc = 1U << 2U;
constexpr std::uint32_t eos = 1U << 3U;
constexpr std::uint32_t aden = 1U << 0U;
constexpr std::uint32_t addis = 1U << 1U;
constexpr std::uint32_t adstart = 1U << 2U;
constexpr std::uint32_t adcal = 1U << 31U;
constexpr std::uint32_t continuous = 1U << 13U;

} // namespace

AdcPeripheral::AdcPeripheral(
    std::string name,
    sim::EventLoop* const event_loop,
    sim::TraceRecorder* const trace
) : RegisterPeripheral(std::move(name), 0x50, event_loop, trace) {
    reset();
}

AdcPeripheral::~AdcPeripheral() {
    cancelConversion();
}

mem::MemoryResult<std::uint64_t> AdcPeripheral::read(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const mem::AccessContext& context
) {
    synchronizeLazyConversions();
    return RegisterPeripheral::read(offset, size, context);
}

mem::MemoryResult<std::uint64_t> AdcPeripheral::write(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const std::uint64_t value,
    const mem::AccessContext& context
) {
    synchronizeLazyConversions();
    auto result = RegisterPeripheral::write(offset, size, value, context);
    if (result) refreshConversionScheduling();
    return result;
}

void AdcPeripheral::setChannelValue(const unsigned int channel, const std::uint16_t value) {
    synchronizeLazyConversions();
    if (channel < channel_values_.size()) {
        channel_values_[channel] = static_cast<std::uint16_t>(std::min<std::uint16_t>(value, 0x0fffU));
    }
}

void AdcPeripheral::overrideChannelValue(const unsigned int channel, const std::uint16_t value) {
    synchronizeLazyConversions();
    if (channel < channel_overrides_.size()) {
        channel_overrides_[channel] = static_cast<std::uint16_t>(std::min<std::uint16_t>(value, 0x0fffU));
    }
}

void AdcPeripheral::setChannelProvider(ChannelProvider provider) {
    synchronizeLazyConversions();
    channel_provider_ = std::move(provider);
}

void AdcPeripheral::setSampleCallback(SampleCallback callback) {
    synchronizeLazyConversions();
    sample_callback_ = std::move(callback);
    refreshConversionScheduling();
}

void AdcPeripheral::setInterruptCallback(InterruptCallback callback) {
    synchronizeLazyConversions();
    interrupt_callback_ = std::move(callback);
    refreshConversionScheduling();
}

void AdcPeripheral::setInputClockHz(const std::uint64_t frequency_hz) {
    synchronizeLazyConversions();
    input_clock_hz_ = std::max<std::uint64_t>(frequency_hz, 1U);
}

void AdcPeripheral::setSampleHistoryEnabled(const bool enabled) {
    // Synchronize before changing observability so elapsed unobservable samples
    // update DR/ISR once without being retroactively added to diagnostics.
    synchronizeLazyConversions();
    sample_history_enabled_ = enabled;
    refreshConversionScheduling();
}

std::uint32_t AdcPeripheral::loadRegister(
    const std::uint32_t word_offset,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    const std::uint32_t value = registerValue(word_offset);
    if (word_offset == dr) {
        setRegister(isr, registerValue(isr) & ~(eoc | eos));
    }
    return value;
}

void AdcPeripheral::storeRegister(
    const std::uint32_t word_offset,
    const std::uint32_t previous,
    const std::uint32_t value,
    const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(context);
    if (word_offset == isr) {
        setRegister(isr, previous & ~(value & write_mask));
        return;
    }
    if (word_offset == dr) {
        setRegister(dr, previous);
        return;
    }
    if (word_offset != cr) {
        return;
    }

    std::uint32_t control = value;
    if ((control & adcal) != 0U) {
        control &= ~adcal; // Calibration completes immediately.
        traceEvent("adc_calibrated");
    }
    if ((control & addis) != 0U) {
        control &= ~(addis | aden | adstart);
        setRegister(isr, registerValue(isr) & ~adrdy);
        cancelConversion();
    }
    if ((control & aden) != 0U) {
        setRegister(isr, registerValue(isr) | adrdy);
    }
    setRegister(cr, control);
    if ((control & adstart) != 0U) {
        startConversion();
    }
}

void AdcPeripheral::onReset() {
    cancelConversion();
    sequence_rank_ = 0U;
}

unsigned int AdcPeripheral::sequenceLength() const noexcept {
    return std::min<unsigned int>((registerValue(sqr1) & 0x0fU) + 1U, 16U);
}

unsigned int AdcPeripheral::channelForRank(const unsigned int rank) const noexcept {
    std::uint32_t sequence_register = sqr1;
    unsigned int shift = 6U;
    if (rank >= 14U) {
        sequence_register = sqr4;
        shift = 6U * (rank - 14U);
    } else if (rank >= 9U) {
        sequence_register = sqr3;
        shift = 6U * (rank - 9U);
    } else if (rank >= 4U) {
        sequence_register = sqr2;
        shift = 6U * (rank - 4U);
    } else {
        shift = 6U + 6U * rank;
    }
    const unsigned int channel = (registerValue(sequence_register) >> shift) & 0x1fU;
    return channel < channel_values_.size() ? channel : 0U;
}

sim::SimTimeNs AdcPeripheral::conversionDelayForRank(const unsigned int rank) const noexcept {
    if (conversion_delay_override_ns_ != 0U) return conversion_delay_override_ns_;
    const unsigned int channel = channelForRank(rank);
    const std::uint32_t sample_register = channel <= 9U ? smpr1 : smpr2;
    const unsigned int sample_shift = 3U * (channel <= 9U ? channel : channel - 10U);
    const unsigned int sample_selector =
        (registerValue(sample_register) >> sample_shift) & 0x7U;
    static constexpr std::array<std::uint16_t, 8> sample_half_cycles{
        5U, 13U, 25U, 49U, 95U, 185U, 495U, 1281U,
    };
    static constexpr std::array<std::uint8_t, 4> conversion_half_cycles{
        25U, 21U, 17U, 13U,
    };
    const unsigned int resolution = (registerValue(cfgr) >> 3U) & 0x3U;
    const std::uint64_t half_cycles =
        sample_half_cycles[sample_selector] + conversion_half_cycles[resolution];
    const std::uint64_t denominator = 2U * std::max<std::uint64_t>(input_clock_hz_, 1U);
    if (half_cycles > std::numeric_limits<std::uint64_t>::max() / 1'000'000'000ULL) {
        return std::numeric_limits<sim::SimTimeNs>::max();
    }
    return std::max<sim::SimTimeNs>(
        (half_cycles * 1'000'000'000ULL + denominator - 1U) / denominator, 1U
    );
}

bool AdcPeripheral::continuousMode() const noexcept {
    return (registerValue(cfgr) & continuous) != 0U;
}

bool AdcPeripheral::conversionObservable() const noexcept {
    const bool conversion_interrupt_enabled = (registerValue(ier) & (eoc | eos)) != 0U;
    return conversion_interrupt_enabled
        || static_cast<bool>(sample_callback_)
        || traceEnabled()
        || sample_history_enabled_;
}

bool AdcPeripheral::lazyConversionEligible() const noexcept {
    return eventLoop() != nullptr
        && sequenceLength() == 1U
        && continuousMode()
        && (registerValue(cr) & adstart) != 0U
        && !conversionObservable();
}

void AdcPeripheral::startConversion() {
    cancelConversion();
    sequence_rank_ = 0U;
    if (eventLoop() == nullptr) {
        materializeConversion(currentTime(), true);
        setRegister(cr, registerValue(cr) & ~adstart);
        return;
    }
    const sim::SimTimeNs delay = conversionDelayForRank(sequence_rank_);
    if (delay > std::numeric_limits<sim::SimTimeNs>::max() - currentTime()) {
        throw std::overflow_error("ADC conversion time overflow");
    }
    armNextConversion(currentTime() + delay);
}

void AdcPeripheral::completeConversion() {
    materializeConversion(currentTime(), true);
    const bool sequence_complete = sequence_rank_ + 1U >= sequenceLength();
    sequence_rank_ = sequence_complete ? 0U : sequence_rank_ + 1U;

    if ((!sequence_complete || continuousMode()) && (registerValue(cr) & adstart) != 0U) {
        const sim::SimTimeNs delay = conversionDelayForRank(sequence_rank_);
        if (delay > std::numeric_limits<sim::SimTimeNs>::max() - currentTime()) {
            throw std::overflow_error("ADC conversion time overflow");
        }
        armNextConversion(currentTime() + delay);
    } else {
        setRegister(cr, registerValue(cr) & ~adstart);
    }
}

void AdcPeripheral::materializeConversion(
    const sim::SimTimeNs completion_time,
    const bool observable
) {
    const unsigned int channel = channelForRank(sequence_rank_);
    const std::uint16_t value = channel_overrides_[channel].value_or(
        channel_provider_ ? channel_provider_(channel, completion_time) : channel_values_[channel]
    );
    const AdcSample sample{completion_time, channel, value};
    setRegister(dr, value);
    const bool sequence_complete = sequence_rank_ + 1U >= sequenceLength();
    setRegister(isr, registerValue(isr) | eoc | (sequence_complete ? eos : 0U));
    if (observable && sample_history_enabled_) samples_.push_back(sample);
    if (observable && traceEnabled()) {
        traceEvent("adc_sample", {
            {"channel", std::to_string(channel)},
            {"value", std::to_string(value)},
        });
    }
    if (observable && sample_callback_) {
        sample_callback_(sample);
    }
    const std::uint32_t enabled_interrupts = registerValue(ier);
    if (observable && interrupt_callback_
        && ((enabled_interrupts & eoc) != 0U || (enabled_interrupts & eos) != 0U)) {
        interrupt_callback_();
    }
}

void AdcPeripheral::synchronizeLazyConversions() {
    if (conversion_event_ != 0U || !next_conversion_ns_) return;
    const sim::SimTimeNs now = currentTime();
    if (now < *next_conversion_ns_) return;

    const sim::SimTimeNs delay = conversionDelayForRank(0U);
    if (delay == 0U) return;
    const sim::SimTimeNs elapsed = now - *next_conversion_ns_;
    const sim::SimTimeNs whole_periods = elapsed / delay;
    const sim::SimTimeNs latest = *next_conversion_ns_ + whole_periods * delay;
    materializeConversion(latest, false);

    if (latest > std::numeric_limits<sim::SimTimeNs>::max() - delay) {
        next_conversion_ns_.reset();
    } else {
        next_conversion_ns_ = latest + delay;
    }
}

void AdcPeripheral::refreshConversionScheduling() {
    if (!next_conversion_ns_) return;
    if (lazyConversionEligible()) {
        if (conversion_event_ != 0U && eventLoop() != nullptr) {
            static_cast<void>(eventLoop()->cancel(conversion_event_));
            conversion_event_ = 0;
        }
        return;
    }
    if (conversion_event_ == 0U) scheduleConversionEvent();
}

void AdcPeripheral::armNextConversion(const sim::SimTimeNs completion_time) {
    next_conversion_ns_ = completion_time;
    refreshConversionScheduling();
}

void AdcPeripheral::scheduleConversionEvent() {
    if (eventLoop() == nullptr || !next_conversion_ns_) return;
    conversion_event_ = eventLoop()->scheduleAt(*next_conversion_ns_, [this]() {
        conversion_event_ = 0;
        next_conversion_ns_.reset();
        completeConversion();
    });
}

void AdcPeripheral::cancelConversion() noexcept {
    if (conversion_event_ != 0 && eventLoop() != nullptr) {
        static_cast<void>(eventLoop()->cancel(conversion_event_));
    }
    conversion_event_ = 0;
    next_conversion_ns_.reset();
}

} // namespace fil::stm32g4
