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
) : RegisterPeripheral(std::move(name), 0x50, event_loop, trace),
    conversion_event_(event_loop) {
    reset();
}

AdcPeripheral::~AdcPeripheral() = default;

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
        channel_overrides_[channel] = true;
    }
}

void AdcPeripheral::setChannelProvider(ChannelProvider provider) {
    synchronizeLazyConversions();
    certified_provider_ = false;
    channel_provider_ = std::move(provider);
}

void AdcPeripheral::setCertifiedChannelProvider(ChannelProvider provider) {
    synchronizeLazyConversions();
    channel_provider_ = std::move(provider);
    certified_provider_ = true;
    refreshConversionScheduling();
}

void AdcPeripheral::setSampleCallback(SampleCallback callback) {
    synchronizeLazyConversions();
    sample_callback_ = std::move(callback);
    certified_sample_callback_ = false;
    owner_local_guard_ = {};
    refreshConversionScheduling();
}

void AdcPeripheral::setCertifiedSampleCallback(
    SampleCallback callback, std::function<bool()> guard
) {
    synchronizeLazyConversions();
    sample_callback_ = std::move(callback);
    owner_local_guard_ = std::move(guard);
    certified_sample_callback_ = true;
    certified_interrupt_generation_ = interruptLevelCallbackGeneration();
    refreshConversionScheduling();
}

bool AdcPeripheral::ownerLocalTrusted() const {
    return certified_sample_callback_ && certified_provider_ && !interrupt_callback_
        && !traceObserverActive()
        && certified_interrupt_generation_ == interruptLevelCallbackGeneration()
        && owner_local_guard_ && owner_local_guard_();
}

void AdcPeripheral::setInterruptCallback(InterruptCallback callback) {
    synchronizeLazyConversions();
    interrupt_callback_ = std::move(callback);
    certified_sample_callback_ = false;
    setInterruptLevel(0, (registerValue(ier) & registerValue(isr) & (eoc | eos)) != 0U);
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
        setInterruptLevel(0, (registerValue(ier) & registerValue(isr) & (eoc | eos)) != 0U);
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
        setInterruptLevel(0, (registerValue(ier) & registerValue(isr) & (eoc | eos)) != 0U);
        return;
    }
    if (word_offset == ier) {
        setInterruptLevel(0, (value & registerValue(isr) & (eoc | eos)) != 0U);
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
    scan_index_ = 0U;
    skip_scan_ = false;
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
    scan_index_ = 0U;
    skip_scan_ = false;
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
    const bool sequence_complete = sequence_rank_ + 1U >= sequenceLength();
    // Decimation is evaluated at scan granularity so multi-rank DMA
    // alignment is preserved: a skipped scan advances ranks and time
    // exactly as usual but leaves DR/ISR, DMA, interrupts, and sample
    // history untouched. Single-shot conversions always materialize.
    bool keep = true;
    if (continuousMode() && decimation_ > 1U) {
        if (sequence_rank_ == 0U) skip_scan_ = (scan_index_ % decimation_) != 0U;
        keep = !skip_scan_;
    }
    if (keep) materializeConversion(currentTime(), true);
    sequence_rank_ = sequence_complete ? 0U : sequence_rank_ + 1U;
    if (sequence_complete) {
        ++scan_index_;
        beginNextScan();
        return;
    }

    if ((registerValue(cr) & adstart) != 0U) {
        const sim::SimTimeNs delay = conversionDelayForRank(sequence_rank_);
        if (delay > std::numeric_limits<sim::SimTimeNs>::max() - currentTime()) {
            throw std::overflow_error("ADC conversion time overflow");
        }
        armNextConversion(currentTime() + delay);
    } else {
        setRegister(cr, registerValue(cr) & ~adstart);
    }
}

void AdcPeripheral::completeSkippedScans(const unsigned int skipped) {
    sequence_rank_ = 0U;
    scan_index_ += skipped;
    beginNextScan();
}

void AdcPeripheral::beginNextScan() {
    sequence_rank_ = 0U;
    if (!continuousMode() || (registerValue(cr) & adstart) == 0U) {
        setRegister(cr, registerValue(cr) & ~adstart);
        return;
    }
    // Jump the whole decimated gap in one event: scans scan_index_ .. the
    // next kept scan are unobservable while skipped, so their internal
    // event granularity collapses. The span repeats the live scan period;
    // timing rewrites during a gap land at the next kept scan at latest.
    if (decimation_ > 1U) {
        const unsigned int upcoming = static_cast<unsigned int>(
            scan_index_ % decimation_);
        if (upcoming != 0U) {
            skip_scan_ = true;
            armSkippedScans(decimation_ - upcoming);
            return;
        }
    }
    skip_scan_ = false;
    const sim::SimTimeNs delay = conversionDelayForRank(0U);
    if (delay > std::numeric_limits<sim::SimTimeNs>::max() - currentTime()) {
        throw std::overflow_error("ADC conversion time overflow");
    }
    armNextConversion(currentTime() + delay);
}

void AdcPeripheral::armSkippedScans(const unsigned int count) {
    const sim::SimTimeNs now = currentTime();
    const unsigned int length = sequenceLength();
    sim::SimTimeNs period = 0U;
    bool representable = length > 0U && count > 0U;
    for (unsigned int rank = 0U; rank < length && representable; ++rank) {
        const sim::SimTimeNs delay = conversionDelayForRank(rank);
        representable = delay <= std::numeric_limits<sim::SimTimeNs>::max() - period;
        period += delay;
    }
    representable = representable
        && period > 0U
        && count <= std::numeric_limits<sim::SimTimeNs>::max() / period
        && count * period <= std::numeric_limits<sim::SimTimeNs>::max() - now;
    if (!representable || eventLoop() == nullptr) {
        // Degrades to per-rank arming; the rank path already skips the
        // side effects for this scan.
        armNextConversion(now + conversionDelayForRank(0U));
        return;
    }
    next_conversion_ns_ = now + count * period;
    if (lazyConversionEligible()) {
        conversion_event_.cancel();
        return;
    }
    static_cast<void>(conversion_event_.scheduleOwnerLocalAt(next_conversion_ns_.value(), [this, count]() {
        next_conversion_ns_.reset();
        completeSkippedScans(count);
    }, [this]() { return ownerLocalTrusted(); }));
}

void AdcPeripheral::materializeConversion(
    const sim::SimTimeNs completion_time,
    const bool observable
) {
    const unsigned int channel = channelForRank(sequence_rank_);
    const std::uint16_t value = channel_overrides_[channel] || !channel_provider_
        ? channel_values_[channel]
        : channel_provider_(channel, completion_time);
    const AdcSample sample{completion_time, channel, value};
    setRegister(dr, value);
    const bool sequence_complete = sequence_rank_ + 1U >= sequenceLength();
    setRegister(isr, registerValue(isr) | eoc | (sequence_complete ? eos : 0U));
    setInterruptLevel(0, (registerValue(ier) & registerValue(isr) & (eoc | eos)) != 0U);
    if (observable && sample_history_enabled_) samples_.push_back(sample);
    if (observable && traceEnabled() && tracePasses("adc_sample")) {
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
    if (conversion_event_.pending() || !next_conversion_ns_) return;
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
        conversion_event_.cancel();
        return;
    }
    if (!conversion_event_.pending()) scheduleConversionEvent();
}

void AdcPeripheral::armNextConversion(const sim::SimTimeNs completion_time) {
    next_conversion_ns_ = completion_time;
    refreshConversionScheduling();
}

void AdcPeripheral::scheduleConversionEvent() {
    if (eventLoop() == nullptr || !next_conversion_ns_) return;
    static_cast<void>(conversion_event_.scheduleOwnerLocalAt(*next_conversion_ns_, [this]() {
        next_conversion_ns_.reset();
        completeConversion();
    }, [this]() { return ownerLocalTrusted(); }));
}

void AdcPeripheral::cancelConversion() noexcept {
    conversion_event_.cancel();
    next_conversion_ns_.reset();
}

} // namespace fil::stm32g4
