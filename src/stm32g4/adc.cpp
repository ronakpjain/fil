#include "fil/stm32g4/peripheral.hpp"

#include <algorithm>
#include <cstdint>
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

AdcCommonPeripheral::AdcCommonPeripheral(
    std::string name, std::vector<AdcPeripheral*> members,
    sim::EventLoop* const event_loop, sim::TraceRecorder* const trace
) : RegisterPeripheral(std::move(name), 0x100, event_loop, trace), members_(std::move(members)) {
    reset();
}

void AdcCommonPeripheral::setSystemClockHz(const std::uint64_t frequency_hz) {
    // Legacy standalone helper keeps the prior test-facing behavior explicitly.
    setClockInputs(frequency_hz, frequency_hz);
}

void AdcCommonPeripheral::setClockInputs(
    const std::uint64_t hclk_hz, const std::uint64_t async_kernel_hz,
    const bool clock_enabled, const bool hclk_div1_allowed
) {
    hclk_hz_ = hclk_hz;
    async_kernel_hz_ = async_kernel_hz;
    clock_enabled_ = clock_enabled;
    hclk_div1_allowed_ = hclk_div1_allowed;
    updateMemberClocks();
}

void AdcCommonPeripheral::setGroupReset(const bool asserted) {
    if (reset_asserted_ == asserted) return;
    reset_asserted_ = asserted;
    if (asserted) RegisterPeripheral::reset();
    for (AdcPeripheral* member : members_) {
        if (member != nullptr) member->setResetHeld(asserted);
    }
    updateMemberClocks();
}

void AdcCommonPeripheral::storeRegister(
    const std::uint32_t word_offset, const std::uint32_t previous,
    const std::uint32_t value, const std::uint32_t write_mask,
    const mem::AccessContext& context
) {
    static_cast<void>(write_mask);
    static_cast<void>(context);
    if (reset_asserted_) {
        setRegister(word_offset, previous);
        return;
    }
    if (word_offset != 0x08U) return;
    constexpr std::uint32_t clock_fields = (3U << 16U) | (0x0fU << 18U);
    constexpr std::uint32_t active = (1U << 0U) | (1U << 1U) | (1U << 2U)
        | (1U << 3U) | (1U << 4U) | (1U << 31U);
    const bool member_enabled = std::any_of(members_.begin(), members_.end(),
        [](const AdcPeripheral* member) {
            return member != nullptr && (member->peekRegister(0x08U) & active) != 0U;
        });
    if (member_enabled) {
        setRegister(word_offset, (value & ~clock_fields) | (previous & clock_fields));
    }
    updateMemberClocks();
}

void AdcCommonPeripheral::onReset() { updateMemberClocks(); }

void AdcCommonPeripheral::updateMemberClocks() {
    const std::uint32_t ccr = registerValue(0x08U);
    const std::uint32_t mode = (ccr >> 16U) & 0x3U;
    static constexpr std::uint16_t async_dividers[16]{1,2,4,6,8,10,12,16,32,64,128,256,0,0,0,0};
    const std::uint32_t presc = (ccr >> 18U) & 0x0fU;
    const std::uint64_t async_clock = presc < 12U
        ? async_kernel_hz_ / async_dividers[presc] : 0U;
    const std::uint64_t synchronous_clock = mode == 1U
        ? (hclk_div1_allowed_ ? hclk_hz_ : 0U)
        : mode == 2U ? hclk_hz_ / 2U : mode == 3U ? hclk_hz_ / 4U : async_clock;
    const std::uint64_t clock = reset_asserted_ ? 0U : mode == 0U ? async_clock
        : clock_enabled_ ? synchronous_clock : 0U;
    for (AdcPeripheral* member : members_) {
        if (member != nullptr) member->setInputClockHz(clock);
    }
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

void AdcPeripheral::setChannelValue(const std::uint32_t channel, const std::uint16_t value) {
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

void AdcPeripheral::setResetHeld(const bool asserted) {
    if (reset_held_ == asserted) return;
    reset_held_ = asserted;
    if (asserted) RegisterPeripheral::reset();
}

void AdcPeripheral::setInputClockHz(const std::uint64_t frequency_hz) {
    const std::uint64_t next_hz = frequency_hz;
    if (next_hz == input_clock_hz_) return;

    synchronizeLazyConversions();
    if (next_hz == 0U) {
        if (next_conversion_ns_) {
            const sim::SimTimeNs now = currentTime();
            const sim::SimTimeNs remaining_ns = *next_conversion_ns_ > now
                ? *next_conversion_ns_ - now : 0U;
            if (conversion_delay_override_ns_ != 0U) {
                suspended_override_ns_ = remaining_ns;
            } else {
                if (remaining_ns > std::numeric_limits<std::uint64_t>::max() / input_clock_hz_) {
                    throw std::overflow_error("ADC suspended-cycle count overflow");
                }
                suspended_conversion_cycles_ = remaining_ns * input_clock_hz_;
            }
        }
        conversion_event_.cancel();
        next_conversion_ns_.reset();
        input_clock_hz_ = 0U;
        return;
    }
    const std::uint64_t previous_hz = input_clock_hz_;
    const sim::SimTimeNs now = currentTime();
    const bool event_pending = conversion_event_.pending();
    std::optional<sim::SimTimeNs> rescheduled_time;
    if (suspended_override_ns_ != 0U) {
        if (suspended_override_ns_ > std::numeric_limits<sim::SimTimeNs>::max() - now)
            throw std::overflow_error("ADC conversion time overflow during override resume");
        rescheduled_time = now + suspended_override_ns_;
        suspended_override_ns_ = 0U;
    } else if (suspended_conversion_cycles_ != 0U) {
        sim::SimTimeNs delay = suspended_conversion_cycles_ / next_hz;
        if (suspended_conversion_cycles_ % next_hz != 0U) ++delay;
        delay = std::max<sim::SimTimeNs>(delay, 1U);
        if (delay > std::numeric_limits<sim::SimTimeNs>::max() - now)
            throw std::overflow_error("ADC conversion time overflow during clock resume");
        rescheduled_time = now + delay;
        suspended_conversion_cycles_ = 0U;
    } else if (next_conversion_ns_ && conversion_delay_override_ns_ == 0U) {
        const sim::SimTimeNs remaining_ns = *next_conversion_ns_ > now
            ? *next_conversion_ns_ - now : 0U;
        // Integer checked arithmetic avoids an out-of-range float-to-time cast.
        // Overflow is a deterministic configuration/runtime error; no event or
        // clock state is mutated until the replacement deadline is representable.
        if (remaining_ns > std::numeric_limits<sim::SimTimeNs>::max() / previous_hz) {
            throw std::overflow_error("ADC clock reschedule multiplication overflow");
        }
        const std::uint64_t cycle_time_product = remaining_ns * previous_hz;
        sim::SimTimeNs delay = cycle_time_product / next_hz;
        if (cycle_time_product % next_hz != 0U) {
            if (delay == std::numeric_limits<sim::SimTimeNs>::max()) {
                throw std::overflow_error("ADC clock reschedule rounding overflow");
            }
            ++delay;
        }
        delay = std::max<sim::SimTimeNs>(delay, 1U);
        if (delay > std::numeric_limits<sim::SimTimeNs>::max() - now) {
            throw std::overflow_error("ADC conversion time overflow during clock change");
        }
        rescheduled_time = now + delay;
    }

    if (event_pending) conversion_event_.cancel();
    input_clock_hz_ = next_hz;
    if (rescheduled_time) next_conversion_ns_ = *rescheduled_time;
    if (!next_conversion_ns_ && (registerValue(cr) & adstart) != 0U) {
        const sim::SimTimeNs delay = conversionDelayForRank(sequence_rank_);
        if (delay <= std::numeric_limits<sim::SimTimeNs>::max() - now)
            next_conversion_ns_ = now + delay;
    }
    if (event_pending || rescheduled_time || next_conversion_ns_) refreshConversionScheduling();
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
    if (reset_held_) {
        setRegister(word_offset, previous);
        return;
    }
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
    samples_.clear();
    sequence_rank_ = 0U;
    scan_index_ = 0U;
    skip_scan_ = false;
}

std::uint32_t AdcPeripheral::sequenceLength() const noexcept {
    return std::min<std::uint32_t>((registerValue(sqr1) & 0x0fU) + 1U, 16U);
}

std::uint32_t AdcPeripheral::channelForRank(const std::uint32_t rank) const noexcept {
    std::uint32_t sequence_register = sqr1;
    std::uint32_t shift = 6U;
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
    const std::uint32_t channel = (registerValue(sequence_register) >> shift) & 0x1fU;
    return channel < channel_values_.size() ? channel : 0U;
}

std::uint64_t AdcPeripheral::conversionCycleBudget(const std::uint32_t rank) const noexcept {
    const std::uint32_t channel = channelForRank(rank);
    const std::uint32_t sample_register = channel <= 9U ? smpr1 : smpr2;
    const std::uint32_t sample_shift = 3U * (channel <= 9U ? channel : channel - 10U);
    const std::uint32_t sample_selector =
        (registerValue(sample_register) >> sample_shift) & 0x7U;
    static constexpr std::array<std::uint16_t, 8> sample_half_cycles{
        5U, 13U, 25U, 49U, 95U, 185U, 495U, 1281U,
    };
    static constexpr std::array<std::uint8_t, 4> conversion_half_cycles{
        25U, 21U, 17U, 13U,
    };
    const std::uint32_t resolution = (registerValue(cfgr) >> 3U) & 0x3U;
    const std::uint64_t half_cycles =
        sample_half_cycles[sample_selector] + conversion_half_cycles[resolution];
    return half_cycles * 500'000'000ULL;
}

sim::SimTimeNs AdcPeripheral::conversionDelayForRank(const std::uint32_t rank) const noexcept {
    if (conversion_delay_override_ns_ != 0U) return conversion_delay_override_ns_;
    if (input_clock_hz_ == 0U) return std::numeric_limits<sim::SimTimeNs>::max();
    const std::uint64_t budget = conversionCycleBudget(rank);
    return std::max<sim::SimTimeNs>((budget + input_clock_hz_ - 1U) / input_clock_hz_, 1U);
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
    if (input_clock_hz_ == 0U) {
        if (conversion_delay_override_ns_ != 0U) suspended_override_ns_ = conversion_delay_override_ns_;
        else suspended_conversion_cycles_ = conversionCycleBudget(sequence_rank_);
        return;
    }
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

    if (input_clock_hz_ == 0U) return;
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

void AdcPeripheral::completeSkippedScans(const std::uint32_t skipped) {
    skipped_scan_event_ = false;
    skipped_scan_count_ = 0U;
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
    if (input_clock_hz_ == 0U) return;
    // Jump the whole decimated gap in one event: scans scan_index_ .. the
    // next kept scan are unobservable while skipped, so their internal
    // event granularity collapses. The span repeats the live scan period;
    // timing rewrites during a gap land at the next kept scan at latest.
    if (decimation_ > 1U) {
        const std::uint32_t upcoming = static_cast<std::uint32_t>(
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

void AdcPeripheral::armSkippedScans(const std::uint32_t count) {
    const sim::SimTimeNs now = currentTime();
    const std::uint32_t length = sequenceLength();
    sim::SimTimeNs period = 0U;
    bool representable = length > 0U && count > 0U;
    for (std::uint32_t rank = 0U; rank < length && representable; ++rank) {
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
    skipped_scan_event_ = true;
    skipped_scan_count_ = count;
    refreshConversionScheduling();
}

void AdcPeripheral::materializeConversion(
    const sim::SimTimeNs completion_time,
    const bool observable
) {
    const std::uint32_t channel = channelForRank(sequence_rank_);
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
    if (skipped_scan_event_) {
        const std::uint32_t skipped = skipped_scan_count_;
        next_conversion_ns_.reset();
        completeSkippedScans(skipped);
        return;
    }

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
    if (skipped_scan_event_) {
        const std::uint32_t skipped = skipped_scan_count_;
        static_cast<void>(conversion_event_.scheduleOwnerLocalAt(*next_conversion_ns_, [this, skipped]() {
            next_conversion_ns_.reset();
            completeSkippedScans(skipped);
        }, [this]() { return ownerLocalTrusted(); }));
        return;
    }
    static_cast<void>(conversion_event_.scheduleOwnerLocalAt(*next_conversion_ns_, [this]() {
        next_conversion_ns_.reset();
        completeConversion();
    }, [this]() { return ownerLocalTrusted(); }));
}

void AdcPeripheral::cancelConversion() noexcept {
    conversion_event_.cancel();
    suspended_conversion_cycles_ = 0U;
    suspended_override_ns_ = 0U;
    next_conversion_ns_.reset();
    skipped_scan_event_ = false;
    skipped_scan_count_ = 0U;
}

} // namespace fil::stm32g4
