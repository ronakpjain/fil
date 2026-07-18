#pragma once

/** @file peripheral.hpp
 *  @brief Deterministic STM32G4 register-backed peripheral foundations.
 */

#include "fil/mem/memory_bus.hpp"
#include "fil/sim/event_loop.hpp"
#include "fil/sim/trace.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace fil::stm32g4 {

/**
 * @brief Reusable little-endian 32-bit register-block MMIO implementation.
 *
 * Accesses of every MemoryBus width are merged at byte granularity. Derived
 * peripherals override per-word hooks only for registers with side effects.
 */
class RegisterPeripheral : public mem::MmioDevice {
public:
    RegisterPeripheral(
        std::string name,
        std::uint32_t register_block_size,
        sim::EventLoop* event_loop = nullptr,
        sim::TraceRecorder* trace = nullptr
    );
    ~RegisterPeripheral() override;

    RegisterPeripheral(const RegisterPeripheral&) = delete;
    RegisterPeripheral& operator=(const RegisterPeripheral&) = delete;
    RegisterPeripheral(RegisterPeripheral&&) = delete;
    RegisterPeripheral& operator=(RegisterPeripheral&&) = delete;

    [[nodiscard]] mem::MemoryResult<std::uint64_t> read(
        std::uint32_t offset,
        mem::AccessSize size,
        const mem::AccessContext& context
    ) override;

    [[nodiscard]] mem::MemoryResult<std::uint64_t> write(
        std::uint32_t offset,
        mem::AccessSize size,
        std::uint64_t value,
        const mem::AccessContext& context
    ) override;

    [[nodiscard]] std::string_view name() const noexcept override { return name_; }
    [[nodiscard]] std::uint32_t size() const noexcept { return block_size_; }

    /** @brief Restores every register to its configured reset value. */
    void reset();

    /** @brief Reads stored state without invoking device side effects. */
    [[nodiscard]] std::uint32_t peekRegister(std::uint32_t word_offset) const noexcept;

    /** @brief Replaces the recorder used by future side-effect traces. */
    void setTraceRecorder(sim::TraceRecorder* trace) noexcept { trace_ = trace; }

    /** @brief Qualifies future trace sources as `prefix.device`; empty restores the device name. */
    void setTraceSourcePrefix(std::string_view prefix);

    /** @brief Gets the fully qualified source used by future trace events. */
    [[nodiscard]] std::string_view traceSource() const noexcept { return trace_source_; }

protected:
    [[nodiscard]] virtual std::uint32_t loadRegister(
        std::uint32_t word_offset,
        const mem::AccessContext& context
    );

    virtual void storeRegister(
        std::uint32_t word_offset,
        std::uint32_t previous,
        std::uint32_t value,
        std::uint32_t write_mask,
        const mem::AccessContext& context
    );

    virtual void onReset();

    void setResetValue(std::uint32_t word_offset, std::uint32_t value) noexcept;
    void setRegister(std::uint32_t word_offset, std::uint32_t value) noexcept;
    [[nodiscard]] std::uint32_t registerValue(std::uint32_t word_offset) const noexcept;
    [[nodiscard]] sim::SimTimeNs currentTime() const noexcept;
    [[nodiscard]] sim::EventLoop* eventLoop() const noexcept { return event_loop_; }
    [[nodiscard]] bool traceEnabled() const noexcept {
        return trace_ != nullptr && trace_->enabled();
    }

    void traceEvent(std::string type, std::vector<sim::TraceField> fields = {});

private:
    [[nodiscard]] mem::BusFault accessFault(
        std::uint32_t offset,
        mem::AccessSize size,
        const mem::AccessContext& context,
        std::string message
    ) const;

    std::string name_;
    std::string trace_source_;
    std::uint32_t block_size_{0};
    std::vector<std::uint32_t> registers_;
    std::vector<std::uint32_t> reset_values_;
    sim::EventLoop* event_loop_{nullptr};
    sim::TraceRecorder* trace_{nullptr};
};

/** @brief One observed access handled by a lenient unknown MMIO device. */
struct PeripheralAccess {
    sim::SimTimeNs time_ns{0};
    bool write{false};
    std::uint32_t offset{0};
    mem::AccessSize size{mem::AccessSize::byte};
    std::uint64_t value{0};
    std::uint32_t pc{0};
};

/** @brief Sparse, optionally strict fallback that preserves unknown writes. */
class UnknownMmioDevice final : public mem::MmioDevice {
public:
    explicit UnknownMmioDevice(
        std::string name = "unknown-mmio",
        std::uint32_t absolute_base = 0,
        bool strict = false,
        std::uint64_t default_read_value = 0,
        sim::EventLoop* event_loop = nullptr,
        sim::TraceRecorder* trace = nullptr
    );

    [[nodiscard]] mem::MemoryResult<std::uint64_t> read(
        std::uint32_t offset,
        mem::AccessSize size,
        const mem::AccessContext& context
    ) override;
    [[nodiscard]] mem::MemoryResult<std::uint64_t> write(
        std::uint32_t offset,
        mem::AccessSize size,
        std::uint64_t value,
        const mem::AccessContext& context
    ) override;
    [[nodiscard]] std::string_view name() const noexcept override { return name_; }

    void setStrict(bool strict) noexcept { strict_ = strict; }
    [[nodiscard]] bool strict() const noexcept { return strict_; }
    [[nodiscard]] const std::vector<PeripheralAccess>& accesses() const noexcept { return accesses_; }
    /** @brief Qualifies future trace sources as `prefix.device`; empty restores the device name. */
    void setTraceSourcePrefix(std::string_view prefix);
    void clear();

private:
    [[nodiscard]] mem::BusFault fault(
        std::uint32_t offset,
        mem::AccessSize size,
        const mem::AccessContext& context,
        std::string message
    ) const;
    [[nodiscard]] sim::SimTimeNs currentTime() const noexcept;
    void traceAccess(const PeripheralAccess& access);

    std::string name_;
    std::string trace_source_;
    std::uint32_t absolute_base_{0};
    bool strict_{false};
    std::uint64_t default_read_value_{0};
    sim::EventLoop* event_loop_{nullptr};
    sim::TraceRecorder* trace_{nullptr};
    std::map<std::uint32_t, std::uint8_t> bytes_;
    std::vector<PeripheralAccess> accesses_;
};

/** @brief STM32G4 reset-and-clock-control startup model. */
class RccPeripheral final : public RegisterPeripheral {
public:
    explicit RccPeripheral(
        bool hse_present = true,
        std::uint64_t hse_hz = 8000000,
        sim::EventLoop* event_loop = nullptr,
        sim::TraceRecorder* trace = nullptr
    );

    [[nodiscard]] std::uint64_t systemClockHz() const noexcept { return system_clock_hz_; }
    void setClockChangedCallback(std::function<void(std::uint64_t)> callback);

protected:
    void storeRegister(
        std::uint32_t word_offset,
        std::uint32_t previous,
        std::uint32_t value,
        std::uint32_t write_mask,
        const mem::AccessContext& context
    ) override;
    void onReset() override;

private:
    void updateClockReadyBits();
    void updateSystemClock();
    [[nodiscard]] std::uint64_t pllClockHz() const noexcept;

    bool hse_present_{true};
    std::uint64_t hse_hz_{8000000};
    std::uint64_t system_clock_hz_{16000000};
    std::function<void(std::uint64_t)> clock_changed_;
};

/** @brief STM32G4 FLASH control-register model with key-based lock state. */
class FlashPeripheral final : public RegisterPeripheral {
public:
    explicit FlashPeripheral(sim::EventLoop* event_loop = nullptr, sim::TraceRecorder* trace = nullptr);

protected:
    [[nodiscard]] std::uint32_t loadRegister(
        std::uint32_t word_offset,
        const mem::AccessContext& context
    ) override;
    void storeRegister(
        std::uint32_t word_offset,
        std::uint32_t previous,
        std::uint32_t value,
        std::uint32_t write_mask,
        const mem::AccessContext& context
    ) override;
    void onReset() override;

private:
    unsigned int key_step_{0};
    unsigned int option_key_step_{0};
};

/** @brief Permissive power-control register model. */
class PwrPeripheral final : public RegisterPeripheral {
public:
    explicit PwrPeripheral(sim::EventLoop* event_loop = nullptr, sim::TraceRecorder* trace = nullptr);

protected:
    [[nodiscard]] std::uint32_t loadRegister(
        std::uint32_t word_offset,
        const mem::AccessContext& context
    ) override;
};

/** @brief One deterministic GPIO output transition. */
struct GpioTransition {
    sim::SimTimeNs time_ns{0};
    unsigned int pin{0};
    bool high{false};
};

/** @brief STM32G4 GPIO port with external-input overrides and output callbacks. */
class GpioPeripheral final : public RegisterPeripheral {
public:
    using OutputCallback = std::function<void(unsigned int pin, bool high, sim::SimTimeNs time_ns)>;

    explicit GpioPeripheral(
        std::string name = "GPIO",
        sim::EventLoop* event_loop = nullptr,
        sim::TraceRecorder* trace = nullptr
    );

    void setInput(unsigned int pin, bool high);
    void releaseInput(unsigned int pin);
    [[nodiscard]] bool output(unsigned int pin) const noexcept;
    void setOutputCallback(OutputCallback callback);
    [[nodiscard]] const std::vector<GpioTransition>& transitions() const noexcept { return transitions_; }
    void clearTransitions() noexcept { transitions_.clear(); }

protected:
    [[nodiscard]] std::uint32_t loadRegister(
        std::uint32_t word_offset,
        const mem::AccessContext& context
    ) override;
    void storeRegister(
        std::uint32_t word_offset,
        std::uint32_t previous,
        std::uint32_t value,
        std::uint32_t write_mask,
        const mem::AccessContext& context
    ) override;

private:
    void applyOutput(std::uint32_t new_output);
    [[nodiscard]] std::uint32_t inputValue() const noexcept;

    std::uint16_t external_input_mask_{0};
    std::uint16_t external_input_value_{0};
    OutputCallback output_callback_;
    std::vector<GpioTransition> transitions_;
};

/** @brief Timestamped byte emitted by a USART. */
struct UsartTxByte {
    sim::SimTimeNs time_ns{0};
    std::uint8_t value{0};
};

/** @brief STM32G4 USART model with scripted RX and immediate ready flags. */
class UsartPeripheral final : public RegisterPeripheral {
public:
    using TxCallback = std::function<void(std::uint8_t value, sim::SimTimeNs time_ns)>;
    using RxProvider = std::function<std::optional<std::uint8_t>(sim::SimTimeNs time_ns)>;
    using InterruptCallback = std::function<void()>;

    explicit UsartPeripheral(
        std::string name = "USART",
        sim::EventLoop* event_loop = nullptr,
        sim::TraceRecorder* trace = nullptr
    );
    ~UsartPeripheral() override;

    void injectRx(std::span<const std::uint8_t> bytes);
    void injectRx(std::uint8_t byte);
    void setTxCallback(TxCallback callback);
    void setRxProvider(RxProvider provider);
    void setInterruptCallback(InterruptCallback callback);
    void setIdleGap(sim::SimTimeNs idle_gap_ns) noexcept { idle_gap_ns_ = idle_gap_ns; }
    [[nodiscard]] const std::vector<UsartTxByte>& txLog() const noexcept { return tx_log_; }
    void clearTxLog() noexcept { tx_log_.clear(); }

protected:
    [[nodiscard]] std::uint32_t loadRegister(
        std::uint32_t word_offset,
        const mem::AccessContext& context
    ) override;
    void storeRegister(
        std::uint32_t word_offset,
        std::uint32_t previous,
        std::uint32_t value,
        std::uint32_t write_mask,
        const mem::AccessContext& context
    ) override;
    void onReset() override;

private:
    void refillRx();
    void updateStatus();
    void signalInterruptIfEnabled();
    void scheduleIdle();
    void cancelIdle() noexcept;

    std::deque<std::uint8_t> rx_queue_;
    std::vector<UsartTxByte> tx_log_;
    TxCallback tx_callback_;
    RxProvider rx_provider_;
    InterruptCallback interrupt_callback_;
    sim::SimTimeNs idle_gap_ns_{1000000};
    sim::EventId idle_event_{0};
};

/** @brief STM32G4 timer update event record. */
struct TimerUpdate {
    sim::SimTimeNs time_ns{0};
    std::uint32_t count_before_update{0};
};

/** @brief Basic/general-purpose TIM model driven by simulated time. */
class TimerPeripheral final : public RegisterPeripheral {
public:
    using InterruptCallback = std::function<void()>;
    using UpdateCallback = std::function<void(sim::SimTimeNs time_ns)>;

    explicit TimerPeripheral(
        std::string name = "TIM",
        std::uint64_t input_clock_hz = 16000000,
        sim::EventLoop* event_loop = nullptr,
        sim::TraceRecorder* trace = nullptr
    );
    ~TimerPeripheral() override;

    void setInputClockHz(std::uint64_t frequency_hz);
    [[nodiscard]] std::uint64_t inputClockHz() const noexcept { return input_clock_hz_; }
    void setInterruptCallback(InterruptCallback callback);
    void setUpdateCallback(UpdateCallback callback);
    [[nodiscard]] const std::vector<TimerUpdate>& updates() const noexcept { return updates_; }

protected:
    [[nodiscard]] std::uint32_t loadRegister(
        std::uint32_t word_offset,
        const mem::AccessContext& context
    ) override;
    void storeRegister(
        std::uint32_t word_offset,
        std::uint32_t previous,
        std::uint32_t value,
        std::uint32_t write_mask,
        const mem::AccessContext& context
    ) override;
    void onReset() override;

private:
    void captureCounter();
    [[nodiscard]] std::uint32_t liveCounter() const noexcept;
    void scheduleUpdate();
    void cancelUpdate() noexcept;
    void fireUpdate(bool forced);

    std::uint64_t input_clock_hz_{16000000};
    sim::SimTimeNs counter_epoch_ns_{0};
    std::uint32_t counter_epoch_value_{0};
    sim::EventId update_event_{0};
    InterruptCallback interrupt_callback_;
    UpdateCallback update_callback_;
    std::vector<TimerUpdate> updates_;
};

/** @brief ADC conversion result with selected channel metadata. */
struct AdcSample {
    sim::SimTimeNs time_ns{0};
    unsigned int channel{0};
    std::uint16_t value{0};
};

/** @brief STM32G4 ADC model with sequenced, clock-derived conversions. */
class AdcPeripheral final : public RegisterPeripheral {
public:
    using ChannelProvider = std::function<std::uint16_t(unsigned int channel, sim::SimTimeNs time_ns)>;
    using SampleCallback = std::function<void(const AdcSample& sample)>;
    using InterruptCallback = std::function<void()>;

    explicit AdcPeripheral(
        std::string name = "ADC",
        sim::EventLoop* event_loop = nullptr,
        sim::TraceRecorder* trace = nullptr
    );
    ~AdcPeripheral() override;

    [[nodiscard]] mem::MemoryResult<std::uint64_t> read(
        std::uint32_t offset,
        mem::AccessSize size,
        const mem::AccessContext& context
    ) override;
    [[nodiscard]] mem::MemoryResult<std::uint64_t> write(
        std::uint32_t offset,
        mem::AccessSize size,
        std::uint64_t value,
        const mem::AccessContext& context
    ) override;

    void setChannelValue(unsigned int channel, std::uint16_t value);
    void setChannelProvider(ChannelProvider provider);
    void setSampleCallback(SampleCallback callback);
    void setInterruptCallback(InterruptCallback callback);
    /** Overrides register-derived conversion timing, primarily for focused tests. */
    void setConversionDelay(sim::SimTimeNs delay_ns) noexcept { conversion_delay_override_ns_ = delay_ns; }
    void setInputClockHz(std::uint64_t frequency_hz);
    /** @brief Enables timestamped sample history; disabling it permits lazy continuous conversion. */
    void setSampleHistoryEnabled(bool enabled);
    [[nodiscard]] bool sampleHistoryEnabled() const noexcept { return sample_history_enabled_; }
    [[nodiscard]] const std::vector<AdcSample>& samples() const noexcept { return samples_; }

protected:
    [[nodiscard]] std::uint32_t loadRegister(
        std::uint32_t word_offset,
        const mem::AccessContext& context
    ) override;
    void storeRegister(
        std::uint32_t word_offset,
        std::uint32_t previous,
        std::uint32_t value,
        std::uint32_t write_mask,
        const mem::AccessContext& context
    ) override;
    void onReset() override;

private:
    [[nodiscard]] unsigned int sequenceLength() const noexcept;
    [[nodiscard]] unsigned int channelForRank(unsigned int rank) const noexcept;
    [[nodiscard]] sim::SimTimeNs conversionDelayForRank(unsigned int rank) const noexcept;
    [[nodiscard]] bool continuousMode() const noexcept;
    [[nodiscard]] bool conversionObservable() const noexcept;
    [[nodiscard]] bool lazyConversionEligible() const noexcept;
    void startConversion();
    void completeConversion();
    void materializeConversion(sim::SimTimeNs completion_time, bool observable);
    void synchronizeLazyConversions();
    void refreshConversionScheduling();
    void armNextConversion(sim::SimTimeNs completion_time);
    void scheduleConversionEvent();
    void cancelConversion() noexcept;

    std::array<std::uint16_t, 20> channel_values_{};
    ChannelProvider channel_provider_;
    SampleCallback sample_callback_;
    InterruptCallback interrupt_callback_;
    std::uint64_t input_clock_hz_{16'000'000U};
    sim::SimTimeNs conversion_delay_override_ns_{0};
    unsigned int sequence_rank_{0};
    sim::EventId conversion_event_{0};
    std::optional<sim::SimTimeNs> next_conversion_ns_;
    bool sample_history_enabled_{true};
    std::vector<AdcSample> samples_;
};

/** @brief One complete SPI frame transaction. */
struct SpiTransfer {
    sim::SimTimeNs time_ns{0};
    std::vector<std::uint8_t> transmitted;
    std::vector<std::uint8_t> received;
};

/** @brief STM32G4 SPI model with an attachable deterministic transfer callback. */
class SpiPeripheral final : public RegisterPeripheral {
public:
    using TransferCallback = std::function<std::vector<std::uint8_t>(
        std::span<const std::uint8_t> transmitted,
        sim::SimTimeNs time_ns
    )>;
    using InterruptCallback = std::function<void()>;
    using DmaRequestCallback = std::function<void(bool transmit)>;

    explicit SpiPeripheral(
        std::string name = "SPI",
        sim::EventLoop* event_loop = nullptr,
        sim::TraceRecorder* trace = nullptr
    );

    [[nodiscard]] mem::MemoryResult<std::uint64_t> read(
        std::uint32_t offset,
        mem::AccessSize size,
        const mem::AccessContext& context
    ) override;

    void setTransferCallback(TransferCallback callback);
    void setInterruptCallback(InterruptCallback callback);
    void setDmaRequestCallback(DmaRequestCallback callback);
    void setEcho(bool enabled) noexcept { echo_ = enabled; }
    [[nodiscard]] const std::vector<SpiTransfer>& transferLog() const noexcept { return transfers_; }
    void clearTransferLog() noexcept { transfers_.clear(); }

protected:
    [[nodiscard]] std::uint32_t loadRegister(
        std::uint32_t word_offset,
        const mem::AccessContext& context
    ) override;
    void storeRegister(
        std::uint32_t word_offset,
        std::uint32_t previous,
        std::uint32_t value,
        std::uint32_t write_mask,
        const mem::AccessContext& context
    ) override;
    void onReset() override;

private:
    void transferWord(std::uint32_t value, std::uint32_t write_mask);
    void updateStatus();
    void signalRequests();

    std::deque<std::uint8_t> receive_bytes_;
    TransferCallback transfer_callback_;
    InterruptCallback interrupt_callback_;
    DmaRequestCallback dma_request_callback_;
    bool echo_{false};
    std::vector<SpiTransfer> transfers_;
};

/** @brief One serviced DMA peripheral request. */
struct DmaTransfer {
    sim::SimTimeNs time_ns{0};
    unsigned int channel{0};
    std::uint32_t items{0};
    bool memory_to_peripheral{false};
    bool success{false};
};

/** @brief STM32G4 DMA controller with request-driven channel progress. */
class DmaPeripheral final : public RegisterPeripheral {
public:
    using InterruptCallback = std::function<void(unsigned int channel)>;

    explicit DmaPeripheral(
        std::string name = "DMA",
        unsigned int channel_count = 7,
        mem::MemoryBus* memory = nullptr,
        sim::EventLoop* event_loop = nullptr,
        sim::TraceRecorder* trace = nullptr
    );

    void setMemory(mem::MemoryBus* memory) noexcept { memory_ = memory; }
    void setInterruptCallback(InterruptCallback callback);
    void setTransferHistoryEnabled(bool enabled) noexcept { transfer_history_enabled_ = enabled; }
    /** @brief Services one peripheral request for an already-enabled channel. */
    [[nodiscard]] bool request(unsigned int channel);
    [[nodiscard]] unsigned int channelCount() const noexcept { return channel_count_; }
    [[nodiscard]] const std::vector<DmaTransfer>& transferLog() const noexcept { return transfers_; }

protected:
    void storeRegister(
        std::uint32_t word_offset,
        std::uint32_t previous,
        std::uint32_t value,
        std::uint32_t write_mask,
        const mem::AccessContext& context
    ) override;

private:
    [[nodiscard]] bool channelForOffset(std::uint32_t offset, unsigned int& channel) const noexcept;
    void initializeChannel(unsigned int channel);
    void runChannelRequest(unsigned int channel);
    [[nodiscard]] mem::MemoryResult<std::uint64_t> readMemory(
        std::uint32_t address,
        std::uint32_t width
    );
    [[nodiscard]] mem::MemoryResult<std::uint64_t> writeMemory(
        std::uint32_t address,
        std::uint32_t width,
        std::uint64_t value
    );
    void setChannelFlag(unsigned int channel, unsigned int flag_bit);

    unsigned int channel_count_{7};
    mem::MemoryBus* memory_{nullptr};
    InterruptCallback interrupt_callback_;
    std::array<std::uint32_t, 8> reload_counts_{};
    bool transfer_history_enabled_{true};
    std::vector<DmaTransfer> transfers_;
};

/** @brief Register-backed STM32G4 DMAMUX request selector. */
class DmamuxPeripheral final : public RegisterPeripheral {
public:
    explicit DmamuxPeripheral(
        std::string name = "DMAMUX",
        unsigned int channel_count = 16,
        sim::EventLoop* event_loop = nullptr,
        sim::TraceRecorder* trace = nullptr
    );

    [[nodiscard]] std::uint8_t requestForChannel(unsigned int channel) const noexcept;
    [[nodiscard]] std::uint64_t routingGeneration() const noexcept { return routing_generation_; }

protected:
    void storeRegister(
        std::uint32_t word_offset,
        std::uint32_t previous,
        std::uint32_t value,
        std::uint32_t write_mask,
        const mem::AccessContext& context
    ) override;

private:
    unsigned int channel_count_{16};
    std::uint64_t routing_generation_{1U};
};

/** @brief Independent watchdog with key reload and optional reset callback. */
class IwdgPeripheral final : public RegisterPeripheral {
public:
    using ResetCallback = std::function<void()>;

    explicit IwdgPeripheral(
        bool reset_enabled = false,
        sim::EventLoop* event_loop = nullptr,
        sim::TraceRecorder* trace = nullptr
    );
    ~IwdgPeripheral() override;

    void setResetCallback(ResetCallback callback);
    void setResetEnabled(bool enabled);
    [[nodiscard]] bool running() const noexcept { return running_; }

protected:
    [[nodiscard]] std::uint32_t loadRegister(
        std::uint32_t word_offset,
        const mem::AccessContext& context
    ) override;
    void storeRegister(
        std::uint32_t word_offset,
        std::uint32_t previous,
        std::uint32_t value,
        std::uint32_t write_mask,
        const mem::AccessContext& context
    ) override;
    void onReset() override;

private:
    void reload();
    void cancelTimeout() noexcept;

    bool reset_enabled_{false};
    bool running_{false};
    bool registers_unlocked_{false};
    sim::EventId timeout_event_{0};
    ResetCallback reset_callback_;
};

/** @brief Window watchdog with optional timeout reset callback. */
class WwdgPeripheral final : public RegisterPeripheral {
public:
    using ResetCallback = std::function<void()>;

    explicit WwdgPeripheral(
        bool reset_enabled = false,
        std::uint64_t peripheral_clock_hz = 16000000,
        sim::EventLoop* event_loop = nullptr,
        sim::TraceRecorder* trace = nullptr
    );
    ~WwdgPeripheral() override;

    void setResetCallback(ResetCallback callback);
    void setResetEnabled(bool enabled);

protected:
    void storeRegister(
        std::uint32_t word_offset,
        std::uint32_t previous,
        std::uint32_t value,
        std::uint32_t write_mask,
        const mem::AccessContext& context
    ) override;
    void onReset() override;

private:
    void reload();
    void cancelTimeout() noexcept;

    bool reset_enabled_{false};
    std::uint64_t peripheral_clock_hz_{16000000};
    sim::EventId timeout_event_{0};
    ResetCallback reset_callback_;
};

/** @brief Compatibility name matching the STM32 TIM abbreviation. */
using TimPeripheral = TimerPeripheral;

} // namespace fil::stm32g4
