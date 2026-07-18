#pragma once

/** @file system_control.hpp
 *  @brief Cortex-M4 SysTick, NVIC, SCB, DWT, and CoreDebug MMIO model.
 */

#include "fil/mem/memory_bus.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace fil::cortexm {

/** @brief Architecturally visible exception numbers used by Cortex-M firmware. */
enum class ExceptionNumber : std::uint16_t {
    reset = 1,
    nmi = 2,
    hard_fault = 3,
    memory_management = 4,
    bus_fault = 5,
    usage_fault = 6,
    sv_call = 11,
    debug_monitor = 12,
    pend_sv = 14,
    sys_tick = 15,
};

/**
 * @brief Combined implementation of the private peripheral bus system window.
 *
 * The model intentionally omits timing details that firmware cannot observe in
 * the current interpreter. It retains interrupt enable/pending/priority state,
 * implements SCB pending side effects, advances SysTick deterministically in
 * target cycles, and exposes a monotonically wrapping DWT cycle counter.
 */
class SystemControl final : public mem::MmioDevice {
public:
    /** @brief Creates reset-state system control with the supplied VTOR value. */
    explicit SystemControl(std::uint32_t vector_base = 0x08000000U);

    /** @brief Restores all registers and pending/active state to reset values. */
    void reset(std::uint32_t vector_base);

    /** @brief Advances SysTick and DWT by a deterministic number of CPU cycles. */
    void advanceCycles(std::uint64_t cycles);

    /** @brief Cycles until an enabled SysTick first pends, or none if it cannot. */
    [[nodiscard]] std::optional<std::uint64_t> cyclesUntilSysTickInterrupt() const noexcept;

    /** @brief Pends any architectural or external exception number. */
    void pend(std::uint16_t exception_number);

    /** @brief Clears a pending exception. */
    void clearPending(std::uint16_t exception_number);

    /** @brief Marks an exception active after CPU exception entry. */
    void enter(std::uint16_t exception_number);

    /** @brief Clears active state after CPU exception return. */
    void leave(std::uint16_t exception_number);

    /**
     * @brief Selects the highest-urgency unmasked pending exception.
     * @param primask Nonzero masks configurable-priority exceptions.
     * @param basepri Nonzero masks priorities numerically greater than or equal to it.
     * @param faultmask Nonzero masks every exception except NMI.
     * @return Exception number, or no value when nothing can preempt.
     */
    [[nodiscard]] std::optional<std::uint16_t> nextPending(
        std::uint32_t primask,
        std::uint32_t basepri,
        std::uint32_t faultmask
    ) const;

    /** @brief Gets the eight-bit programmed priority for one exception. */
    [[nodiscard]] std::uint8_t priority(std::uint16_t exception_number) const noexcept;

    /** @brief Gets the current vector-table base. */
    [[nodiscard]] std::uint32_t vectorBase() const noexcept { return vtor_; }

    /** @brief Gets whether firmware requested a system reset through AIRCR. */
    [[nodiscard]] bool resetRequested() const noexcept { return reset_requested_; }

    /** @brief Tests and clears the simulated system-reset request. */
    [[nodiscard]] bool consumeResetRequest() noexcept;

    /** @brief Gets whether CP10 and CP11 are both granted full FPU access. */
    [[nodiscard]] bool fpuEnabled() const noexcept;

    /** @brief Gets SCB CCR for alignment/fault policy checks. */
    [[nodiscard]] std::uint32_t ccr() const noexcept { return ccr_; }

    /** @brief Gets the currently active exception number, or zero in thread mode. */
    [[nodiscard]] std::uint16_t activeException() const noexcept { return active_exception_; }

    /** @brief Fast conservative test for any enabled pending exception. */
    [[nodiscard]] bool hasEnabledPending() const noexcept {
        return pendsv_pending_ || systick_pending_ || external_pending_enabled_;
    }

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

    [[nodiscard]] std::string_view name() const noexcept override { return "cortexm-system"; }

private:
    [[nodiscard]] std::uint32_t readWord(std::uint32_t aligned_offset);
    void writeWord(std::uint32_t aligned_offset, std::uint32_t value, std::uint32_t lane_mask);
    [[nodiscard]] static mem::BusFault accessFault(
        std::uint32_t offset,
        mem::AccessSize size,
        const mem::AccessContext& context,
        std::string message
    );
    [[nodiscard]] bool isPending(std::uint16_t exception_number) const noexcept;
    [[nodiscard]] bool isEnabled(std::uint16_t exception_number) const noexcept;
    void refreshPendingSummary() noexcept;

    std::array<std::uint32_t, 8> nvic_enable_{};
    std::array<std::uint32_t, 8> nvic_pending_{};
    std::array<std::uint32_t, 8> nvic_active_{};
    std::array<std::uint8_t, 240> nvic_priority_{};
    std::array<std::uint8_t, 12> system_priority_{};

    std::uint32_t systick_ctrl_{0};
    std::uint32_t systick_load_{0};
    std::uint32_t systick_value_{0};
    std::uint32_t systick_cycles_to_wrap_{0};
    bool systick_count_flag_{false};

    std::uint32_t vtor_{0x08000000U};
    std::uint32_t aircr_{0};
    std::uint32_t scr_{0};
    std::uint32_t ccr_{0x00000200U};
    std::uint32_t shcsr_{0};
    std::uint32_t cfsr_{0};
    std::uint32_t hfsr_{0};
    std::uint32_t mmfar_{0};
    std::uint32_t bfar_{0};
    std::uint32_t cpacr_{0};
    std::uint32_t demcr_{0};
    std::uint32_t fpccr_{0xc0000000U};
    std::uint32_t dwt_ctrl_{0};
    std::uint32_t dwt_cyccnt_{0};
    std::uint16_t active_exception_{0};
    bool pendsv_pending_{false};
    bool systick_pending_{false};
    bool external_pending_enabled_{false};
    bool reset_requested_{false};
};

} // namespace fil::cortexm
