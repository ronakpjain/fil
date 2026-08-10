#include "fil/cortexm/system_control.hpp"

#include <algorithm>
#include <bit>
#include <limits>
#include <string>

namespace fil::cortexm {
namespace {

constexpr std::uint32_t system_base = 0xe0000000U;
constexpr std::uint32_t cpuid_value = 0x410fc241U;

std::uint32_t widthMask(const mem::AccessSize size) {
    const std::uint32_t bits = mem::byteCount(size) * 8U;
    return bits == 32U ? std::numeric_limits<std::uint32_t>::max() : (std::uint32_t{1} << bits) - 1U;
}

} // namespace

SystemControl::SystemControl(const std::uint32_t vector_base) {
    reset(vector_base);
}

void SystemControl::reset(const std::uint32_t vector_base) {
    nvic_enable_.fill(0);
    nvic_pending_.fill(0);
    nvic_active_.fill(0);
    nvic_priority_.fill(0);
    system_priority_.fill(0);
    systick_ctrl_ = 0;
    systick_load_ = 0;
    systick_value_ = 0;
    systick_cycles_to_wrap_ = 0;
    systick_count_flag_ = false;
    vtor_ = vector_base & 0xffffff80U;
    aircr_ = 0;
    scr_ = 0;
    ccr_ = 0x00000200U;
    shcsr_ = 0;
    cfsr_ = 0;
    hfsr_ = 0;
    mmfar_ = 0;
    bfar_ = 0;
    cpacr_ = 0;
    demcr_ = 0;
    fpccr_ = 0xc0000000U;
    dwt_ctrl_ = 0;
    dwt_cyccnt_ = 0;
    active_exception_ = 0;
    pendsv_pending_ = false;
    systick_pending_ = false;
    external_pending_enabled_ = false;
    reset_requested_ = false;
}

void SystemControl::advanceCycles(const std::uint64_t cycles) {
    if ((dwt_ctrl_ & 1U) != 0) dwt_cyccnt_ += static_cast<std::uint32_t>(cycles);
    if ((systick_ctrl_ & 1U) == 0 || systick_load_ == 0 || cycles == 0) {
        return;
    }

    const std::uint32_t period = systick_load_ + 1U;
    if (systick_cycles_to_wrap_ == 0U) systick_cycles_to_wrap_ = period;
    std::uint64_t remaining = cycles;
    if (remaining < systick_cycles_to_wrap_) {
        systick_cycles_to_wrap_ -= static_cast<std::uint32_t>(remaining);
        systick_value_ = systick_cycles_to_wrap_ - 1U;
        return;
    }
    remaining -= systick_cycles_to_wrap_;
    systick_count_flag_ = true;
    if ((systick_ctrl_ & 2U) != 0) systick_pending_ = true;

    if (remaining != 0) {
        if (remaining >= period) {
            systick_count_flag_ = true;
            if ((systick_ctrl_ & 2U) != 0) systick_pending_ = true;
            remaining %= period;
        }
        systick_cycles_to_wrap_ = remaining == 0U
            ? period : period - static_cast<std::uint32_t>(remaining);
    } else {
        systick_cycles_to_wrap_ = period;
    }
    systick_value_ = systick_cycles_to_wrap_ - 1U;
}

std::optional<std::uint64_t> SystemControl::cyclesUntilSysTickInterrupt() const noexcept {
    if (systick_pending_) return 0U;
    if ((systick_ctrl_ & 3U) != 3U || systick_load_ == 0U) return std::nullopt;
    return systick_cycles_to_wrap_ == 0U
        ? static_cast<std::uint64_t>(systick_load_) + 1U
        : systick_cycles_to_wrap_;
}

void SystemControl::pend(const std::uint16_t exception_number) {
    if (exception_number == static_cast<std::uint16_t>(ExceptionNumber::pend_sv)) {
        pendsv_pending_ = true;
    } else if (exception_number == static_cast<std::uint16_t>(ExceptionNumber::sys_tick)) {
        systick_pending_ = true;
    } else if (exception_number >= 16U && exception_number < 256U) {
        const std::uint16_t irq = exception_number - 16U;
        nvic_pending_[irq / 32U] |= std::uint32_t{1} << (irq % 32U);
        refreshPendingSummary();
    }
}

void SystemControl::clearPending(const std::uint16_t exception_number) {
    if (exception_number == static_cast<std::uint16_t>(ExceptionNumber::pend_sv)) {
        pendsv_pending_ = false;
    } else if (exception_number == static_cast<std::uint16_t>(ExceptionNumber::sys_tick)) {
        systick_pending_ = false;
    } else if (exception_number >= 16U && exception_number < 256U) {
        const std::uint16_t irq = exception_number - 16U;
        nvic_pending_[irq / 32U] &= ~(std::uint32_t{1} << (irq % 32U));
        refreshPendingSummary();
    }
}

void SystemControl::enter(const std::uint16_t exception_number) {
    clearPending(exception_number);
    active_exception_ = exception_number;
    if (exception_number >= 16U && exception_number < 256U) {
        const std::uint16_t irq = exception_number - 16U;
        nvic_active_[irq / 32U] |= std::uint32_t{1} << (irq % 32U);
    }
}

void SystemControl::leave(const std::uint16_t exception_number) {
    if (exception_number >= 16U && exception_number < 256U) {
        const std::uint16_t irq = exception_number - 16U;
        nvic_active_[irq / 32U] &= ~(std::uint32_t{1} << (irq % 32U));
    }
    if (active_exception_ == exception_number) active_exception_ = 0;
}

void SystemControl::refreshPendingSummary() noexcept {
    external_pending_enabled_ = false;
    for (std::size_t index = 0; index < nvic_pending_.size(); ++index) {
        if ((nvic_pending_[index] & nvic_enable_[index]) != 0U) {
            external_pending_enabled_ = true;
            return;
        }
    }
}

bool SystemControl::isPending(const std::uint16_t exception_number) const noexcept {
    if (exception_number == static_cast<std::uint16_t>(ExceptionNumber::pend_sv)) return pendsv_pending_;
    if (exception_number == static_cast<std::uint16_t>(ExceptionNumber::sys_tick)) return systick_pending_;
    if (exception_number < 16U || exception_number >= 256U) return false;
    const std::uint16_t irq = exception_number - 16U;
    return (nvic_pending_[irq / 32U] & (std::uint32_t{1} << (irq % 32U))) != 0;
}

bool SystemControl::isEnabled(const std::uint16_t exception_number) const noexcept {
    if (exception_number < 16U) return true;
    if (exception_number >= 256U) return false;
    const std::uint16_t irq = exception_number - 16U;
    return (nvic_enable_[irq / 32U] & (std::uint32_t{1} << (irq % 32U))) != 0;
}

std::uint8_t SystemControl::priority(const std::uint16_t exception_number) const noexcept {
    if (exception_number >= 4U && exception_number <= 15U) {
        return system_priority_[exception_number - 4U];
    }
    if (exception_number >= 16U && exception_number < 256U) {
        return nvic_priority_[exception_number - 16U];
    }
    return 0;
}

std::optional<std::uint16_t> SystemControl::nextPending(
    const std::uint32_t primask,
    const std::uint32_t basepri,
    const std::uint32_t faultmask
) const {
    if (!hasEnabledPending()) return std::nullopt;

    std::optional<std::uint16_t> selected;
    std::uint16_t selected_priority = 0x100U;
    const auto architectural_priority = [this](const std::uint16_t exception_number) -> int {
        if (exception_number == static_cast<std::uint16_t>(ExceptionNumber::nmi)) return -2;
        if (exception_number == static_cast<std::uint16_t>(ExceptionNumber::hard_fault)) return -1;
        return static_cast<int>(priority(exception_number));
    };
    const auto consider = [&](const std::uint16_t exception_number) {
        if (!isPending(exception_number) || !isEnabled(exception_number)) return;
        if (faultmask != 0 && exception_number != static_cast<std::uint16_t>(ExceptionNumber::nmi)) return;
        if (primask != 0 && exception_number > static_cast<std::uint16_t>(ExceptionNumber::hard_fault)) return;
        const std::uint16_t candidate_priority = priority(exception_number);
        if (active_exception_ != 0U
            && architectural_priority(exception_number) >= architectural_priority(active_exception_)) {
            return;
        }
        if (basepri != 0 && exception_number > static_cast<std::uint16_t>(ExceptionNumber::hard_fault)
            && candidate_priority >= (basepri & 0xffU)) return;
        if (!selected || candidate_priority < selected_priority
            || (candidate_priority == selected_priority && exception_number < *selected)) {
            selected = exception_number;
            selected_priority = candidate_priority;
        }
    };

    consider(static_cast<std::uint16_t>(ExceptionNumber::pend_sv));
    consider(static_cast<std::uint16_t>(ExceptionNumber::sys_tick));
    for (std::size_t word_index = 0; word_index < nvic_pending_.size(); ++word_index) {
        std::uint32_t candidates = nvic_pending_[word_index] & nvic_enable_[word_index];
        while (candidates != 0U) {
            const auto bit = static_cast<std::uint16_t>(std::countr_zero(candidates));
            const auto exception_number = static_cast<std::uint16_t>(
                16U + word_index * 32U + bit
            );
            consider(exception_number);
            candidates &= candidates - 1U;
        }
    }
    return selected;
}

bool SystemControl::consumeResetRequest() noexcept {
    const bool requested = reset_requested_;
    reset_requested_ = false;
    return requested;
}

bool SystemControl::fpuEnabled() const noexcept {
    return (cpacr_ & 0x00f00000U) == 0x00f00000U;
}

mem::BusFault SystemControl::accessFault(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const mem::AccessContext& context,
    std::string message
) {
    return mem::BusFault{
        mem::BusFaultReason::device_error,
        system_base + offset,
        size,
        context,
        "cortexm-system",
        std::move(message),
    };
}

std::uint32_t SystemControl::readWord(const std::uint32_t offset) {
    if (offset == 0x1000U) return dwt_ctrl_;
    if (offset == 0x1004U) return dwt_cyccnt_;
    if (offset == 0xe010U) {
        const std::uint32_t value = (systick_ctrl_ & 7U) | (systick_count_flag_ ? (1U << 16U) : 0U);
        systick_count_flag_ = false;
        return value;
    }
    if (offset == 0xe014U) return systick_load_;
    if (offset == 0xe018U) return systick_value_;
    if (offset == 0xe01cU) return 0;
    if (offset >= 0xe100U && offset < 0xe120U) return nvic_enable_[(offset - 0xe100U) / 4U];
    if (offset >= 0xe180U && offset < 0xe1a0U) return nvic_enable_[(offset - 0xe180U) / 4U];
    if (offset >= 0xe200U && offset < 0xe220U) return nvic_pending_[(offset - 0xe200U) / 4U];
    if (offset >= 0xe280U && offset < 0xe2a0U) return nvic_pending_[(offset - 0xe280U) / 4U];
    if (offset >= 0xe300U && offset < 0xe320U) return nvic_active_[(offset - 0xe300U) / 4U];
    if (offset >= 0xe400U && offset < 0xe4f0U) {
        const std::size_t first = offset - 0xe400U;
        return static_cast<std::uint32_t>(nvic_priority_[first])
            | (static_cast<std::uint32_t>(nvic_priority_[first + 1U]) << 8U)
            | (static_cast<std::uint32_t>(nvic_priority_[first + 2U]) << 16U)
            | (static_cast<std::uint32_t>(nvic_priority_[first + 3U]) << 24U);
    }
    if (offset == 0xed00U) return cpuid_value;
    if (offset == 0xed04U) {
        std::uint32_t value = active_exception_;
        if (pendsv_pending_) value |= 1U << 28U;
        if (systick_pending_) value |= 1U << 26U;
        if (const auto pending = nextPending(0, 0, 0)) value |= static_cast<std::uint32_t>(*pending) << 12U;
        return value;
    }
    if (offset == 0xed08U) return vtor_;
    if (offset == 0xed0cU) return 0xfa050000U | (aircr_ & 0x00000700U);
    if (offset == 0xed10U) return scr_;
    if (offset == 0xed14U) return ccr_;
    if (offset >= 0xed18U && offset < 0xed24U) {
        const std::size_t first = offset - 0xed18U;
        return static_cast<std::uint32_t>(system_priority_[first])
            | (static_cast<std::uint32_t>(system_priority_[first + 1U]) << 8U)
            | (static_cast<std::uint32_t>(system_priority_[first + 2U]) << 16U)
            | (static_cast<std::uint32_t>(system_priority_[first + 3U]) << 24U);
    }
    if (offset == 0xed24U) return shcsr_;
    if (offset == 0xed28U) return cfsr_;
    if (offset == 0xed2cU) return hfsr_;
    if (offset == 0xed34U) return mmfar_;
    if (offset == 0xed38U) return bfar_;
    if (offset == 0xed88U) return cpacr_;
    if (offset == 0xedfcU) return demcr_;
    if (offset == 0xef34U) return fpccr_;
    return 0;
}

void SystemControl::writeWord(
    const std::uint32_t offset,
    const std::uint32_t value,
    const std::uint32_t lane_mask
) {
    const auto merge = [=](const std::uint32_t old_value) {
        return (old_value & ~lane_mask) | (value & lane_mask);
    };
    const bool mutates_external_pending =
        (offset >= 0xe100U && offset < 0xe120U)
        || (offset >= 0xe180U && offset < 0xe1a0U)
        || (offset >= 0xe200U && offset < 0xe220U)
        || (offset >= 0xe280U && offset < 0xe2a0U);
    if (offset == 0x1000U) dwt_ctrl_ = merge(dwt_ctrl_);
    else if (offset == 0x1004U) dwt_cyccnt_ = merge(dwt_cyccnt_);
    else if (offset == 0xe010U) {
        const bool was_enabled = (systick_ctrl_ & 1U) != 0U;
        systick_ctrl_ = merge(systick_ctrl_) & 7U;
        if ((systick_ctrl_ & 1U) == 0U || !was_enabled) systick_cycles_to_wrap_ = 0U;
    }
    else if (offset == 0xe014U) {
        systick_load_ = merge(systick_load_) & 0x00ffffffU;
        systick_cycles_to_wrap_ = 0U;
    }
    else if (offset == 0xe018U) {
        systick_value_ = 0;
        systick_cycles_to_wrap_ = 0U;
        systick_count_flag_ = false;
    }
    else if (offset >= 0xe100U && offset < 0xe120U) nvic_enable_[(offset - 0xe100U) / 4U] |= value & lane_mask;
    else if (offset >= 0xe180U && offset < 0xe1a0U) nvic_enable_[(offset - 0xe180U) / 4U] &= ~(value & lane_mask);
    else if (offset >= 0xe200U && offset < 0xe220U) nvic_pending_[(offset - 0xe200U) / 4U] |= value & lane_mask;
    else if (offset >= 0xe280U && offset < 0xe2a0U) nvic_pending_[(offset - 0xe280U) / 4U] &= ~(value & lane_mask);
    else if (offset >= 0xe400U && offset < 0xe4f0U) {
        const std::size_t first = offset - 0xe400U;
        for (std::size_t index = 0; index < 4; ++index) {
            const std::uint32_t byte_mask = 0xffU << (index * 8U);
            if ((lane_mask & byte_mask) != 0) {
                nvic_priority_[first + index] = static_cast<std::uint8_t>(value >> (index * 8U)) & 0xf0U;
            }
        }
    } else if (offset == 0xef00U) {
        const std::uint16_t irq = static_cast<std::uint16_t>(value & 0x1ffU);
        if (irq < 240U) pend(irq + 16U);
    } else if (offset == 0xed04U) {
        if ((value & (1U << 28U)) != 0) pendsv_pending_ = true;
        if ((value & (1U << 27U)) != 0) pendsv_pending_ = false;
        if ((value & (1U << 26U)) != 0) systick_pending_ = true;
        if ((value & (1U << 25U)) != 0) systick_pending_ = false;
    } else if (offset == 0xed08U) vtor_ = merge(vtor_) & 0xffffff80U;
    else if (offset == 0xed0cU && (value >> 16U) == 0x05faU) {
        aircr_ = value & 0x00000700U;
        if ((value & (1U << 2U)) != 0) reset_requested_ = true;
    } else if (offset == 0xed10U) scr_ = merge(scr_);
    else if (offset == 0xed14U) ccr_ = merge(ccr_);
    else if (offset >= 0xed18U && offset < 0xed24U) {
        const std::size_t first = offset - 0xed18U;
        for (std::size_t index = 0; index < 4; ++index) {
            const std::uint32_t byte_mask = 0xffU << (index * 8U);
            if ((lane_mask & byte_mask) != 0) {
                system_priority_[first + index] = static_cast<std::uint8_t>(value >> (index * 8U)) & 0xf0U;
            }
        }
    } else if (offset == 0xed24U) shcsr_ = merge(shcsr_);
    else if (offset == 0xed28U) cfsr_ &= ~(value & lane_mask);
    else if (offset == 0xed2cU) hfsr_ &= ~(value & lane_mask);
    else if (offset == 0xed34U) mmfar_ = merge(mmfar_);
    else if (offset == 0xed38U) bfar_ = merge(bfar_);
    else if (offset == 0xed88U) cpacr_ = merge(cpacr_);
    else if (offset == 0xedfcU) demcr_ = merge(demcr_);
    else if (offset == 0xef34U) fpccr_ = merge(fpccr_);
    if (mutates_external_pending) refreshPendingSummary();
}

mem::MemoryResult<std::uint64_t> SystemControl::read(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const mem::AccessContext& context
) {
    const std::uint32_t width = mem::byteCount(size);
    if (width > 4U || (offset & 3U) + width > 4U || offset > 0x000fffffU - (width - 1U)) {
        return accessFault(offset, size, context, "unsupported or cross-register system-control read");
    }
    const std::uint32_t lane = offset & 3U;
    const std::uint32_t value = readWord(offset & ~3U);
    return static_cast<std::uint64_t>((value >> (lane * 8U)) & widthMask(size));
}

mem::MemoryResult<std::uint64_t> SystemControl::write(
    const std::uint32_t offset,
    const mem::AccessSize size,
    const std::uint64_t value,
    const mem::AccessContext& context
) {
    const std::uint32_t width = mem::byteCount(size);
    if (width > 4U || (offset & 3U) + width > 4U || offset > 0x000fffffU - (width - 1U)) {
        return accessFault(offset, size, context, "unsupported or cross-register system-control write");
    }
    const std::uint32_t shift = (offset & 3U) * 8U;
    const std::uint32_t mask = widthMask(size) << shift;
    writeWord(offset & ~3U, static_cast<std::uint32_t>(value) << shift, mask);
    return std::uint64_t{0};
}

} // namespace fil::cortexm
