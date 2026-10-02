#include "fil/cortexm/exceptions.hpp"

#include "fil/cortexm/system_control.hpp"
#include "fil/cpu/cortex_m4.hpp"
#include "fil/mem/memory_bus.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

namespace fil::cortexm {
namespace {

Error runtimeError(std::string message) {
    return Error{ErrorCategory::runtime, std::move(message), std::nullopt};
}

} // namespace

ExceptionController::ExceptionController(mem::MemoryBus& memory, SystemControl& system) noexcept
    : memory_(memory), system_(system) {}

Result<void> ExceptionController::validateStackRange(
    const std::uint32_t address,
    const std::uint32_t size
) const {
    if (memory_.containsWritableRange(address, size)) return {};
    return runtimeError("exception stack frame is outside writable memory");
}

Result<void> ExceptionController::enter(cpu::CpuState& state, const std::uint16_t exception_number) {
    if (exception_number == 0 || exception_number >= 256U) {
        return runtimeError("invalid Cortex-M exception number");
    }
    const bool from_handler = state.inHandlerMode();
    const bool used_psp = !from_handler && (state.control & 2U) != 0;
    const bool extended_fp_frame = (state.control & 4U) != 0U;
    std::uint32_t& stack_pointer = used_psp ? state.psp : state.msp;
    const std::uint32_t old_sp = stack_pointer;
    const bool add_padding = (system_.ccr() & (1U << 9U)) != 0 && (old_sp & 7U) != 0;
    constexpr std::uint32_t fp_frame_size = 18U * 4U;
    const std::uint32_t frame_size = 32U + (extended_fp_frame ? fp_frame_size : 0U)
        + (add_padding ? 4U : 0U);
    if (old_sp < frame_size) return runtimeError("exception stack pointer underflow");
    const std::uint32_t new_sp = old_sp - frame_size;
    auto stack_valid = validateStackRange(new_sp, frame_size);
    if (!stack_valid) return stack_valid.error();

    const std::uint64_t vector_address = static_cast<std::uint64_t>(system_.vectorBase())
        + static_cast<std::uint64_t>(exception_number) * 4U;
    if (vector_address > 0xffffffffULL) return runtimeError("exception vector address wraps target space");
    auto handler = memory_.read32(
        static_cast<std::uint32_t>(vector_address),
        {mem::AccessType::data_read, state.currentInstrAddr()}
    );
    if (!handler) return runtimeError("unable to read exception vector: " + mem::formatBusFault(handler.fault()));
    if ((handler.value() & 1U) == 0) return runtimeError("exception handler vector does not select Thumb state");

    // Lazy FP stacking (LSPEN): reserve the 72-byte area but defer pushing
    // S0-S15/FPSCR until handler code first touches the FP register file.
    // A nested entry while one reservation is pending falls back to eager
    // stacking for that level (memory-safe; see CpuState::fp_lazy_active).
    const bool lazy_fp = extended_fp_frame
        && (system_.fpccr() & SystemControl::fpccr_lspen) != 0U
        && !state.fp_lazy_active;
    if (extended_fp_frame && !lazy_fp) {
        for (std::size_t index = 0; index < 16U; ++index) {
            auto written = memory_.write32(
                new_sp + static_cast<std::uint32_t>(index * 4U),
                std::bit_cast<std::uint32_t>(state.s[index]),
                {mem::AccessType::data_write, state.currentInstrAddr()}
            );
            if (!written) return runtimeError("unable to stack floating-point exception frame: " + mem::formatBusFault(written.fault()));
        }
        auto fpscr_written = memory_.write32(
            new_sp + 64U, state.fpscr,
            {mem::AccessType::data_write, state.currentInstrAddr()}
        );
        if (!fpscr_written) return runtimeError("unable to stack FPSCR: " + mem::formatBusFault(fpscr_written.fault()));
        auto reserved_written = memory_.write32(
            new_sp + 68U, 0U,
            {mem::AccessType::data_write, state.currentInstrAddr()}
        );
        if (!reserved_written) return runtimeError("unable to stack floating-point reserved word: " + mem::formatBusFault(reserved_written.fault()));
    }
    if (lazy_fp) {
        state.fp_lazy_active = true;
        state.fp_lazy_base = new_sp;
    } else if (state.fp_lazy_active) {
        // Nested eager entry inside a pending lazy reservation abandons the
        // outer reservation: this level wrote (or will use only) its own
        // frame, so subsequent returns unstack eagerly. Memory-safe; exact
        // except for FP use across more than one lazy nesting level.
        state.fp_lazy_active = false;
    }

    const std::uint32_t core_frame_base = new_sp + (extended_fp_frame ? fp_frame_size : 0U);
    const std::array<std::uint32_t, 8> frame{
        state.r[0], state.r[1], state.r[2], state.r[3], state.r[12], state.r[14],
        state.r[15] | 1U,
        state.xpsr | cpu::xpsr_t | (add_padding ? (1U << 9U) : 0U),
    };
    for (std::size_t index = 0; index < frame.size(); ++index) {
        auto written = memory_.write32(
            core_frame_base + static_cast<std::uint32_t>(index * 4U),
            frame[index],
            {mem::AccessType::data_write, state.currentInstrAddr()}
        );
        if (!written) return runtimeError("unable to stack exception frame: " + mem::formatBusFault(written.fault()));
    }

    stack_pointer = new_sp;
    state.r[13] = state.msp;
    state.r[14] = extended_fp_frame
        ? (from_handler ? 0xffffffe1U : (used_psp ? 0xffffffedU : 0xffffffe9U))
        : (from_handler ? 0xfffffff1U : (used_psp ? 0xfffffffdU : 0xfffffff9U));
    state.r[15] = handler.value() & ~1U;
    state.instruction_address = state.r[15];
    state.thumb = true;
    state.xpsr = (state.xpsr & ~cpu::xpsr_ipsr_mask) | exception_number | cpu::xpsr_t;
    state.setItState(0);
    active_stack_.push_back(exception_number);
    system_.enter(exception_number);
    return {};
}

Result<bool> ExceptionController::enterPending(cpu::CpuState& state) {
    const auto pending = system_.nextPending(state.primask, state.basepri, state.faultmask);
    if (!pending) return false;
    auto entered = enter(state, *pending);
    if (!entered) return entered.error();
    return true;
}

bool ExceptionController::isExceptionReturn(const std::uint32_t value) noexcept {
    return cpu::isExceptionReturn(value);
}

Result<void> ExceptionController::exceptionReturn(
    cpu::CpuState& state,
    const std::uint32_t exc_return
) {
    if (!isExceptionReturn(exc_return)) return runtimeError("invalid EXC_RETURN value");
    if (active_stack_.empty() || !state.inHandlerMode()) {
        return runtimeError("EXC_RETURN attempted outside an active exception");
    }
    const bool use_psp = (exc_return & (1U << 2U)) != 0;
    const bool extended_fp_frame = (exc_return & (1U << 4U)) == 0U;
    constexpr std::uint32_t fp_frame_size = 18U * 4U;
    const std::uint32_t core_frame_offset = extended_fp_frame ? fp_frame_size : 0U;
    std::uint32_t& stack_pointer = use_psp ? state.psp : state.msp;
    auto stack_valid = validateStackRange(stack_pointer, core_frame_offset + 32U);
    if (!stack_valid) return stack_valid.error();

    // A pending lazy reservation means this level never pushed FP state:
    // skip the reads, deallocate the reserved area via the existing SP
    // advance below, and clear the reservation.
    if (extended_fp_frame && !state.fp_lazy_active) {
        for (std::size_t index = 0; index < 16U; ++index) {
            auto value = memory_.read32(
                stack_pointer + static_cast<std::uint32_t>(index * 4U),
                {mem::AccessType::data_read, state.currentInstrAddr()}
            );
            if (!value) return runtimeError("unable to unstack floating-point exception frame: " + mem::formatBusFault(value.fault()));
            state.s[index] = std::bit_cast<float>(value.value());
        }
        auto fpscr = memory_.read32(
            stack_pointer + 64U,
            {mem::AccessType::data_read, state.currentInstrAddr()}
        );
        if (!fpscr) return runtimeError("unable to unstack FPSCR: " + mem::formatBusFault(fpscr.fault()));
        state.fpscr = fpscr.value();
    }
    state.fp_lazy_active = false;
    std::array<std::uint32_t, 8> frame{};
    for (std::size_t index = 0; index < frame.size(); ++index) {
        auto value = memory_.read32(
            stack_pointer + core_frame_offset + static_cast<std::uint32_t>(index * 4U),
            {mem::AccessType::data_read, state.currentInstrAddr()}
        );
        if (!value) return runtimeError("unable to unstack exception frame: " + mem::formatBusFault(value.fault()));
        frame[index] = value.value();
    }
    if ((frame[7] & cpu::xpsr_t) == 0) {
        std::ostringstream message;
        message << "exception return frame clears Thumb state"
                << " sp=0x" << std::hex << std::setfill('0') << std::setw(8) << stack_pointer
                << " pc=0x" << std::setw(8) << frame[6]
                << " xpsr=0x" << std::setw(8) << frame[7];
        return runtimeError(message.str());
    }
    const bool had_padding = (frame[7] & (1U << 9U)) != 0;
    const std::uint16_t leaving = active_stack_.back();
    active_stack_.pop_back();
    system_.leave(leaving);

    state.r[0] = frame[0];
    state.r[1] = frame[1];
    state.r[2] = frame[2];
    state.r[3] = frame[3];
    state.r[12] = frame[4];
    state.r[14] = frame[5];
    state.r[15] = frame[6] & ~1U;
    state.xpsr = frame[7] & ~(1U << 9U);
    state.thumb = true;
    stack_pointer += core_frame_offset + 32U + (had_padding ? 4U : 0U);
    if ((exc_return & (1U << 3U)) != 0U) {
        state.control = (state.control & ~std::uint32_t{6})
            | (use_psp ? 2U : 0U)
            | (extended_fp_frame ? 4U : 0U);
    }
    state.r[13] = state.inHandlerMode() ? state.msp : (((state.control & 2U) != 0) ? state.psp : state.msp);
    state.instruction_address = state.r[15];
    state.setItState(static_cast<std::uint8_t>(
        ((state.xpsr >> 8U) & 0xfcU) | ((state.xpsr >> 25U) & 0x03U)
    ));
    if (!active_stack_.empty()) system_.resume(active_stack_.back());
    return {};
}

} // namespace fil::cortexm
