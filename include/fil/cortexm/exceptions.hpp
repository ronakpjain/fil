#pragma once

/** @file exceptions.hpp
 *  @brief Cortex-M basic exception stacking, vectoring, and EXC_RETURN handling.
 */

#include "fil/common/result.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace fil::cpu {
struct CpuState;
}
namespace fil::mem {
class MemoryBus;
}

namespace fil::cortexm {

class SystemControl;

/** @brief Coordinates architectural exception frames with SCB/NVIC state. */
class ExceptionController {
public:
    ExceptionController(mem::MemoryBus& memory, SystemControl& system) noexcept;

    /** @brief Enters a specific exception and stacks a basic integer frame. */
    [[nodiscard]] Result<void> enter(cpu::CpuState& state, std::uint16_t exception_number);

    /** @brief Enters the highest-priority pending unmasked exception, when any. */
    [[nodiscard]] Result<bool> enterPending(cpu::CpuState& state);

    /** @brief Recognizes the EXC_RETURN encodings supported by this model. */
    [[nodiscard]] static bool isExceptionReturn(std::uint32_t value) noexcept;

    /** @brief Restores one basic frame selected by an EXC_RETURN value. */
    [[nodiscard]] Result<void> exceptionReturn(cpu::CpuState& state, std::uint32_t exc_return);

    /** @brief Gets nested active exceptions from oldest to newest. */
    [[nodiscard]] const std::vector<std::uint16_t>& activeStack() const noexcept { return active_stack_; }

    /** @brief Restores the active stack from a transactional board checkpoint. */
    void restoreActiveStack(std::vector<std::uint16_t> stack) {
        active_stack_ = std::move(stack);
    }

private:
    [[nodiscard]] Result<void> validateStackRange(std::uint32_t address, std::uint32_t size) const;

    mem::MemoryBus& memory_;
    SystemControl& system_;
    std::vector<std::uint16_t> active_stack_;
};

} // namespace fil::cortexm
