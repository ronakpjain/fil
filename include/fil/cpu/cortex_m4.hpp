#pragma once

/** @file cortex_m4.hpp
 *  @brief Cortex-M4 architectural state and bounded Thumb execution API.
 */

#include "fil/cpu/instruction.hpp"
#include "fil/mem/address.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace fil::elf {
class ElfImage;
}

namespace fil::mem {
class MemoryBus;
}

namespace fil::cpu {

inline constexpr std::uint32_t xpsr_n = 1U << 31U;
inline constexpr std::uint32_t xpsr_z = 1U << 30U;
inline constexpr std::uint32_t xpsr_c = 1U << 29U;
inline constexpr std::uint32_t xpsr_v = 1U << 28U;
inline constexpr std::uint32_t xpsr_q = 1U << 27U;
inline constexpr std::uint32_t xpsr_t = 1U << 24U;
inline constexpr std::uint32_t xpsr_ge_mask = 0x000f0000U;
inline constexpr std::uint32_t xpsr_ipsr_mask = 0x1ffU;

/** @brief Complete integer and FPv4-SP-D16 architectural state. */
struct CpuState {
    std::array<std::uint32_t, 16> r{};
    std::uint32_t xpsr{xpsr_t};

    std::uint32_t msp{0};
    std::uint32_t psp{0};
    std::uint32_t primask{0};
    std::uint32_t basepri{0};
    std::uint32_t faultmask{0};
    std::uint32_t control{0};

    bool thumb{true};
    bool halted{false};

    /** @brief Exception request raised by the just-executed instruction. */
    std::optional<std::uint16_t> pending_exception;
    /** @brief Architectural EXC_RETURN requested by BX/POP/LDM. */
    std::optional<std::uint32_t> pending_exc_return;

    std::array<float, 32> s{};
    std::uint32_t fpscr{0};
    std::uint8_t it_state{0};

    std::uint32_t instruction_address{0};

    /** @brief Restores Cortex-M reset state from an already validated ELF image. */
    [[nodiscard]] bool reset(const elf::ElfImage& image) noexcept;

    /** @brief Returns the exception number encoded in xPSR.IPSR. */
    [[nodiscard]] std::uint16_t ipsr() const noexcept;

    /** @brief Returns true while executing in handler mode. */
    [[nodiscard]] bool inHandlerMode() const noexcept;

    /** @brief Selects MSP or PSP according to mode and CONTROL.SPSEL. */
    [[nodiscard]] std::uint32_t& activeSp() noexcept;
    [[nodiscard]] const std::uint32_t& activeSp() const noexcept;

    /** @brief Reads a core register using architectural SP/PC behavior. */
    [[nodiscard]] std::uint32_t readRegister(std::uint8_t index) const noexcept;

    /** @brief Writes a non-PC core register and synchronizes visible r13. */
    void writeRegister(std::uint8_t index, std::uint32_t value) noexcept;

    /** @brief Gets the address of the instruction currently being executed. */
    [[nodiscard]] std::uint32_t currentInstrAddr() const noexcept;

    /** @brief Gets the normal Thumb architectural PC read value. */
    [[nodiscard]] std::uint32_t architecturalPcForRead() const noexcept;

    /** @brief Exchanges into a loaded branch target, validating the Thumb bit. */
    [[nodiscard]] bool branchWritePc(std::uint32_t target) noexcept;

    /** @brief Installs/advances IT state and mirrors it into xPSR.EPSR. */
    void setItState(std::uint8_t value) noexcept;
    void advanceIt() noexcept;
};

/** @brief Stable reason a one-step or bounded execution request stopped. */
enum class StopReason : std::uint8_t {
    step_complete,
    instruction_budget,
    breakpoint,
    halted,
    bus_fault,
    synchronization_required,
    undefined_instruction,
    invalid_state,
};

/** @brief Register and instruction snapshot captured at an execution boundary. */
struct DiagnosticSnapshot {
    std::uint32_t instruction_address{0};
    std::uint32_t next_pc{0};
    std::uint32_t raw{0};
    std::uint8_t instruction_size{0};
    std::array<std::uint32_t, 16> registers{};
    std::uint32_t xpsr{0};
    std::optional<mem::BusFault> bus_fault;
    std::string message;
};

/** @brief Result of one bounded CPU execution request. */
struct RunResult {
    StopReason reason{StopReason::instruction_budget};
    std::uint64_t instructions{0};
    std::uint64_t cycles{0};
    DiagnosticSnapshot diagnostic;

    /** @brief True when no architectural/decoder/bus failure stopped execution. */
    [[nodiscard]] bool succeeded() const noexcept;
};

/** @brief Allocation-free result for the successful per-instruction hot path. */
struct FastStepResult {
    StopReason reason{StopReason::step_complete};
    std::uint32_t instruction_address{0};
    std::uint32_t raw{0};
    std::uint8_t instruction_size{0};
    std::uint8_t instructions{0};
    std::uint8_t cycles{0};
    bool suppress_loop_observation{false};
};

/** @brief Minimal deterministic Cortex-M4 Thumb interpreter. */
class CortexM4 {
public:
    explicit CortexM4(mem::MemoryBus& memory) noexcept;

    /** @brief Gets mutable architectural state for inspection or setup. */
    [[nodiscard]] CpuState& state() noexcept { return state_; }
    /** @brief Gets immutable architectural state. */
    [[nodiscard]] const CpuState& state() const noexcept { return state_; }

    /** @brief Resets the core from ELF vector metadata. */
    [[nodiscard]] bool reset(const elf::ElfImage& image) noexcept;

    /** @brief Fetches, decodes, and executes at most one instruction. */
    [[nodiscard]] RunResult step();

    /**
     * @brief Executes one instruction without materializing a full success snapshot.
     *
     * When this returns anything other than `step_complete`, `lastDiagnostic()`
     * contains the same structured failure information returned by `step()`.
     */
    [[nodiscard]] FastStepResult stepFast();

    /** @brief Gets the diagnostic captured by the most recent failed fast step. */
    [[nodiscard]] const DiagnosticSnapshot& lastDiagnostic() const noexcept {
        return last_diagnostic_;
    }

    /** @brief Executes until budget exhaustion, breakpoint, halt, or a fault. */
    [[nodiscard]] RunResult run(std::uint64_t instruction_budget);

private:
    struct InstructionCacheEntry {
        std::uint64_t generation{0};
        std::uint32_t pc{0};
        std::uint32_t raw{0};
        DecodedInstruction decoded{};
        std::uint8_t size{0};
    };

    static constexpr std::size_t instruction_cache_entries = 16384U;

    [[nodiscard]] StopReason execute(
        const DecodedInstruction& instruction,
        DiagnosticSnapshot& diagnostic
    );
    void capture(DiagnosticSnapshot& diagnostic) const;

    mem::MemoryBus& memory_;
    CpuState state_{};
    std::array<InstructionCacheEntry, instruction_cache_entries> instruction_cache_{};
    DiagnosticSnapshot last_diagnostic_{};
};

/** @brief Stable lowercase name for diagnostics and tests. */
[[nodiscard]] const char* stopReasonName(StopReason reason) noexcept;

} // namespace fil::cpu
