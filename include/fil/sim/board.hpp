#pragma once

/** @file board.hpp
 *  @brief One deterministic MCU/firmware instance and bounded execution loop.
 */

#include "fil/common/result.hpp"
#include "fil/config/config.hpp"
#include "fil/cpu/cortex_m4.hpp"
#include "fil/elf/elf_loader.hpp"
#include "fil/mem/memory_bus.hpp"
#include "fil/sim/event_loop.hpp"
#include "fil/sim/trace.hpp"

#include <cstdint>
#include <array>
#include <memory>
#include <optional>
#include <string>

namespace fil::cortexm {
class ExceptionController;
class SystemControl;
}
namespace fil::stm32g4 {
class Stm32G4;
}

namespace fil::sim {

class World;

/** @brief Stable board-level reason a bounded simulation stopped. */
enum class BoardStopReason : std::uint8_t {
    target_reached,
    breakpoint,
    halted,
    instruction_budget,
    time_budget,
    spin_detected,
    unimplemented_instruction,
    architectural_fault,
    reset_requested,
    synchronization_required,
    host_error,
};

/** @brief User-selected run limits and optional diagnostics. */
struct BoardRunOptions {
    std::uint64_t max_instructions{50'000'000}; ///< Zero means no instructions may execute.
    SimTimeNs duration_ns{1'000'000'000};       ///< Zero disables simulated-time limiting.
    std::optional<std::uint32_t> stop_address;  ///< Stop before executing this even Thumb address.
    bool trace_instructions{false};              ///< Emit one trace record per instruction.
    bool detect_spin{false};                     ///< Stop on a proven exact-state loop.
    std::uint64_t spin_threshold{1'000'000};     ///< Logical instructions in a repeated exact-state loop.
    bool enable_loop_batching{true};             ///< Fast-forward proven side-effect-free loops.
    bool enable_jit{false};                      ///< Opt-in cached hot-path compilation.
    unsigned int adc_decimation{1}; ///< Keep 1 of N continuous ADC scans; higher skips side effects.
};

/** @brief Aggregate result and final CPU diagnostic for a board run. */
struct BoardRunResult {
    BoardStopReason reason{BoardStopReason::instruction_budget};
    std::uint64_t instructions{0};
    std::uint64_t cycles{0};
    SimTimeNs time_ns{0};
    cpu::DiagnosticSnapshot diagnostic;
    std::string message;

    /** @brief Whether the reason is a requested/normal execution boundary. */
    [[nodiscard]] bool succeeded() const noexcept;
};

/** @brief Loaded and wired firmware board instance. */
class Board {
public:
    struct TransactionCheckpoint;
    using TransactionCheckpointPtr = std::shared_ptr<const TransactionCheckpoint>;

    /** @brief Loads config references, ELF, MCU map, CPU, and peripherals. */
    [[nodiscard]] static Result<std::unique_ptr<Board>> load(
        const config::BoardConfig& config,
        bool strict_mmio = false,
        EventLoop* shared_event_loop = nullptr,
        TraceRecorder* shared_trace = nullptr
    );

    ~Board();
    Board(const Board&) = delete;
    Board& operator=(const Board&) = delete;

    /** @brief Rebuilds reset memory and resets CPU/system/peripheral state. */
    [[nodiscard]] Result<void> reset();

    /** @brief Runs until a configured boundary or architectural failure. */
    [[nodiscard]] BoardRunResult run(const BoardRunOptions& options);

    /** @brief Runs one lane using only owner-local events until shared synchronization. */
    [[nodiscard]] BoardRunResult runWorkerSlice(
        EventOwner owner,
        std::uint64_t instruction_budget,
        SimTimeNs deadline_ns,
        bool enable_loop_batching = true,
        bool trap_all_mmio = false,
        bool enable_jit = false
    );

    /** @brief Captures reversible CPU, RAM, system, scheduler, and lane-clock state. */
    [[nodiscard]] TransactionCheckpointPtr captureTransaction(EventOwner owner) const;

    /** @brief Restores a checkpoint when no MMIO or owner event escaped the slice. */
    [[nodiscard]] bool restoreTransaction(const TransactionCheckpointPtr& checkpoint);

    [[nodiscard]] cpu::CortexM4& cpu() noexcept { return *cpu_; }
    [[nodiscard]] const cpu::CortexM4& cpu() const noexcept { return *cpu_; }
    /** @brief Gets mutable target memory for controlled inspection or injection. */
    [[nodiscard]] mem::MemoryBus& memory() noexcept { return memory_; }
    /** @brief Gets read access to the target memory map. */
    [[nodiscard]] const mem::MemoryBus& memory() const noexcept { return memory_; }
    [[nodiscard]] const elf::ElfImage& image() const noexcept { return image_; }
    [[nodiscard]] EventLoop& eventLoop() noexcept { return *event_loop_; }
    [[nodiscard]] TraceRecorder& trace() noexcept { return *trace_; }
    [[nodiscard]] stm32g4::Stm32G4& peripherals() noexcept { return *peripherals_; }
    [[nodiscard]] const config::BoardConfig& config() const noexcept { return config_; }

private:
    friend class World;

    struct ConcurrentStepResult {
        cpu::FastStepResult cpu_result;
        SimTimeNs elapsed_ns{0};
    };

    struct BoundaryStop {
        BoardStopReason reason{BoardStopReason::host_error};
        std::string message;
    };

    struct ProvenLoop {
        std::uint32_t boundary_pc{0};
        std::uint64_t instructions_per_iteration{0};
        std::uint64_t cycles_per_iteration{0};
        mem::MemoryBus::SideEffectCheckpoint side_effect_checkpoint{};
        std::uint16_t observation_index{0};
        std::uint64_t observation_revision{0};
        mem::MemoryBus::ReadFootprint read_footprint{};
        bool read_footprint_complete{false};
        // Real-timing proof inputs: batching multiplies a measured
        // cycles_per_iteration, so a clock or FLASH_ACR change must fail
        // the proof closed rather than replay stale cycle counts.
        std::uint64_t clock_hz{0};
        std::uint64_t flash_acr_generation{0};
    };

    struct LoopSkip {
        std::uint64_t instructions{0};
        std::uint64_t cycles{0};
        SimTimeNs elapsed_ns{0};
    };

    struct LoopObservation {
        bool valid{false};
        std::uint64_t generation{0};
        std::uint64_t revision{0};
        std::uint32_t boundary_pc{0};
        cpu::CpuState state{};
        mem::MemoryBus::SideEffectCheckpoint side_effect_checkpoint{};
        std::uint64_t instructions{0};
        std::uint64_t cycles{0};
        std::uint64_t clock_hz{0};
        std::uint64_t flash_acr_generation{0};
    };

    Board(
        config::BoardConfig config,
        config::McuConfig mcu,
        elf::ElfImage image,
        EventLoop* shared_event_loop,
        TraceRecorder* shared_trace
    );
    [[nodiscard]] Result<void> initialize(bool strict_mmio);
    [[nodiscard]] SimTimeNs accountCycles(std::uint64_t cycles);
    [[nodiscard]] SimTimeNs advanceTime(std::uint64_t cycles);
    /**
     * @brief Steps once and adds the simplified-ART flash fetch stall.
     *
     * The CPU returns pipeline cycles (variable per DDI0439C class);
     * this wrapper adds 0-LATENCY flash stall cycles based on whether the
     * fetch address is sequential to the previous fetch and on the cached
     * FLASH_ACR prefetch/cache enables. The stall is folded into
     * `result.cycles` so instruction counters, loop batching, and the
     * world scheduler all observe final cycle counts. Clock/ACR values are
     * cached with generation checks to keep the per-instruction overhead
     * to a few loads and compares.
     */
    [[nodiscard]] cpu::FastStepResult stepWithFetchTiming(
        bool allow_jit_block = false,
        std::optional<SimTimeNs> deadline = std::nullopt,
        std::size_t max_instructions = cpu::CortexM4::JitStepOutcome::max_block,
        bool block_prevalidated = false);
    /** Executes one instruction and accrues board-local cycles without moving shared time. */
    [[nodiscard]] ConcurrentStepResult beginConcurrentStep(
        bool trace_instructions, bool allow_jit = false,
        std::size_t max_instructions = 1U);
    /** @brief Exact next-instruction cost inputs for the burst gate. */
    struct PredictedCost {
        std::uint64_t cycles{0};    ///< Pipeline + ART flash stall cycles.
        std::uint64_t frequency{0}; ///< Current SYSCLK Hz.
        std::uint64_t fraction{0};  ///< Fractional-ns accumulator.
    };

    /**
     * @brief Exact predicted next-instruction cost, if knowable.
     *
     * Used by the world lockstep-burst gate: unlike
     * `nextInstructionElapsedNs()` (a 1-cycle minimum for deadline fits),
     * this forecasts the real cost (pipeline class + exact ART flash stall
     * from live ACR and fetch sequencing). Returns nullopt for
     * memory-loaded targets, decode-cache misses, or fault states, in
     * which case the burst fails closed to the exact general scheduler.
     * Predictions are exact, so no post-step rollback is ever required.
     * The gate compares frequencies and time numerators across lanes and
     * performs a single ns division per burst round.
     */
    [[nodiscard]] std::optional<PredictedCost> peekPredictedCost() const noexcept;
    struct PredictedBlockCost {
        PredictedCost cost;
        std::uint8_t instructions{0U};
    };
    /** Pure, fixed-cost blocks that finish before every observable boundary. */
    [[nodiscard]] std::optional<PredictedBlockCost> peekPredictedBlockCost(
        std::size_t max_instructions, std::optional<SimTimeNs> deadline);
    /** Applies exception/reset effects due at the just-completed instruction boundary. */
    [[nodiscard]] bool boundaryWorkPending() const noexcept;
    [[nodiscard]] std::optional<BoundaryStop> settleInstructionBoundary();
    [[nodiscard]] std::optional<BoundaryStop> settleInstructionBoundarySlow();
    [[nodiscard]] std::optional<ProvenLoop> observeLoopBoundary(
        const cpu::FastStepResult& step,
        std::uint64_t logical_instructions,
        std::uint64_t logical_cycles
    );
    [[nodiscard]] std::uint64_t maximumLoopIterations(
        const ProvenLoop& loop,
        std::uint64_t instruction_budget,
        std::optional<SimTimeNs> horizon_ns
    ) const;
    /** Applies iterations already bounded by maximumLoopIterations(). */
    [[nodiscard]] LoopSkip applyLoopIterations(
        const ProvenLoop& loop, std::uint64_t validated_iterations
    );
    [[nodiscard]] bool loopProofStillValid(const ProvenLoop& loop) const noexcept;
    [[nodiscard]] bool loopHasNoMmioSince(const ProvenLoop& loop) const noexcept;
    [[nodiscard]] std::optional<SimTimeNs> nextObservableTime(SimTimeNs boundary_time) const;
    [[nodiscard]] SimTimeNs nextInstructionElapsedNs() const noexcept {
        return elapsedForCycles(1U);
    }
    void refreshLoopObservation(
        const ProvenLoop& loop, std::uint64_t logical_instructions, std::uint64_t logical_cycles
    );
    void invalidateLoopObservations() noexcept;
    [[nodiscard]] BoardRunResult cpuFailure(const cpu::RunResult& result) const;
    [[nodiscard]] BoardRunResult cpuFailure(const cpu::FastStepResult& result) const;
    [[nodiscard]] SimTimeNs elapsedForCycles(std::uint64_t cycles) const noexcept;

    config::BoardConfig config_;
    config::McuConfig mcu_config_;
    elf::ElfImage image_;
    std::unique_ptr<EventLoop> owned_event_loop_;
    std::unique_ptr<TraceRecorder> owned_trace_;
    EventLoop* event_loop_{nullptr};
    TraceRecorder* trace_{nullptr};
    std::unique_ptr<cortexm::SystemControl> system_;
    std::unique_ptr<stm32g4::Stm32G4> peripherals_;
    mem::MemoryBus memory_;
    std::unique_ptr<cpu::CortexM4> cpu_;
    std::unique_ptr<cortexm::ExceptionController> exceptions_;
    std::uint64_t time_fraction_{0};
    std::array<LoopObservation, 256> loop_observations_{};
    std::uint64_t loop_observation_generation_{1U};
    std::optional<std::uint32_t> read_footprint_boundary_;
    // Cached real-timing inputs (clock + flash ACR) with fetch-sequencing
    // for the simplified ART model. Refreshed on change, not per reset.
    std::uint64_t cached_clock_hz_{0};
    std::uint64_t cached_flash_acr_generation_{0};
    std::uint32_t cached_flash_ws_{0};
    bool cached_flash_art_hit_capable_{false};
    std::uint32_t last_fetch_end_{0};
    bool have_last_fetch_{false};
};

/** @brief Stable lowercase stop-reason name for CLI/trace output. */
[[nodiscard]] std::string_view boardStopReasonName(BoardStopReason reason) noexcept;

} // namespace fil::sim
