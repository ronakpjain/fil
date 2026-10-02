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
    struct IdempotentPeriodCertificate;
public:
    struct TransactionCheckpoint;

    /**
     * @brief Admission certificate for deferred execution of a pure CPU prefix.
     *
     * Admission prepares and certifies code but does not execute the CPU or
     * advance board time. Until materialization, the canonical CPU state is
     * stale relative to the scheduler's deferred logical progress. Callers
     * must materialize (or discard) before observing CPU state or dispatching
     * an impure operation/callback. Completion timestamps are absolute and
     * strictly increasing.
     */
    struct DeferredPurePrefix {
        static constexpr std::size_t max_instructions = 64U;
        SimTimeNs start_time_ns{0};
        std::uint8_t count{0};
        std::array<SimTimeNs, max_instructions> completion_times_ns{};
        std::uint32_t entry_pc{0};
        std::uint64_t flash_generation{0};
        std::uint64_t execution_generation{0};
        std::uint64_t clock_hz{0};
        std::uint64_t time_fraction{0};
        std::array<std::uint64_t, max_instructions> cumulative_cycles{};
    };

    /**
     * @brief Reversible private-RAM experiment; speculative CPU/RAM is not observable.
     *
     * Only fast integer/backed-memory handlers run, with MMIO trapped and
     * non-executable RAM-only stores. Every event, shared delivery and public
     * observation must materialize before inspecting or mutating this board.
     */
    struct ReversibleRamPrefix : DeferredPurePrefix {
        static constexpr std::size_t max_instructions = 64U;
        struct ReversibleExecution {
            cpu::FastStepResult result{};
            std::array<std::uint32_t, max_instructions> pcs{};
            std::array<std::uint8_t, max_instructions> sizes{};
            std::array<std::uint16_t, max_instructions> instruction_cycles{};
            std::uint8_t count{0U};
        };
        // FP instructions are excluded from the capsule, so copying the FP
        // register bank on every admission is unnecessary. Keep every integer
        // field, including exception requests, for exact interrupted replay.
        struct IntegerSnapshot {
            std::array<std::uint32_t, 16> r{};
            std::uint32_t xpsr{0}, msp{0}, psp{0};
            std::uint32_t primask{0}, basepri{0}, faultmask{0}, control{0};
            std::uint32_t instruction_address{0};
            bool thumb{true}, halted{false};
            std::uint8_t it_state{0};
            std::optional<std::uint16_t> pending_exception;
            std::optional<std::uint32_t> pending_exc_return;
            void capture(const cpu::CpuState& state) noexcept {
                r = state.r;
                xpsr = state.xpsr; msp = state.msp; psp = state.psp;
                primask = state.primask; basepri = state.basepri;
                faultmask = state.faultmask; control = state.control;
                instruction_address = state.instruction_address;
                thumb = state.thumb; halted = state.halted; it_state = state.it_state;
                pending_exception = state.pending_exception;
                pending_exc_return = state.pending_exc_return;
            }
            [[nodiscard]] bool matches(const cpu::CpuState& state) const noexcept {
                return r == state.r && xpsr == state.xpsr && msp == state.msp && psp == state.psp
                    && primask == state.primask && basepri == state.basepri
                    && faultmask == state.faultmask && control == state.control
                    && instruction_address == state.instruction_address
                    && thumb == state.thumb && halted == state.halted && it_state == state.it_state
                    && pending_exception == state.pending_exception
                    && pending_exc_return == state.pending_exc_return;
            }
            void restore(cpu::CpuState& state) const noexcept {
                state.r = r;
                state.xpsr = xpsr; state.msp = msp; state.psp = psp;
                state.primask = primask; state.basepri = basepri;
                state.faultmask = faultmask; state.control = control;
                state.instruction_address = instruction_address;
                state.thumb = thumb; state.halted = halted; state.it_state = it_state;
                state.pending_exception = pending_exception;
                state.pending_exc_return = pending_exc_return;
            }
        };
        IntegerSnapshot entry_state{};
        mem::MemoryBus::SideEffectCheckpoint memory_checkpoint{};
        mem::MemoryBus::SideEffectCheckpoint evaluated_checkpoint{};
        mem::MemoryBus::ReadFootprint entry_read_footprint{};
        ReversibleExecution evaluated{};
        std::uint8_t folded_instructions{0U};
        bool entry_have_fetch{false};
        std::uint32_t entry_fetch_end{0};
        std::shared_ptr<const IdempotentPeriodCertificate> period_certificate;
        std::uint8_t period_phase{0U};
        std::uint8_t memoized_count{0U};
    };

    /** Result of executing and charging one (possibly batched) board prefix. */
    struct ConcurrentStepResult {
        cpu::FastStepResult cpu_result;
        SimTimeNs elapsed_ns{0};
    };
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

    /**
     * @brief Certifies a deferred, memory-free fixed-cycle instruction prefix.
     *
     * Returns no certificate unless at least two instructions can complete
     * strictly before the next scheduled event, SysTick, and optional run
     * deadline. This only prepares/peeks JIT metadata: CPU registers, cycles,
     * and simulated time are unchanged.
     */
    [[nodiscard]] std::optional<DeferredPurePrefix> prepareDeferredPurePrefix(
        std::size_t max_instructions, std::optional<SimTimeNs> deadline = std::nullopt);

    /**
     * @brief Materializes the requested leading instructions of a certificate.
     *
     * Execute only after any observation barrier has selected a count no larger
     * than the certificate. A stale/mismatched certificate returns an empty
     * result and does not charge board time. CPU observations before this call
     * see the pre-prefix canonical state, not deferred progress.
     */
    [[nodiscard]] ConcurrentStepResult materializeDeferredPurePrefix(
        const DeferredPurePrefix& prefix, std::size_t count);

    /**
     * @brief Speculates up to 192 reversible RAM/ALU instructions without advancing timers.
     *
     * Chains prepared CPU fast blocks across taken/not-taken integer branches,
     * stopping at boundaries or when a later block is not reversible/ready.
     */
    [[nodiscard]] std::optional<ReversibleRamPrefix> prepareReversibleRamPrefix(
        std::size_t max_instructions, std::optional<SimTimeNs> deadline = std::nullopt);
    /** @brief Fills reusable scheduler-owned storage, avoiding certificate copies. */
    [[nodiscard]] bool prepareReversibleRamPrefix(
        ReversibleRamPrefix& out, std::size_t max_instructions,
        std::optional<SimTimeNs> deadline = std::nullopt,
        bool allow_single_prefix = false);

    /** @brief Commits the started prefix, restoring/replaying an interrupted suffix. */
    [[nodiscard]] ConcurrentStepResult materializeReversibleRamPrefix(
        const ReversibleRamPrefix& prefix, std::size_t count);

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
    [[nodiscard]] cpu::FastStepResult stepSingleWithFetchTiming(bool allow_jit);
    [[nodiscard]] cpu::FastStepResult stepBlockWithFetchTiming(
        bool allow_jit_block, std::optional<SimTimeNs> deadline,
        std::size_t max_instructions, bool block_prevalidated);
    /** Executes one instruction and accrues board-local cycles without moving shared time. */
    [[nodiscard]] ConcurrentStepResult beginConcurrentStep(
        bool trace_instructions, bool allow_jit = false,
        std::size_t max_instructions = 1U, bool block_prevalidated = true);
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
    struct PredictedBlockCosts {
        std::array<PredictedBlockCost, cpu::CortexM4::JitStepOutcome::max_block> prefixes{};
        std::uint8_t count{0U};
    };
    /** Pure, exact prefixes that finish before every observable boundary. */
    [[nodiscard]] PredictedBlockCosts peekPredictedBlockCosts(
        std::size_t max_instructions, std::optional<SimTimeNs> deadline);
    /** Applies exception/reset effects due at the just-completed instruction boundary. */
    [[nodiscard]] bool boundaryWorkPending() const noexcept;
    /**
     * @brief Whether JIT block batching must defer at this boundary.
     *
     * Mirrors settle's own entry predicate: only a takable exception
     * (per PRIMASK/BASEPRI/FAULTMASK/active priority, as enterPending
     * evaluates it), a pended return, or a reset request blocks batching.
     * Merely masked or lower-priority pending interrupts do not: no
     * admitted block op can change masking (MSR/CPS never compile into
     * blocks) or take an exception early, so batching past them is exact.
     */
    [[nodiscard]] bool blockBoundaryPending() const noexcept;
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
    [[nodiscard]] std::shared_ptr<const IdempotentPeriodCertificate> buildIdempotentPeriodCertificate(
        std::size_t period, std::uint64_t period_cycles);
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
    // Bounded to the most recently certified natural period.
    std::shared_ptr<const IdempotentPeriodCertificate> idempotent_period_cache_;
    mem::MemoryBus::SideEffectCheckpoint period_validation_checkpoint_{};
    std::uint64_t period_validation_restoration_generation_{0U};
    std::uint64_t loop_observation_generation_{1U};
    std::optional<std::uint32_t> read_footprint_boundary_;
    // Cached real-timing inputs (clock + flash ACR) with fetch-sequencing
    // for the simplified ART model. Refreshed on change, not per reset.
    std::uint64_t cached_clock_hz_{0};
    /// Divide-free cycles->ns conversion: (c*1e9)/F split into quotient and
    /// remainder per cycle count. Remainder plus carry is always < 2*F, so
    /// the per-step carry resolves with one compare (exact, no division).
    // Cover complete 64-instruction capsule spans as well as single steps;
    // the conversion remains exact, with division retained for larger jumps.
    static constexpr std::size_t cycle_table_size = 1024U;
    struct CycleTableEntry { std::uint64_t quotient{0}; std::uint64_t remainder{0}; };
    mutable std::array<CycleTableEntry, cycle_table_size> cycle_table_{};
    mutable std::uint64_t cycle_table_hz_{0};
    void rebuildCycleTable(std::uint64_t frequency) const;
    std::uint64_t cached_flash_acr_generation_{0};
    std::uint32_t cached_flash_ws_{0};
    bool cached_flash_art_hit_capable_{false};
    std::uint32_t last_fetch_end_{0};
    bool have_last_fetch_{false};
};

/** @brief Stable lowercase stop-reason name for CLI/trace output. */
[[nodiscard]] std::string_view boardStopReasonName(BoardStopReason reason) noexcept;

} // namespace fil::sim
