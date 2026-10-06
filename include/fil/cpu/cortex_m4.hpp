#pragma once

/** @file cortex_m4.hpp
 *  @brief Cortex-M4 architectural state and bounded Thumb execution API.
 */

#include "fil/cpu/instruction.hpp"
#include "fil/mem/address.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <memory>
#include <string>

namespace fil::elf {
class ElfImage;
}

namespace fil::mem {
class MemoryBus;
}

namespace fil::cortexm {
class SystemControl;
}

namespace fil::cpu {

class NativeJitKernel;

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

    /**
     * @brief Pending lazy FP stacking reservation (LSPEN-style).
     *
     * Set on exception entry when the incoming task owns FP state but the
     * model defers pushing S0-S15/FPSCR until handler code first touches
     * the FP register file. fp_lazy_base is the reserved area base and is
     * valid only while fp_lazy_active. A nested entry while active falls
     * back to eager stacking for that level (always memory-safe; exact
     * except for FP use across more than one lazy nesting level, for which
     * the core manual's nesting rule would require ARM ARM text to encode).
     */
    bool fp_lazy_active{false};
    std::uint32_t fp_lazy_base{0};

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

/** @brief Compares complete CPU state by stored representation, including FP bits. */
[[nodiscard]] bool bitwiseEqual(
    const CpuState& left,
    const CpuState& right
) noexcept;

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
    // Keep the successful hot result compact; the instruction counter is wide
    // enough for compressed period materialization.
    StopReason reason{StopReason::step_complete};
    bool suppress_loop_observation{false};
    std::uint8_t instruction_size{0};
    std::uint32_t instructions{0};
    std::uint32_t instruction_address{0};
    std::uint32_t raw{0};
    // Widened from 8 to 16 bits: realistic pipeline + flash-stall totals
    // reach ~21 cycles (e.g. 16-register LDM to PC with wait states).
    std::uint16_t cycles{0};
};

/** @brief Minimal deterministic Cortex-M4 Thumb interpreter. */
class CortexM4 {
public:
    explicit CortexM4(mem::MemoryBus& memory);
    ~CortexM4();
    CortexM4(const CortexM4&) = delete;
    CortexM4& operator=(const CortexM4&) = delete;

    /** Whether this build includes the LLVM host-machine-code backend. */
    [[nodiscard]] static bool nativeJitAvailable() noexcept;

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

    /** @brief Executes one cached, specialized instruction or interprets exactly one step. */
    [[nodiscard]] FastStepResult stepJitFast();

    /**
     * @brief Enables LLVM-compiled one-instruction kernels for direct CPU stepping.
     *
     * Multi-instruction native blocks remain independently available. The
     * multi-board runner disables single kernels by default because measured
     * real-firmware runs did not recover their dispatch/codegen overhead.
     */
    void setNativeSingleInstructionJitEnabled(bool enabled) noexcept {
        native_single_instruction_jit_enabled_ = enabled;
    }

    /** @brief Gets the diagnostic captured by the most recent failed fast step. */
    [[nodiscard]] const DiagnosticSnapshot& lastDiagnostic() const noexcept {
        return last_diagnostic_;
    }

    /** @brief Copies current architectural state into a diagnostic snapshot. */
    void captureDiagnostic(DiagnosticSnapshot& diagnostic) const;

    /**
     * @brief Peeks the exact pipeline cycles of the current instruction.
     *
     * Succeeds only on a decode-cache hit for a non-control-flow form
     * (no possible PC discontinuity): callers get the same base cost the
     * stepper will charge, including data-dependent DIV and failed-
     * condition short-circuiting. Control flow, cache misses, and fault
     * paths return false so burst prediction fails closed to the exact
     * general scheduler.
     */
    [[nodiscard]] bool peekPredictableCycles(std::uint16_t& cycles_out) const noexcept;

    /** @brief Executes until budget exhaustion, breakpoint, halt, or a fault. */
    [[nodiscard]] RunResult run(std::uint64_t instruction_budget);

    /** @brief Aggregated multi-instruction step produced by the hot-path JIT. */
    struct JitStepOutcome {
        FastStepResult result{};
        // Per-instruction PCs/sizes for host fetch-stall accounting.
        static constexpr std::size_t max_block = 16U;
        std::array<std::uint32_t, max_block> pcs{};
        std::array<std::uint8_t, max_block> sizes{};
        std::uint8_t count{0};
        /// True when no executed op touched memory (ACR stable mid-block).
        bool memory_free{true};
    };

    /** @brief Read-only metadata and conservative CPU-cycle bound for a JIT prefix. */
    struct JitBlockPreview {
        std::array<std::uint32_t, JitStepOutcome::max_block> pcs{};
        std::array<std::uint8_t, JitStepOutcome::max_block> sizes{};
        std::array<std::uint16_t, JitStepOutcome::max_block> instruction_cycles{};
        std::uint8_t count{0};
        std::uint8_t exact_cycle_prefix_count{0};
        std::uint64_t max_cycles{0};
        /** True only for a memory-free, non-faulting prefix with fixed cycle cost. */
        bool cycles_exact{true};
    };

    /** @brief Fast-handler-only execution with exact per-committed-op pipeline costs. */
    struct TimedJitStepOutcome {
        JitStepOutcome execution{};
        std::array<std::uint16_t, JitStepOutcome::max_block> instruction_cycles{};
    };

    /**
     * @brief Caller-owned conservative cycle credit and fetch-stall state.
     *
     * The optional callback is called before each candidate instruction and
     * must be pure/stable for the duration of the attempt. A null callback
     * means zero additional fetch cycles.
     */
    struct ReversibleCycleBudget {
        using FetchStallCallback = std::uint16_t (*)(
            const void* context, std::uint32_t pc, bool sequential);
        std::uint64_t remaining_cycles{0U};
        FetchStallCallback fetch_stall{nullptr};
        const void* context{nullptr};
        bool have_fetch{false};
        std::uint32_t fetch_end{0U};
        std::array<std::uint16_t, JitStepOutcome::max_block>
            total_instruction_cycles{};
    };

    /**
     * @brief Tries to execute a cached hot-path block (0 = fallback).
     *
     * Returns nullopt when no compiled block covers the current PC or the
     * architectural state forbids block execution (IT active, pending
     * exception, invalid Thumb state). Otherwise executes the block with
     * exact per-instruction semantics, precise MMIO-restart partial counts,
     * and fault diagnostics identical to repeated stepFast() calls.
     */
    [[nodiscard]] std::optional<JitStepOutcome> tryStepJitBlock(
        std::size_t max_instructions = JitStepOutcome::max_block);

    /**
     * @brief Executes a block prepared by prepareAndPeekJitBlock().
     *
     * Same as tryStepJitBlock() but skips hotness/validity probing: the
     * caller prepared the entry moments ago on this thread with no
     * intervening CPU/memory change (board-side flash/event queries only).
     * A defensive slot recheck still guards against misuse.
     */
    [[nodiscard]] std::optional<JitStepOutcome> tryStepPreparedJitBlock(
        std::size_t max_instructions = JitStepOutcome::max_block);

    /**
     * @brief Prepares and executes only cached fast handlers, with exact costs.
     *
     * Stops before generic operations and at fast-handler declines, returning
     * only the committed prefix. Intended for a caller that provides rollback
     * for CPU and RAM state and enforces its MMIO/store restrictions externally.
     * No peripheral timing is included; returned costs are CPU pipeline cycles.
     */
    [[nodiscard]] std::optional<TimedJitStepOutcome> tryStepReversibleJitBlock(
        std::size_t max_instructions = JitStepOutcome::max_block);

    /** @brief Executes a guarded reversible prefix without exceeding cycle credit. */
    [[nodiscard]] std::optional<TimedJitStepOutcome> tryStepBudgetedReversibleJitBlock(
        std::size_t max_instructions, ReversibleCycleBudget& budget);

    /**
     * @brief Predicts, without executing or mutating state, whether a trusted
     * reversible block attempt at the current PC would execute no instruction.
     *
     * Replicates the entry checks and head-operation admission cost of
     * executeTrustedReversibleJitBlock, including the runtime fast-path
     * declines of register-indirect word load/store whose effective address
     * cannot be served by the backed-memory fast path (MMIO, faults, or, for
     * stores, the reversible-RAM restriction). Callers use it to skip
     * reversible admission setup that would otherwise roll back unchanged.
     */
    [[nodiscard]] bool peekReversibleBlockHeadDeclines(
        const ReversibleCycleBudget& budget) noexcept;

    /**
     * @brief Previews a ready block prefix without executing it.
     *
     * Returns nullopt for unavailable blocks, zero-length requests, or when
     * the first selected instruction is not block-admitted (system/FP forms
     * or memory without a fast handler). Otherwise previews the maximal
     * admittable prefix; non-handler memory and control flow terminate it.
     * max_cycles includes maximum modeled divide latency and the possible
     * control-flow refill.
     */
    [[nodiscard]] std::optional<JitBlockPreview> peekJitBlock(
        std::size_t max_instructions = JitStepOutcome::max_block) const noexcept;

    /**
     * @brief Prepares a block and fills its preview in a single slot lookup.
     *
     * Hot-path fusion of prepareJitBlock() + peekJitBlock(): advances
     * hotness/compilation once, then fills the caller-owned preview from
     * compile-time data without big-struct returns. Returns false (preview
     * untouched) when no block is ready or the admitted prefix is empty.
     *
     * The optimistic variant admits handler-covered transfers for the
     * board horizon gate (backed RAM executes inline; MMIO/faults stop
     * the block precisely). The exact variant stops before any memory op
     * for synchronized-burst prediction.
     */
    [[nodiscard]] bool prepareAndPeekJitBlock(
        std::size_t max_instructions, JitBlockPreview& preview_out);
    [[nodiscard]] bool prepareAndPeekExactJitBlock(
        std::size_t max_instructions, JitBlockPreview& preview_out);

    /**
     * @brief Whether a compiled JIT block covers the current PC.
     *
     * Cheap lockstep probe for the world burst gate: true only when a
     * cached block for the current entry PC is valid for the current
     * execution generation and block execution is architecturally allowed
     * (Thumb state, no active IT block, no pending exception traffic).
     */
    [[nodiscard]] bool jitBlockReady() const noexcept;

    /** @brief Advances hotness and prepares a block without changing guest CPU state. */
    /** Reversible spans may opt into single-instruction branch/call links. */
    [[nodiscard]] bool prepareJitBlock(bool allow_single = false);
    struct JitStats {
        std::uint64_t block_executions{0};
        std::uint64_t block_instructions{0};
        std::uint64_t fallbacks{0};
        std::uint64_t compilations{0};
        std::uint64_t single_fast{0};
        std::uint64_t single_generic{0};
        std::uint64_t block_generic{0};
        /// Fast-handler declines on the single-step path (MMIO/faults).
        std::uint64_t single_decline{0};
        std::uint64_t native_compilations{0};
        std::uint64_t native_executions{0};
        std::uint64_t native_instructions{0};
        std::uint64_t native_compilation_failures{0};
        std::uint64_t native_evictions{0};
    };
    [[nodiscard]] JitStats jitStats() const noexcept { return jit_stats_; }
    [[nodiscard]] const std::string& nativeJitError() const noexcept { return native_jit_error_; }

    /**
     * @brief Provides FPCCR (ASPEN) reads for automatic CONTROL.FPCA maintenance.
     * May remain unset (unit tests); FP ownership tracking then assumes ASPEN.
     */
    void setSystemControl(const cortexm::SystemControl* system) noexcept { system_ = system; }

private:
    using NativeFunction = void (*)(CpuState*, std::size_t);
    struct InstructionCacheEntry {
        std::uint64_t generation{0};
        std::uint32_t pc{0};
        std::uint32_t raw{0};
        DecodedInstruction decoded{};
        std::uint8_t size{0};
        // Hot-path cycle cost, memoized at decode time so stepping and
        // burst prediction pay one load instead of a ~90-case switch.
        // Divide forms are data-dependent: divide_form marks entries whose
        // cost is resolved live via divideCycles().
        std::uint16_t base_cycles{1};
        bool divide_form{false};
        std::uint8_t jit_fast{0};
        std::uint16_t branch_penalty{0};
        bool fast_unconditional{false};
        bool suppress_loop_observation{false};
        /// True when execution may report synchronization_required (memory).
        /// Lets the stepper skip its restart copy for pure ALU/branch ops.
        bool may_trap{false};
        bool native_supported{false};
        NativeFunction native_function{nullptr};
        std::uint8_t native_slot{0U};
        std::uint16_t native_hits{0U};
        bool native_attempted{false};
    };

    static constexpr std::size_t instruction_cache_entries = 65536U;

    /// Direct-mapped decode-cache index with high-bit folding so firmware
    /// spread across flash does not alias every 32 KiB of code.
    [[nodiscard]] static constexpr std::size_t instructionCacheIndex(
        const std::uint32_t pc) noexcept {
        return ((pc >> 1U) ^ (pc >> 17U)) & (instruction_cache_entries - 1U);
    }

    [[nodiscard]] StopReason execute(
        const DecodedInstruction& instruction,
        DiagnosticSnapshot& diagnostic
    );
    void capture(DiagnosticSnapshot& diagnostic) const;

    mem::MemoryBus& memory_;
    const cortexm::SystemControl* system_{nullptr};
    CpuState state_{};
    // Heap-boxed: ~8 MiB combined at current capacities; inline members
    // would overflow caller stacks (unit tests construct CPUs as locals).
    std::unique_ptr<std::array<InstructionCacheEntry, instruction_cache_entries>> instruction_cache_;
    DiagnosticSnapshot last_diagnostic_{};

    // Hot-path JIT: direct-mapped block cache keyed by entry PC.
    // Per-op metadata is fully precomputed at compile time so block
    // execution pays no classification, flag-suppression, or dispatch
    // switches for inlined integer ops.
    enum class JitFast : std::uint8_t {
        generic,
        nop,
        b,
        bl,
        cbz,
        cbnz,
        mov_imm,
        mov_reg,
        movw,
        movt,
        add_imm,
        add_reg,
        sub_imm,
        sub_reg,
        cmp_imm,
        cmp_reg,
        logic_reg,
        mul_reg,
        mla_mls,
        long_mul,
        alu_single,
        alu_rsb_tst,
        dsp_extend,
        shift,
        bx_blx,
        divide,
        pop_pc,
        ldr_imm,
        str_imm,
        ldr_sub,
        str_sub,
        ldrd_strd,
        push_pop,
        ldm_stm,
        ldr_word_gpr,
        str_word_gpr,
    };
    struct JitBlockEntry {
        std::uint64_t generation{0};
        std::uint32_t pc{0};
        std::uint8_t count{0};
        bool valid{false};
        bool attempted{false};
        NativeFunction native_function{nullptr};
        std::uint8_t native_count{0}; ///< Pure supported prefix; suffix uses cached handlers.
        std::uint8_t native_slot{0U};
        std::uint16_t native_hits{0U};
        bool native_attempted{false};
        std::array<DecodedInstruction, JitStepOutcome::max_block> ops{};
        std::array<std::uint32_t, JitStepOutcome::max_block> pcs{};
        std::array<std::uint8_t, JitStepOutcome::max_block> sizes{};
        std::array<std::uint32_t, JitStepOutcome::max_block> raws{};
        std::array<std::uint16_t, JitStepOutcome::max_block> base_cycles{};
        std::array<bool, JitStepOutcome::max_block> divide_form{};
        std::array<JitFast, JitStepOutcome::max_block> fast{};
        std::array<bool, JitStepOutcome::max_block> is_memory{};
        std::array<bool, JitStepOutcome::max_block> is_terminator{};
        std::array<std::uint16_t, JitStepOutcome::max_block> branch_penalty{};
        // Precomputed fast-block metadata: when every op is a JitFast integer
        // op and nothing touches memory, block execution skips per-op
        // generation/PC/IT/pending checks and cycle branches.
        bool all_fast{false};
        bool has_memory{false};
        bool has_store{false};
        std::uint16_t fast_base_cycles{0};
        bool fast_last_suppress{false};
        // Precomputed preview: pure function of slot contents, so peek and
        // board-side budget checks reuse it instead of recomputing per step.
        std::array<std::uint16_t, JitStepOutcome::max_block> preview_cycles{};
        std::array<std::uint32_t, JitStepOutcome::max_block + 1U> preview_prefix_cycles{};
        std::array<std::uint32_t, JitStepOutcome::max_block + 1U> base_prefix_cycles{};
        std::array<bool, JitStepOutcome::max_block> suppress_obs{};
        std::uint8_t preview_count{0}; ///< Memory-free prefix length.
        /// Optimistic prefix incl. handler-covered transfers (horizon gate).
        std::uint8_t preview_extended_count{0};
        /// Consecutive entry-declines (MMIO/fault polls); the gate singles
        /// these directly instead of re-wasting prepare/preview/execute.
        /// Clears on any commit or slot rebuild; false positives only cost
        /// batching, never correctness (single-step is always exact).
        std::uint8_t decline_streak{0};
        std::uint8_t preview_exact_prefix{0}; ///< Leading exact-cycle ops.
    };
    static constexpr std::size_t jit_block_entries = 4096U;

    /// Direct-mapped block-cache index with high-bit folding matching the
    /// decode cache strategy.
    [[nodiscard]] static constexpr std::size_t jitBlockIndex(
        const std::uint32_t pc) noexcept {
        return ((pc >> 1U) ^ (pc >> 13U)) & (jit_block_entries - 1U);
    }
    std::unique_ptr<std::array<JitBlockEntry, jit_block_entries>> jit_blocks_;
    std::array<std::uint16_t, jit_block_entries> jit_hot_{};
    JitStats jit_stats_{};
    static constexpr std::uint16_t jit_compile_threshold = 50U;
    static constexpr std::uint16_t native_compile_threshold = 16384U;
    static constexpr std::uint16_t native_block_compile_threshold = 512U;
    static constexpr std::size_t max_native_kernels = 32U;
    struct NativeState;
    std::unique_ptr<NativeState> native_state_;
    std::uint64_t native_clock_{0}; ///< Native candidates/block execution, not simulated time.
    bool native_single_instruction_jit_enabled_{true};
    std::string native_jit_error_;
    [[nodiscard]] bool ensureNativeCompiler();
    [[nodiscard]] std::uint8_t retainNativeKernel(std::shared_ptr<const NativeJitKernel> kernel);
    void touchNativeKernel(std::uint8_t slot) noexcept;
    [[nodiscard]] bool executeNativeInstruction(InstructionCacheEntry& entry);
    void prepareNativeBlock(JitBlockEntry& entry);

    [[nodiscard]] FastStepResult stepFastImpl(bool use_jit, bool fast_handler_declined = false);
    /// Shared block-execution body; prepared skips hotness/validity probing.
    [[nodiscard]] std::optional<JitStepOutcome> tryStepJitBlockImpl(
        std::size_t max_instructions, bool prepared, bool fast_only = false,
        std::array<std::uint16_t, JitStepOutcome::max_block>* instruction_cycles = nullptr);
    [[nodiscard]] std::optional<TimedJitStepOutcome> executeTrustedReversibleJitBlock(
        std::size_t max_instructions, ReversibleCycleBudget* budget = nullptr);
    [[nodiscard]] std::optional<TimedJitStepOutcome> tryStepItScalarFallback(
        ReversibleCycleBudget* budget);
    [[nodiscard]] bool executeJitFast(const DecodedInstruction& op, JitFast fast,
                                      std::uint32_t pc) noexcept;
    [[nodiscard]] std::optional<DecodedInstruction> fetchDecode(
        std::uint32_t pc, std::uint32_t& raw_out, std::uint8_t& size_out);
    [[nodiscard]] static JitFast classifyJitFast(const DecodedInstruction& op) noexcept;
};

/** @brief Stable lowercase name for diagnostics and tests. */
[[nodiscard]] const char* stopReasonName(StopReason reason) noexcept;

} // namespace fil::cpu
