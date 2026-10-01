#include "fil/cpu/cortex_m4.hpp"

#include "fil/cpu/decoder.hpp"
#include "fil/cpu/jit.hpp"
#if FIL_HAS_LLVM_JIT
#include "fil/cpu/llvm_jit.hpp"
#include <vector>
#endif
#include "fil/elf/elf_loader.hpp"
#include "fil/mem/memory_bus.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>

namespace fil::cpu {
namespace {

constexpr std::uint32_t xpsr_it_mask = (0x3U << 25U) | (0x3fU << 10U);

[[nodiscard]] bool suppressImplicitFlagsInIt(const InstrKind kind) noexcept {
    switch (kind) {
    case InstrKind::mov:
    case InstrKind::add:
    case InstrKind::adc:
    case InstrKind::sub:
    case InstrKind::sbc:
    case InstrKind::rsb:
    case InstrKind::and_:
    case InstrKind::orr:
    case InstrKind::eor:
    case InstrKind::bic:
    case InstrKind::mvn:
    case InstrKind::orn:
    case InstrKind::mul:
    case InstrKind::lsl:
    case InstrKind::lsr:
    case InstrKind::asr:
    case InstrKind::ror:
    case InstrKind::rrx:
        return true;
    default:
        return false;
    }
}

[[nodiscard]] std::string undefinedMessage(
    const std::uint32_t pc,
    const std::uint32_t raw,
    const std::uint8_t size
) {
    std::ostringstream output;
    output << "unimplemented instruction pc=0x" << std::hex << std::setw(8)
           << std::setfill('0') << pc << " raw" << std::dec
           << static_cast<unsigned int>(size * 8U) << "=0x" << std::hex
           << std::setw(size == 2U ? 4 : 8) << raw;
    return output.str();
}

inline void jitSetNz(CpuState& state, const std::uint32_t value) noexcept {
    state.xpsr = (state.xpsr & ~(xpsr_n | xpsr_z))
        | (value & xpsr_n) | (value == 0U ? xpsr_z : 0U);
}

inline void jitSetNzc(CpuState& state, const std::uint32_t value, const bool carry) noexcept {
    state.xpsr = (state.xpsr & ~(xpsr_n | xpsr_z | xpsr_c))
        | (value & xpsr_n) | (value == 0U ? xpsr_z : 0U)
        | (carry ? xpsr_c : 0U);
}

// Produce the architectural NZCV bit positions directly. Materializing an
// AddResult packs four booleans only to unpack/shift them again on retirement.
inline std::uint32_t jitAdd(CpuState& state, const std::uint32_t left,
    const std::uint32_t right, const bool carry, const bool set_flags) noexcept {
    const std::uint64_t wide = static_cast<std::uint64_t>(left) + right + (carry ? 1U : 0U);
    const auto value = static_cast<std::uint32_t>(wide);
    if (set_flags) {
        const auto overflow = ((~(left ^ right) & (left ^ value)) >> 3U) & xpsr_v;
        state.xpsr = (state.xpsr & ~(xpsr_n | xpsr_z | xpsr_c | xpsr_v))
            | (value & xpsr_n) | (value == 0U ? xpsr_z : 0U)
            | (static_cast<std::uint32_t>(wide >> 32U) << 29U) | overflow;
    }
    return value;
}

} // namespace

CortexM4::JitFast CortexM4::classifyJitFast(const DecodedInstruction& op) noexcept {
    // Inlined handlers are condition-agnostic: both dispatch sites evaluate
    // the condition first (failed conditions retire for 1 cycle without
    // dispatch), and compile-time pure blocks admit only unconditional ops.
    // Anything else stays on the exact generic execute() path.
    switch (op.kind) {
    case InstrKind::nop:
        return JitFast::nop;
    case InstrKind::b:
        return JitFast::b;
    case InstrKind::bl:
        return JitFast::bl;
    case InstrKind::cbz:
        return JitFast::cbz;
    case InstrKind::cbnz:
        return JitFast::cbnz;
    case InstrKind::bx:
        // Indirect branch/return; exception-return tokens and invalid
        // targets decline at runtime for exact generic handling.
        return JitFast::bx_blx;
    case InstrKind::mov:
        if (op.rd == 15U || op.rm == 15U) return JitFast::generic;
        if (op.form == OperandForm::immediate) return JitFast::mov_imm;
        return op.form == OperandForm::register_value
            ? JitFast::mov_reg : JitFast::generic;
    case InstrKind::movw:
        return op.rd != 15U ? JitFast::movw : JitFast::generic;
    case InstrKind::movt:
        return op.rd != 15U ? JitFast::movt : JitFast::generic;
    case InstrKind::add:
    case InstrKind::adc:
        if (op.rd == 15U) return JitFast::generic;
        if (op.form == OperandForm::immediate) return JitFast::add_imm;
        return op.form == OperandForm::register_value
            ? JitFast::add_reg : JitFast::generic;
    case InstrKind::sub:
    case InstrKind::sbc:
        if (op.rd == 15U) return JitFast::generic;
        if (op.form == OperandForm::immediate) return JitFast::sub_imm;
        return op.form == OperandForm::register_value
            ? JitFast::sub_reg : JitFast::generic;
    case InstrKind::cmp:
    case InstrKind::cmn:
        if (op.form == OperandForm::immediate) return JitFast::cmp_imm;
        return op.form == OperandForm::register_value
            ? JitFast::cmp_reg : JitFast::generic;
    case InstrKind::and_:
    case InstrKind::orr:
    case InstrKind::eor:
    case InstrKind::bic:
    case InstrKind::mvn:
    case InstrKind::orn:
        return op.form == OperandForm::register_value && op.rd != 15U
            && op.rn != 15U && op.rm != 15U
            ? JitFast::logic_reg : JitFast::generic;
    case InstrKind::mul:
        return op.form == OperandForm::register_value && op.rd != 15U
            && op.rn != 15U && op.rm != 15U
            ? JitFast::mul_reg : JitFast::generic;
    case InstrKind::udiv:
    case InstrKind::sdiv:
        return op.rd != 15U && op.rn != 15U && op.rm != 15U
            ? JitFast::divide : JitFast::generic;
    case InstrKind::mla:
    case InstrKind::mls:
        return op.form == OperandForm::register_value
            ? JitFast::mla_mls : JitFast::generic;
    case InstrKind::umull:
    case InstrKind::smull:
        return op.form == OperandForm::register_value
            ? JitFast::long_mul : JitFast::generic;
    case InstrKind::rsb:
    case InstrKind::tst:
        return op.form == OperandForm::immediate
                || op.form == OperandForm::register_value
            ? JitFast::alu_rsb_tst : JitFast::generic;
    case InstrKind::clz:
    case InstrKind::rev:
    case InstrKind::rev16:
    case InstrKind::revsh:
        return JitFast::alu_single;
    case InstrKind::bfc:
    case InstrKind::ubfx:
    case InstrKind::sxtb:
    case InstrKind::sxth:
    case InstrKind::uxtb:
    case InstrKind::uxth:
    case InstrKind::uadd8:
    case InstrKind::sel:
        return JitFast::dsp_extend;
    case InstrKind::lsl:
    case InstrKind::lsr:
    case InstrKind::asr:
    case InstrKind::ror:
    case InstrKind::rrx:
        return op.rd != 15U && op.rn != 15U && op.rm != 15U
            && (op.form == OperandForm::immediate
                || op.form == OperandForm::register_value)
            ? JitFast::shift : JitFast::generic;
    case InstrKind::ldr:
        if (op.form == OperandForm::immediate && op.rn < 13U && op.rd < 13U
            && op.index && op.add && !op.writeback) return JitFast::ldr_word_gpr;
        // Word-only direct forms (immediate/register/literal) without PC
        // destination; the handler proves backed RAM at runtime and
        // declines MMIO/faults with untouched state so the block stops
        // precisely. PC loads stay exact via execute() and end the preview.
        return (op.form == OperandForm::immediate
                || op.form == OperandForm::register_value
                || op.form == OperandForm::literal)
                && op.rd != 15U
            ? JitFast::ldr_imm : JitFast::generic;
    case InstrKind::str:
        if (op.form == OperandForm::immediate && op.rn < 13U && op.rd < 13U
            && op.index && op.add && !op.writeback) return JitFast::str_word_gpr;
        return (op.form == OperandForm::immediate
                || op.form == OperandForm::register_value
                || op.form == OperandForm::literal)
                && op.rd != 15U
            ? JitFast::str_imm : JitFast::generic;
    case InstrKind::ldrb:
    case InstrKind::ldrh:
    case InstrKind::ldrsb:
    case InstrKind::ldrsh:
        return (op.form == OperandForm::immediate
                || op.form == OperandForm::register_value
                || op.form == OperandForm::literal)
                && op.rd != 15U
            ? JitFast::ldr_sub : JitFast::generic;
    case InstrKind::strb:
    case InstrKind::strh:
        return (op.form == OperandForm::immediate
                || op.form == OperandForm::register_value
                || op.form == OperandForm::literal)
                && op.rd != 15U
            ? JitFast::str_sub : JitFast::generic;
    case InstrKind::ldrd:
    case InstrKind::strd:
        // Double-word immediate/register forms; rd/ra select the pair.
        // Runtime range probe declines MMIO/faults with untouched state.
        return (op.form == OperandForm::immediate
                || op.form == OperandForm::register_value
                || op.form == OperandForm::literal)
            ? JitFast::ldrd_strd : JitFast::generic;
    case InstrKind::push:
    case InstrKind::pop:
    case InstrKind::ldm:
    case InstrKind::stm:
        // Multi-register transfers without a PC load; the runtime range
        // probe declines MMIO/faults before any effect. PC-loading forms
        // use the pop_pc handler below (admitted as block terminators).
        return (op.register_list & (std::uint16_t{1U} << 15U)) == 0U
            ? (op.kind == InstrKind::push || op.kind == InstrKind::pop
                ? JitFast::push_pop : JitFast::ldm_stm)
            : JitFast::pop_pc;
    default:
        return JitFast::generic;
    }
}

bool CpuState::reset(const elf::ElfImage& image) noexcept {
    *this = CpuState{};
    msp = image.initialMsp();
    r[13] = msp;
    const std::uint32_t reset_handler = image.resetHandler();
    thumb = (reset_handler & 1U) != 0;
    r[15] = reset_handler & ~std::uint32_t{1};
    instruction_address = r[15];
    xpsr = thumb ? xpsr_t : 0U;
    halted = !thumb;
    return thumb;
}

std::uint16_t CpuState::ipsr() const noexcept {
    return static_cast<std::uint16_t>(xpsr & xpsr_ipsr_mask);
}

bool CpuState::inHandlerMode() const noexcept {
    return ipsr() != 0U;
}

std::uint32_t& CpuState::activeSp() noexcept {
    if (inHandlerMode() || (control & 0x2U) == 0U) return msp;
    return psp;
}

const std::uint32_t& CpuState::activeSp() const noexcept {
    if (inHandlerMode() || (control & 0x2U) == 0U) return msp;
    return psp;
}

std::uint32_t CpuState::readRegister(const std::uint8_t index) const noexcept {
    if (index == 13U) return activeSp();
    if (index == 15U) return architecturalPcForRead();
    return index < r.size() ? r[index] : 0U;
}

void CpuState::writeRegister(const std::uint8_t index, const std::uint32_t value) noexcept {
    if (index >= r.size()) return;
    if (index == 13U) {
        activeSp() = value;
        r[13] = value;
        return;
    }
    r[index] = value;
}

std::uint32_t CpuState::currentInstrAddr() const noexcept {
    return instruction_address;
}

std::uint32_t CpuState::architecturalPcForRead() const noexcept {
    return instruction_address + 4U;
}

bool CpuState::branchWritePc(const std::uint32_t target) noexcept {
    if ((target & 1U) == 0U) return false;
    r[15] = target & ~std::uint32_t{1};
    thumb = true;
    xpsr |= xpsr_t;
    return true;
}

void CpuState::setItState(const std::uint8_t value) noexcept {
    it_state = value;
    xpsr &= ~xpsr_it_mask;
    xpsr |= (static_cast<std::uint32_t>(value & 0x03U) << 25U)
        | (static_cast<std::uint32_t>(value & 0xfcU) << 8U);
}

void CpuState::advanceIt() noexcept {
    setItState(advanceItState(it_state));
}

bool bitwiseEqual(const CpuState& left, const CpuState& right) noexcept {
    if (left.r != right.r || left.xpsr != right.xpsr || left.msp != right.msp
        || left.psp != right.psp || left.primask != right.primask
        || left.basepri != right.basepri || left.faultmask != right.faultmask
        || left.control != right.control || left.thumb != right.thumb
        || left.halted != right.halted || left.fpscr != right.fpscr
        || left.it_state != right.it_state || left.fp_lazy_active != right.fp_lazy_active
        || left.fp_lazy_base != right.fp_lazy_base
        || left.pending_exception != right.pending_exception
        || left.pending_exc_return != right.pending_exc_return
        || left.fpscr != right.fpscr || left.it_state != right.it_state
        || left.instruction_address != right.instruction_address) {
        return false;
    }
    return std::memcmp(left.s.data(), right.s.data(), sizeof(left.s)) == 0;
}

bool RunResult::succeeded() const noexcept {
    return reason != StopReason::bus_fault
        && reason != StopReason::undefined_instruction
        && reason != StopReason::invalid_state;
}

struct CortexM4::NativeState {
#if FIL_HAS_LLVM_JIT
    std::unique_ptr<LlvmJit> compiler;
    struct Resident {
        std::shared_ptr<const NativeJitKernel> kernel;
        std::uint64_t last_used{0};
    };
    std::vector<Resident> kernels;
#endif
};

CortexM4::CortexM4(mem::MemoryBus& memory) : memory_(memory) {
    assert(decoderTablesHaveNoOverlaps());
    instruction_cache_ = std::make_unique<std::array<InstructionCacheEntry, instruction_cache_entries>>();
    jit_blocks_ = std::make_unique<std::array<JitBlockEntry, jit_block_entries>>();
}

CortexM4::~CortexM4() = default;

bool CortexM4::nativeJitAvailable() noexcept {
    return FIL_HAS_LLVM_JIT != 0;
}

bool CortexM4::ensureNativeCompiler() {
#if FIL_HAS_LLVM_JIT
    if (!native_state_) {
        native_state_ = std::make_unique<NativeState>();
        native_state_->compiler = LlvmJit::create(native_jit_error_);
        if (!native_state_->compiler) ++jit_stats_.native_compilation_failures;
    }
    if (!native_state_->compiler) return false;
    if (native_state_->kernels.size() < max_native_kernels) return true;
    const auto victim = std::min_element(native_state_->kernels.begin(),
        native_state_->kernels.end(),
        [](const auto& a, const auto& b) { return a.last_used < b.last_used; });
    // Admission hysteresis: do not evict the actively used working set.
    return native_clock_ - victim->last_used >= native_compile_threshold;
#else
    return false;
#endif
}

std::uint8_t CortexM4::retainNativeKernel(std::shared_ptr<const NativeJitKernel> kernel) {
#if FIL_HAS_LLVM_JIT
    auto& residents = native_state_->kernels;
    if (residents.size() < max_native_kernels) {
        residents.push_back({std::move(kernel), native_clock_});
        return static_cast<std::uint8_t>(residents.size() - 1U);
    }
    const auto victim = std::min_element(residents.begin(), residents.end(),
        [](const auto& a, const auto& b) { return a.last_used < b.last_used; });
    const auto slot = static_cast<std::uint8_t>(victim - residents.begin());
    const auto old_function = victim->kernel->entryPoint();
    // Clear every raw entry point before releasing its ORC resources. Evicted
    // sites must pass their hotness gates again rather than immediately churn.
    for (auto& entry : *instruction_cache_) {
        if (entry.native_function != old_function) continue;
        entry.native_function = nullptr;
        entry.native_hits = 0U;
        entry.native_attempted = false;
    }
    for (auto& entry : *jit_blocks_) {
        if (entry.native_function != old_function) continue;
        entry.native_function = nullptr;
        entry.native_count = 0U;
        entry.native_hits = 0U;
        entry.native_attempted = false;
    }
    *victim = {std::move(kernel), native_clock_};
    ++jit_stats_.native_evictions;
    return slot;
#else
    static_cast<void>(kernel);
    return 0U;
#endif
}

void CortexM4::touchNativeKernel(const std::uint8_t slot) noexcept {
#if FIL_HAS_LLVM_JIT
    native_state_->kernels[slot].last_used = native_clock_;
#else
    static_cast<void>(slot);
#endif
}

bool CortexM4::executeNativeInstruction(InstructionCacheEntry& entry) {
#if FIL_HAS_LLVM_JIT
    if (!entry.native_supported && !entry.native_function) return false;
    ++native_clock_;
    if (state_.it_state != 0U || state_.pending_exception || state_.pending_exc_return) return false;
    if (entry.native_function) {
        touchNativeKernel(entry.native_slot);
        entry.native_function(&state_, 1U);
        ++jit_stats_.native_executions;
        ++jit_stats_.native_instructions;
        return true;
    }
    if (entry.native_attempted) return false;
    if (entry.native_hits < native_compile_threshold) {
        ++entry.native_hits;
        return false;
    }
    entry.native_attempted = true;
    const NativeJitInstruction instruction{entry.decoded, entry.pc, entry.size};
    if (!ensureNativeCompiler()) {
        entry.native_hits = 0U;
        entry.native_attempted = false;
        return false;
    }
    auto kernel = native_state_->compiler->compile(
        std::span<const NativeJitInstruction>{&instruction, 1U}, native_jit_error_);
    if (!kernel || !kernel->entryPoint()) {
        ++jit_stats_.native_compilation_failures;
        entry.native_supported = false;
        return false;
    }
    entry.native_function = kernel->entryPoint();
    entry.native_slot = retainNativeKernel(std::move(kernel));
    ++jit_stats_.native_compilations;
    entry.native_function(&state_, 1U);
    ++jit_stats_.native_executions;
    ++jit_stats_.native_instructions;
    return true;
#else
    static_cast<void>(entry);
    return false;
#endif
}

void CortexM4::prepareNativeBlock(JitBlockEntry& entry) {
    entry.native_attempted = true;
#if FIL_HAS_LLVM_JIT
    std::array<NativeJitInstruction, JitStepOutcome::max_block> instructions{};
    std::uint8_t supported_count = 0U;
    for (; supported_count < entry.count; ++supported_count) {
        instructions[supported_count] = {entry.ops[supported_count],
            entry.pcs[supported_count], entry.sizes[supported_count]};
        if (!LlvmJit::supports(instructions[supported_count])) break;
    }
    // Spend scarce native slots on multi-instruction kernels. Short mixed
    // prefixes retain cached execution; sustained hot singles use their own gate.
    if (supported_count == 0U
        || (supported_count < entry.count && supported_count < 4U)) return;
    if (!ensureNativeCompiler()) {
        entry.native_hits = 0U;
        entry.native_attempted = false;
        return;
    }
    auto kernel = native_state_->compiler->compile(
        std::span<const NativeJitInstruction>{instructions.data(), supported_count}, native_jit_error_);
    if (!kernel || !kernel->entryPoint()) {
        ++jit_stats_.native_compilation_failures;
        return;
    }
    entry.native_function = kernel->entryPoint();
    entry.native_count = supported_count;
    entry.native_slot = retainNativeKernel(std::move(kernel));
    ++jit_stats_.native_compilations;
#else
    static_cast<void>(entry);
#endif
}

bool CortexM4::reset(const elf::ElfImage& image) noexcept {
    return state_.reset(image);
}

void CortexM4::capture(DiagnosticSnapshot& diagnostic) const {
    diagnostic.next_pc = state_.r[15];
    diagnostic.registers = state_.r;
    diagnostic.registers[13] = state_.activeSp();
    diagnostic.xpsr = state_.xpsr;
}

void CortexM4::captureDiagnostic(DiagnosticSnapshot& diagnostic) const {
    capture(diagnostic);
}

FastStepResult CortexM4::stepFast() {
    return stepFastImpl(false);
}

FastStepResult CortexM4::stepJitFast() {
    // A warm, unconditional cached handler needs neither the cold fetch/fault
    // frame nor the generic IT/restart state. Keep those off its call stack.
    const auto pc = state_.r[15];
    auto& cache = (*instruction_cache_)[instructionCacheIndex(pc)];
    if (!native_single_instruction_jit_enabled_ && !state_.halted && state_.thumb
        && (state_.xpsr & xpsr_t) != 0U && (pc & 1U) == 0U && state_.it_state == 0U
        && cache.pc == pc && cache.generation == memory_.executionGeneration()
        && static_cast<JitFast>(cache.jit_fast) != JitFast::generic) [[likely]] {
        const auto& decoded = cache.decoded;
        const bool condition_passed = decoded.condition == Condition::al
            || conditionPasses(decoded.condition, state_.xpsr);
        const auto cycles = !condition_passed ? std::uint16_t{1U} : cache.divide_form
            ? divideCycles(state_.readRegister(decoded.rm)) : cache.base_cycles;
        const auto old_instruction_address = state_.instruction_address;
        state_.instruction_address = pc;
        state_.r[15] = pc + cache.size;
        if (!condition_passed || executeJitFast(decoded, static_cast<JitFast>(cache.jit_fast), pc)) {
            if (condition_passed) ++jit_stats_.single_fast;
            FastStepResult result;
            result.instruction_address = pc;
            result.raw = cache.raw;
            result.instruction_size = cache.size;
            result.instructions = 1U;
            result.cycles = static_cast<std::uint16_t>(cycles
                + (state_.r[15] != pc + cache.size ? cache.branch_penalty : 0U));
            result.suppress_loop_observation = cache.suppress_loop_observation;
            return result;
        }
        // Declining handlers are side-effect-free. Restore the speculative
        // PC fields and interpret once, without retrying the same handler.
        state_.instruction_address = old_instruction_address;
        state_.r[15] = pc;
        return stepFastImpl(true, true);
    }
    return stepFastImpl(true);
}

#if defined(__clang__) || defined(__GNUC__)
__attribute__((noinline))
#endif
FastStepResult CortexM4::stepFastImpl(const bool use_jit, bool fast_handler_declined) {
    FastStepResult result;
    if (state_.halted) [[unlikely]] {
        result.reason = StopReason::halted;
        result.instruction_address = state_.r[15];
        last_diagnostic_.instruction_address = state_.r[15];
        last_diagnostic_.message = "CPU is halted";
        last_diagnostic_.bus_fault.reset();
        capture(last_diagnostic_);
        return result;
    }

    const std::uint32_t pc = state_.r[15];
    result.instruction_address = pc;
    if (!state_.thumb || (state_.xpsr & xpsr_t) == 0U || (pc & 1U) != 0U) [[unlikely]] {
        state_.halted = true;
        result.reason = StopReason::invalid_state;
        last_diagnostic_.instruction_address = pc;
        last_diagnostic_.message = "invalid Cortex-M Thumb execution state";
        last_diagnostic_.bus_fault.reset();
        capture(last_diagnostic_);
        return result;
    }

    auto& cache = (*instruction_cache_)[instructionCacheIndex(pc)];
    const std::uint64_t execution_generation = memory_.executionGeneration();
    std::uint8_t instruction_size = 0;
    const DecodedInstruction* decoded = nullptr;
    if (cache.generation == execution_generation && cache.pc == pc) [[likely]] {
        instruction_size = cache.size;
        result.raw = cache.raw;
        decoded = &cache.decoded;
    } else {
        const mem::AccessContext fetch_context{mem::AccessType::instruction_fetch, pc};
        const auto first = memory_.read16(pc, fetch_context);
        if (!first) {
            result.reason = StopReason::bus_fault;
            last_diagnostic_.instruction_address = pc;
            last_diagnostic_.raw = 0;
            last_diagnostic_.instruction_size = 0;
            last_diagnostic_.bus_fault = first.fault();
            last_diagnostic_.message = "instruction fetch failed";
            capture(last_diagnostic_);
            return result;
        }

        const bool wide = is32BitThumbPrefix(first.value());
        instruction_size = wide ? 4U : 2U;
        std::uint16_t second_halfword = 0;
        if (wide) {
            if (pc > std::numeric_limits<std::uint32_t>::max() - 2U) {
                mem::BusFault fault;
                fault.reason = mem::BusFaultReason::address_overflow;
                fault.address = pc;
                fault.size = mem::AccessSize::word;
                fault.context = fetch_context;
                fault.message = "32-bit instruction fetch wraps target address space";
                result.reason = StopReason::bus_fault;
                result.raw = static_cast<std::uint32_t>(first.value()) << 16U;
                result.instruction_size = instruction_size;
                last_diagnostic_.instruction_address = pc;
                last_diagnostic_.raw = result.raw;
                last_diagnostic_.instruction_size = instruction_size;
                last_diagnostic_.bus_fault = std::move(fault);
                last_diagnostic_.message = "second instruction halfword fetch failed";
                capture(last_diagnostic_);
                return result;
            }
            const auto second = memory_.read16(pc + 2U, fetch_context);
            if (!second) {
                result.reason = StopReason::bus_fault;
                result.raw = static_cast<std::uint32_t>(first.value()) << 16U;
                result.instruction_size = instruction_size;
                last_diagnostic_.instruction_address = pc;
                last_diagnostic_.raw = result.raw;
                last_diagnostic_.instruction_size = instruction_size;
                last_diagnostic_.bus_fault = second.fault();
                last_diagnostic_.message = "second instruction halfword fetch failed";
                capture(last_diagnostic_);
                return result;
            }
            second_halfword = second.value();
        }

        result.raw = wide
            ? (static_cast<std::uint32_t>(first.value()) << 16U) | second_halfword
            : first.value();
        const auto newly_decoded = wide
            ? decode32(first.value(), second_halfword) : decode16(first.value());
        if (!newly_decoded) {
            result.reason = StopReason::undefined_instruction;
            result.instruction_size = instruction_size;
            last_diagnostic_.instruction_address = pc;
            last_diagnostic_.raw = result.raw;
            last_diagnostic_.instruction_size = instruction_size;
            last_diagnostic_.message = undefinedMessage(
                pc, result.raw, instruction_size
            );
            last_diagnostic_.bus_fault.reset();
            capture(last_diagnostic_);
            return result;
        }
        cache.native_function = nullptr;
        cache.native_supported = false;
        cache.native_hits = 0U;
        cache.native_attempted = false;
        cache.generation = execution_generation;
        cache.pc = pc;
        cache.raw = result.raw;
        cache.decoded = *newly_decoded;
        cache.size = instruction_size;
        cache.divide_form = newly_decoded->kind == InstrKind::udiv
            || newly_decoded->kind == InstrKind::sdiv;
        cache.base_cycles = cache.divide_form
            ? 7U : basePipelineCycles(*newly_decoded);
        cache.jit_fast = static_cast<std::uint8_t>(classifyJitFast(*newly_decoded));
        cache.branch_penalty = takenBranchPenalty(newly_decoded->kind);
        cache.fast_unconditional = newly_decoded->condition == Condition::al
            && static_cast<JitFast>(cache.jit_fast) != JitFast::generic;
        cache.suppress_loop_observation = newly_decoded->kind == InstrKind::bl
            || newly_decoded->kind == InstrKind::blx
            || (newly_decoded->kind == InstrKind::bx && newly_decoded->rm == 14U)
            || (newly_decoded->kind == InstrKind::mov
                && newly_decoded->rd == 15U && newly_decoded->rm == 14U)
            || ((newly_decoded->kind == InstrKind::pop || newly_decoded->kind == InstrKind::ldm)
                && (newly_decoded->register_list & (std::uint16_t{1U} << 15U)) != 0U);
        // Only non-ALU/non-branch forms can report synchronization_required;
        // the stepper skips its restart copy for the rest.
        const auto may_trap_boundary = classifyJitBoundary(*newly_decoded);
        cache.may_trap = may_trap_boundary != JitBoundary::none
            && may_trap_boundary != JitBoundary::control_flow;
#if FIL_HAS_LLVM_JIT
        cache.native_supported = native_single_instruction_jit_enabled_
            && LlvmJit::supports(NativeJitInstruction{*newly_decoded, pc, instruction_size});
#endif
        decoded = &cache.decoded;
    }
    result.instruction_size = instruction_size;
    result.suppress_loop_observation = cache.suppress_loop_observation;

    // The common world-run configuration disables native single-instruction
    // compilation. On a decode-cache hit, unconditional non-IT JIT handlers
    // can then retire directly without constructing the generic restart/IT
    // machinery or evaluating condition state. A declining handler is
    // side-effect-free by contract; rewind the two speculative PC fields and
    // continue through the exact interpreter fallback once.
    if (!fast_handler_declined && use_jit && !native_single_instruction_jit_enabled_
        && state_.it_state == 0U && cache.fast_unconditional) {
        // Capture data-dependent divide cost from the pre-state: rd can
        // alias rm, so it must be sampled before the fast handler writes back.
        const std::uint16_t fast_cycles = cache.divide_form
            ? divideCycles(state_.readRegister(decoded->rm)) : cache.base_cycles;
        const std::uint32_t old_instruction_address = state_.instruction_address;
        state_.instruction_address = pc;
        state_.r[15] = pc + instruction_size;
        if (executeJitFast(*decoded, static_cast<JitFast>(cache.jit_fast), pc)) {
            result.reason = StopReason::step_complete;
            result.instructions = 1U;
            result.cycles = fast_cycles;
            if (state_.r[15] != pc + instruction_size) {
                result.cycles = static_cast<std::uint16_t>(
                    result.cycles + cache.branch_penalty);
            }
            ++jit_stats_.single_fast;
            return result;
        }
        state_.instruction_address = old_instruction_address;
        state_.r[15] = pc;
        fast_handler_declined = true;
    }

    // Memoized base cost (one load) sampled before execute() mutates
    // architectural state. Only DIV reads a live operand; every other form
    // uses the decode-time table value.
    const bool was_in_it = inItBlock(state_.it_state);
    const Condition effective_condition = decoded->kind == InstrKind::it
        ? Condition::al
        : was_in_it ? currentItCondition(state_.it_state) : decoded->condition;
    // AL dominates the common non-IT path; skip the out-of-line condition
    // evaluator for the overwhelmingly common unconditional instruction.
    const bool condition_passed = effective_condition == Condition::al
        || conditionPasses(effective_condition, state_.xpsr);
    std::uint16_t base_cycles = 1U;
    if (condition_passed) [[likely]] {
        base_cycles = cache.divide_form
            ? divideCycles(state_.readRegister(decoded->rm))
            : cache.base_cycles;
    }

    // Stack checkpoint, skipped for trap-free ops (pure ALU/branches can
    // never report synchronization_required). Copied only when a restart
    // may be needed.
    // Avoid zero-initializing the large architectural state on every step;
    // materialize its checkpoint only for a trapping memory instruction.
    std::optional<CpuState> restart_state;
    const bool need_restart = cache.may_trap && memory_.mmioTrapping();
    if (need_restart) restart_state.emplace(state_);
    state_.instruction_address = pc;
    state_.r[15] = pc + instruction_size;
    std::optional<DecodedInstruction> it_adjusted;
    if (was_in_it && decoded->set_flags && suppressImplicitFlagsInIt(decoded->kind)) {
        it_adjusted = *decoded;
        it_adjusted->set_flags = false;
        decoded = &*it_adjusted;
    }

    StopReason stop = StopReason::step_complete;
    if (condition_passed) [[likely]] {
        bool jit_handled = false;
        if (use_jit && !was_in_it && !fast_handler_declined) {
            if (native_single_instruction_jit_enabled_
                && (cache.native_supported || cache.native_function)) {
                jit_handled = executeNativeInstruction(cache);
            }
            if (!jit_handled) {
                const auto fast = static_cast<JitFast>(cache.jit_fast);
                const bool attempted = fast != JitFast::generic
                    && use_jit && !was_in_it;
                jit_handled = attempted && executeJitFast(*decoded, fast, pc);
                if (attempted && !jit_handled) ++jit_stats_.single_decline;
            }
        } else if (fast_handler_declined) {
            ++jit_stats_.single_decline;
        }
        if (!jit_handled) stop = execute(*decoded, last_diagnostic_);
        // Coverage accounting (one predictable counter per retired op).
        if (use_jit && !was_in_it && condition_passed) {
            if (jit_handled) ++jit_stats_.single_fast;
            else ++jit_stats_.single_generic;
        }
    }
    if (decoded->kind != InstrKind::it && was_in_it) state_.advanceIt();

    result.reason = stop;
    result.instructions = 1;
    // Failed conditions retire for 1 cycle with no memory/branch effect.
    // Taken control flow adds the DDI0439C pipeline refill when the PC
    // proves discontinuous against the sequential fallthrough.
    result.cycles = base_cycles;
    if (condition_passed && stop == StopReason::step_complete
        && state_.r[15] != pc + instruction_size) {
        result.cycles = static_cast<std::uint16_t>(
            result.cycles + cache.branch_penalty
        );
    }
    if (stop != StopReason::step_complete) [[unlikely]] {
        last_diagnostic_.instruction_address = pc;
        last_diagnostic_.raw = result.raw;
        last_diagnostic_.instruction_size = instruction_size;
        if (stop == StopReason::synchronization_required && need_restart) {
            state_ = *restart_state;
            result.instructions = 0;
            result.cycles = 0;
        }
        capture(last_diagnostic_);
    }
    return result;
}

bool CortexM4::executeJitFast(
    const DecodedInstruction& op, const JitFast fast, const std::uint32_t pc
) noexcept {
    switch (fast) {
    case JitFast::nop:
        return true;
    case JitFast::b:
        state_.r[15] = static_cast<std::uint32_t>(
            static_cast<std::int64_t>(pc) + 4 + op.branch_offset
        ) & ~std::uint32_t{1};
        return true;
    case JitFast::bl:
        state_.r[14] = state_.r[15] | 1U;
        state_.r[15] = static_cast<std::uint32_t>(
            static_cast<std::int64_t>(pc) + 4 + op.branch_offset
        ) & ~std::uint32_t{1};
        return true;
    case JitFast::cbz:
    case JitFast::cbnz: {
        const bool zero = state_.readRegister(op.rn) == 0U;
        const bool take = fast == JitFast::cbz ? zero : !zero;
        if (take) state_.r[15] = (pc + 4U + op.imm) & ~std::uint32_t{1};
        return true;
    }
    case JitFast::mov_reg: {
        const ShiftResult shifted = shiftC(
            state_.readRegister(op.rm), op.shift_type, op.shift_amount,
            (state_.xpsr & xpsr_c) != 0U);
        state_.writeRegister(op.rd, shifted.value);
        if (op.set_flags) jitSetNzc(state_, shifted.value, shifted.carry);
        return true;
    }
    case JitFast::mov_imm:
        state_.writeRegister(op.rd, op.imm);
        if (op.set_flags) {
            const bool carry = op.immediate_carry_valid
                ? op.immediate_carry : (state_.xpsr & xpsr_c) != 0U;
            jitSetNzc(state_, op.imm, carry);
        }
        return true;
    case JitFast::movw:
        state_.writeRegister(op.rd, op.imm);
        return true;
    case JitFast::movt: {
        const std::uint32_t value = (state_.readRegister(op.rd) & 0xffffU)
            | (op.imm << 16U);
        state_.writeRegister(op.rd, value);
        return true;
    }
    case JitFast::add_reg:
    case JitFast::sub_reg:
    case JitFast::add_imm:
    case JitFast::sub_imm: {
        std::uint32_t left = state_.readRegister(op.rn);
        if (op.rn == 15U && op.form == OperandForm::immediate) {
            left &= ~std::uint32_t{3};
        }
        const bool subtract = fast == JitFast::sub_imm || fast == JitFast::sub_reg;
        const std::uint32_t right = fast == JitFast::add_reg || fast == JitFast::sub_reg
            ? shiftC(state_.readRegister(op.rm), op.shift_type, op.shift_amount,
                     (state_.xpsr & xpsr_c) != 0U).value
            : op.imm;
        const bool carry_in = op.kind == InstrKind::adc || op.kind == InstrKind::sbc
            ? (state_.xpsr & xpsr_c) != 0U : subtract;
        const auto value = jitAdd(state_, left, subtract ? ~right : right, carry_in, op.set_flags);
        state_.writeRegister(op.rd, value);
        return true;
    }
    case JitFast::cmp_imm:
    case JitFast::cmp_reg: {
        const std::uint32_t left = state_.readRegister(op.rn);
        const std::uint32_t right = fast == JitFast::cmp_reg
            ? shiftC(state_.readRegister(op.rm), op.shift_type, op.shift_amount,
                     (state_.xpsr & xpsr_c) != 0U).value
            : op.imm;
        static_cast<void>(jitAdd(state_, left, op.kind == InstrKind::cmp ? ~right : right,
            op.kind == InstrKind::cmp, true));
        return true;
    }
    case JitFast::logic_reg: {
        const std::uint32_t left = state_.readRegister(op.rn);
        const ShiftResult shifted = shiftC(
            state_.readRegister(op.rm), op.shift_type, op.shift_amount,
            (state_.xpsr & xpsr_c) != 0U);
        std::uint32_t value = 0U;
        if (op.kind == InstrKind::and_) value = left & shifted.value;
        if (op.kind == InstrKind::orr) value = left | shifted.value;
        if (op.kind == InstrKind::eor) value = left ^ shifted.value;
        if (op.kind == InstrKind::bic) value = left & ~shifted.value;
        if (op.kind == InstrKind::mvn) value = ~shifted.value;
        if (op.kind == InstrKind::orn) value = left | ~shifted.value;
        state_.writeRegister(op.rd, value);
        if (op.set_flags) jitSetNzc(state_, value, shifted.carry);
        return true;
    }
    case JitFast::mul_reg: {
        const std::uint64_t product = static_cast<std::uint64_t>(state_.readRegister(op.rn))
            * state_.readRegister(op.rm);
        const std::uint32_t value = static_cast<std::uint32_t>(product);
        state_.writeRegister(op.rd, value);
        if (op.set_flags) jitSetNz(state_, value);
        return true;
    }
    case JitFast::divide: {
        if (op.kind == InstrKind::udiv) {
            const std::uint32_t divisor = state_.readRegister(op.rm);
            const std::uint32_t value = divisor == 0U
                ? 0U : state_.readRegister(op.rn) / divisor;
            state_.writeRegister(op.rd, value);
        } else {
            const std::int32_t dividend = std::bit_cast<std::int32_t>(
                state_.readRegister(op.rn));
            const std::int32_t divisor = std::bit_cast<std::int32_t>(
                state_.readRegister(op.rm));
            std::int32_t quotient = 0;
            if (divisor != 0) {
                if (dividend == std::numeric_limits<std::int32_t>::min()
                    && divisor == -1) {
                    quotient = std::numeric_limits<std::int32_t>::min();
                } else {
                    quotient = dividend / divisor;
                }
            }
            state_.writeRegister(op.rd, std::bit_cast<std::uint32_t>(quotient));
        }
        return true;
    }
    case JitFast::shift: {
        const bool immediate = op.form == OperandForm::immediate;
        const std::uint32_t value = immediate
            ? state_.readRegister(op.rm) : state_.readRegister(op.rn);
        const std::uint32_t amount = immediate ? op.shift_amount
            : (state_.readRegister(op.rm) & 0xffU);
        const ShiftResult shifted = shiftC(
            value, op.shift_type, amount, (state_.xpsr & xpsr_c) != 0U);
        state_.writeRegister(op.rd, shifted.value);
        if (op.set_flags) jitSetNzc(state_, shifted.value, shifted.carry);
        return true;
    }
    case JitFast::mla_mls: {
        // Flags never update (matches execute(), which ignores set_flags).
        const std::uint32_t product = state_.readRegister(op.rn)
            * state_.readRegister(op.rm);
        const std::uint32_t addend = state_.readRegister(op.ra);
        state_.writeRegister(op.rd,
            op.kind == InstrKind::mla ? product + addend : addend - product);
        return true;
    }
    case JitFast::long_mul: {
        if (op.kind == InstrKind::umull) {
            const std::uint64_t product = static_cast<std::uint64_t>(
                state_.readRegister(op.rn)) * state_.readRegister(op.rm);
            state_.writeRegister(op.rd, static_cast<std::uint32_t>(product));
            state_.writeRegister(op.ra, static_cast<std::uint32_t>(product >> 32U));
        } else {
            const std::int64_t product = static_cast<std::int64_t>(
                std::bit_cast<std::int32_t>(state_.readRegister(op.rn)))
                * std::bit_cast<std::int32_t>(state_.readRegister(op.rm));
            const std::uint64_t bits = std::bit_cast<std::uint64_t>(product);
            state_.writeRegister(op.rd, static_cast<std::uint32_t>(bits));
            state_.writeRegister(op.ra, static_cast<std::uint32_t>(bits >> 32U));
        }
        return true;
    }
    case JitFast::alu_rsb_tst: {
        // Operand selection mirrors execute() exactly, including its
        // degenerate 16-bit fallbacks.
        ShiftResult shifted{op.imm,
            op.immediate_carry_valid ? op.immediate_carry
                                     : (state_.xpsr & xpsr_c) != 0U};
        if (op.form == OperandForm::register_value) {
            shifted = shiftC(state_.readRegister(op.rm), op.shift_type,
                op.shift_amount, (state_.xpsr & xpsr_c) != 0U);
        }
        if (op.kind == InstrKind::tst) {
            const std::uint32_t value = state_.readRegister(op.rn) & shifted.value;
            jitSetNzc(state_, value, shifted.carry);
            return true;
        }
        const std::uint32_t left = op.form == OperandForm::immediate
            ? op.imm : op.is_32bit ? shifted.value : 0U;
        const std::uint32_t right = op.form == OperandForm::immediate
            ? state_.readRegister(op.rn)
            : op.is_32bit ? state_.readRegister(op.rn)
                           : state_.readRegister(op.rm);
        const auto value = jitAdd(state_, left, ~right, true, op.set_flags);
        state_.writeRegister(op.rd, value);
        return true;
    }
    case JitFast::alu_single: {
        const std::uint32_t source = state_.readRegister(op.rm);
        switch (op.kind) {
        case InstrKind::clz:
            state_.writeRegister(op.rd,
                static_cast<std::uint32_t>(std::countl_zero(source)));
            return true;
        case InstrKind::rev:
            state_.writeRegister(op.rd,
                ((source & 0x000000ffU) << 24U) | ((source & 0x0000ff00U) << 8U)
                    | ((source & 0x00ff0000U) >> 8U)
                    | ((source & 0xff000000U) >> 24U));
            return true;
        case InstrKind::rev16:
            state_.writeRegister(op.rd,
                ((source & 0x00ff00ffU) << 8U) | ((source & 0xff00ff00U) >> 8U));
            return true;
        default: { // revsh
            std::uint32_t value = ((source & 0xffU) << 8U) | ((source >> 8U) & 0xffU);
            if ((value & 0x8000U) != 0U) value |= 0xffff0000U;
            state_.writeRegister(op.rd, value);
            return true;
        }
        }
    }
    case JitFast::dsp_extend: {
        switch (op.kind) {
        case InstrKind::bfc: {
            const std::uint32_t width = op.imm;
            const std::uint32_t field_mask = width == 32U
                ? 0xffffffffU
                : ((std::uint32_t{1} << width) - 1U) << op.shift_amount;
            state_.writeRegister(op.rd, state_.readRegister(op.rd) & ~field_mask);
            return true;
        }
        case InstrKind::ubfx: {
            const std::uint32_t width = op.imm;
            const std::uint32_t mask = width == 32U
                ? 0xffffffffU : (std::uint32_t{1} << width) - 1U;
            state_.writeRegister(op.rd,
                (state_.readRegister(op.rn) >> op.shift_amount) & mask);
            return true;
        }
        case InstrKind::uxtb:
        case InstrKind::uxth:
        case InstrKind::sxtb:
        case InstrKind::sxth: {
            const std::uint32_t rotated = std::rotr(
                state_.readRegister(op.rm), static_cast<int>(op.shift_amount));
            const bool is_byte = op.kind == InstrKind::uxtb || op.kind == InstrKind::sxtb;
            const bool is_signed = op.kind == InstrKind::sxtb || op.kind == InstrKind::sxth;
            std::uint32_t value = is_byte ? rotated & 0xffU : rotated & 0xffffU;
            const std::uint32_t sign = is_byte ? 0x80U : 0x8000U;
            if (is_signed && (value & sign) != 0U) {
                value |= is_byte ? 0xffffff00U : 0xffff0000U;
            }
            state_.writeRegister(op.rd, value);
            return true;
        }
        case InstrKind::uadd8: {
            const std::uint32_t left = state_.readRegister(op.rn);
            const std::uint32_t right = state_.readRegister(op.rm);
            std::uint32_t value = 0U;
            std::uint32_t ge = 0U;
            for (std::uint32_t lane = 0U; lane < 4U; ++lane) {
                const std::uint32_t shift = lane * 8U;
                const std::uint32_t sum = ((left >> shift) & 0xffU)
                    + ((right >> shift) & 0xffU);
                value |= (sum & 0xffU) << shift;
                if (sum >= 0x100U) ge |= std::uint32_t{1} << (16U + lane);
            }
            state_.writeRegister(op.rd, value);
            state_.xpsr = (state_.xpsr & ~xpsr_ge_mask) | ge;
            return true;
        }
        default: { // sel
            const std::uint32_t left = state_.readRegister(op.rn);
            const std::uint32_t right = state_.readRegister(op.rm);
            std::uint32_t value = 0U;
            for (std::uint32_t lane = 0U; lane < 4U; ++lane) {
                const std::uint32_t shift = lane * 8U;
                const bool use_left = (state_.xpsr & (std::uint32_t{1} << (16U + lane))) != 0U;
                value |= ((use_left ? left : right) >> shift & 0xffU) << shift;
            }
            state_.writeRegister(op.rd, value);
            return true;
        }
        }
    }
    case JitFast::bx_blx: {
        // BX only (BLX writes LR before validation, so it stays generic).
        // Exception-return tokens set the pending flag exactly like
        // execute(); non-Thumb targets decline for exact fault handling.
        const std::uint32_t target = state_.readRegister(op.rm);
        if (op.kind == InstrKind::bx && isExceptionReturn(target)) {
            state_.pending_exc_return = target;
            return true;
        }
        if (!state_.branchWritePc(target)) return false;
        return true;
    }
    case JitFast::pop_pc: {
        // PC-loading multi-register transfer admitted as a block
        // terminator: probe the span (decline = untouched), buffer all
        // words, validate the PC word before committing anything, then
        // commit registers, writeback, and the branch in interpreter order.
        const std::uint32_t count = static_cast<std::uint32_t>(
            std::popcount(op.register_list));
        const std::uint32_t original_base = state_.readRegister(op.rn);
        const bool decrement_before = !op.add && op.index;
        const std::uint32_t start = decrement_before
            ? original_base - count * 4U : original_base;
        if (count == 0U) return false;
        const mem::AccessContext context{mem::AccessType::data_read, pc};
        std::array<std::uint32_t, 16> dense{};
        if (!memory_.tryFastReadWords(start, count, dense.data(), context)) {
            return false;
        }
        // Locate the PC word in transfer order (ascending registers).
        std::uint32_t pc_word = 0U;
        {
            std::uint32_t w = 0U;
            for (std::uint8_t reg = 0; reg < 16U; ++reg) {
                if ((op.register_list & (std::uint16_t{1} << reg)) == 0U) continue;
                if (reg == 15U) pc_word = dense[w];
                ++w;
            }
        }
        // Validate before committing: exception-return pends, non-Thumb
        // targets decline for exact fault handling (generic path reproduces
        // the interpreter's register updates on that path).
        const bool exc_return = isExceptionReturn(pc_word);
        if (!exc_return && (pc_word & 1U) == 0U) return false;
        {
            std::uint32_t w = 0U;
            for (std::uint8_t reg = 0; reg < 15U; ++reg) {
                if ((op.register_list & (std::uint16_t{1} << reg)) == 0U) continue;
                state_.writeRegister(reg, dense[w++]);
            }
        }
        if (op.writeback) {
            const std::uint32_t final_base = decrement_before
                ? start : original_base + count * 4U;
            state_.writeRegister(op.rn, final_base);
        }
        if (exc_return) {
            state_.pending_exc_return = pc_word;
            return true;
        }
        const bool branched = state_.branchWritePc(pc_word);
        assert(branched); // Thumb bit validated above.
        static_cast<void>(branched);
        return true;
    }
    case JitFast::ldr_word_gpr: {
        // Decode proves ordinary registers, positive immediate indexing and
        // no writeback. Runtime memory guards still retain every bus effect.
        const auto address = state_.r[op.rn] + op.imm;
        std::uint32_t value = 0U;
        if (!memory_.tryFastRead32(address, value, {mem::AccessType::data_read, pc})) return false;
        state_.r[op.rd] = value;
        return true;
    }
    case JitFast::str_word_gpr: {
        const auto address = state_.r[op.rn] + op.imm;
        return memory_.tryFastWrite32(address, state_.r[op.rd], {mem::AccessType::data_write, pc});
    }
    case JitFast::ldr_imm:
    case JitFast::str_imm: {
        // Address computation mirrors execute()'s word LDR/STR exactly;
        // only the memory dispatch is specialized. tryFast* touches no
        // state on false, so the caller stops the block (single-step runs
        // the MMIO/fault at its exact boundary) or falls back to execute().
        // Note: callers advance r15 before dispatch, so rn==15 reads and
        // the access context match the generic path.
        const std::uint32_t base = op.rn == 15U
            ? state_.architecturalPcForRead() & ~std::uint32_t{3}
            : state_.readRegister(op.rn);
        std::uint32_t offset = op.imm;
        if (op.form == OperandForm::register_value) {
            offset = shiftC(state_.readRegister(op.rm), op.shift_type,
                op.shift_amount, (state_.xpsr & xpsr_c) != 0U).value;
        }
        const std::uint32_t offset_address = op.add
            ? base + offset : base - offset;
        const std::uint32_t address = op.index ? offset_address : base;
        if (fast == JitFast::ldr_imm) {
            const mem::AccessContext context{mem::AccessType::data_read, pc};
            std::uint32_t value = 0U;
            if (!memory_.tryFastRead32(address, value, context)) return false;
            state_.writeRegister(op.rd, value);
        } else {
            const mem::AccessContext context{mem::AccessType::data_write, pc};
            const std::uint32_t value = state_.readRegister(op.rd);
            if (!memory_.tryFastWrite32(address, value, context)) return false;
        }
        if (op.writeback) state_.writeRegister(op.rn, offset_address);
        return true;
    }
    case JitFast::ldr_sub:
    case JitFast::str_sub: {
        // Sub-word mirror of the word handler with execute()'s zero/sign
        // extension; tryFast* declines MMIO/faults with untouched state.
        const std::uint32_t base = op.rn == 15U
            ? state_.architecturalPcForRead() & ~std::uint32_t{3}
            : state_.readRegister(op.rn);
        std::uint32_t offset = op.imm;
        if (op.form == OperandForm::register_value) {
            offset = shiftC(state_.readRegister(op.rm), op.shift_type,
                op.shift_amount, (state_.xpsr & xpsr_c) != 0U).value;
        }
        const std::uint32_t sub_offset_address = op.add ? base + offset : base - offset;
        const std::uint32_t sub_address = op.index ? sub_offset_address : base;
        const bool is_byte = fast == JitFast::ldr_sub
            ? (op.kind == InstrKind::ldrb || op.kind == InstrKind::ldrsb)
            : op.kind == InstrKind::strb;
        const mem::AccessSize width = is_byte
            ? mem::AccessSize::byte : mem::AccessSize::halfword;
        if (fast == JitFast::ldr_sub) {
            const mem::AccessContext context{mem::AccessType::data_read, pc};
            std::uint64_t raw = 0U;
            if (!memory_.tryFastRead(sub_address, width, raw, context)) return false;
            std::uint32_t value = static_cast<std::uint32_t>(raw);
            if (op.kind == InstrKind::ldrsb && (value & 0x80U) != 0U) {
                value |= 0xffffff00U;
            } else if (op.kind == InstrKind::ldrsh && (value & 0x8000U) != 0U) {
                value |= 0xffff0000U;
            }
            state_.writeRegister(op.rd, value);
        } else {
            const mem::AccessContext context{mem::AccessType::data_write, pc};
            if (!memory_.tryFastWrite(sub_address, width,
                    state_.readRegister(op.rd), context)) return false;
        }
        if (op.writeback) state_.writeRegister(op.rn, sub_offset_address);
        return true;
    }
    case JitFast::ldrd_strd: {
        // Mirrors execute()'s double-word transfer with one region probe
        // for both words: MMIO/faults decline before any effect.
        const bool load = op.kind == InstrKind::ldrd;
        const std::uint32_t base = state_.readRegister(op.rn);
        std::uint32_t offset = op.imm;
        if (op.form == OperandForm::register_value) {
            offset = shiftC(state_.readRegister(op.rm), op.shift_type,
                op.shift_amount, (state_.xpsr & xpsr_c) != 0U).value;
        }
        const std::uint32_t offset_address = op.add ? base + offset : base - offset;
        const std::uint32_t address = op.index ? offset_address : base;
        const mem::AccessContext context{
            load ? mem::AccessType::data_read : mem::AccessType::data_write, pc};
        std::array<std::uint32_t, 2> words{};
        if (!load) {
            words[0] = state_.readRegister(op.rd);
            words[1] = state_.readRegister(op.ra);
        }
        const bool ok = load
            ? memory_.tryFastReadWords(address, 2U, words.data(), context)
            : memory_.tryFastWriteWords(address, 2U, words.data(), context);
        if (!ok) return false;
        if (load) {
            state_.writeRegister(op.rd, words[0]);
            state_.writeRegister(op.ra, words[1]);
        }
        if (op.writeback) state_.writeRegister(op.rn, offset_address);
        return true;
    }
    case JitFast::push_pop:
    case JitFast::ldm_stm: {
        // Mirrors execute()'s multi-register transfer exactly, with one
        // region probe for the whole span: decline leaves everything
        // untouched, success buffers loads then commits registers and
        // writeback in interpreter order (non-PC, per classification).
        const bool load = fast == JitFast::push_pop
            ? op.kind == InstrKind::pop : op.kind == InstrKind::ldm;
        const std::uint32_t count = static_cast<std::uint32_t>(
            std::popcount(op.register_list));
        const std::uint32_t original_base = state_.readRegister(op.rn);
        const bool decrement_before = !op.add && op.index;
        const std::uint32_t start = decrement_before
            ? original_base - count * 4U : original_base;
        if (count == 0U) return false;
        const mem::AccessContext context{
            load ? mem::AccessType::data_read : mem::AccessType::data_write, pc};
        std::array<std::uint32_t, 16> dense{};
        if (!load) {
            std::uint32_t w = 0U;
            for (std::uint8_t reg = 0; reg < 16U; ++reg) {
                if ((op.register_list & (std::uint16_t{1} << reg)) == 0U) continue;
                dense[w++] = state_.readRegister(reg);
            }
        }
        const bool ok = load
            ? memory_.tryFastReadWords(start, count, dense.data(), context)
            : memory_.tryFastWriteWords(start, count, dense.data(), context);
        if (!ok) return false;
        if (load) {
            std::uint32_t w = 0U;
            for (std::uint8_t reg = 0; reg < 15U; ++reg) {
                if ((op.register_list & (std::uint16_t{1} << reg)) == 0U) continue;
                state_.writeRegister(reg, dense[w++]);
            }
        }
        if (op.writeback) {
            const std::uint32_t final_base = decrement_before
                ? start : original_base + count * 4U;
            state_.writeRegister(op.rn, final_base);
        }
        return true;
    }
    case JitFast::generic:
        return false;
    }
    return false;
}

std::optional<DecodedInstruction> CortexM4::fetchDecode(
    const std::uint32_t pc, std::uint32_t& raw_out, std::uint8_t& size_out) {
    const std::uint64_t generation = memory_.executionGeneration();
    auto& cache = (*instruction_cache_)[instructionCacheIndex(pc)];
    if (cache.generation == generation && cache.pc == pc) {
        raw_out = cache.raw;
        size_out = cache.size;
        return cache.decoded;
    }
    const mem::AccessContext fetch_context{mem::AccessType::instruction_fetch, pc};
    const auto first = memory_.read16(pc, fetch_context);
    if (!first) return std::nullopt;
    const bool wide = is32BitThumbPrefix(first.value());
    const std::uint8_t size = wide ? 4U : 2U;
    std::uint16_t second_halfword = 0;
    if (wide) {
        if (pc > std::numeric_limits<std::uint32_t>::max() - 2U) return std::nullopt;
        const auto second = memory_.read16(pc + 2U, fetch_context);
        if (!second) return std::nullopt;
        second_halfword = second.value();
    }
    const std::uint32_t raw = wide
        ? (static_cast<std::uint32_t>(first.value()) << 16U) | second_halfword
        : first.value();
    const auto decoded = wide ? decode32(first.value(), second_halfword)
                               : decode16(first.value());
    if (!decoded) return std::nullopt;
    cache.native_function = nullptr;
    cache.native_supported = false;
    cache.native_hits = 0U;
    cache.native_attempted = false;
    cache.generation = generation;
    cache.pc = pc;
    cache.raw = raw;
    cache.decoded = *decoded;
    cache.size = size;
    cache.divide_form = decoded->kind == InstrKind::udiv
        || decoded->kind == InstrKind::sdiv;
    cache.base_cycles = cache.divide_form ? 7U : basePipelineCycles(*decoded);
    cache.jit_fast = static_cast<std::uint8_t>(classifyJitFast(*decoded));
    cache.branch_penalty = takenBranchPenalty(decoded->kind);
    cache.fast_unconditional = decoded->condition == Condition::al
        && static_cast<JitFast>(cache.jit_fast) != JitFast::generic;
    cache.suppress_loop_observation = decoded->kind == InstrKind::bl
        || decoded->kind == InstrKind::blx
        || (decoded->kind == InstrKind::bx && decoded->rm == 14U)
        || (decoded->kind == InstrKind::mov && decoded->rd == 15U && decoded->rm == 14U)
        || ((decoded->kind == InstrKind::pop || decoded->kind == InstrKind::ldm)
            && (decoded->register_list & (std::uint16_t{1U} << 15U)) != 0U);
    const auto fetch_may_trap = classifyJitBoundary(*decoded);
    cache.may_trap = fetch_may_trap != JitBoundary::none
        && fetch_may_trap != JitBoundary::control_flow;
#if FIL_HAS_LLVM_JIT
    cache.native_supported = native_single_instruction_jit_enabled_
        && LlvmJit::supports(NativeJitInstruction{*decoded, pc, size});
#endif
    raw_out = raw;
    size_out = size;
    return *decoded;
}

bool CortexM4::jitBlockReady() const noexcept {
    if (state_.halted) return false;
    if (!state_.thumb || (state_.xpsr & xpsr_t) == 0U) return false;
    const std::uint32_t entry_pc = state_.r[15];
    if ((entry_pc & 1U) != 0U) return false;
    if (inItBlock(state_.it_state)) return false;
    if (state_.pending_exception || state_.pending_exc_return) return false;
    const auto& slot = (*jit_blocks_)[jitBlockIndex(entry_pc)];
    return slot.valid && slot.pc == entry_pc
        && slot.generation == memory_.executionGeneration();
}

bool CortexM4::prepareJitBlock(const bool allow_single) {
    if (state_.halted || !state_.thumb || (state_.xpsr & xpsr_t) == 0U) return false;
    const std::uint32_t entry_pc = state_.r[15];
    if ((entry_pc & 1U) != 0U || inItBlock(state_.it_state)
        || state_.pending_exception || state_.pending_exc_return) return false;

    const std::uint64_t generation = memory_.executionGeneration();
    auto& slot = (*jit_blocks_)[jitBlockIndex(entry_pc)];
    const std::size_t hot_index = jitBlockIndex(entry_pc);
    const bool matching_attempt = slot.attempted && slot.pc == entry_pc
        && slot.generation == generation;
    if (matching_attempt) {
        if (slot.valid) return allow_single || slot.count >= 2U;
        // A previously rejected one-op region can link reversible capsules
        // across an otherwise supported BL/branch without scalar dispatch.
        if (!allow_single || slot.count != 1U) return false;
    }
    if (jit_hot_[hot_index] < jit_compile_threshold) {
        ++jit_hot_[hot_index];
        return false;
    }

    JitBlockEntry fresh{};
    fresh.pc = entry_pc;
    fresh.generation = generation;
    fresh.attempted = true;
    std::uint32_t pc = entry_pc;
    for (std::size_t i = 0; i < JitStepOutcome::max_block; ++i) {
        std::uint32_t raw = 0;
        std::uint8_t size = 0;
        auto decoded = fetchDecode(pc, raw, size);
        if (!decoded) break;
        const JitBoundary boundary = classifyJitBoundary(*decoded);
        if (boundary == JitBoundary::exception_or_system
            || boundary == JitBoundary::floating_point
            || boundary == JitBoundary::unsupported || decoded->kind == InstrKind::it) {
            break;
        }
        fresh.ops[fresh.count] = *decoded;
        fresh.pcs[fresh.count] = pc;
        fresh.sizes[fresh.count] = size;
        fresh.raws[fresh.count] = raw;
        // fetchDecode() has just populated (or hit) the decode entry for this
        // exact PC/generation. Reuse its expensive decode-time classifications
        // instead of repeating the cycle and fast-handler switches here.
        const auto& decoded_cache =
            (*instruction_cache_)[instructionCacheIndex(pc)];
        const bool div = decoded_cache.divide_form;
        fresh.divide_form[fresh.count] = div;
        fresh.base_cycles[fresh.count] = decoded_cache.base_cycles;
        fresh.fast[fresh.count] = static_cast<JitFast>(decoded_cache.jit_fast);
        const bool touches_memory = boundary == JitBoundary::memory_may_trap
            || decoded->kind == InstrKind::ldr || decoded->kind == InstrKind::ldm
            || decoded->kind == InstrKind::pop;
        fresh.is_memory[fresh.count] = touches_memory;
        // Superblocks flow THROUGH not-taken conditional branches: only
        // unconditional control flow (b/bl, bx, PC loads) and calls end the
        // block. A taken conditional commits precisely and stops via the
        // PC-continuity check; preview costs assume taken (conservative).
        const bool cond_branch = (decoded->kind == InstrKind::b
                || decoded->kind == InstrKind::cbz
                || decoded->kind == InstrKind::cbnz)
            && decoded->condition != Condition::al;
        const bool terminates = boundary == JitBoundary::control_flow && !cond_branch;
        fresh.is_terminator[fresh.count] = terminates;
        fresh.branch_penalty[fresh.count] = boundary == JitBoundary::control_flow
            ? takenBranchPenalty(decoded->kind) : 0U;
        ++fresh.count;
        if (terminates) break;
        pc += size;
    }
    if (fresh.count == 0U || (!allow_single && fresh.count < 2U)) {
        // Remember failed/single-op compilation for this PC/generation; repeated
        // safe preflights do not repeatedly fetch and classify the same bytes.
        slot = fresh;
        return false;
    }
    fresh.valid = true;
    // Precompute pure-integer fast-block metadata. Control-flow kinds always
    // terminate the block, so mid-block fast ops are straight-line ALU and
    // the block's final suppress flag and base cycle sum are compile-time.
    bool all_fast = true;
    bool has_memory = false;
    bool has_store = false;
    std::uint16_t base_sum = 0;
    for (std::size_t i = 0; i < fresh.count; ++i) {
        // Memory ops stay on the checked slow loop even when a fast handler
        // exists, and conditional ops need per-execution evaluation, so the
        // pure fast path below only sees unconditional non-memory ops.
        if (fresh.fast[i] == JitFast::generic || fresh.is_memory[i]
            || fresh.divide_form[i]
            || fresh.ops[i].condition != Condition::al) all_fast = false;
        if (fresh.is_memory[i]) has_memory = true;
        const auto kind = fresh.ops[i].kind;
        if (kind == InstrKind::str || kind == InstrKind::strb
            || kind == InstrKind::strh || kind == InstrKind::strd
            || kind == InstrKind::stm || kind == InstrKind::push) has_store = true;
        // Loop-observation suppression per op (calls/returns), so the
        // execution loop pays one load instead of kind comparisons.
        const auto& cop = fresh.ops[i];
        fresh.suppress_obs[i] = cop.kind == InstrKind::bl || cop.kind == InstrKind::blx
            || (cop.kind == InstrKind::bx && cop.rm == 14U)
            || (cop.kind == InstrKind::mov && cop.rd == 15U && cop.rm == 14U)
            || ((cop.kind == InstrKind::pop || cop.kind == InstrKind::ldm)
                && (cop.register_list & (std::uint16_t{1U} << 15U)) != 0U);
        base_sum = static_cast<std::uint16_t>(base_sum + fresh.base_cycles[i]);
    }
    fresh.all_fast = all_fast;
    fresh.has_memory = has_memory;
    fresh.has_store = has_store;
    fresh.fast_base_cycles = base_sum;
    // Precompute the peek preview: leading prefix cycles with divide at
    // modeled maximum, branch penalties, and exactness flags.
    // Handler-covered loads/stores (including multi-word and PC-loading
    // forms) extend the optimistic prefix: backed RAM executes inline with
    // identical effects, while MMIO/faults stop the block precisely at
    // runtime. The exact prefix still stops before any memory op,
    // preserving synchronized-burst boundaries bit-for-bit.
    {
        std::uint8_t safe = 0U;
        std::uint8_t exact_len = 0U;
        bool exact_len_done = false;
        std::uint8_t exact_run = 0U;
        bool exact_run_done = false;
        for (std::size_t i = 0; i < fresh.count; ++i) {
            // Only non-handler memory stays out (PC loads without handlers
            // and forms FP classification rejected earlier): handler-covered
            // transfers execute inline for backed RAM and stop the block
            // precisely otherwise.
            if (fresh.is_memory[i] && fresh.fast[i] == JitFast::generic) break;
            if (fresh.is_memory[i] && !exact_len_done) {
                exact_len = safe;
                exact_len_done = true;
            }
            const bool is_mem = fresh.is_memory[i];
            const std::uint64_t base = fresh.divide_form[i]
                ? 12U : fresh.base_cycles[i];
            std::uint16_t penalty = fresh.branch_penalty[i];
            const auto& op = fresh.ops[i];
            bool exact = !is_mem && !fresh.divide_form[i] && op.condition == Condition::al;
            if (fresh.is_terminator[i]) {
                if (exact && (op.kind == InstrKind::b || op.kind == InstrKind::bl)) {
                    const auto target = static_cast<std::uint32_t>(
                        static_cast<std::int64_t>(fresh.pcs[i]) + 4 + op.branch_offset)
                        & ~std::uint32_t{1};
                    if (target == fresh.pcs[i] + fresh.sizes[i]) penalty = 0U;
                } else {
                    exact = false;
                }
            }
            fresh.preview_cycles[safe] = static_cast<std::uint16_t>(base + penalty);
            fresh.preview_prefix_cycles[safe + 1U] = fresh.preview_prefix_cycles[safe]
                + static_cast<std::uint32_t>(base + penalty);
            if (exact && !exact_run_done) exact_run = static_cast<std::uint8_t>(safe + 1U);
            if (!exact) exact_run_done = true;
            ++safe;
        }
        if (!exact_len_done) exact_len = safe;
        fresh.preview_count = exact_len;
        fresh.preview_extended_count = safe;
        fresh.preview_exact_prefix = exact_run;
        for (std::size_t i = 0; i < fresh.count; ++i) {
            fresh.base_prefix_cycles[i + 1U] = fresh.base_prefix_cycles[i]
                + fresh.base_cycles[i];
        }
    }
    {
        const DecodedInstruction& last = fresh.ops[fresh.count - 1U];
        fresh.fast_last_suppress = last.kind == InstrKind::bl
            || last.kind == InstrKind::blx
            || (last.kind == InstrKind::bx && last.rm == 14U)
            || (last.kind == InstrKind::mov && last.rd == 15U && last.rm == 14U)
            || ((last.kind == InstrKind::pop || last.kind == InstrKind::ldm)
                && (last.register_list & (std::uint16_t{1U} << 15U)) != 0U);
    }
    slot = fresh;
    ++jit_stats_.compilations;
    return true;
}

std::optional<CortexM4::JitStepOutcome> CortexM4::tryStepJitBlock(
    const std::size_t max_instructions) {
    return tryStepJitBlockImpl(max_instructions, false);
}

std::optional<CortexM4::JitStepOutcome> CortexM4::tryStepPreparedJitBlock(
    const std::size_t max_instructions) {
    return tryStepJitBlockImpl(max_instructions, true);
}

std::optional<CortexM4::TimedJitStepOutcome> CortexM4::tryStepReversibleJitBlock(
    const std::size_t max_instructions) {
    if (max_instructions == 0U || !prepareJitBlock(true)) return std::nullopt;
    if (memory_.reversibleRamOnly() && memory_.allMmioTrapping()) {
        return executeTrustedReversibleJitBlock(max_instructions);
    }
    TimedJitStepOutcome timed{};
    auto execution = tryStepJitBlockImpl(max_instructions, true, true,
        &timed.instruction_cycles);
    if (!execution) return std::nullopt;
    timed.execution = std::move(*execution);
    return timed;
}

std::optional<CortexM4::TimedJitStepOutcome> CortexM4::tryStepBudgetedReversibleJitBlock(
    const std::size_t max_instructions, ReversibleCycleBudget& budget) {
    if (max_instructions == 0U || !memory_.reversibleRamOnly()
        || !memory_.allMmioTrapping() || !prepareJitBlock(true)) return std::nullopt;
    return executeTrustedReversibleJitBlock(max_instructions, &budget);
}

std::optional<CortexM4::TimedJitStepOutcome> CortexM4::executeTrustedReversibleJitBlock(
    const std::size_t max_instructions, ReversibleCycleBudget* budget) {
    const std::uint32_t entry_pc = state_.r[15];
    auto& slot = (*jit_blocks_)[jitBlockIndex(entry_pc)];
    if (!slot.valid || slot.pc != entry_pc
        || slot.generation != memory_.executionGeneration()
        || state_.halted || !state_.thumb || (state_.xpsr & xpsr_t) == 0U
        || (entry_pc & 1U) != 0U || inItBlock(state_.it_state)
        || state_.pending_exception || state_.pending_exc_return) {
        return std::nullopt;
    }

    TimedJitStepOutcome timed{};
    auto& out = timed.execution;
    std::uint32_t total_cycles = 0U;
    std::uint32_t last_pc = entry_pc;
    std::uint32_t last_raw = slot.raws[0];
    std::uint8_t last_size = slot.sizes[0];
    bool last_suppress = false;
    const auto execution_count = static_cast<std::uint8_t>(
        std::min<std::size_t>(max_instructions, slot.count));
    if (budget) budget->total_instruction_cycles.fill(0U);

    for (std::uint8_t i = 0U; i < execution_count; ++i) {
        const DecodedInstruction& op = slot.ops[i];
        const std::uint32_t pc = slot.pcs[i];
        if (state_.r[15] != pc) break;
        // Stop before all generic operations, especially generic memory ops
        // that may partially commit before trapping in execute().
        const JitFast fast = slot.fast[i];
        if (fast == JitFast::generic) break;

        const std::uint8_t size = slot.sizes[i];
        // Determine the actual condition/cost and conservative admission cost
        // before installing speculative PC state or touching RAM.
        const bool condition_passed = op.condition == Condition::al
            || conditionPasses(op.condition, state_.xpsr);
        std::uint16_t cycles = condition_passed
            ? slot.divide_form[i]
                ? divideCycles(state_.readRegister(op.rm)) : slot.base_cycles[i]
            : 1U;
        const bool sequential_fetch = budget && budget->have_fetch
            && pc == budget->fetch_end;
        const std::uint16_t fetch_stall = budget && budget->fetch_stall
            ? budget->fetch_stall(budget->context, pc, sequential_fetch) : 0U;
        // Preparation precomputed a refill bound only for PC-changing forms;
        // takenBranchPenalty() deliberately has a generic default and must not
        // be applied to ordinary non-control ALU/memory instructions.
        const std::uint32_t worst_cycles = static_cast<std::uint32_t>(cycles)
            + (condition_passed ? slot.branch_penalty[i] : 0U)
            + fetch_stall;
        if (budget && (worst_cycles > budget->remaining_cycles
            || worst_cycles > std::numeric_limits<std::uint16_t>::max())) break;

        const std::uint32_t previous_instruction_address = state_.instruction_address;
        state_.instruction_address = pc;
        state_.r[15] = pc + size;
        bool handled = true;
        if (condition_passed) {
            // DIV's cost was sampled above before any potentially aliasing
            // destination write. Declines leave only speculative PC fields.
            handled = executeJitFast(op, fast, pc);
        }
        if (!handled) {
            state_.r[15] = pc;
            state_.instruction_address = previous_instruction_address;
            if (out.count == 0U && slot.decline_streak < 2U) {
                ++slot.decline_streak;
            }
            break;
        }
        if (condition_passed && state_.r[15] != pc + size) {
            cycles = static_cast<std::uint16_t>(cycles + slot.branch_penalty[i]);
        }
        const std::uint32_t charged_cycles =
            static_cast<std::uint32_t>(cycles) + fetch_stall;
        timed.instruction_cycles[out.count] = cycles;
        if (budget) {
            budget->total_instruction_cycles[out.count] =
                static_cast<std::uint16_t>(charged_cycles);
            budget->remaining_cycles -= charged_cycles;
            budget->have_fetch = true;
            budget->fetch_end = pc + size;
        }
        total_cycles += cycles;
        if (condition_passed && slot.is_memory[i]) out.memory_free = false;
        out.pcs[out.count] = pc;
        out.sizes[out.count] = size;
        ++out.count;
        last_pc = pc;
        last_raw = slot.raws[i];
        last_size = size;
        last_suppress = slot.suppress_obs[i];

        // Branches and PC-loading handlers terminate at the actual PC change;
        // pending EXC_RETURN is also produced only by a terminating PC op.
        if (state_.r[15] != pc + size || slot.is_terminator[i]) break;
    }

    if (out.count == 0U) {
        ++jit_stats_.fallbacks;
        return std::nullopt;
    }
    slot.decline_streak = 0U;
    auto& result = out.result;
    result.reason = StopReason::step_complete;
    result.instruction_address = last_pc;
    result.raw = last_raw;
    result.instruction_size = last_size;
    result.instructions = out.count;
    result.cycles = static_cast<std::uint16_t>(total_cycles);
    result.suppress_loop_observation = last_suppress;
    ++jit_stats_.block_executions;
    jit_stats_.block_instructions += out.count;
    return timed;
}

std::optional<CortexM4::JitStepOutcome> CortexM4::tryStepJitBlockImpl(
    const std::size_t max_instructions, const bool prepared, const bool fast_only,
    std::array<std::uint16_t, JitStepOutcome::max_block>* instruction_cycles) {
#if FIL_HAS_LLVM_JIT
    if (max_instructions != 0U && !fast_only) ++native_clock_;
#endif
    const std::uint32_t entry_pc = state_.r[15];
    if (max_instructions == 0U) {
        if (!prepared) ++jit_stats_.fallbacks;
        return std::nullopt;
    }
    if (!prepared) {
        if (!prepareJitBlock()) {
            ++jit_stats_.fallbacks;
            return std::nullopt;
        }
    } else {
        // Caller prepared moments ago on this thread; recheck cheaply.
        const auto& slot = (*jit_blocks_)[jitBlockIndex(entry_pc)];
        if (!slot.valid || slot.pc != entry_pc
            || slot.generation != memory_.executionGeneration()
            || state_.halted || !state_.thumb || (state_.xpsr & xpsr_t) == 0U
            || (entry_pc & 1U) != 0U || inItBlock(state_.it_state)
            || state_.pending_exception || state_.pending_exc_return) {
            return std::nullopt;
        }
    }
    const std::uint64_t generation = memory_.executionGeneration();
    const bool trapping = memory_.mmioTrapping();
    auto& slot = (*jit_blocks_)[jitBlockIndex(entry_pc)];
    // Execute the cached block with exact single-step semantics.
    JitStepOutcome out{};
    // Restart storage lives at function scope so all early exits can cross it.
    CpuState checkpoint{};
    bool have_checkpoint = false;
    std::uint32_t total_cycles = 0;
    std::uint32_t last_pc = entry_pc;
    std::uint32_t last_raw = slot.raws[0];
    std::uint8_t last_size = slot.sizes[0];
    bool last_suppress = false;
    const std::uint8_t execution_count = static_cast<std::uint8_t>(
        std::min<std::size_t>(max_instructions, slot.count));
    // Only stores can change executable generations (loads never do), so
    // load-only blocks pay a single pre-loop generation check.
    const bool check_generation_per_op = slot.has_store;
#if FIL_HAS_LLVM_JIT
    // Previews alone must not spend native slots. Compile only a block that
    // is actually admitted for multi-instruction execution by the scheduler.
    if (!fast_only && execution_count > 1U
        && !slot.native_function && !slot.native_attempted) {
        if (slot.native_hits < native_block_compile_threshold) ++slot.native_hits;
        else prepareNativeBlock(slot);
    }
    if (!fast_only && slot.native_function && state_.it_state == 0U) {
        // Only independently lowered, pure unconditional instructions reach
        // this path. The scheduler still controls the allowed prefix length.
        const auto native_count = std::min(execution_count, slot.native_count);
        touchNativeKernel(slot.native_slot);
        slot.native_function(&state_, native_count);
        std::memcpy(out.pcs.data(), slot.pcs.data(), native_count * sizeof(std::uint32_t));
        std::memcpy(out.sizes.data(), slot.sizes.data(), native_count * sizeof(std::uint8_t));
        total_cycles = slot.base_prefix_cycles[native_count];
        const auto last = static_cast<std::size_t>(native_count - 1U);
        if (state_.r[15] != slot.pcs[last] + slot.sizes[last]) {
            total_cycles += slot.branch_penalty[last];
        }
        out.count = native_count;
        last_pc = slot.pcs[last];
        last_raw = slot.raws[last];
        last_size = slot.sizes[last];
        last_suppress = slot.ops[last].kind == InstrKind::bl;
        ++jit_stats_.native_executions;
        jit_stats_.native_instructions += native_count;
    }
#endif
    // Pure-integer fast block: no memory ops (no traps/generation changes),
    // no generic ops (no conditions/faults), IT/pending checked at prepare.
    // Control flow always terminates the block, so mid-block PCs are
    // sequential by construction and per-op checks collapse to one tail lag.
    if (!fast_only && out.count == 0U && slot.all_fast && !slot.has_memory) {
        for (std::uint8_t i = 0U; i < execution_count; ++i) {
            const DecodedInstruction& op = slot.ops[i];
            const std::uint32_t pc = slot.pcs[i];
            const std::uint8_t size = slot.sizes[i];
            state_.r[15] = pc + size;
            const bool handled = executeJitFast(op, slot.fast[i], pc);
            assert(handled);
            static_cast<void>(handled);
            if (slot.is_terminator[i]) break;
        }
        // Bulk-commit per-instruction metadata for flash-stall accounting.
        const std::uint8_t last = static_cast<std::uint8_t>(execution_count - 1U);
        const std::uint8_t committed = execution_count;
        for (std::uint8_t i = 0U; i < committed; ++i) {
            out.pcs[i] = slot.pcs[i];
            out.sizes[i] = slot.sizes[i];
        }
        out.count = committed;
        state_.instruction_address = slot.pcs[last];
        std::uint32_t total = slot.fast_base_cycles;
        if (committed < slot.count) {
            // Capped prefix: subtract the unexecuted suffix base cycles.
            // The prefix holds straight-line ops only (the terminator, if
            // any, is the final block op), so no branch penalty applies.
            for (std::uint8_t i = committed; i < slot.count; ++i) {
                total = static_cast<std::uint32_t>(total - slot.base_cycles[i]);
            }
            last_suppress = false;
        } else {
            if (state_.r[15] != slot.pcs[last] + slot.sizes[last]) {
                total += slot.branch_penalty[last];
            }
            last_suppress = slot.fast_last_suppress;
        }
        last_pc = slot.pcs[last];
        last_raw = slot.raws[last];
        last_size = slot.sizes[last];
        total_cycles = total;
        goto commit_block;
    }
    // IT state and pending traffic are validated at prepare, and no admitted
    // op produces them mid-block: control flow terminates the block while
    // system/FP/IT forms never compile into one. Likewise only stores can
    // change executable generations (loads never do), so load-only blocks
    // pay a single generation check. One pre-loop probe covers all three.
    have_checkpoint = false;
    if (inItBlock(state_.it_state)
        || state_.pending_exception || state_.pending_exc_return) {
        if (out.count == 0U) {
            ++jit_stats_.fallbacks;
            return std::nullopt;
        }
        goto commit_block;
    }
    for (std::uint8_t i = out.count; i < execution_count; ++i) {
        const DecodedInstruction& op = slot.ops[i];
        const std::uint32_t pc = slot.pcs[i];
        // A committed store may have changed executable memory. Never run
        // another instruction decoded from the previous generation.
        if (check_generation_per_op
            && memory_.executionGeneration() != generation) {
            break;
        }
        // Self-modifying code or eviction aborts precisely at the boundary.
        if (state_.r[15] != pc) break;
        // Reversible capsules never enter the generic executor: generic
        // memory forms may partially commit before reporting a trap.
        if (fast_only && slot.fast[i] == JitFast::generic) break;
        if (slot.is_memory[i] && !fast_only) out.memory_free = false;
        const std::uint8_t size = slot.sizes[i];
        const JitFast fast = slot.fast[i];
        const std::uint32_t previous_instruction_address = state_.instruction_address;
        state_.instruction_address = pc;
        state_.r[15] = pc + size;
        if (fast != JitFast::generic) {
            // Failed conditions retire for 1 cycle with no effect and no
            // dispatch; handlers only ever see passing conditions.
            if (op.condition != Condition::al
                && !conditionPasses(op.condition, state_.xpsr)) {
                total_cycles += 1U;
                if (instruction_cycles) (*instruction_cycles)[out.count] = 1U;
                last_pc = pc;
                last_raw = slot.raws[i];
                last_size = size;
                last_suppress = false;
                out.pcs[out.count] = pc;
                out.sizes[out.count] = size;
                ++out.count;
                continue;
            }
            // The shared handler keeps single-step and block ALU semantics aligned.
            // Declinable handlers (backed-memory loads/stores) return false
            // for MMIO/faults with untouched state: rewind the speculative
            // PC and stop -- the access runs at its exact single-step
            // boundary next. RAM success needs no restart checkpoint (a
            // backed access cannot trap), so none is taken here.
            const std::uint16_t op_cycles = slot.divide_form[i]
                ? divideCycles(state_.readRegister(op.rm)) : slot.base_cycles[i];
            const bool handled = executeJitFast(op, fast, pc);
            if (!handled) {
                state_.r[15] = pc;
                if (fast_only) {
                    state_.instruction_address = previous_instruction_address;
                    break;
                }
                // Poll-at-entry: remember MMIO/fault PCs so the gate
                // singles them directly instead of re-wasting prepare and
                // preview. Clears on any commit or slot rebuild below.
                if (out.count == 0U && slot.decline_streak < 2U) {
                    ++slot.decline_streak;
                }
                break;
            }
            if (handled) {
            if (slot.is_memory[i]) out.memory_free = false;
            std::uint16_t cycles = op_cycles;
            if (state_.r[15] != pc + size) {
                cycles = static_cast<std::uint16_t>(
                    cycles + slot.branch_penalty[i]);
            }
            if (instruction_cycles) (*instruction_cycles)[out.count] = cycles;
            total_cycles += cycles;
            last_pc = pc;
            last_raw = slot.raws[i];
            last_size = size;
            last_suppress = slot.suppress_obs[i];
            out.pcs[out.count] = pc;
            out.sizes[out.count] = size;
            ++out.count;
            if (slot.is_terminator[i]) break;
            continue;
            }
        }
        // Save the exact pre-instruction state on the generic path: execute()
        // may report a synchronization trap after the sequential PC install.
        // (Fast handlers need none: RAM success cannot trap, decline rewinds.)
        // Storage is function-scoped so block-entry exits can cross it.
        have_checkpoint = slot.is_memory[i] && trapping;
        if (have_checkpoint) checkpoint = state_;
        ++jit_stats_.block_generic;
        const bool cond_pass = conditionPasses(op.condition, state_.xpsr);
        std::uint16_t base = 1U;
        if (cond_pass) {
            base = slot.divide_form[i]
                ? divideCycles(state_.readRegister(op.rm))
                : slot.base_cycles[i];
        }
        if (!cond_pass) {
            state_.r[15] = pc + size;
            total_cycles += 1U;
            last_pc = pc;
            last_raw = slot.raws[i];
            last_size = size;
            last_suppress = false;
            out.pcs[out.count] = pc;
            out.sizes[out.count] = size;
            ++out.count;
            continue;
        }
        const StopReason stop = execute(op, last_diagnostic_);
        std::uint16_t cycles = base;
        if (stop == StopReason::step_complete && state_.r[15] != pc + size) {
            cycles = static_cast<std::uint16_t>(
                cycles + slot.branch_penalty[i]);
        }
        if (stop == StopReason::synchronization_required) {
            if (have_checkpoint) state_ = checkpoint;
            last_diagnostic_.instruction_address = pc;
            last_diagnostic_.raw = slot.raws[i];
            last_diagnostic_.instruction_size = size;
            capture(last_diagnostic_);
            if (out.count == 0U) {
                // Nothing committed: report the trap like a single step.
                out.result.reason = StopReason::synchronization_required;
                out.result.instruction_address = pc;
                out.result.raw = slot.raws[i];
                out.result.instruction_size = size;
                out.result.instructions = 0;
                out.result.cycles = 0;
                return out;
            }
            break; // Commit prefix, trap on next interpreter step.
        }
        if (stop != StopReason::step_complete) {
            last_diagnostic_.instruction_address = pc;
            last_diagnostic_.raw = slot.raws[i];
            last_diagnostic_.instruction_size = size;
            capture(last_diagnostic_);
            total_cycles += cycles;
            last_pc = pc;
            last_raw = slot.raws[i];
            last_size = size;
            out.pcs[out.count] = pc;
            out.sizes[out.count] = size;
            ++out.count;
            out.result.reason = stop;
            out.result.instruction_address = last_pc;
            out.result.raw = last_raw;
            out.result.instruction_size = last_size;
            out.result.instructions = out.count;
            out.result.cycles = static_cast<std::uint16_t>(total_cycles);
            out.result.suppress_loop_observation = slot.suppress_obs[i];
            ++jit_stats_.block_executions;
            jit_stats_.block_instructions += out.count;
            return out;
        }
        total_cycles += cycles;
        last_pc = pc;
        last_raw = slot.raws[i];
        last_size = size;
        last_suppress = slot.suppress_obs[i];
        out.pcs[out.count] = pc;
        out.sizes[out.count] = size;
        ++out.count;
        if (slot.is_terminator[i]) break;
    }
    commit_block:
    if (out.count == 0U) {
        ++jit_stats_.fallbacks;
        return std::nullopt;
    }
    slot.decline_streak = 0U;
    out.result.reason = StopReason::step_complete;
    out.result.instruction_address = last_pc;
    out.result.raw = last_raw;
    out.result.instruction_size = last_size;
    out.result.instructions = out.count;
    out.result.cycles = static_cast<std::uint16_t>(total_cycles);
    out.result.suppress_loop_observation = last_suppress;
    ++jit_stats_.block_executions;
    jit_stats_.block_instructions += out.count;
    return out;
}

bool CortexM4::prepareAndPeekJitBlock(
    const std::size_t max_instructions, JitBlockPreview& preview_out) {
    if (max_instructions == 0U || !prepareJitBlock()) return false;
    // prepareJitBlock validated architecture state and slot readiness; a
    // single lookup serves the preview fill (no second readiness probe).
    const std::uint32_t entry_pc = state_.r[15];
    const auto& slot = (*jit_blocks_)[jitBlockIndex(entry_pc)];
    if (!slot.valid || slot.pc != entry_pc
        || slot.generation != memory_.executionGeneration()) return false;
    const std::size_t count = std::min<std::size_t>(max_instructions, slot.count);
    const std::size_t safe_count = std::min(count,
        static_cast<std::size_t>(slot.preview_extended_count));
    if (safe_count == 0U || slot.decline_streak >= 2U) return false;
    std::memcpy(preview_out.pcs.data(), slot.pcs.data(),
        safe_count * sizeof(preview_out.pcs[0]));
    std::memcpy(preview_out.sizes.data(), slot.sizes.data(),
        safe_count * sizeof(preview_out.sizes[0]));
    std::memcpy(preview_out.instruction_cycles.data(), slot.preview_cycles.data(),
        safe_count * sizeof(preview_out.instruction_cycles[0]));
    preview_out.max_cycles = slot.preview_prefix_cycles[safe_count];
    preview_out.exact_cycle_prefix_count = static_cast<std::uint8_t>(
        std::min<std::size_t>(safe_count, slot.preview_exact_prefix));
    preview_out.cycles_exact = slot.preview_exact_prefix >= safe_count;
    preview_out.count = static_cast<std::uint8_t>(safe_count);
    return true;
}

bool CortexM4::prepareAndPeekExactJitBlock(
    const std::size_t max_instructions, JitBlockPreview& preview_out) {
    if (max_instructions == 0U || !prepareJitBlock()) return false;
    const std::uint32_t entry_pc = state_.r[15];
    const auto& slot = (*jit_blocks_)[jitBlockIndex(entry_pc)];
    if (!slot.valid || slot.pc != entry_pc
        || slot.generation != memory_.executionGeneration()) return false;
    const std::size_t count = std::min<std::size_t>(max_instructions, slot.count);
    const std::size_t safe_count = std::min(count,
        static_cast<std::size_t>(slot.preview_count));
    if (safe_count == 0U) return false;
    std::memcpy(preview_out.pcs.data(), slot.pcs.data(),
        safe_count * sizeof(preview_out.pcs[0]));
    std::memcpy(preview_out.sizes.data(), slot.sizes.data(),
        safe_count * sizeof(preview_out.sizes[0]));
    std::memcpy(preview_out.instruction_cycles.data(), slot.preview_cycles.data(),
        safe_count * sizeof(preview_out.instruction_cycles[0]));
    preview_out.max_cycles = slot.preview_prefix_cycles[safe_count];
    preview_out.exact_cycle_prefix_count = static_cast<std::uint8_t>(
        std::min<std::size_t>(safe_count, slot.preview_exact_prefix));
    preview_out.cycles_exact = slot.preview_exact_prefix >= safe_count;
    preview_out.count = static_cast<std::uint8_t>(safe_count);
    return true;
}

std::optional<CortexM4::JitBlockPreview> CortexM4::peekJitBlock(
    const std::size_t max_instructions) const noexcept {
    if (max_instructions == 0U || !jitBlockReady()) return std::nullopt;
    const std::uint32_t entry_pc = state_.r[15];
    const auto& slot = (*jit_blocks_)[jitBlockIndex(entry_pc)];
    const std::size_t count = std::min<std::size_t>(max_instructions, slot.count);
    if (count == 0U) return std::nullopt;
    // Preview only the maximal memory-free prefix; the memory instruction
    // itself must run at an exact board-time boundary. Cycle data is
    // precomputed at compile time; apply the caller cap here.
    const std::size_t safe_count = std::min(count, static_cast<std::size_t>(slot.preview_count));
    if (safe_count == 0U) return std::nullopt;

    JitBlockPreview preview{};
    std::memcpy(preview.pcs.data(), slot.pcs.data(),
        safe_count * sizeof(preview.pcs[0]));
    std::memcpy(preview.sizes.data(), slot.sizes.data(),
        safe_count * sizeof(preview.sizes[0]));
    std::memcpy(preview.instruction_cycles.data(), slot.preview_cycles.data(),
        safe_count * sizeof(preview.instruction_cycles[0]));
    preview.max_cycles = slot.preview_prefix_cycles[safe_count];
    preview.exact_cycle_prefix_count = static_cast<std::uint8_t>(
        std::min<std::size_t>(safe_count, slot.preview_exact_prefix));
    preview.cycles_exact = slot.preview_exact_prefix >= safe_count;
    preview.count = static_cast<std::uint8_t>(safe_count);
    return preview;
}

bool CortexM4::peekPredictableCycles(std::uint16_t& cycles_out) const noexcept {
    if (state_.halted) return false;
    const std::uint32_t pc = state_.r[15];
    if (!state_.thumb || (state_.xpsr & xpsr_t) == 0U || (pc & 1U) != 0U) return false;
    const auto& cache = (*instruction_cache_)[instructionCacheIndex(pc)];
    if (cache.generation != memory_.executionGeneration() || cache.pc != pc) return false;
    const DecodedInstruction& decoded = cache.decoded;
    const std::uint8_t size = cache.size;
    const std::uint32_t fallthrough = pc + size;
    // Memory-loaded targets (LDR/MOV-to-PC, LDM/POP-to-PC) need execute-
    // time knowledge (loaded value, fault behavior), so refuse. Direct
    // branches evaluate exactly from pre-state below, replicating stepFast's
    // `r15 != fallthrough` refill rule including pathological coincidences.
    switch (decoded.kind) {
    case InstrKind::mov:
    case InstrKind::ldr:
    case InstrKind::ldrb:
    case InstrKind::ldrh:
    case InstrKind::ldrsb:
    case InstrKind::ldrsh:
    case InstrKind::ldrd:
        if (decoded.rd == 15U) return false;
        break;
    case InstrKind::ldm:
    case InstrKind::pop:
        if ((decoded.register_list & (std::uint16_t{1U} << 15U)) != 0U) return false;
        break;
    default:
        break;
    }
    const bool was_in_it = inItBlock(state_.it_state);
    const Condition effective = decoded.kind == InstrKind::it
        ? Condition::al
        : was_in_it ? currentItCondition(state_.it_state) : decoded.condition;
    const bool cond_pass = conditionPasses(effective, state_.xpsr);
    if (!cond_pass) {
        cycles_out = 1U;
        return true;
    }
    std::uint16_t base = 0U;
    bool branch_taken = false;
    bool exc_return = false;
    switch (decoded.kind) {
    case InstrKind::b:
    case InstrKind::bl: {
        const auto wide = static_cast<std::int64_t>(pc) + 4 + decoded.branch_offset;
        const std::uint32_t target = static_cast<std::uint32_t>(wide) & ~std::uint32_t{1};
        base = cache.base_cycles;
        branch_taken = target != fallthrough;
        break;
    }
    case InstrKind::bx:
    case InstrKind::blx: {
        const std::uint32_t target = state_.readRegister(decoded.rm);
        if (decoded.kind == InstrKind::bx && isExceptionReturn(target)) {
            exc_return = true;
            base = cache.base_cycles;
            break;
        }
        if ((target & 1U) == 0U) return false; // fault path needs the stepper
        base = cache.base_cycles;
        branch_taken = (target & ~std::uint32_t{1}) != fallthrough;
        break;
    }
    case InstrKind::cbz:
    case InstrKind::cbnz: {
        const bool zero = state_.readRegister(decoded.rn) == 0U;
        const bool take = decoded.kind == InstrKind::cbz ? zero : !zero;
        base = cache.base_cycles;
        branch_taken = take
            && ((state_.architecturalPcForRead() + decoded.imm) & ~std::uint32_t{1})
                != fallthrough;
        break;
    }
    case InstrKind::udiv:
    case InstrKind::sdiv:
        cycles_out = divideCycles(state_.readRegister(cache.decoded.rm));
        return true;
    default:
        base = cache.base_cycles;
        break;
    }
    if (exc_return) {
        // EXC_RETURN pseudo-branch: the stepper leaves the PC sequential
        // and defers to the boundary handler, which breaks any burst.
        cycles_out = base;
        return true;
    }
    cycles_out = static_cast<std::uint16_t>(
        base + (branch_taken ? takenBranchPenalty(decoded.kind) : 0U)
    );
    return true;
}

RunResult CortexM4::step() {
    const FastStepResult fast = stepFast();
    RunResult result;
    result.reason = fast.reason;
    result.instructions = fast.instructions;
    result.cycles = fast.cycles;
    if (fast.reason == StopReason::step_complete) {
        result.diagnostic.instruction_address = fast.instruction_address;
        result.diagnostic.raw = fast.raw;
        result.diagnostic.instruction_size = fast.instruction_size;
        capture(result.diagnostic);
    } else {
        result.diagnostic = last_diagnostic_;
    }
    return result;
}

RunResult CortexM4::run(const std::uint64_t instruction_budget) {
    RunResult aggregate;
    aggregate.reason = StopReason::instruction_budget;
    aggregate.diagnostic.instruction_address = state_.r[15];
    capture(aggregate.diagnostic);

    FastStepResult one;
    for (std::uint64_t count = 0; count < instruction_budget; ++count) {
        one = stepFast();
        aggregate.instructions += one.instructions;
        aggregate.cycles += one.cycles;
        if (one.reason != StopReason::step_complete) {
            aggregate.reason = one.reason;
            aggregate.diagnostic = last_diagnostic_;
            return aggregate;
        }
    }
    if (instruction_budget != 0U) {
        aggregate.diagnostic.instruction_address = one.instruction_address;
        aggregate.diagnostic.raw = one.raw;
        aggregate.diagnostic.instruction_size = one.instruction_size;
    }
    capture(aggregate.diagnostic);
    return aggregate;
}

const char* stopReasonName(const StopReason reason) noexcept {
    switch (reason) {
    case StopReason::step_complete: return "step-complete";
    case StopReason::instruction_budget: return "instruction-budget";
    case StopReason::breakpoint: return "breakpoint";
    case StopReason::halted: return "halted";
    case StopReason::bus_fault: return "bus-fault";
    case StopReason::synchronization_required: return "synchronization-required";
    case StopReason::undefined_instruction: return "undefined-instruction";
    case StopReason::invalid_state: return "invalid-state";
    }
    return "unknown";
}

} // namespace fil::cpu
