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
    state.xpsr &= ~(xpsr_n | xpsr_z);
    if ((value & 0x80000000U) != 0U) state.xpsr |= xpsr_n;
    if (value == 0U) state.xpsr |= xpsr_z;
}

inline void jitSetNzc(CpuState& state, const std::uint32_t value, const bool carry) noexcept {
    jitSetNz(state, value);
    state.xpsr &= ~xpsr_c;
    if (carry) state.xpsr |= xpsr_c;
}

inline void jitSetNzcv(CpuState& state, const AddResult& result) noexcept {
    state.xpsr &= ~(xpsr_n | xpsr_z | xpsr_c | xpsr_v);
    if (result.n) state.xpsr |= xpsr_n;
    if (result.z) state.xpsr |= xpsr_z;
    if (result.c) state.xpsr |= xpsr_c;
    if (result.v) state.xpsr |= xpsr_v;
}

} // namespace

CortexM4::JitFast CortexM4::classifyJitFast(const DecodedInstruction& op) noexcept {
    // Inlined handlers require unconditional (non-IT) integer ops; anything
    // else stays on the exact generic execute() path.
    if (op.condition != Condition::al) return JitFast::generic;
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
    case InstrKind::lsl:
    case InstrKind::lsr:
    case InstrKind::asr:
    case InstrKind::ror:
    case InstrKind::rrx:
        return op.rd != 15U && op.rn != 15U && op.rm != 15U
            && (op.form == OperandForm::immediate
                || op.form == OperandForm::register_value)
            ? JitFast::shift : JitFast::generic;
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

CortexM4::CortexM4(mem::MemoryBus& memory) noexcept : memory_(memory) {
    assert(decoderTablesHaveNoOverlaps());
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
    for (auto& entry : instruction_cache_) {
        if (entry.native_function != old_function) continue;
        entry.native_function = nullptr;
        entry.native_hits = 0U;
        entry.native_attempted = false;
    }
    for (auto& entry : jit_blocks_) {
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
    if (!LlvmJit::supports(instruction)) return false;
    if (!ensureNativeCompiler()) {
        entry.native_hits = 0U;
        entry.native_attempted = false;
        return false;
    }
    auto kernel = native_state_->compiler->compile(
        std::span<const NativeJitInstruction>{&instruction, 1U}, native_jit_error_);
    if (!kernel || !kernel->entryPoint()) {
        ++jit_stats_.native_compilation_failures;
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
    return stepFastImpl(true);
}

FastStepResult CortexM4::stepFastImpl(const bool use_jit) {
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

    auto& cache = instruction_cache_[(pc >> 1U) & (instruction_cache_entries - 1U)];
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
        decoded = &cache.decoded;
    }
    result.instruction_size = instruction_size;
    const bool stack_return =
        (decoded->kind == InstrKind::pop || decoded->kind == InstrKind::ldm)
        && (decoded->register_list & (std::uint16_t{1U} << 15U)) != 0U;
    result.suppress_loop_observation = decoded->kind == InstrKind::bl
        || decoded->kind == InstrKind::blx
        || (decoded->kind == InstrKind::bx && decoded->rm == 14U)
        || (decoded->kind == InstrKind::mov
            && decoded->rd == 15U && decoded->rm == 14U)
        || stack_return;

    // Memoized base cost (one load) sampled before execute() mutates
    // architectural state. Only DIV reads a live operand; every other form
    // uses the decode-time table value.
    const bool condition_passed = conditionPasses(
        decoded->kind == InstrKind::it
            ? Condition::al
            : inItBlock(state_.it_state)
                ? currentItCondition(state_.it_state)
                : decoded->condition,
        state_.xpsr
    );
    std::uint16_t base_cycles = 1U;
    if (condition_passed) [[likely]] {
        base_cycles = cache.divide_form
            ? divideCycles(state_.readRegister(decoded->rm))
            : cache.base_cycles;
    }

    // Heap-boxed: a 250-byte optional here triggers per-instruction stack
    // probes (chkstk) in the hot frame; the trapping path is cold/rare.
    std::unique_ptr<CpuState> restart_state;
    if (memory_.mmioTrapping()) restart_state = std::make_unique<CpuState>(state_);
    state_.instruction_address = pc;
    state_.r[15] = pc + instruction_size;
    const bool was_in_it = inItBlock(state_.it_state);
    std::optional<DecodedInstruction> it_adjusted;
    if (was_in_it && decoded->set_flags && suppressImplicitFlagsInIt(decoded->kind)) {
        it_adjusted = *decoded;
        it_adjusted->set_flags = false;
        decoded = &*it_adjusted;
    }

    StopReason stop = StopReason::step_complete;
    if (condition_passed) [[likely]] {
        bool jit_handled = false;
        if (use_jit && !was_in_it) {
            jit_handled = executeNativeInstruction(cache);
            if (!jit_handled) {
                const auto fast = static_cast<JitFast>(cache.jit_fast);
                jit_handled = fast != JitFast::generic
                    && executeJitFast(*decoded, fast, pc);
            }
        }
        if (!jit_handled) stop = execute(*decoded, last_diagnostic_);
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
            result.cycles + takenBranchPenalty(decoded->kind)
        );
    }
    if (stop != StopReason::step_complete) [[unlikely]] {
        last_diagnostic_.instruction_address = pc;
        last_diagnostic_.raw = result.raw;
        last_diagnostic_.instruction_size = instruction_size;
        if (stop == StopReason::synchronization_required && restart_state) {
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
        const AddResult sum = addWithCarry(left, subtract ? ~right : right, carry_in);
        state_.writeRegister(op.rd, sum.value);
        if (op.set_flags) jitSetNzcv(state_, sum);
        return true;
    }
    case JitFast::cmp_imm:
    case JitFast::cmp_reg: {
        const std::uint32_t left = state_.readRegister(op.rn);
        const std::uint32_t right = fast == JitFast::cmp_reg
            ? shiftC(state_.readRegister(op.rm), op.shift_type, op.shift_amount,
                     (state_.xpsr & xpsr_c) != 0U).value
            : op.imm;
        const AddResult sum = op.kind == InstrKind::cmp
            ? addWithCarry(left, ~right, true)
            : addWithCarry(left, right, false);
        jitSetNzcv(state_, sum);
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
    case JitFast::generic:
        return false;
    }
    return false;
}

std::optional<DecodedInstruction> CortexM4::fetchDecode(
    const std::uint32_t pc, std::uint32_t& raw_out, std::uint8_t& size_out) {
    const std::uint64_t generation = memory_.executionGeneration();
    auto& cache = instruction_cache_[(pc >> 1U) & (instruction_cache_entries - 1U)];
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
    const auto& slot = jit_blocks_[(entry_pc >> 1U) & (jit_block_entries - 1U)];
    return slot.valid && slot.pc == entry_pc
        && slot.generation == memory_.executionGeneration();
}

bool CortexM4::prepareJitBlock() {
    if (state_.halted || !state_.thumb || (state_.xpsr & xpsr_t) == 0U) return false;
    const std::uint32_t entry_pc = state_.r[15];
    if ((entry_pc & 1U) != 0U || inItBlock(state_.it_state)
        || state_.pending_exception || state_.pending_exc_return) return false;

    const std::uint64_t generation = memory_.executionGeneration();
    auto& slot = jit_blocks_[(entry_pc >> 1U) & (jit_block_entries - 1U)];
    const std::size_t hot_index = (entry_pc >> 1U) & (jit_block_entries - 1U);
    const bool matching_attempt = slot.attempted && slot.pc == entry_pc
        && slot.generation == generation;
    if (matching_attempt) return slot.valid;
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
        const bool div = decoded->kind == InstrKind::udiv
            || decoded->kind == InstrKind::sdiv;
        fresh.divide_form[fresh.count] = div;
        fresh.base_cycles[fresh.count] = div ? 7U : basePipelineCycles(*decoded);
        fresh.fast[fresh.count] = classifyJitFast(*decoded);
        const bool touches_memory = boundary == JitBoundary::memory_may_trap
            || decoded->kind == InstrKind::ldr || decoded->kind == InstrKind::ldm
            || decoded->kind == InstrKind::pop;
        fresh.is_memory[fresh.count] = touches_memory;
        fresh.is_terminator[fresh.count] = boundary == JitBoundary::control_flow;
        fresh.branch_penalty[fresh.count] = boundary == JitBoundary::control_flow
            ? takenBranchPenalty(decoded->kind) : 0U;
        ++fresh.count;
        if (boundary == JitBoundary::control_flow) break;
        pc += size;
    }
    if (fresh.count < 2U) {
        // Remember failed/single-op compilation for this PC/generation; repeated
        // safe preflights do not repeatedly fetch and classify the same bytes.
        slot = fresh;
        return false;
    }
    fresh.valid = true;
    slot = fresh;
    ++jit_stats_.compilations;
    return true;
}

std::optional<CortexM4::JitStepOutcome> CortexM4::tryStepJitBlock(
    const std::size_t max_instructions) {
#if FIL_HAS_LLVM_JIT
    if (max_instructions != 0U) ++native_clock_;
#endif
    if (max_instructions == 0U || !prepareJitBlock()) {
        if (max_instructions != 0U) ++jit_stats_.fallbacks;
        return std::nullopt;
    }
    const std::uint32_t entry_pc = state_.r[15];
    const std::uint64_t generation = memory_.executionGeneration();
    auto& slot = jit_blocks_[(entry_pc >> 1U) & (jit_block_entries - 1U)];
    // Execute the cached block with exact single-step semantics.
    JitStepOutcome out{};
    std::uint32_t total_cycles = 0;
    std::uint32_t last_pc = entry_pc;
    std::uint32_t last_raw = slot.raws[0];
    std::uint8_t last_size = slot.sizes[0];
    bool last_suppress = false;
    const std::uint8_t execution_count = static_cast<std::uint8_t>(
        std::min<std::size_t>(max_instructions, slot.count));
#if FIL_HAS_LLVM_JIT
    // Previews alone must not spend native slots. Compile only a block that
    // is actually admitted for multi-instruction execution by the scheduler.
    if (execution_count > 1U && !slot.native_function && !slot.native_attempted) {
        if (slot.native_hits < native_block_compile_threshold) ++slot.native_hits;
        else prepareNativeBlock(slot);
    }
    if (slot.native_function && state_.it_state == 0U) {
        // Only independently lowered, pure unconditional instructions reach
        // this path. The scheduler still controls the allowed prefix length.
        const auto native_count = std::min(execution_count, slot.native_count);
        touchNativeKernel(slot.native_slot);
        slot.native_function(&state_, native_count);
        for (std::uint8_t i = 0U; i < native_count; ++i) {
            out.pcs[i] = slot.pcs[i];
            out.sizes[i] = slot.sizes[i];
            total_cycles += slot.base_cycles[i];
        }
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
    for (std::uint8_t i = out.count; i < execution_count; ++i) {
        const DecodedInstruction& op = slot.ops[i];
        const std::uint32_t pc = slot.pcs[i];
        // A committed write may have changed executable memory. Never run
        // another instruction decoded from the previous execution generation.
        if (memory_.executionGeneration() != generation) break;
        // Self-modifying code or eviction aborts precisely at the boundary.
        if (state_.r[15] != pc) break;
        if (inItBlock(state_.it_state)) break;
        if (state_.pending_exception || state_.pending_exc_return) break;
        const std::uint8_t size = slot.sizes[i];
        const JitFast fast = slot.fast[i];
        // Save the exact pre-instruction state: execute() may report a
        // synchronization trap after the sequential PC has been installed.
        std::optional<CpuState> checkpoint;
        if (slot.is_memory[i] && memory_.mmioTrapping()) checkpoint = state_;
        state_.instruction_address = pc;
        state_.r[15] = pc + size;
        if (fast != JitFast::generic) {
            // The shared handler keeps single-step and block ALU semantics aligned.
            const bool handled = executeJitFast(op, fast, pc);
            assert(handled);
            static_cast<void>(handled);
            std::uint16_t cycles = slot.base_cycles[i];
            if (state_.r[15] != pc + size) {
                cycles = static_cast<std::uint16_t>(
                    cycles + slot.branch_penalty[i]);
            }
            total_cycles += cycles;
            last_pc = pc;
            last_raw = slot.raws[i];
            last_size = size;
            const bool sret_fast =
                (op.kind == InstrKind::pop || op.kind == InstrKind::ldm)
                && (op.register_list & (std::uint16_t{1U} << 15U)) != 0U;
            last_suppress = op.kind == InstrKind::bl || op.kind == InstrKind::blx
                || (op.kind == InstrKind::bx && op.rm == 14U)
                || (op.kind == InstrKind::mov && op.rd == 15U && op.rm == 14U)
                || sret_fast;
            out.pcs[out.count] = pc;
            out.sizes[out.count] = size;
            ++out.count;
            if (slot.is_terminator[i]) break;
            continue;
        }
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
            if (checkpoint) state_ = *checkpoint;
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
            const bool sret = (op.kind == InstrKind::pop || op.kind == InstrKind::ldm)
                && (op.register_list & (std::uint16_t{1U} << 15U)) != 0U;
            out.result.suppress_loop_observation = op.kind == InstrKind::bl
                || op.kind == InstrKind::blx
                || (op.kind == InstrKind::bx && op.rm == 14U)
                || (op.kind == InstrKind::mov && op.rd == 15U && op.rm == 14U)
                || sret;
            ++jit_stats_.block_executions;
            jit_stats_.block_instructions += out.count;
            return out;
        }
        total_cycles += cycles;
        last_pc = pc;
        last_raw = slot.raws[i];
        last_size = size;
        const bool sret2 = (op.kind == InstrKind::pop || op.kind == InstrKind::ldm)
            && (op.register_list & (std::uint16_t{1U} << 15U)) != 0U;
        last_suppress = op.kind == InstrKind::bl || op.kind == InstrKind::blx
            || (op.kind == InstrKind::bx && op.rm == 14U)
            || (op.kind == InstrKind::mov && op.rd == 15U && op.rm == 14U)
            || sret2;
        out.pcs[out.count] = pc;
        out.sizes[out.count] = size;
        ++out.count;
        if (slot.is_terminator[i]) break;
    }
    if (out.count == 0U) {
        ++jit_stats_.fallbacks;
        return std::nullopt;
    }
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

std::optional<CortexM4::JitBlockPreview> CortexM4::peekJitBlock(
    const std::size_t max_instructions) const noexcept {
    if (max_instructions == 0U || !jitBlockReady()) return std::nullopt;
    const std::uint32_t entry_pc = state_.r[15];
    const auto& slot = jit_blocks_[(entry_pc >> 1U) & (jit_block_entries - 1U)];
    const std::size_t count = std::min<std::size_t>(max_instructions, slot.count);
    if (count == 0U) return std::nullopt;

    JitBlockPreview preview{};
    for (std::size_t i = 0; i < count; ++i) {
        // Preview must not allow speculative MMIO or RAM side effects.
        if (slot.is_memory[i]) return std::nullopt;
        preview.pcs[i] = slot.pcs[i];
        preview.sizes[i] = slot.sizes[i];
        // divideCycles ranges from 2 (zero divisor) through 12 cycles;
        // use its modeled maximum without speculatively reading registers.
        const std::uint64_t base = slot.divide_form[i]
            ? 12U : slot.base_cycles[i];
        std::uint16_t penalty = slot.branch_penalty[i];
        const auto& op = slot.ops[i];
        bool exact = !slot.divide_form[i] && op.condition == Condition::al;
        if (slot.is_terminator[i]) {
            if (exact && (op.kind == InstrKind::b || op.kind == InstrKind::bl)) {
                const auto target = static_cast<std::uint32_t>(
                    static_cast<std::int64_t>(slot.pcs[i]) + 4 + op.branch_offset)
                    & ~std::uint32_t{1};
                if (target == slot.pcs[i] + slot.sizes[i]) penalty = 0U;
            } else {
                // Register/conditional targets can depend on earlier ops.
                exact = false;
            }
        }
        preview.max_cycles += base + penalty;
        preview.cycles_exact = preview.cycles_exact && exact;
    }
    preview.count = static_cast<std::uint8_t>(count);
    return preview;
}

bool CortexM4::peekPredictableCycles(std::uint16_t& cycles_out) const noexcept {
    if (state_.halted) return false;
    const std::uint32_t pc = state_.r[15];
    if (!state_.thumb || (state_.xpsr & xpsr_t) == 0U || (pc & 1U) != 0U) return false;
    const auto& cache = instruction_cache_[(pc >> 1U) & (instruction_cache_entries - 1U)];
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
