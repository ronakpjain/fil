#include "fil/cpu/cortex_m4.hpp"

#include "fil/cpu/decoder.hpp"
#include "fil/elf/elf_loader.hpp"
#include "fil/mem/memory_bus.hpp"

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

} // namespace

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
        || left.halted != right.halted
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

CortexM4::CortexM4(mem::MemoryBus& memory) noexcept : memory_(memory) {
    assert(decoderTablesHaveNoOverlaps());
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
        cache.generation = execution_generation;
        cache.pc = pc;
        cache.raw = result.raw;
        cache.decoded = *newly_decoded;
        cache.size = instruction_size;
        // Memoize the static pipeline cost once per decode; divide forms
        // resolve data-dependently on the hot path via divide_form.
        cache.divide_form = newly_decoded->kind == InstrKind::udiv
            || newly_decoded->kind == InstrKind::sdiv;
        cache.base_cycles = cache.divide_form
            ? 7U : basePipelineCycles(*newly_decoded);
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
        stop = execute(*decoded, last_diagnostic_);
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
