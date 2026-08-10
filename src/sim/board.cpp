#include "fil/sim/board.hpp"

#include "fil/cortexm/exceptions.hpp"
#include "fil/cortexm/system_control.hpp"
#include "fil/mem/mcu_map.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include <algorithm>
#include <iomanip>
#include <cstdio>
#include <cstring>
#include <limits>
#include <sstream>
#include <utility>

namespace fil::sim {
namespace {

constexpr std::uint64_t nanoseconds_per_second = 1'000'000'000ULL;

std::string hex32(const std::uint32_t value) {
    std::ostringstream output;
    output << "0x" << std::hex << std::setfill('0') << std::setw(8) << value;
    return output.str();
}

Error runtimeError(std::string message) {
    return Error{ErrorCategory::runtime, std::move(message), std::nullopt};
}

SimTimeNs deadlineAfter(const SimTimeNs start, const SimTimeNs duration) noexcept {
    return duration > std::numeric_limits<SimTimeNs>::max() - start
        ? std::numeric_limits<SimTimeNs>::max() : start + duration;
}

bool sameCpuState(const cpu::CpuState& left, const cpu::CpuState& right) noexcept {
    if (left.r != right.r || left.xpsr != right.xpsr || left.msp != right.msp
        || left.psp != right.psp || left.primask != right.primask
        || left.basepri != right.basepri || left.faultmask != right.faultmask
        || left.control != right.control || left.thumb != right.thumb
        || left.halted != right.halted || left.pending_exception != right.pending_exception
        || left.pending_exc_return != right.pending_exc_return || left.fpscr != right.fpscr
        || left.it_state != right.it_state
        || left.instruction_address != right.instruction_address) {
        return false;
    }
    return std::memcmp(left.s.data(), right.s.data(), sizeof(left.s)) == 0;
}

} // namespace

struct Board::TransactionCheckpoint {
    EventOwner owner{shared_event_owner};
    cpu::CpuState cpu_state;
    cortexm::SystemControl system_state;
    std::vector<std::uint16_t> active_exceptions;
    mem::MemoryBus::SideEffectCheckpoint memory_checkpoint;
    EventLoop::OwnerCheckpoint event_checkpoint;
    std::uint64_t time_fraction{0};
    std::array<LoopObservation, 256> loop_observations{};
    std::uint64_t loop_observation_generation{1U};
    std::optional<std::uint32_t> read_footprint_boundary;
    mem::MemoryBus::ReadFootprint read_footprint;
};

bool BoardRunResult::succeeded() const noexcept {
    return reason == BoardStopReason::target_reached
        || reason == BoardStopReason::breakpoint
        || reason == BoardStopReason::halted
        || reason == BoardStopReason::instruction_budget
        || reason == BoardStopReason::time_budget
        || reason == BoardStopReason::synchronization_required;
}

Board::Board(
    config::BoardConfig config,
    config::McuConfig mcu,
    elf::ElfImage image,
    EventLoop* shared_event_loop,
    TraceRecorder* shared_trace
) : config_(std::move(config)), mcu_config_(std::move(mcu)), image_(std::move(image)) {
    if (shared_event_loop == nullptr) {
        owned_event_loop_ = std::make_unique<EventLoop>();
        event_loop_ = owned_event_loop_.get();
    } else {
        event_loop_ = shared_event_loop;
    }
    if (shared_trace == nullptr) {
        owned_trace_ = std::make_unique<TraceRecorder>();
        trace_ = owned_trace_.get();
    } else {
        trace_ = shared_trace;
    }
}

Board::~Board() = default;

Result<std::unique_ptr<Board>> Board::load(
    const config::BoardConfig& config,
    const bool strict_mmio,
    EventLoop* shared_event_loop,
    TraceRecorder* shared_trace
) {
    auto mcu = config::loadMcuConfig(config.mcu_path);
    if (!mcu) return mcu.error();
    auto image = elf::load(config.elf_path, config.vector_base);
    if (!image) return image.error();
    auto board = std::unique_ptr<Board>(new Board(
        config, std::move(mcu).value(), std::move(image).value(), shared_event_loop, shared_trace
    ));
    auto initialized = board->initialize(strict_mmio);
    if (!initialized) return initialized.error();
    return board;
}

Result<void> Board::initialize(const bool strict_mmio) {
    system_ = std::make_unique<cortexm::SystemControl>(image_.vectorBase());
    auto peripherals = stm32g4::Stm32G4::create(
        *event_loop_, *trace_, *system_, true, mcu_config_.hse_hz
    );
    if (!peripherals) return peripherals.error();
    peripherals_ = std::move(peripherals).value();
    peripherals_->setTraceSourcePrefix(config_.name);
    peripherals_->router().setLenient(!strict_mmio);
    return reset();
}

Result<void> Board::reset() {
    if (!system_ || !peripherals_) return runtimeError("board reset before peripheral initialization");
    if (owned_event_loop_) event_loop_->clear();
    system_->reset(image_.vectorBase());
    peripherals_->reset();
    auto mapped = mem::buildMcuMemoryMap(
        mcu_config_, image_, {peripherals_->mmio(), *system_}
    );
    if (!mapped) return mapped.error();
    memory_ = std::move(mapped).value();
    peripherals_->attachMemory(memory_);
    auto configured = peripherals_->configure(config_);
    if (!configured) return configured.error();
    cpu_ = std::make_unique<cpu::CortexM4>(memory_);
    if (!cpu_->reset(image_)) return runtimeError("CPU rejected validated reset vectors");
    exceptions_ = std::make_unique<cortexm::ExceptionController>(memory_, *system_);
    time_fraction_ = 0;
    loop_observations_ = {};
    loop_observation_generation_ = 1U;
    read_footprint_boundary_.reset();
    static_cast<void>(memory_.takeReadFootprint());
    return {};
}

SimTimeNs Board::accountCycles(const std::uint64_t cycles) {
    system_->advanceCycles(cycles);
    const std::uint64_t frequency = peripherals_->rcc().systemClockHz();
    const std::uint64_t numerator = cycles * nanoseconds_per_second + time_fraction_;
    const SimTimeNs elapsed = frequency == 0 ? 0 : numerator / frequency;
    time_fraction_ = frequency == 0 ? 0 : numerator % frequency;
    return elapsed;
}

SimTimeNs Board::advanceTime(const std::uint64_t cycles) {
    const SimTimeNs elapsed = accountCycles(cycles);
    const auto events = event_loop_->advanceBy(elapsed);
    if (events.same_time_limit_hit) {
        trace_->record(event_loop_->now(), config_.name, "event_livelock");
    }
    return elapsed;
}

Board::ConcurrentStepResult Board::beginConcurrentStep(const bool trace_instructions) {
    auto result = cpu_->stepFast();
    if (trace_instructions) {
        trace_->record(
            event_loop_->now(), config_.name, "instr",
            {
                {"pc", hex32(result.instruction_address)},
                {"raw", hex32(result.raw)},
            }
        );
    }
    return ConcurrentStepResult{result, accountCycles(result.cycles)};
}

bool Board::boundaryWorkPending() const noexcept {
    return cpu_->state().pending_exc_return || cpu_->state().pending_exception
        || system_->hasEnabledPending() || system_->resetRequested()
        || peripherals_->resetRequested();
}

std::optional<Board::BoundaryStop> Board::settleInstructionBoundary() {
    if (!boundaryWorkPending()) [[likely]] return std::nullopt;
    return settleInstructionBoundarySlow();
}

#if defined(__clang__) || defined(__GNUC__)
__attribute__((noinline))
#endif
std::optional<Board::BoundaryStop> Board::settleInstructionBoundarySlow() {
    if (cpu_->state().pending_exc_return) {
        const std::uint32_t exc_return = *cpu_->state().pending_exc_return;
        cpu_->state().pending_exc_return.reset();
        auto returned = exceptions_->exceptionReturn(cpu_->state(), exc_return);
        if (!returned) {
            return BoundaryStop{BoardStopReason::architectural_fault, returned.error().message};
        }
        trace_->record(
            event_loop_->now(), config_.name, "exception_return",
            {{"exc_return", hex32(exc_return)}}
        );
        invalidateLoopObservations();
    }

    if (cpu_->state().pending_exception) {
        const std::uint16_t exception_number = *cpu_->state().pending_exception;
        cpu_->state().pending_exception.reset();
        auto entered = exceptions_->enter(cpu_->state(), exception_number);
        if (!entered) {
            return BoundaryStop{BoardStopReason::architectural_fault, entered.error().message};
        }
        trace_->record(
            event_loop_->now(), config_.name, "exception_enter",
            {{"exception", std::to_string(exception_number)}}
        );
        invalidateLoopObservations();
    }

    if (system_->hasEnabledPending()) {
        auto pending = exceptions_->enterPending(cpu_->state());
        if (!pending) {
            return BoundaryStop{BoardStopReason::architectural_fault, pending.error().message};
        }
        if (pending.value()) {
            trace_->record(
                event_loop_->now(), config_.name, "exception_enter",
                {{"exception", std::to_string(cpu_->state().ipsr())}}
            );
            invalidateLoopObservations();
        }
    }
    if (system_->consumeResetRequest() || peripherals_->consumeResetRequest()) {
        return BoundaryStop{
            BoardStopReason::reset_requested, "firmware or watchdog requested reset"
        };
    }
    return std::nullopt;
}

std::optional<Board::ProvenLoop> Board::observeLoopBoundary(
    const cpu::FastStepResult& step,
    const std::uint64_t logical_instructions,
    const std::uint64_t logical_cycles
) {
    const std::uint32_t boundary_pc = cpu_->state().r[15];
    if (step.reason != cpu::StopReason::step_complete
        || step.suppress_loop_observation
        || boundary_pc > step.instruction_address) {
        return std::nullopt;
    }

    const std::size_t observation_index =
        (boundary_pc >> 1U) % loop_observations_.size();
    LoopObservation& observation = loop_observations_[observation_index];
    const auto read_footprint = memory_.takeReadFootprint();
    const bool read_footprint_complete = read_footprint_boundary_
        && *read_footprint_boundary_ == boundary_pc;
    read_footprint_boundary_ = boundary_pc;
    const auto checkpoint = memory_.sideEffectCheckpoint();
    if (observation.valid
        && observation.generation == loop_observation_generation_
        && observation.boundary_pc == boundary_pc
        && sameCpuState(observation.state, cpu_->state())
        && memory_.sideEffectsRestoredSince(observation.side_effect_checkpoint)
        && logical_instructions > observation.instructions
        && logical_cycles > observation.cycles) {
        ProvenLoop loop{
            boundary_pc,
            logical_instructions - observation.instructions,
            logical_cycles - observation.cycles,
            checkpoint,
            static_cast<std::uint16_t>(observation_index),
            observation.revision,
            read_footprint,
            read_footprint_complete,
        };
        observation.instructions = logical_instructions;
        observation.cycles = logical_cycles;
        observation.side_effect_checkpoint = checkpoint;
        return loop;
    }

    observation.valid = true;
    observation.generation = loop_observation_generation_;
    ++observation.revision;
    if (observation.revision == 0U) observation.revision = 1U;
    observation.boundary_pc = boundary_pc;
    observation.state = cpu_->state();
    observation.side_effect_checkpoint = checkpoint;
    observation.instructions = logical_instructions;
    observation.cycles = logical_cycles;
    return std::nullopt;
}

bool Board::loopProofStillValid(const ProvenLoop& loop) const noexcept {
    if (loop.observation_index >= loop_observations_.size()) return false;
    const LoopObservation& observation = loop_observations_[loop.observation_index];
    return loop.instructions_per_iteration != 0U && loop.cycles_per_iteration != 0U
        && cpu_->state().r[15] == loop.boundary_pc
        && observation.valid
        && observation.generation == loop_observation_generation_
        && observation.revision == loop.observation_revision
        && observation.boundary_pc == loop.boundary_pc
        && sameCpuState(cpu_->state(), observation.state)
        && (loop.read_footprint_complete
            ? memory_.sideEffectsCompatibleSince(
                loop.side_effect_checkpoint, loop.read_footprint
            )
            : memory_.sideEffectsRestoredSince(loop.side_effect_checkpoint));
}

bool Board::loopHasNoMmioSince(const ProvenLoop& loop) const noexcept {
    return memory_.mmioUnchangedSince(loop.side_effect_checkpoint);
}

SimTimeNs Board::elapsedForCycles(const std::uint64_t cycles) const noexcept {
    const std::uint64_t frequency = peripherals_->rcc().systemClockHz();
    if (frequency == 0U || cycles >
            (std::numeric_limits<std::uint64_t>::max() - time_fraction_)
                / nanoseconds_per_second) {
        return std::numeric_limits<SimTimeNs>::max();
    }
    return (cycles * nanoseconds_per_second + time_fraction_) / frequency;
}

std::optional<SimTimeNs> Board::nextObservableTime(const SimTimeNs boundary_time) const {
    if (system_->nextPending(
            cpu_->state().primask, cpu_->state().basepri, cpu_->state().faultmask)) {
        return boundary_time;
    }
    const auto cycles = system_->cyclesUntilSysTickInterrupt();
    if (!cycles) return std::nullopt;
    const SimTimeNs elapsed = elapsedForCycles(*cycles);
    if (elapsed > std::numeric_limits<SimTimeNs>::max() - boundary_time) {
        return std::numeric_limits<SimTimeNs>::max();
    }
    return boundary_time + elapsed;
}

std::uint64_t Board::maximumLoopIterations(
    const ProvenLoop& loop,
    const std::uint64_t instruction_budget,
    const std::optional<SimTimeNs> horizon_ns
) const {
    if (!loopProofStillValid(loop)) return 0U;
    if (system_->nextPending(
            cpu_->state().primask, cpu_->state().basepri, cpu_->state().faultmask)) {
        return 0U;
    }

    std::uint64_t iterations = instruction_budget / loop.instructions_per_iteration;
    if (iterations == 0U) return 0U;
    const std::uint64_t maximum_accountable_cycles =
        (std::numeric_limits<std::uint64_t>::max() - time_fraction_)
        / nanoseconds_per_second;
    iterations = std::min(
        iterations, maximum_accountable_cycles / loop.cycles_per_iteration
    );
    if (iterations == 0U) return 0U;
    if (const auto systick_cycles = system_->cyclesUntilSysTickInterrupt()) {
        iterations = std::min(iterations, *systick_cycles / loop.cycles_per_iteration);
    }
    if (iterations == 0U) return 0U;

    if (horizon_ns) {
        const SimTimeNs now = event_loop_->now();
        if (*horizon_ns <= now) return 0U;
        const SimTimeNs available_ns = *horizon_ns - now;
        const std::uint64_t frequency = peripherals_->rcc().systemClockHz();
        if (frequency == 0U) return 0U;

        // floor((cycles * 1e9 + fraction) / frequency) <= available
        // exactly when the numerator is below (available + 1) * frequency.
        // Solve that inequality directly instead of binary-searching iteration
        // counts. If the right side cannot fit in uint64_t, the earlier
        // accountable-cycle bound is already stricter than this horizon.
        if (available_ns != std::numeric_limits<SimTimeNs>::max()
            && available_ns + 1U
                <= std::numeric_limits<std::uint64_t>::max() / frequency) {
            const std::uint64_t exclusive_numerator =
                (available_ns + 1U) * frequency;
            if (exclusive_numerator <= time_fraction_) return 0U;
            const std::uint64_t maximum_cycles =
                (exclusive_numerator - 1U - time_fraction_)
                / nanoseconds_per_second;
            iterations = std::min(
                iterations, maximum_cycles / loop.cycles_per_iteration
            );
        }
    }
    return iterations;
}

Board::LoopSkip Board::applyLoopIterations(
    const ProvenLoop& loop,
    const std::uint64_t validated_iterations
) {
    if (validated_iterations == 0U) return {};
    LoopSkip skip;
    skip.instructions = validated_iterations * loop.instructions_per_iteration;
    skip.cycles = validated_iterations * loop.cycles_per_iteration;
    skip.elapsed_ns = accountCycles(skip.cycles);
    return skip;
}

void Board::invalidateLoopObservations() noexcept {
    ++loop_observation_generation_;
    read_footprint_boundary_.reset();
    static_cast<void>(memory_.takeReadFootprint());
    if (loop_observation_generation_ == 0U) {
        loop_observations_ = {};
        loop_observation_generation_ = 1U;
    }
}

void Board::refreshLoopObservation(
    const ProvenLoop& loop,
    const std::uint64_t logical_instructions,
    const std::uint64_t logical_cycles
) {
    LoopObservation& observation =
        loop_observations_[(loop.boundary_pc >> 1U) % loop_observations_.size()];
    observation.valid = true;
    observation.generation = loop_observation_generation_;
    observation.boundary_pc = loop.boundary_pc;
    observation.state = cpu_->state();
    observation.side_effect_checkpoint = memory_.sideEffectCheckpoint();
    observation.instructions = logical_instructions;
    observation.cycles = logical_cycles;
}

BoardRunResult Board::cpuFailure(const cpu::RunResult& result) const {
    BoardRunResult board;
    board.cycles = result.cycles;
    board.instructions = result.instructions;
    board.time_ns = event_loop_->now();
    board.diagnostic = result.diagnostic;
    board.message = result.diagnostic.message;
    if (const elf::ElfSymbol* symbol = image_.symbolAtOrBefore(result.diagnostic.instruction_address)) {
        const std::uint32_t offset = result.diagnostic.instruction_address - symbol->address;
        board.message += " symbol=" + symbol->name + "+" + hex32(offset);
    }
    switch (result.reason) {
    case cpu::StopReason::breakpoint: board.reason = BoardStopReason::breakpoint; break;
    case cpu::StopReason::halted: board.reason = BoardStopReason::halted; break;
    case cpu::StopReason::undefined_instruction: board.reason = BoardStopReason::unimplemented_instruction; break;
    case cpu::StopReason::bus_fault:
    case cpu::StopReason::invalid_state: board.reason = BoardStopReason::architectural_fault; break;
    case cpu::StopReason::synchronization_required:
        board.reason = BoardStopReason::synchronization_required;
        break;
    case cpu::StopReason::instruction_budget: board.reason = BoardStopReason::instruction_budget; break;
    case cpu::StopReason::step_complete: board.reason = BoardStopReason::host_error; break;
    }
    return board;
}

Board::TransactionCheckpointPtr Board::captureTransaction(
    const EventOwner owner
) const {
    auto checkpoint = std::make_shared<TransactionCheckpoint>();
    checkpoint->owner = owner;
    checkpoint->cpu_state = cpu_->state();
    checkpoint->system_state = *system_;
    checkpoint->active_exceptions = exceptions_->activeStack();
    checkpoint->memory_checkpoint = memory_.sideEffectCheckpoint();
    checkpoint->event_checkpoint = event_loop_->ownerCheckpoint(owner);
    checkpoint->time_fraction = time_fraction_;
    checkpoint->loop_observations = loop_observations_;
    checkpoint->loop_observation_generation = loop_observation_generation_;
    checkpoint->read_footprint_boundary = read_footprint_boundary_;
    checkpoint->read_footprint = memory_.readFootprint();
    return checkpoint;
}

bool Board::restoreTransaction(const TransactionCheckpointPtr& checkpoint) {
    if (!checkpoint || !memory_.canRestoreSideEffects(checkpoint->memory_checkpoint)
        || !event_loop_->canRestoreOwnerCheckpoint(checkpoint->event_checkpoint)) {
        return false;
    }
    if (!memory_.restoreSideEffects(checkpoint->memory_checkpoint)
        || !event_loop_->restoreOwnerCheckpoint(checkpoint->event_checkpoint)) {
        return false;
    }
    cpu_->state() = checkpoint->cpu_state;
    *system_ = checkpoint->system_state;
    exceptions_->restoreActiveStack(checkpoint->active_exceptions);
    time_fraction_ = checkpoint->time_fraction;
    loop_observations_ = checkpoint->loop_observations;
    loop_observation_generation_ = checkpoint->loop_observation_generation;
    read_footprint_boundary_ = checkpoint->read_footprint_boundary;
    memory_.restoreReadFootprint(checkpoint->read_footprint);
    return true;
}

BoardRunResult Board::runWorkerSlice(
    const EventOwner owner,
    const std::uint64_t instruction_budget,
    const SimTimeNs deadline_ns,
    const bool enable_loop_batching,
    const bool trap_all_mmio
) {
    BoardRunResult aggregate;
    aggregate.reason = BoardStopReason::instruction_budget;
    auto owner_scope = event_loop_->useOwner(owner);
    const bool previous_shared_trapping = memory_.sharedMmioTrapping();
    const bool previous_all_trapping = memory_.allMmioTrapping();
    memory_.setSharedMmioTrapping(true);
    memory_.setAllMmioTrapping(trap_all_mmio);
    SimTimeNs local_now = event_loop_->now(owner);

    const auto restore_trapping = [&]() {
        memory_.setSharedMmioTrapping(previous_shared_trapping);
        memory_.setAllMmioTrapping(previous_all_trapping);
    };
    const auto finish = [&]() {
        if (trap_all_mmio) {
            static_cast<void>(event_loop_->runOwnedEvents(owner, local_now));
        }
        restore_trapping();
        aggregate.time_ns = trap_all_mmio ? local_now : event_loop_->now(owner);
        aggregate.diagnostic.next_pc = cpu_->state().r[15];
        aggregate.diagnostic.registers = cpu_->state().r;
        aggregate.diagnostic.xpsr = cpu_->state().xpsr;
        return aggregate;
    };

    while (aggregate.instructions < instruction_budget) {
        const SimTimeNs now = trap_all_mmio ? local_now : event_loop_->now(owner);
        if (now >= deadline_ns) {
            aggregate.reason = BoardStopReason::time_budget;
            aggregate.message = "worker slice deadline reached";
            break;
        }
        if (!trap_all_mmio) {
            const auto due = event_loop_->runOwnedEvents(owner, now);
            if (due.same_time_limit_hit) {
                aggregate.reason = BoardStopReason::host_error;
                aggregate.message = "owner-local event livelock";
                break;
            }
        }
        if (auto boundary = settleInstructionBoundary()) {
            aggregate.reason = boundary->reason;
            aggregate.message = std::move(boundary->message);
            break;
        }
        const SimTimeNs next_elapsed = elapsedForCycles(1U);
        if (next_elapsed > deadline_ns - now) {
            if (trap_all_mmio) local_now = deadline_ns;
            else static_cast<void>(event_loop_->runOwnedEvents(owner, deadline_ns));
            aggregate.reason = BoardStopReason::time_budget;
            aggregate.message = "worker slice deadline reached";
            break;
        }

        const cpu::FastStepResult result = cpu_->stepFast();
        aggregate.diagnostic.instruction_address = result.instruction_address;
        aggregate.diagnostic.raw = result.raw;
        aggregate.diagnostic.instruction_size = result.instruction_size;
        if (result.reason == cpu::StopReason::synchronization_required) {
            aggregate.reason = BoardStopReason::synchronization_required;
            aggregate.message = "shared MMIO requires coordinator commit";
            aggregate.diagnostic = cpu_->lastDiagnostic();
            break;
        }

        aggregate.instructions += result.instructions;
        aggregate.cycles += result.cycles;
        const SimTimeNs elapsed = accountCycles(result.cycles);
        const SimTimeNs completion = deadlineAfter(
            trap_all_mmio ? local_now : event_loop_->now(owner), elapsed
        );
        if (trap_all_mmio) local_now = completion;
        else {
            const auto events = event_loop_->runOwnedEvents(owner, completion);
            if (events.same_time_limit_hit) {
                aggregate.reason = BoardStopReason::host_error;
                aggregate.message = "owner-local event livelock";
                break;
            }
        }
        if (result.reason != cpu::StopReason::step_complete) {
            cpu::RunResult detailed;
            detailed.reason = result.reason;
            detailed.instructions = result.instructions;
            detailed.cycles = result.cycles;
            detailed.diagnostic = cpu_->lastDiagnostic();
            BoardRunResult stopped = cpuFailure(detailed);
            stopped.instructions = aggregate.instructions;
            stopped.cycles = aggregate.cycles;
            restore_trapping();
            return stopped;
        }
        if (auto boundary = settleInstructionBoundary()) {
            aggregate.reason = boundary->reason;
            aggregate.message = std::move(boundary->message);
            break;
        }

        const auto loop = observeLoopBoundary(
            result, aggregate.instructions, aggregate.cycles
        );
        if (!enable_loop_batching || !loop) continue;
        std::optional<SimTimeNs> horizon = deadline_ns;
        if (const auto local_event = event_loop_->nextScheduledTime(owner);
            local_event && *local_event < *horizon) {
            horizon = *local_event;
        }
        const std::uint64_t remaining = instruction_budget - aggregate.instructions;
        const std::uint64_t iterations = maximumLoopIterations(*loop, remaining, horizon);
        if (iterations == 0U) continue;
        const LoopSkip skip = applyLoopIterations(*loop, iterations);
        aggregate.instructions += skip.instructions;
        aggregate.cycles += skip.cycles;
        const SimTimeNs skipped_completion = deadlineAfter(
            trap_all_mmio ? local_now : event_loop_->now(owner), skip.elapsed_ns
        );
        if (trap_all_mmio) local_now = skipped_completion;
        else {
            const auto skipped_events = event_loop_->runOwnedEvents(
                owner, skipped_completion
            );
            if (skipped_events.same_time_limit_hit) {
                aggregate.reason = BoardStopReason::host_error;
                aggregate.message = "owner-local event livelock";
                break;
            }
        }
        refreshLoopObservation(*loop, aggregate.instructions, aggregate.cycles);
        if (auto boundary = settleInstructionBoundary()) {
            aggregate.reason = boundary->reason;
            aggregate.message = std::move(boundary->message);
            break;
        }
    }

    return finish();
}

BoardRunResult Board::run(const BoardRunOptions& options) {
    BoardRunResult aggregate;
    aggregate.reason = BoardStopReason::instruction_budget;
    std::optional<std::uint32_t> proven_spin_pc;
    std::uint64_t proven_spin_instructions = 0U;
    const SimTimeNs deadline = options.duration_ns == 0U
        ? 0U : deadlineAfter(event_loop_->now(), options.duration_ns);

    while (aggregate.instructions < options.max_instructions) {
        const std::uint32_t pc = cpu_->state().r[15];
        if (options.stop_address && pc == (*options.stop_address & ~1U)) {
            aggregate.reason = BoardStopReason::target_reached;
            aggregate.message = "target address reached at " + hex32(pc);
            break;
        }
        if (deadline != 0U && event_loop_->now() >= deadline) {
            aggregate.reason = BoardStopReason::time_budget;
            aggregate.message = "simulated-time budget exhausted";
            break;
        }
        auto result = cpu_->stepFast();
        aggregate.instructions += result.instructions;
        aggregate.cycles += result.cycles;
        aggregate.diagnostic.instruction_address = result.instruction_address;
        aggregate.diagnostic.raw = result.raw;
        aggregate.diagnostic.instruction_size = result.instruction_size;
        if (options.trace_instructions) {
            trace_->record(
                event_loop_->now(), config_.name, "instr",
                {{"pc", hex32(result.instruction_address)}, {"raw", hex32(result.raw)}}
            );
        }
        static_cast<void>(advanceTime(result.cycles));
        if (result.reason != cpu::StopReason::step_complete) {
            cpu::RunResult detailed;
            detailed.reason = result.reason;
            detailed.instructions = result.instructions;
            detailed.cycles = result.cycles;
            detailed.diagnostic = cpu_->lastDiagnostic();
            BoardRunResult failure = cpuFailure(detailed);
            failure.instructions = aggregate.instructions;
            failure.cycles = aggregate.cycles;
            return failure;
        }

        if (auto boundary = settleInstructionBoundary()) {
            aggregate.reason = boundary->reason;
            aggregate.message = std::move(boundary->message);
            break;
        }

        const auto loop = observeLoopBoundary(result, aggregate.instructions, aggregate.cycles);
        if (!loop) continue;
        if (proven_spin_pc && *proven_spin_pc == loop->boundary_pc) {
            proven_spin_instructions += loop->instructions_per_iteration;
        } else {
            proven_spin_pc = loop->boundary_pc;
            proven_spin_instructions = loop->instructions_per_iteration;
        }
        if (options.detect_spin && proven_spin_instructions >= options.spin_threshold) {
            aggregate.reason = BoardStopReason::spin_detected;
            aggregate.message = "exact-state loop at " + hex32(loop->boundary_pc)
                + " period_instructions=" + std::to_string(loop->instructions_per_iteration)
                + " period_cycles=" + std::to_string(loop->cycles_per_iteration);
            trace_->record(
                event_loop_->now(), config_.name, "spin_detected",
                {
                    {"pc", hex32(loop->boundary_pc)},
                    {"period_instructions", std::to_string(loop->instructions_per_iteration)},
                    {"period_cycles", std::to_string(loop->cycles_per_iteration)},
                }
            );
            break;
        }
        if (!options.enable_loop_batching || options.trace_instructions
            || options.detect_spin) {
            continue;
        }

        std::optional<SimTimeNs> horizon;
        if (deadline != 0U) horizon = deadline;
        if (const auto event_time = event_loop_->nextScheduledTime()) {
            if (!horizon || *event_time < *horizon) horizon = *event_time;
        }
        const std::uint64_t remaining = options.max_instructions - aggregate.instructions;
        const std::uint64_t iterations = maximumLoopIterations(*loop, remaining, horizon);
        if (iterations == 0U) continue;
        const LoopSkip skip = applyLoopIterations(*loop, iterations);
        aggregate.instructions += skip.instructions;
        aggregate.cycles += skip.cycles;
        const auto events = event_loop_->advanceBy(skip.elapsed_ns);
        if (events.same_time_limit_hit) {
            trace_->record(event_loop_->now(), config_.name, "event_livelock");
        }
        refreshLoopObservation(*loop, aggregate.instructions, aggregate.cycles);
        if (auto boundary = settleInstructionBoundary()) {
            aggregate.reason = boundary->reason;
            aggregate.message = std::move(boundary->message);
            break;
        }
    }

    aggregate.time_ns = event_loop_->now();
    aggregate.diagnostic.next_pc = cpu_->state().r[15];
    aggregate.diagnostic.registers = cpu_->state().r;
    aggregate.diagnostic.xpsr = cpu_->state().xpsr;
    if (aggregate.instructions == options.max_instructions && aggregate.message.empty()) {
        aggregate.reason = BoardStopReason::instruction_budget;
        aggregate.message = "instruction budget exhausted";
    }
    return aggregate;
}

std::string_view boardStopReasonName(const BoardStopReason reason) noexcept {
    switch (reason) {
    case BoardStopReason::target_reached: return "target-reached";
    case BoardStopReason::breakpoint: return "breakpoint";
    case BoardStopReason::halted: return "halted";
    case BoardStopReason::instruction_budget: return "instruction-budget";
    case BoardStopReason::time_budget: return "time-budget";
    case BoardStopReason::spin_detected: return "spin-detected";
    case BoardStopReason::unimplemented_instruction: return "unimplemented-instruction";
    case BoardStopReason::architectural_fault: return "architectural-fault";
    case BoardStopReason::reset_requested: return "reset-requested";
    case BoardStopReason::synchronization_required: return "synchronization-required";
    case BoardStopReason::host_error: return "host-error";
    }
    return "unknown";
}

} // namespace fil::sim
