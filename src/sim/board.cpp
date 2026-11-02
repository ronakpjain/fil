#include "fil/sim/board.hpp"

#include "fil/common/format.hpp"
#include "fil/common/numeric.hpp"
#include "fil/cpu/decoder.hpp"
#include "fil/cortexm/exceptions.hpp"
#include "fil/cortexm/system_control.hpp"
#include "fil/mem/mcu_map.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>

namespace fil::sim {
namespace {

constexpr std::uint64_t nanoseconds_per_second = 1'000'000'000ULL;

SimTimeNs cycleTimeQuotient(const std::uint64_t numerator, const std::uint64_t frequency) noexcept {
    // Constant divisors lower to multiply/shift arithmetic. Other RCC clock
    // configurations retain exact integer division, without retained results.
    switch (frequency) {
    case 16'000'000U: return numerator / 16'000'000U;
    case 170'000'000U: return numerator / 170'000'000U;
    default: return numerator / frequency;
    }
}


struct ReversibleMemoryGuard {
    mem::MemoryBus& memory;
    bool all_trapping;
    bool ram_only;
    explicit ReversibleMemoryGuard(mem::MemoryBus& bus)
        : memory(bus), all_trapping(bus.allMmioTrapping()), ram_only(bus.reversibleRamOnly()) {
        memory.setAllMmioTrapping(true);
        memory.setReversibleRamOnly(true);
    }
    ~ReversibleMemoryGuard() {
        memory.setAllMmioTrapping(all_trapping);
        memory.setReversibleRamOnly(ram_only);
    }
};

Error runtimeError(std::string message) {
    return Error{ErrorCategory::runtime, std::move(message), std::nullopt};
}

} // namespace

bool Board::appendReversibleChunk(
    ReversibleRamPrefix::ReversibleExecution& aggregate,
    const cpu::CortexM4::TimedJitStepOutcome& chunk) {
    const auto count = chunk.execution.count;
    if (count == 0U || count > cpu::CortexM4::JitStepOutcome::max_block
        || static_cast<std::size_t>(aggregate.count) + count
            > ReversibleRamPrefix::max_instructions) return false;
    if (aggregate.count == 0U) aggregate.result = chunk.execution.result;
    else {
        aggregate.result.instruction_address = chunk.execution.result.instruction_address;
        aggregate.result.raw = chunk.execution.result.raw;
        aggregate.result.instruction_size = chunk.execution.result.instruction_size;
        aggregate.result.suppress_loop_observation =
            chunk.execution.result.suppress_loop_observation;
    }
    for (std::uint8_t i = 0U; i < count; ++i) {
        const std::size_t destination = aggregate.count++;
        aggregate.pcs[destination] = chunk.execution.pcs[i];
        aggregate.sizes[destination] = chunk.execution.sizes[i];
        aggregate.instruction_cycles[destination] = chunk.instruction_cycles[i];
    }
    aggregate.result.instructions = aggregate.count;
    aggregate.result.reason = cpu::StopReason::step_complete;
    return true;
}

struct Board::IdempotentPeriodCertificate {
    static constexpr std::size_t max_instructions = 32U;
    // 2048 buckets cover every phase of a 32-instruction period through its
    // low-bit-folded PC bucket without changing certificate semantics.
    static constexpr std::size_t pc_index_entries = 2048U;
    [[nodiscard]] static constexpr std::size_t pcBucket(const std::uint32_t pc) noexcept {
        return ((pc >> 1U) ^ (pc >> 12U)) & (pc_index_entries - 1U);
    }
    std::uint8_t count{0U};
    std::array<cpu::CpuState, max_instructions + 1U> states{};
    std::array<std::uint32_t, max_instructions> pcs{};
    std::array<std::uint8_t, max_instructions> sizes{};
    std::array<std::uint32_t, max_instructions> raw{};
    std::array<std::uint16_t, max_instructions> instruction_cycles{};
    std::array<std::uint64_t, max_instructions + 1U> cumulative_cycles{};
    std::array<mem::MemoryBus::ReadFootprint, max_instructions + 1U> footprints{};
    std::bitset<max_instructions + 1U> have_fetch{};
    std::array<std::uint32_t, max_instructions + 1U> fetch_end{};
    // Direct pc -> phase index so the admission hot path avoids scanning every
    // phase with a full CpuState compare. PC collisions still fall back to the
    // exact per-candidate compare, so the certificate stays conservative.
    std::array<std::uint32_t, pc_index_entries> pc_index{};
    std::array<std::uint8_t, pc_index_entries> pc_index_count{};
    mem::MemoryBus::ReadFootprint full_footprint{};
    mem::MemoryBus::SideEffectCheckpoint memory_checkpoint{};
    std::uint64_t restoration_generation{0U};
    std::uint64_t execution_generation{0U};
    std::uint64_t clock_hz{0U};
    std::uint64_t flash_acr_generation{0U};
};

struct Board::WholeRamNeutralPeriodCertificate {
    static constexpr std::size_t max_instructions = 32U;
    std::uint8_t count{0U};
    cpu::CpuState entry_state{};
    bool entry_have_fetch{false};
    std::uint32_t entry_fetch_end{0U};
    std::array<std::uint32_t, max_instructions> pcs{};
    std::array<std::uint8_t, max_instructions> sizes{};
    std::array<std::uint32_t, max_instructions> raw{};
    // Total charged timing includes fetch stalls; retain pipeline-only cost too
    // because reversible replay compares it before reapplying live fetch timing.
    std::array<std::uint16_t, max_instructions> cpu_instruction_cycles{};
    std::array<std::uint16_t, max_instructions> instruction_cycles{};
    std::array<std::uint64_t, max_instructions + 1U> cumulative_cycles{};
    mem::MemoryBus::ReadFootprint full_footprint{};
    mem::MemoryBus::SideEffectCheckpoint checkpoint{};
    std::uint64_t restoration_generation{0U};
    std::uint64_t execution_generation{0U};
    std::uint64_t clock_hz{0U};
    std::uint64_t flash_generation{0U};
};

struct Board::RamNeutralTransitionCertificate {
    static constexpr std::size_t max_instructions = ReversibleRamPrefix::max_instructions;
    std::uint8_t count{0U};
    ReversibleRamPrefix::IntegerSnapshot entry{};
    bool entry_have_fetch{false};
    std::uint32_t entry_fetch_end{0U};
    std::array<ReversibleRamPrefix::IntegerSnapshot, max_instructions + 1U> states{};
    std::array<std::uint32_t, max_instructions> pcs{};
    std::array<std::uint8_t, max_instructions> sizes{};
    std::array<std::uint32_t, max_instructions> raw{};
    std::array<std::uint16_t, max_instructions> instruction_cycles{};
    std::array<std::uint64_t, max_instructions + 1U> cumulative_cycles{};
    std::array<mem::MemoryBus::ReadFootprint, max_instructions + 1U> footprints{};
    std::bitset<max_instructions + 1U> have_fetch{};
    std::array<std::uint32_t, max_instructions + 1U> fetch_end{};
    mem::MemoryBus::ReadFootprint full_footprint{};
    mem::MemoryBus::SideEffectCheckpoint checkpoint{};
    std::uint64_t restoration_generation{0U};
    std::uint64_t execution_generation{0U};
    std::uint64_t clock_hz{0U};
    std::uint64_t flash_generation{0U};
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
    native_programs_ = {};
    native_program_count_ = 0U;
    program_compiler_.reset();
    program_execution_generation_ = 0U;
    memory_ = std::move(mapped).value();
    peripherals_->attachMemory(memory_);
    auto configured = peripherals_->configure(config_);
    if (!configured) return configured.error();
    cpu_ = std::make_unique<cpu::CortexM4>(memory_);
    cpu_->setSystemControl(system_.get());
    if (!cpu_->reset(image_)) return runtimeError("CPU rejected validated reset vectors");
    exceptions_ = std::make_unique<cortexm::ExceptionController>(memory_, *system_);
    time_fraction_ = 0;
    loop_observations_ = {};
    loop_observation_generation_ = 1U;
    idempotent_period_cache_.reset();
    whole_ram_neutral_period_cache_.reset();
    whole_ram_neutral_validation_checkpoint_ = {};
    ram_neutral_transition_cache_ = {};
    transition_validation_checkpoints_ = {};
    transition_restoration_generations_.fill(memory_.restorationGeneration());
    transition_hotness_.fill(0U);
    transition_probes_ = {};
    period_validation_checkpoint_ = {};
    period_validation_restoration_generation_ = memory_.restorationGeneration();
    read_footprint_boundary_.reset();
    static_cast<void>(memory_.takeReadFootprint());
    have_last_fetch_ = false;
    last_fetch_end_ = 0U;
    return {};
}

SimTimeNs Board::accountCycles(const std::uint64_t cycles) {
    system_->advanceCycles(cycles);
    const std::uint64_t frequency = peripherals_->rcc().systemClockHz();
    if (frequency == 0U) {
        time_fraction_ = 0U;
        return 0U;
    }
    const std::uint64_t numerator = cycles * nanoseconds_per_second + time_fraction_;
    const SimTimeNs elapsed = cycleTimeQuotient(numerator, frequency);
    time_fraction_ = numerator - elapsed * frequency;
    return elapsed;
}

cpu::FastStepResult Board::stepWithFetchTiming(
    const bool allow_jit_block, const std::optional<SimTimeNs> deadline,
    const std::size_t max_instructions, const bool block_prevalidated) {
    // Keep the exact single-instruction scheduler path out of the block
    // preview's large metadata frame.
    if (max_instructions == 1U) return stepSingleWithFetchTiming(allow_jit_block);
    return stepBlockWithFetchTiming(
        allow_jit_block, deadline, max_instructions, block_prevalidated);
}

cpu::FastStepResult Board::stepSingleWithFetchTiming(const bool allow_jit) {
    auto result = allow_jit ? cpu_->stepJitFast() : cpu_->stepFast();
    if (result.instructions == 0U || result.instruction_size == 0U) return result;
    const auto& flash = peripherals_->flash();
    const bool sequential = have_last_fetch_
        && result.instruction_address == last_fetch_end_;
    last_fetch_end_ = result.instruction_address + result.instruction_size;
    have_last_fetch_ = true;
    const std::uint32_t stall = flash.fetchStallCycles(
        result.instruction_address, sequential);
    result.cycles = static_cast<std::uint16_t>(result.cycles + stall);
    return result;
}

cpu::FastStepResult Board::stepBlockWithFetchTiming(
    const bool allow_jit_block, const std::optional<SimTimeNs> deadline,
    const std::size_t max_instructions, const bool block_prevalidated) {
    std::size_t jit_limit = max_instructions;
    if (allow_jit_block && jit_limit > 1U && !block_prevalidated && !blockBoundaryPending()) {
        // Do not execute memory/MMIO ahead of board time. Only a pure integer
        // block with a conservative completion before every boundary may batch.
        cpu::CortexM4::JitBlockPreview preview{};
        const bool have_preview = cpu_->prepareAndPeekJitBlock(jit_limit, preview);
        if (!have_preview) {
            jit_limit = 1U;
        } else {
            jit_limit = std::min(jit_limit, static_cast<std::size_t>(preview.count));
            const auto& flash = peripherals_->flash();
            std::uint64_t cycles = preview.max_cycles;
            // Zero wait states add no fetch stalls for any address; a single
            // register load replaces one fetchStallCycles call per preview op.
            if (flash.waitStates() != 0U) {
                bool have_fetch = have_last_fetch_;
                std::uint32_t fetch_end = last_fetch_end_;
                for (std::uint8_t i = 0U; i < preview.count; ++i) {
                    const bool sequential = have_fetch && preview.pcs[i] == fetch_end;
                    cycles += flash.fetchStallCycles(preview.pcs[i], sequential);
                    fetch_end = preview.pcs[i] + preview.sizes[i];
                    have_fetch = true;
                }
            }
            const SimTimeNs now = event_loop_->now();
            const SimTimeNs elapsed = elapsedForCycles(cycles);
            auto horizon = deadline;
            if (const auto event = event_loop_->nextScheduledTime();
                event && (!horizon || *event < *horizon)) horizon = event;
            const auto systick = system_->cyclesUntilSysTickInterrupt();
            if ((horizon && (*horizon <= now || elapsed >= *horizon - now))
                || (systick && cycles >= *systick)) {
                jit_limit = 1U;
            }
        }
    }
    if (allow_jit_block && jit_limit > 1U && !blockBoundaryPending()) {
        // The preview above prepared this entry; skip re-probing it.
        if (auto jit = cpu_->tryStepPreparedJitBlock(jit_limit)) {
            // Fold ART flash stalls per instruction in the block.
            auto& flash = peripherals_->flash();
            std::uint32_t extra = 0U;
            if (flash.waitStates() == 0U) {
                const auto last = static_cast<std::uint8_t>(jit->count - 1U);
                last_fetch_end_ = jit->pcs[last] + jit->sizes[last];
                have_last_fetch_ = true;
            } else {
                for (std::uint8_t i = 0U; i < jit->count; ++i) {
                    const bool sequential = have_last_fetch_ && jit->pcs[i] == last_fetch_end_;
                    last_fetch_end_ = jit->pcs[i] + jit->sizes[i];
                    have_last_fetch_ = true;
                    extra += flash.fetchStallCycles(jit->pcs[i], sequential);
                }
            }
            cpu::FastStepResult result = jit->result;
            result.cycles = static_cast<std::uint16_t>(result.cycles + extra);
            return result;
        }
    }
    return stepSingleWithFetchTiming(allow_jit_block);
}

std::optional<Board::PredictedCost> Board::peekPredictedCost() const noexcept {
    const std::uint64_t frequency = peripherals_->rcc().systemClockHz();
    if (frequency == 0U) return std::nullopt;
    std::uint16_t pipeline = 0U;
    if (!cpu_->peekPredictableCycles(pipeline)) return std::nullopt;
    const auto& flash = peripherals_->flash();
    const std::uint32_t pc = cpu_->state().r[15];
    const bool sequential = have_last_fetch_ && pc == last_fetch_end_;
    PredictedCost cost;
    cost.cycles = static_cast<std::uint64_t>(pipeline)
        + flash.fetchStallCycles(pc, sequential);
    cost.frequency = frequency;
    cost.fraction = time_fraction_;
    return cost;
}

Board::PredictedBlockCosts Board::peekPredictedBlockCosts(
    const std::size_t max_instructions, const std::optional<SimTimeNs> deadline) {
    PredictedBlockCosts costs{};
    if (max_instructions == 0U || blockBoundaryPending()) return costs;
    const auto append_exact_single = [&] {
        const auto predicted = peekPredictedCost();
        if (!predicted || predicted->frequency == 0U) return;
        const auto systick = system_->cyclesUntilSysTickInterrupt();
        if (systick && predicted->cycles >= *systick) return;
        auto horizon = deadline;
        if (const auto event = event_loop_->nextScheduledTime();
            event && (!horizon || *event < *horizon)) horizon = event;
        const SimTimeNs now = event_loop_->now();
        if (horizon && (*horizon <= now
            || elapsedForCycles(predicted->cycles) >= *horizon - now)) return;
        costs.prefixes[0] = PredictedBlockCost{*predicted, 1U};
        costs.count = 1U;
    };
    // A ready block can be inspected without preparing/copying the exact
    // preview. If it has no exact pure prefix, skip that more expensive path
    // and retain the same exact single-instruction fallback below. Cold blocks
    // still pass through preparation so ordinary JIT warmup is preserved.
    if (cpu_->jitBlockReady()) {
        const auto ready_preview = cpu_->peekJitBlock(max_instructions);
        if (ready_preview && ready_preview->exact_cycle_prefix_count == 0U) {
            append_exact_single();
            return costs;
        }
    }
    cpu::CortexM4::JitBlockPreview preview_fill{};
    if (!cpu_->prepareAndPeekExactJitBlock(max_instructions, preview_fill)
        || preview_fill.exact_cycle_prefix_count == 0U) {
        append_exact_single();
        return costs;
    }
    const auto preview = std::optional<cpu::CortexM4::JitBlockPreview>{preview_fill};

    const auto& flash = peripherals_->flash();
    const std::uint64_t frequency = peripherals_->rcc().systemClockHz();
    if (frequency == 0U) return costs;
    bool have_fetch = have_last_fetch_;
    std::uint32_t fetch_end = last_fetch_end_;
    std::uint64_t cycles = 0U;
    auto horizon = deadline;
    if (const auto event = event_loop_->nextScheduledTime();
        event && (!horizon || *event < *horizon)) horizon = event;
    const SimTimeNs now = event_loop_->now();
    if (horizon && *horizon <= now) return costs;
    const auto systick = system_->cyclesUntilSysTickInterrupt();
    for (std::uint8_t i = 0U; i < preview->exact_cycle_prefix_count; ++i) {
        cycles += preview->instruction_cycles[i];
        cycles += flash.fetchStallCycles(
            preview->pcs[i], have_fetch && preview->pcs[i] == fetch_end);
        fetch_end = preview->pcs[i] + preview->sizes[i];
        have_fetch = true;
        if (systick && cycles >= *systick) break;

        PredictedBlockCost& prefix = costs.prefixes[costs.count++];
        prefix.instructions = static_cast<std::uint8_t>(i + 1U);
        prefix.cost = PredictedCost{cycles, frequency, time_fraction_};
    }
    // Prefix completion times are monotone. When a time horizon is present,
    // validate from the longest candidate backward instead of dividing once
    // for every prefix on the common no-boundary path.
    if (horizon) {
        const SimTimeNs available_ns = *horizon - now;
        while (costs.count != 0U
            && elapsedForCycles(costs.prefixes[costs.count - 1U].cost.cycles)
                >= available_ns) {
            --costs.count;
        }
    }
    if (costs.count == 0U) append_exact_single();
    return costs;
}

std::optional<Board::DeferredPurePrefix> Board::prepareDeferredPurePrefix(
    const std::size_t max_instructions, const std::optional<SimTimeNs> deadline) {
    if (max_instructions < 2U || blockBoundaryPending()) return std::nullopt;
    cpu::CortexM4::JitBlockPreview preview{};
    if (!cpu_->prepareAndPeekExactJitBlock(
            std::min(max_instructions, cpu::CortexM4::JitStepOutcome::max_block), preview)
        || preview.exact_cycle_prefix_count < 2U) return std::nullopt;

    const std::uint64_t frequency = peripherals_->rcc().systemClockHz();
    if (frequency == 0U) return std::nullopt;
    auto horizon = deadline;
    if (const auto event = event_loop_->nextScheduledTime();
        event && (!horizon || *event < *horizon)) horizon = *event;
    const SimTimeNs now = event_loop_->now();
    if (horizon && *horizon <= now) return std::nullopt;
    const auto systick = system_->cyclesUntilSysTickInterrupt();
    const auto& flash = peripherals_->flash();
    DeferredPurePrefix prefix{};
    prefix.start_time_ns = now;
    prefix.entry_pc = cpu_->state().r[15];
    prefix.flash_generation = flash.acrGeneration();
    prefix.execution_generation = memory_.executionGeneration();
    prefix.clock_hz = frequency;
    prefix.time_fraction = time_fraction_;
    bool have_fetch = have_last_fetch_;
    std::uint32_t fetch_end = last_fetch_end_;
    std::uint64_t cycles = 0U;
    for (std::uint8_t i = 0U; i < preview.exact_cycle_prefix_count; ++i) {
        const std::uint64_t step_cycles = preview.instruction_cycles[i]
            + flash.fetchStallCycles(preview.pcs[i], have_fetch && preview.pcs[i] == fetch_end);
        cycles += step_cycles;
        if (systick && cycles >= *systick) break;
        const std::uint64_t numerator = cycles * nanoseconds_per_second + time_fraction_;
        const SimTimeNs elapsed = numerator / frequency;
        if (horizon && elapsed >= *horizon - now) break;
        const SimTimeNs completion = saturatingAdd(now, elapsed);
        // Deferred phase advancement must make strict temporal progress even
        // for the first instruction (e.g. clocks above 1 GHz).
        if (completion <= now
            || (prefix.count != 0U && completion <= prefix.completion_times_ns[prefix.count - 1U])) break;
        prefix.cumulative_cycles[prefix.count] = cycles;
        prefix.completion_times_ns[prefix.count] = completion;
        ++prefix.count;
        fetch_end = preview.pcs[i] + preview.sizes[i];
        have_fetch = true;
    }
    if (prefix.count < 2U) return std::nullopt;
    return prefix;
}

Board::ConcurrentStepResult Board::materializeDeferredPurePrefix(
    const DeferredPurePrefix& prefix, const std::size_t count) {
    ConcurrentStepResult rejected{};
    rejected.cpu_result.reason = cpu::StopReason::synchronization_required;
    if (count == 0U || count > prefix.count || count > cpu::CortexM4::JitStepOutcome::max_block
        || cpu_->state().r[15] != prefix.entry_pc
        || peripherals_->rcc().systemClockHz() != prefix.clock_hz
        || peripherals_->flash().acrGeneration() != prefix.flash_generation
        || memory_.executionGeneration() != prefix.execution_generation
        || time_fraction_ != prefix.time_fraction || blockBoundaryPending()) return rejected;

    auto jit = cpu_->tryStepPreparedJitBlock(count);
    if (!jit || jit->count != count
        || !jit->memory_free
        || peripherals_->flash().acrGeneration() != prefix.flash_generation) return rejected;

    auto& flash = peripherals_->flash();
    std::uint64_t cycles = jit->result.cycles;
    bool have_fetch = have_last_fetch_;
    std::uint32_t fetch_end = last_fetch_end_;
    for (std::uint8_t i = 0U; i < jit->count; ++i) {
        const bool sequential = have_fetch && jit->pcs[i] == fetch_end;
        cycles += flash.fetchStallCycles(jit->pcs[i], sequential);
        fetch_end = jit->pcs[i] + jit->sizes[i];
        have_fetch = true;
    }
    if (cycles != prefix.cumulative_cycles[count - 1U]
        || prefix.completion_times_ns[count - 1U] < prefix.start_time_ns) {
        rejected.cpu_result = jit->result;
        rejected.cpu_result.reason = cpu::StopReason::synchronization_required;
        return rejected;
    }
    last_fetch_end_ = fetch_end;
    have_last_fetch_ = have_fetch;
    cpu::FastStepResult result = jit->result;
    result.cycles = static_cast<std::uint16_t>(cycles);
    return ConcurrentStepResult{result, accountCycles(cycles)};
}

std::shared_ptr<const Board::IdempotentPeriodCertificate>
Board::buildIdempotentPeriodCertificate(
    const std::size_t period, const std::uint64_t period_cycles) {
    using Certificate = IdempotentPeriodCertificate;
    if (period == 0U || period > Certificate::max_instructions
        || !memory_.readFootprintTracking()) return {};

    const cpu::CpuState endpoint_state = cpu_->state();
    const bool endpoint_have_fetch = have_last_fetch_;
    const std::uint32_t endpoint_fetch_end = last_fetch_end_;
    const auto endpoint_footprint = memory_.readFootprint();
    const auto checkpoint = memory_.sideEffectCheckpoint();
    const auto restoration_generation = memory_.restorationGeneration();
    const auto execution_generation = memory_.executionGeneration();
    const auto clock_hz = peripherals_->rcc().systemClockHz();
    const auto flash_generation = peripherals_->flash().acrGeneration();
    const std::uint64_t sequence = checkpoint.mutation_sequence;
    auto certificate = std::make_shared<Certificate>();
    certificate->count = static_cast<std::uint8_t>(period);
    certificate->memory_checkpoint = checkpoint;
    certificate->restoration_generation = restoration_generation;
    certificate->execution_generation = execution_generation;
    certificate->clock_hz = clock_hz;
    certificate->flash_acr_generation = flash_generation;
    certificate->states[0] = endpoint_state;
    certificate->footprints[0] = endpoint_footprint;
    certificate->have_fetch[0] = endpoint_have_fetch;
    certificate->fetch_end[0] = endpoint_fetch_end;
    certificate->full_footprint = mem::MemoryBus::ReadFootprint{};
    memory_.restoreReadFootprint(mem::MemoryBus::ReadFootprint{});

    const auto restore_endpoint = [&] {
        const auto current = memory_.sideEffectCheckpoint();
        if (current.mutation_sequence != sequence) {
            static_cast<void>(memory_.restoreSideEffects(checkpoint));
        }
        cpu_->state() = endpoint_state;
        have_last_fetch_ = endpoint_have_fetch;
        last_fetch_end_ = endpoint_fetch_end;
        memory_.restoreReadFootprint(endpoint_footprint);
    };
    bool valid = endpoint_footprint.complete;
    std::uint64_t total_cycles = 0U;
    const auto& flash = peripherals_->flash();
    // This budget is consumed under the all-MMIO-trapping reversible guard,
    // without event callbacks; neither ACR nor flash geometry can change.
    cpu::CortexM4::ReversibleCycleBudget budget{};
    budget.remaining_cycles = std::numeric_limits<std::uint64_t>::max();
    budget.have_fetch = endpoint_have_fetch;
    budget.fetch_end = endpoint_fetch_end;
    const auto wait = static_cast<std::uint16_t>(flash.waitStates());
    budget.fetch_timing = {flash.flashBase(), flash.flashBase() + flash.bankSize(),
        0U, flash.bankSize(), wait,
        static_cast<std::uint16_t>((flash.prefetchEnabled() || flash.instructionCacheEnabled()) ? 0U : wait)};
    {
        ReversibleMemoryGuard guard(memory_);
        mem::MemoryBus::StoreFootprintScope store_footprints(memory_);
        for (std::size_t i = 0U; valid && i < period; ++i) {
            auto step = cpu_->tryStepBudgetedReversibleJitBlock(1U, budget);
            if (!step || step->execution.count != 1U
                || step->execution.result.reason != cpu::StopReason::step_complete) {
                valid = false;
                break;
            }
            const auto& result = step->execution.result;
            const auto now = memory_.sideEffectCheckpoint();
            if (now.mutation_sequence != sequence || result.instruction_address == 0U
                || memory_.executionGeneration() != execution_generation
                || peripherals_->flash().acrGeneration() != flash_generation) {
                valid = false;
                break;
            }
            auto footprint = memory_.takeReadFootprint();
            if (!footprint.complete) {
                valid = false;
                break;
            }
            const std::uint16_t charged = budget.total_instruction_cycles[0];
            if (charged == 0U || total_cycles > std::numeric_limits<std::uint64_t>::max() - charged) {
                valid = false;
                break;
            }
            total_cycles += charged;
            certificate->pcs[i] = result.instruction_address;
            certificate->sizes[i] = result.instruction_size;
            certificate->raw[i] = result.raw;
            certificate->instruction_cycles[i] = charged;
            certificate->cumulative_cycles[i + 1U] = total_cycles;
            certificate->states[i + 1U] = cpu_->state();
            certificate->footprints[i + 1U] = footprint;
            certificate->have_fetch[i + 1U] = budget.have_fetch;
            certificate->fetch_end[i + 1U] = budget.fetch_end;
            for (std::size_t word = 0U; word < footprint.words.size(); ++word) {
                certificate->full_footprint.words[word] |= footprint.words[word];
            }
            certificate->full_footprint.complete = certificate->full_footprint.complete
                && footprint.complete;
        }
        valid = valid && total_cycles == period_cycles
            && memory_.sideEffectCheckpoint().mutation_sequence == sequence
            && memory_.executionGeneration() == execution_generation
            && peripherals_->flash().acrGeneration() == flash_generation
            && cpu::bitwiseEqual(cpu_->state(), endpoint_state)
            && budget.have_fetch == endpoint_have_fetch
            && budget.fetch_end == endpoint_fetch_end;
    }
    restore_endpoint();
    if (!valid || memory_.restorationGeneration() != restoration_generation) return {};
    // Build the pc -> phase index once. Buckets whose phases disagree on PC
    // are marked ambiguous and keep the exact linear scan at admission time.
    // pc_index_count is 1 when the bucket maps to exactly one phase, 0 when the
    // bucket is empty, and 0xff when phases in the bucket disagree on PC
    // (a hash collision) so admission must fall back to the exact scan.
    for (std::size_t i = 0U; i < period; ++i) {
        const auto bucket = Certificate::pcBucket(certificate->pcs[i]);
        const auto existing = certificate->pc_index_count[bucket];
        if (existing == 0U) {
            certificate->pc_index[bucket] = static_cast<std::uint32_t>(i);
            certificate->pc_index_count[bucket] = 1U;
        } else if (existing != 0xffU
            && certificate->pcs[certificate->pc_index[bucket]] != certificate->pcs[i]) {
            certificate->pc_index_count[bucket] = 0xffU;
        }
    }
    idempotent_period_cache_ = certificate;
    period_validation_checkpoint_ = checkpoint;
    period_validation_restoration_generation_ = restoration_generation;
    return idempotent_period_cache_;
}

std::shared_ptr<const Board::WholeRamNeutralPeriodCertificate>
Board::buildWholeRamNeutralPeriodCertificate(
    const std::size_t period, const std::uint64_t period_cycles) {
    using Certificate = WholeRamNeutralPeriodCertificate;
    if (period == 0U || period > Certificate::max_instructions
        || !memory_.readFootprintTracking()) return {};

    const auto endpoint_state = cpu_->state();
    const bool endpoint_have_fetch = have_last_fetch_;
    const auto endpoint_fetch_end = last_fetch_end_;
    const auto endpoint_footprint = memory_.readFootprint();
    const auto start_checkpoint = memory_.sideEffectCheckpoint();
    const auto start_restoration_generation = memory_.restorationGeneration();
    const auto execution_generation = memory_.executionGeneration();
    const auto clock_hz = peripherals_->rcc().systemClockHz();
    const auto flash_generation = peripherals_->flash().acrGeneration();
    auto certificate = std::make_shared<Certificate>();
    certificate->count = static_cast<std::uint8_t>(period);
    certificate->entry_state = endpoint_state;
    certificate->entry_have_fetch = endpoint_have_fetch;
    certificate->entry_fetch_end = endpoint_fetch_end;

    memory_.restoreReadFootprint(mem::MemoryBus::ReadFootprint{});
    bool valid = endpoint_footprint.complete;
    std::uint64_t total_cycles = 0U;
    const auto& flash = peripherals_->flash();
    cpu::CortexM4::ReversibleCycleBudget budget{};
    budget.remaining_cycles = std::numeric_limits<std::uint64_t>::max();
    budget.have_fetch = endpoint_have_fetch;
    budget.fetch_end = endpoint_fetch_end;
    const auto wait = static_cast<std::uint16_t>(flash.waitStates());
    budget.fetch_timing = {flash.flashBase(), flash.flashBase() + flash.bankSize(),
        0U, flash.bankSize(), wait,
        static_cast<std::uint16_t>((flash.prefetchEnabled() || flash.instructionCacheEnabled()) ? 0U : wait)};
    {
        ReversibleMemoryGuard guard(memory_);
        mem::MemoryBus::StoreFootprintScope store_footprints(memory_);
        for (std::size_t i = 0U; valid && i < period; ++i) {
            auto step = cpu_->tryStepBudgetedReversibleJitBlock(1U, budget);
            if (!step || step->execution.count != 1U
                || step->execution.result.reason != cpu::StopReason::step_complete) {
                valid = false;
                break;
            }
            auto footprint = memory_.takeReadFootprint();
            const auto charged = budget.total_instruction_cycles[0];
            if (!footprint.complete || charged == 0U
                || total_cycles > std::numeric_limits<std::uint64_t>::max() - charged) {
                valid = false;
                break;
            }
            const auto& result = step->execution.result;
            total_cycles += charged;
            certificate->pcs[i] = result.instruction_address;
            certificate->sizes[i] = result.instruction_size;
            certificate->raw[i] = result.raw;
            certificate->cpu_instruction_cycles[i] = step->instruction_cycles[0];
            certificate->instruction_cycles[i] = charged;
            certificate->cumulative_cycles[i + 1U] = total_cycles;
            for (std::size_t word = 0U; word < footprint.words.size(); ++word)
                certificate->full_footprint.words[word] |= footprint.words[word];
            certificate->full_footprint.complete = certificate->full_footprint.complete
                && footprint.complete;
        }
        valid = valid && total_cycles == period_cycles
            && cpu::bitwiseEqual(cpu_->state(), endpoint_state)
            && budget.have_fetch == endpoint_have_fetch
            && budget.fetch_end == endpoint_fetch_end
            && memory_.sideEffectsRestoredSince(start_checkpoint)
            && memory_.executionGeneration() == execution_generation
            && peripherals_->rcc().systemClockHz() == clock_hz
            && peripherals_->flash().acrGeneration() == flash_generation;
    }
    if (!valid) {
        const auto current = memory_.sideEffectCheckpoint();
        if (current.mutation_sequence != start_checkpoint.mutation_sequence
            && memory_.canRestoreSideEffects(start_checkpoint)) {
            static_cast<void>(memory_.restoreSideEffects(start_checkpoint));
        }
        cpu_->state() = endpoint_state;
        have_last_fetch_ = endpoint_have_fetch;
        last_fetch_end_ = endpoint_fetch_end;
        memory_.restoreReadFootprint(endpoint_footprint);
        return {};
    }
    // A successful replay is byte-neutral. Keep its journal history rather
    // than restoring an empty/no-op undo (which changes restoration generation).
    cpu_->state() = endpoint_state;
    have_last_fetch_ = endpoint_have_fetch;
    last_fetch_end_ = endpoint_fetch_end;
    memory_.restoreReadFootprint(endpoint_footprint);
    certificate->full_footprint.complete = certificate->full_footprint.complete
        && certificate->count != 0U;
    if (!certificate->full_footprint.complete
        || memory_.restorationGeneration() != start_restoration_generation) return {};
    certificate->checkpoint = memory_.sideEffectCheckpoint();
    certificate->restoration_generation = memory_.restorationGeneration();
    certificate->execution_generation = execution_generation;
    certificate->clock_hz = clock_hz;
    certificate->flash_generation = flash_generation;
    whole_ram_neutral_period_cache_ = certificate;
    whole_ram_neutral_validation_checkpoint_ = certificate->checkpoint;
    return whole_ram_neutral_period_cache_;
}

std::optional<Board::ReversibleRamPrefix> Board::prepareReversibleRamPrefix(
    const std::size_t max_instructions, const std::optional<SimTimeNs> deadline) {
    ReversibleRamPrefix prefix{};
    if (!prepareReversibleRamPrefix(prefix, max_instructions, deadline)) return std::nullopt;
    return prefix;
}

bool Board::prepareReversibleRamPrefix(ReversibleRamPrefix& prefix,
    const std::size_t max_instructions, const std::optional<SimTimeNs> deadline,
    const bool allow_single_prefix, const bool owner_local_barrier_installed,
    const bool cache_only) {
    prefix.count = 0U;
    prefix.memoized_count = 0U;
    prefix.compressed_period = false;
    prefix.period_length = 0U;
    prefix.period_cycles = 0U;
    prefix.first_fetch_delta = 0;
    prefix.folded_instructions = 0U;
    prefix.period_certificate.reset();
    prefix.whole_ram_neutral_period_certificate.reset();
    prefix.transition_certificate.reset();
    prefix.period_phase = 0U;
    prefix.evaluated.count = 0U;
    prefix.evaluated.result = cpu::FastStepResult{};
    std::size_t limit = std::min(max_instructions, DeferredPurePrefix::max_memoized_instructions);
    if (limit < 2U || !memory_.writeJournalTracking() || blockBoundaryPending()) return false;
    const auto frequency = peripherals_->rcc().systemClockHz();
    if (frequency == 0U || time_fraction_ >= frequency) return false;
    const auto now = event_loop_->now();
    auto horizon = deadline;
    // Only a World-installed callback barrier permits evaluation across another
    // lane's audited local ADC event. Standalone callers retain the global
    // horizon because no observer is available to materialize later cuts.
    const auto owner = event_loop_->activeOwner();
    const auto event = !owner_local_barrier_installed || owner == shared_event_owner
        ? event_loop_->nextScheduledTime() : event_loop_->nextObservationTime(owner);
    if (event && (!horizon || *event < *horizon)) horizon = *event;
    if (horizon && *horizon <= now) return false;
    const auto systick = system_->cyclesUntilSysTickInterrupt();
    if (systick && *systick == 0U) return false;
    auto cycle_credit = systick ? *systick - 1U : std::numeric_limits<std::uint64_t>::max();
    if (horizon) {
        const auto available_ns = *horizon - now;
        if (available_ns <= std::numeric_limits<std::uint64_t>::max() / frequency) {
            const auto numerator = available_ns * frequency - 1U;
            const auto time_credit = numerator >= time_fraction_
                ? (numerator - time_fraction_) / nanoseconds_per_second : 0U;
            cycle_credit = std::min(cycle_credit, time_credit);
        }
    }
    if (cycle_credit == 0U) return false;

    // The fixed-capacity metadata arrays are live only below their count.
    // Avoid value-initializing all 64 entries on every admission; callers must
    // likewise consult only the admitted prefix.
    prefix.start_time_ns = now;
    prefix.entry_pc = cpu_->state().r[15];
    prefix.clock_hz = frequency;
    prefix.time_fraction = time_fraction_;
    prefix.flash_generation = peripherals_->flash().acrGeneration();
    prefix.execution_generation = memory_.executionGeneration();
    prefix.memory_checkpoint = memory_.sideEffectCheckpoint();
    prefix.entry_read_footprint = memory_.readFootprint();
    prefix.entry_have_fetch = have_last_fetch_;
    prefix.entry_fetch_end = last_fetch_end_;
    const std::size_t transition_bucket = ((prefix.entry_pc >> 1U)
        ^ (prefix.entry_pc >> 5U)) & (transition_bucket_count - 1U);
    const auto restoration_generation = memory_.restorationGeneration();
    const auto restore_entry = [&] {
        const auto current = memory_.sideEffectCheckpoint();
        if (current.mutation_sequence != prefix.memory_checkpoint.mutation_sequence
            && !memory_.restoreSideEffects(prefix.memory_checkpoint)) {
            throw std::logic_error("reversible RAM admission cannot restore its writes");
        }
        if (!memory_.mmioUnchangedSince(prefix.memory_checkpoint)) {
            throw std::logic_error("reversible RAM execution dispatched MMIO");
        }
        prefix.entry_state.restore(cpu_->state());
        memory_.restoreReadFootprint(prefix.entry_read_footprint);
    };

    // MMIO is trapped and every admitted store is non-executable RAM, so
    // clock/flash timing inputs cannot change during this speculative span.
    const auto& flash = peripherals_->flash();
    const auto elapsed_for_span = [&](const std::uint64_t total_cycles) {
        return cycleTimeQuotient(total_cycles * nanoseconds_per_second + prefix.time_fraction, frequency);
    };
    // RAM-neutral periods are reusable only at their exact entry CPU/fetch
    // phase. Intermediate phases may have different RAM bytes, so never map
    // an arbitrary phase directly from CPU snapshots.
    if (whole_ram_neutral_period_cache_) {
        const auto& cert = *whole_ram_neutral_period_cache_;
        // A phase miss needs no journal walk. Once the exact entry state is
        // proved, validate only mutations since the last successful lookup;
        // rescanning from construction grows the work until journal rollover.
        const bool valid = cert.count != 0U
            && prefix.entry_pc == cert.entry_state.r[15]
            && prefix.entry_have_fetch == cert.entry_have_fetch
            && prefix.entry_fetch_end == cert.entry_fetch_end
            && cpu::bitwiseEqual(cpu_->state(), cert.entry_state)
            && restoration_generation == cert.restoration_generation
            && prefix.execution_generation == cert.execution_generation
            && frequency == cert.clock_hz
            && prefix.flash_generation == cert.flash_generation
            && memory_.ramInputsCompatibleSince(
                whole_ram_neutral_validation_checkpoint_, cert.full_footprint);
        if (valid) {
            // At the exact certificate entry phase, timing is periodic from
            // instruction zero. Keep only one period of deltas and represent
            // the larger admitted span through DeferredPurePrefix's compressed
            // cumulative-cycle accessor.
            const bool compressed = frequency <= nanoseconds_per_second;
            std::size_t admission_limit = std::min(limit,
                compressed ? DeferredPurePrefix::max_memoized_instructions
                           : DeferredPurePrefix::max_instructions);
            if (compressed) {
                prefix.compressed_period = true;
                prefix.period_length = cert.count;
                prefix.period_cycles = cert.cumulative_cycles[cert.count];
                for (std::size_t n = 0U; n < cert.count; ++n) {
                    prefix.rotated_cumulative_cycles[n] = cert.cumulative_cycles[n + 1U];
                }
                std::size_t low = 0U, high = admission_limit;
                while (low < high) {
                    const auto mid = low + (high - low + 1U) / 2U;
                    const auto cycles = prefix.cumulativeCycles(mid - 1U);
                    if (cycles <= cycle_credit && cycles < 65536U
                        && (!systick || cycles < *systick)) low = mid;
                    else high = mid - 1U;
                }
                admission_limit = low;
            }
            std::uint64_t cycles = 0U;
            if (compressed) {
                prefix.count = admission_limit;
                if (prefix.count != 0U) cycles = prefix.cumulativeCycles(prefix.count - 1U);
            } else {
                for (std::size_t n = 0U; n < admission_limit; ++n) {
                    const auto charged = cert.instruction_cycles[n % cert.count];
                    if (charged == 0U || charged > cycle_credit - std::min(cycle_credit, cycles)
                        || cycles + charged >= 65536U) break;
                    cycles += charged;
                    if (systick && cycles >= *systick) break;
                    const auto elapsed = elapsed_for_span(cycles);
                    if (horizon && elapsed >= *horizon - now) break;
                    const auto completion = saturatingAdd(now, elapsed);
                    if (completion <= now || (prefix.count != 0U
                        && completion <= prefix.completion_times_ns[prefix.count - 1U])) break;
                    prefix.cumulative_cycles[prefix.count] = cycles;
                    prefix.completion_times_ns[prefix.count] = completion;
                    ++prefix.count;
                }
            }
            if (compressed && prefix.count != 0U) {
                const auto elapsed = elapsed_for_span(cycles);
                if (elapsed == std::numeric_limits<SimTimeNs>::max()
                    || (horizon && elapsed >= *horizon - now)
                    || saturatingAdd(now, elapsed) <= now) {
                    prefix.count = 0U;
                }
            }
            if (prefix.count >= (allow_single_prefix ? 1U : 2U)) {
                prefix.memoized_count = prefix.count;
                prefix.whole_ram_neutral_period_certificate = whole_ram_neutral_period_cache_;
                prefix.evaluated.result.reason = cpu::StopReason::step_complete;
                prefix.evaluated.result.instructions = static_cast<std::uint32_t>(prefix.count);
                prefix.evaluated.count = static_cast<std::uint8_t>(std::min<std::size_t>(
                    prefix.count, ReversibleRamPrefix::max_instructions));
                prefix.evaluated_checkpoint = prefix.memory_checkpoint;
                whole_ram_neutral_validation_checkpoint_ = prefix.memory_checkpoint;
                return true;
            }
            prefix.count = 0U;
            prefix.compressed_period = false;
            prefix.period_length = 0U;
            prefix.period_cycles = 0U;
        }
    }
    // A certificate is reusable at every phase, provided no RAM input read by
    // the complete period has changed since its last successful validation.
    if (idempotent_period_cache_) {
        const auto& cert = *idempotent_period_cache_;
        const bool compatible = cert.count != 0U
            && restoration_generation == cert.restoration_generation
            && restoration_generation == period_validation_restoration_generation_
            && memory_.ramInputsCompatibleSince(period_validation_checkpoint_, cert.full_footprint)
            && prefix.execution_generation == cert.execution_generation
            && frequency == cert.clock_hz
            && prefix.flash_generation == cert.flash_acr_generation;
        bool valid = compatible;
        std::size_t phase = 0U;
        if (valid) {
            // Fast path: the pc->phase index narrows the search to (almost
            // always) one candidate. Ambiguous buckets and differing CPU state
            // fall back to the exact per-phase comparison below.
            const auto bucket = IdempotentPeriodCertificate::pcBucket(prefix.entry_pc);
            if (cert.pc_index_count[bucket] == 1U
                && cert.pcs[cert.pc_index[bucket]] == prefix.entry_pc) {
                const std::size_t candidate = cert.pc_index[bucket];
                if (cpu::bitwiseEqual(cpu_->state(), cert.states[candidate])) {
                    phase = candidate;
                } else {
                    valid = false;
                }
            } else {
                valid = false;
                for (std::size_t i = 0U; i < cert.count; ++i) {
                    if (prefix.entry_pc == cert.pcs[i]
                        && cpu::bitwiseEqual(cpu_->state(), cert.states[i])) {
                        phase = i;
                        valid = true;
                        break;
                    }
                }
            }
        }
        if (!valid) {
            // A different CPU phase is a lookup miss, not invalidation. Keep
            // the immutable certificate through temporary excursions; its
            // inputs and generation guards must still pass on every lookup.
            if (!compatible) idempotent_period_cache_.reset();
        } else {
            const bool compressed = frequency <= nanoseconds_per_second;
            std::size_t admission_limit = std::min<std::size_t>(limit,
                compressed ? DeferredPurePrefix::max_memoized_instructions
                           : DeferredPurePrefix::max_instructions);
            const bool current_sequential = have_last_fetch_
                && last_fetch_end_ == prefix.entry_pc;
            const std::size_t first_index = phase;
            std::int64_t first_delta = 0;
            if (compressed) {
                const bool certified_sequential = cert.have_fetch[first_index]
                    && cert.fetch_end[first_index] == cert.pcs[first_index];
                const auto certified_stall = flash.fetchStallCycles(
                    cert.pcs[first_index], certified_sequential);
                const auto current_stall = flash.fetchStallCycles(
                    cert.pcs[first_index], current_sequential);
                first_delta = static_cast<std::int64_t>(current_stall)
                    - static_cast<std::int64_t>(certified_stall);
                prefix.compressed_period = true;
                prefix.period_length = cert.count;
                prefix.first_fetch_delta = first_delta;
                prefix.period_cycles = cert.cumulative_cycles[cert.count];
                std::uint64_t rotated_cycles = 0U;
                for (std::size_t n = 0U; n < cert.count; ++n) {
                    const auto index = (phase + n) % cert.count;
                    const auto charged = static_cast<std::uint64_t>(cert.instruction_cycles[index]);
                    if (charged == 0U) {
                        prefix.compressed_period = false;
                        break;
                    }
                    rotated_cycles += charged;
                    prefix.rotated_cumulative_cycles[n] = rotated_cycles;
                }
                if (prefix.compressed_period) {
                    prefix.period_cycles = rotated_cycles;
                    // Find the longest valid leading prefix using monotone cycle credit.
                    std::size_t low = 0U, high = admission_limit;
                    while (low < high) {
                        const auto mid = low + (high - low + 1U) / 2U;
                        const auto full = mid / cert.count;
                        const auto rem = mid % cert.count;
                        auto total = full * rotated_cycles
                            + (rem == 0U ? 0U : prefix.rotated_cumulative_cycles[rem - 1U]);
                        if (first_delta < 0) total -= static_cast<std::uint64_t>(-first_delta);
                        else total += static_cast<std::uint64_t>(first_delta);
                        if (total <= cycle_credit && total < 65536U) low = mid;
                        else high = mid - 1U;
                    }
                    prefix.count = low;
                    prefix.memoized_count = low;
                    if (low != 0U) {
                        const auto elapsed = (prefix.cumulativeCycles(low - 1U)
                            * nanoseconds_per_second + prefix.time_fraction) / frequency;
                        if (elapsed >= std::numeric_limits<SimTimeNs>::max() - now) {
                            // Saturated timestamps can no longer certify strict
                            // completion order. Re-admit through the bounded array path.
                            prefix.compressed_period = false;
                            prefix.period_length = 0U;
                            prefix.period_cycles = 0U;
                            prefix.first_fetch_delta = 0;
                            prefix.count = 0U;
                            prefix.memoized_count = 0U;
                        }
                    }
                }
            }
            if (!prefix.compressed_period) {
                admission_limit = std::min(admission_limit, DeferredPurePrefix::max_instructions);
                std::uint64_t cycles = 0U;
                for (std::size_t n = 0U; n < admission_limit; ++n) {
                    const auto index = (phase + n) % cert.count;
                    std::uint64_t charged = cert.instruction_cycles[index];
                    if (n == 0U) {
                        const bool certified_sequential = cert.have_fetch[index]
                            && cert.fetch_end[index] == cert.pcs[index];
                        if (current_sequential != certified_sequential) {
                            charged = charged - flash.fetchStallCycles(cert.pcs[index], certified_sequential)
                                + flash.fetchStallCycles(cert.pcs[index], current_sequential);
                        }
                    }
                    if (charged == 0U || charged > cycle_credit - std::min(cycle_credit, cycles)
                        || cycles + charged >= 65536U) break;
                    cycles += charged;
                    if (systick && cycles >= *systick) break;
                    const auto elapsed = elapsed_for_span(cycles);
                    if (horizon && elapsed >= *horizon - now) break;
                    const auto completion = saturatingAdd(now, elapsed);
                    if (completion <= now || (prefix.count != 0U
                        && completion <= prefix.completion_times_ns[prefix.count - 1U])) break;
                    prefix.cumulative_cycles[prefix.count] = cycles;
                    prefix.completion_times_ns[prefix.count] = completion;
                    ++prefix.count;
                }
            } else {
                // The cycle-credit calculation already excludes SysTick and the
                // horizon. Positive cycles at <=1GHz guarantee strict progress.
                prefix.count = std::min(prefix.count, admission_limit);
            }
            if (prefix.count < (allow_single_prefix ? 1U : 2U)) {
                prefix.count = 0U;
                prefix.memoized_count = 0U;
                prefix.compressed_period = false;
            } else {
                prefix.evaluated.result = cpu::FastStepResult{};
                prefix.evaluated.result.reason = cpu::StopReason::step_complete;
                prefix.evaluated.result.instructions = static_cast<std::uint32_t>(prefix.count);
                const std::size_t end_phase = (phase + prefix.count) % cert.count;
                cpu_->state() = cert.states[end_phase];
                have_last_fetch_ = cert.have_fetch[end_phase];
                last_fetch_end_ = cert.fetch_end[end_phase];
                prefix.period_certificate = idempotent_period_cache_;
                prefix.period_phase = static_cast<std::uint8_t>(phase);
                prefix.memoized_count = prefix.count;
                prefix.evaluated.count = static_cast<std::uint8_t>(std::min<std::size_t>(
                    prefix.count, ReversibleRamPrefix::max_instructions));
                // Validation and state matching above do not mutate memory.
                prefix.evaluated_checkpoint = prefix.memory_checkpoint;
                period_validation_checkpoint_ = prefix.memory_checkpoint;
                period_validation_restoration_generation_ = restoration_generation;
                return true;
            }
        }
    }
    // Periods remain preferred. Search only four ways in this PC hash bucket;
    // an entry/fetch mismatch is a miss, not a reason to evict another phase.
    const std::size_t first_slot = transition_bucket * transition_ways;
    for (std::size_t way = 0U; way < transition_ways; ++way) {
        const std::size_t slot = first_slot + way;
        const auto& cert = ram_neutral_transition_cache_[slot];
        if (!cert || cert->count == 0U
            || cert->entry.r[15] != prefix.entry_pc
            || !cert->entry.matches(cpu_->state())
            || cert->entry_have_fetch != prefix.entry_have_fetch
            || cert->entry_fetch_end != prefix.entry_fetch_end) continue;
        const bool valid = restoration_generation == cert->restoration_generation
            && restoration_generation == transition_restoration_generations_[slot]
            && memory_.ramInputsCompatibleSince(transition_validation_checkpoints_[slot], cert->full_footprint)
            && prefix.execution_generation == cert->execution_generation
            && frequency == cert->clock_hz
            && prefix.flash_generation == cert->flash_generation;
        if (!valid) {
            ram_neutral_transition_cache_[slot].reset();
            transition_hotness_[slot] = 0U;
            continue;
        }
        std::uint64_t cycles = 0U;
        for (std::size_t i = 0U; i < cert->count && i < limit; ++i) {
            const auto charged = cert->instruction_cycles[i];
            if (charged == 0U || charged > cycle_credit - std::min(cycle_credit, cycles)
                || cycles + charged >= 65536U) break;
            cycles += charged;
            if (systick && cycles >= *systick) break;
            const auto elapsed = elapsed_for_span(cycles);
            if (horizon && elapsed >= *horizon - now) break;
            const auto completion = saturatingAdd(now, elapsed);
            if (completion <= now) break;
            prefix.cumulative_cycles[i] = cycles;
            prefix.completion_times_ns[i] = completion;
            ++prefix.count;
        }
        if (prefix.count < (allow_single_prefix ? 1U : 2U)) {
            prefix.count = 0U;
            continue;
        }
        prefix.memoized_count = prefix.count;
        prefix.transition_certificate = cert;
        prefix.transition_cache_slot = slot;
        prefix.transition_validation_checkpoint = prefix.memory_checkpoint;
        prefix.transition_restoration_generation = restoration_generation;
        prefix.evaluated.result.reason = cpu::StopReason::step_complete;
        prefix.evaluated.result.instructions = static_cast<std::uint32_t>(prefix.count);
        prefix.evaluated.count = static_cast<std::uint8_t>(prefix.count);
        cert->states[prefix.count].restore(cpu_->state());
        have_last_fetch_ = cert->have_fetch[prefix.count];
        last_fetch_end_ = cert->fetch_end[prefix.count];
        prefix.evaluated_checkpoint = prefix.memory_checkpoint;
        transition_validation_checkpoints_[slot] = prefix.transition_validation_checkpoint;
        transition_restoration_generations_[slot] = prefix.transition_restoration_generation;
        ++transition_hotness_[slot];
        return true;
    }
    // Owner-local execution can commit ordinary RAM spans directly. Keep
    // certificate hits, but avoid speculative snapshots/metadata on sampled
    // discovery misses; this declines optimization, never guest work.
    if (cache_only) return false;
    // Cached cuts restore certificate states directly, never entry_state.
    // Capture the rollback snapshot only before actual speculative execution.
    prefix.entry_state.capture(cpu_->state());
    // Only misses need hotness tracking for building a new transition. A hit
    // already has a certificate; avoid another full entry-state comparison.
    auto& probe = transition_probes_[transition_bucket];
    const bool same_probe = probe.valid && probe.entry.matches(cpu_->state())
        && probe.have_fetch == prefix.entry_have_fetch
        && probe.fetch_end == prefix.entry_fetch_end
        && probe.execution_generation == prefix.execution_generation
        && probe.restoration_generation == restoration_generation
        && probe.clock_hz == frequency && probe.flash_generation == prefix.flash_generation;
    if (same_probe) {
        if (probe.observations < transition_hotness_threshold) ++probe.observations;
    } else {
        probe.entry = prefix.entry_state;
        probe.valid = true;
        probe.have_fetch = prefix.entry_have_fetch;
        probe.fetch_end = prefix.entry_fetch_end;
        probe.observations = 1U;
        probe.execution_generation = prefix.execution_generation;
        probe.restoration_generation = restoration_generation;
        probe.clock_hz = frequency;
        probe.flash_generation = prefix.flash_generation;
    }
    // Cache hits need neither the flash timing snapshot nor the speculative
    // budget (including its per-instruction arrays). Initialize only on a miss.
    // MMIO and event barriers end this span before timing can change.
    cpu::CortexM4::ReversibleCycleBudget budget;
    budget.remaining_cycles = cycle_credit;
    budget.have_fetch = prefix.entry_have_fetch;
    budget.fetch_end = prefix.entry_fetch_end;
    const auto wait = static_cast<std::uint16_t>(flash.waitStates());
    budget.fetch_timing = {flash.flashBase(), flash.flashBase() + flash.bankSize(),
        0U, flash.bankSize(), wait,
        static_cast<std::uint16_t>((flash.prefetchEnabled() || flash.instructionCacheEnabled()) ? 0U : wait)};
    // The ordinary reversible-evaluation metadata remains fixed-capacity;
    // only a validated period certificate can admit the larger compressed span.
    limit = std::min(limit, ReversibleRamPrefix::max_instructions);
    // The speculative pass writes per-instruction metadata straight into
    // prefix.evaluated so a committed uncut span needs no second metadata copy;
    // the cut-replay branch below overwrites it from `replayed` when needed.
    auto& speculative = prefix.evaluated;
    speculative.count = 0U;
    speculative.result = cpu::FastStepResult{};
    bool stop = false;
    bool boundary_cut = false;
    std::uint64_t cycles = 0U;
    {
        ReversibleMemoryGuard guard(memory_);
        while (speculative.count < limit && !stop) {
            const std::size_t request = std::min<std::size_t>(
                limit - speculative.count, cpu::CortexM4::JitStepOutcome::max_block);
            auto chunk = cpu_->tryStepBudgetedReversibleJitBlock(request, budget);
            if (!chunk || chunk->execution.count == 0U) break;
            const std::uint8_t chunk_count = chunk->execution.count;
            if (!appendReversibleChunk(speculative, *chunk)) break;
            for (std::uint8_t i = 0U; i < chunk_count; ++i) {
                cycles += budget.total_instruction_cycles[i];
                if ((systick && cycles >= *systick)) {
                    boundary_cut = true;
                    stop = true;
                    break;
                }
                const SimTimeNs elapsed = elapsed_for_span(cycles);
                if (horizon && elapsed >= *horizon - now) {
                    boundary_cut = true;
                    stop = true;
                    break;
                }
                const SimTimeNs completion = saturatingAdd(now, elapsed);
                if (completion <= now || (prefix.count != 0U
                    && completion <= prefix.completion_times_ns[prefix.count - 1U])) {
                    boundary_cut = true;
                    stop = true;
                    break;
                }
                prefix.cumulative_cycles[prefix.count] = cycles;
                prefix.completion_times_ns[prefix.count++] = completion;

            }
            if (chunk->execution.result.reason != cpu::StopReason::step_complete) stop = true;
            // A naturally restored handler-only period is deterministic until
            // an observation barrier. Certify before any rollback: both RAM
            // and all handler-writable CPU/fetch state must equal the entry.
            // Metadata-only repeats still replay normally at every partial cut.
            const std::size_t period = prefix.count;
            if (!stop && period != 0U && period == speculative.count
                && cpu_->state().r[15] == prefix.entry_pc
                && prefix.entry_state.matchesExceptInstructionAddress(cpu_->state())
                && memory_.executionGeneration() == prefix.execution_generation
                && memory_.sideEffectsRestoredSince(prefix.memory_checkpoint)) {
                const std::uint64_t period_cycles = cycles;
                const bool same_fetch = budget.have_fetch == prefix.entry_have_fetch
                    && budget.fetch_end == prefix.entry_fetch_end;
                const bool discovery_only = !same_fetch
                    || prefix.entry_state.instruction_address != cpu_->state().instruction_address;
                std::uint64_t certificate_cycles = period_cycles;
                bool adjusted_cycles_valid = true;
                if (discovery_only) {
                    const auto& flash = peripherals_->flash();
                    const std::uint32_t original_stall = flash.fetchStallCycles(prefix.entry_pc,
                        prefix.entry_have_fetch && prefix.entry_pc == prefix.entry_fetch_end);
                    const std::uint32_t endpoint_stall = flash.fetchStallCycles(prefix.entry_pc,
                        budget.have_fetch && prefix.entry_pc == budget.fetch_end);
                    if (endpoint_stall >= original_stall) {
                        const auto delta = static_cast<std::uint64_t>(endpoint_stall - original_stall);
                        adjusted_cycles_valid = period_cycles <= std::numeric_limits<std::uint64_t>::max() - delta;
                        if (adjusted_cycles_valid) certificate_cycles += delta;
                    } else {
                        const auto delta = static_cast<std::uint64_t>(original_stall - endpoint_stall);
                        adjusted_cycles_valid = period_cycles >= delta;
                        if (adjusted_cycles_valid) certificate_cycles -= delta;
                    }
                }
                // Certification is worthwhile only when a later folded period
                // fits this admission's instruction and cycle credit.
                const bool can_certify = limit - prefix.count >= period
                    && adjusted_cycles_valid && certificate_cycles <= budget.remaining_cycles;
                bool whole_period_certified = false;
                if (can_certify && period <= IdempotentPeriodCertificate::max_instructions) {
                    struct FetchMetadataRestore {
                        Board& board;
                        bool have_fetch;
                        std::uint32_t fetch_end;
                        ~FetchMetadataRestore() {
                            board.have_last_fetch_ = have_fetch;
                            board.last_fetch_end_ = fetch_end;
                        }
                    };
                    const bool saved_have_fetch = have_last_fetch_;
                    const std::uint32_t saved_fetch_end = last_fetch_end_;
                    if (discovery_only) {
                        have_last_fetch_ = budget.have_fetch;
                        last_fetch_end_ = budget.fetch_end;
                    }
                    {
                        FetchMetadataRestore restore{*this, saved_have_fetch, saved_fetch_end};
                        const auto now_checkpoint = memory_.sideEffectCheckpoint();
                        if (now_checkpoint.mutation_sequence == prefix.memory_checkpoint.mutation_sequence) {
                            prefix.period_certificate = buildIdempotentPeriodCertificate(
                                period, certificate_cycles);
                        } else {
                            const auto certificate = buildWholeRamNeutralPeriodCertificate(
                                period, certificate_cycles);
                            whole_period_certified = certificate != nullptr;
                            if (whole_period_certified) {
                                prefix.evaluated_checkpoint = memory_.sideEffectCheckpoint();
                                if (!discovery_only)
                                    prefix.whole_ram_neutral_period_certificate = certificate;
                            }
                        }
                    }
                }
                // A certificate discovered from the actual endpoint is useful
                // for future admissions, but its first ordinary prefix must
                // remain the exact un-folded execution just observed.
                if (discovery_only) {
                    if (prefix.period_certificate || whole_period_certified) {
                        speculative.count = static_cast<std::uint8_t>(prefix.count);
                        speculative.result.instructions = static_cast<std::uint32_t>(prefix.count);
                        break;
                    }
                    // Certification may decline at a cold interior instruction.
                    // Keep collecting the ordinary checked span rather than
                    // caching a needlessly short transition at this boundary.
                    continue;
                }
                // The actual checked period already returned CPU, fetch and
                // every RAM byte to its entry state under the MMIO guard.
                // Repeating it within this same event-bounded transaction needs
                // no persistent certificate; partial cuts retain checked replay.
                const std::uint64_t fold_period_cycles = prefix.whole_ram_neutral_period_certificate
                    ? prefix.whole_ram_neutral_period_certificate->cumulative_cycles[period]
                    : period_cycles;
                while (limit - prefix.count >= period
                       && fold_period_cycles <= budget.remaining_cycles) {
                    for (std::size_t i = 0U; i < period; ++i) {
                        const std::size_t dst = prefix.count++;
                        const auto charged = prefix.whole_ram_neutral_period_certificate
                            ? prefix.whole_ram_neutral_period_certificate->instruction_cycles[i]
                            : prefix.cumulative_cycles[i]
                                - (i == 0U ? 0U : prefix.cumulative_cycles[i - 1U]);
                        cycles += charged;
                        prefix.cumulative_cycles[dst] = cycles;
                        prefix.completion_times_ns[dst] = saturatingAdd(now, elapsed_for_span(cycles));
                        speculative.pcs[dst] = prefix.whole_ram_neutral_period_certificate
                            ? prefix.whole_ram_neutral_period_certificate->pcs[i] : speculative.pcs[i];
                        speculative.sizes[dst] = prefix.whole_ram_neutral_period_certificate
                            ? prefix.whole_ram_neutral_period_certificate->sizes[i] : speculative.sizes[i];
                        speculative.instruction_cycles[dst] = prefix.whole_ram_neutral_period_certificate
                            ? prefix.whole_ram_neutral_period_certificate->cpu_instruction_cycles[i]
                            : speculative.instruction_cycles[i];
                        ++prefix.folded_instructions;
                    }
                    budget.remaining_cycles -= fold_period_cycles;
                }
                speculative.count = static_cast<std::uint8_t>(prefix.count);
                speculative.result.instructions = static_cast<std::uint32_t>(prefix.count);
                // The canonical state is already the exact full-period endpoint.
                // Do not append a speculative partial period to this certificate.
                break;
            }
        }
    }
    const auto after = memory_.sideEffectCheckpoint();
    constexpr std::size_t max_writes_per_instruction = 4U;
    if (after.mutation_sequence - prefix.memory_checkpoint.mutation_sequence
            > max_writes_per_instruction * mem::MemoryBus::max_reversible_ram_mutations
        || !memory_.canRestoreSideEffects(prefix.memory_checkpoint)) {
        throw std::logic_error("reversible RAM prefix exceeded its journal bound");
    }
    if (prefix.count < (allow_single_prefix ? 1U : 2U)) {
        restore_entry();
        return false;
    }

    // A chunk can execute past the first event/deadline/SysTick cut. Undo that
    // speculative suffix and regenerate exactly the certified instruction count.
    // The same path also makes every chained block cut replayable.
    if (speculative.count != prefix.count || boundary_cut) {
        restore_entry();
        ReversibleRamPrefix::ReversibleExecution replayed;
        ReversibleMemoryGuard guard(memory_);
        while (replayed.count < prefix.count) {
            const std::size_t request = std::min<std::size_t>(
                static_cast<std::size_t>(prefix.count - replayed.count),
                cpu::CortexM4::JitStepOutcome::max_block);
            auto chunk = cpu_->tryStepReversibleJitBlock(request);
            if (!chunk || chunk->execution.count == 0U
                || !appendReversibleChunk(replayed, *chunk)) {
                throw std::logic_error("reversible RAM admission replay made no progress");
            }
            if (chunk->execution.result.reason != cpu::StopReason::step_complete
                && replayed.count < prefix.count) {
                throw std::logic_error("reversible RAM admission replay stopped early");
            }
        }
        if (replayed.count != prefix.count) {
            throw std::logic_error("reversible RAM admission replay count differs");
        }
        prefix.evaluated.result = replayed.result;
        prefix.evaluated.count = replayed.count;
        for (std::size_t i = 0U; i < replayed.count; ++i) {
            prefix.evaluated.pcs[i] = replayed.pcs[i];
            prefix.evaluated.sizes[i] = replayed.sizes[i];
            prefix.evaluated.instruction_cycles[i] = replayed.instruction_cycles[i];
        }
    }
    // Uncut path: `speculative` already aliases prefix.evaluated, so the
    // executed metadata is in place and needs no further copying.
    prefix.evaluated_checkpoint = memory_.sideEffectCheckpoint();

    // Re-run a short RAM-neutral span to retain per-instruction CPU endpoints
    // and read footprints. Crucially, a neutral replay needs no journal restore:
    // restoreSideEffects advances restorationGeneration even for an empty undo.
    if (probe.observations >= transition_hotness_threshold
        && prefix.count <= RamNeutralTransitionCertificate::max_instructions
        && prefix.count >= 2U
        && prefix.evaluated_checkpoint.mutation_sequence == prefix.memory_checkpoint.mutation_sequence
        && memory_.readFootprintTracking()) {
        auto cert = std::make_shared<RamNeutralTransitionCertificate>();
        cert->count = static_cast<std::uint8_t>(prefix.count);
        cert->entry = prefix.entry_state;
        cert->entry_have_fetch = prefix.entry_have_fetch;
        cert->entry_fetch_end = prefix.entry_fetch_end;
        cert->checkpoint = prefix.memory_checkpoint;
        cert->execution_generation = prefix.execution_generation;
        cert->clock_hz = prefix.clock_hz;
        cert->flash_generation = prefix.flash_generation;
        cert->states[0] = prefix.entry_state;
        cert->have_fetch[0] = prefix.entry_have_fetch;
        cert->fetch_end[0] = prefix.entry_fetch_end;
        const auto endpoint = cpu_->state();
        const auto endpoint_footprint = memory_.readFootprint();
        const bool endpoint_have_fetch = have_last_fetch_;
        const auto endpoint_fetch_end = last_fetch_end_;
        memory_.restoreReadFootprint(mem::MemoryBus::ReadFootprint{});
        prefix.entry_state.restore(cpu_->state());
        have_last_fetch_ = prefix.entry_have_fetch;
        last_fetch_end_ = prefix.entry_fetch_end;
        bool valid = true;
        {
            ReversibleMemoryGuard guard(memory_);
            // Skipped idempotent stores depend on their destination retaining
            // its value, even when no load reads that address. Record store
            // destinations along with reads before certifying each cut.
            mem::MemoryBus::StoreFootprintScope store_footprints(memory_);
            for (std::size_t i = 0U; i < prefix.count; ++i) {
                const auto step = cpu_->stepFast();
                if (step.reason != cpu::StopReason::step_complete || step.instructions != 1U
                    || memory_.sideEffectCheckpoint().mutation_sequence
                        != prefix.memory_checkpoint.mutation_sequence) {
                    valid = false;
                    break;
                }
                auto part = memory_.takeReadFootprint();
                if (!part.complete) { valid = false; break; }
                cert->pcs[i] = step.instruction_address;
                cert->raw[i] = step.raw;
                cert->sizes[i] = step.instruction_size;
                cert->instruction_cycles[i] = static_cast<std::uint16_t>(
                    prefix.cumulative_cycles[i] - (i == 0U ? 0U : prefix.cumulative_cycles[i - 1U]));
                cert->cumulative_cycles[i + 1U] = prefix.cumulative_cycles[i];
                cert->states[i + 1U].capture(cpu_->state());
                cert->footprints[i + 1U] = part;
                cert->have_fetch[i + 1U] = true;
                cert->fetch_end[i + 1U] = step.instruction_address + step.instruction_size;
                for (std::size_t word = 0U; word < part.words.size(); ++word)
                    cert->full_footprint.words[word] |= part.words[word];
                cert->full_footprint.complete = cert->full_footprint.complete && part.complete;
            }
        }
        const auto after_replay = memory_.sideEffectCheckpoint();
        if (after_replay.mutation_sequence != prefix.memory_checkpoint.mutation_sequence) {
            valid = false;
            static_cast<void>(memory_.restoreSideEffects(prefix.memory_checkpoint));
        }
        valid = valid && cpu::bitwiseEqual(cpu_->state(), endpoint)
            && memory_.mmioUnchangedSince(prefix.memory_checkpoint)
            && memory_.executionGeneration() == prefix.execution_generation
            && peripherals_->rcc().systemClockHz() == prefix.clock_hz
            && peripherals_->flash().acrGeneration() == prefix.flash_generation;
        cpu_->state() = endpoint;
        have_last_fetch_ = endpoint_have_fetch;
        last_fetch_end_ = endpoint_fetch_end;
        memory_.restoreReadFootprint(endpoint_footprint);
        if (valid && cert->full_footprint.complete) {
            cert->restoration_generation = memory_.restorationGeneration();
            std::size_t slot = first_slot;
            for (std::size_t way = 0U; way < transition_ways; ++way) {
                const auto candidate = first_slot + way;
                if (!ram_neutral_transition_cache_[candidate]) {
                    slot = candidate;
                    break;
                }
                if (transition_hotness_[candidate] < transition_hotness_[slot]) slot = candidate;
            }
            ram_neutral_transition_cache_[slot] = std::move(cert);
            transition_validation_checkpoints_[slot] = prefix.memory_checkpoint;
            transition_restoration_generations_[slot] = memory_.restorationGeneration();
            transition_hotness_[slot] = probe.observations;
        }
    }
    return true;
}

Board::ConcurrentStepResult Board::materializeReversibleRamPrefix(
    const ReversibleRamPrefix& prefix, const std::size_t count) {
    const auto current = memory_.sideEffectCheckpoint();
    if (count == 0U || count > prefix.count
        || current.mutation_sequence != prefix.evaluated_checkpoint.mutation_sequence
        || current.mmio_generation != prefix.evaluated_checkpoint.mmio_generation
        || peripherals_->rcc().systemClockHz() != prefix.clock_hz
        || peripherals_->flash().acrGeneration() != prefix.flash_generation
        || memory_.executionGeneration() != prefix.execution_generation
        || time_fraction_ != prefix.time_fraction) {
        throw std::logic_error("stale reversible RAM prefix");
    }
    if (prefix.whole_ram_neutral_period_certificate && prefix.memoized_count != 0U) {
        const auto& cert = *prefix.whole_ram_neutral_period_certificate;
        if (whole_ram_neutral_period_cache_ != prefix.whole_ram_neutral_period_certificate
            || cert.count == 0U || count > prefix.memoized_count
            || memory_.restorationGeneration() != cert.restoration_generation
            || !memory_.ramInputsCompatibleSince(prefix.memory_checkpoint, cert.full_footprint)
            || !cpu::bitwiseEqual(cpu_->state(), cert.entry_state)
            || have_last_fetch_ != cert.entry_have_fetch
            || last_fetch_end_ != cert.entry_fetch_end) {
            throw std::logic_error("stale whole RAM-neutral period certificate");
        }
        const std::size_t remainder = count % cert.count;
        if (remainder != 0U) {
            ReversibleMemoryGuard guard(memory_);
            for (std::size_t i = 0U; i < remainder; ++i) {
                const auto step = cpu_->stepFast();
                if (step.reason != cpu::StopReason::step_complete || step.instructions != 1U
                    || step.instruction_address != cert.pcs[i]
                    || step.instruction_size != cert.sizes[i]) {
                    throw std::logic_error("whole RAM-neutral period residual replay differs");
                }
                const auto actual = static_cast<std::uint64_t>(step.cycles)
                    + peripherals_->flash().fetchStallCycles(step.instruction_address,
                        i == 0U ? cert.entry_have_fetch
                            && step.instruction_address == cert.entry_fetch_end
                            : step.instruction_address == cert.pcs[i - 1U] + cert.sizes[i - 1U]);
                if (actual != cert.instruction_cycles[i]) {
                    throw std::logic_error("whole RAM-neutral period residual timing differs");
                }
            }
            last_fetch_end_ = cert.pcs[remainder - 1U] + cert.sizes[remainder - 1U];
            have_last_fetch_ = true;
        }
        auto footprint = prefix.entry_read_footprint;
        if (count != 0U) {
            for (std::size_t word = 0U; word < footprint.words.size(); ++word)
                footprint.words[word] |= cert.full_footprint.words[word];
            footprint.complete = footprint.complete && cert.full_footprint.complete;
        }
        memory_.restoreReadFootprint(footprint);
        const auto last = (count - 1U) % cert.count;
        auto result = prefix.evaluated.result;
        result.instructions = static_cast<std::uint32_t>(count);
        result.instruction_address = cert.pcs[last];
        result.raw = cert.raw[last];
        result.instruction_size = cert.sizes[last];
        const auto cycles = prefix.cumulativeCycles(count - 1U);
        result.cycles = static_cast<std::uint16_t>(cycles);
        return ConcurrentStepResult{result, accountCycles(cycles)};
    }
    if (prefix.transition_certificate && prefix.memoized_count != 0U) {
        const auto& cert = *prefix.transition_certificate;
        if (count > prefix.memoized_count || count > cert.count
            || prefix.transition_cache_slot >= transition_slot_count
            || ram_neutral_transition_cache_[prefix.transition_cache_slot] != prefix.transition_certificate
            || memory_.restorationGeneration() != cert.restoration_generation
            || memory_.restorationGeneration() != prefix.transition_restoration_generation
            || memory_.restorationGeneration() != transition_restoration_generations_[prefix.transition_cache_slot]
            || !cert.states[prefix.memoized_count].matches(cpu_->state())) {
            if (prefix.transition_cache_slot < transition_slot_count
                && ram_neutral_transition_cache_[prefix.transition_cache_slot] == prefix.transition_certificate) {
                ram_neutral_transition_cache_[prefix.transition_cache_slot].reset();
                transition_hotness_[prefix.transition_cache_slot] = 0U;
            }
            throw std::logic_error("stale RAM-neutral transition certificate");
        }
        cert.states[count].restore(cpu_->state());
        have_last_fetch_ = cert.have_fetch[count];
        last_fetch_end_ = cert.fetch_end[count];
        auto footprint = prefix.entry_read_footprint;
        for (std::size_t n = 0U; n < count; ++n) {
            const auto& part = cert.footprints[n + 1U];
            for (std::size_t word = 0U; word < footprint.words.size(); ++word)
                footprint.words[word] |= part.words[word];
            footprint.complete = footprint.complete && part.complete;
        }
        memory_.restoreReadFootprint(footprint);
        auto result = prefix.evaluated.result;
        result.instructions = static_cast<std::uint32_t>(count);
        result.instruction_address = cert.pcs[count - 1U];
        result.raw = cert.raw[count - 1U];
        result.instruction_size = cert.sizes[count - 1U];
        const auto cycles = prefix.cumulative_cycles[count - 1U];
        result.cycles = static_cast<std::uint16_t>(cycles);
        return ConcurrentStepResult{result, accountCycles(cycles)};
    }
    if (prefix.period_certificate && prefix.memoized_count != 0U) {
        const auto& cert = *prefix.period_certificate;
        if (cert.count == 0U) {
            idempotent_period_cache_.reset();
            throw std::logic_error("stale idempotent period certificate");
        }
        const std::size_t admitted_end_phase =
            (prefix.period_phase + prefix.memoized_count) % cert.count;
        if (memory_.restorationGeneration() != period_validation_restoration_generation_
            || memory_.restorationGeneration() != cert.restoration_generation
            || count > prefix.memoized_count
            || !cpu::bitwiseEqual(cpu_->state(), cert.states[admitted_end_phase])
            || have_last_fetch_ != cert.have_fetch[admitted_end_phase]
            || last_fetch_end_ != cert.fetch_end[admitted_end_phase]) {
            idempotent_period_cache_.reset();
            throw std::logic_error("stale idempotent period certificate");
        }
        const std::uint64_t cycles = prefix.cumulativeCycles(count - 1U);
        const std::size_t end_phase = (prefix.period_phase + count) % cert.count;
        cpu_->state() = cert.states[end_phase];
        have_last_fetch_ = cert.have_fetch[end_phase];
        last_fetch_end_ = cert.fetch_end[end_phase];
        auto footprint = prefix.entry_read_footprint;
        const std::size_t footprint_count = std::min<std::size_t>(count, cert.count);
        for (std::size_t n = 0U; n < footprint_count; ++n) {
            const auto& part = cert.footprints[(prefix.period_phase + n) % cert.count + 1U];
            for (std::size_t word = 0U; word < footprint.words.size(); ++word) {
                footprint.words[word] |= part.words[word];
            }
            footprint.complete = footprint.complete && part.complete;
        }
        if (count >= cert.count) {
            for (std::size_t word = 0U; word < footprint.words.size(); ++word) {
                footprint.words[word] |= cert.full_footprint.words[word];
            }
            footprint.complete = footprint.complete && cert.full_footprint.complete;
        }
        memory_.restoreReadFootprint(footprint);
        auto result = prefix.evaluated.result;
        const auto last = count - 1U;
        const auto last_phase = (prefix.period_phase + last) % cert.count;
        result.instructions = static_cast<std::uint32_t>(count);
        result.instruction_address = cert.pcs[last_phase];
        result.raw = cert.raw[last_phase];
        result.instruction_size = cert.sizes[last_phase];
        result.cycles = static_cast<std::uint16_t>(cycles);
        return ConcurrentStepResult{result, accountCycles(cycles)};
    }
    if (count == prefix.count) {
        // Admission already measured every executed instruction and its flash
        // cost. Stable guards above make an uncut commit O(1), not another
        // metadata copy and per-instruction timing walk.
        const auto last = count - 1U;
        last_fetch_end_ = prefix.evaluated.pcs[last] + prefix.evaluated.sizes[last];
        have_last_fetch_ = true;
        auto result = prefix.evaluated.result;
        const auto cycles = prefix.cumulative_cycles[last];
        result.cycles = static_cast<std::uint16_t>(cycles);
        return ConcurrentStepResult{result, accountCycles(cycles)};
    }

    if (current.mutation_sequence != prefix.memory_checkpoint.mutation_sequence
        && !memory_.restoreSideEffects(prefix.memory_checkpoint)) {
        throw std::logic_error("reversible RAM interruption cannot restore writes");
    }
    prefix.entry_state.restore(cpu_->state());
    memory_.restoreReadFootprint(prefix.entry_read_footprint);
    ReversibleRamPrefix::ReversibleExecution replayed;
    {
        ReversibleMemoryGuard guard(memory_);
        while (replayed.count < count) {
            const std::size_t request = std::min<std::size_t>(
                count - replayed.count, cpu::CortexM4::JitStepOutcome::max_block);
            auto chunk = cpu_->tryStepReversibleJitBlock(request);
            if (!chunk || !appendReversibleChunk(replayed, *chunk)) {
                throw std::logic_error("reversible RAM interrupted replay differs");
            }
            if (chunk->execution.result.reason != cpu::StopReason::step_complete
                && replayed.count < count) {
                throw std::logic_error("reversible RAM interrupted replay stopped early");
            }
        }
    }
    if (replayed.count != count) throw std::logic_error("reversible RAM replay count differs");

    bool have_fetch = prefix.entry_have_fetch;
    auto fetch_end = prefix.entry_fetch_end;
    std::uint64_t cycles = 0U;
    for (std::size_t i = 0U; i < count; ++i) {
        if (replayed.pcs[i] != prefix.evaluated.pcs[i]
            || replayed.sizes[i] != prefix.evaluated.sizes[i]
            || replayed.instruction_cycles[i] != prefix.evaluated.instruction_cycles[i]) {
            throw std::logic_error("reversible RAM replay metadata differs");
        }
        cycles += replayed.instruction_cycles[i];
        cycles += peripherals_->flash().fetchStallCycles(replayed.pcs[i],
            have_fetch && replayed.pcs[i] == fetch_end);
        fetch_end = replayed.pcs[i] + replayed.sizes[i];
        have_fetch = true;
    }
    if (cycles != prefix.cumulative_cycles[count - 1U]) {
        throw std::logic_error("reversible RAM materialization timing differs");
    }
    last_fetch_end_ = fetch_end;
    have_last_fetch_ = have_fetch;
    auto result = replayed.result;
    result.cycles = static_cast<std::uint16_t>(cycles);
    return ConcurrentStepResult{result, accountCycles(cycles)};
}

SimTimeNs Board::advanceTime(const std::uint64_t cycles) {
    const SimTimeNs elapsed = accountCycles(cycles);
    const auto events = owner_slice_active_
        ? event_loop_->runOwnedEvents(event_loop_->activeOwner(),
            saturatingAdd(event_loop_->now(), elapsed))
        : event_loop_->advanceBy(elapsed);
    if (events.same_time_limit_hit) {
        trace_->record(event_loop_->now(), config_.name, "event_livelock");
    }
    return elapsed;
}

Board::ConcurrentStepResult Board::beginConcurrentStep(
    const bool trace_instructions, const bool allow_jit, const std::size_t max_instructions,
    const bool block_prevalidated) {
    // General concurrent dispatch stays single-step: another lane can add a
    // new event while it is in flight. Only the synchronized world burst can
    // supply a larger, prevalidated pure fixed-cost block limit.
    auto result = stepWithFetchTiming(!trace_instructions && allow_jit,
        std::nullopt, max_instructions, block_prevalidated && max_instructions > 1U);
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
    const auto& state = cpu_->state();
    if (state.pending_exc_return || state.pending_exception
        || system_->resetRequested() || peripherals_->resetRequested()) return true;
    return system_->hasTakablePending(state.primask, state.basepri, state.faultmask);
}

bool Board::blockBoundaryPending() const noexcept {
    const auto& state = cpu_->state();
    if (state.pending_exc_return || state.pending_exception) return true;
    if (system_->resetRequested() || peripherals_->resetRequested()) return true;
    // Cheap short-circuit first: the full takability scan runs only when
    // something is actually pending.
    return system_->hasEnabledPending()
        && system_->nextPending(state.primask, state.basepri, state.faultmask)
            .has_value();
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
        if (trace_->passesFilter("exception_return")) trace_->record(
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
        if (trace_->passesFilter("exception_enter")) trace_->record(
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
            if (trace_->passesFilter("exception_enter")) trace_->record(
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
    const std::uint64_t clock_hz = peripherals_->rcc().systemClockHz();
    const std::uint64_t flash_generation = peripherals_->flash().acrGeneration();
    if (observation.valid
        && observation.generation == loop_observation_generation_
        && observation.boundary_pc == boundary_pc
        && observation.clock_hz == clock_hz
        && observation.flash_acr_generation == flash_generation
        && cpu::bitwiseEqual(observation.state, cpu_->state())
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
            clock_hz,
            flash_generation,
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
    observation.clock_hz = clock_hz;
    observation.flash_acr_generation = flash_generation;
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
        && observation.clock_hz == peripherals_->rcc().systemClockHz()
        && observation.flash_acr_generation == peripherals_->flash().acrGeneration()
        && loop.clock_hz == peripherals_->rcc().systemClockHz()
        && loop.flash_acr_generation == peripherals_->flash().acrGeneration()
        && cpu::bitwiseEqual(cpu_->state(), observation.state)
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
    if (frequency == 0U) return 0U;
    if (cycles > (std::numeric_limits<std::uint64_t>::max() - time_fraction_)
                / nanoseconds_per_second) {
        return std::numeric_limits<SimTimeNs>::max();
    }
    return cycleTimeQuotient(cycles * nanoseconds_per_second + time_fraction_, frequency);
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

// Outlined: the generation-wrap path value-initializes a ~90 KiB
// observation table. Inlining it reserves that frame (and a chkstk probe)
// in every hot caller (step/accounting); the wrap itself is near-impossible.
#if defined(__clang__) || defined(__GNUC__)
__attribute__((noinline))
#endif
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
    observation.clock_hz = peripherals_->rcc().systemClockHz();
    observation.flash_acr_generation = peripherals_->flash().acrGeneration();
}

BoardRunResult Board::cpuFailure(const cpu::FastStepResult& result) const {
    cpu::RunResult detailed;
    detailed.reason = result.reason;
    detailed.instructions = result.instructions;
    detailed.cycles = result.cycles;
    detailed.diagnostic = cpu_->lastDiagnostic();
    return cpuFailure(detailed);
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

BoardRunResult Board::runOwnerSlice(const BoardRunOptions& options) {
    struct Scope {
        bool& active;
        bool previous;
        explicit Scope(bool& value) : active(value), previous(value) { active = true; }
        ~Scope() { active = previous; }
    } scope(owner_slice_active_);
    if (options.max_instructions != 0U) {
        if (auto boundary = settleInstructionBoundary()) {
            BoardRunResult result;
            result.reason = boundary->reason;
            result.message = std::move(boundary->message);
            result.time_ns = event_loop_->now();
            cpu_->captureDiagnostic(result.diagnostic);
            return result;
        }
    }
    return run(options);
}

void Board::prepareNativePrograms() {
#if defined(__x86_64__) && !defined(_WIN32)
    if (!cpu::MicroJit::hostSupported()) return;
    if (program_compiler_ && program_execution_generation_ == memory_.executionGeneration()) return;
    // Code ownership follows the executable image, rather than observed PCs.
    // A backing change discards every section and recompiles from live bytes.
    native_programs_ = {};
    native_program_count_ = 0U;
    std::string error;
    program_compiler_ = cpu::MicroJit::create(error);
    if (!program_compiler_) throw std::runtime_error(error);
    for (const auto& segment : image_.segments()) {
        if (!segment.executable || segment.file_size < 2U) continue;
        if (native_program_count_ == native_programs_.size())
            throw std::runtime_error("native-program: too many executable image segments");
        mem::MemoryBus::DirectReadWindow backing;
        if (!memory_.directReadWindow(segment.physical_address, backing))
            throw std::runtime_error("native-program: executable backing unavailable");
        const auto offset = segment.physical_address - backing.begin;
        const auto size = segment.file_size & ~1U;
        if (offset > backing.size || size > backing.size - offset)
            throw std::runtime_error("native-program: executable segment exceeds backing");
        auto program = program_compiler_->compileProgram(
            {backing.bytes + offset, size}, segment.physical_address, error);
        if (!program) throw std::runtime_error(error);
        native_programs_[native_program_count_++] = std::move(program);
    }
    cpu_->mutableJitStats().native_compilations += program_compiler_->compilations();
    program_execution_generation_ = memory_.executionGeneration();
#endif
}

struct Board::NativeServiceState {
    Board& board;
    std::optional<SimTimeNs> deadline;
    std::uint64_t frequency, flash_generation;
    std::optional<cpu::FastStepResult> stop;
    std::exception_ptr exception;
};

bool Board::nativeCheckedStep(cpu::NativeRegionContext* const context) noexcept {
    auto& service = *static_cast<NativeServiceState*>(context->checked_context);
    auto& board = service.board;
    try {
        if (context->retired_count >= context->instruction_limit
            || context->retired_cycles > context->cycle_limit) return false;
        std::uint16_t pipeline = 0U;
        if (!board.cpu_->peekPredictableCycles(pipeline)) return false;
        // FLASH_ACR can change during a checked store. The modeled LATENCY
        // field is three bits; reserve its worst post-write fetch charge before
        // any effects, and charge only the actual checked step afterwards.
        constexpr std::uint64_t maximum_flash_stall = 7U;
        if (pipeline + maximum_flash_stall > context->cycle_limit - context->retired_cycles)
            return false;
        // Admission keeps this prefix strictly before every owner event. Only
        // advance the already retired prefix here: events scheduled by this
        // checked instruction are drained after the native frame returns.
        const auto pending = context->retired_cycles - board.span_accounted_cycles_;
        if (pending != 0U) {
            static_cast<void>(board.advanceTime(pending));
            board.span_accounted_cycles_ = context->retired_cycles;
        }
        board.have_last_fetch_ = context->have_fetch;
        board.last_fetch_end_ = context->fetch_end;
        const auto step = board.stepSingleWithFetchTiming(false);
        context->retired_count += step.instructions;
        context->retired_cycles += step.cycles;
        context->have_fetch = board.have_last_fetch_;
        context->fetch_end = board.last_fetch_end_;
        if (step.reason != cpu::StopReason::step_complete) {
            service.stop = step;
            return false;
        }
        if (board.blockBoundaryPending()
            || board.memory_.executionGeneration() != board.program_execution_generation_
            || board.peripherals_->rcc().systemClockHz() != service.frequency
            || board.peripherals_->flash().acrGeneration() != service.flash_generation
            || board.memory_.readFootprintTracking() || board.memory_.writeJournalTracking()) return false;
        auto horizon = service.deadline;
        if (const auto event = board.event_loop_->nextScheduledTime(board.event_loop_->activeOwner());
            event && (!horizon || *event < *horizon)) horizon = event;
        const auto now = board.event_loop_->now();
        if (horizon && *horizon <= now) return false;
        const auto systick = board.system_->cyclesUntilSysTickInterrupt();
        if (systick && *systick == 0U) return false;
        auto credit = systick ? *systick - 1U : std::numeric_limits<std::uint64_t>::max();
        if (horizon && *horizon - now <= std::numeric_limits<std::uint64_t>::max() / service.frequency) {
            const auto numerator = (*horizon - now) * service.frequency - 1U;
            credit = std::min(credit, numerator >= board.time_fraction_
                ? (numerator - board.time_fraction_) / nanoseconds_per_second : 0U);
        }
        credit = std::min<std::uint64_t>(credit,
            std::numeric_limits<std::uint16_t>::max() - board.span_accounted_cycles_);
        // The checked step may have scheduled an event inside its own charge.
        // It is already retired, just as in checked execution. Leave its charge
        // pending for the scheduler, without running that event in native code.
        if (credit < step.cycles) return false;
        context->cycle_limit = board.span_accounted_cycles_ + credit;
        return true;
    } catch (...) {
        // Generated sections have no C++ unwind metadata. Carry exceptions back
        // through the ordinary native epilogue before rethrowing on the host.
        service.exception = std::current_exception();
        return false;
    }
}

std::optional<cpu::FastStepResult> Board::stepNativeLocal(
    const std::size_t max_instructions, const std::optional<SimTimeNs> deadline) {
    if (max_instructions < 2U || blockBoundaryPending()) return std::nullopt;
    prepareNativePrograms();
    const auto start = cpu_->state().r[15];
    cpu::NativeRegionFunction function = nullptr;
    for (std::size_t i = 0U; i < native_program_count_; ++i) {
        function = native_programs_[i]->entryFor(start);
        if (function != nullptr) break;
    }
    if (function == nullptr) return std::nullopt;
    const auto now = event_loop_->now();
    const auto frequency = peripherals_->rcc().systemClockHz();
    if (frequency == 0U || time_fraction_ >= frequency) return std::nullopt;
    auto horizon = deadline;
    if (const auto event = event_loop_->nextScheduledTime(event_loop_->activeOwner());
        event && (!horizon || *event < *horizon)) horizon = event;
    if (horizon && *horizon <= now) return std::nullopt;
    const auto systick = system_->cyclesUntilSysTickInterrupt();
    if (systick && *systick == 0U) return std::nullopt;
    auto credit = systick ? *systick - 1U : std::numeric_limits<std::uint64_t>::max();
    if (horizon && *horizon - now <= std::numeric_limits<std::uint64_t>::max() / frequency) {
        const auto numerator = (*horizon - now) * frequency - 1U;
        credit = std::min(credit, numerator >= time_fraction_
            ? (numerator - time_fraction_) / nanoseconds_per_second : 0U);
    }
    credit = std::min<std::uint64_t>(credit, std::numeric_limits<std::uint16_t>::max());
    if (credit == 0U) return std::nullopt;
    cpu::NativeRegionContext context;
    context.cpu_state = &cpu_->state();
    context.instruction_limit = static_cast<std::uint32_t>(
        std::min<std::size_t>(max_instructions, UINT32_MAX));
    context.cycle_limit = credit;
    context.have_fetch = have_last_fetch_;
    context.fetch_end = last_fetch_end_;
    const auto& flash = peripherals_->flash();
    context.sequential_fetch_stall = flash.fetchStallCycles(start, true);
    context.nonsequential_fetch_stall = flash.fetchStallCycles(start, false);
    static_cast<void>(memory_.directReadWindow(start, context.code_read));
    const auto sp = cpu_->state().activeSp();
    // The top of a descending stack may be one byte beyond its RAM mapping.
    const auto probe = sp == 0U ? 0U : sp - 1U;
    static_cast<void>(memory_.directReadWindow(probe, context.ram_read));
    static_cast<void>(memory_.directWriteWindow(probe, context.ram_write));
    NativeServiceState service{*this, deadline, frequency, flash.acrGeneration(), {}, {}};
    context.checked_context = &service;
    context.checked_step = &Board::nativeCheckedStep;
    function(&context);
    if (service.exception) std::rethrow_exception(service.exception);
    if (context.retired_count == 0U && !service.stop) return std::nullopt;
    have_last_fetch_ = context.have_fetch;
    last_fetch_end_ = context.fetch_end;
    cpu::FastStepResult result = service.stop.value_or(cpu::FastStepResult{});
    result.instructions = context.retired_count;
    result.cycles = static_cast<std::uint16_t>(context.retired_cycles);
    result.instruction_address = cpu_->state().instruction_address;
    const auto offset = result.instruction_address - context.code_read.begin;
    if (offset < context.code_read.size && context.code_read.size - offset >= 2U) {
        const auto* bytes = context.code_read.bytes + offset;
        const auto first = static_cast<std::uint16_t>(bytes[0] | (bytes[1] << 8U));
        result.instruction_size = cpu::is32BitThumbPrefix(first) ? 4U : 2U;
        result.raw = first;
        if (result.instruction_size == 4U && context.code_read.size - offset >= 4U)
            result.raw = (static_cast<std::uint32_t>(first) << 16U) | bytes[2] | (bytes[3] << 8U);
    }
    auto& stats = cpu_->mutableJitStats();
    ++stats.native_executions;
    stats.native_instructions += context.retired_count - context.checked_count;
    return result;
}

BoardRunResult Board::run(const BoardRunOptions& options) {
    // A standalone run owns the board and its event loop until it returns.
    // Hold one exclusive lease so every per-instruction clock/event query
    // avoids re-locking; other threads still block until the run finishes.
    const auto exclusive_execution = event_loop_->useExclusiveExecution();
    static_cast<void>(exclusive_execution);
    BoardRunResult aggregate;
    aggregate.reason = BoardStopReason::instruction_budget;
    std::optional<std::uint32_t> proven_spin_pc;
    std::uint64_t proven_spin_instructions = 0U;
    peripherals_->setAdcDecimation(options.adc_decimation);
    const SimTimeNs deadline = options.duration_ns == 0U
        ? 0U : saturatingAdd(event_loop_->now(), options.duration_ns);

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
        const bool jit_ok = !options.trace_instructions && options.enable_jit;
        // Proven loop batching already amortizes idle loops; avoid competing
        // block preparation/preview overhead when that path is enabled.
        const std::size_t jit_limit = options.stop_address || options.detect_spin
            || options.enable_loop_batching
            ? 1U : static_cast<std::size_t>(std::min<std::uint64_t>(
                options.max_instructions - aggregate.instructions,
                owner_slice_active_ ? cpu::NativeTracePlan::max_ops
                                    : cpu::CortexM4::JitStepOutcome::max_block));
        const auto limit_time = deadline == 0U
            ? std::nullopt : std::optional<SimTimeNs>{deadline};
        span_accounted_cycles_ = 0U;
        const bool native_owner = owner_slice_active_ && jit_ok
            && !options.stop_address && !options.detect_spin && !options.enable_loop_batching;
        const auto native = native_owner
            ? stepNativeLocal(static_cast<std::size_t>(std::min<std::uint64_t>(
                options.max_instructions - aggregate.instructions, UINT32_MAX)), limit_time)
            : std::nullopt;
        auto result = native ? *native
            : stepWithFetchTiming(jit_ok && !native_owner, limit_time, owner_slice_active_ ? 1U : jit_limit);
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
        // Checked services may have settled a native prefix before MMIO.
        // Charge only the remainder after the native frame has returned.
        const auto preaccounted = std::exchange(span_accounted_cycles_, 0U);
        static_cast<void>(advanceTime(result.cycles > preaccounted
            ? result.cycles - preaccounted : 0U));
        if (result.reason != cpu::StopReason::step_complete) {
            BoardRunResult failure = cpuFailure(result);
            failure.instructions = aggregate.instructions;
            failure.cycles = aggregate.cycles;
            failure.memoized_prefixes = aggregate.memoized_prefixes;
            failure.memoized_instructions = aggregate.memoized_instructions;
            return failure;
        }

        if (auto boundary = settleInstructionBoundary()) {
            aggregate.reason = boundary->reason;
            aggregate.message = std::move(boundary->message);
            break;
        }

        // Proof work is skipped unless spin detection or usable batching
        // needs it: with tracing on, batching is disabled, and without
        // spin detection nothing consumes the proof.
        const bool need_proof = options.detect_spin
            || (options.enable_loop_batching && !options.trace_instructions);
        std::optional<ProvenLoop> loop;
        if (need_proof) {
            loop = observeLoopBoundary(
                result, aggregate.instructions, aggregate.cycles
            );
        }
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
    cpu_->captureDiagnostic(aggregate.diagnostic);
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
