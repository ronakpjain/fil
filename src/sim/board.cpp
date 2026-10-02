#include "fil/sim/board.hpp"

#include "fil/common/format.hpp"
#include "fil/common/numeric.hpp"
#include "fil/cortexm/exceptions.hpp"
#include "fil/cortexm/system_control.hpp"
#include "fil/mem/mcu_map.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace fil::sim {
namespace {

constexpr std::uint64_t nanoseconds_per_second = 1'000'000'000ULL;

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

struct Board::IdempotentPeriodCertificate {
    static constexpr std::size_t max_instructions = 32U;
    std::uint8_t count{0U};
    std::array<cpu::CpuState, max_instructions + 1U> states{};
    std::array<std::uint32_t, max_instructions> pcs{};
    std::array<std::uint8_t, max_instructions> sizes{};
    std::array<std::uint32_t, max_instructions> raw{};
    std::array<std::uint16_t, max_instructions> instruction_cycles{};
    std::array<std::uint64_t, max_instructions + 1U> cumulative_cycles{};
    std::array<mem::MemoryBus::ReadFootprint, max_instructions + 1U> footprints{};
    std::array<bool, max_instructions + 1U> have_fetch{};
    std::array<std::uint32_t, max_instructions + 1U> fetch_end{};
    mem::MemoryBus::ReadFootprint full_footprint{};
    mem::MemoryBus::SideEffectCheckpoint memory_checkpoint{};
    std::uint64_t restoration_generation{0U};
    std::uint64_t execution_generation{0U};
    std::uint64_t clock_hz{0U};
    std::uint64_t flash_acr_generation{0U};
};

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
    std::uint64_t cached_clock_hz{0};
    std::uint64_t cached_flash_acr_generation{0};
    std::uint32_t cached_flash_ws{0};
    bool cached_flash_art_hit_capable{false};
    std::uint32_t last_fetch_end{0};
    bool have_last_fetch{false};
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
    cpu_->setSystemControl(system_.get());
    if (!cpu_->reset(image_)) return runtimeError("CPU rejected validated reset vectors");
    exceptions_ = std::make_unique<cortexm::ExceptionController>(memory_, *system_);
    time_fraction_ = 0;
    loop_observations_ = {};
    loop_observation_generation_ = 1U;
    idempotent_period_cache_.reset();
    period_validation_checkpoint_ = {};
    period_validation_restoration_generation_ = memory_.restorationGeneration();
    read_footprint_boundary_.reset();
    static_cast<void>(memory_.takeReadFootprint());
    cached_clock_hz_ = peripherals_->rcc().systemClockHz();
    cached_flash_acr_generation_ = peripherals_->flash().acrGeneration();
    cached_flash_ws_ = peripherals_->flash().waitStates();
    cached_flash_art_hit_capable_ = peripherals_->flash().prefetchEnabled()
        || peripherals_->flash().instructionCacheEnabled();
    have_last_fetch_ = false;
    last_fetch_end_ = 0U;
    return {};
}

SimTimeNs Board::accountCycles(const std::uint64_t cycles) {
    system_->advanceCycles(cycles);
    // Cached-clock fast path: the RCC value changes only on firmware clock
    // writes, so the common case is one inline load plus one compare.
    // Small cycle counts convert divide-free via the remainder table
    // (remainder + carry < 2*F resolves with one compare); only large
    // loop-skip jumps pay a real division. Exact in both cases.
    const std::uint64_t frequency = peripherals_->rcc().systemClockHz();
    if (frequency != cached_clock_hz_) {
        cached_clock_hz_ = frequency;
        invalidateLoopObservations();
    }
    if (frequency == 0U) {
        time_fraction_ = 0U;
        return 0U;
    }
    if (cycle_table_hz_ != frequency) rebuildCycleTable(frequency);
    if (cycles < cycle_table_size) {
        const auto& entry = cycle_table_[static_cast<std::size_t>(cycles)];
        const std::uint64_t carried = entry.remainder + time_fraction_;
        if (carried >= frequency) {
            time_fraction_ = carried - frequency;
            return entry.quotient + 1U;
        }
        time_fraction_ = carried;
        return entry.quotient;
    }
    const std::uint64_t numerator = cycles * nanoseconds_per_second + time_fraction_;
    const SimTimeNs elapsed = numerator / frequency;
    time_fraction_ = numerator % frequency;
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
    const std::uint64_t generation = flash.acrGeneration();
    if (generation != cached_flash_acr_generation_) {
        cached_flash_acr_generation_ = generation;
        cached_flash_ws_ = flash.waitStates();
        cached_flash_art_hit_capable_ = flash.prefetchEnabled()
            || flash.instructionCacheEnabled();
        invalidateLoopObservations();
    }
    if (cached_flash_ws_ == 0U) {
        last_fetch_end_ = result.instruction_address + result.instruction_size;
        have_last_fetch_ = true;
        return result;
    }
    const bool sequential = have_last_fetch_
        && result.instruction_address == last_fetch_end_;
    last_fetch_end_ = result.instruction_address + result.instruction_size;
    have_last_fetch_ = true;
    if (sequential && cached_flash_art_hit_capable_) return result;
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
            // Memory-free blocks cannot change flash config mid-block: check
            // the ACR generation once and skip the loop when no wait states.
            const bool hoist_flash = jit->memory_free;
            if (hoist_flash) {
                const std::uint64_t generation = flash.acrGeneration();
                if (generation != cached_flash_acr_generation_) {
                    cached_flash_acr_generation_ = generation;
                    cached_flash_ws_ = flash.waitStates();
                    cached_flash_art_hit_capable_ = flash.prefetchEnabled()
                        || flash.instructionCacheEnabled();
                    invalidateLoopObservations();
                }
            }
            if (hoist_flash && cached_flash_ws_ == 0U) {
                const auto last = static_cast<std::uint8_t>(jit->count - 1U);
                last_fetch_end_ = jit->pcs[last] + jit->sizes[last];
                have_last_fetch_ = true;
            } else {
            for (std::uint8_t i = 0; i < jit->count; ++i) {
                const std::uint64_t generation = hoist_flash
                    ? cached_flash_acr_generation_ : flash.acrGeneration();
                if (generation != cached_flash_acr_generation_) {
                    cached_flash_acr_generation_ = generation;
                    cached_flash_ws_ = flash.waitStates();
                    cached_flash_art_hit_capable_ = flash.prefetchEnabled()
                        || flash.instructionCacheEnabled();
                    invalidateLoopObservations();
                }
                if (cached_flash_ws_ == 0U) {
                    last_fetch_end_ = jit->pcs[i] + jit->sizes[i];
                    have_last_fetch_ = true;
                    continue;
                }
                const bool sequential = have_last_fetch_
                    && jit->pcs[i] == last_fetch_end_;
                last_fetch_end_ = jit->pcs[i] + jit->sizes[i];
                have_last_fetch_ = true;
                if (sequential && cached_flash_art_hit_capable_) continue;
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
    struct Timing { const stm32g4::FlashPeripheral* flash; } timing{&flash};
    cpu::CortexM4::ReversibleCycleBudget budget{};
    budget.remaining_cycles = std::numeric_limits<std::uint64_t>::max();
    budget.context = &timing;
    budget.have_fetch = endpoint_have_fetch;
    budget.fetch_end = endpoint_fetch_end;
    budget.fetch_stall = +[](const void* context, const std::uint32_t pc, const bool sequential) {
        const auto& value = *static_cast<const Timing*>(context);
        if (value.flash->waitStates() == 0U) return std::uint16_t{0U};
        return static_cast<std::uint16_t>(value.flash->fetchStallCycles(pc, sequential));
    };
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
    idempotent_period_cache_ = certificate;
    period_validation_checkpoint_ = checkpoint;
    period_validation_restoration_generation_ = restoration_generation;
    return idempotent_period_cache_;
}

std::optional<Board::ReversibleRamPrefix> Board::prepareReversibleRamPrefix(
    const std::size_t max_instructions, const std::optional<SimTimeNs> deadline) {
    ReversibleRamPrefix prefix{};
    if (!prepareReversibleRamPrefix(prefix, max_instructions, deadline)) return std::nullopt;
    return prefix;
}

bool Board::prepareReversibleRamPrefix(ReversibleRamPrefix& prefix,
    const std::size_t max_instructions, const std::optional<SimTimeNs> deadline,
    const bool allow_single_prefix) {
    const std::size_t limit = std::min(max_instructions, ReversibleRamPrefix::max_instructions);
    if (limit < 2U || !memory_.writeJournalTracking() || blockBoundaryPending()) return false;
    const auto frequency = peripherals_->rcc().systemClockHz();
    if (frequency == 0U || time_fraction_ >= frequency) return false;
    const auto now = event_loop_->now();
    auto horizon = deadline;
    if (const auto event = event_loop_->nextScheduledTime();
        event && (!horizon || *event < *horizon)) horizon = *event;
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

    prefix.count = 0U;
    prefix.folded_instructions = 0U;
    prefix.period_certificate.reset();
    prefix.period_phase = 0U;
    prefix.memoized_count = 0U;
    // The fixed-capacity metadata arrays are live only below their count.
    // Avoid value-initializing all 64 entries on every admission; callers must
    // likewise consult only the admitted prefix.
    prefix.evaluated.count = 0U;
    prefix.evaluated.result = cpu::FastStepResult{};
    prefix.start_time_ns = now;
    prefix.entry_pc = cpu_->state().r[15];
    prefix.entry_state.capture(cpu_->state());
    prefix.clock_hz = frequency;
    prefix.time_fraction = time_fraction_;
    prefix.flash_generation = peripherals_->flash().acrGeneration();
    prefix.execution_generation = memory_.executionGeneration();
    prefix.memory_checkpoint = memory_.sideEffectCheckpoint();
    prefix.entry_read_footprint = memory_.readFootprint();
    prefix.entry_have_fetch = have_last_fetch_;
    prefix.entry_fetch_end = last_fetch_end_;
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
    const auto append_chunk = [](ReversibleRamPrefix::ReversibleExecution& aggregate,
                                 const cpu::CortexM4::TimedJitStepOutcome& chunk) {
        const auto n = chunk.execution.count;
        if (n == 0U || n > cpu::CortexM4::JitStepOutcome::max_block
            || static_cast<std::size_t>(aggregate.count) + n
                > ReversibleRamPrefix::max_instructions) return false;
        if (aggregate.count == 0U) aggregate.result = chunk.execution.result;
        else {
            aggregate.result.instruction_address = chunk.execution.result.instruction_address;
            aggregate.result.raw = chunk.execution.result.raw;
            aggregate.result.instruction_size = chunk.execution.result.instruction_size;
            aggregate.result.suppress_loop_observation = chunk.execution.result.suppress_loop_observation;
        }
        for (std::uint8_t i = 0U; i < n; ++i) {
            const std::size_t dst = aggregate.count++;
            aggregate.pcs[dst] = chunk.execution.pcs[i];
            aggregate.sizes[dst] = chunk.execution.sizes[i];
            aggregate.instruction_cycles[dst] = chunk.instruction_cycles[i];
        }
        aggregate.result.instructions = aggregate.count;
        aggregate.result.reason = cpu::StopReason::step_complete;
        return true;
    };

    // MMIO is trapped and every admitted store is non-executable RAM, so
    // clock/flash timing inputs cannot change during this speculative span.
    if (cycle_table_hz_ != frequency) rebuildCycleTable(frequency);
    const auto& flash = peripherals_->flash();
    struct FetchTiming {
        const stm32g4::FlashPeripheral* flash;
        std::uint16_t wait;
        bool art_hit;
    };
    const FetchTiming fetch_timing{&flash, static_cast<std::uint16_t>(flash.waitStates()),
        flash.prefetchEnabled() || flash.instructionCacheEnabled()};
    cpu::CortexM4::ReversibleCycleBudget budget;
    budget.remaining_cycles = cycle_credit;
    budget.context = &fetch_timing;
    budget.have_fetch = prefix.entry_have_fetch;
    budget.fetch_end = prefix.entry_fetch_end;
    budget.fetch_stall = +[](const void* context, const std::uint32_t pc, const bool sequential) {
        const auto& timing = *static_cast<const FetchTiming*>(context);
        if (timing.wait == 0U || (sequential && timing.art_hit)) return std::uint16_t{0U};
        return timing.flash->isFlashAddress(pc) ? timing.wait : std::uint16_t{0U};
    };
    const auto elapsed_for_span = [&](const std::uint64_t total_cycles) {
        if (total_cycles < cycle_table_size) {
            const auto& entry = cycle_table_[static_cast<std::size_t>(total_cycles)];
            return entry.quotient + static_cast<SimTimeNs>(
                entry.remainder + prefix.time_fraction >= frequency);
        }
        return (total_cycles * nanoseconds_per_second + prefix.time_fraction) / frequency;
    };
    // A certificate is reusable at every phase, provided no RAM input read by
    // the complete period has changed since its last successful validation.
    if (idempotent_period_cache_) {
        const auto& cert = *idempotent_period_cache_;
        bool valid = cert.count != 0U
            && memory_.restorationGeneration() == cert.restoration_generation
            && memory_.restorationGeneration() == period_validation_restoration_generation_
            && memory_.ramInputsCompatibleSince(period_validation_checkpoint_, cert.full_footprint)
            && memory_.executionGeneration() == cert.execution_generation
            && peripherals_->rcc().systemClockHz() == cert.clock_hz
            && peripherals_->flash().acrGeneration() == cert.flash_acr_generation;
        std::size_t phase = 0U;
        if (valid) {
            valid = false;
            for (std::size_t i = 0U; i < cert.count; ++i) {
                if (prefix.entry_pc == cert.pcs[i]
                    && cpu::bitwiseEqual(cpu_->state(), cert.states[i])
                    && have_last_fetch_ == cert.have_fetch[i]
                    && last_fetch_end_ == cert.fetch_end[i]) {
                    phase = i;
                    valid = true;
                    break;
                }
            }
        }
        if (!valid) {
            idempotent_period_cache_.reset();
        } else {
            std::uint64_t cycles = 0U;
            bool admitted = true;
            for (std::size_t n = 0U; n < limit; ++n) {
                const std::size_t index = (phase + n) % cert.count;
                const auto charged = cert.instruction_cycles[index];
                if (charged == 0U || charged > cycle_credit - std::min(cycle_credit, cycles)) {
                    admitted = false;
                    break;
                }
                cycles += charged;
                if (systick && cycles >= *systick) { admitted = false; break; }
                const SimTimeNs elapsed = elapsed_for_span(cycles);
                if (horizon && elapsed >= *horizon - now) { admitted = false; break; }
                const SimTimeNs completion = saturatingAdd(now, elapsed);
                if (completion <= now || (prefix.count != 0U
                    && completion <= prefix.completion_times_ns[prefix.count - 1U])) {
                    admitted = false;
                    break;
                }
                prefix.evaluated.pcs[n] = cert.pcs[index];
                prefix.evaluated.sizes[n] = cert.sizes[index];
                prefix.evaluated.instruction_cycles[n] = charged;
                prefix.cumulative_cycles[n] = cycles;
                prefix.completion_times_ns[n] = completion;
                prefix.count = static_cast<std::uint8_t>(n + 1U);
                prefix.evaluated.result = cpu::FastStepResult{};
                prefix.evaluated.result.reason = cpu::StopReason::step_complete;
                prefix.evaluated.result.instructions = prefix.count;
                prefix.evaluated.result.instruction_address = cert.pcs[index];
                prefix.evaluated.result.raw = cert.raw[index];
                prefix.evaluated.result.instruction_size = cert.sizes[index];
                prefix.evaluated.result.cycles = static_cast<std::uint16_t>(cycles);
            }
            if (!admitted || prefix.count < (allow_single_prefix ? 1U : 2U)) {
                prefix.count = 0U;
            } else {
                const std::size_t end_phase = (phase + prefix.count) % cert.count;
                cpu_->state() = cert.states[end_phase];
                have_last_fetch_ = cert.have_fetch[end_phase];
                last_fetch_end_ = cert.fetch_end[end_phase];
                prefix.period_certificate = idempotent_period_cache_;
                prefix.period_phase = static_cast<std::uint8_t>(phase);
                prefix.memoized_count = prefix.count;
                prefix.evaluated.count = prefix.count;
                prefix.memory_checkpoint = memory_.sideEffectCheckpoint();
                prefix.evaluated_checkpoint = prefix.memory_checkpoint;
                period_validation_checkpoint_ = prefix.memory_checkpoint;
                period_validation_restoration_generation_ = memory_.restorationGeneration();
                return true;
            }
        }
    }
    ReversibleRamPrefix::ReversibleExecution speculative;
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
            if (!append_chunk(speculative, *chunk)) break;
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
                && budget.have_fetch == prefix.entry_have_fetch
                && budget.fetch_end == prefix.entry_fetch_end
                && prefix.entry_state.matches(cpu_->state())
                && memory_.executionGeneration() == prefix.execution_generation
                && memory_.sideEffectCheckpoint().mutation_sequence
                    == prefix.memory_checkpoint.mutation_sequence
                && memory_.sideEffectsRestoredSince(prefix.memory_checkpoint)) {
                const std::uint64_t period_cycles = cycles;
                // Certification replays a whole period. Build the optional
                // cache only when this admission can immediately fold at least
                // one period; otherwise the extra execution cannot shorten
                // this capsule and a later admission may certify if useful.
                const bool can_fold = limit - prefix.count >= period
                    && period_cycles <= budget.remaining_cycles;
                if (can_fold && period <= IdempotentPeriodCertificate::max_instructions) {
                    prefix.period_certificate = buildIdempotentPeriodCertificate(
                        period, period_cycles);
                }
                while (limit - prefix.count >= period
                       && period_cycles <= budget.remaining_cycles) {
                    for (std::size_t i = 0U; i < period; ++i) {
                        const std::size_t dst = prefix.count++;
                        const auto charged = prefix.cumulative_cycles[i]
                            - (i == 0U ? 0U : prefix.cumulative_cycles[i - 1U]);
                        cycles += charged;
                        prefix.cumulative_cycles[dst] = cycles;
                        prefix.completion_times_ns[dst] = saturatingAdd(now, elapsed_for_span(cycles));
                        speculative.pcs[dst] = speculative.pcs[i];
                        speculative.sizes[dst] = speculative.sizes[i];
                        speculative.instruction_cycles[dst] = speculative.instruction_cycles[i];
                        ++prefix.folded_instructions;
                    }
                    budget.remaining_cycles -= period_cycles;
                }
                speculative.count = prefix.count;
                speculative.result.instructions = prefix.count;
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
        ReversibleRamPrefix::ReversibleExecution replayed{};
        ReversibleMemoryGuard guard(memory_);
        while (replayed.count < prefix.count) {
            const std::size_t request = std::min<std::size_t>(
                static_cast<std::size_t>(prefix.count - replayed.count),
                cpu::CortexM4::JitStepOutcome::max_block);
            auto chunk = cpu_->tryStepReversibleJitBlock(request);
            if (!chunk || chunk->execution.count == 0U || !append_chunk(replayed, *chunk)) {
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
    } else {
        prefix.evaluated.result = speculative.result;
        prefix.evaluated.count = speculative.count;
        for (std::size_t i = 0U; i < speculative.count; ++i) {
            prefix.evaluated.pcs[i] = speculative.pcs[i];
            prefix.evaluated.sizes[i] = speculative.sizes[i];
            prefix.evaluated.instruction_cycles[i] = speculative.instruction_cycles[i];
        }
    }
    prefix.evaluated_checkpoint = memory_.sideEffectCheckpoint();
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
        std::uint64_t cycles = prefix.cumulative_cycles[count - 1U];
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
        result.instructions = static_cast<std::uint8_t>(count);
        result.instruction_address = prefix.evaluated.pcs[last];
        result.raw = cert.raw[(prefix.period_phase + last) % cert.count];
        result.instruction_size = prefix.evaluated.sizes[last];
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
    ReversibleRamPrefix::ReversibleExecution replayed{};
    const auto append_chunk = [](ReversibleRamPrefix::ReversibleExecution& aggregate,
                                 const cpu::CortexM4::TimedJitStepOutcome& chunk) {
        const auto n = chunk.execution.count;
        if (n == 0U || n > cpu::CortexM4::JitStepOutcome::max_block
            || static_cast<std::size_t>(aggregate.count) + n
                > ReversibleRamPrefix::max_instructions) return false;
        if (aggregate.count == 0U) aggregate.result = chunk.execution.result;
        else {
            aggregate.result.instruction_address = chunk.execution.result.instruction_address;
            aggregate.result.raw = chunk.execution.result.raw;
            aggregate.result.instruction_size = chunk.execution.result.instruction_size;
            aggregate.result.suppress_loop_observation = chunk.execution.result.suppress_loop_observation;
        }
        for (std::uint8_t i = 0U; i < n; ++i) {
            const std::size_t dst = aggregate.count++;
            aggregate.pcs[dst] = chunk.execution.pcs[i];
            aggregate.sizes[dst] = chunk.execution.sizes[i];
            aggregate.instruction_cycles[dst] = chunk.instruction_cycles[i];
        }
        aggregate.result.instructions = aggregate.count;
        aggregate.result.reason = cpu::StopReason::step_complete;
        return true;
    };
    {
        ReversibleMemoryGuard guard(memory_);
        while (replayed.count < count) {
            const std::size_t request = std::min<std::size_t>(
                count - replayed.count, cpu::CortexM4::JitStepOutcome::max_block);
            auto chunk = cpu_->tryStepReversibleJitBlock(request);
            if (!chunk || !append_chunk(replayed, *chunk)) {
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
    const auto events = event_loop_->advanceBy(elapsed);
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
    if (cycle_table_hz_ != frequency) rebuildCycleTable(frequency);
    if (cycles < cycle_table_size) {
        const auto& entry = cycle_table_[static_cast<std::size_t>(cycles)];
        const std::uint64_t carried = entry.remainder + time_fraction_;
        return entry.quotient + (carried >= frequency ? 1U : 0U);
    }
    if (cycles > (std::numeric_limits<std::uint64_t>::max() - time_fraction_)
                / nanoseconds_per_second) {
        return std::numeric_limits<SimTimeNs>::max();
    }
    return (cycles * nanoseconds_per_second + time_fraction_) / frequency;
}

void Board::rebuildCycleTable(const std::uint64_t frequency) const {
    for (std::size_t c = 0U; c < cycle_table_size; ++c) {
        const std::uint64_t product = static_cast<std::uint64_t>(c)
            * nanoseconds_per_second;
        cycle_table_[c].quotient = frequency == 0U ? 0U : product / frequency;
        cycle_table_[c].remainder = frequency == 0U ? 0U : product % frequency;
    }
    cycle_table_hz_ = frequency;
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
    checkpoint->cached_clock_hz = cached_clock_hz_;
    checkpoint->cached_flash_acr_generation = cached_flash_acr_generation_;
    checkpoint->cached_flash_ws = cached_flash_ws_;
    checkpoint->cached_flash_art_hit_capable = cached_flash_art_hit_capable_;
    checkpoint->last_fetch_end = last_fetch_end_;
    checkpoint->have_last_fetch = have_last_fetch_;
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
    idempotent_period_cache_.reset();
    period_validation_checkpoint_ = memory_.sideEffectCheckpoint();
    period_validation_restoration_generation_ = memory_.restorationGeneration();
    exceptions_->restoreActiveStack(checkpoint->active_exceptions);
    time_fraction_ = checkpoint->time_fraction;
    loop_observations_ = checkpoint->loop_observations;
    loop_observation_generation_ = checkpoint->loop_observation_generation;
    read_footprint_boundary_ = checkpoint->read_footprint_boundary;
    memory_.restoreReadFootprint(checkpoint->read_footprint);
    cached_clock_hz_ = checkpoint->cached_clock_hz;
    cached_flash_acr_generation_ = checkpoint->cached_flash_acr_generation;
    cached_flash_ws_ = checkpoint->cached_flash_ws;
    cached_flash_art_hit_capable_ = checkpoint->cached_flash_art_hit_capable;
    last_fetch_end_ = checkpoint->last_fetch_end;
    have_last_fetch_ = checkpoint->have_last_fetch;
    return true;
}

BoardRunResult Board::runWorkerSlice(
    const EventOwner owner,
    const std::uint64_t instruction_budget,
    const SimTimeNs deadline_ns,
    const bool enable_loop_batching,
    const bool trap_all_mmio,
    const bool enable_jit
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
        cpu_->captureDiagnostic(aggregate.diagnostic);
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

        const cpu::FastStepResult result = stepWithFetchTiming(
            enable_jit, deadline_ns,
            enable_loop_batching ? 1U : static_cast<std::size_t>(std::min<std::uint64_t>(
                instruction_budget - aggregate.instructions,
                cpu::CortexM4::JitStepOutcome::max_block)));
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
        const SimTimeNs completion = saturatingAdd(
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
            BoardRunResult stopped = cpuFailure(result);
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

        // Proof work (memcmp of full CPU state, footprint, checkpoint) is
        // skipped when loop batching is off: nothing consumes the proof.
        std::optional<ProvenLoop> loop;
        if (enable_loop_batching) {
            loop = observeLoopBoundary(
                result, aggregate.instructions, aggregate.cycles
            );
        }
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
        const SimTimeNs skipped_completion = saturatingAdd(
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
                cpu::CortexM4::JitStepOutcome::max_block));
        auto result = stepWithFetchTiming(jit_ok,
            deadline == 0U ? std::nullopt : std::optional<SimTimeNs>{deadline}, jit_limit);
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
            BoardRunResult failure = cpuFailure(result);
            failure.instructions = aggregate.instructions;
            failure.cycles = aggregate.cycles;
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
