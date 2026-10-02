#include "fil/sim/world.hpp"

#include "fil/common/numeric.hpp"
#include "fil/devices/can_bus.hpp"
#include "fil/stm32g4/fdcan.hpp"
#include "fil/stm32g4/stm32g4.hpp"
#include "fil/sim/worker_pool.hpp"

#include <algorithm>
#include <bit>
#include <limits>
#include <stdexcept>
#include <utility>

namespace fil::sim {
namespace {

Error configError(std::string message, const std::filesystem::path& source = {}) {
    std::optional<SourceContext> context;
    if (!source.empty()) context = SourceContext{source, 0, 0};
    return Error{ErrorCategory::config, std::move(message), std::move(context)};
}

Error argumentError(std::string message) {
    return Error{ErrorCategory::invalid_argument, std::move(message), std::nullopt};
}

Error runtimeError(std::string message) {
    return Error{ErrorCategory::runtime, std::move(message), std::nullopt};
}

// Small networks use a fixed-width timestamp reduction. Padding with the
// inactive sentinel permits unrolled/vectorized selection without changing
// equal-time board ordering; the caller masks out inactive lanes.
std::pair<SimTimeNs, std::uint64_t> smallNetworkFrontier(const SimTimeNs* times) {
    SimTimeNs next = std::numeric_limits<SimTimeNs>::max();
    for (std::size_t i = 0U; i < 8U; ++i) next = std::min(next, times[i]);
    std::uint64_t mask = 0U;
    for (std::size_t i = 0U; i < 8U; ++i) {
        mask |= static_cast<std::uint64_t>(times[i] == next) << i;
    }
    return {next, mask};
}

struct ConcurrentEventGuard {
    EventLoop* loop{nullptr};
    ~ConcurrentEventGuard() {
        if (loop != nullptr) loop->setConcurrentAccess(false);
    }
};

void initializeSnapshot(WorldBoardRunResult& output, const Board& board, const SimTimeNs now) {
    output.name = board.config().name;
    output.result.reason = BoardStopReason::instruction_budget;
    output.result.time_ns = now;
    board.cpu().captureDiagnostic(output.result.diagnostic);
}

void stopBoard(
    WorldBoardRunResult& output,
    const Board& board,
    const BoardStopReason reason,
    std::string message,
    const SimTimeNs now
) {
    output.result.reason = reason;
    output.result.time_ns = now;
    output.result.message = std::move(message);
    board.cpu().captureDiagnostic(output.result.diagnostic);
    output.terminal = true;
}

void accumulate(
    WorldBoardRunResult& aggregate,
    const BoardRunResult& slice
) {
    aggregate.result.reason = slice.reason;
    aggregate.result.instructions = saturatingAdd(
        aggregate.result.instructions, slice.instructions
    );
    aggregate.result.cycles = saturatingAdd(aggregate.result.cycles, slice.cycles);
    aggregate.result.time_ns = slice.time_ns;
    aggregate.result.diagnostic = slice.diagnostic;
    aggregate.result.message = slice.message;
}

} // namespace

struct World::BusEntry {
    std::string name;
    std::unique_ptr<devices::VirtualCanBus> bus;
};

struct World::BoardEntry {
    std::string name;
    std::unique_ptr<Board> board;
};

bool WorldRunResult::succeeded() const noexcept {
    return reason != WorldStopReason::board_failure;
}

World::World(config::NetworkConfig config) : config_(std::move(config)) {}

World::~World() = default;

Result<std::unique_ptr<World>> World::load(
    const config::NetworkConfig& config,
    const bool strict_mmio
) {
    auto world = std::unique_ptr<World>(new World(config));
    auto initialized = world->initialize(strict_mmio);
    if (!initialized) return initialized.error();
    return world;
}

Result<void> World::initialize(const bool strict_mmio) {
    if (config_.schema_version != config::current_schema_version) {
        return configError("unsupported network schema version", config_.source_path);
    }
    if (config_.name.empty()) {
        return configError("network name is empty", config_.source_path);
    }
    if (config_.board_paths.empty()) {
        return configError("network must contain at least one board", config_.source_path);
    }

    buses_.reserve(config_.buses.size());
    for (const config::CanBusConfig& bus_config : config_.buses) {
        if (bus_config.name.empty()) {
            return configError("CAN bus name is empty", config_.source_path);
        }
        if (bus_config.bitrate == 0U) {
            return configError(
                "CAN bus '" + bus_config.name + "' has a zero bitrate", config_.source_path
            );
        }
        if (canBus(bus_config.name) != nullptr) {
            return configError(
                "duplicate CAN bus name '" + bus_config.name + "'", config_.source_path
            );
        }

        auto entry = std::make_unique<BusEntry>();
        entry->name = bus_config.name;
        entry->bus = std::make_unique<devices::VirtualCanBus>(
            bus_config.name, bus_config.bitrate
        );
        entry->bus->setTraceCallback(
            [this, bus_name = bus_config.name](const devices::CanTraceRecord& record) {
                CanTraceFrame frame;
                frame.id = record.frame.id;
                frame.extended = record.frame.extended;
                frame.fd = record.frame.fd;
                frame.brs = record.frame.brs;
                frame.dlc = record.frame.dlc;
                const std::size_t length = devices::dlcToLength(record.frame.dlc);
                frame.data.assign(record.frame.data.begin(), record.frame.data.begin() + length);
                static_cast<void>(trace_.recordCanFrame(
                    record.time_ns,
                    bus_name + "/" + record.node,
                    record.direction == devices::CanTraceRecord::Direction::transmit,
                    frame,
                    bus_name,
                    record.node
                ));
            }
        );
        buses_.push_back(std::move(entry));
    }

    boards_.reserve(config_.board_paths.size());
    for (const std::filesystem::path& path : config_.board_paths) {
        auto board_config = config::loadBoardConfig(path);
        if (!board_config) {
            Error error = board_config.error();
            error.message = "cannot load network board '" + path.string() + "': " + error.message;
            return error;
        }
        if (board_config.value().name.empty()) {
            return configError("board name is empty", path);
        }
        if (board(board_config.value().name) != nullptr) {
            return configError(
                "duplicate board name '" + board_config.value().name + "'", path
            );
        }

        for (std::size_t index = 0; index < board_config.value().can.size(); ++index) {
            const config::CanControllerConfig& attachment = board_config.value().can[index];
            if (canBus(attachment.bus) == nullptr) {
                return configError(
                    "board '" + board_config.value().name + "' attaches "
                    + attachment.instance + " to undeclared CAN bus '" + attachment.bus + "'",
                    path
                );
            }
            const auto duplicate = std::find_if(
                board_config.value().can.begin(),
                board_config.value().can.begin() + static_cast<std::ptrdiff_t>(index),
                [&](const config::CanControllerConfig& prior) {
                    return prior.instance == attachment.instance;
                }
            );
            if (duplicate != board_config.value().can.begin()
                + static_cast<std::ptrdiff_t>(index)) {
                return configError(
                    "board '" + board_config.value().name
                    + "' attaches FDCAN instance '" + attachment.instance + "' more than once",
                    path
                );
            }
        }

        auto owner_scope = event_loop_.useOwner(
            static_cast<EventOwner>(boards_.size())
        );
        auto loaded = Board::load(board_config.value(), strict_mmio, &event_loop_, &trace_);
        if (!loaded) {
            Error error = loaded.error();
            error.message = "cannot initialize network board '" + board_config.value().name
                + "': " + error.message;
            return error;
        }

        auto entry = std::make_unique<BoardEntry>();
        entry->name = board_config.value().name;
        entry->board = std::move(loaded).value();
        Board* loaded_board = entry->board.get();
        boards_.push_back(std::move(entry));

        for (const config::CanControllerConfig& attachment : board_config.value().can) {
            devices::VirtualCanBus* bus = canBus(attachment.bus);
            stm32g4::FdcanPeripheral* controller =
                loaded_board->peripherals().fdcan(attachment.instance);
            if (controller == nullptr) {
                return configError(
                    "board '" + board_config.value().name + "' names unknown FDCAN instance '"
                    + attachment.instance + "'",
                    path
                );
            }
            auto attached = controller->attachBus(
                *bus,
                board_config.value().name + "." + attachment.instance,
                attachment.loopback
            );
            if (!attached) {
                Error error = attached.error();
                error.message = "cannot attach board '" + board_config.value().name + "' "
                    + attachment.instance + " to bus '" + attachment.bus + "': " + error.message;
                return error;
            }
        }
    }
    return {};
}

Result<WorldRunResult> World::run(const WorldRunOptions& requested_options) {
    // Run configuration is immutable for this invocation, including across
    // peripheral callbacks. Keep a non-aliased snapshot for the hot scheduler.
    const WorldRunOptions options = [&] {
        auto effective = requested_options;
        // Fast defaults must not turn diagnostic or alternative schedulers
        // into invalid combinations. Their exact paths retain full fidelity.
        effective.enable_ram_capsules = effective.enable_ram_capsules
            && effective.enable_jit && !effective.enable_loop_batching
            && !effective.enable_deferred_prefixes && !effective.trace_instructions
            && !effective.detect_spin && !effective.enable_transactional_slices;
        return effective;
    }();
    if (options.instruction_quantum == 0U) {
        return argumentError("world instruction quantum must be nonzero");
    }
    if (options.detect_spin && options.spin_threshold == 0U) {
        return argumentError("world spin threshold must be nonzero when spin detection is enabled");
    }

    if ((options.enable_deferred_prefixes || options.enable_ram_capsules)
        && (!options.enable_jit || options.trace_instructions || options.detect_spin
            || options.enable_transactional_slices)) {
        return argumentError("deferred prefixes require JIT without instruction tracing, spin detection, or transactional slices");
    }
    const bool deferred_enabled = options.enable_deferred_prefixes || options.enable_ram_capsules;

    WorldRunResult output;
    output.start_time_ns = event_loop_.now();
    output.end_time_ns = output.start_time_ns;
    output.boards.resize(boards_.size());

    const bool exact_single_fast_path = boards_.size() <= 64U && !options.enable_loop_batching
        && !options.detect_spin && !options.trace_instructions
        && !options.enable_transactional_slices && !deferred_enabled;
    struct SchedulerState {
        bool runnable{true};
        bool in_flight{false};
        bool loop_skip_in_flight{false};
        SimTimeNs ready_time_ns{0};
        std::optional<Board::ConcurrentStepResult> step;
        std::optional<Board::DeferredPurePrefix> deferred;
        std::optional<Board::ReversibleRamPrefix> ram;
        bool ram_active{false};
        std::optional<Board::ProvenLoop> proven_loop;
        bool inside_proven_loop{false};
        Board::LoopSkip loop_skip;
        std::uint64_t proven_loop_instructions{0};
        std::uint64_t same_time_dispatches{0};
    };
    std::vector<SchedulerState> states;
    if (!exact_single_fast_path) states.resize(boards_.size());
    std::vector<Board*> lane_boards;
    lane_boards.reserve(boards_.size());
    for (const auto& entry : boards_) lane_boards.push_back(entry->board.get());
    std::vector<std::uint64_t> planned_iterations;
    std::vector<Board::ConcurrentStepResult> burst_steps;
    std::vector<std::size_t> burst_block_limits;
    std::vector<std::size_t> candidate_block_limits;
    std::vector<std::size_t> best_block_limits;
    std::vector<Board::PredictedBlockCosts> burst_block_costs;
    std::vector<std::uint8_t> burst_prefix_indices;
    if (!exact_single_fast_path) {
        planned_iterations.assign(boards_.size(), 0U);
        burst_steps.resize(boards_.size());
        burst_block_limits.assign(boards_.size(), 1U);
        candidate_block_limits.assign(boards_.size(), 1U);
        best_block_limits.assign(boards_.size(), 1U);
        burst_block_costs.resize(boards_.size());
        burst_prefix_indices.assign(boards_.size(), 0U);
    }
    std::unique_ptr<LaneWorkerPool> worker_pool;
    ConcurrentEventGuard concurrent_guard;
    if (options.enable_transactional_slices && boards_.size() > 1U
        && !trace_.enabled() && !options.trace_instructions && !options.detect_spin) {
        event_loop_.setConcurrentAccess(true);
        concurrent_guard.loop = &event_loop_;
        worker_pool = std::make_unique<LaneWorkerPool>(boards_.size());
    }
    for (std::size_t index = 0; index < boards_.size(); ++index) {
        initializeSnapshot(output.boards[index], *boards_[index]->board, output.start_time_ns);
        boards_[index]->board->cpu().setNativeSingleInstructionJitEnabled(
            options.enable_native_single_jit);
        boards_[index]->board->peripherals().setAdcDecimation(options.adc_decimation);
        if (!exact_single_fast_path) states[index].ready_time_ns = output.start_time_ns;
        if (options.max_instructions_per_board == 0U) {
            if (!exact_single_fast_path) states[index].runnable = false;
            stopBoard(
                output.boards[index], *boards_[index]->board,
                BoardStopReason::instruction_budget, "instruction budget exhausted",
                output.start_time_ns
            );
        }
    }

    const SimTimeNs deadline = options.duration_ns == 0U
        ? 0U : saturatingAdd(output.start_time_ns, options.duration_ns);
    std::size_t current_dispatch_index = states.size();
    std::uint64_t active_capsule_mask = 0U;
    struct DispatchScope {
        std::size_t& current;
        std::size_t previous;
        DispatchScope(std::size_t& context, const std::size_t index)
            : current(context), previous(context) { current = index; }
        ~DispatchScope() { current = previous; }
    };
    // Deferred instructions have not executed yet. An observation barrier
    // evaluates only instructions that exact dispatch would already have
    // started, including the active instruction when the observation falls
    // strictly between two completion timestamps. No future suffix is run.
    const auto materialize_deferred = [&](const SimTimeNs at, const bool all,
                                          const std::size_t dispatch_index,
                                          const std::optional<std::size_t> owner_only = std::nullopt) {
        const bool use_mask = states.size() <= 64U;
        auto pending = active_capsule_mask;
        std::size_t fallback_index = 0U;
        for (;;) {
            std::size_t index;
            if (use_mask) {
                if (pending == 0U) break;
                index = static_cast<std::size_t>(std::countr_zero(pending));
                pending &= pending - 1U;
            } else {
                if (fallback_index == states.size()) break;
                index = fallback_index++;
            }
            if (owner_only && index != *owner_only) continue;
            auto& state = states[index];
            if ((!use_mask && !state.deferred && !state.ram_active)
                || (!all && state.ready_time_ns > at)) continue;
            const auto& prefix = state.ram_active
                ? static_cast<const Board::DeferredPurePrefix&>(*state.ram) : *state.deferred;
            std::size_t count = 1U;
            while (count < prefix.count && prefix.completion_times_ns[count - 1U] < at) ++count;
            const SimTimeNs completion = prefix.completion_times_ns[count - 1U];
            if (count < prefix.count) ++output.deferred_truncations;
            auto owner = event_loop_.useOwner(static_cast<EventOwner>(index));
            state.step = state.ram_active
                ? lane_boards[index]->materializeReversibleRamPrefix(*state.ram, count)
                : lane_boards[index]->materializeDeferredPurePrefix(prefix, count);
            state.ready_time_ns = completion;
            output.deferred_instructions = saturatingAdd(output.deferred_instructions, count);
            output.exact_dispatches = saturatingAdd(output.exact_dispatches, count);
            state.deferred.reset();
            state.ram_active = false;
            if (use_mask) active_capsule_mask &= ~(std::uint64_t{1U} << index);
            // At a dispatch observation, completions at this timestamp were
            // already settled before any lane started work. Preserve that
            // phase ordering for an elided internal prefix boundary, too.
            if (dispatch_index < states.size() && completion == at) {
                const auto result = state.step->cpu_result;
                if (result.reason != cpu::StopReason::step_complete) {
                    throw std::logic_error("deferred pure-prefix materialization failed");
                }
                auto& board_output = output.boards[index];
                board_output.result.reason = BoardStopReason::instruction_budget;
                board_output.result.instructions = saturatingAdd(
                    board_output.result.instructions, result.instructions);
                board_output.result.cycles = saturatingAdd(board_output.result.cycles, result.cycles);
                board_output.result.time_ns = at;
                board_output.result.diagnostic.instruction_address = result.instruction_address;
                board_output.result.diagnostic.raw = result.raw;
                board_output.result.diagnostic.instruction_size = result.instruction_size;
                state.step.reset();
                state.in_flight = false;
                if (lane_boards[index]->settleInstructionBoundary()) {
                    throw std::logic_error("unexpected boundary in certified pure prefix");
                }
                if (board_output.result.instructions >= options.max_instructions_per_board) {
                    state.runnable = false;
                    stopBoard(board_output, *lane_boards[index], BoardStopReason::instruction_budget,
                              "instruction budget exhausted", at);
                } else if (index < dispatch_index) {
                    // This earlier lane's next start precedes the observing
                    // lane in deterministic board order. The certified suffix
                    // is private and side-effect-free outside its own RAM.
                    state.step = lane_boards[index]->beginConcurrentStep(false, true);
                    state.ready_time_ns = saturatingAdd(at, state.step->elapsed_ns);
                    state.in_flight = true;
                    ++state.same_time_dispatches;
                    ++output.dispatches;
                    ++output.exact_dispatches;
                }
            }
        }
    };
    struct EventBarrierGuard {
        EventLoop* loop;
        EventLoop::ObservationBarrier previous;
        ~EventBarrierGuard() {
            static_cast<void>(loop->exchangeSchedulerObservationBarrier(std::move(previous)));
        }
    };
    std::unique_ptr<EventBarrierGuard> event_barrier_guard;
    if (deferred_enabled) {
        event_barrier_guard = std::make_unique<EventBarrierGuard>();
        event_barrier_guard->loop = &event_loop_;
        auto previous = event_loop_.exchangeSchedulerObservationBarrier({});
        event_barrier_guard->previous = std::move(previous);
        static_cast<void>(event_loop_.exchangeSchedulerObservationBarrier(
            [&](const SimTimeNs at, const EventOwner owner,
                      const EventObservation observation) {
                if (observation == EventObservation::owner_local && owner < states.size()) {
                    materialize_deferred(at, true, states.size(), owner);
                } else {
                    materialize_deferred(at, true, states.size());
                }
            }));
    }
    struct DeliveryBarrierGuard {
        devices::VirtualCanBus* bus;
        devices::VirtualCanBus::DeliveryBarrier previous;
        ~DeliveryBarrierGuard() {
            static_cast<void>(bus->exchangeDeliveryBarrier(std::move(previous)));
        }
    };
    std::vector<std::unique_ptr<DeliveryBarrierGuard>> delivery_guards;
    if (deferred_enabled) {
        for (auto& entry : buses_) {
            auto previous = entry->bus->exchangeDeliveryBarrier({});
            auto guard = std::make_unique<DeliveryBarrierGuard>();
            guard->bus = entry->bus.get();
            guard->previous = previous;
            static_cast<void>(entry->bus->exchangeDeliveryBarrier(
                [&, previous = std::move(previous)](const std::uint64_t at) {
                    // FDCAN switches event ownership to shared before bus
                    // delivery; ownership is not the CPU dispatch phase.
                    materialize_deferred(at, true, current_dispatch_index);
                    if (previous) previous(at);
                }));
            delivery_guards.push_back(std::move(guard));
        }
    }
    static_cast<void>(trace_.record(
        output.start_time_ns,
        config_.name,
        "world_start",
        {
            {"boards", std::to_string(boards_.size())},
            {"quantum", std::to_string(options.instruction_quantum)},
        }
    ));
    const auto initial_events = event_loop_.runDueEvents(output.start_time_ns);
    output.event_callbacks = saturatingAdd(
        output.event_callbacks, initial_events.events_executed
    );
    if (initial_events.same_time_limit_hit) {
        trace_.record(event_loop_.now(), config_.name, "event_livelock");
    }

    bool time_exhausted = false;
    bool board_failed = false;
    bool stop_requested = false;
    SimTimeNs dispatch_time = output.start_time_ns;
    std::uint64_t transaction_backoff = 0U;

    if (exact_single_fast_path) {
        // This scheduler never consumes loop proofs or reversible epochs.
        // Avoid their RAM bookkeeping, not the modeled loads/stores. Scopes
        // restore the caller's tracking settings even if a callback throws.
        std::vector<std::unique_ptr<mem::MemoryBus::TrackingScope>> tracking_scopes;
        tracking_scopes.reserve(lane_boards.size());
        for (auto* board : lane_boards) {
            tracking_scopes.push_back(std::make_unique<mem::MemoryBus::TrackingScope>(
                board->memory(), false, false));
        }
        struct ExactLane {
            cpu::FastStepResult step{};
            cpu::FastStepResult retired{};
            SimTimeNs retired_time_ns{0};
            SimTimeNs dispatch_time_ns{0};
            std::uint64_t instructions{0};
            std::uint64_t cycles{0};
            std::uint64_t same_time_dispatches{0};
        };
        std::vector<ExactLane> lanes(boards_.size());
        // Frontier selection reads only contiguous timestamps, not the
        // larger lane/counter records. Inactive lanes use the maximum time.
        std::vector<SimTimeNs> completion_times(std::max(lanes.size(), std::size_t{8U}),
                                              std::numeric_limits<SimTimeNs>::max());
        const std::uint64_t all_lanes = lanes.size() == 64U
            ? std::numeric_limits<std::uint64_t>::max()
            : (std::uint64_t{1U} << lanes.size()) - 1U;
        std::uint64_t runnable = options.max_instructions_per_board == 0U ? 0U : all_lanes;
        std::uint64_t ready = runnable;
        std::uint64_t in_flight = 0U;
        std::uint64_t completing = 0U;
        SimTimeNs now = output.start_time_ns;
        for (auto& lane : lanes) lane.retired_time_ns = now;
        // Keep counters and the last retired instruction in the compact lane
        // array. Public result/diagnostic storage is only published at a stop
        // or on return; it is not an observation API while a run is active.
        const auto publish = [&](const std::size_t index) {
            const auto& lane = lanes[index];
            auto& result = output.boards[index].result;
            result.instructions = lane.instructions;
            result.cycles = lane.cycles;
            result.time_ns = lane.retired_time_ns;
            if (lane.instructions != 0U) {
                result.diagnostic.instruction_address = lane.retired.instruction_address;
                result.diagnostic.raw = lane.retired.raw;
                result.diagnostic.instruction_size = lane.retired.instruction_size;
            }
        };
        for (;;) {
            // Events precede completions, and every completion precedes every
            // start at the same timestamp. Low-bit traversal retains board order.
            auto pending = completing;
            while (pending != 0U) {
                const auto index = static_cast<std::size_t>(std::countr_zero(pending));
                const auto bit = std::uint64_t{1U} << index;
                pending &= pending - 1U;
                ExactLane& lane = lanes[index];
                Board& board = *lane_boards[index];
                auto& board_output = output.boards[index];
                in_flight &= ~bit;
                completion_times[index] = std::numeric_limits<SimTimeNs>::max();
                if (lane.step.reason != cpu::StopReason::step_complete) [[unlikely]] {
                    publish(index);
                    BoardRunResult failure = board.cpuFailure(lane.step);
                    failure.time_ns = now;
                    accumulate(board_output, failure);
                    runnable &= ~bit;
                    board_output.terminal = true;
                    if (!failure.succeeded()) {
                        board_failed = true;
                        stop_requested = stop_requested || options.stop_on_board_failure;
                    }
                    continue;
                }
                lane.instructions = saturatingAdd(lane.instructions, lane.step.instructions);
                lane.cycles = saturatingAdd(lane.cycles, lane.step.cycles);
                lane.retired_time_ns = now;
                lane.retired = lane.step;
                if (auto boundary = board.settleInstructionBoundary()) {
                    publish(index);
                    runnable &= ~bit;
                    stopBoard(board_output, board, boundary->reason,
                              std::move(boundary->message), now);
                    board_failed = true;
                    stop_requested = stop_requested || options.stop_on_board_failure;
                    continue;
                }
                if (deadline != 0U && now >= deadline) {
                    publish(index);
                    runnable &= ~bit;
                    stopBoard(board_output, board, BoardStopReason::time_budget,
                              "simulated-time budget exhausted", now);
                    time_exhausted = true;
                } else if (lane.instructions >= options.max_instructions_per_board) {
                    publish(index);
                    runnable &= ~bit;
                    stopBoard(board_output, board, BoardStopReason::instruction_budget,
                              "instruction budget exhausted", now);
                } else {
                    ready |= bit;
                }
            }

            if (deadline != 0U && now >= deadline) {
                time_exhausted = true;
                pending = ready;
                while (pending != 0U) {
                    const auto index = static_cast<std::size_t>(std::countr_zero(pending));
                    const auto bit = std::uint64_t{1U} << index;
                    pending &= pending - 1U;
                    publish(index);
                    runnable &= ~bit;
                    stopBoard(output.boards[index], *lane_boards[index],
                              BoardStopReason::time_budget,
                              "simulated-time budget exhausted", now);
                }
                ready = 0U;
            }

            bool dispatched = false;
            if (!stop_requested && !time_exhausted) {
                pending = ready;
                while (pending != 0U) {
                    const auto index = static_cast<std::size_t>(std::countr_zero(pending));
                    const auto bit = std::uint64_t{1U} << index;
                    pending &= pending - 1U;
                    ExactLane& lane = lanes[index];
                    if (lane.dispatch_time_ns != now) {
                        lane.dispatch_time_ns = now;
                        lane.same_time_dispatches = 0U;
                    }
                    if (lane.same_time_dispatches >= options.instruction_quantum) continue;
                    auto owner_scope = event_loop_.useOwner(static_cast<EventOwner>(index));
                    const auto step = lane_boards[index]->beginConcurrentStep(false, options.enable_jit);
                    lane.step = step.cpu_result;
                    completion_times[index] = saturatingAdd(now, step.elapsed_ns);
                    ready &= ~bit;
                    in_flight |= bit;
                    ++lane.same_time_dispatches;
                    ++output.dispatches;
                    ++output.exact_dispatches;
                    dispatched = true;
                }
                if (dispatched) ++output.rounds;
            }

            if (in_flight == 0U) {
                if (runnable == 0U || stop_requested || time_exhausted) break;
                for (auto& lane : lanes) lane.same_time_dispatches = 0U;
                completing = 0U;
                continue;
            }
            SimTimeNs next_completion = std::numeric_limits<SimTimeNs>::max();
            completing = 0U;
            if (lanes.size() <= 8U) {
                const auto frontier = smallNetworkFrontier(completion_times.data());
                next_completion = frontier.first;
                completing = frontier.second;
            } else {
                for (std::size_t index = 0U; index < completion_times.size(); ++index) {
                    const auto at = completion_times[index];
                    if (at <= next_completion) {
                        if (at < next_completion) completing = 0U;
                        next_completion = at;
                        completing |= std::uint64_t{1U} << index;
                    }
                }
            }
            completing &= in_flight;
            if (const auto next_event = event_loop_.nextScheduledTime();
                next_event && *next_event < next_completion) {
                next_completion = *next_event;
                completing = 0U;
            }
            const auto events = event_loop_.runDueEvents(next_completion);
            now = events.stopped_at;
            if (now != next_completion) completing = 0U;
            output.event_callbacks = saturatingAdd(output.event_callbacks, events.events_executed);
            if (events.same_time_limit_hit) {
                trace_.record(now, config_.name, "event_livelock");
            }
        }
        // Stopped lanes already have their full stop diagnostic, which must
        // not be overwritten by the last successfully retired instruction.
        for (std::size_t index = 0U; index < lanes.size(); ++index) {
            if ((runnable & (std::uint64_t{1U} << index)) != 0U) publish(index);
        }
    } else for (;;) {
        const SimTimeNs now = event_loop_.now();
        if (now != dispatch_time) {
            dispatch_time = now;
            for (SchedulerState& state : states) state.same_time_dispatches = 0;
        }

        // All events through `now` have fired. Complete every CPU instruction
        // ending at this frontier before allowing any board to start another.
        for (std::size_t index = 0; index < boards_.size(); ++index) {
            SchedulerState& state = states[index];
            if (!state.in_flight || state.ready_time_ns != now) continue;

            Board& board = *boards_[index]->board;
            WorldBoardRunResult& board_output = output.boards[index];
            if (state.loop_skip_in_flight) {
                state.loop_skip_in_flight = false;
                state.in_flight = false;
                board_output.result.time_ns = now;
                board.cpu().captureDiagnostic(board_output.result.diagnostic);
                if (state.inside_proven_loop && state.proven_loop) {
                    board.refreshLoopObservation(
                        *state.proven_loop,
                        board_output.result.instructions,
                        board_output.result.cycles
                    );
                }
                const std::uint32_t pc_before_settle = board.cpu().state().r[15];
                const std::uint16_t ipsr_before_settle = board.cpu().state().ipsr();
                if (auto boundary = board.settleInstructionBoundary()) {
                    state.runnable = false;
                    stopBoard(
                        board_output, board, boundary->reason, std::move(boundary->message), now
                    );
                    state.proven_loop.reset();
                    state.inside_proven_loop = false;
                    board_failed = true;
                    stop_requested = stop_requested || options.stop_on_board_failure;
                    continue;
                }
                if (board.cpu().state().r[15] != pc_before_settle
                    || board.cpu().state().ipsr() != ipsr_before_settle
                    || (state.proven_loop
                        && !board.loopProofStillValid(*state.proven_loop))) {
                    state.proven_loop.reset();
                    state.inside_proven_loop = false;
                }
                if (deadline != 0U && now >= deadline) {
                    state.runnable = false;
                    stopBoard(
                        board_output, board, BoardStopReason::time_budget,
                        "simulated-time budget exhausted", now
                    );
                    time_exhausted = true;
                } else if (board_output.result.instructions
                           >= options.max_instructions_per_board) {
                    state.runnable = false;
                    stopBoard(
                        board_output, board, BoardStopReason::instruction_budget,
                        "instruction budget exhausted", now
                    );
                }
                continue;
            }
            Board::ConcurrentStepResult completed = std::move(*state.step);
            state.step.reset();
            state.in_flight = false;

            if (completed.cpu_result.reason != cpu::StopReason::step_complete) [[unlikely]] {
                BoardRunResult slice = board.cpuFailure(completed.cpu_result);
                slice.time_ns = now;
                accumulate(board_output, slice);
                state.runnable = false;
                board_output.terminal = true;
                state.proven_loop.reset();
                state.inside_proven_loop = false;
                if (!slice.succeeded()) {
                    board_failed = true;
                    stop_requested = stop_requested || options.stop_on_board_failure;
                    if (deferred_enabled && stop_requested) {
                        materialize_deferred(now, true, states.size());
                    }
                }
                continue;
            }

            board_output.result.reason = BoardStopReason::instruction_budget;
            board_output.result.instructions = saturatingAdd(
                board_output.result.instructions, completed.cpu_result.instructions
            );
            board_output.result.cycles = saturatingAdd(
                board_output.result.cycles, completed.cpu_result.cycles
            );
            board_output.result.time_ns = now;
            board_output.result.diagnostic.instruction_address =
                completed.cpu_result.instruction_address;
            board_output.result.diagnostic.raw = completed.cpu_result.raw;
            board_output.result.diagnostic.instruction_size =
                completed.cpu_result.instruction_size;

            const std::uint32_t pc_before_settle = board.cpu().state().r[15];
            const std::uint16_t ipsr_before_settle = board.cpu().state().ipsr();
            if (auto boundary = board.settleInstructionBoundary()) {
                state.runnable = false;
                stopBoard(
                    board_output, board, boundary->reason, std::move(boundary->message), now
                );
                board_failed = true;
                state.proven_loop.reset();
                state.inside_proven_loop = false;
                stop_requested = stop_requested || options.stop_on_board_failure;
                if (deferred_enabled && stop_requested) {
                    materialize_deferred(now, true, states.size());
                }
                continue;
            }
            if (board.cpu().state().r[15] != pc_before_settle
                || board.cpu().state().ipsr() != ipsr_before_settle) {
                state.proven_loop.reset();
                state.inside_proven_loop = false;
            }
            if (state.inside_proven_loop && state.proven_loop
                && !board.loopHasNoMmioSince(*state.proven_loop)) {
                state.proven_loop.reset();
                state.inside_proven_loop = false;
            }

            std::optional<Board::ProvenLoop> observed_loop;
            // Keep this cheap gate at the hot call site. observeLoopBoundary()
            // repeats it as a defensive check for less frequent callers.
            // Proof work is skipped when neither batching nor spin
            // detection can consume it (also keeps no-batch lockstep
            // bursts firing instead of latching lanes "proven").
            if (((!deferred_enabled && options.enable_loop_batching) || options.detect_spin)
                && !completed.cpu_result.suppress_loop_observation
                && board.cpu().state().r[15]
                    <= completed.cpu_result.instruction_address) {
                observed_loop = board.observeLoopBoundary(
                    completed.cpu_result,
                    board_output.result.instructions,
                    board_output.result.cycles
                );
            }
            if (observed_loop) {
                const auto& loop = *observed_loop;
                if (state.proven_loop
                    && state.proven_loop->boundary_pc == loop.boundary_pc
                    && state.proven_loop->instructions_per_iteration
                        == loop.instructions_per_iteration) {
                    state.proven_loop_instructions += loop.instructions_per_iteration;
                } else {
                    state.proven_loop_instructions = loop.instructions_per_iteration;
                }
                state.proven_loop = loop;
                state.inside_proven_loop = true;
                if (options.detect_spin
                    && state.proven_loop_instructions >= options.spin_threshold) {
                    state.runnable = false;
                    stopBoard(
                        board_output, board, BoardStopReason::spin_detected,
                        "exact-state loop period_instructions="
                            + std::to_string(loop.instructions_per_iteration)
                            + " period_cycles=" + std::to_string(loop.cycles_per_iteration),
                        now
                    );
                    board_failed = true;
                    stop_requested = stop_requested || options.stop_on_board_failure;
                    continue;
                }
            } else if (state.inside_proven_loop && state.proven_loop
                       && board.cpu().state().r[15] == state.proven_loop->boundary_pc) {
                state.proven_loop.reset();
                state.inside_proven_loop = false;
            }

            if (deadline != 0U && now >= deadline) {
                state.runnable = false;
                stopBoard(
                    board_output, board, BoardStopReason::time_budget,
                    "simulated-time budget exhausted", now
                );
                time_exhausted = true;
                continue;
            }
            if (board_output.result.instructions >= options.max_instructions_per_board) {
                state.runnable = false;
                stopBoard(
                    board_output, board, BoardStopReason::instruction_budget,
                    "instruction budget exhausted", now
                );
            }
        }

        if (deadline != 0U && now >= deadline) {
            time_exhausted = true;
            for (std::size_t index = 0; index < boards_.size(); ++index) {
                if (!states[index].runnable || states[index].in_flight) continue;
                states[index].runnable = false;
                stopBoard(
                    output.boards[index], *boards_[index]->board,
                    BoardStopReason::time_budget, "simulated-time budget exhausted", now
                );
            }
        }

        bool burst_eligible = !deferred_enabled && !options.enable_transactional_slices
            && !options.trace_instructions && !options.detect_spin
            && !stop_requested && !time_exhausted && !states.empty();
        bool all_lanes_proven = !states.empty();
        for (std::size_t index = 0; index < states.size(); ++index) {
            burst_eligible = burst_eligible && states[index].runnable
                && !states[index].in_flight && states[index].ready_time_ns == now
                && output.boards[index].result.instructions + 64U
                    <= options.max_instructions_per_board;
            all_lanes_proven = all_lanes_proven
                && states[index].inside_proven_loop
                && states[index].proven_loop.has_value();
        }
        burst_eligible = burst_eligible && !all_lanes_proven;
        if (burst_eligible) {
            std::uint64_t completed_rounds = 0U;
            ++output.lockstep_bursts;
            for (; completed_rounds < 64U; ++completed_rounds) {
                const SimTimeNs round_start = event_loop_.now();
                std::fill(burst_block_limits.begin(), burst_block_limits.end(), 1U);
                SimTimeNs elapsed = 0U;
                bool synchronized_blocks = false;
                if (options.enable_jit) {
                    const auto block_deadline = deadline == 0U
                        ? std::nullopt : std::optional<SimTimeNs>{deadline};
                    for (std::size_t index = 0U; index < boards_.size(); ++index) {
                        const auto lane_remaining = options.max_instructions_per_board
                            - output.boards[index].result.instructions;
                        const auto limit = static_cast<std::size_t>(std::min<std::uint64_t>(
                            lane_remaining, cpu::CortexM4::JitStepOutcome::max_block));
                        burst_block_costs[index] = lane_boards[index]->peekPredictedBlockCosts(
                            limit, block_deadline);
                    }
                    std::uint64_t best_instruction_score = 0U;
                    SimTimeNs best_elapsed = 0U;
                    // Prefix numerators are strictly increasing. Intersect the
                    // per-lane frontiers with a multiway merge rather than
                    // rescanning every lane for every first-lane candidate.
                    const std::uint64_t common_frequency =
                        burst_block_costs.front().count == 0U ? 0U
                        : burst_block_costs.front().prefixes[0].cost.frequency;
                    bool all_lanes_have_costs = common_frequency != 0U;
                    for (const auto& lane_costs : burst_block_costs) {
                        all_lanes_have_costs = all_lanes_have_costs
                            && lane_costs.count != 0U
                            && lane_costs.prefixes[0].cost.frequency == common_frequency;
                    }
                    if (all_lanes_have_costs && !synchronized_blocks) {
                        std::fill(burst_prefix_indices.begin(),
                            burst_prefix_indices.end(), 0U);
                        while (true) {
                            std::uint64_t target_numerator = 0U;
                            for (std::size_t index = 0U; index < boards_.size(); ++index) {
                                const auto& prefix = burst_block_costs[index].prefixes[
                                    burst_prefix_indices[index]];
                                const std::uint64_t numerator = prefix.cost.cycles
                                    * 1'000'000'000ULL + prefix.cost.fraction;
                                target_numerator = std::max(target_numerator, numerator);
                            }
                            bool exhausted = false;
                            for (std::size_t index = 0U; index < boards_.size(); ++index) {
                                const auto& lane_costs = burst_block_costs[index];
                                auto& prefix_index = burst_prefix_indices[index];
                                while (prefix_index < lane_costs.count) {
                                    const auto& prefix = lane_costs.prefixes[prefix_index];
                                    const std::uint64_t numerator = prefix.cost.cycles
                                        * 1'000'000'000ULL + prefix.cost.fraction;
                                    if (numerator >= target_numerator) break;
                                    ++prefix_index;
                                }
                                if (prefix_index == lane_costs.count) {
                                    exhausted = true;
                                    break;
                                }
                            }
                            if (exhausted) break;

                            bool shared = true;
                            bool all_prefixes_pure = true;
                            std::uint64_t instruction_score = 0U;
                            bool has_multi_instruction_lane = false;
                            for (std::size_t index = 0U; index < boards_.size(); ++index) {
                                const auto& prefix = burst_block_costs[index].prefixes[
                                    burst_prefix_indices[index]];
                                const std::uint64_t numerator = prefix.cost.cycles
                                    * 1'000'000'000ULL + prefix.cost.fraction;
                                if (numerator != target_numerator) {
                                    shared = false;
                                    break;
                                }
                                candidate_block_limits[index] = prefix.instructions;
                                instruction_score += prefix.instructions;
                                has_multi_instruction_lane = has_multi_instruction_lane
                                    || prefix.instructions > 1U;
                                // A single-instruction prediction may be a
                                // memory/MMIO access. It can schedule an event
                                // after the shared horizon was inspected, so
                                // it must not share a frontier with a peer
                                // block that would advance past that event.
                                const auto pure_preview = lane_boards[index]->cpu()
                                    .peekJitBlock(prefix.instructions);
                                all_prefixes_pure = all_prefixes_pure && pure_preview
                                    && prefix.instructions
                                        <= pure_preview->exact_cycle_prefix_count;
                            }
                            if (shared && all_prefixes_pure && has_multi_instruction_lane) {
                                const SimTimeNs candidate_elapsed =
                                    target_numerator / common_frequency;
                                if (candidate_elapsed != 0U
                                    && (deadline == 0U
                                        || candidate_elapsed <= deadline - round_start)
                                    && (instruction_score > best_instruction_score
                                        || (instruction_score == best_instruction_score
                                            && candidate_elapsed > best_elapsed))) {
                                    best_instruction_score = instruction_score;
                                    best_elapsed = candidate_elapsed;
                                    std::copy(candidate_block_limits.begin(),
                                        candidate_block_limits.end(), best_block_limits.begin());
                                }
                            }
                            if (shared) {
                                bool exhausted = false;
                                for (std::size_t index = 0U; index < boards_.size(); ++index) {
                                    auto& prefix_index = burst_prefix_indices[index];
                                    ++prefix_index;
                                    exhausted = exhausted
                                        || prefix_index == burst_block_costs[index].count;
                                }
                                if (exhausted) break;
                            }
                        }
                    }
                    if (best_instruction_score != 0U) {
                        synchronized_blocks = true;
                        elapsed = best_elapsed;
                        std::copy(best_block_limits.begin(), best_block_limits.end(),
                            burst_block_limits.begin());
                    }
                }
                // A next-instruction forecast is not enough for lockstep:
                // executing that instruction may change RCC/flash timing or
                // schedule shared work. Fall back to the exact async scheduler
                // unless a common pure fixed-cost block was fully proven.
                if (!synchronized_blocks) break;

                for (std::size_t index = 0; index < boards_.size(); ++index) {
                    auto owner_scope = event_loop_.useOwner(static_cast<EventOwner>(index));
                    burst_steps[index] = lane_boards[index]->beginConcurrentStep(
                        false, options.enable_jit, burst_block_limits[index], true);
                    ++output.dispatches;
                    ++output.exact_dispatches;
                }
                // Multi-instruction blocks are admitted only when every lane has a pure,
                // fixed-cost prefix (which may be a single instruction on some lanes)
                // with the same exact completion time and no
                // intervening event/interrupt/deadline. No lane can create a
                // new MMIO event or start its next dispatch inside another's block.
                const SimTimeNs completion = saturatingAdd(round_start, elapsed);
                const auto events = event_loop_.runDueEvents(completion);
                output.event_callbacks = saturatingAdd(
                    output.event_callbacks, events.events_executed
                );

                if (events.events_executed != 0U) [[unlikely]] {
                    for (std::size_t index = 0; index < states.size(); ++index) {
                        const bool local_event = index < 64U
                            && (events.local_owner_mask
                                & (std::uint64_t{1U} << index)) != 0U;
                        if (!events.shared_event_executed && !local_event) continue;
                        states[index].proven_loop.reset();
                        states[index].inside_proven_loop = false;
                    }
                }

                bool leave_burst = events.events_executed != 0U;
                for (std::size_t index = 0; index < boards_.size(); ++index) {
                    Board& board = *lane_boards[index];
                    WorldBoardRunResult& board_output = output.boards[index];
                    const cpu::FastStepResult& step = burst_steps[index].cpu_result;
                    if (step.reason != cpu::StopReason::step_complete) [[unlikely]] {
                        BoardRunResult slice = board.cpuFailure(step);
                        slice.time_ns = completion;
                        accumulate(board_output, slice);
                        states[index].runnable = false;
                        board_output.terminal = true;
                        if (!slice.succeeded()) {
                            board_failed = true;
                            stop_requested = stop_requested
                                || options.stop_on_board_failure;
                        }
                        leave_burst = true;
                        continue;
                    }

                    board_output.result.reason = BoardStopReason::instruction_budget;
                    board_output.result.instructions = saturatingAdd(
                        board_output.result.instructions, step.instructions
                    );
                    board_output.result.cycles = saturatingAdd(
                        board_output.result.cycles, step.cycles
                    );
                    board_output.result.time_ns = completion;
                    board_output.result.diagnostic.instruction_address =
                        step.instruction_address;
                    board_output.result.diagnostic.raw = step.raw;
                    board_output.result.diagnostic.instruction_size = step.instruction_size;
                    states[index].ready_time_ns = completion;

                    const std::uint32_t pc_before_settle = board.cpu().state().r[15];
                    const std::uint16_t ipsr_before_settle = board.cpu().state().ipsr();
                    if (auto boundary = board.settleInstructionBoundary()) {
                        states[index].runnable = false;
                        stopBoard(
                            board_output, board, boundary->reason,
                            std::move(boundary->message), completion
                        );
                        board_failed = true;
                        stop_requested = stop_requested || options.stop_on_board_failure;
                        leave_burst = true;
                        continue;
                    }
                    if (board.cpu().state().r[15] != pc_before_settle
                        || board.cpu().state().ipsr() != ipsr_before_settle) {
                        states[index].proven_loop.reset();
                        states[index].inside_proven_loop = false;
                        leave_burst = true;
                    }

                    std::optional<Board::ProvenLoop> observed;
                    // Avoid entering the proof machinery for ordinary forward flow.
                    // Skipped entirely when batching is off so no-batch bursts
                    // keep firing instead of latching lanes "proven".
                    if (options.enable_loop_batching
                        && !step.suppress_loop_observation
                        && board.cpu().state().r[15] <= step.instruction_address) {
                        observed = board.observeLoopBoundary(
                            step, board_output.result.instructions,
                            board_output.result.cycles
                        );
                    }
                    if (observed && options.enable_loop_batching) {
                        const bool newly_proven = !states[index].inside_proven_loop
                            || !states[index].proven_loop;
                        states[index].proven_loop = *observed;
                        states[index].inside_proven_loop = true;
                        leave_burst = leave_burst || newly_proven;
                    }
                    if (deadline != 0U && completion >= deadline) {
                        states[index].runnable = false;
                        stopBoard(
                            board_output, board, BoardStopReason::time_budget,
                            "simulated-time budget exhausted", completion
                        );
                        time_exhausted = true;
                        leave_burst = true;
                    } else if (board_output.result.instructions
                               >= options.max_instructions_per_board) {
                        states[index].runnable = false;
                        stopBoard(
                            board_output, board, BoardStopReason::instruction_budget,
                            "instruction budget exhausted", completion
                        );
                        leave_burst = true;
                    }
                }
                ++output.rounds;
                if (leave_burst) {
                    ++completed_rounds;
                    break;
                }
            }
            if (completed_rounds != 0U) continue;
        }

        if (transaction_backoff != 0U) --transaction_backoff;
        if (options.enable_transactional_slices && transaction_backoff == 0U
            && !trace_.enabled() && !options.trace_instructions && !options.detect_spin
            && !stop_requested && !time_exhausted) {
            constexpr std::uint64_t slice_instructions = 256U;
            bool eligible = !states.empty();
            for (std::size_t index = 0; index < states.size() && eligible; ++index) {
                eligible = states[index].runnable && !states[index].in_flight
                    && states[index].ready_time_ns == now
                    && output.boards[index].result.instructions + slice_instructions
                        <= options.max_instructions_per_board;
            }

            std::optional<SimTimeNs> event_horizon = event_loop_.nextScheduledTime();
            SimTimeNs slice_deadline = deadline == 0U
                ? std::numeric_limits<SimTimeNs>::max() : deadline;
            if (event_horizon && *event_horizon <= slice_deadline) {
                slice_deadline = *event_horizon == 0U ? 0U : *event_horizon - 1U;
            }
            eligible = eligible && slice_deadline > now;

            if (eligible) {
                ++output.transactional_attempts;
                std::vector<Board::TransactionCheckpointPtr> checkpoints;
                std::vector<BoardRunResult> slices(boards_.size());
                checkpoints.reserve(boards_.size());
                for (std::size_t index = 0; index < boards_.size(); ++index) {
                    checkpoints.push_back(
                        boards_[index]->board->captureTransaction(
                            static_cast<EventOwner>(index)
                        )
                    );
                }
                const auto run_slice = [&](const std::size_t index) {
                    slices[index] = boards_[index]->board->runWorkerSlice(
                        static_cast<EventOwner>(index), slice_instructions,
                        slice_deadline, false, true, options.enable_jit
                    );
                };
                if (worker_pool) worker_pool->run(run_slice);
                else {
                    for (std::size_t index = 0; index < boards_.size(); ++index) {
                        run_slice(index);
                    }
                }

                const SimTimeNs committed_time = slices.front().time_ns;
                bool commit = committed_time > now && committed_time <= slice_deadline;
                if (event_horizon && committed_time >= *event_horizon) commit = false;
                for (const BoardRunResult& slice : slices) {
                    commit = commit
                        && slice.reason == BoardStopReason::instruction_budget
                        && slice.instructions == slice_instructions
                        && slice.time_ns == committed_time;
                }

                if (commit) {
                    const auto events = event_loop_.runDueEvents(committed_time);
                    output.event_callbacks = saturatingAdd(
                        output.event_callbacks, events.events_executed
                    );
                    for (std::size_t index = 0; index < boards_.size(); ++index) {
                        accumulate(output.boards[index], slices[index]);
                        states[index].ready_time_ns = committed_time;
                        states[index].proven_loop.reset();
                        states[index].inside_proven_loop = false;
                    }
                    output.dispatches = saturatingAdd(
                        output.dispatches, static_cast<std::uint64_t>(boards_.size())
                    );
                    output.exact_dispatches = saturatingAdd(
                        output.exact_dispatches,
                        slice_instructions * static_cast<std::uint64_t>(boards_.size())
                    );
                    output.transactional_instructions = saturatingAdd(
                        output.transactional_instructions,
                        slice_instructions * static_cast<std::uint64_t>(boards_.size())
                    );
                    ++output.transactional_commits;
                    ++output.rounds;
                    continue;
                }

                for (std::size_t index = 0; index < boards_.size(); ++index) {
                    if (!boards_[index]->board->restoreTransaction(checkpoints[index])) {
                        return runtimeError(
                            "transactional worker slice for board '"
                            + boards_[index]->name + "' escaped reversible state"
                        );
                    }
                }
                transaction_backoff = 4096U;
            }
        }

        bool dispatched = false;
        if (!stop_requested && !time_exhausted) {
            // A zero-duration clock configuration could otherwise let one board
            // monopolize a timestamp. The user quantum caps each same-time burst;
            // ordinary positive-duration instructions naturally yield every step.
            std::fill(planned_iterations.begin(), planned_iterations.end(), 0U);
            bool can_batch_all_lanes = !deferred_enabled && options.enable_loop_batching
                && !options.trace_instructions && !options.detect_spin;
            bool has_runnable_lane = false;
            std::optional<SimTimeNs> batch_horizon;
            if (deadline != 0U) batch_horizon = deadline;
            if (const auto event_time = event_loop_.nextScheduledTime()) {
                if (!batch_horizon || *event_time < *batch_horizon) {
                    batch_horizon = *event_time;
                }
            }

            // A proven loop supplies conservative lookahead: until its next
            // serviceable interrupt or shared event, the lane cannot affect a
            // different board. Lanes need not land on the same loop boundary;
            // they only need to stay behind the earliest observable frontier.
            for (std::size_t index = 0;
                 index < boards_.size() && can_batch_all_lanes;
                 ++index) {
                SchedulerState& state = states[index];
                if (!state.runnable) continue;
                has_runnable_lane = true;
                Board& board = *boards_[index]->board;
                if (!state.inside_proven_loop || !state.proven_loop
                    || !board.loopHasNoMmioSince(*state.proven_loop)) {
                    can_batch_all_lanes = false;
                    break;
                }

                const SimTimeNs lane_boundary = state.in_flight
                    ? state.ready_time_ns : now;
                if (const auto observable = board.nextObservableTime(lane_boundary)) {
                    if (!batch_horizon || *observable < *batch_horizon) {
                        batch_horizon = *observable;
                    }
                }
            }
            can_batch_all_lanes = can_batch_all_lanes && has_runnable_lane
                && (!batch_horizon || *batch_horizon > now);

            if (can_batch_all_lanes) {
                for (std::size_t index = 0; index < boards_.size(); ++index) {
                    SchedulerState& state = states[index];
                    if (!state.runnable || state.in_flight || !state.proven_loop) continue;
                    Board& board = *boards_[index]->board;
                    const std::uint64_t remaining = options.max_instructions_per_board
                        - output.boards[index].result.instructions;
                    planned_iterations[index] = board.maximumLoopIterations(
                        *state.proven_loop, remaining, batch_horizon
                    );
                }
            }

            for (std::size_t index = 0; index < boards_.size(); ++index) {
                SchedulerState& state = states[index];
                if (!state.runnable || state.in_flight) continue;
                if (state.same_time_dispatches >= options.instruction_quantum) {
                    continue;
                }

                Board& board = *boards_[index]->board;
                WorldBoardRunResult& board_output = output.boards[index];
                if (board_output.result.instructions >= options.max_instructions_per_board) {
                    state.runnable = false;
                    stopBoard(
                        board_output, board, BoardStopReason::instruction_budget,
                        "instruction budget exhausted", now
                    );
                    continue;
                }

                if (planned_iterations[index] != 0U && state.proven_loop) {
                    const std::uint64_t iterations = planned_iterations[index];
                    state.loop_skip = board.applyLoopIterations(
                        *state.proven_loop, iterations
                    );
                    if (state.loop_skip.instructions != 0U) {
                        board_output.result.instructions = saturatingAdd(
                            board_output.result.instructions, state.loop_skip.instructions
                        );
                        board_output.result.cycles = saturatingAdd(
                            board_output.result.cycles, state.loop_skip.cycles
                        );
                        state.ready_time_ns = saturatingAdd(now, state.loop_skip.elapsed_ns);
                        state.loop_skip_in_flight = true;
                        state.in_flight = true;
                        ++output.dispatches;
                        ++output.loop_batches;
                        output.batched_instructions = saturatingAdd(
                            output.batched_instructions, state.loop_skip.instructions
                        );
                        dispatched = true;
                        continue;
                    }
                }

                auto owner_scope = event_loop_.useOwner(static_cast<EventOwner>(index));
                if (deferred_enabled) {
                    const auto remaining = options.max_instructions_per_board
                        - output.boards[index].result.instructions;
                    const auto maximum = options.enable_ram_capsules
                        ? Board::ReversibleRamPrefix::max_instructions
                        : cpu::CortexM4::JitStepOutcome::max_block;
                    const auto limit = static_cast<std::size_t>(std::min<std::uint64_t>(remaining, maximum));
                    const auto lane_deadline = deadline == 0U
                        ? std::nullopt : std::optional<SimTimeNs>{deadline};
                    if (options.enable_ram_capsules) {
                        if (!state.ram) state.ram.emplace();
                        state.ram_active = board.prepareReversibleRamPrefix(*state.ram, limit, lane_deadline, true, true);
                    } else {
                        state.deferred = board.prepareDeferredPurePrefix(limit, lane_deadline);
                    }
                    if (state.ram_active && state.ram->count == 1U) {
                        // Keep an already evaluated singleton instead of undoing
                        // it and executing the same instruction a second time.
                        // It is an ordinary eager step, not a hidden future span.
                        state.step = board.materializeReversibleRamPrefix(*state.ram, 1U);
                        state.ready_time_ns = state.ram->completion_times_ns[0U];
                        state.ram_active = false;
                        state.in_flight = true;
                        ++output.dispatches;
                        ++output.exact_dispatches;
                        dispatched = true;
                        continue;
                    }
                    if (state.ram_active || state.deferred) {
                        const auto& prefix = state.ram_active
                            ? static_cast<const Board::DeferredPurePrefix&>(*state.ram) : *state.deferred;
                        state.ready_time_ns = prefix.completion_times_ns[prefix.count - 1U];
                        state.in_flight = true;
                        ++state.same_time_dispatches;
                        ++output.dispatches;
                        ++output.deferred_prefixes;
                        if (index < 64U) active_capsule_mask |= std::uint64_t{1U} << index;
                        dispatched = true;
                        continue;
                    }
                    // Board-local MMIO cannot observe peer CPU state. The
                    // CAN delivery hook below materializes peers before the
                    // only wired synchronous cross-board peripheral effect.
                }
                // A different lane may schedule a shared event while this
                // instruction is in flight. Keep general concurrent dispatch
                // atomic; only the lockstep path may admit pure multi-op
                // prefixes after proving every lane's common frontier.
                DispatchScope dispatch_scope(current_dispatch_index, index);
                state.step = board.beginConcurrentStep(
                    options.trace_instructions, options.enable_jit);
                state.ready_time_ns = saturatingAdd(now, state.step->elapsed_ns);
                state.in_flight = true;
                ++state.same_time_dispatches;
                ++output.dispatches;
                ++output.exact_dispatches;
                dispatched = true;
            }
            if (dispatched) ++output.rounds;

        }

        const bool any_in_flight = std::any_of(
            states.begin(), states.end(), [](const SchedulerState& state) {
                return state.in_flight;
            }
        );
        const bool any_runnable = std::any_of(
            states.begin(), states.end(), [](const SchedulerState& state) {
                return state.runnable;
            }
        );
        if (!any_in_flight) {
            if (!any_runnable || stop_requested || time_exhausted) break;
            // All runnable CPUs have zero-duration work and consumed their
            // same-time quantum. Begin the next deterministic fairness pass.
            for (SchedulerState& state : states) state.same_time_dispatches = 0;
            // Every runnable board was stopped while examining this frontier.
            continue;
        }

        SimTimeNs next_completion = std::numeric_limits<SimTimeNs>::max();
        for (const SchedulerState& state : states) {
            if (state.in_flight) next_completion = std::min(next_completion, state.ready_time_ns);
        }
        if (deferred_enabled) {
            const auto next_event = event_loop_.nextScheduledTime();
            if (next_event && *next_event <= next_completion) {
                next_completion = *next_event;
                // Each callback's guarded observation barrier materializes
                // its owner or all lanes immediately before it runs.
            } else {
                materialize_deferred(next_completion, false, states.size());
            }
        }
        const auto events = event_loop_.runDueEvents(next_completion);
        if (deferred_enabled) {
            // Event callbacks precede instruction completion at equal times.
            // Publish any remaining natural completions only after callbacks.
            materialize_deferred(events.stopped_at, false, states.size());
        }
        output.event_callbacks = saturatingAdd(output.event_callbacks, events.events_executed);
        if (events.same_time_limit_hit) {
            trace_.record(event_loop_.now(), config_.name, "event_livelock");
        }
        if (events.events_executed != 0U) [[unlikely]] {
            // Shared callbacks can affect every lane. Board-owned callbacks only
            // invalidate their originating lane; ownership is inherited by nested
            // peripheral scheduling and is a prerequisite for independent workers.
            for (std::size_t index = 0; index < states.size(); ++index) {
                const bool local_event = index < 64U
                    && (events.local_owner_mask & (std::uint64_t{1U} << index)) != 0U;
                if (!events.shared_event_executed && !local_event) continue;
                if (!events.shared_event_executed && local_event
                    && states[index].loop_skip_in_flight
                    && states[index].proven_loop) {
                    // Keep the prior proof only as a revalidation template. The
                    // completion path refreshes its observation at post-event
                    // memory state; maximumLoopIterations still rejects the stale
                    // checkpoint until one exact iteration proves it again.
                    continue;
                }
                states[index].proven_loop.reset();
                states[index].inside_proven_loop = false;
            }
        }
    }

    // Only per-lane totals affect scheduling. Summing once at the end is
    // equivalent to saturating after each nonnegative increment, including
    // loops and failure slices, and removes two hot-path updates per step.
    for (const auto& lane : output.boards) {
        output.instructions = saturatingAdd(output.instructions, lane.result.instructions);
        output.cycles = saturatingAdd(output.cycles, lane.result.cycles);
    }
    output.end_time_ns = event_loop_.now();
    if (board_failed) {
        output.reason = WorldStopReason::board_failure;
        output.message = "one or more boards stopped on an emulation failure";
    } else if (time_exhausted) {
        output.reason = WorldStopReason::time_budget;
        output.message = "shared simulated-time budget exhausted";
    } else {
        const bool only_instruction_budgets = std::all_of(
            output.boards.begin(), output.boards.end(), [](const WorldBoardRunResult& board_result) {
                return board_result.result.reason == BoardStopReason::instruction_budget;
            }
        );
        if (only_instruction_budgets) {
            output.reason = WorldStopReason::instruction_budget;
            output.message = "per-board instruction budgets exhausted";
        } else {
            output.reason = WorldStopReason::all_boards_stopped;
            output.message = "all boards reached terminal execution boundaries";
        }
    }

    static_cast<void>(trace_.record(
        output.end_time_ns,
        config_.name,
        "world_stop",
        {
            {"reason", std::string(worldStopReasonName(output.reason))},
            {"instructions", std::to_string(output.instructions)},
            {"cycles", std::to_string(output.cycles)},
        }
    ));
    return output;
}

void World::setDiagnosticsEnabled(const bool enabled) {
    trace_.setEnabled(enabled);
    for (auto& entry : boards_) {
        entry->board->peripherals().setAdcDiagnosticsEnabled(enabled);
    }
}

Board* World::board(const std::string_view name) noexcept {
    for (auto& entry : boards_) {
        if (entry->name == name) return entry->board.get();
    }
    return nullptr;
}

const Board* World::board(const std::string_view name) const noexcept {
    for (const auto& entry : boards_) {
        if (entry->name == name) return entry->board.get();
    }
    return nullptr;
}

devices::VirtualCanBus* World::canBus(const std::string_view name) noexcept {
    for (auto& entry : buses_) {
        if (entry->name == name) return entry->bus.get();
    }
    return nullptr;
}

const devices::VirtualCanBus* World::canBus(const std::string_view name) const noexcept {
    for (const auto& entry : buses_) {
        if (entry->name == name) return entry->bus.get();
    }
    return nullptr;
}

std::string_view worldStopReasonName(const WorldStopReason reason) noexcept {
    switch (reason) {
    case WorldStopReason::all_boards_stopped: return "all-boards-stopped";
    case WorldStopReason::instruction_budget: return "instruction-budget";
    case WorldStopReason::time_budget: return "time-budget";
    case WorldStopReason::board_failure: return "board-failure";
    }
    return "unknown";
}

} // namespace fil::sim
