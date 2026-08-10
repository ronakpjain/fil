#include "fil/sim/world.hpp"

#include "fil/devices/can_bus.hpp"
#include "fil/stm32g4/fdcan.hpp"
#include "fil/stm32g4/stm32g4.hpp"
#include "fil/sim/worker_pool.hpp"

#include <algorithm>
#include <limits>
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

struct ConcurrentEventGuard {
    EventLoop* loop{nullptr};
    ~ConcurrentEventGuard() {
        if (loop != nullptr) loop->setConcurrentAccess(false);
    }
};

std::uint64_t saturatingAdd(const std::uint64_t left, const std::uint64_t right) noexcept {
    if (right > std::numeric_limits<std::uint64_t>::max() - left) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return left + right;
}

void initializeSnapshot(WorldBoardRunResult& output, const Board& board, const SimTimeNs now) {
    output.name = board.config().name;
    output.result.reason = BoardStopReason::instruction_budget;
    output.result.time_ns = now;
    output.result.diagnostic.next_pc = board.cpu().state().r[15];
    output.result.diagnostic.registers = board.cpu().state().r;
    output.result.diagnostic.xpsr = board.cpu().state().xpsr;
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
    output.result.diagnostic.next_pc = board.cpu().state().r[15];
    output.result.diagnostic.registers = board.cpu().state().r;
    output.result.diagnostic.xpsr = board.cpu().state().xpsr;
    output.terminal = true;
}

void accumulate(
    WorldBoardRunResult& aggregate,
    const BoardRunResult& slice,
    WorldRunResult& world
) {
    aggregate.result.reason = slice.reason;
    aggregate.result.instructions = saturatingAdd(
        aggregate.result.instructions, slice.instructions
    );
    aggregate.result.cycles = saturatingAdd(aggregate.result.cycles, slice.cycles);
    aggregate.result.time_ns = slice.time_ns;
    aggregate.result.diagnostic = slice.diagnostic;
    aggregate.result.message = slice.message;
    world.instructions = saturatingAdd(world.instructions, slice.instructions);
    world.cycles = saturatingAdd(world.cycles, slice.cycles);
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
                    frame
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

Result<WorldRunResult> World::run(const WorldRunOptions& options) {
    if (options.instruction_quantum == 0U) {
        return argumentError("world instruction quantum must be nonzero");
    }
    if (options.detect_spin && options.spin_threshold == 0U) {
        return argumentError("world spin threshold must be nonzero when spin detection is enabled");
    }

    WorldRunResult output;
    output.start_time_ns = event_loop_.now();
    output.end_time_ns = output.start_time_ns;
    output.boards.resize(boards_.size());

    struct SchedulerState {
        bool runnable{true};
        bool in_flight{false};
        bool loop_skip_in_flight{false};
        SimTimeNs ready_time_ns{0};
        std::optional<Board::ConcurrentStepResult> step;
        std::optional<Board::ProvenLoop> proven_loop;
        bool inside_proven_loop{false};
        Board::LoopSkip loop_skip;
        std::uint64_t proven_loop_instructions{0};
        std::uint64_t same_time_dispatches{0};
    };
    std::vector<SchedulerState> states(boards_.size());
    std::vector<std::uint64_t> planned_iterations(boards_.size(), 0U);
    std::vector<Board::ConcurrentStepResult> burst_steps(boards_.size());
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
        states[index].ready_time_ns = output.start_time_ns;
        if (options.max_instructions_per_board == 0U) {
            states[index].runnable = false;
            stopBoard(
                output.boards[index], *boards_[index]->board,
                BoardStopReason::instruction_budget, "instruction budget exhausted",
                output.start_time_ns
            );
        }
    }

    const SimTimeNs deadline = options.duration_ns == 0U
        ? 0U : saturatingAdd(output.start_time_ns, options.duration_ns);
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

    for (;;) {
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
                board_output.result.diagnostic.next_pc = board.cpu().state().r[15];
                board_output.result.diagnostic.registers = board.cpu().state().r;
                board_output.result.diagnostic.xpsr = board.cpu().state().xpsr;
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

            if (completed.cpu_result.reason != cpu::StopReason::step_complete) {
                cpu::RunResult detailed;
                detailed.reason = completed.cpu_result.reason;
                detailed.instructions = completed.cpu_result.instructions;
                detailed.cycles = completed.cpu_result.cycles;
                detailed.diagnostic = board.cpu().lastDiagnostic();
                BoardRunResult slice = board.cpuFailure(detailed);
                slice.time_ns = now;
                accumulate(board_output, slice, output);
                state.runnable = false;
                board_output.terminal = true;
                state.proven_loop.reset();
                state.inside_proven_loop = false;
                if (!slice.succeeded()) {
                    board_failed = true;
                    stop_requested = stop_requested || options.stop_on_board_failure;
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
            output.instructions = saturatingAdd(
                output.instructions, completed.cpu_result.instructions
            );
            output.cycles = saturatingAdd(output.cycles, completed.cpu_result.cycles);

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
            if (board.cpu().state().r[15]
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

        bool burst_eligible = !options.enable_transactional_slices
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
                const SimTimeNs elapsed = boards_.front()->board->nextInstructionElapsedNs();
                if (elapsed == 0U
                    || (deadline != 0U && elapsed > deadline - round_start)) break;
                bool same_elapsed = true;
                for (std::size_t index = 1; index < boards_.size(); ++index) {
                    same_elapsed = same_elapsed
                        && boards_[index]->board->nextInstructionElapsedNs() == elapsed;
                }
                if (!same_elapsed) break;

                for (std::size_t index = 0; index < boards_.size(); ++index) {
                    auto owner_scope = event_loop_.useOwner(static_cast<EventOwner>(index));
                    burst_steps[index] =
                        boards_[index]->board->beginConcurrentStep(false);
                    ++output.dispatches;
                    ++output.exact_dispatches;
                }
                const SimTimeNs completion = saturatingAdd(round_start, elapsed);
                const auto events = event_loop_.runDueEvents(completion);
                output.event_callbacks = saturatingAdd(
                    output.event_callbacks, events.events_executed
                );

                if (events.events_executed != 0U) {
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
                    Board& board = *boards_[index]->board;
                    WorldBoardRunResult& board_output = output.boards[index];
                    const cpu::FastStepResult& step = burst_steps[index].cpu_result;
                    if (step.reason != cpu::StopReason::step_complete) {
                        cpu::RunResult detailed;
                        detailed.reason = step.reason;
                        detailed.instructions = step.instructions;
                        detailed.cycles = step.cycles;
                        detailed.diagnostic = board.cpu().lastDiagnostic();
                        BoardRunResult slice = board.cpuFailure(detailed);
                        slice.time_ns = completion;
                        accumulate(board_output, slice, output);
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
                    output.instructions = saturatingAdd(output.instructions, step.instructions);
                    output.cycles = saturatingAdd(output.cycles, step.cycles);
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
                    if (board.cpu().state().r[15] <= step.instruction_address) {
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
                        slice_deadline, false, true
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
                        accumulate(output.boards[index], slices[index], output);
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
            bool safe_loop_mode = options.enable_loop_batching
                && !options.trace_instructions && !options.detect_spin;
            bool saw_runnable = false;
            std::optional<SimTimeNs> safe_horizon;
            if (deadline != 0U) safe_horizon = deadline;
            if (const auto event_time = event_loop_.nextScheduledTime()) {
                if (!safe_horizon || *event_time < *safe_horizon) {
                    safe_horizon = *event_time;
                }
            }

            // A proven loop supplies conservative lookahead: until its next
            // serviceable interrupt or shared event, the lane cannot affect a
            // different board. Lanes need not land on the same loop boundary;
            // they only need to stay behind the earliest observable frontier.
            for (std::size_t index = 0; index < boards_.size() && safe_loop_mode; ++index) {
                SchedulerState& state = states[index];
                if (!state.runnable) continue;
                saw_runnable = true;
                Board& board = *boards_[index]->board;
                if (!state.inside_proven_loop || !state.proven_loop
                    || !board.loopHasNoMmioSince(*state.proven_loop)) {
                    safe_loop_mode = false;
                    break;
                }

                const SimTimeNs lane_boundary = state.in_flight
                    ? state.ready_time_ns : now;
                if (const auto observable = board.nextObservableTime(lane_boundary)) {
                    if (!safe_horizon || *observable < *safe_horizon) {
                        safe_horizon = *observable;
                    }
                }
            }
            safe_loop_mode = safe_loop_mode && saw_runnable
                && (!safe_horizon || *safe_horizon > now);

            if (safe_loop_mode) {
                for (std::size_t index = 0; index < boards_.size(); ++index) {
                    SchedulerState& state = states[index];
                    if (!state.runnable || state.in_flight || !state.proven_loop) continue;
                    Board& board = *boards_[index]->board;
                    if (!board.loopProofStillValid(*state.proven_loop)) continue;
                    const std::uint64_t remaining = options.max_instructions_per_board
                        - output.boards[index].result.instructions;
                    planned_iterations[index] = board.maximumLoopIterations(
                        *state.proven_loop, remaining, safe_horizon
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
                        output.instructions = saturatingAdd(
                            output.instructions, state.loop_skip.instructions
                        );
                        output.cycles = saturatingAdd(output.cycles, state.loop_skip.cycles);
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
                state.step = board.beginConcurrentStep(options.trace_instructions);
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
        const auto events = event_loop_.runDueEvents(next_completion);
        output.event_callbacks = saturatingAdd(output.event_callbacks, events.events_executed);
        if (events.same_time_limit_hit) {
            trace_.record(event_loop_.now(), config_.name, "event_livelock");
        }
        if (events.events_executed != 0U) {
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
