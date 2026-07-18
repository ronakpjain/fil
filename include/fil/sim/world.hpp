#pragma once

/** @file world.hpp
 *  @brief Deterministic multi-board simulation and shared virtual CAN buses.
 */

#include "fil/common/result.hpp"
#include "fil/config/config.hpp"
#include "fil/sim/board.hpp"
#include "fil/sim/event_loop.hpp"
#include "fil/sim/trace.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace fil::devices {
class VirtualCanBus;
}

namespace fil::sim {

/** @brief Stable reason a multi-board run reached its boundary. */
enum class WorldStopReason : std::uint8_t {
    all_boards_stopped, ///< Every board reached a normal terminal CPU boundary.
    instruction_budget, ///< Every still-runnable board exhausted its instruction budget.
    time_budget,        ///< The shared simulated-time deadline was reached.
    board_failure,      ///< At least one board stopped on an emulation failure.
};

/** @brief Bounded deterministic scheduler options for one world run. */
struct WorldRunOptions {
    std::uint64_t max_instructions_per_board{50'000'000}; ///< Budget applied independently to each board.
    SimTimeNs duration_ns{1'000'000'000}; ///< Relative shared-time limit; zero disables it.
    std::uint64_t instruction_quantum{1'024}; ///< Fairness cap for CPU starts at one timestamp.
    bool trace_instructions{false}; ///< Emit instruction records into the shared trace.
    bool detect_spin{false}; ///< Stop a lane on a repeatedly proven exact-state loop.
    std::uint64_t spin_threshold{1'000'000}; ///< Logical loop instructions required before stopping.
    bool enable_loop_batching{true}; ///< Fast-forward jointly proven side-effect-free loops.
    bool enable_transactional_slices{false}; ///< Execute experimental reversible lane epochs.
    bool stop_on_board_failure{true}; ///< Stop immediately instead of finishing other boards.
};

/** @brief Accumulated outcome for one board in a world run. */
struct WorldBoardRunResult {
    std::string name; ///< Stable board name from its board configuration.
    BoardRunResult result; ///< Aggregate counters and latest board diagnostic.
    bool terminal{false}; ///< Whether this board itself reached a terminal boundary.
};

/** @brief Aggregate outcome of one deterministic virtual-time world run. */
struct WorldRunResult {
    WorldStopReason reason{WorldStopReason::all_boards_stopped};
    SimTimeNs start_time_ns{0}; ///< Shared clock at invocation.
    SimTimeNs end_time_ns{0}; ///< Shared clock when scheduling stopped.
    std::uint64_t instructions{0}; ///< Sum executed by all boards in this invocation.
    std::uint64_t cycles{0}; ///< Sum consumed by all boards in this invocation.
    std::uint64_t rounds{0}; ///< Virtual-time frontiers at which one or more CPUs started work.
    std::uint64_t dispatches{0}; ///< Total instruction and proven-loop batch dispatches.
    std::uint64_t exact_dispatches{0}; ///< Target instructions executed by the interpreter.
    std::uint64_t loop_batches{0}; ///< Proven-loop batches applied without interpretation.
    std::uint64_t batched_instructions{0}; ///< Logical instructions represented by loop batches.
    std::uint64_t event_callbacks{0}; ///< Shared event callbacks executed during the run.
    std::uint64_t transactional_attempts{0}; ///< Reversible multi-lane epochs attempted.
    std::uint64_t transactional_commits{0}; ///< MMIO-free epochs committed without rollback.
    std::uint64_t transactional_instructions{0}; ///< Instructions committed by lane epochs.
    std::uint64_t lockstep_bursts{0}; ///< Tight exact multi-board dispatch loops entered.
    std::vector<WorldBoardRunResult> boards; ///< Outcomes in network configuration order.
    std::string message;

    /** @brief Whether the world stopped at a requested, non-failure boundary. */
    [[nodiscard]] bool succeeded() const noexcept;
};

/**
 * @brief Multiple firmware boards sharing one event loop, trace, and CAN fabric.
 *
 * Each board advances on an independent CPU timeline. Boards ready at the same
 * virtual time are dispatched in network configuration order, then the shared
 * event loop advances to the earliest instruction, proven-loop batch, or event
 * frontier. Host execution speed and thread scheduling therefore cannot alter
 * simulation ordering.
 */
class World {
public:
    /** @brief Loads all referenced boards, constructs buses, and attaches FDCAN nodes. */
    [[nodiscard]] static Result<std::unique_ptr<World>> load(
        const config::NetworkConfig& config,
        bool strict_mmio = false
    );

    ~World();
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    /** @brief Runs boards concurrently on one deterministic virtual-time clock. */
    [[nodiscard]] Result<WorldRunResult> run(const WorldRunOptions& options);

    [[nodiscard]] const config::NetworkConfig& config() const noexcept { return config_; }
    [[nodiscard]] EventLoop& eventLoop() noexcept { return event_loop_; }
    [[nodiscard]] const EventLoop& eventLoop() const noexcept { return event_loop_; }
    [[nodiscard]] TraceRecorder& trace() noexcept { return trace_; }
    [[nodiscard]] const TraceRecorder& trace() const noexcept { return trace_; }
    [[nodiscard]] std::size_t boardCount() const noexcept { return boards_.size(); }
    [[nodiscard]] std::size_t canBusCount() const noexcept { return buses_.size(); }

    /** @brief Enables trace emission and optionally retains passive peripheral histories. */
    void setDiagnosticsEnabled(bool enabled, bool retain_passive_history = true);

    /** @brief Finds a board by its configuration name. */
    [[nodiscard]] Board* board(std::string_view name) noexcept;
    [[nodiscard]] const Board* board(std::string_view name) const noexcept;

    /** @brief Finds a named virtual CAN bus. */
    [[nodiscard]] devices::VirtualCanBus* canBus(std::string_view name) noexcept;
    [[nodiscard]] const devices::VirtualCanBus* canBus(std::string_view name) const noexcept;

private:
    struct BusEntry;
    struct BoardEntry;

    explicit World(config::NetworkConfig config);
    [[nodiscard]] Result<void> initialize(bool strict_mmio);

    config::NetworkConfig config_;
    EventLoop event_loop_;
    TraceRecorder trace_;
    // Buses must outlive boards because FDCAN destructors detach their nodes.
    std::vector<std::unique_ptr<BusEntry>> buses_;
    std::vector<std::unique_ptr<BoardEntry>> boards_;
};

/** @brief Stable lowercase stop-reason name for CLI and trace output. */
[[nodiscard]] std::string_view worldStopReasonName(WorldStopReason reason) noexcept;

} // namespace fil::sim
