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
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace fil::devices {
class VirtualCanBus;
}

namespace fil::sim {
namespace detail { class LaneWorkers; }

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
    bool enable_jit{true}; ///< Native compilation; preserves lane boundaries.
    bool enable_ram_capsules{true}; ///< Guarded reversible RAM prefixes; incompatible modes fall back automatically.
    bool stop_on_board_failure{true}; ///< Stop on failure; parallel lanes finish their current window before stopping.
    std::uint32_t adc_decimation{1}; ///< Keep 1 of N continuous ADC scans on every board.
    SimTimeNs cross_board_window_ns{100'000}; ///< Bounded-skew owner slices; zero selects exact reference scheduling.
    bool enable_parallel_lanes{true}; ///< Eligible unobserved owner lanes run concurrently; false is a validation reference.
    /// Coordinator-only control at independent window boundaries, after trace
    /// merge and shared delivery. Return false to yield with a time-budget stop.
    /// Workers are parked; reentrant run/event dispatch is unsupported.
    std::function<bool(SimTimeNs)> window_control;
};

/** @brief Accumulated outcome for one board in a world run. */
struct WorldBoardRunResult {
    std::string name; ///< Stable board name from its board configuration.
    BoardRunResult result; ///< Aggregate counters and latest board diagnostic.
    bool terminal{false}; ///< Whether this board itself reached a terminal boundary.
    std::uint64_t memoized_prefixes{0}; ///< RAM-period certificate admissions for this board.
    std::uint64_t memoized_instructions{0}; ///< Logical instructions admitted by those certificates (before cuts).
};

/** @brief Aggregate outcome of one deterministic virtual-time world run. */
struct WorldRunResult {
    WorldStopReason reason{WorldStopReason::all_boards_stopped};
    SimTimeNs start_time_ns{0}; ///< Shared clock at invocation.
    SimTimeNs end_time_ns{0}; ///< Shared clock when scheduling stopped.
    std::uint64_t instructions{0}; ///< Sum executed by all boards in this invocation.
    std::uint64_t cycles{0}; ///< Sum consumed by all boards in this invocation.
    std::uint64_t rounds{0}; ///< Virtual-time frontiers at which one or more CPUs started work.
    std::size_t parallel_lanes{0}; ///< Concurrent owner executors used (zero for serial scheduling).
    std::uint64_t dispatches{0}; ///< Total instruction dispatches.
    std::uint64_t exact_dispatches{0}; ///< Target instructions executed by the interpreter.
    std::uint64_t event_callbacks{0}; ///< Shared event callbacks executed during the run.
    std::uint64_t lockstep_bursts{0}; ///< Tight exact multi-board dispatch loops entered.
    std::uint64_t deferred_prefixes{0}; ///< Certified pure prefixes admitted without eager execution.
    std::uint64_t deferred_instructions{0}; ///< Instructions evaluated by deferred materialization.
    std::uint64_t memoized_prefixes{0}; ///< Admissions served by a validated RAM-period certificate.
    std::uint64_t memoized_instructions{0}; ///< Logical instructions admitted by RAM-period certificates (before cuts).
    std::uint64_t deferred_truncations{0}; ///< Prefixes shortened by an observation barrier.
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
 * event loop advances to the earliest instruction or event frontier. Host execution speed and thread scheduling therefore cannot alter
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

    /**
     * @brief Runs independent board timelines on the deterministic serial scheduler.
     *
     * Holds the event loop's exclusive lease until return, including exception
     * unwinding. Mutex-protected event-loop calls on other threads wait for the
     * run to finish. Concurrent or reentrant World mutation is unsupported.
     */
    [[nodiscard]] Result<WorldRunResult> run(const WorldRunOptions& options);

    [[nodiscard]] const config::NetworkConfig& config() const noexcept { return config_; }
    [[nodiscard]] EventLoop& eventLoop() noexcept { return event_loop_; }
    [[nodiscard]] const EventLoop& eventLoop() const noexcept { return event_loop_; }
    [[nodiscard]] TraceRecorder& trace() noexcept { return trace_; }
    [[nodiscard]] const TraceRecorder& trace() const noexcept { return trace_; }
    [[nodiscard]] std::size_t boardCount() const noexcept { return boards_.size(); }
    [[nodiscard]] std::size_t canBusCount() const noexcept { return buses_.size(); }

    /** @brief Enables/disables shared trace history and passive ADC diagnostics. */
    void setDiagnosticsEnabled(bool enabled);

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
    [[nodiscard]] Result<WorldRunResult> runIndependent(const WorldRunOptions& options);

    config::NetworkConfig config_;
    EventLoop event_loop_;
    TraceRecorder trace_;
    // Buses must outlive boards because FDCAN destructors detach their nodes.
    std::vector<std::unique_ptr<BusEntry>> buses_;
    std::vector<std::unique_ptr<BoardEntry>> boards_;
    std::unique_ptr<detail::LaneWorkers> lane_workers_;
};

/** @brief Stable lowercase stop-reason name for CLI and trace output. */
[[nodiscard]] std::string_view worldStopReasonName(WorldStopReason reason) noexcept;

} // namespace fil::sim
