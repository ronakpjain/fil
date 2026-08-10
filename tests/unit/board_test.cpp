#include "fil/sim/board.hpp"
#include "../test_support.hpp"

#include <filesystem>
#include <sstream>
#include <vector>

namespace {

fil::config::BoardConfig fixtureBoard() {
    fil::config::BoardConfig config;
    config.name = "fixture";
    config.mcu_path = std::filesystem::path(FIL_SOURCE_DIR) / "configs/mcus/stm32g474retx.json";
    config.elf_path = std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";
    config.vector_base = 0x08000000U;
    return config;
}

void runsBoardToBreakpoint() {
    auto board = fil::sim::Board::load(fixtureBoard());
    fil::test::check(board.hasValue(), "loads integrated fixture board");
    if (!board) return;
    fil::sim::BoardRunOptions options;
    options.max_instructions = 20;
    options.duration_ns = 0;
    options.detect_spin = false;
    const auto result = board.value()->run(options);
    fil::test::check(result.reason == fil::sim::BoardStopReason::breakpoint, "runs fixture firmware to BKPT");
    fil::test::check(result.instructions == 3, "counts fixture instructions deterministically");
    fil::test::check(result.time_ns > 0, "advances simulated board time from CPU cycles");
}

void workerSliceMatchesStandaloneExecution() {
    auto exact = fil::sim::Board::load(fixtureBoard());
    auto worker = fil::sim::Board::load(fixtureBoard());
    fil::test::check(exact.hasValue() && worker.hasValue(),
                     "loads exact and owner-local worker boards");
    if (!exact || !worker) return;

    fil::sim::BoardRunOptions options;
    options.max_instructions = 20U;
    options.duration_ns = 0U;
    const auto exact_result = exact.value()->run(options);
    const auto worker_result = worker.value()->runWorkerSlice(0U, 20U, 1'000'000U);
    fil::test::check(worker_result.reason == exact_result.reason
                         && worker_result.instructions == exact_result.instructions
                         && worker_result.cycles == exact_result.cycles
                         && worker_result.time_ns == exact_result.time_ns
                         && worker_result.diagnostic.next_pc
                             == exact_result.diagnostic.next_pc,
                     "owner-local slice preserves standalone architectural results");
}

void restoresTransactionalBoardState() {
    auto board = fil::sim::Board::load(fixtureBoard());
    fil::test::check(board.hasValue(), "loads board for transaction rollback");
    if (!board) return;

    const std::uint32_t original_pc = board.value()->cpu().state().r[15];
    const auto original_ram = board.value()->memory().read32(0x20000000U);
    const auto checkpoint = board.value()->captureTransaction(0U);
    board.value()->cpu().state().r[0] = 0xdeadbeefU;
    static_cast<void>(board.value()->memory().write32(0x20000000U, 0x12345678U));
    static_cast<void>(board.value()->eventLoop().runOwnedEvents(0U, 100U));
    fil::test::check(board.value()->restoreTransaction(checkpoint),
                     "restores a slice with only CPU RAM and clock mutations");
    const auto restored_ram = board.value()->memory().read32(0x20000000U);
    fil::test::check(board.value()->cpu().state().r[0] == 0U
                         && board.value()->cpu().state().r[15] == original_pc
                         && original_ram && restored_ram
                         && restored_ram.value() == original_ram.value()
                         && board.value()->eventLoop().now(0U) == 0U,
                     "transaction rollback restores architectural and lane-clock state");
}

bool installIdleLoop(fil::sim::Board& board) {
    const std::uint32_t start = board.cpu().state().r[15] & ~1U;
    const std::vector<std::uint8_t> code{
        0x00U, 0xbfU, // nop
        0xfdU, 0xe7U, // b start
    };
    return board.memory().loadBytes(start, code).hasValue();
}

void loopBatchingMatchesExactBoardExecution() {
    auto exact = fil::sim::Board::load(fixtureBoard());
    auto batched = fil::sim::Board::load(fixtureBoard());
    fil::test::check(exact.hasValue() && batched.hasValue(),
                     "loads boards for standalone batching equivalence");
    if (!exact || !batched) return;
    fil::test::check(installIdleLoop(*exact.value()) && installIdleLoop(*batched.value()),
                     "installs standalone idle loops");
    exact.value()->trace().setEnabled(false);
    batched.value()->trace().setEnabled(false);

    fil::sim::BoardRunOptions exact_options;
    exact_options.max_instructions = 10'000U;
    exact_options.duration_ns = 0U;
    exact_options.detect_spin = false;
    exact_options.enable_loop_batching = false;
    auto batched_options = exact_options;
    batched_options.enable_loop_batching = true;

    const auto exact_result = exact.value()->run(exact_options);
    const auto batched_result = batched.value()->run(batched_options);
    const auto& exact_state = exact.value()->cpu().state();
    const auto& batched_state = batched.value()->cpu().state();
    fil::test::check(exact_result.reason == fil::sim::BoardStopReason::instruction_budget
                         && batched_result.reason == exact_result.reason
                         && batched_result.instructions == exact_result.instructions
                         && batched_result.cycles == exact_result.cycles
                         && batched_result.time_ns == exact_result.time_ns,
                     "standalone batching preserves limits and logical time");
    fil::test::check(batched_state.r == exact_state.r
                         && batched_state.xpsr == exact_state.xpsr
                         && batched_state.s == exact_state.s
                         && batched_state.instruction_address
                             == exact_state.instruction_address,
                     "standalone batching preserves final CPU state");
}

void spinDetectionTakesPriorityOverBatching() {
    auto board = fil::sim::Board::load(fixtureBoard());
    if (!board) {
        fil::test::check(false, "loads board for spin detection");
        return;
    }
    fil::test::check(installIdleLoop(*board.value()),
                     "installs idle loop for spin detection");
    board.value()->trace().setEnabled(false);

    fil::sim::BoardRunOptions options;
    options.max_instructions = 1'000U;
    options.duration_ns = 0U;
    options.detect_spin = true;
    options.spin_threshold = 20U;
    options.enable_loop_batching = true;
    const auto result = board.value()->run(options);
    fil::test::check(result.reason == fil::sim::BoardStopReason::spin_detected
                         && result.instructions >= options.spin_threshold
                         && result.instructions < options.max_instructions,
                     "spin diagnosis stops instead of batching through the loop");
}

void stopsAtRequestedAddress() {
    auto board = fil::sim::Board::load(fixtureBoard());
    if (!board) {
        fil::test::check(false, "loads board for stop-address test");
        return;
    }
    fil::sim::BoardRunOptions options;
    options.max_instructions = 20;
    options.duration_ns = 0;
    options.stop_address = 0x0800000cU;
    const auto result = board.value()->run(options);
    fil::test::check(result.reason == fil::sim::BoardStopReason::target_reached, "stops before requested PC");
}

void producesByteIdenticalTraceForRepeatedRuns() {
    auto first = fil::sim::Board::load(fixtureBoard());
    auto second = fil::sim::Board::load(fixtureBoard());
    fil::test::check(first.hasValue() && second.hasValue(), "loads two deterministic trace boards");
    if (!first || !second) return;
    fil::sim::BoardRunOptions options;
    options.max_instructions = 20;
    options.duration_ns = 0;
    options.detect_spin = false;
    options.trace_instructions = true;
    const auto first_result = first.value()->run(options);
    const auto second_result = second.value()->run(options);
    std::ostringstream first_trace;
    std::ostringstream second_trace;
    first.value()->trace().writeJsonLines(first_trace);
    second.value()->trace().writeJsonLines(second_trace);
    fil::test::check(first_result.reason == second_result.reason
                         && first_result.instructions == second_result.instructions,
                     "repeated fixture runs reach the same boundary");
    fil::test::check(first_trace.str() == second_trace.str(),
                     "repeated fixture runs produce byte-identical normalized traces");
}

} // namespace

void runBoardTests() {
    runsBoardToBreakpoint();
    workerSliceMatchesStandaloneExecution();
    restoresTransactionalBoardState();
    loopBatchingMatchesExactBoardExecution();
    spinDetectionTakesPriorityOverBatching();
    stopsAtRequestedAddress();
    producesByteIdenticalTraceForRepeatedRuns();
}
