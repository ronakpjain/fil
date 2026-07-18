#include "fil/sim/board.hpp"
#include "../test_support.hpp"

#include <filesystem>
#include <sstream>

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
    stopsAtRequestedAddress();
    producesByteIdenticalTraceForRepeatedRuns();
}
