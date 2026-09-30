#include "fil/sim/board.hpp"
#include "../fixture_support.hpp"

#include <gtest/gtest.h>

#include <sstream>
#include <vector>

namespace {

fil::config::BoardConfig fixtureBoard() {
    return fil::test::fixtureBoardConfig();
}

TEST(BoardTest, RunsBoardToBreakpoint) {
    auto board = fil::sim::Board::load(fixtureBoard());
    EXPECT_TRUE(board.hasValue()) << "loads integrated fixture board";
    if (!board) return;
    fil::sim::BoardRunOptions options;
    options.max_instructions = 20;
    options.duration_ns = 0;
    options.detect_spin = false;
    const auto result = board.value()->run(options);
    EXPECT_TRUE(result.reason == fil::sim::BoardStopReason::breakpoint)
        << "runs fixture firmware to BKPT";
    EXPECT_TRUE(result.instructions == 3) << "counts fixture instructions deterministically";
    EXPECT_TRUE(result.time_ns > 0) << "advances simulated board time from CPU cycles";
}

TEST(BoardTest, WorkerSliceMatchesStandaloneExecution) {
    auto exact = fil::sim::Board::load(fixtureBoard());
    auto worker = fil::sim::Board::load(fixtureBoard());
    EXPECT_TRUE(exact.hasValue() && worker.hasValue())
        << "loads exact and owner-local worker boards";
    if (!exact || !worker) return;

    fil::sim::BoardRunOptions options;
    options.max_instructions = 20U;
    options.duration_ns = 0U;
    const auto exact_result = exact.value()->run(options);
    const auto worker_result = worker.value()->runWorkerSlice(0U, 20U, 1'000'000U);
    EXPECT_TRUE(worker_result.reason == exact_result.reason &&
                worker_result.instructions == exact_result.instructions &&
                worker_result.cycles == exact_result.cycles &&
                worker_result.time_ns == exact_result.time_ns &&
                worker_result.diagnostic.next_pc == exact_result.diagnostic.next_pc)
        << "owner-local slice preserves standalone architectural results";
}

TEST(BoardTest, RestoresTransactionalBoardState) {
    auto board = fil::sim::Board::load(fixtureBoard());
    EXPECT_TRUE(board.hasValue()) << "loads board for transaction rollback";
    if (!board) return;

    const std::uint32_t original_pc = board.value()->cpu().state().r[15];
    const auto original_ram = board.value()->memory().read32(0x20000000U);
    const auto checkpoint = board.value()->captureTransaction(0U);
    board.value()->cpu().state().r[0] = 0xdeadbeefU;
    static_cast<void>(board.value()->memory().write32(0x20000000U, 0x12345678U));
    static_cast<void>(board.value()->eventLoop().runOwnedEvents(0U, 100U));
    EXPECT_TRUE(board.value()->restoreTransaction(checkpoint))
        << "restores a slice with only CPU RAM and clock mutations";
    const auto restored_ram = board.value()->memory().read32(0x20000000U);
    EXPECT_TRUE(board.value()->cpu().state().r[0] == 0U &&
                board.value()->cpu().state().r[15] == original_pc && original_ram && restored_ram &&
                restored_ram.value() == original_ram.value() &&
                board.value()->eventLoop().now(0U) == 0U)
        << "transaction rollback restores architectural and lane-clock state";
}

bool installIdleLoop(fil::sim::Board& board) {
    const std::uint32_t start = board.cpu().state().r[15] & ~1U;
    const std::vector<std::uint8_t> code{
        0x00U, 0xbfU, // nop
        0xfdU, 0xe7U, // b start
    };
    return board.memory().loadBytes(start, code).hasValue();
}

bool installAndWarmJitLoop(fil::sim::Board& board, const bool mmio = false) {
    const auto initial = board.cpu().state();
    const std::uint32_t start = initial.r[15];
    std::vector<std::uint8_t> code{
        0x01U, 0x30U, // adds r0, #1
        0x01U, 0x30U, // adds r0, #1
        0x01U, 0x30U, // adds r0, #1
        0xfbU, 0xe7U, // b start
    };
    if (mmio) { code[4] = 0x0aU; code[5] = 0x68U; } // ldr r2, [r1]
    if (!board.memory().loadBytes(start, code)) return false;
    for (unsigned int i = 0U; i < 60U; ++i) {
        board.cpu().state() = initial;
        board.cpu().state().r[1] = 0xe0001004U; // DWT_CYCCNT
        static_cast<void>(board.cpu().tryStepJitBlock());
    }
    board.cpu().state() = initial;
    board.cpu().state().r[1] = 0xe0001004U;
    return board.cpu().jitBlockReady();
}

TEST(BoardTest, JitPreservesScheduledEventAndDeadlineBoundaries) {
    auto exact = fil::sim::Board::load(fixtureBoard());
    auto jit = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(exact && jit);
    ASSERT_TRUE(installAndWarmJitLoop(*exact.value()));
    ASSERT_TRUE(installAndWarmJitLoop(*jit.value()));
    std::uint32_t exact_observed = 0U;
    std::uint32_t jit_observed = 0U;
    static_cast<void>(exact.value()->eventLoop().scheduleAt(100U, [&] {
        exact_observed = exact.value()->cpu().state().r[0];
        exact.value()->cpu().state().r[0] += 10U;
    }));
    static_cast<void>(jit.value()->eventLoop().scheduleAt(100U, [&] {
        jit_observed = jit.value()->cpu().state().r[0];
        jit.value()->cpu().state().r[0] += 10U;
    }));
    fil::sim::BoardRunOptions options;
    options.enable_loop_batching = false;
    options.max_instructions = 1'000U;
    options.duration_ns = 1'001U;
    options.enable_jit = false;
    const auto reference = exact.value()->run(options);
    options.enable_jit = true;
    const auto result = jit.value()->run(options);
    EXPECT_EQ(jit_observed, exact_observed);
    EXPECT_EQ(result.reason, reference.reason);
    EXPECT_EQ(result.instructions, reference.instructions);
    EXPECT_EQ(result.cycles, reference.cycles);
    EXPECT_EQ(result.time_ns, reference.time_ns);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.value()->cpu().state(), exact.value()->cpu().state()));
}

TEST(BoardTest, JitStopsBeforeInteriorTargetAndHonorsSmallBudget) {
    for (const bool stop_at_target : {false, true}) {
        auto board = fil::sim::Board::load(fixtureBoard());
        ASSERT_TRUE(board);
        ASSERT_TRUE(installAndWarmJitLoop(*board.value()));
        const auto start = board.value()->cpu().state().r[15];
        fil::sim::BoardRunOptions options;
        options.enable_loop_batching = false;
        options.duration_ns = 0U;
        options.max_instructions = stop_at_target ? 100U : 2U;
        options.enable_jit = true;
        if (stop_at_target) options.stop_address = start + 2U;
        const auto result = board.value()->run(options);
        EXPECT_EQ(result.reason, stop_at_target
            ? fil::sim::BoardStopReason::target_reached
            : fil::sim::BoardStopReason::instruction_budget);
        EXPECT_EQ(result.instructions, stop_at_target ? 1U : 2U);
        EXPECT_EQ(board.value()->cpu().state().r[15], start + (stop_at_target ? 2U : 4U));
    }
}

TEST(BoardTest, JitTimesMmioAfterCommittedAluPrefix) {
    auto exact = fil::sim::Board::load(fixtureBoard());
    auto jit = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(exact && jit);
    ASSERT_TRUE(installAndWarmJitLoop(*exact.value(), true));
    ASSERT_TRUE(installAndWarmJitLoop(*jit.value(), true));
    ASSERT_TRUE(exact.value()->memory().write32(0xe0001000U, 1U));
    ASSERT_TRUE(jit.value()->memory().write32(0xe0001000U, 1U));
    fil::sim::BoardRunOptions options;
    options.enable_loop_batching = false;
    options.max_instructions = 19U;
    options.duration_ns = 0U;
    options.enable_jit = false;
    const auto reference = exact.value()->run(options);
    options.enable_jit = true;
    const auto result = jit.value()->run(options);
    EXPECT_EQ(result.instructions, reference.instructions);
    EXPECT_EQ(result.cycles, reference.cycles);
    EXPECT_EQ(result.time_ns, reference.time_ns);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.value()->cpu().state(), exact.value()->cpu().state()));
    EXPECT_GT(jit.value()->cpu().state().r[2], 0U);
}

TEST(BoardTest, LoopBatchingMatchesExactBoardExecution) {
    auto exact = fil::sim::Board::load(fixtureBoard());
    auto batched = fil::sim::Board::load(fixtureBoard());
    EXPECT_TRUE(exact.hasValue() && batched.hasValue())
        << "loads boards for standalone batching equivalence";
    if (!exact || !batched) return;
    EXPECT_TRUE(installIdleLoop(*exact.value()) && installIdleLoop(*batched.value()))
        << "installs standalone idle loops";
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
    EXPECT_TRUE(exact_result.reason == fil::sim::BoardStopReason::instruction_budget &&
                batched_result.reason == exact_result.reason &&
                batched_result.instructions == exact_result.instructions &&
                batched_result.cycles == exact_result.cycles &&
                batched_result.time_ns == exact_result.time_ns)
        << "standalone batching preserves limits and logical time";
    EXPECT_TRUE(batched_state.r == exact_state.r && batched_state.xpsr == exact_state.xpsr &&
                batched_state.s == exact_state.s &&
                batched_state.instruction_address == exact_state.instruction_address)
        << "standalone batching preserves final CPU state";
}

TEST(BoardTest, SpinDetectionTakesPriorityOverBatching) {
    auto board = fil::sim::Board::load(fixtureBoard());
    if (!board) {
        EXPECT_TRUE(false) << "loads board for spin detection";
        return;
    }
    EXPECT_TRUE(installIdleLoop(*board.value())) << "installs idle loop for spin detection";
    board.value()->trace().setEnabled(false);

    fil::sim::BoardRunOptions options;
    options.max_instructions = 1'000U;
    options.duration_ns = 0U;
    options.detect_spin = true;
    options.spin_threshold = 20U;
    options.enable_loop_batching = true;
    const auto result = board.value()->run(options);
    EXPECT_TRUE(result.reason == fil::sim::BoardStopReason::spin_detected &&
                result.instructions >= options.spin_threshold &&
                result.instructions < options.max_instructions)
        << "spin diagnosis stops instead of batching through the loop";
}

TEST(BoardTest, StopsAtRequestedAddress) {
    auto board = fil::sim::Board::load(fixtureBoard());
    if (!board) {
        EXPECT_TRUE(false) << "loads board for stop-address test";
        return;
    }
    fil::sim::BoardRunOptions options;
    options.max_instructions = 20;
    options.duration_ns = 0;
    options.stop_address = 0x0800000cU;
    const auto result = board.value()->run(options);
    EXPECT_TRUE(result.reason == fil::sim::BoardStopReason::target_reached)
        << "stops before requested PC";
}

TEST(BoardTest, ProducesByteIdenticalTraceForRepeatedRuns) {
    auto first = fil::sim::Board::load(fixtureBoard());
    auto second = fil::sim::Board::load(fixtureBoard());
    EXPECT_TRUE(first.hasValue() && second.hasValue()) << "loads two deterministic trace boards";
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
    EXPECT_TRUE(first_result.reason == second_result.reason &&
                first_result.instructions == second_result.instructions)
        << "repeated fixture runs reach the same boundary";
    EXPECT_TRUE(first_trace.str() == second_trace.str())
        << "repeated fixture runs produce byte-identical normalized traces";
}

} // namespace
