#include "fil/sim/board.hpp"
#include "../fixture_support.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include <cstdint>
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

TEST(BoardTest, WorkerBoundaryStillHonorsResetRequestsWithoutTakableIrqs) {
    auto board = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(board);
    ASSERT_TRUE(board.value()->memory().write32(0xe000ed0cU, 0x05fa0004U));

    const auto result = board.value()->runWorkerSlice(0U, 10U, 1'000U);
    EXPECT_EQ(result.reason, fil::sim::BoardStopReason::reset_requested);
    EXPECT_EQ(result.instructions, 0U);
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
    for (std::uint32_t i = 0U; i < 60U; ++i) {
        board.cpu().state() = initial;
        board.cpu().state().r[1] = 0xe0001004U; // DWT_CYCCNT
        static_cast<void>(board.cpu().tryStepJitBlock());
    }
    board.cpu().state() = initial;
    board.cpu().state().r[1] = 0xe0001004U;
    return board.cpu().jitBlockReady();
}

bool warmIdempotentPeriodCache(fil::sim::Board& board) {
    const std::vector<std::uint8_t> code{
        0x08U,0xb5U, 0x00U,0xf0U,0x05U,0xf8U, 0x07U,0x4bU,
        0x1bU,0x68U, 0x01U,0x2bU, 0xf9U,0xd9U, 0xfeU,0xe7U,
        0x08U,0xb5U, 0x00U,0xe0U, 0x00U,0xbfU, 0x02U,0x4bU,
        0x1bU,0x68U, 0x00U,0x2bU, 0xfaU,0xd1U, 0x08U,0xbdU,
        0x40U,0x00U,0x00U,0x20U, 0x44U,0x00U,0x00U,0x20U};
    board.cpu().state().r[3] = 1U;
    board.cpu().setNativeSingleInstructionJitEnabled(false);
    if (!board.memory().write32(0x40022000U, 0U) // Pin the period-cache fixture's fetch cost.
        || !board.memory().write32(0x20000040U, 0U)
        || !board.memory().write32(0x20000044U, 1U)
        || !board.memory().write32(0xe0001000U, 1U)
        || !board.memory().loadBytes(board.cpu().state().r[15], code)) return false;
    for (std::uint32_t i = 0U; i < 64U; ++i) {
        static_cast<void>(board.cpu().prepareJitBlock());
    }
    fil::sim::BoardRunOptions warm;
    warm.duration_ns = 0U;
    warm.enable_loop_batching = false;
    warm.enable_jit = true;
    warm.max_instructions = 13U;
    if (board.run(warm).instructions != 13U) return false;
    const auto entry = board.cpu().state();
    const auto base = entry.r[15] - 2U;
    for (const auto offset : {2U, 6U, 8U, 10U, 12U, 16U, 18U, 22U, 24U, 26U, 28U, 30U}) {
        board.cpu().state().r[15] = base + offset;
        for (std::uint32_t i = 0U; i < 64U; ++i) {
            static_cast<void>(board.cpu().prepareJitBlock(true));
        }
    }
    board.cpu().state() = entry;
    auto built = board.prepareReversibleRamPrefix(36U);
    if (!built || !built->period_certificate || built->count != 36U) return false;
    static_cast<void>(board.materializeReversibleRamPrefix(*built, built->count));
    const auto reused = board.prepareReversibleRamPrefix(12U);
    if (!reused || reused->memoized_count == 0U) return false;
    static_cast<void>(board.materializeReversibleRamPrefix(*reused, reused->count));
    return true;
}

bool installBranchingReversibleProgram(fil::sim::Board& board) {
    const std::uint32_t start = board.cpu().state().r[15];
    std::vector<std::uint8_t> code;
    for (std::uint32_t i = 0U; i < 80U; ++i) {
        code.push_back(0x01U); code.push_back(0x30U); // adds r0, #1
        if (i == 5U) {
            code.push_back(0x00U); code.push_back(0xd1U); // bne +0: skip next halfword
            code.push_back(0x01U); code.push_back(0x30U);
        } else if (i == 12U) {
            code.push_back(0x00U); code.push_back(0xe0U); // b +0: skip next halfword
            code.push_back(0x01U); code.push_back(0x30U);
        }
    }
    if (!board.memory().loadBytes(start, code)) return false;
    const auto initial = board.cpu().state();
    // Warm entry points reached after conditional/unconditional branches and
    // the 16-op CPU block cap; the Board chain itself must never execute cold.
    for (const std::uint32_t instruction : {0U, 8U, 17U, 33U, 49U, 65U, 81U}) {
        for (std::uint32_t attempt = 0U; attempt < 64U; ++attempt) {
            board.cpu().state().r[15] = start + instruction * 2U;
            static_cast<void>(board.cpu().prepareJitBlock());
        }
    }
    board.cpu().state() = initial;
    board.cpu().state().r[0] = 0U;
    board.cpu().setNativeSingleInstructionJitEnabled(false);
    return true;
}

TEST(BoardTest, ReversibleRamPrefixChainsBranchesAndReplaysLongCuts) {
    auto exact = fil::sim::Board::load(fixtureBoard());
    auto speculative = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(exact && speculative);
    ASSERT_TRUE(installBranchingReversibleProgram(*exact.value()));
    ASSERT_TRUE(installBranchingReversibleProgram(*speculative.value()));

    const auto prefix = speculative.value()->prepareReversibleRamPrefix(64U);
    ASSERT_TRUE(prefix);
    EXPECT_EQ(prefix->count, 64U);
    EXPECT_EQ(prefix->evaluated.count, prefix->count);
    bool crossed_branch = false;
    for (std::size_t i = 1U; i < prefix->count; ++i) {
        crossed_branch |= prefix->evaluated.pcs[i]
            != prefix->evaluated.pcs[i - 1U] + prefix->evaluated.sizes[i - 1U];
    }
    EXPECT_TRUE(crossed_branch) << "the chained metadata follows taken conditional/unconditional branches";

    constexpr std::size_t cut = 23U;
    const auto committed = speculative.value()->materializeReversibleRamPrefix(*prefix, cut);
    fil::sim::BoardRunOptions options;
    options.max_instructions = cut;
    options.duration_ns = 0U;
    options.enable_loop_batching = false;
    const auto reference = exact.value()->run(options);
    EXPECT_EQ(reference.instructions, cut);
    EXPECT_EQ(committed.cpu_result.instructions, cut);
    EXPECT_EQ(committed.cpu_result.instruction_address, prefix->evaluated.pcs[cut - 1U]);
    EXPECT_EQ(committed.cpu_result.cycles, reference.cycles);
    EXPECT_EQ(committed.elapsed_ns, reference.time_ns);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(speculative.value()->cpu().state(), exact.value()->cpu().state()));

    auto full_exact = fil::sim::Board::load(fixtureBoard());
    auto full_speculative = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(full_exact && full_speculative);
    ASSERT_TRUE(installBranchingReversibleProgram(*full_exact.value()));
    ASSERT_TRUE(installBranchingReversibleProgram(*full_speculative.value()));
    const auto full_prefix = full_speculative.value()->prepareReversibleRamPrefix(64U);
    ASSERT_TRUE(full_prefix);
    ASSERT_EQ(full_prefix->count, 64U);
    const auto full_commit = full_speculative.value()->materializeReversibleRamPrefix(
        *full_prefix, full_prefix->count);
    options.max_instructions = 64U;
    const auto full_reference = full_exact.value()->run(options);
    EXPECT_EQ(full_commit.cpu_result.instructions, 64U);
    EXPECT_EQ(full_commit.cpu_result.cycles, full_reference.cycles);
    EXPECT_EQ(full_commit.elapsed_ns, full_reference.time_ns);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(full_speculative.value()->cpu().state(),
                                       full_exact.value()->cpu().state()));
}

TEST(BoardTest, ReusableReversibleRamOutputUsesOnlyLiveMetadata) {
    auto board = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(board);
    ASSERT_TRUE(installBranchingReversibleProgram(*board.value()));

    fil::sim::Board::ReversibleRamPrefix prefix;
    ASSERT_TRUE(board.value()->prepareReversibleRamPrefix(prefix, 64U));
    ASSERT_EQ(prefix.count, 64U);
    ASSERT_TRUE(board.value()->reset());
    ASSERT_TRUE(installBranchingReversibleProgram(*board.value()));

    ASSERT_TRUE(board.value()->prepareReversibleRamPrefix(prefix, 17U));
    EXPECT_EQ(prefix.count, 17U);
    EXPECT_EQ(prefix.evaluated.count, prefix.count);
    for (std::size_t i = 0U; i < prefix.count; ++i) {
        EXPECT_NE(prefix.evaluated.pcs[i], 0U);
        EXPECT_NE(prefix.completion_times_ns[i], 0U);
        if (i != 0U) {
            EXPECT_GT(prefix.completion_times_ns[i], prefix.completion_times_ns[i - 1U]);
            EXPECT_GT(prefix.cumulative_cycles[i], prefix.cumulative_cycles[i - 1U]);
        }
    }
}

TEST(BoardTest, IdempotentPeriodCacheReusesNonzeroPeriodPhase) {
    auto exact = fil::sim::Board::load(fixtureBoard());
    auto speculative = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(exact && speculative);
    ASSERT_TRUE(warmIdempotentPeriodCache(*exact.value()));
    ASSERT_TRUE(warmIdempotentPeriodCache(*speculative.value()));

    // Establish a certificate, then leave the cached cycle at a nonzero phase.
    const auto seed = speculative.value()->prepareReversibleRamPrefix(36U);
    ASSERT_TRUE(seed);
    ASSERT_TRUE(seed->period_certificate);
    const auto first = speculative.value()->materializeReversibleRamPrefix(*seed, 1U);
    fil::sim::BoardRunOptions one;
    one.duration_ns = 0U;
    one.enable_loop_batching = false;
    one.max_instructions = 1U;
    const auto first_reference = exact.value()->run(one);
    ASSERT_EQ(first.cpu_result.instructions, first_reference.instructions);
    ASSERT_EQ(first.cpu_result.cycles, first_reference.cycles);
    ASSERT_TRUE(fil::cpu::bitwiseEqual(speculative.value()->cpu().state(),
                                       exact.value()->cpu().state()));

    const auto cached = speculative.value()->prepareReversibleRamPrefix(12U);
    ASSERT_TRUE(cached);
    EXPECT_GT(cached->memoized_count, 0U)
        << "a valid period certificate must be reusable from an interior phase";
    const auto committed = speculative.value()->materializeReversibleRamPrefix(
        *cached, cached->count);
    one.max_instructions = cached->count;
    const auto reference = exact.value()->run(one);
    EXPECT_EQ(committed.cpu_result.cycles, reference.cycles);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(speculative.value()->cpu().state(),
                                       exact.value()->cpu().state()));
}

TEST(BoardTest, IdempotentPeriodCacheRejectsChangedFlashClockAndCodeGeneration) {
    {
        auto board = fil::sim::Board::load(fixtureBoard());
        ASSERT_TRUE(board);
        ASSERT_TRUE(warmIdempotentPeriodCache(*board.value()));
        const auto old_prefix = board.value()->prepareReversibleRamPrefix(8U);
        ASSERT_TRUE(old_prefix);
        ASSERT_NE(old_prefix->memoized_count, 0U);
        static_cast<void>(board.value()->materializeReversibleRamPrefix(
            *old_prefix, old_prefix->count));
        const auto old_frequency = board.value()->peripherals().rcc().systemClockHz();
        ASSERT_TRUE(board.value()->memory().write32(0x40021008U, 0U)); // select MSI
        ASSERT_NE(board.value()->peripherals().rcc().systemClockHz(), old_frequency);
        const auto fresh = board.value()->prepareReversibleRamPrefix(8U);
        ASSERT_TRUE(fresh);
        EXPECT_EQ(fresh->memoized_count, 0U)
            << "a clock change must not reuse cached cycle cost";
    }
    {
        auto board = fil::sim::Board::load(fixtureBoard());
        ASSERT_TRUE(board);
        ASSERT_TRUE(warmIdempotentPeriodCache(*board.value()));
        const auto old_prefix = board.value()->prepareReversibleRamPrefix(8U);
        ASSERT_TRUE(old_prefix);
        ASSERT_NE(old_prefix->memoized_count, 0U);
        static_cast<void>(board.value()->materializeReversibleRamPrefix(
            *old_prefix, old_prefix->count));
        ASSERT_TRUE(board.value()->memory().write32(0x40022000U, 2U)); // FLASH ACR latency
        const auto fresh = board.value()->prepareReversibleRamPrefix(8U);
        ASSERT_TRUE(fresh);
        EXPECT_EQ(fresh->memoized_count, 0U)
            << "FLASH ACR changes must not reuse cached fetch costs";
    }
    {
        auto board = fil::sim::Board::load(fixtureBoard());
        ASSERT_TRUE(board);
        ASSERT_TRUE(warmIdempotentPeriodCache(*board.value()));
        const auto old_prefix = board.value()->prepareReversibleRamPrefix(8U);
        ASSERT_TRUE(old_prefix);
        ASSERT_NE(old_prefix->memoized_count, 0U);
        static_cast<void>(board.value()->materializeReversibleRamPrefix(
            *old_prefix, old_prefix->count));
        const std::uint32_t pc = board.value()->cpu().state().r[15] & ~1U;
        const auto first = board.value()->memory().read8(pc);
        const auto second = board.value()->memory().read8(pc + 1U);
        ASSERT_TRUE(first && second);
        const std::vector<std::uint8_t> same_instruction{first.value(), second.value()};
        ASSERT_TRUE(board.value()->memory().loadBytes(pc, same_instruction));
        const auto fresh = board.value()->prepareReversibleRamPrefix(8U);
        ASSERT_TRUE(fresh);
        EXPECT_EQ(fresh->memoized_count, 0U)
            << "executable writes must invalidate a cached period certificate";
    }
}

TEST(BoardTest, ReversibleRamPrefixRespectsInstructionAndTimeBoundaries) {
    auto budget_board = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(budget_board);
    ASSERT_TRUE(installBranchingReversibleProgram(*budget_board.value()));
    const auto budget = budget_board.value()->prepareReversibleRamPrefix(17U);
    ASSERT_TRUE(budget);
    EXPECT_EQ(budget->count, 17U);

    auto event_board = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(event_board);
    ASSERT_TRUE(installBranchingReversibleProgram(*event_board.value()));
    const auto baseline = event_board.value()->prepareReversibleRamPrefix(40U);
    ASSERT_TRUE(baseline);
    ASSERT_GE(baseline->count, 2U);
    const auto event_time = baseline->completion_times_ns[1U];
    ASSERT_TRUE(event_board.value()->eventLoop().scheduleAt(event_time, [] {}));
    EXPECT_FALSE(event_board.value()->prepareReversibleRamPrefix(40U));

    auto deadline_board = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(deadline_board);
    ASSERT_TRUE(installBranchingReversibleProgram(*deadline_board.value()));
    const auto deadline_baseline = deadline_board.value()->prepareReversibleRamPrefix(40U);
    ASSERT_TRUE(deadline_baseline);
    EXPECT_FALSE(deadline_board.value()->prepareReversibleRamPrefix(
        40U, deadline_baseline->completion_times_ns[1U]));

    auto tick_board = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(tick_board);
    ASSERT_TRUE(installBranchingReversibleProgram(*tick_board.value()));
    const auto tick_baseline = tick_board.value()->prepareReversibleRamPrefix(40U);
    ASSERT_TRUE(tick_baseline);
    ASSERT_GE(tick_baseline->count, 2U);
    const auto tick_cycles = tick_baseline->cumulative_cycles[1U];
    ASSERT_GT(tick_cycles, 0U);
    ASSERT_TRUE(tick_board.value()->memory().write32(
        0xe000e014U, static_cast<std::uint32_t>(tick_cycles - 1U)));
    ASSERT_TRUE(tick_board.value()->memory().write32(0xe000e010U, 3U));
    EXPECT_FALSE(tick_board.value()->prepareReversibleRamPrefix(40U));
}

TEST(BoardTest, DeferredPurePrefixAdmissionDoesNotExecuteAndCanMaterializeOne) {
    auto board = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(board);
    ASSERT_TRUE(installAndWarmJitLoop(*board.value()));
    const auto initial = board.value()->cpu().state();
    const auto admitted_time = board.value()->eventLoop().now();
    EXPECT_FALSE(board.value()->prepareDeferredPurePrefix(1U));
    EXPECT_FALSE(board.value()->prepareDeferredPurePrefix(8U, admitted_time + 1U));
    const auto prefix = board.value()->prepareDeferredPurePrefix(8U);
    ASSERT_TRUE(prefix);
    EXPECT_GE(prefix->count, 2U);
    EXPECT_EQ(prefix->entry_pc, initial.r[15]);
    EXPECT_EQ(board.value()->eventLoop().now(), admitted_time);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(board.value()->cpu().state(), initial));

    const auto result = board.value()->materializeDeferredPurePrefix(*prefix, 1U);
    EXPECT_EQ(result.cpu_result.instructions, 1U);
    EXPECT_EQ(board.value()->cpu().state().r[15], initial.r[15] + 2U);
    EXPECT_GT(result.cpu_result.cycles, 0U);
    EXPECT_GT(result.elapsed_ns, 0U);
}

TEST(BoardTest, DeferredPurePrefixWithFractionAndFlashWaitsMatchesExactExecution) {
    auto exact = fil::sim::Board::load(fixtureBoard());
    auto deferred = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(exact && deferred);
    ASSERT_TRUE(exact.value()->memory().write32(0x40022000U, 2U));
    ASSERT_TRUE(deferred.value()->memory().write32(0x40022000U, 2U));
    ASSERT_TRUE(installAndWarmJitLoop(*exact.value()));
    ASSERT_TRUE(installAndWarmJitLoop(*deferred.value()));

    // Advance both boards by one instruction to leave a fractional-ns phase.
    fil::sim::BoardRunOptions one;
    one.max_instructions = 1U;
    one.duration_ns = 0U;
    one.enable_loop_batching = false;
    const auto exact_first = exact.value()->run(one);
    const auto deferred_first = deferred.value()->run(one);
    ASSERT_EQ(exact_first.instructions, 1U);
    ASSERT_EQ(deferred_first.instructions, 1U);
    ASSERT_TRUE(installAndWarmJitLoop(*exact.value()));
    ASSERT_TRUE(installAndWarmJitLoop(*deferred.value()));

    const auto prefix = deferred.value()->prepareDeferredPurePrefix(8U);
    ASSERT_TRUE(prefix);
    EXPECT_NE(prefix->time_fraction, 0U);
    ASSERT_GE(prefix->count, 2U);
    const std::size_t count = 2U;
    one.max_instructions = count;
    const auto reference = exact.value()->run(one);
    ASSERT_EQ(reference.instructions, count);
    const auto materialized = deferred.value()->materializeDeferredPurePrefix(*prefix, count);
    EXPECT_EQ(materialized.cpu_result.instructions, count);
    EXPECT_EQ(materialized.cpu_result.cycles, reference.cycles);
    EXPECT_EQ(materialized.elapsed_ns, reference.time_ns - exact_first.time_ns);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(deferred.value()->cpu().state(), exact.value()->cpu().state()));
}

TEST(BoardTest, DeferredPurePrefixRejectsExecutableCodeInvalidation) {
    auto board = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(board);
    ASSERT_TRUE(installAndWarmJitLoop(*board.value()));
    const auto prefix = board.value()->prepareDeferredPurePrefix(8U);
    ASSERT_TRUE(prefix);
    const auto before = board.value()->cpu().state();
    const std::uint32_t pc = before.r[15];
    const std::vector<std::uint8_t> replacement{0x02U, 0x30U};
    ASSERT_TRUE(board.value()->memory().loadBytes(pc, replacement));
    const auto result = board.value()->materializeDeferredPurePrefix(*prefix, 1U);
    EXPECT_EQ(result.cpu_result.instructions, 0U);
    EXPECT_EQ(result.elapsed_ns, 0U);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(board.value()->cpu().state(), before));
}

TEST(BoardTest, DeferredPurePrefixStopsStrictlyBeforeEventAndSysTick) {
    auto event_board = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(event_board);
    ASSERT_TRUE(installAndWarmJitLoop(*event_board.value()));
    const auto baseline = event_board.value()->prepareDeferredPurePrefix(8U);
    ASSERT_TRUE(baseline);
    ASSERT_GE(baseline->count, 2U);
    ASSERT_TRUE(event_board.value()->eventLoop().scheduleAt(
        baseline->completion_times_ns[1U], [] {}));
    EXPECT_FALSE(event_board.value()->prepareDeferredPurePrefix(8U));

    auto tick_board = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(tick_board);
    ASSERT_TRUE(installAndWarmJitLoop(*tick_board.value()));
    const auto tick_baseline = tick_board.value()->prepareDeferredPurePrefix(8U);
    ASSERT_TRUE(tick_baseline);
    ASSERT_GE(tick_baseline->count, 2U);
    const auto cycles_to_boundary = tick_baseline->cumulative_cycles[1U];
    ASSERT_GT(cycles_to_boundary, 0U);
    ASSERT_TRUE(tick_board.value()->memory().write32(
        0xe000e014U, static_cast<std::uint32_t>(cycles_to_boundary - 1U)));
    ASSERT_TRUE(tick_board.value()->memory().write32(0xe000e010U, 3U));
    EXPECT_FALSE(tick_board.value()->prepareDeferredPurePrefix(8U));
}

TEST(BoardTest, PureJitBlocksMatchExactAcrossFlashTimingChanges) {
    auto exact = fil::sim::Board::load(fixtureBoard());
    auto jit = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(exact && jit);
    ASSERT_TRUE(installAndWarmJitLoop(*exact.value()));
    ASSERT_TRUE(installAndWarmJitLoop(*jit.value()));

    fil::sim::BoardRunOptions options;
    options.max_instructions = 37U; // Includes branches and a partial final block.
    options.duration_ns = 0U;
    options.enable_loop_batching = false;
    // Rewrite ACR between spans to exercise generation invalidation, ART on
    // sequential fetches, taken branches, and the zero-latency fast path.
    for (const std::uint32_t acr : {4U, 4U | (1U << 8U), 2U | (1U << 9U), 0U, 3U}) {
        SCOPED_TRACE(acr);
        ASSERT_TRUE(exact.value()->memory().write32(0x40022000U, acr));
        ASSERT_TRUE(jit.value()->memory().write32(0x40022000U, acr));
        options.enable_jit = false;
        const auto reference = exact.value()->run(options);
        options.enable_jit = true;
        const auto accelerated = jit.value()->run(options);
        EXPECT_EQ(accelerated.reason, reference.reason);
        EXPECT_EQ(accelerated.instructions, reference.instructions);
        EXPECT_EQ(accelerated.cycles, reference.cycles);
        EXPECT_EQ(accelerated.time_ns, reference.time_ns);
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.value()->cpu().state(), exact.value()->cpu().state()));
    }
}

TEST(BoardTest, SingleInstructionDispatchMatchesWithFlashWaits) {
    auto exact = fil::sim::Board::load(fixtureBoard());
    auto jit = fil::sim::Board::load(fixtureBoard());
    ASSERT_TRUE(exact && jit);
    ASSERT_TRUE(exact.value()->memory().write32(0x40022000U, 2U));
    ASSERT_TRUE(jit.value()->memory().write32(0x40022000U, 2U));
    ASSERT_TRUE(installAndWarmJitLoop(*exact.value()));
    ASSERT_TRUE(installAndWarmJitLoop(*jit.value()));
    const std::uint32_t target = exact.value()->cpu().state().r[15] + 6U;

    fil::sim::BoardRunOptions options;
    options.max_instructions = 10U;
    options.duration_ns = 0U;
    options.enable_loop_batching = false;
    options.stop_address = target; // Forces exact one-instruction dispatch.
    options.enable_jit = false;
    const auto reference = exact.value()->run(options);
    options.enable_jit = true;
    const auto accelerated = jit.value()->run(options);

    EXPECT_EQ(accelerated.reason, reference.reason);
    EXPECT_EQ(accelerated.instructions, reference.instructions);
    EXPECT_EQ(accelerated.cycles, reference.cycles);
    EXPECT_EQ(accelerated.time_ns, reference.time_ns);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.value()->cpu().state(), exact.value()->cpu().state()));
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
