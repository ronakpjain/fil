#include "fil/sim/board.hpp"

#include "../fixture_support.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace {

using fil::sim::Board;
using fil::sim::BoardRunOptions;
using fil::sim::BoardStopReason;
using fil::sim::Watchpoint;

std::unique_ptr<Board> loadFixture() {
    auto board = Board::load(fil::test::fixtureBoardConfig());
    if (!board) return nullptr;
    return std::move(board.value());
}

/// @brief Installs Thumb code at the reset vector and seeds the registers.
bool install(Board& board, const std::vector<std::uint8_t>& code,
             const std::uint32_t r0 = 0U, const std::uint32_t r1 = 0U) {
    board.cpu().state().r[0] = r0;
    board.cpu().state().r[1] = r1;
    return board.memory().loadBytes(board.cpu().state().r[15], code).hasValue();
}

BoardRunOptions optionsFor(const std::span<const Watchpoint> watchpoints) {
    BoardRunOptions options;
    options.max_instructions = 100U;
    options.duration_ns = 0U;
    options.detect_spin = false;
    options.watchpoints = watchpoints;
    return options;
}

} // namespace

TEST(WatchpointTest, FiresOnStoreAfterTheAccessingInstruction) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    const std::uint32_t start = board->cpu().state().r[15];
    // str r0, [r1]; b .
    ASSERT_TRUE(install(*board, {0x08U, 0x60U, 0xfeU, 0xe7U}, 0xdeadbeefU, 0x20000000U));
    const Watchpoint watchpoints[]{Watchpoint{0x20000000U, 4U, Watchpoint::Access::write}};

    const auto result = board->run(optionsFor(watchpoints));

    EXPECT_EQ(result.reason, BoardStopReason::watchpoint);
    ASSERT_TRUE(result.watchpoint.has_value());
    EXPECT_EQ(result.watchpoint->address, 0x20000000U);
    EXPECT_TRUE(result.watchpoint->write);
    EXPECT_EQ(result.instructions, 1U) << "stops after the store retires";
    EXPECT_EQ(board->cpu().state().r[15], start + 2U);
    EXPECT_EQ(board->memory().read32(0x20000000U).value(), 0xdeadbeefU);
}

TEST(WatchpointTest, WriteWatchpointIgnoresLoads) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    // ldr r2, [r1]; b .
    ASSERT_TRUE(install(*board, {0x0aU, 0x68U, 0xfeU, 0xe7U}, 0U, 0x20000000U));
    const Watchpoint watchpoints[]{Watchpoint{0x20000000U, 4U, Watchpoint::Access::write}};

    const auto result = board->run(optionsFor(watchpoints));

    EXPECT_EQ(result.reason, BoardStopReason::instruction_budget);
    EXPECT_FALSE(result.watchpoint.has_value());
    EXPECT_EQ(result.instructions, 100U);
}

TEST(WatchpointTest, ReadWatchpointFiresOnLoadsOnly) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    // ldr r2, [r1]; b .
    ASSERT_TRUE(install(*board, {0x0aU, 0x68U, 0xfeU, 0xe7U}, 0U, 0x20000000U));
    const Watchpoint watchpoints[]{Watchpoint{0x20000000U, 4U, Watchpoint::Access::read}};

    const auto result = board->run(optionsFor(watchpoints));

    EXPECT_EQ(result.reason, BoardStopReason::watchpoint);
    ASSERT_TRUE(result.watchpoint.has_value());
    EXPECT_FALSE(result.watchpoint->write);
    EXPECT_EQ(result.instructions, 1U);
}

TEST(WatchpointTest, AccessWatchpointFiresOnStores) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    // str r0, [r1]; b .
    ASSERT_TRUE(install(*board, {0x08U, 0x60U, 0xfeU, 0xe7U}, 0x11223344U, 0x20000000U));
    const Watchpoint watchpoints[]{Watchpoint{0x20000000U, 4U, Watchpoint::Access::access}};

    const auto result = board->run(optionsFor(watchpoints));

    EXPECT_EQ(result.reason, BoardStopReason::watchpoint);
    EXPECT_TRUE(result.watchpoint->write);
}

TEST(WatchpointTest, FiresOnMmioStore) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    // str r0, [r1]; b .
    ASSERT_TRUE(install(*board, {0x08U, 0x60U, 0xfeU, 0xe7U}, 0x1U, 0x48000000U));
    const Watchpoint watchpoints[]{Watchpoint{0x48000000U, 4U, Watchpoint::Access::write}};

    const auto result = board->run(optionsFor(watchpoints));

    EXPECT_EQ(result.reason, BoardStopReason::watchpoint)
        << "peripheral stores are visible to watchpoints";
    EXPECT_TRUE(result.watchpoint->write);
    EXPECT_EQ(result.instructions, 1U);
}

TEST(WatchpointTest, HonourRangeOverlap) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    // str r0, [r1]; b .
    ASSERT_TRUE(install(*board, {0x08U, 0x60U, 0xfeU, 0xe7U}, 0xaabbccddU, 0x20000000U));
    const Watchpoint watchpoints[]{Watchpoint{0x20000002U, 2U, Watchpoint::Access::write}};

    const auto result = board->run(optionsFor(watchpoints));

    EXPECT_EQ(result.reason, BoardStopReason::watchpoint);
    EXPECT_EQ(result.watchpoint->address, 0x20000002U);
}

TEST(WatchpointTest, DisabledWatchpointsDoNotStop) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    // str r0, [r1]; b .
    ASSERT_TRUE(install(*board, {0x08U, 0x60U, 0xfeU, 0xe7U}, 0U, 0x20000000U));

    const auto result = board->run(optionsFor({}));

    EXPECT_EQ(result.reason, BoardStopReason::instruction_budget);
    EXPECT_FALSE(result.watchpoint.has_value());
}
