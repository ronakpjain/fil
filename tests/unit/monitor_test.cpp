#include "fil/debug/monitor.hpp"

#include "fil/sim/board.hpp"

#include "../fixture_support.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace {

std::unique_ptr<fil::sim::Board> loadFixture() {
    auto board = fil::sim::Board::load(fil::test::fixtureBoardConfig());
    if (!board) return nullptr;
    return std::move(board.value());
}

fil::debug::MonitorReply run(
    fil::sim::Board& board,
    const std::string_view command,
    const bool running = false
) {
    return fil::debug::executeMonitorCommand(board, command, running);
}

} // namespace

TEST(MonitorTest, ReadsMemoryAtEveryWidth) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ASSERT_TRUE(board->memory().write32(0x20000000U, 0xdeadbeefU).hasValue());

    const auto words = run(*board, "mdw 0x20000000 1");
    EXPECT_NE(words.output.find("0x20000000:"), std::string::npos);
    EXPECT_NE(words.output.find("deadbeef"), std::string::npos);

    const auto halfwords = run(*board, "mdh 0x20000000 2");
    EXPECT_NE(halfwords.output.find("beef dead"), std::string::npos);

    const auto bytes = run(*board, "mdb 0x20000000 4");
    EXPECT_NE(bytes.output.find("ef be ad de"), std::string::npos);
    EXPECT_NE(bytes.output.find("...."), std::string::npos);
}

TEST(MonitorTest, RefusesSideEffectingMmioWhileRunning) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);

    const auto stopped = run(*board, "mdw 0x40021000 1");
    EXPECT_EQ(stopped.output.find("error:"), std::string::npos);
    EXPECT_NE(stopped.output.find("0x40021000:"), std::string::npos);

    const auto running = run(*board, "mdw 0x40021000 1", true);
    EXPECT_NE(running.output.find("target is running"), std::string::npos);

    const auto ram = run(*board, "mdw 0x20000000 1", true);
    EXPECT_EQ(ram.output.find("error:"), std::string::npos);
    EXPECT_NE(ram.output.find("0x20000000:"), std::string::npos);
}

TEST(MonitorTest, DumpsRegistersAndInformation) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    board->cpu().state().r[3] = 0x1234U;

    const auto registers = run(*board, "regs");
    EXPECT_NE(registers.output.find("r3"), std::string::npos);
    EXPECT_NE(registers.output.find("0x00001234"), std::string::npos);
    EXPECT_NE(registers.output.find("fpscr"), std::string::npos);

    const auto stopped = run(*board, "info");
    EXPECT_NE(stopped.output.find("board = fixture"), std::string::npos);
    EXPECT_NE(stopped.output.find("state = stopped"), std::string::npos);
    EXPECT_NE(stopped.output.find("clock = 16000000 Hz"), std::string::npos);

    const auto running = run(*board, "info", true);
    EXPECT_NE(running.output.find("state = running"), std::string::npos);
}

TEST(MonitorTest, ReadsNamedRegisters) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    board->cpu().state().r[3] = 0x1234U;
    board->cpu().state().s[0] = 1.5F;

    EXPECT_NE(run(*board, "reg r3").output.find("r3 = 0x00001234"), std::string::npos);
    EXPECT_NE(run(*board, "reg sp").output.find("sp = 0x"), std::string::npos);
    const auto single = run(*board, "reg s0");
    EXPECT_NE(single.output.find("0x3fc00000"), std::string::npos);
    EXPECT_NE(single.output.find("(1.5)"), std::string::npos);
    EXPECT_NE(run(*board, "reg d0").output.find("0x000000003fc00000"), std::string::npos);
    EXPECT_NE(run(*board, "reg psr").output.find("unknown register"), std::string::npos);
}

TEST(MonitorTest, ResetsOnlyWhileStopped) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    const std::uint32_t reset_pc = board->cpu().state().r[15];
    board->cpu().state().r[0] = 0xfeedfaceU;

    const auto refused = run(*board, "reset", true);
    EXPECT_NE(refused.output.find("target is running"), std::string::npos);
    EXPECT_EQ(board->cpu().state().r[0], 0xfeedfaceU);

    const auto reset = run(*board, "reset");
    EXPECT_NE(reset.output.find("reset complete"), std::string::npos);
    EXPECT_EQ(board->cpu().state().r[0], 0U);
    EXPECT_EQ(board->cpu().state().r[15], reset_pc);
}

TEST(MonitorTest, AcceptsOpenOcdStyleResetAndHalt) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    const std::uint32_t reset_pc = board->cpu().state().r[15];
    board->cpu().state().r[0] = 0xfeedfaceU;

    const auto halted = run(*board, "halt", true);
    EXPECT_NE(halted.output.find("target halted"), std::string::npos);
    EXPECT_EQ(board->cpu().state().r[0], 0xfeedfaceU);

    const auto refused = run(*board, "reset halt", true);
    EXPECT_NE(refused.output.find("target is running"), std::string::npos);
    EXPECT_EQ(board->cpu().state().r[0], 0xfeedfaceU);

    EXPECT_NE(run(*board, "reset halt").output.find("reset complete"), std::string::npos);
    EXPECT_EQ(board->cpu().state().r[0], 0U);
    EXPECT_NE(run(*board, "reset init").output.find("reset complete"), std::string::npos);
    EXPECT_NE(run(*board, "reset run").output.find("accepts 'halt' or 'init'"),
        std::string::npos);
    EXPECT_EQ(board->cpu().state().r[15], reset_pc);
}

TEST(MonitorTest, ReportsUnknownAndMalformedCommands) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);

    EXPECT_NE(run(*board, "bogus").output.find("unknown command"), std::string::npos);
    EXPECT_NE(run(*board, "mdw").output.find("requires an address"), std::string::npos);
    EXPECT_NE(run(*board, "mdw zzz").output.find("invalid address"), std::string::npos);
    EXPECT_NE(run(*board, "mdw 0x20000000 0").output.find("count must be"), std::string::npos);
    EXPECT_NE(run(*board, "mdw 0xfffffff0 64").output.find("overflows"), std::string::npos);
    EXPECT_NE(run(*board, "help").output.find("mdw ADDR [COUNT]"), std::string::npos);

    const auto empty = run(*board, "   ");
    EXPECT_TRUE(empty.output.empty());
    EXPECT_FALSE(empty.close);
}

TEST(MonitorTest, QuitClosesTheConnection) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    EXPECT_TRUE(run(*board, "quit").close);
    EXPECT_TRUE(run(*board, "exit").close);
    EXPECT_FALSE(run(*board, "info").close);
}
