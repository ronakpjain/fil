#include "fil/debug/live_gdb.hpp"

#include "fil/debug/gdb_server.hpp"
#include "fil/sim/board.hpp"

#include "../fixture_support.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

using fil::debug::LiveGdbSession;
using fil::debug::RspTransport;
using fil::debug::TransportWait;

/** @brief Scripted transport shared by the live-session tests. */
class ScriptedTransport final : public RspTransport {
public:
    void send(const std::string_view payload) {
        std::string frame = "$";
        frame.append(payload);
        frame.push_back('#');
        std::uint8_t checksum = 0U;
        for (const char character : payload) {
            checksum = static_cast<std::uint8_t>(checksum + static_cast<std::uint8_t>(character));
        }
        constexpr char digits[] = "0123456789abcdef";
        frame.push_back(digits[checksum >> 4U]);
        frame.push_back(digits[checksum & 0x0fU]);
        for (const char byte : frame) input_.push_back(static_cast<std::uint8_t>(byte));
    }

    void reportEndOfInput() { report_eof_pending_ = true; }

    [[nodiscard]] const std::string& output() const noexcept { return output_; }

    std::optional<std::uint8_t> readByte() override {
        if (pending_acks_ > 0U) {
            --pending_acks_;
            return static_cast<std::uint8_t>('+');
        }
        if (input_.empty()) return std::nullopt;
        const auto byte = input_.front();
        input_.pop_front();
        return byte;
    }

    bool inputPending() override { return report_eof_pending_ || !input_.empty(); }

    TransportWait waitForInput(std::chrono::milliseconds) override {
        return input_.empty() ? TransportWait::closed : TransportWait::ready;
    }

    bool write(const std::string_view bytes) override {
        output_.append(bytes);
        ++pending_acks_;
        return true;
    }

private:
    std::deque<std::uint8_t> input_;
    std::string output_;
    std::size_t pending_acks_{0U};
    bool report_eof_pending_{false};
};

std::vector<std::string> repliesOf(const std::string& output) {
    std::vector<std::string> replies;
    std::size_t position = 0U;
    while (position < output.size()) {
        if (output[position] != '$') {
            ++position;
            continue;
        }
        const auto hash = output.find('#', position);
        if (hash == std::string::npos) break;
        replies.push_back(output.substr(position + 1U, hash - position - 1U));
        position = hash + 3U;
    }
    return replies;
}

std::unique_ptr<fil::sim::Board> loadFixture() {
    auto board = fil::sim::Board::load(fil::test::fixtureBoardConfig());
    if (!board) return nullptr;
    return std::move(board.value());
}

/** @brief Runs every scripted packet through one live session. */
std::vector<std::string> runLive(
    fil::sim::Board& board,
    ScriptedTransport& transport,
    const bool running
) {
    LiveGdbSession session(board, transport);
    session.service(running);
    return repliesOf(transport.output());
}

} // namespace

TEST(LiveGdbTest, ServesIdentificationAndTargetDescription) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("?");
    transport.send("qSupported:multiprocess+");
    transport.send("qXfer:features:read:target.xml:0,5");
    transport.send("qfThreadInfo");
    transport.send("qsThreadInfo");
    transport.send("qAttached");
    transport.send("qOffsets");
    transport.send("vMustReplyEmpty");

    const auto replies = runLive(*board, transport, false);
    ASSERT_GE(replies.size(), 8U);
    EXPECT_EQ(replies[0], "S05");
    EXPECT_NE(replies[1].find("qXfer:features:read+"), std::string::npos);
    EXPECT_NE(replies[1].find("QStartNoAckMode+"), std::string::npos);
    EXPECT_EQ(replies[2], "m<?xml");
    EXPECT_EQ(replies[3], "m1");
    EXPECT_EQ(replies[4], "l");
    EXPECT_EQ(replies[5], "1");
    EXPECT_EQ(replies[6], "Text=0;Data=0;Bss=0");
    EXPECT_EQ(replies[7], "");
}

TEST(LiveGdbTest, ReadsRegistersAndMemoryWhileRunning) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    board->cpu().state().r[0] = 0x12345678U;
    ASSERT_TRUE(board->memory().write32(0x20000000U, 0xa5a5a5a5U).hasValue());
    ScriptedTransport transport;
    transport.send("g");
    transport.send("p0");
    transport.send("m20000000,4");

    const auto replies = runLive(*board, transport, true);
    ASSERT_GE(replies.size(), 3U);
    EXPECT_EQ(replies[0].size(), 448U);
    EXPECT_EQ(replies[0].substr(0U, 8U), "78563412");
    EXPECT_EQ(replies[1], "78563412");
    EXPECT_EQ(replies[2], "a5a5a5a5");
}

TEST(LiveGdbTest, RefusesMmioReadsWhileRunning) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("m40021000,4");
    transport.send("m20000000,4");

    const auto replies = runLive(*board, transport, true);
    ASSERT_GE(replies.size(), 2U);
    EXPECT_EQ(replies[0], "E01") << "peripheral reads would perturb the target";
    EXPECT_EQ(replies[1], "00000000") << "RAM stays readable while running";
}

TEST(LiveGdbTest, AllowsMmioReadsWhenStopped) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("m40021000,4");

    const auto replies = runLive(*board, transport, false);
    ASSERT_GE(replies.size(), 1U);
    EXPECT_EQ(replies[0].find("E01"), std::string::npos);
    EXPECT_EQ(replies[0].size(), 8U) << "register block reads are served while stopped";
}

TEST(LiveGdbTest, RefusesEveryWriteAndControlRequest) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    const auto before = board->cpu().state().r[0];
    const auto before_memory = board->memory().read32(0x20000000U).value();
    ScriptedTransport transport;
    transport.send("M20000000,4:aabbccdd");
    transport.send("X20000000,1:Z");
    transport.send("P0=efbeadde");
    transport.send("G" + std::string(448U, '0'));
    transport.send("c");
    transport.send("s");
    transport.send("vCont;c");
    transport.send("Z0,8000012,2");
    transport.send("z0,8000012,2");
    transport.send("R00");
    transport.send("qRcmd,7265736574");
    transport.send("k");

    const auto replies = runLive(*board, transport, true);
    ASSERT_GE(replies.size(), 12U);
    EXPECT_EQ(replies[0], "E01");
    EXPECT_EQ(replies[1], "E01");
    EXPECT_EQ(replies[2], "E01");
    EXPECT_EQ(replies[3], "E01");
    EXPECT_EQ(replies[4], "E01");
    EXPECT_EQ(replies[5], "E01");
    EXPECT_EQ(replies[6], "E01");
    EXPECT_EQ(replies[7], "E01");
    EXPECT_EQ(replies[8], "E01");
    EXPECT_EQ(replies[9], "E01");
    EXPECT_EQ(replies[10], "");
    EXPECT_EQ(replies[11], "E01");
    EXPECT_EQ(board->cpu().state().r[0], before);
    EXPECT_EQ(board->memory().read32(0x20000000U).value(), before_memory);
}

TEST(LiveGdbTest, DetachClosesOnlyTheLiveSession) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("D");

    LiveGdbSession session(*board, transport);
    session.service(false);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 1U);
    EXPECT_EQ(replies[0], "OK");
    EXPECT_TRUE(session.closed());
}

TEST(LiveGdbTest, EndOfInputClosesTheSession) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("?");
    transport.send("m20000000,4");

    LiveGdbSession session(*board, transport);
    session.service(false);
    EXPECT_FALSE(session.closed());
    transport.reportEndOfInput();
    session.service(false);
    EXPECT_TRUE(session.closed()) << "an exhausted transport ends the live session";
}
