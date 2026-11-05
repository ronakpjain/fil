#include "fil/debug/gdb_server.hpp"

#include "fil/cpu/cortex_m4.hpp"
#include "fil/debug/monitor.hpp"
#include "fil/sim/board.hpp"

#include "../fixture_support.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using fil::debug::GdbServerOptions;
using fil::debug::GdbSession;
using fil::debug::GdbSessionEnd;
using fil::debug::RspTransport;

/** @brief Deterministic scripted RSP transport with automatic packet acks. */
class ScriptedTransport final : public RspTransport {
public:
    /** @brief Queues one well-formed packet for the session to read. */
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
        sendRaw(frame);
    }

    /** @brief Queues raw bytes, such as a corrupt frame or an interrupt. */
    void sendRaw(const std::string_view bytes) {
        for (const char byte : bytes) input_.push_back(static_cast<std::uint8_t>(byte));
    }

    /** @brief Reports a pending interrupt once the predicate turns true. */
    void stopWhen(std::function<bool()> predicate) {
        pending_predicate_ = std::move(predicate);
    }

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

    bool inputPending() override {
        if (pending_predicate_ && pending_predicate_()) return true;
        return !input_.empty() && input_.front() == 0x03U;
    }

    fil::debug::TransportWait waitForInput(std::chrono::milliseconds) override {
        if (!input_.empty()) return fil::debug::TransportWait::ready;
        return fil::debug::TransportWait::closed;
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
    std::function<bool()> pending_predicate_;
};

/** @brief Extracts complete reply payloads from recorded transport output. */
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

/** @brief Decodes one monitor `O` packet payload into its console text. */
std::string decodeMonitorOutput(const std::string& payload) {
    std::string text;
    for (std::size_t index = 1U; index + 1U < payload.size(); index += 2U) {
        text.push_back(static_cast<char>(
            std::stoi(payload.substr(index, 2U), nullptr, 16)
        ));
    }
    return text;
}

/** @brief Renders an address as a bare hexadecimal RSP field. */
std::string hexWord(const std::uint32_t value) {
    std::string text(8U, '0');
    constexpr char digits[] = "0123456789abcdef";
    for (std::size_t index = 0U; index < 8U; ++index) {
        text[7U - index] = digits[(value >> (index * 4U)) & 0x0fU];
    }
    return text;
}

std::unique_ptr<fil::sim::Board> loadFixture() {
    auto board = fil::sim::Board::load(fil::test::fixtureBoardConfig());
    if (!board) return nullptr;
    return std::move(board.value());
}

GdbSessionEnd serve(
    fil::sim::Board& board,
    ScriptedTransport& transport,
    const GdbServerOptions& options = {},
    fil::debug::MonitorEndpoint* monitor = nullptr
) {
    GdbSession session(board, transport, options, monitor);
    const auto result = session.run();
    EXPECT_TRUE(result.hasValue());
    return result.hasValue() ? result.value() : GdbSessionEnd::disconnected;
}

/** @brief Monitor endpoint that executes one command on the first running service. */
class FakeMonitorEndpoint final : public fil::debug::MonitorEndpoint {
public:
    FakeMonitorEndpoint(fil::sim::Board& board, std::string command)
        : board_(board), command_(std::move(command)) {}

    void service(const bool running) override {
        if (executed_ || !running) return;
        executed_ = true;
        reply_ = fil::debug::executeMonitorCommand(board_, command_, running);
    }

    [[nodiscard]] bool active() const noexcept override { return true; }
    [[nodiscard]] bool executed() const noexcept { return executed_; }
    [[nodiscard]] const std::string& reply() const noexcept { return reply_.output; }

private:
    fil::sim::Board& board_;
    std::string command_;
    fil::debug::MonitorReply reply_;
    bool executed_{false};
};

} // namespace

TEST(GdbServerTest, ServesIdentificationQueries) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("?");
    transport.send("qSupported:multiprocess+");
    transport.send("qfThreadInfo");
    transport.send("qsThreadInfo");
    transport.send("qC");
    transport.send("qAttached");
    transport.send("qOffsets");
    transport.send("vCont?");
    transport.send("vMustReplyEmpty");
    transport.send("!");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 11U);
    EXPECT_EQ(replies[0], "S05");
    EXPECT_NE(replies[1].find("qXfer:features:read+"), std::string::npos);
    EXPECT_NE(replies[1].find("swbreak+"), std::string::npos);
    EXPECT_EQ(replies[2], "m1");
    EXPECT_EQ(replies[3], "l");
    EXPECT_EQ(replies[4], "QC1");
    EXPECT_EQ(replies[5], "1");
    EXPECT_EQ(replies[6], "Text=0;Data=0;Bss=0");
    EXPECT_EQ(replies[7], "vCont;c;C;s;S");
    EXPECT_EQ(replies[8], "");
    EXPECT_EQ(replies[9], "OK");
    EXPECT_EQ(replies[10], "OK");
}

TEST(GdbServerTest, ServesTargetDescriptionInChunks) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("qXfer:features:read:target.xml:0,10");
    transport.send("qXfer:features:read:target.xml:0,100000");
    transport.send("qXfer:features:read:target.xml:100000,10");
    transport.send("qXfer:features:read:other.xml:0,10");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 5U);
    EXPECT_EQ(replies[0].substr(0U, 5U), "m<?xm");
    EXPECT_EQ(replies[1].front(), 'l');
    EXPECT_NE(replies[1].find("org.gnu.gdb.arm.m-profile"), std::string::npos);
    EXPECT_NE(replies[1].find("org.gnu.gdb.arm.vfp"), std::string::npos);
    EXPECT_EQ(replies[2], "l");
    EXPECT_EQ(replies[3], "");
}

TEST(GdbServerTest, ServesFullAndSingleRegisters) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    board->cpu().state().r[0] = 0x12345678U;
    ScriptedTransport transport;
    transport.send("g");
    transport.send("p0");
    transport.send("P1=efbeadde");
    transport.send("p1");
    transport.send("P27=10000000");
    transport.send("p27");
    transport.send("p100");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 8U);
    EXPECT_EQ(replies[0].size(), 448U);
    EXPECT_EQ(replies[0].substr(0U, 8U), "78563412"); // r0 bytes in target order.
    EXPECT_EQ(replies[1], "78563412");
    EXPECT_EQ(replies[2], "OK");
    EXPECT_EQ(board->cpu().state().r[1], 0xdeadbeefU);
    EXPECT_EQ(replies[3], "efbeadde");
    EXPECT_EQ(replies[4], "OK");
    EXPECT_EQ(board->cpu().state().fpscr, 0x10U);
    EXPECT_EQ(replies[5], "10000000");
    EXPECT_EQ(replies[6], "E01");
}

TEST(GdbServerTest, WritesEveryRegister) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("G" + std::string(448U, '0'));
    transport.send("p0");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 3U);
    EXPECT_EQ(replies[0], "OK");
    EXPECT_EQ(board->cpu().state().r[0], 0U);
    EXPECT_EQ(board->cpu().state().xpsr, 0U);
    EXPECT_EQ(replies[1], "00000000");
}

TEST(GdbServerTest, ReadsAndWritesMemory) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("M20000000,4:efbeadde");
    transport.send("m20000000,4");
    transport.send(std::string("X20000010,4:hi}]}\x03"));
    transport.send("m20000010,4");
    transport.send("mfffffff0,80");
    transport.send("X20000020,2:0");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 7U);
    EXPECT_EQ(replies[0], "OK");
    EXPECT_EQ(replies[1], "efbeadde");
    const auto written = board->memory().read32(0x20000000U);
    ASSERT_TRUE(written.hasValue());
    EXPECT_EQ(written.value(), 0xdeadbeefU);
    EXPECT_EQ(replies[2], "OK");
    EXPECT_EQ(replies[3], "68697d23");
    EXPECT_EQ(replies[4], "E01");
    EXPECT_EQ(replies[5], "E01");
}

TEST(GdbServerTest, WritesLargeBinaryBlocks) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    std::string data(16U * 1024U, '\0');
    data.front() = 'A';
    data.back() = 'Z';
    ScriptedTransport transport;
    transport.send("X20000000,4000:" + data); // 16 KiB, as GDB's load sends.
    transport.send("m20000000,1");
    transport.send("m20003fff,1");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 4U);
    EXPECT_EQ(replies[0], "OK");
    EXPECT_EQ(replies[1], "41");
    EXPECT_EQ(replies[2], "5a");
    const auto first = board->memory().read8(0x20000000U);
    const auto last = board->memory().read8(0x20003fffU);
    ASSERT_TRUE(first.hasValue());
    ASSERT_TRUE(last.hasValue());
    EXPECT_EQ(first.value(), 'A');
    EXPECT_EQ(last.value(), 'Z');
}

TEST(GdbServerTest, StopsAtSoftwareAndHardwareBreakpoints) {
    auto probe = loadFixture();
    ASSERT_NE(probe, nullptr);
    fil::sim::BoardRunOptions probe_options;
    probe_options.max_instructions = 2U;
    probe_options.duration_ns = 0U;
    probe_options.detect_spin = false;
    const auto probe_result = probe->run(probe_options);
    ASSERT_EQ(probe_result.reason, fil::sim::BoardStopReason::instruction_budget);
    const std::uint32_t target = probe->cpu().state().r[15];

    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("Z0," + hexWord(target) + ",2");
    transport.send("c");
    transport.send("z0," + hexWord(target) + ",2");
    transport.send("Z1," + hexWord(target) + ",4");
    transport.send("c");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 6U);
    EXPECT_EQ(replies[0], "OK");
    EXPECT_EQ(replies[1], "T05swbreak:;");
    EXPECT_EQ(replies[2], "OK");
    EXPECT_EQ(replies[3], "OK");
    EXPECT_EQ(replies[4], "T05hwbreak:;");
    EXPECT_EQ(replies[5], "OK");
    EXPECT_EQ(board->cpu().state().r[15], target);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(board->cpu().state(), probe->cpu().state()));
}

TEST(GdbServerTest, MonitorReadsDoNotHaltRunningTarget) {
    auto probe = loadFixture();
    ASSERT_NE(probe, nullptr);
    fil::sim::BoardRunOptions probe_options;
    probe_options.max_instructions = 2U;
    probe_options.duration_ns = 0U;
    probe_options.detect_spin = false;
    static_cast<void>(probe->run(probe_options));
    const std::uint32_t target = probe->cpu().state().r[15];

    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ASSERT_TRUE(board->memory().write32(0x20000000U, 0x11223344U).hasValue());
    ScriptedTransport transport;
    FakeMonitorEndpoint monitor(*board, "mdw 0x20000000 1");
    transport.send("Z0," + hexWord(target) + ",2");
    transport.send("c");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport, {}, &monitor), GdbSessionEnd::detached);
    EXPECT_TRUE(monitor.executed()) << "monitor is serviced while the target runs";
    EXPECT_NE(monitor.reply().find("11223344"), std::string::npos);
    EXPECT_EQ(board->cpu().state().r[15], target)
        << "a monitor read cannot halt or advance the target";
}

TEST(GdbServerTest, StepsOneInstruction) {
    auto probe = loadFixture();
    ASSERT_NE(probe, nullptr);
    fil::sim::BoardRunOptions probe_options;
    probe_options.max_instructions = 1U;
    probe_options.duration_ns = 0U;
    probe_options.detect_spin = false;
    static_cast<void>(probe->run(probe_options));
    const std::uint32_t next = probe->cpu().state().r[15];

    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("s");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 2U);
    EXPECT_EQ(replies[0], "T05");
    EXPECT_EQ(board->cpu().state().r[15], next);
}

TEST(GdbServerTest, InterruptStopsRunningTarget) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    const std::uint32_t start = board->cpu().state().r[15];
    const std::vector<std::uint8_t> loop{0x00U, 0xbfU, 0xfdU, 0xe7U};
    ASSERT_TRUE(board->memory().loadBytes(start, loop).hasValue());

    ScriptedTransport transport;
    transport.stopWhen([&board] { return board->eventLoop().now() >= 10'000U; });
    GdbServerOptions options;
    options.slice_instructions = 32U;
    options.slice_time_ns = 0U;
    transport.send("c");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport, options), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 2U);
    EXPECT_EQ(replies[0], "T02");
    EXPECT_EQ(replies[1], "OK");
    EXPECT_GE(board->eventLoop().now(), 10'000U);
}

TEST(GdbServerTest, RecoversFromCorruptChecksum) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.sendRaw("$qC#00");
    transport.send("qC");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 2U);
    EXPECT_EQ(replies[0], "QC1");
    EXPECT_EQ(replies[1], "OK");
    EXPECT_NE(transport.output().find('-'), std::string::npos);
}

TEST(GdbServerTest, ResetMonitorCommandRestoresResetState) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    const std::uint32_t reset_pc = board->cpu().state().r[15];
    board->cpu().state().r[0] = 0xfeedfaceU;
    board->cpu().state().r[15] = reset_pc + 8U;

    ScriptedTransport transport;
    transport.send("qRcmd,7265736574");
    transport.send("qRcmd,6e6f7065");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 5U);
    EXPECT_EQ(decodeMonitorOutput(replies[0]), "reset complete\n");
    EXPECT_EQ(replies[1], "OK");
    EXPECT_NE(decodeMonitorOutput(replies[2]).find("unknown command 'nope'"),
        std::string::npos);
    EXPECT_EQ(replies[3], "OK");
    EXPECT_EQ(replies[4], "OK");
    EXPECT_EQ(board->cpu().state().r[15], reset_pc);
    EXPECT_EQ(board->cpu().state().r[0], 0U);
}

TEST(GdbServerTest, ExtendedRunRestartsFromResetState) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    const std::uint32_t reset_pc = board->cpu().state().r[15];
    board->cpu().state().r[0] = 0xfeedfaceU;
    board->cpu().state().r[15] = reset_pc + 8U;

    ScriptedTransport transport;
    transport.send("vKill;1");
    transport.send("vRun;");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 3U);
    EXPECT_EQ(replies[0], "OK");
    EXPECT_EQ(replies[1], "T05");
    EXPECT_EQ(board->cpu().state().r[15], reset_pc);
    EXPECT_EQ(board->cpu().state().r[0], 0U);
    EXPECT_EQ(replies[2], "OK");
}

TEST(GdbServerTest, MonitorCommandsShareTheTelnetLayer) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    board->cpu().state().r[0] = 0xfeedfaceU;
    ScriptedTransport transport;
    transport.send("qRcmd,72657365742068616c74"); // reset halt
    transport.send("qRcmd,68616c74");             // halt
    transport.send("qRcmd,6d647720307832303030303030302031"); // mdw 0x20000000 1
    transport.send("D");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 7U);
    EXPECT_EQ(decodeMonitorOutput(replies[0]), "reset complete\n");
    EXPECT_EQ(replies[1], "OK");
    EXPECT_EQ(board->cpu().state().r[0], 0U);
    EXPECT_EQ(decodeMonitorOutput(replies[2]), "target halted\n");
    EXPECT_EQ(replies[3], "OK");
    EXPECT_NE(decodeMonitorOutput(replies[4]).find("0x20000000:"), std::string::npos);
    EXPECT_EQ(replies[5], "OK");
    EXPECT_EQ(replies[6], "OK");
}

TEST(GdbServerTest, NoAckModeSuppressesAcknowledgements) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("QStartNoAckMode");
    transport.send("p0");
    transport.send("D");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::detached);
    const auto replies = repliesOf(transport.output());
    ASSERT_GE(replies.size(), 3U);
    EXPECT_EQ(replies[0], "OK");
    EXPECT_EQ(replies[1], "00000000");
    EXPECT_EQ(replies[2], "OK");
}

TEST(GdbServerTest, KillEndsSessionWithoutReply) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("k");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::killed);
    EXPECT_TRUE(repliesOf(transport.output()).empty());
}

TEST(GdbServerTest, EndOfInputEndsSession) {
    auto board = loadFixture();
    ASSERT_NE(board, nullptr);
    ScriptedTransport transport;
    transport.send("p0");

    EXPECT_EQ(serve(*board, transport), GdbSessionEnd::disconnected);
}
