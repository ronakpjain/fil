#include "fil/devices/can_bus.hpp"
#include "fil/sim/event_loop.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <vector>
#include <thread>
#include <algorithm>

namespace {

TEST(CanBusTest, BroadcastsDeterministically) {
    fil::devices::VirtualCanBus bus("vehicle");
    std::vector<std::int32_t> deliveries;
    std::vector<fil::devices::CanTraceRecord> trace;
    bus.setTraceCallback([&](const auto& record) { trace.push_back(record); });
    const auto first = bus.attach("first", false, [&](const auto&, std::uint64_t) { deliveries.push_back(1); });
    const auto second = bus.attach("second", true, [&](const auto&, std::uint64_t) { deliveries.push_back(2); });
    EXPECT_TRUE(first && second) << "attaches virtual CAN nodes";
    if (!first || !second) return;

    fil::devices::CanFrame frame;
    frame.id = 0x123U;
    frame.dlc = 2;
    frame.data[0] = 0xaaU;
    frame.data[1] = 0x55U;
    EXPECT_TRUE(bus.send(first.value(), frame, 100U).hasValue()) << "sends valid CAN frame";
    EXPECT_TRUE(deliveries == std::vector<std::int32_t>({2})) << "skips sender without loopback";
    EXPECT_TRUE(trace.size() == 2 && trace[0].sequence < trace[1].sequence)
        << "traces TX then RX in stable order";

    deliveries.clear();
    EXPECT_TRUE(bus.inject(frame, 200U).hasValue()) << "injects external CAN frame";
    EXPECT_TRUE(deliveries == std::vector<std::int32_t>({1, 2})) << "delivers injection in attachment order";
}

TEST(CanBusTest, DeliveryBarrierPrecedesObserversAndCanBeRestored) {
    fil::devices::VirtualCanBus bus("vehicle");
    std::vector<std::int32_t> order;
    const auto sender = bus.attach("sender", false, [](const auto&, std::uint64_t) {});
    const auto receiver = bus.attach("receiver", false,
        [&](const auto&, std::uint64_t) { order.push_back(3); });
    ASSERT_TRUE(sender);
    ASSERT_TRUE(receiver);
    bus.setTraceCallback([&](const auto&) { order.push_back(2); });
    auto original = bus.exchangeDeliveryBarrier([&](const auto time) {
        EXPECT_EQ(time, 100U);
        order.push_back(1);
    });
    EXPECT_FALSE(original);
    fil::devices::CanFrame frame;
    frame.id = 0x123U;
    EXPECT_FALSE(bus.send(999U, frame, 100U));
    EXPECT_TRUE(order.empty());
    ASSERT_TRUE(bus.send(sender.value(), frame, 100U));
    EXPECT_EQ(order, (std::vector<std::int32_t>{1, 2, 2, 3}));
    auto barrier = bus.exchangeDeliveryBarrier(std::move(original));
    EXPECT_TRUE(barrier);
    order.clear();
    ASSERT_TRUE(bus.inject(frame, 200U));
    EXPECT_EQ(order, (std::vector<std::int32_t>{2, 2, 2, 3}));
}

TEST(CanBusTest, CountsStuffedClassicFramesAndIncludesIntermission) {
    fil::devices::CanFrame frame;
    frame.id = 0x123U;
    frame.dlc = 7U;
    for (std::size_t i = 0; i < 7U; ++i) frame.data[i] = 0U;
    const auto standard = fil::devices::classicCanStuffedFrameBits(frame);
    ASSERT_TRUE(standard);
    EXPECT_EQ(standard.value(), 112U); // 100 base bits + 2 stuff bits + 10 unstuffed tail bits.
    const auto duration = fil::devices::classicCanDurationNs(frame, 500'000U);
    ASSERT_TRUE(duration);
    EXPECT_EQ(duration.value(), 230'000U); // 115 wire bits including intermission.

    frame.extended = true;
    frame.id = 0x1234567U;
    const auto extended = fil::devices::classicCanStuffedFrameBits(frame);
    ASSERT_TRUE(extended);
    EXPECT_EQ(extended.value(), 132U);
    EXPECT_FALSE(fil::devices::classicCanDurationNs(frame, 0U));
    frame.fd = true;
    EXPECT_FALSE(fil::devices::classicCanStuffedFrameBits(frame));
}

TEST(CanBusTest, TimedBusArbitratesAndBoundsQueuedRequests) {
    fil::sim::EventLoop loop;
    fil::devices::VirtualCanBus bus("timed", 500'000U,
        fil::devices::VirtualCanBus::DeliveryMode::timed, 2U);
    bus.setEventLoop(&loop);
    std::vector<std::uint64_t> received;
    std::vector<fil::devices::CanTraceRecord> records;
    bus.setTraceCallback([&](const auto& record) { records.push_back(record); });
    const auto node = bus.attach("receiver", false,
        [&](const auto&, const auto time) { received.push_back(time); });
    ASSERT_TRUE(node);
    fil::devices::CanFrame frame;
    frame.id = 0x123U;
    frame.dlc = 7U;
    for (std::size_t i = 0; i < 7U; ++i) frame.data[i] = 0U;
    ASSERT_TRUE(bus.inject(frame, 0U));
    ASSERT_TRUE(bus.inject(frame, 0U));
    EXPECT_FALSE(bus.inject(frame, 0U));
    EXPECT_EQ(bus.queuedTransmissions(), 2U);
    EXPECT_TRUE(received.empty());
    static_cast<void>(loop.runDueEvents(223'999U));
    EXPECT_TRUE(received.empty());
    static_cast<void>(loop.runDueEvents(224'000U));
    ASSERT_EQ(received.size(), 1U);
    EXPECT_EQ(received[0], 224'000U);
    static_cast<void>(loop.runDueEvents(453'999U));
    ASSERT_EQ(received.size(), 1U);
    static_cast<void>(loop.runDueEvents(454'000U));
    ASSERT_EQ(received.size(), 2U);
    EXPECT_EQ(received[1], 454'000U);
    EXPECT_EQ(bus.queuedTransmissions(), 0U);
    ASSERT_EQ(records.size(), 5U); // rejected third request, two SOF and two EOF records.
    EXPECT_EQ(records[0].direction, fil::devices::CanTraceRecord::Direction::rejected);
    EXPECT_EQ(records[1].direction, fil::devices::CanTraceRecord::Direction::transmit);
    EXPECT_EQ(records[1].time_ns, 0U);
    EXPECT_EQ(records[2].direction, fil::devices::CanTraceRecord::Direction::receive);
    EXPECT_EQ(records[2].time_ns, 224'000U);
    EXPECT_EQ(records[3].direction, fil::devices::CanTraceRecord::Direction::transmit);
    EXPECT_EQ(records[3].time_ns, 230'000U);
}

TEST(CanBusTest, DetachingSenderCancelsItsInFlightFrame) {
    fil::sim::EventLoop loop;
    fil::devices::VirtualCanBus bus("timed", 500'000U,
        fil::devices::VirtualCanBus::DeliveryMode::timed, 2U);
    bus.setEventLoop(&loop);
    std::vector<std::uint64_t> received;
    const auto sender = bus.attach("sender", false, [](const auto&, const auto) {});
    ASSERT_TRUE(sender);
    ASSERT_TRUE(bus.attach("receiver", false,
        [&](const auto&, const auto at) { received.push_back(at); }));
    fil::devices::CanFrame frame;
    frame.id = 0x123U;
    ASSERT_TRUE(bus.send(sender.value(), frame, 0U));
    bus.detach(sender.value());
    static_cast<void>(loop.runDueEvents(1'000'000U));
    EXPECT_TRUE(received.empty());
    EXPECT_EQ(bus.queuedTransmissions(), 0U);
}

TEST(CanBusTest, SofCancellationCannotCompleteReplacementAtStaleEof) {
    for (const bool detach : {false, true}) {
        SCOPED_TRACE(detach);
        fil::sim::EventLoop loop;
        fil::devices::VirtualCanBus bus("timed", 7U,
            fil::devices::VirtualCanBus::DeliveryMode::timed);
        bus.setEventLoop(&loop);
        const auto a = bus.attach("a", false, [](const auto&, auto) {});
        const auto b = bus.attach("b", false, [](const auto&, auto) {});
        ASSERT_TRUE(a && b);
        std::vector<std::uint64_t> received;
        ASSERT_TRUE(bus.attach("receiver", false,
            [&](const auto&, auto time) { received.push_back(time); }));
        fil::devices::CanFrame short_frame; short_frame.id = 0x123U;
        fil::devices::CanFrame long_frame; long_frame.id = 0x300U; long_frame.dlc = 8U;
        const auto short_bits = fil::devices::classicCanStuffedFrameBits(short_frame);
        const auto long_bits = fil::devices::classicCanStuffedFrameBits(long_frame);
        ASSERT_TRUE(short_bits && long_bits);
        const auto stale_eof = short_bits.value() * 1'000'000'000ULL / 7U;
        const auto expected_eof = long_bits.value() * 1'000'000'000ULL / 7U;
        ASSERT_GT(expected_eof, stale_eof);
        std::uint32_t canceled_completions = 0U;
        std::vector<std::uint64_t> completions;
        bus.setTraceCallback([&](const auto& record) {
            if (record.direction == fil::devices::CanTraceRecord::Direction::transmit
                && record.node == "a") {
                if (detach) bus.detach(a.value());
                else bus.cancelSender(a.value());
                // Replacing the active sink must not destroy its callable mid-call.
                bus.setTraceCallback({});
                EXPECT_EQ(record.frame.id, short_frame.id);
            }
        });
        ASSERT_TRUE(bus.send(a.value(), short_frame, 0U,
            [&](auto, bool) { ++canceled_completions; }));
        ASSERT_TRUE(bus.send(b.value(), long_frame, 0U,
            [&](auto time, bool ok) { EXPECT_TRUE(ok); completions.push_back(time); }));
        static_cast<void>(loop.runDueEvents(stale_eof));
        EXPECT_TRUE(received.empty());
        EXPECT_TRUE(completions.empty());
        EXPECT_EQ(bus.queuedTransmissions(), 1U);
        EXPECT_EQ(loop.pending(), 1U);
        static_cast<void>(loop.runDueEvents(expected_eof));
        EXPECT_EQ(received, (std::vector<std::uint64_t>{expected_eof}));
        EXPECT_EQ(completions, (std::vector<std::uint64_t>{expected_eof}));
        EXPECT_EQ(canceled_completions, 0U);
        EXPECT_EQ(bus.queuedTransmissions(), 0U);
        EXPECT_EQ(loop.pending(), 0U);
    }
}

TEST(CanBusTest, ReceiveTraceMayDetachItsReceiver) {
    fil::devices::VirtualCanBus bus("immediate");
    std::uint32_t received = 0U;
    const auto receiver = bus.attach("receiver", false,
        [&](const auto&, auto) { ++received; });
    ASSERT_TRUE(receiver);
    bus.setTraceCallback([&](const auto& record) {
        if (record.direction == fil::devices::CanTraceRecord::Direction::receive)
            bus.detach(receiver.value());
    });
    ASSERT_TRUE(bus.inject({}, 0U));
    EXPECT_EQ(received, 0U);
}

TEST(CanBusTest, TimedArbitrationUsesSenderHeadsAndStandardFormatPriority) {
    fil::sim::EventLoop loop;
    fil::devices::VirtualCanBus bus("arb", 500'000U,
        fil::devices::VirtualCanBus::DeliveryMode::timed, 8U);
    bus.setEventLoop(&loop);
    std::vector<std::uint32_t> transmitted;
    std::vector<std::string> transmitters;
    bus.setTraceCallback([&](const auto& record) {
        if (record.direction == fil::devices::CanTraceRecord::Direction::transmit) {
            transmitters.push_back(record.node);
            transmitted.push_back(record.frame.id);
        }
    });
    const auto a = bus.attach("a", false, [](const auto&, auto) {});
    const auto b = bus.attach("b", false, [](const auto&, auto) {});
    ASSERT_TRUE(a && b);
    fil::devices::CanFrame a_head; a_head.id = 0x300U;
    fil::devices::CanFrame a_next; a_next.id = 0x001U;
    fil::devices::CanFrame b_frame; b_frame.id = 0x200U;
    ASSERT_TRUE(bus.send(a.value(), a_head, 0U));
    ASSERT_TRUE(bus.send(a.value(), a_next, 0U));
    ASSERT_TRUE(bus.send(b.value(), b_frame, 0U));
    static_cast<void>(loop.runDueEvents(10'000'000U));
    ASSERT_EQ(transmitted.size(), 3U);
    EXPECT_EQ(transmitted[0], 0x200U);
    EXPECT_EQ(transmitted[1], 0x300U); // a_next cannot overtake a_head.
    EXPECT_EQ(transmitted[2], 0x001U);

    fil::sim::EventLoop second_loop;
    fil::devices::VirtualCanBus format_bus("format", 500'000U,
        fil::devices::VirtualCanBus::DeliveryMode::timed, 4U);
    format_bus.setEventLoop(&second_loop);
    std::vector<std::string> format_winners;
    const auto standard = format_bus.attach("standard", false, [](const auto&, auto) {});
    const auto extended = format_bus.attach("extended", false, [](const auto&, auto) {});
    ASSERT_TRUE(standard && extended);
    format_bus.setTraceCallback([&](const auto& record) {
        if (record.direction == fil::devices::CanTraceRecord::Direction::transmit)
            format_winners.push_back(record.node);
    });
    fil::devices::CanFrame ext; ext.extended = true; ext.id = 0x123U << 18U;
    fil::devices::CanFrame std; std.id = 0x123U;
    ASSERT_TRUE(format_bus.send(extended.value(), ext, 0U));
    ASSERT_TRUE(format_bus.send(standard.value(), std, 0U));
    static_cast<void>(second_loop.runDueEvents(2'000'000U));
    ASSERT_EQ(format_winners.size(), 2U);
    EXPECT_EQ(format_winners[0], "standard");
}

TEST(CanBusTest, TimedEqualIdTieUsesStableAdmissionOrder) {
    fil::sim::EventLoop loop;
    fil::devices::VirtualCanBus bus("tie", 500'000U,
        fil::devices::VirtualCanBus::DeliveryMode::timed, 4U);
    bus.setEventLoop(&loop);
    std::vector<std::string> winners;
    bus.setTraceCallback([&](const auto& record) {
        if (record.direction == fil::devices::CanTraceRecord::Direction::transmit)
            winners.push_back(record.node);
    });
    const auto later = bus.attach("first-admitted", false, [](const auto&, auto) {});
    const auto earlier_id = bus.attach("second-admitted", false, [](const auto&, auto) {});
    ASSERT_TRUE(later && earlier_id);
    fil::devices::CanFrame frame; frame.id = 0x321U;
    ASSERT_TRUE(bus.send(later.value(), frame, 0U));
    ASSERT_TRUE(bus.send(earlier_id.value(), frame, 0U));
    static_cast<void>(loop.runDueEvents(2'000'000U));
    ASSERT_EQ(winners.size(), 2U);
    EXPECT_EQ(winners[0], "first-admitted");
    EXPECT_EQ(winners[1], "second-admitted");
}

TEST(CanBusTest, TimedArbitrationKeepsExternalInjectionOrder) {
    fil::sim::EventLoop loop;
    fil::devices::VirtualCanBus bus("arb", 500'000U,
        fil::devices::VirtualCanBus::DeliveryMode::timed, 8U);
    bus.setEventLoop(&loop);
    std::vector<std::uint32_t> transmitted;
    bus.setTraceCallback([&](const auto& record) {
        if (record.direction == fil::devices::CanTraceRecord::Direction::transmit)
            transmitted.push_back(record.frame.id);
    });
    const auto sender = bus.attach("firmware", false, [](const auto&, auto) {});
    ASSERT_TRUE(sender);
    fil::devices::CanFrame external_first; external_first.id = 0x300U;
    fil::devices::CanFrame external_next; external_next.id = 0x001U;
    fil::devices::CanFrame firmware; firmware.id = 0x200U;
    ASSERT_TRUE(bus.inject(external_first, 0U));
    ASSERT_TRUE(bus.inject(external_next, 0U));
    ASSERT_TRUE(bus.send(sender.value(), firmware, 0U));
    static_cast<void>(loop.runDueEvents(10'000'000U));
    ASSERT_EQ(transmitted.size(), 3U);
    EXPECT_EQ(transmitted[0], 0x200U);
    EXPECT_EQ(transmitted[1], 0x300U); // second external frame cannot overtake its head.
    EXPECT_EQ(transmitted[2], 0x001U);
}

TEST(CanBusTest, CancellingFutureHeadReschedulesSurvivingRequestAndZeroBitrateRejects) {
    fil::sim::EventLoop loop;
    fil::devices::VirtualCanBus bus("arb", 500'000U,
        fil::devices::VirtualCanBus::DeliveryMode::timed, 8U);
    bus.setEventLoop(&loop);
    const auto canceled = bus.attach("canceled", false, [](const auto&, auto) {});
    const auto survivor = bus.attach("survivor", false, [](const auto&, auto) {});
    ASSERT_TRUE(canceled && survivor);
    std::vector<std::uint32_t> received;
    ASSERT_TRUE(bus.attach("receiver", false, [&](const auto& f, auto) { received.push_back(f.id); }));
    fil::devices::CanFrame future; future.id = 0x100U;
    fil::devices::CanFrame now; now.id = 0x200U;
    ASSERT_TRUE(bus.send(canceled.value(), future, 100'000U));
    ASSERT_TRUE(bus.send(survivor.value(), now, 0U));
    bus.cancelSender(canceled.value());
    static_cast<void>(loop.runDueEvents(1'000'000U));
    ASSERT_EQ(received.size(), 1U);
    EXPECT_EQ(received[0], 0x200U);

    fil::sim::EventLoop zero_loop;
    fil::devices::VirtualCanBus zero("zero", 0U,
        fil::devices::VirtualCanBus::DeliveryMode::timed, 2U);
    zero.setEventLoop(&zero_loop);
    EXPECT_FALSE(zero.inject(now, 0U));
    EXPECT_EQ(zero.queuedTransmissions(), 0U);

    fil::sim::EventLoop near_overflow;
    static_cast<void>(near_overflow.runDueEvents(std::numeric_limits<std::uint64_t>::max()));
    fil::devices::VirtualCanBus overflow("overflow", 500'000U,
        fil::devices::VirtualCanBus::DeliveryMode::timed, 2U);
    overflow.setEventLoop(&near_overflow);
    bool completed = true;
    ASSERT_TRUE(overflow.inject(now, near_overflow.now(),
        [&](std::uint64_t, bool success) { completed = success; }));
    static_cast<void>(near_overflow.runDueEvents(std::numeric_limits<std::uint64_t>::max()));
    EXPECT_FALSE(completed);
    EXPECT_EQ(overflow.queuedTransmissions(), 0U);
}

TEST(CanBusTest, NewlyEligibleSenderAdvancesFutureArbitrationFrontier) {
    fil::sim::EventLoop loop;
    fil::devices::VirtualCanBus bus("future", 500'000U,
        fil::devices::VirtualCanBus::DeliveryMode::timed, 4U);
    bus.setEventLoop(&loop);
    const auto future_sender = bus.attach("future", false, [](const auto&, auto) {});
    const auto ready_sender = bus.attach("ready", false, [](const auto&, auto) {});
    ASSERT_TRUE(future_sender && ready_sender);
    std::vector<std::uint32_t> transmitted;
    bus.setTraceCallback([&](const auto& record) {
        if (record.direction == fil::devices::CanTraceRecord::Direction::transmit)
            transmitted.push_back(record.frame.id);
    });
    fil::devices::CanFrame future; future.id = 1U;
    fil::devices::CanFrame ready; ready.id = 2U;
    ASSERT_TRUE(bus.send(future_sender.value(), future, 1'000'000U));
    ASSERT_TRUE(bus.send(ready_sender.value(), ready, 0U));
    static_cast<void>(loop.runDueEvents(0U));
    ASSERT_EQ(transmitted.size(), 1U);
    EXPECT_EQ(transmitted.front(), 2U);
}

TEST(CanBusTest, TimedQueueCarriesFractionalBitTimeRemainder) {
    fil::sim::EventLoop loop;
    fil::devices::VirtualCanBus bus("timed", 7U,
        fil::devices::VirtualCanBus::DeliveryMode::timed, 16U);
    bus.setEventLoop(&loop);
    std::vector<std::uint64_t> times;
    ASSERT_TRUE(bus.attach("receiver", false,
        [&](const auto&, const auto time) { times.push_back(time); }));
    fil::devices::CanFrame frame;
    frame.id = 0x123U;
    constexpr std::uint64_t bits = 48U;
    constexpr std::uint64_t expected_second = (2U * bits - 3U) * 1'000'000'000ULL / 7U;
    ASSERT_TRUE(bus.inject(frame, 0U));
    ASSERT_TRUE(bus.inject(frame, 0U));
    static_cast<void>(loop.runDueEvents(expected_second));
    ASSERT_EQ(times.size(), 2U);
    EXPECT_EQ(times[1], expected_second);
}

TEST(CanBusTest, DeferredCommandsStageOnWorkersAndReplayInDeterministicOrder) {
  fil::sim::EventLoop clock;
  fil::devices::VirtualCanBus bus("deferred");
  std::vector<std::uint32_t> received;
  const auto a = bus.attach("a", false, [](const auto &, auto) {});
  const auto b = bus.attach("b", false, [](const auto &, auto) {});
  ASSERT_TRUE(a && b);
  ASSERT_TRUE(bus.attach("receiver", false, [&](const auto &f, auto) {
    received.push_back(f.id);
  }));
  fil::devices::DeferredCanOutbox outbox_a, outbox_b;
  std::thread first([&] {
    fil::devices::DeferredCanScope scope(outbox_b, 2U, clock, 2U);
    fil::devices::CanFrame f;
    f.id = 0x200U;
    ASSERT_TRUE(bus.send(b.value(), f, 999U));
  });
  std::thread second([&] {
    fil::devices::DeferredCanScope scope(outbox_a, 1U, clock, 1U);
    fil::devices::CanFrame f;
    f.id = 0x100U;
    ASSERT_TRUE(bus.send(a.value(), f, 999U));
  });
  first.join();
  second.join();
  EXPECT_TRUE(received.empty());
  fil::devices::DeferredCanOutbox outbox;
  outbox.insert(outbox.end(), std::make_move_iterator(outbox_a.begin()),
                std::make_move_iterator(outbox_a.end()));
  outbox.insert(outbox.end(), std::make_move_iterator(outbox_b.begin()),
                std::make_move_iterator(outbox_b.end()));
  ASSERT_EQ(outbox.size(), 2U);
  std::sort(outbox.begin(), outbox.end(), [](const auto &lhs, const auto &rhs) {
    return std::tie(lhs.timestamp, lhs.board_index, lhs.lane_sequence) <
           std::tie(rhs.timestamp, rhs.board_index, rhs.lane_sequence);
  });
  for (auto &command : outbox)
    ASSERT_TRUE(command.replay());
  EXPECT_EQ(received, (std::vector<std::uint32_t>{0x100U, 0x200U}));
}

TEST(CanBusTest, DeferredCancellationIsOrderedAndDoesNotMutateBusUntilReplay) {
  fil::sim::EventLoop clock;
  fil::devices::VirtualCanBus bus(
      "deferred-cancel", 500'000U,
      fil::devices::VirtualCanBus::DeliveryMode::timed, 4U);
  bus.setEventLoop(&clock);
  const auto sender = bus.attach("sender", false, [](const auto &, auto) {});
  ASSERT_TRUE(sender);
  fil::devices::CanFrame frame;
  frame.id = 0x123U;
  ASSERT_TRUE(bus.send(sender.value(), frame, 0U));
  ASSERT_EQ(bus.queuedTransmissions(), 1U);
  fil::devices::DeferredCanOutbox outbox;
  {
    fil::devices::DeferredCanScope scope(outbox, 0U, clock, 0U);
    bus.cancelSender(sender.value());
  }
  EXPECT_EQ(bus.queuedTransmissions(), 1U);
  ASSERT_EQ(outbox.size(), 1U);
  EXPECT_EQ(outbox.front().operation,
            fil::devices::DeferredCanCommand::Operation::cancel_sender);
  ASSERT_TRUE(outbox.front().replay());
  EXPECT_EQ(bus.queuedTransmissions(), 0U);
}

TEST(CanBusTest, DeferredCommandsKeepIssuingTimesRatherThanWindowStart) {
    fil::sim::EventLoop clock;
    fil::devices::VirtualCanBus bus("deferred-times");
    const auto sender = bus.attach("sender", false, [](const auto&, auto) {});
    ASSERT_TRUE(sender);
    fil::devices::DeferredCanOutbox outbox;
    fil::devices::CanFrame frame;
    frame.id = 1U;
    {
        fil::devices::DeferredCanScope scope(outbox, 0U, clock, 0U);
        ASSERT_TRUE(bus.send(sender.value(), frame, 17U));
        static_cast<void>(clock.runOwnedEvents(0U, 50U));
        ASSERT_TRUE(bus.inject(frame, 75U));
        static_cast<void>(clock.runOwnedEvents(0U, 100U));
        bus.cancelSender(sender.value());
    }
    ASSERT_EQ(outbox.size(), 3U);
    EXPECT_EQ(outbox[0].timestamp, 17U);
    EXPECT_EQ(outbox[1].timestamp, 75U);
    EXPECT_EQ(outbox[2].timestamp, 100U);
    EXPECT_EQ(outbox[0].lane_sequence, 0U);
    EXPECT_EQ(outbox[2].lane_sequence, 2U);
}

TEST(CanBusTest, DeferredQueueAdmissionFailureInvokesCompletion) {
  fil::sim::EventLoop clock;
  fil::devices::VirtualCanBus bus(
      "deferred-full", 500'000U,
      fil::devices::VirtualCanBus::DeliveryMode::timed, 0U);
  bus.setEventLoop(&clock);
  bool completed = false;
  fil::devices::DeferredCanOutbox outbox;
  fil::devices::CanFrame frame;
  frame.id = 1U;
  {
    fil::devices::DeferredCanScope scope(outbox, 0U, clock, 0U);
    ASSERT_TRUE(bus.inject(frame, 123U,
                           [&](auto, bool success) { completed = !success; }));
  }
  ASSERT_EQ(outbox.size(), 1U);
  EXPECT_FALSE(outbox.front().replay());
  EXPECT_TRUE(completed);
  EXPECT_EQ(bus.queuedTransmissions(), 0U);
}

TEST(CanBusTest, ValidatesFramesAndAttachments) {
  fil::devices::VirtualCanBus bus("vehicle");
  const auto node =
      bus.attach("node", false, [](const auto &, std::uint64_t) {});
  EXPECT_TRUE(node.hasValue()) << "attaches validation node";
  EXPECT_TRUE(!bus.attach("node", false, [](const auto &, std::uint64_t) {}))
      << "rejects duplicate CAN node";
  fil::devices::CanFrame invalid;
  invalid.id = 0x800U;
  EXPECT_TRUE(node && !bus.send(node.value(), invalid, 0))
      << "rejects oversized standard CAN ID";
  EXPECT_TRUE(fil::devices::dlcToLength(15) == 64)
      << "maps CAN-FD DLC 15 to 64 bytes";
  EXPECT_TRUE(fil::devices::lengthToDlc(13) == 10)
      << "rounds payload length up to representable DLC";
}

} // namespace
