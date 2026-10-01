#include "fil/devices/can_bus.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace {

TEST(CanBusTest, BroadcastsDeterministically) {
    fil::devices::VirtualCanBus bus("vehicle");
    std::vector<int> deliveries;
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
    EXPECT_TRUE(deliveries == std::vector<int>({2})) << "skips sender without loopback";
    EXPECT_TRUE(trace.size() == 2 && trace[0].sequence < trace[1].sequence)
        << "traces TX then RX in stable order";

    deliveries.clear();
    EXPECT_TRUE(bus.inject(frame, 200U).hasValue()) << "injects external CAN frame";
    EXPECT_TRUE(deliveries == std::vector<int>({1, 2})) << "delivers injection in attachment order";
}

TEST(CanBusTest, DeliveryBarrierPrecedesObserversAndCanBeRestored) {
    fil::devices::VirtualCanBus bus("vehicle");
    std::vector<int> order;
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
    EXPECT_EQ(order, (std::vector<int>{1, 2, 2, 3}));
    auto barrier = bus.exchangeDeliveryBarrier(std::move(original));
    EXPECT_TRUE(barrier);
    order.clear();
    ASSERT_TRUE(bus.inject(frame, 200U));
    EXPECT_EQ(order, (std::vector<int>{2, 2, 2, 3}));
}

TEST(CanBusTest, ValidatesFramesAndAttachments) {
    fil::devices::VirtualCanBus bus("vehicle");
    const auto node = bus.attach("node", false, [](const auto&, std::uint64_t) {});
    EXPECT_TRUE(node.hasValue()) << "attaches validation node";
    EXPECT_TRUE(!bus.attach("node", false, [](const auto &, std::uint64_t) {}))
        << "rejects duplicate CAN node";
    fil::devices::CanFrame invalid;
    invalid.id = 0x800U;
    EXPECT_TRUE(node && !bus.send(node.value(), invalid, 0)) << "rejects oversized standard CAN ID";
    EXPECT_TRUE(fil::devices::dlcToLength(15) == 64) << "maps CAN-FD DLC 15 to 64 bytes";
    EXPECT_TRUE(fil::devices::lengthToDlc(13) == 10)
        << "rounds payload length up to representable DLC";
}

} // namespace
