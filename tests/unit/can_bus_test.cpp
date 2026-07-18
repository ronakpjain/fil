#include "fil/devices/can_bus.hpp"
#include "../test_support.hpp"

#include <cstdint>
#include <vector>

namespace {

void broadcastsDeterministically() {
    fil::devices::VirtualCanBus bus("vehicle");
    std::vector<int> deliveries;
    std::vector<fil::devices::CanTraceRecord> trace;
    bus.setTraceCallback([&](const auto& record) { trace.push_back(record); });
    const auto first = bus.attach("first", false, [&](const auto&, std::uint64_t) { deliveries.push_back(1); });
    const auto second = bus.attach("second", true, [&](const auto&, std::uint64_t) { deliveries.push_back(2); });
    fil::test::check(first && second, "attaches virtual CAN nodes");
    if (!first || !second) return;

    fil::devices::CanFrame frame;
    frame.id = 0x123U;
    frame.dlc = 2;
    frame.data[0] = 0xaaU;
    frame.data[1] = 0x55U;
    fil::test::check(bus.send(first.value(), frame, 100U).hasValue(), "sends valid CAN frame");
    fil::test::check(deliveries == std::vector<int>({2}), "skips sender without loopback");
    fil::test::check(trace.size() == 2 && trace[0].sequence < trace[1].sequence, "traces TX then RX in stable order");

    deliveries.clear();
    fil::test::check(bus.inject(frame, 200U).hasValue(), "injects external CAN frame");
    fil::test::check(deliveries == std::vector<int>({1, 2}), "delivers injection in attachment order");
}

void validatesFramesAndAttachments() {
    fil::devices::VirtualCanBus bus("vehicle");
    const auto node = bus.attach("node", false, [](const auto&, std::uint64_t) {});
    fil::test::check(node.hasValue(), "attaches validation node");
    fil::test::check(!bus.attach("node", false, [](const auto&, std::uint64_t) {}), "rejects duplicate CAN node");
    fil::devices::CanFrame invalid;
    invalid.id = 0x800U;
    fil::test::check(node && !bus.send(node.value(), invalid, 0), "rejects oversized standard CAN ID");
    fil::test::check(fil::devices::dlcToLength(15) == 64, "maps CAN-FD DLC 15 to 64 bytes");
    fil::test::check(fil::devices::lengthToDlc(13) == 10, "rounds payload length up to representable DLC");
}

} // namespace

void runCanBusTests() {
    broadcastsDeterministically();
    validatesFramesAndAttachments();
}
