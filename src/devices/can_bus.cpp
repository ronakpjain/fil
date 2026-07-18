#include "fil/devices/can_bus.hpp"

#include <algorithm>
#include <utility>

namespace fil::devices {

std::uint8_t dlcToLength(const std::uint8_t dlc) noexcept {
    constexpr std::array<std::uint8_t, 16> lengths{
        0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64,
    };
    return lengths[dlc & 0x0fU];
}

std::optional<std::uint8_t> lengthToDlc(const std::size_t length) noexcept {
    for (std::uint8_t dlc = 0; dlc < 16U; ++dlc) {
        if (length <= dlcToLength(dlc)) return dlc;
    }
    return std::nullopt;
}

Result<void> validate(const CanFrame& frame) {
    if (frame.dlc > 15U) {
        return Error{ErrorCategory::invalid_argument, "CAN DLC exceeds 15", std::nullopt};
    }
    if (frame.extended ? frame.id > 0x1fffffffU : frame.id > 0x7ffU) {
        return Error{ErrorCategory::invalid_argument, "CAN identifier exceeds selected format", std::nullopt};
    }
    if (!frame.fd && frame.dlc > 8U) {
        return Error{ErrorCategory::invalid_argument, "classic CAN frame payload exceeds 8 bytes", std::nullopt};
    }
    if (!frame.fd && frame.brs) {
        return Error{ErrorCategory::invalid_argument, "classic CAN frame cannot enable bit-rate switching", std::nullopt};
    }
    return {};
}

VirtualCanBus::VirtualCanBus(std::string name, const std::uint32_t bitrate)
    : name_(std::move(name)), bitrate_(bitrate) {}

Result<VirtualCanBus::NodeId> VirtualCanBus::attach(
    std::string node,
    const bool loopback,
    ReceiveCallback callback
) {
    if (node.empty()) {
        return Error{ErrorCategory::invalid_argument, "CAN node name is empty", std::nullopt};
    }
    if (!callback) {
        return Error{ErrorCategory::invalid_argument, "CAN node receive callback is empty", std::nullopt};
    }
    const auto duplicate = std::find_if(nodes_.begin(), nodes_.end(), [&](const Node& existing) {
        return existing.name == node;
    });
    if (duplicate != nodes_.end()) {
        return Error{ErrorCategory::invalid_argument, "duplicate CAN node name '" + node + "'", std::nullopt};
    }
    const NodeId id = next_node_id_++;
    nodes_.push_back(Node{id, std::move(node), loopback, std::move(callback)});
    return id;
}

void VirtualCanBus::detach(const NodeId id) noexcept {
    nodes_.erase(std::remove_if(nodes_.begin(), nodes_.end(), [=](const Node& node) {
        return node.id == id;
    }), nodes_.end());
}

VirtualCanBus::Node* VirtualCanBus::find(const NodeId id) noexcept {
    const auto found = std::find_if(nodes_.begin(), nodes_.end(), [=](const Node& node) {
        return node.id == id;
    });
    return found == nodes_.end() ? nullptr : &*found;
}

void VirtualCanBus::trace(
    const CanTraceRecord::Direction direction,
    const std::uint64_t time_ns,
    const std::string& node,
    const CanFrame& frame
) {
    if (trace_callback_) {
        trace_callback_(CanTraceRecord{direction, time_ns, next_sequence_++, node, frame});
    } else {
        ++next_sequence_;
    }
}

void VirtualCanBus::deliver(
    const CanFrame& frame,
    const std::uint64_t time_ns,
    const std::optional<NodeId> sender
) {
    struct Delivery {
        std::string name;
        ReceiveCallback receive;
    };
    std::vector<Delivery> deliveries;
    deliveries.reserve(nodes_.size());
    for (const Node& node : nodes_) {
        if (sender && node.id == *sender && !node.loopback) continue;
        deliveries.push_back(Delivery{node.name, node.receive});
    }
    for (const Delivery& delivery : deliveries) {
        trace(CanTraceRecord::Direction::receive, time_ns, delivery.name, frame);
        delivery.receive(frame, time_ns);
    }
}

Result<void> VirtualCanBus::send(
    const NodeId sender,
    const CanFrame& frame,
    const std::uint64_t time_ns
) {
    auto valid = validate(frame);
    if (!valid) return valid.error();
    Node* node = find(sender);
    if (node == nullptr) {
        return Error{ErrorCategory::invalid_argument, "CAN sender is not attached", std::nullopt};
    }
    const std::string sender_name = node->name;
    trace(CanTraceRecord::Direction::transmit, time_ns, sender_name, frame);
    deliver(frame, time_ns, sender);
    return {};
}

Result<void> VirtualCanBus::inject(const CanFrame& frame, const std::uint64_t time_ns) {
    auto valid = validate(frame);
    if (!valid) return valid.error();
    trace(CanTraceRecord::Direction::transmit, time_ns, "external", frame);
    deliver(frame, time_ns, std::nullopt);
    return {};
}

} // namespace fil::devices
