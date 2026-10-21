#include "fil/devices/can_bus.hpp"
#include "fil/sim/event_loop.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

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

namespace {

constexpr std::uint16_t classic_crc_polynomial = 0x4599U;
constexpr std::uint16_t classic_crc_mask = 0x7fffU;

void appendBits(std::vector<bool>& bits, const std::uint32_t value, const unsigned width) {
    for (unsigned shift = width; shift > 0; --shift) {
        bits.push_back(((value >> (shift - 1U)) & 1U) != 0U);
    }
}

} // namespace

Result<std::uint32_t> classicCanStuffedFrameBits(const CanFrame& frame) {
    auto valid = validate(frame);
    if (!valid) return valid.error();
    if (frame.fd) {
        return Error{ErrorCategory::invalid_argument,
                     "classic CAN duration is undefined for CAN-FD frames", std::nullopt};
    }

    std::vector<bool> bits;
    bits.reserve(frame.extended ? 100U : 80U);
    bits.push_back(false); // SOF
    if (frame.extended) {
        appendBits(bits, (frame.id >> 18U) & 0x7ffU, 11U);
        bits.push_back(true); // SRR
        bits.push_back(true); // IDE
        appendBits(bits, frame.id & 0x3ffffU, 18U);
        bits.push_back(false); // RTR
        bits.push_back(false); // r1
        bits.push_back(false); // r0
    } else {
        appendBits(bits, frame.id, 11U);
        bits.push_back(false); // RTR
        bits.push_back(false); // IDE
        bits.push_back(false); // r0
    }
    appendBits(bits, frame.dlc, 4U);
    const std::uint8_t length = dlcToLength(frame.dlc);
    for (std::uint32_t byte = 0; byte < length; ++byte) {
        appendBits(bits, frame.data[byte], 8U);
    }

    std::uint16_t crc = 0;
    for (const bool bit : bits) {
        const bool feedback = bit != ((crc >> 14U) & 1U);
        crc = static_cast<std::uint16_t>((crc << 1U) & classic_crc_mask);
        if (feedback) crc ^= classic_crc_polynomial;
    }
    appendBits(bits, crc, 15U);

    std::uint32_t stuffed = static_cast<std::uint32_t>(bits.size());
    bool previous = bits.front();
    unsigned run = 1U;
    for (std::size_t index = 1; index < bits.size(); ++index) {
        if (bits[index] == previous) {
            ++run;
        } else {
            previous = bits[index];
            run = 1U;
        }
        if (run == 5U) {
            ++stuffed;
            previous = !previous;
            run = 1U;
        }
    }
    // CRC delimiter, ACK slot, ACK delimiter and EOF are not bit-stuffed.
    return stuffed + 10U;
}

Result<std::uint64_t> classicCanDurationNs(const CanFrame& frame,
                                            const std::uint32_t bitrate) {
    if (bitrate == 0U) {
        return Error{ErrorCategory::invalid_argument, "CAN bitrate must be nonzero", std::nullopt};
    }
    auto bits = classicCanStuffedFrameBits(frame);
    if (!bits) return bits.error();
    const std::uint64_t occupancy_bits = static_cast<std::uint64_t>(bits.value()) + 3U;
    if (occupancy_bits > std::numeric_limits<std::uint64_t>::max() / 1'000'000'000ULL) {
        return Error{ErrorCategory::invalid_argument, "CAN frame duration overflows nanoseconds", std::nullopt};
    }
    const std::uint64_t numerator = occupancy_bits * 1'000'000'000ULL;
    return numerator / bitrate + (numerator % bitrate != 0U ? 1U : 0U);
}

VirtualCanBus::VirtualCanBus(std::string name, const std::uint32_t bitrate,
                             const DeliveryMode mode, const std::size_t queue_capacity)
    : name_(std::move(name)), bitrate_(bitrate), mode_(mode), queue_capacity_(queue_capacity) {}

VirtualCanBus::~VirtualCanBus() {
    if (event_loop_ != nullptr && timed_event_id_ != 0U) {
        static_cast<void>(event_loop_->cancel(timed_event_id_));
    }
}

void VirtualCanBus::cancelSender(const NodeId sender) noexcept {
    pending_.erase(std::remove_if(pending_.begin(), pending_.end(), [sender](const auto& tx) {
        return tx.sender && *tx.sender == sender;
    }), pending_.end());
    if (in_flight_ && in_flight_->sender && *in_flight_->sender == sender) {
        if (event_loop_ != nullptr && timed_event_id_ != 0U)
            static_cast<void>(event_loop_->cancel(timed_event_id_));
        timed_event_id_ = 0U;
        in_flight_.reset();
        fractional_numerator_ = in_flight_fraction_before_start_;
        if (event_loop_ != nullptr) next_start_ns_ = event_loop_->now();
    } else if (!in_flight_ && timed_event_id_ != 0U && event_loop_ != nullptr) {
        // The deferred arbitration event may target a canceled head. Recompute
        // its deadline from the surviving sender heads.
        static_cast<void>(event_loop_->cancel(timed_event_id_));
        timed_event_id_ = 0U;
    }
    scheduleArbitration();
}

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
    cancelSender(id);
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
        NodeId id;
        std::string name;
        ReceiveCallback receive;
    };
    std::vector<Delivery> deliveries;
    deliveries.reserve(nodes_.size());
    for (const Node& node : nodes_) {
        if (sender && node.id == *sender && !node.loopback) continue;
        deliveries.push_back(Delivery{node.id, node.name, node.receive});
    }
    for (const Delivery& delivery : deliveries) {
        // A previous callback may have detached and destroyed this receiver.
        if (find(delivery.id) == nullptr) continue;
        trace(CanTraceRecord::Direction::receive, time_ns, delivery.name, frame);
        delivery.receive(frame, time_ns);
    }
}

Result<void> VirtualCanBus::submit(Transmission transmission) {
    if (mode_ == DeliveryMode::immediate) {
        if (delivery_barrier_) delivery_barrier_(transmission.requested_ns);
        trace(CanTraceRecord::Direction::transmit, transmission.requested_ns,
              transmission.sender_name, transmission.frame);
        deliver(transmission.frame, transmission.requested_ns, transmission.sender);
        if (transmission.completion
            && (!transmission.sender || find(*transmission.sender) != nullptr))
            transmission.completion(transmission.requested_ns, true);
        return {};
    }
    if (event_loop_ == nullptr)
        return Error{ErrorCategory::runtime, "timed CAN bus has no event loop", std::nullopt};
    if (bitrate_ == 0U) {
        trace(CanTraceRecord::Direction::rejected, transmission.requested_ns,
              transmission.sender_name, transmission.frame);
        return Error{ErrorCategory::invalid_argument, "CAN bitrate must be nonzero", std::nullopt};
    }
    if (transmission.frame.fd) {
        trace(CanTraceRecord::Direction::rejected, transmission.requested_ns,
              transmission.sender_name, transmission.frame);
        return Error{ErrorCategory::unsupported,
                     "timed CAN bus currently supports classic CAN frames only", std::nullopt};
    }
    if (queue_capacity_ == 0U || queuedTransmissions() >= queue_capacity_) {
        trace(CanTraceRecord::Direction::rejected, transmission.requested_ns,
              transmission.sender_name, transmission.frame);
        return Error{ErrorCategory::runtime, "CAN transmit queue full", std::nullopt};
    }
    transmission.sequence = next_transmission_sequence_++;
    pending_.push_back(std::move(transmission));
    // An idle bus may be waiting for a future sender head. A newly admitted
    // sender can become eligible earlier, so recompute the arbitration frontier.
    if (!in_flight_ && timed_event_id_ != 0U) {
        static_cast<void>(event_loop_->cancel(timed_event_id_));
        timed_event_id_ = 0U;
    }
    scheduleArbitration();
    return {};
}

namespace {
std::vector<bool> arbitrationKey(const CanFrame& frame) {
    std::vector<bool> key;
    const auto append = [&key](const std::uint32_t value, const unsigned width) {
        for (unsigned shift = width; shift > 0; --shift)
            key.push_back(((value >> (shift - 1U)) & 1U) != 0U);
    };
    if (!frame.extended) {
        append(frame.id, 11U);
        key.push_back(false); // RTR: data frame
        key.push_back(false); // IDE: standard format wins
    } else {
        append((frame.id >> 18U) & 0x7ffU, 11U);
        key.push_back(true); // SRR
        key.push_back(true); // IDE
        append(frame.id & 0x3ffffU, 18U);
        key.push_back(false); // RTR: data frame
    }
    return key;
}
} // namespace

void VirtualCanBus::scheduleArbitration() {
    if (mode_ != DeliveryMode::timed || in_flight_ || pending_.empty()
        || event_loop_ == nullptr || timed_event_id_ != 0U) return;
    std::uint64_t earliest = std::numeric_limits<std::uint64_t>::max();
    for (std::size_t i = 0; i < pending_.size(); ++i) {
        bool has_earlier_from_sender = false;
        for (std::size_t j = 0; j < i; ++j) {
            if (pending_[j].sender == pending_[i].sender) {
                has_earlier_from_sender = true;
                break;
            }
        }
        if (!has_earlier_from_sender)
            earliest = std::min(earliest, pending_[i].requested_ns);
    }
    const std::uint64_t at = std::max({next_start_ns_, earliest, event_loop_->now()});
    auto owner = event_loop_->useOwner(sim::shared_event_owner);
    timed_event_id_ = event_loop_->scheduleAt(at, [this] {
        timed_event_id_ = 0U;
        arbitrate();
    });
}

void VirtualCanBus::arbitrate() {
    if (mode_ != DeliveryMode::timed || in_flight_ || pending_.empty() || event_loop_ == nullptr) return;
    const std::uint64_t start = std::max(next_start_ns_, event_loop_->now());
    std::optional<std::size_t> winner;
    std::vector<bool> winner_key;
    for (std::size_t i = 0; i < pending_.size(); ++i) {
        bool has_earlier_from_sender = false;
        for (std::size_t j = 0; j < i; ++j) {
            if (pending_[j].sender == pending_[i].sender) {
                has_earlier_from_sender = true;
                break;
            }
        }
        if (has_earlier_from_sender || pending_[i].requested_ns > start) continue;
        const std::vector<bool> key = arbitrationKey(pending_[i].frame);
        if (!winner || std::lexicographical_compare(key.begin(), key.end(),
                                                     winner_key.begin(), winner_key.end())
            || (key == winner_key && pending_[i].sequence < pending_[*winner].sequence)) {
            winner = i;
            winner_key = key;
        }
    }
    if (!winner) {
        scheduleArbitration();
        return;
    }
    in_flight_ = std::move(pending_[*winner]);
    pending_.erase(pending_.begin() + static_cast<std::ptrdiff_t>(*winner));
    auto bits = classicCanStuffedFrameBits(in_flight_->frame);
    if (!bits || bitrate_ == 0U) {
        Transmission failed = std::move(*in_flight_);
        in_flight_.reset();
        if (failed.completion) failed.completion(start, false);
        scheduleArbitration();
        return;
    }
    const std::uint64_t prior_fraction = fractional_numerator_;
    const std::uint64_t frame_numerator = static_cast<std::uint64_t>(bits.value())
        * 1'000'000'000ULL + fractional_numerator_;
    const std::uint64_t frame_duration = frame_numerator / bitrate_;
    fractional_numerator_ = frame_numerator % bitrate_;
    const std::uint64_t intermission_numerator = 3'000'000'000ULL + fractional_numerator_;
    const std::uint64_t intermission = intermission_numerator / bitrate_;
    fractional_numerator_ = intermission_numerator % bitrate_;
    if (start > std::numeric_limits<std::uint64_t>::max() - frame_duration
        || start + frame_duration > std::numeric_limits<std::uint64_t>::max() - intermission) {
        fractional_numerator_ = prior_fraction;
        Transmission failed = std::move(*in_flight_);
        in_flight_.reset();
        if (failed.completion) failed.completion(start, false);
        scheduleArbitration();
        return;
    }
    in_flight_fraction_before_start_ = prior_fraction;
    next_start_ns_ = start + frame_duration + intermission;
    trace(CanTraceRecord::Direction::transmit, start, in_flight_->sender_name, in_flight_->frame);
    auto owner = event_loop_->useOwner(sim::shared_event_owner);
    timed_event_id_ = event_loop_->scheduleAt(start + frame_duration,
                                               [this] { completeInFlight(); });
}

void VirtualCanBus::completeInFlight() {
    timed_event_id_ = 0U;
    if (!in_flight_) return;
    Transmission completed = std::move(*in_flight_);
    in_flight_.reset();
    const std::uint64_t at = event_loop_ != nullptr ? event_loop_->now() : next_start_ns_;
    if (delivery_barrier_) delivery_barrier_(at);
    deliver(completed.frame, at, completed.sender);
    if (completed.completion
        && (!completed.sender || find(*completed.sender) != nullptr)) {
        completed.completion(at, true);
    }
    scheduleArbitration();
}

Result<void> VirtualCanBus::send(
    const NodeId sender,
    const CanFrame& frame,
    const std::uint64_t time_ns,
    CompletionCallback completion
) {
    auto valid = validate(frame);
    if (!valid) return valid.error();
    Node* node = find(sender);
    if (node == nullptr) {
        return Error{ErrorCategory::invalid_argument, "CAN sender is not attached", std::nullopt};
    }
    return submit(Transmission{frame, sender, node->name, time_ns, 0U, std::move(completion)});
}

Result<void> VirtualCanBus::inject(const CanFrame& frame, const std::uint64_t time_ns,
                                   CompletionCallback completion) {
    auto valid = validate(frame);
    if (!valid) return valid.error();
    return submit(Transmission{frame, std::nullopt, "external", time_ns, 0U,
                              std::move(completion)});
}

} // namespace fil::devices
