#pragma once

/** @file can_bus.hpp
 *  @brief Deterministic in-process classic CAN/CAN-FD broadcast bus.
 */

#include "fil/common/result.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace fil::devices {

/** @brief One normalized classic CAN or CAN-FD data frame. */
struct CanFrame {
    std::uint32_t id{0};     ///< 11-bit standard or 29-bit extended identifier.
    bool extended{false};    ///< Selects the 29-bit identifier format.
    bool fd{false};          ///< CAN-FD frame format flag.
    bool brs{false};         ///< CAN-FD bit-rate switching flag.
    std::uint8_t dlc{0};     ///< Encoded data-length code.
    std::array<std::uint8_t, 64> data{}; ///< Payload; bytes beyond decoded length are ignored.
};

/** @brief Converts a CAN DLC encoding to payload bytes. */
[[nodiscard]] std::uint8_t dlcToLength(std::uint8_t dlc) noexcept;

/** @brief Returns the smallest DLC capable of representing a payload length. */
[[nodiscard]] std::optional<std::uint8_t> lengthToDlc(std::size_t length) noexcept;

/** @brief Validates identifier, frame-format, and DLC relationships. */
[[nodiscard]] Result<void> validate(const CanFrame& frame);

/** @brief Stable record emitted for bus transmission and node reception. */
struct CanTraceRecord {
    enum class Direction { transmit, receive } direction{Direction::transmit}; ///< Record kind.
    std::uint64_t time_ns{0};  ///< Deterministic simulated timestamp.
    std::uint64_t sequence{0}; ///< Monotonic tie-break sequence.
    std::string node;           ///< Sending or receiving node name.
    CanFrame frame;             ///< Normalized frame value.
};

/**
 * @brief Synchronous deterministic broadcast medium shared by MCU instances.
 *
 * Nodes receive frames in attachment order. Delivery takes a snapshot of the
 * current callback list, so a callback may safely detach itself or another
 * node without invalidating the in-progress broadcast.
 */
class VirtualCanBus {
public:
    using NodeId = std::uint64_t; ///< Opaque attachment identifier.
    using ReceiveCallback = std::function<void(const CanFrame&, std::uint64_t)>;
    using TraceCallback = std::function<void(const CanTraceRecord&)>;

    /** @brief Creates a named bus with an informational nominal bit rate. */
    explicit VirtualCanBus(std::string name, std::uint32_t bitrate = 500000U);

    /** @brief Attaches one uniquely named receive callback. */
    [[nodiscard]] Result<NodeId> attach(
        std::string node,
        bool loopback,
        ReceiveCallback callback
    );

    /** @brief Removes an attachment; unknown IDs are harmless. */
    void detach(NodeId id) noexcept;

    /** @brief Broadcasts a frame from an attached node. */
    [[nodiscard]] Result<void> send(NodeId sender, const CanFrame& frame, std::uint64_t time_ns);

    /** @brief Injects an external frame into every attached node. */
    [[nodiscard]] Result<void> inject(const CanFrame& frame, std::uint64_t time_ns);

    /** @brief Sets an optional synchronous trace sink. */
    void setTraceCallback(TraceCallback callback) { trace_callback_ = std::move(callback); }

    [[nodiscard]] std::string_view name() const noexcept { return name_; }
    [[nodiscard]] std::uint32_t bitrate() const noexcept { return bitrate_; }
    [[nodiscard]] std::size_t nodeCount() const noexcept { return nodes_.size(); }

private:
    struct Node {
        NodeId id{0};
        std::string name;
        bool loopback{false};
        ReceiveCallback receive;
    };

    [[nodiscard]] Node* find(NodeId id) noexcept;
    void trace(CanTraceRecord::Direction direction, std::uint64_t time_ns, const std::string& node, const CanFrame& frame);
    void deliver(const CanFrame& frame, std::uint64_t time_ns, std::optional<NodeId> sender);

    std::string name_;
    std::uint32_t bitrate_{500000U};
    NodeId next_node_id_{1};
    std::uint64_t next_sequence_{0};
    std::vector<Node> nodes_;
    TraceCallback trace_callback_;
};

} // namespace fil::devices
