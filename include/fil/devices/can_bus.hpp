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
#include <utility>
#include <memory>

namespace fil::sim { class EventLoop; }

namespace fil::devices {

/** One coordinator-replayable CAN mutation captured by a worker shard. */
struct DeferredCanCommand {
    enum class Operation { send, inject, cancel_sender };
    std::uint64_t timestamp{0};
    std::uint32_t board_index{0};
    std::uint64_t lane_sequence{0};
    Operation operation{Operation::send};
    std::function<Result<void>()> replay;
};

using DeferredCanOutbox = std::vector<DeferredCanCommand>;

/**
 * Thread-local staging scope. Commands are ordered by timestamp/board/lane sequence.
 * send/inject success accepts an outbox entry, not shared queue admission;
 * replay performs admission and reports rejection through the completion.
 * Attachments/configuration must remain immutable until all workers are parked.
 * Each command must be replayed at most once, with no staging scope active.
 */
class DeferredCanScope {
public:
    DeferredCanScope(DeferredCanOutbox& outbox, std::uint32_t board_index,
                     sim::EventLoop& clock, std::uint32_t owner);
    ~DeferredCanScope();
    DeferredCanScope(const DeferredCanScope&) = delete;
    DeferredCanScope& operator=(const DeferredCanScope&) = delete;
    [[nodiscard]] static bool active() noexcept;
    [[nodiscard]] static std::uint64_t timestamp() noexcept;
    [[nodiscard]] static std::uint32_t boardIndex() noexcept;
    [[nodiscard]] static std::uint64_t nextSequence() noexcept;
    struct State {
        DeferredCanOutbox* outbox;
        std::uint32_t board;
        sim::EventLoop* clock;
        std::uint32_t owner;
        std::uint64_t sequence{0};
    };
private:
    State* previous_{nullptr};
    State state_;
};

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

/** @brief Classic-CAN stuffed bit count from SOF through EOF (excludes intermission). */
[[nodiscard]] Result<std::uint32_t> classicCanStuffedFrameBits(const CanFrame& frame);

/** @brief Classic-CAN wire occupancy in nanoseconds, including 3-bit intermission. */
[[nodiscard]] Result<std::uint64_t> classicCanDurationNs(const CanFrame& frame,
                                                         std::uint32_t bitrate);

/** @brief Stable record emitted for bus transmission and node reception. */
struct CanTraceRecord {
    enum class Direction { transmit, receive, rejected } direction{Direction::transmit}; ///< Record kind.
    std::uint64_t time_ns{0};  ///< Deterministic simulated timestamp.
    std::uint64_t sequence{0}; ///< Monotonic tie-break sequence.
    std::string node;           ///< Sending or receiving node name.
    CanFrame frame;             ///< Normalized frame value.
};

/**
 * @brief Deterministic broadcast medium shared by MCU instances.
 *
 * Immediate mode broadcasts synchronously; timed mode arbitrates sender-head
 * requests on the associated event loop. Nodes receive frames in attachment order. The
 * delivery callback snapshot is revalidated before invocation, so detached
 * nodes are not called through stale references.
 */
class VirtualCanBus {
public:
    enum class DeliveryMode { immediate, timed, timed_fifo = timed }; // timed_fifo is a source-compatibility alias.
    using CompletionCallback = std::function<void(std::uint64_t, bool)>;
    using NodeId = std::uint64_t; ///< Opaque attachment identifier.
    using ReceiveCallback = std::function<void(const CanFrame&, std::uint64_t)>;
    using TraceCallback = std::function<void(const CanTraceRecord&)>;
    using DeliveryBarrier = std::function<void(std::uint64_t)>;

    /** @brief Creates a bus; immediate delivery is the compatibility default. */
    explicit VirtualCanBus(std::string name, std::uint32_t bitrate = 500000U,
                           DeliveryMode mode = DeliveryMode::immediate,
                           std::size_t queue_capacity = 256U);
    ~VirtualCanBus();
    VirtualCanBus(const VirtualCanBus&) = delete;
    VirtualCanBus& operator=(const VirtualCanBus&) = delete;

    /** @brief Associates the world's event loop for timed delivery. */
    void setEventLoop(sim::EventLoop* loop) noexcept { event_loop_ = loop; }
    /** @brief Cancels queued and in-flight transmissions owned by an attachment. */
    void cancelSender(NodeId sender) noexcept;

    /** @brief Attaches one uniquely named receive callback. */
    [[nodiscard]] Result<NodeId> attach(
        std::string node,
        bool loopback,
        ReceiveCallback callback
    );

    /** @brief Removes an attachment; unknown IDs are harmless. */
    void detach(NodeId id) noexcept;

    /** @brief Broadcasts a frame from an attached node. */
    [[nodiscard]] Result<void> send(NodeId sender, const CanFrame& frame, std::uint64_t time_ns,
                                    CompletionCallback completion = {});

    /** @brief Injects an external frame into every attached node. */
    [[nodiscard]] Result<void> inject(const CanFrame& frame, std::uint64_t time_ns,
                                      CompletionCallback completion = {});

    /** @brief Sets an optional synchronous trace sink. */
    void setTraceCallback(TraceCallback callback) { trace_callback_ = std::move(callback); }

    /** @brief Exchanges a pre-transmission observation barrier, before trace/receive callbacks. */
    [[nodiscard]] DeliveryBarrier exchangeDeliveryBarrier(DeliveryBarrier barrier) noexcept {
        return std::exchange(delivery_barrier_, std::move(barrier));
    }

    [[nodiscard]] std::string_view name() const noexcept { return name_; }
    [[nodiscard]] std::uint32_t bitrate() const noexcept { return bitrate_; }
    [[nodiscard]] std::size_t nodeCount() const noexcept { return nodes_.size(); }
    [[nodiscard]] DeliveryMode deliveryMode() const noexcept { return mode_; }
    [[nodiscard]] std::size_t queueCapacity() const noexcept { return queue_capacity_; }
    [[nodiscard]] std::size_t queuedTransmissions() const noexcept { return pending_.size() + (in_flight_ ? 1U : 0U); }

private:
    struct Node {
        NodeId id{0};
        std::string name;
        bool loopback{false};
        ReceiveCallback receive;
    };

    struct Transmission {
        CanFrame frame;
        std::optional<NodeId> sender;
        std::string sender_name;
        std::uint64_t requested_ns{0};
        std::uint64_t sequence{0};
        CompletionCallback completion;
    };

    /** @brief Explicit timed-mode courier state. */
    enum class TimedState : std::uint8_t {
        idle,        ///< No frame on the wire and no arbitration event armed.
        arbitrating, ///< An arbitration event is armed for the queued sender heads.
        in_flight,   ///< One frame occupies the wire until its completion event.
    };

    [[nodiscard]] Node* find(NodeId id) noexcept;
    [[nodiscard]] Result<void> submit(Transmission transmission);
    void scheduleArbitration();
    void arbitrate();
    void completeInFlight();
    static void arbitrationEvent(void* context);
    static void completionEvent(void* context);
    void trace(CanTraceRecord::Direction direction, std::uint64_t time_ns, const std::string& node, const CanFrame& frame);
    void deliver(const CanFrame& frame, std::uint64_t time_ns, std::optional<NodeId> sender);

    std::string name_;
    std::uint32_t bitrate_{500000U};
    NodeId next_node_id_{1};
    std::uint64_t next_sequence_{0};
    std::vector<Node> nodes_;
    TraceCallback trace_callback_;
    DeliveryBarrier delivery_barrier_;
    DeliveryMode mode_{DeliveryMode::immediate};
    std::size_t queue_capacity_{256U};
    sim::EventLoop* event_loop_{nullptr};
    std::vector<Transmission> pending_;
    std::optional<Transmission> in_flight_;
    std::uint64_t next_start_ns_{0};
    std::uint64_t fractional_numerator_{0};
    std::uint64_t in_flight_fraction_before_start_{0};
    std::uint64_t next_transmission_sequence_{0};
    std::uint64_t timed_event_id_{0};
    TimedState timed_state_{TimedState::idle};
};

} // namespace fil::devices
