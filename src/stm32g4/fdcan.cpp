#include "fil/stm32g4/fdcan.hpp"

#include "fil/common/numeric.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <utility>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t crel = 0x000U;
constexpr std::uint32_t endn = 0x004U;
constexpr std::uint32_t dbtp = 0x00cU;
constexpr std::uint32_t cccr = FdcanPeripheral::cccrOffset;
constexpr std::uint32_t nbtp = 0x01cU;
constexpr std::uint32_t tocc = 0x028U;
constexpr std::uint32_t tocv = 0x02cU;
constexpr std::uint32_t ecr = 0x040U;
constexpr std::uint32_t psr = 0x044U;
constexpr std::uint32_t ir = FdcanPeripheral::irOffset;
constexpr std::uint32_t ie = FdcanPeripheral::ieOffset;
constexpr std::uint32_t ils = FdcanPeripheral::ilsOffset;
constexpr std::uint32_t ile = FdcanPeripheral::ileOffset;
constexpr std::uint32_t rxgfc = FdcanPeripheral::rxgfcOffset;
constexpr std::uint32_t xidam = 0x084U;
constexpr std::uint32_t hpms = 0x088U;
constexpr std::uint32_t rxf0s = FdcanPeripheral::rxf0sOffset;
constexpr std::uint32_t rxf0a = FdcanPeripheral::rxf0aOffset;
constexpr std::uint32_t rxf1s = 0x098U;
constexpr std::uint32_t rxf1a = 0x09cU;
constexpr std::uint32_t txbc = 0x0c0U;
constexpr std::uint32_t txfqs = FdcanPeripheral::txfqsOffset;
constexpr std::uint32_t txbrp = 0x0c8U;
constexpr std::uint32_t txbar = FdcanPeripheral::txbarOffset;
constexpr std::uint32_t txbcr = 0x0d0U;
constexpr std::uint32_t txbto = FdcanPeripheral::txbtoOffset;
constexpr std::uint32_t txbcf = 0x0d8U;
constexpr std::uint32_t txbtie = FdcanPeripheral::txbtieOffset;
constexpr std::uint32_t txefs = 0x0e4U;
constexpr std::uint32_t txefa = 0x0e8U;

constexpr std::uint32_t cccr_writable_mask = 0x0000f3f7U;
constexpr std::uint32_t interrupt_mask = 0x00ffffffU;
constexpr std::uint32_t tx_buffer_mask = 0x7U;
constexpr std::uint32_t rx_fifo0_full = 1U << 24U;
constexpr std::uint32_t rx_fifo0_lost = 1U << 25U;
constexpr std::uint32_t rx_fifo0_overwrite = 1U << 9U;
constexpr std::uint32_t interrupt_protocol_arbitration_error = 1U << 21U;

constexpr std::array<std::uint32_t, 7> interrupt_groups{
    0x000007U, // RX FIFO 0
    0x000038U, // RX FIFO 1
    0x0001c0U, // high-priority message and TX status
    0x001e00U, // TX FIFO/event FIFO
    0x00e000U, // timestamp, message RAM, timeout
    0x030000U, // error logging and passive state
    0xfc0000U, // warning, bus-off, watchdog, protocol/access errors
};

unsigned int checkedControllerIndex(const unsigned int instance_number) {
    if (instance_number == 0U || instance_number > FdcanMessageRam::controllerCount) {
        throw std::invalid_argument("FDCAN instance number must be in the range 1..3");
    }
    return instance_number - 1U;
}

std::string defaultControllerName(const unsigned int instance_number) {
    return "FDCAN" + std::to_string(instance_number);
}

bool standardFilterMatches(const std::uint32_t filter_type, const std::uint32_t identifier,
                           const std::uint32_t identifier1,
                           const std::uint32_t identifier2) noexcept {
    switch (filter_type) {
    case 0U:
        return identifier >= identifier1 && identifier <= identifier2;
    case 1U:
        return identifier == identifier1 || identifier == identifier2;
    case 2U:
        return (identifier & identifier2) == (identifier1 & identifier2);
    default:
        return false;
    }
}

bool extendedFilterMatches(const std::uint32_t filter_type, const std::uint32_t identifier,
                           const std::uint32_t identifier1,
                           const std::uint32_t identifier2) noexcept {
    switch (filter_type) {
    case 0U:
    case 3U:
        return identifier >= identifier1 && identifier <= identifier2;
    case 1U:
        return identifier == identifier1 || identifier == identifier2;
    case 2U:
        return (identifier & identifier2) == (identifier1 & identifier2);
    default:
        return false;
    }
}

bool actionStoresInFifo0(const std::uint32_t action) noexcept {
    return action == 1U || action == 5U;
}

} // namespace

FdcanMessageRam::FdcanMessageRam(std::string name) : name_(std::move(name)) {}

mem::BusFault FdcanMessageRam::fault(const std::uint32_t offset, const mem::AccessSize size,
                                     const mem::AccessContext& context, std::string message) const {
    return mem::BusFault{
        mem::BusFaultReason::device_error,
        baseAddress + offset,
        size,
        context,
        name_,
        std::move(message),
    };
}

mem::MemoryResult<std::uint64_t> FdcanMessageRam::read(const std::uint32_t offset,
                                                       const mem::AccessSize size,
                                                       const mem::AccessContext& context) {
    const std::uint32_t width = mem::byteCount(size);
    if (!mem::validAccessSize(size) || !rangeFits(offset, width, sizeBytes)) {
        return fault(offset, size, context, "FDCAN message RAM read is outside the shared RAM");
    }

    std::uint64_t value = 0;
    for (std::uint32_t index = 0; index < width; ++index) {
        value |= static_cast<std::uint64_t>(bytes_[offset + index]) << (index * 8U);
    }
    return value & mem::accessWidthMask(size);
}

mem::MemoryResult<std::uint64_t> FdcanMessageRam::write(const std::uint32_t offset,
                                                        const mem::AccessSize size,
                                                        const std::uint64_t value,
                                                        const mem::AccessContext& context) {
    const std::uint32_t width = mem::byteCount(size);
    if (!mem::validAccessSize(size) || !rangeFits(offset, width, sizeBytes)) {
        return fault(offset, size, context, "FDCAN message RAM write is outside the shared RAM");
    }

    for (std::uint32_t index = 0; index < width; ++index) {
        bytes_[offset + index] = static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU);
    }
    return std::uint64_t{0};
}

void FdcanMessageRam::reset() noexcept { bytes_.fill(0); }

std::optional<std::uint32_t> FdcanMessageRam::absoluteOffset(const unsigned int controller_index,
                                                             const std::uint32_t controller_offset,
                                                             const std::uint32_t width) noexcept {
    if (controller_index >= controllerCount || width == 0U ||
        controller_offset > controllerStride || width > controllerStride - controller_offset) {
        return std::nullopt;
    }
    return static_cast<std::uint32_t>(controller_index) * controllerStride + controller_offset;
}

std::uint32_t FdcanMessageRam::loadWord(const unsigned int controller_index,
                                        const std::uint32_t controller_offset) const noexcept {
    const auto absolute = absoluteOffset(controller_index, controller_offset, 4U);
    if (!absolute.has_value()) {
        return 0;
    }
    const std::uint32_t offset = *absolute;
    return static_cast<std::uint32_t>(bytes_[offset]) |
           (static_cast<std::uint32_t>(bytes_[offset + 1U]) << 8U) |
           (static_cast<std::uint32_t>(bytes_[offset + 2U]) << 16U) |
           (static_cast<std::uint32_t>(bytes_[offset + 3U]) << 24U);
}

void FdcanMessageRam::storeWord(const unsigned int controller_index,
                                const std::uint32_t controller_offset,
                                const std::uint32_t value) noexcept {
    const auto absolute = absoluteOffset(controller_index, controller_offset, 4U);
    if (!absolute.has_value()) {
        return;
    }
    const std::uint32_t offset = *absolute;
    bytes_[offset] = static_cast<std::uint8_t>(value & 0xffU);
    bytes_[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
    bytes_[offset + 2U] = static_cast<std::uint8_t>((value >> 16U) & 0xffU);
    bytes_[offset + 3U] = static_cast<std::uint8_t>((value >> 24U) & 0xffU);
}

void FdcanMessageRam::clearRange(const unsigned int controller_index, const std::uint32_t offset,
                                 const std::uint32_t size) noexcept {
    const auto absolute = absoluteOffset(controller_index, offset, size);
    if (!absolute.has_value()) {
        return;
    }
    const auto begin = bytes_.begin() + static_cast<std::ptrdiff_t>(*absolute);
    std::fill(begin, begin + static_cast<std::ptrdiff_t>(size), std::uint8_t{0});
}

FdcanPeripheral::FdcanPeripheral(const Instance instance, FdcanMessageRam& message_ram,
                                 sim::EventLoop* const event_loop, sim::TraceRecorder* const trace)
    : FdcanPeripheral(static_cast<unsigned int>(instance), message_ram, event_loop, trace) {}

FdcanPeripheral::FdcanPeripheral(const unsigned int instance_number, FdcanMessageRam& message_ram,
                                 sim::EventLoop* const event_loop, sim::TraceRecorder* const trace)
    : FdcanPeripheral(defaultControllerName(instance_number), instance_number, message_ram,
                      event_loop, trace) {}

FdcanPeripheral::FdcanPeripheral(std::string name, const unsigned int instance_number,
                                 FdcanMessageRam& message_ram, sim::EventLoop* const event_loop,
                                 sim::TraceRecorder* const trace)
    : RegisterPeripheral(std::move(name), registerBlockSize, event_loop, trace),
      message_ram_(message_ram), instance_index_(checkedControllerIndex(instance_number)) {
    setResetValue(crel, 0x32141218U);
    setResetValue(endn, 0x87654321U);
    setResetValue(dbtp, 0x00000a33U);
    setResetValue(cccr, cccrInit);
    setResetValue(nbtp, 0x06000a03U);
    setResetValue(tocc, 0xffff0000U);
    setResetValue(tocv, 0x0000ffffU);
    setResetValue(psr, 0x00000707U);
    setResetValue(xidam, 0x1fffffffU);
    reset();
}

FdcanPeripheral::~FdcanPeripheral() { detachBus(); }

Result<void> FdcanPeripheral::attachBus(devices::VirtualCanBus& bus, std::string node_name,
                                        const bool loopback) {
    detachBus();
    auto attached = bus.attach(std::move(node_name), loopback,
                               [this](const devices::CanFrame& frame, const std::uint64_t time_ns) {
                                   static_cast<void>(receiveFrame(frame, time_ns));
                               });
    if (!attached) {
        return attached.error();
    }
    bus_ = &bus;
    bus_node_id_ = attached.value();
    return {};
}

void FdcanPeripheral::detachBus() noexcept {
    if (bus_ != nullptr && bus_node_id_ != 0U) {
        bus_->detach(bus_node_id_);
    }
    bus_ = nullptr;
    bus_node_id_ = 0;
}

void FdcanPeripheral::setInterruptCallback(InterruptCallback callback) {
    interrupt_callback_ = std::move(callback);
    interrupt_line_asserted_.fill(false);
    updateInterruptLines();
}

void FdcanPeripheral::setInterruptLineCallback(const unsigned int line,
                                               LineInterruptCallback callback) {
    if (line >= line_interrupt_callbacks_.size()) {
        return;
    }
    line_interrupt_callbacks_[line] = std::move(callback);
    interrupt_line_asserted_[line] = false;
    updateInterruptLines();
}

bool FdcanPeripheral::operational() const noexcept {
    const std::uint32_t control = registerValue(cccr);
    return (control & (cccrInit | cccrCsr | cccrCsa)) == 0U;
}

FdcanPeripheral::FilterDecision
FdcanPeripheral::filter(const devices::CanFrame& frame) const noexcept {
    const std::uint32_t global_filter = registerValue(rxgfc);

    if (!frame.extended) {
        const std::uint32_t count =
            std::min((global_filter >> 16U) & 0x1fU, FdcanMessageRam::standardFilterCount);
        for (std::uint32_t index = 0; index < count; ++index) {
            const std::uint32_t element = message_ram_.loadWord(
                instance_index_, FdcanMessageRam::standardFilterOffset +
                                     index * FdcanMessageRam::standardFilterElementSize);
            const std::uint32_t action = (element >> 27U) & 0x7U;
            const std::uint32_t filter_type = element >> 30U;
            if (action == 0U || filter_type == 3U) {
                continue;
            }
            const std::uint32_t identifier1 = (element >> 16U) & 0x7ffU;
            const std::uint32_t identifier2 = element & 0x7ffU;
            if (standardFilterMatches(filter_type, frame.id, identifier1, identifier2)) {
                return FilterDecision{
                    actionStoresInFifo0(action),
                    false,
                    static_cast<std::uint8_t>(index),
                };
            }
        }
        const std::uint32_t non_matching_action = (global_filter >> 4U) & 0x3U;
        return FilterDecision{non_matching_action == 0U, true, 0x7fU};
    }

    const std::uint32_t count =
        std::min((global_filter >> 24U) & 0xfU, FdcanMessageRam::extendedFilterCount);
    for (std::uint32_t index = 0; index < count; ++index) {
        const std::uint32_t offset = FdcanMessageRam::extendedFilterOffset +
                                     index * FdcanMessageRam::extendedFilterElementSize;
        const std::uint32_t word0 = message_ram_.loadWord(instance_index_, offset);
        const std::uint32_t word1 = message_ram_.loadWord(instance_index_, offset + 4U);
        const std::uint32_t action = word0 >> 29U;
        if (action == 0U) {
            continue;
        }
        const std::uint32_t filter_type = word1 >> 30U;
        const std::uint32_t identifier1 = word0 & 0x1fffffffU;
        const std::uint32_t identifier2 = word1 & 0x1fffffffU;
        if (extendedFilterMatches(filter_type, frame.id, identifier1, identifier2)) {
            return FilterDecision{
                actionStoresInFifo0(action),
                false,
                static_cast<std::uint8_t>(index),
            };
        }
    }
    const std::uint32_t non_matching_action = (global_filter >> 2U) & 0x3U;
    return FilterDecision{non_matching_action == 0U, true, 0x7fU};
}

bool FdcanPeripheral::receiveFrame(const devices::CanFrame& frame, const std::uint64_t time_ns) {
    if (!operational() || !devices::validate(frame)) {
        return false;
    }
    const FilterDecision decision = filter(frame);
    if (!decision.accepted) {
        return false;
    }

    bool overwritten = false;
    if (rx_fill_level_ >= FdcanMessageRam::rxFifoElementCount) {
        rx_message_lost_ = true;
        if ((registerValue(rxgfc) & rx_fifo0_overwrite) == 0U) {
            updateRxFifo0Status();
            setInterruptFlags(interruptRxFifo0Full | interruptRxFifo0Lost);
            return false;
        }
        rx_get_index_ = static_cast<std::uint8_t>((static_cast<unsigned int>(rx_get_index_) + 1U) %
                                                  FdcanMessageRam::rxFifoElementCount);
        --rx_fill_level_;
        overwritten = true;
    }

    const std::uint32_t element_offset =
        FdcanMessageRam::rxFifo0Offset +
        static_cast<std::uint32_t>(rx_put_index_) * FdcanMessageRam::rxFifoElementSize;
    message_ram_.clearRange(instance_index_, element_offset, FdcanMessageRam::rxFifoElementSize);

    const std::uint32_t word0 =
        frame.extended ? ((frame.id & 0x1fffffffU) | (1U << 30U)) : ((frame.id & 0x7ffU) << 18U);
    std::uint32_t word1 = static_cast<std::uint32_t>(time_ns & 0xffffU) |
                          (static_cast<std::uint32_t>(frame.dlc & 0x0fU) << 16U) |
                          (frame.brs ? (1U << 20U) : 0U) | (frame.fd ? (1U << 21U) : 0U) |
                          (static_cast<std::uint32_t>(decision.index & 0x7fU) << 24U);
    if (decision.non_matching) {
        word1 |= 1U << 31U;
    }
    message_ram_.storeWord(instance_index_, element_offset, word0);
    message_ram_.storeWord(instance_index_, element_offset + 4U, word1);

    const std::uint32_t payload_length = devices::dlcToLength(frame.dlc);
    for (std::uint32_t byte_offset = 0; byte_offset < payload_length; byte_offset += 4U) {
        std::uint32_t data_word = 0;
        for (std::uint32_t byte = 0; byte < 4U && byte_offset + byte < payload_length; ++byte) {
            data_word |= static_cast<std::uint32_t>(frame.data[byte_offset + byte]) << (byte * 8U);
        }
        message_ram_.storeWord(instance_index_, element_offset + 8U + byte_offset, data_word);
    }

    rx_put_index_ = static_cast<std::uint8_t>((static_cast<unsigned int>(rx_put_index_) + 1U) %
                                              FdcanMessageRam::rxFifoElementCount);
    ++rx_fill_level_;
    updateRxFifo0Status();

    std::uint32_t flags = interruptRxFifo0New;
    if (rx_fill_level_ == FdcanMessageRam::rxFifoElementCount) {
        flags |= interruptRxFifo0Full;
    }
    if (overwritten) {
        flags |= interruptRxFifo0Lost;
    }
    setInterruptFlags(flags);
    traceEvent("can_rx", {
                             {"id", std::to_string(frame.id)},
                             {"dlc", std::to_string(frame.dlc)},
                             {"length", std::to_string(payload_length)},
                         });
    return true;
}

bool FdcanPeripheral::transmitBuffer(const unsigned int buffer_index) {
    if (buffer_index >= FdcanMessageRam::txFifoElementCount) {
        return false;
    }
    const std::uint32_t element_offset =
        FdcanMessageRam::txFifoOffset +
        static_cast<std::uint32_t>(buffer_index) * FdcanMessageRam::txFifoElementSize;
    const std::uint32_t word0 = message_ram_.loadWord(instance_index_, element_offset);
    const std::uint32_t word1 = message_ram_.loadWord(instance_index_, element_offset + 4U);

    devices::CanFrame frame;
    frame.extended = (word0 & (1U << 30U)) != 0U;
    frame.id = frame.extended ? (word0 & 0x1fffffffU) : ((word0 >> 18U) & 0x7ffU);
    frame.dlc = static_cast<std::uint8_t>((word1 >> 16U) & 0x0fU);
    frame.brs = (word1 & (1U << 20U)) != 0U;
    frame.fd = (word1 & (1U << 21U)) != 0U;
    const std::uint32_t payload_length = devices::dlcToLength(frame.dlc);
    for (std::uint32_t byte_offset = 0; byte_offset < payload_length; byte_offset += 4U) {
        const std::uint32_t data_word =
            message_ram_.loadWord(instance_index_, element_offset + 8U + byte_offset);
        for (std::uint32_t byte = 0; byte < 4U && byte_offset + byte < payload_length; ++byte) {
            frame.data[byte_offset + byte] =
                static_cast<std::uint8_t>((data_word >> (byte * 8U)) & 0xffU);
        }
    }

    if (!devices::validate(frame)) {
        setInterruptFlags(interrupt_protocol_arbitration_error);
        return false;
    }
    if (bus_ != nullptr) {
        Result<void> sent;
        if (sim::EventLoop* const loop = eventLoop()) {
            auto shared_scope = loop->useOwner(sim::shared_event_owner);
            sent = bus_->send(bus_node_id_, frame, currentTime());
        } else {
            sent = bus_->send(bus_node_id_, frame, currentTime());
        }
        if (!sent) {
            setInterruptFlags(interrupt_protocol_arbitration_error);
            return false;
        }
    }

    const std::uint32_t buffer_bit = 1U << buffer_index;
    setRegister(txbrp, registerValue(txbrp) & ~buffer_bit);
    setRegister(txbto, registerValue(txbto) | buffer_bit);
    tx_get_index_ =
        static_cast<std::uint8_t>((buffer_index + 1U) % FdcanMessageRam::txFifoElementCount);
    tx_put_index_ = tx_get_index_;
    updateTxFifoStatus();
    if ((registerValue(txbtie) & buffer_bit) != 0U) {
        setInterruptFlags(interruptTransmissionComplete);
    }
    traceEvent("can_tx", {
                             {"id", std::to_string(frame.id)},
                             {"dlc", std::to_string(frame.dlc)},
                             {"length", std::to_string(payload_length)},
                         });
    return true;
}

void FdcanPeripheral::acknowledgeRxFifo0(const unsigned int acknowledged_index) {
    if (rx_fill_level_ == 0U || acknowledged_index >= FdcanMessageRam::rxFifoElementCount) {
        return;
    }
    const unsigned int distance = (acknowledged_index + FdcanMessageRam::rxFifoElementCount -
                                   static_cast<unsigned int>(rx_get_index_)) %
                                      FdcanMessageRam::rxFifoElementCount +
                                  1U;
    if (distance > rx_fill_level_) {
        return;
    }
    rx_fill_level_ = static_cast<std::uint8_t>(rx_fill_level_ - distance);
    rx_get_index_ =
        static_cast<std::uint8_t>((acknowledged_index + 1U) % FdcanMessageRam::rxFifoElementCount);
    updateRxFifo0Status();
}

void FdcanPeripheral::updateRxFifo0Status() noexcept {
    std::uint32_t status = rx_fill_level_ | (static_cast<std::uint32_t>(rx_get_index_) << 8U) |
                           (static_cast<std::uint32_t>(rx_put_index_) << 16U);
    if (rx_fill_level_ == FdcanMessageRam::rxFifoElementCount) {
        status |= rx_fifo0_full;
    }
    if (rx_message_lost_) {
        status |= rx_fifo0_lost;
    }
    setRegister(rxf0s, status);
}

void FdcanPeripheral::updateTxFifoStatus() noexcept {
    const bool queue_mode = (registerValue(txbc) & (1U << 24U)) != 0U;
    std::uint32_t status = static_cast<std::uint32_t>(tx_put_index_) << 16U;
    if (!queue_mode) {
        status |=
            FdcanMessageRam::txFifoElementCount | (static_cast<std::uint32_t>(tx_get_index_) << 8U);
    }
    setRegister(txfqs, status);
}

void FdcanPeripheral::setInterruptFlags(const std::uint32_t flags) {
    const std::uint32_t raised = flags & interrupt_mask;
    setRegister(ir, registerValue(ir) | raised);
    const std::uint32_t enabled = raised & registerValue(ie);
    const std::uint32_t selections = registerValue(ils);
    for (std::size_t group = 0; group < interrupt_groups.size(); ++group) {
        if ((enabled & interrupt_groups[group]) == 0U) {
            continue;
        }
        const unsigned int line = (selections & (1U << group)) == 0U ? 0U : 1U;
        interrupt_line_asserted_[line] = false;
    }
    updateInterruptLines();
}

bool FdcanPeripheral::interruptLinePending(const unsigned int line) const noexcept {
    if (line > 1U || (registerValue(ile) & (1U << line)) == 0U) {
        return false;
    }
    const std::uint32_t pending = registerValue(ir) & registerValue(ie) & interrupt_mask;
    const std::uint32_t selections = registerValue(ils);
    for (std::size_t group = 0; group < interrupt_groups.size(); ++group) {
        const unsigned int selected_line = (selections & (1U << group)) == 0U ? 0U : 1U;
        if (selected_line == line && (pending & interrupt_groups[group]) != 0U) {
            return true;
        }
    }
    return false;
}

void FdcanPeripheral::updateInterruptLines() {
    for (unsigned int line = 0; line < interrupt_line_asserted_.size(); ++line) {
        const bool pending = interruptLinePending(line);
        setInterruptLevel(line, pending);
        if (!pending) {
            interrupt_line_asserted_[line] = false;
            continue;
        }
        if (interrupt_line_asserted_[line]) {
            continue;
        }
        interrupt_line_asserted_[line] = true;
        if (interrupt_callback_) {
            interrupt_callback_(line);
        }
        if (line_interrupt_callbacks_[line]) {
            line_interrupt_callbacks_[line]();
        }
    }
}

std::uint32_t FdcanPeripheral::loadRegister(const std::uint32_t word_offset,
                                            const mem::AccessContext& context) {
    static_cast<void>(context);
    if (word_offset == rxf0s) {
        updateRxFifo0Status();
    } else if (word_offset == txfqs) {
        updateTxFifoStatus();
    } else if (word_offset == txbar || word_offset == rxf0a || word_offset == rxf1a ||
               word_offset == txbcr || word_offset == txefa) {
        return 0;
    }
    return registerValue(word_offset);
}

void FdcanPeripheral::storeRegister(const std::uint32_t word_offset, const std::uint32_t previous,
                                    const std::uint32_t value, const std::uint32_t write_mask,
                                    const mem::AccessContext& context) {
    static_cast<void>(context);
    if (word_offset == cccr) {
        std::uint32_t control = value & cccr_writable_mask;
        if ((control & cccrCsr) != 0U) {
            control |= cccrInit | cccrCsa;
        } else {
            control &= ~cccrCsa;
        }
        if ((control & cccrInit) == 0U) {
            control &= ~cccrCce;
        }
        setRegister(cccr, control);
    } else if (word_offset == ir) {
        const std::uint32_t cleared = value & write_mask & interrupt_mask;
        setRegister(ir, previous & ~cleared);
        if ((cleared & interruptRxFifo0Lost) != 0U) {
            rx_message_lost_ = false;
            updateRxFifo0Status();
        }
        updateInterruptLines();
    } else if (word_offset == ie) {
        setRegister(ie, value & interrupt_mask);
        updateInterruptLines();
    } else if (word_offset == ils) {
        setRegister(ils, value & 0x7fU);
        updateInterruptLines();
    } else if (word_offset == ile) {
        setRegister(ile, value & 0x3U);
        updateInterruptLines();
    } else if (word_offset == rxf0a) {
        acknowledgeRxFifo0(value & 0x7U);
        setRegister(rxf0a, 0);
    } else if (word_offset == txbar) {
        const std::uint32_t requests = value & write_mask & tx_buffer_mask;
        setRegister(txbar, 0);
        if (!operational()) {
            return;
        }
        for (unsigned int buffer = 0; buffer < FdcanMessageRam::txFifoElementCount; ++buffer) {
            const std::uint32_t buffer_bit = 1U << buffer;
            if ((requests & buffer_bit) == 0U) {
                continue;
            }
            setRegister(txbrp, registerValue(txbrp) | buffer_bit);
            setRegister(txbto, registerValue(txbto) & ~buffer_bit);
            static_cast<void>(transmitBuffer(buffer));
        }
    } else if (word_offset == txbtie) {
        setRegister(txbtie, value & tx_buffer_mask);
    } else if (word_offset == crel || word_offset == endn || word_offset == ecr ||
               word_offset == psr || word_offset == hpms || word_offset == rxf0s ||
               word_offset == rxf1s || word_offset == txfqs || word_offset == txbrp ||
               word_offset == txbto || word_offset == txbcf || word_offset == txefs) {
        setRegister(word_offset, previous);
    }
}

void FdcanPeripheral::onReset() {
    rx_fill_level_ = 0;
    rx_get_index_ = 0;
    rx_put_index_ = 0;
    rx_message_lost_ = false;
    tx_get_index_ = 0;
    tx_put_index_ = 0;
    interrupt_line_asserted_.fill(false);
    setRegister(cccr, cccrInit);
    setRegister(ir, 0);
    setRegister(rxf0a, 0);
    setRegister(rxf1a, 0);
    setRegister(txbrp, 0);
    setRegister(txbar, 0);
    setRegister(txbto, 0);
    setRegister(txbcr, 0);
    setRegister(txefa, 0);
    updateRxFifo0Status();
    updateTxFifoStatus();
}

} // namespace fil::stm32g4
