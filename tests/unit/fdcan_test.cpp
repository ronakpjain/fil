#include "fil/stm32g4/fdcan.hpp"
#include "fil/mem/memory_bus.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

namespace {

constexpr fil::mem::AccessContext read_context{fil::mem::AccessType::data_read, 0};
constexpr fil::mem::AccessContext write_context{fil::mem::AccessType::data_write, 0};

std::uint32_t readWord(fil::mem::MmioDevice& device, const std::uint32_t offset) {
    auto result = device.read(offset, fil::mem::AccessSize::word, {});
    EXPECT_TRUE(result.hasValue()) << "reads FDCAN word";
    return result ? static_cast<std::uint32_t>(result.value()) : 0U;
}

void writeWord(fil::mem::MmioDevice& device, const std::uint32_t offset,
               const std::uint32_t value) {
    EXPECT_TRUE(device.write(offset, fil::mem::AccessSize::word, value, {}).hasValue())
        << "writes FDCAN word";
}

TEST(FdcanTest, ModelsClockStopAndInitTransitions) {
    fil::stm32g4::FdcanMessageRam ram;
    fil::stm32g4::FdcanPeripheral controller(3U, ram);

    EXPECT_TRUE(readWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset) ==
                fil::stm32g4::FdcanPeripheral::cccrInit)
        << "FDCAN resets in initialization mode";

    writeWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset, 0U);
    EXPECT_TRUE(controller.operational()) << "clearing INIT starts the FDCAN core";

    writeWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset,
              fil::stm32g4::FdcanPeripheral::cccrCsr);
    const std::uint32_t stopped = readWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset);
    EXPECT_TRUE((stopped & (fil::stm32g4::FdcanPeripheral::cccrCsr |
                               fil::stm32g4::FdcanPeripheral::cccrCsa |
                               fil::stm32g4::FdcanPeripheral::cccrInit)) ==
                (fil::stm32g4::FdcanPeripheral::cccrCsr | fil::stm32g4::FdcanPeripheral::cccrCsa |
                    fil::stm32g4::FdcanPeripheral::cccrInit))
        << "clock-stop request immediately acknowledges and enters INIT";

    writeWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset,
              stopped & ~fil::stm32g4::FdcanPeripheral::cccrCsr);
    const std::uint32_t awakened = readWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset);
    EXPECT_TRUE((awakened & fil::stm32g4::FdcanPeripheral::cccrCsa) == 0U &&
                (awakened & fil::stm32g4::FdcanPeripheral::cccrInit) != 0U)
        << "clearing CSR clears CSA but leaves the core in INIT";

    writeWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset,
              fil::stm32g4::FdcanPeripheral::cccrInit | fil::stm32g4::FdcanPeripheral::cccrCce);
    writeWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset, 0U);
    EXPECT_TRUE((readWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset) &
                    fil::stm32g4::FdcanPeripheral::cccrCce) == 0U)
        << "leaving INIT automatically clears CCE";
}

TEST(FdcanTest, ExposesSharedMessageRamAsMmio) {
    fil::stm32g4::FdcanMessageRam ram;
    const std::uint32_t second_slice_word = fil::stm32g4::FdcanMessageRam::controllerStride +
                                            fil::stm32g4::FdcanMessageRam::txFifoOffset + 8U;
    writeWord(ram, second_slice_word, 0xa1b2c3d4U);
    EXPECT_TRUE(readWord(ram, second_slice_word) == 0xa1b2c3d4U)
        << "message RAM MMIO preserves little-endian controller-slice data";
    EXPECT_TRUE(ram.loadWord(1U, fil::stm32g4::FdcanMessageRam::txFifoOffset + 8U) == 0xa1b2c3d4U)
        << "message RAM MMIO and controller view share storage";
    EXPECT_TRUE(!ram.read(fil::stm32g4::FdcanMessageRam::sizeBytes, fil::mem::AccessSize::word, {}))
        << "message RAM faults accesses beyond all three slices";
}

TEST(FdcanTest, TransmitsAndReceivesThroughMessageRam) {
    fil::stm32g4::FdcanMessageRam ram;
    fil::devices::VirtualCanBus bus("vehicle");
    fil::stm32g4::FdcanPeripheral sender(1U, ram);
    fil::stm32g4::FdcanPeripheral receiver(2U, ram);
    EXPECT_TRUE(sender.attachBus(bus, "sender").hasValue()) << "attaches FDCAN sender";
    EXPECT_TRUE(receiver.attachBus(bus, "receiver").hasValue()) << "attaches FDCAN receiver";

    writeWord(sender, fil::stm32g4::FdcanPeripheral::cccrOffset, 0U);
    writeWord(receiver, fil::stm32g4::FdcanPeripheral::cccrOffset, 0U);

    std::vector<std::uint32_t> sender_interrupts;
    std::vector<std::uint32_t> receiver_interrupts;
    sender.setInterruptCallback(
        [&](const std::uint32_t line) { sender_interrupts.push_back(line); });
    receiver.setInterruptCallback(
        [&](const std::uint32_t line) { receiver_interrupts.push_back(line); });

    writeWord(receiver, fil::stm32g4::FdcanPeripheral::ieOffset,
              fil::stm32g4::FdcanPeripheral::interruptRxFifo0New);
    writeWord(receiver, fil::stm32g4::FdcanPeripheral::ileOffset, 1U);
    writeWord(sender, fil::stm32g4::FdcanPeripheral::ieOffset,
              fil::stm32g4::FdcanPeripheral::interruptTransmissionComplete);
    writeWord(sender, fil::stm32g4::FdcanPeripheral::ilsOffset, 1U << 2U);
    writeWord(sender, fil::stm32g4::FdcanPeripheral::ileOffset, 1U << 1U);
    writeWord(sender, fil::stm32g4::FdcanPeripheral::txbtieOffset, 1U);

    constexpr std::uint32_t identifier = 0x321U;
    const std::uint32_t tx_offset = fil::stm32g4::FdcanMessageRam::txFifoOffset;
    ram.storeWord(0U, tx_offset, identifier << 18U);
    ram.storeWord(0U, tx_offset + 4U, 4U << 16U);
    ram.storeWord(0U, tx_offset + 8U, 0x44332211U);

    writeWord(sender, fil::stm32g4::FdcanPeripheral::txbarOffset, 1U);

    EXPECT_TRUE(receiver_interrupts == std::vector<std::uint32_t>({0U}))
        << "RX FIFO new-message interrupt uses line 0";
    EXPECT_TRUE(sender_interrupts == std::vector<std::uint32_t>({1U}))
        << "TX-complete interrupt follows ILS to line 1";

    const std::uint32_t rx_status = readWord(receiver, fil::stm32g4::FdcanPeripheral::rxf0sOffset);
    EXPECT_TRUE((rx_status & 0x0fU) == 1U && ((rx_status >> 16U) & 0x3U) == 1U)
        << "RXF0S exposes fill level and next put index";
    const std::uint32_t rx_offset = fil::stm32g4::FdcanMessageRam::rxFifo0Offset;
    EXPECT_TRUE(ram.loadWord(1U, rx_offset) == identifier << 18U)
        << "received standard identifier is encoded in message RAM";
    EXPECT_TRUE(ram.loadWord(1U, rx_offset + 8U) == 0x44332211U)
        << "received payload is copied into message RAM";

    const std::uint32_t tx_status = readWord(sender, fil::stm32g4::FdcanPeripheral::txfqsOffset);
    EXPECT_TRUE((tx_status & 0x7U) == 3U && ((tx_status >> 16U) & 0x3U) == 1U)
        << "completed TX leaves three slots free and advances TFQPI";
    EXPECT_TRUE((readWord(sender, fil::stm32g4::FdcanPeripheral::txbtoOffset) & 1U) != 0U)
        << "TXBTO records the transmitted FIFO element";

    writeWord(receiver, fil::stm32g4::FdcanPeripheral::irOffset,
              fil::stm32g4::FdcanPeripheral::interruptRxFifo0New);
    writeWord(sender, fil::stm32g4::FdcanPeripheral::irOffset,
              fil::stm32g4::FdcanPeripheral::interruptTransmissionComplete);
    EXPECT_TRUE((readWord(receiver, fil::stm32g4::FdcanPeripheral::irOffset) &
                    fil::stm32g4::FdcanPeripheral::interruptRxFifo0New) == 0U &&
                (readWord(sender, fil::stm32g4::FdcanPeripheral::irOffset) &
                    fil::stm32g4::FdcanPeripheral::interruptTransmissionComplete) == 0U)
        << "IR implements write-one-to-clear semantics";

    writeWord(receiver, fil::stm32g4::FdcanPeripheral::rxf0aOffset, 0U);
    EXPECT_TRUE((readWord(receiver, fil::stm32g4::FdcanPeripheral::rxf0sOffset) & 0x0fU) == 0U)
        << "RXF0A releases the acknowledged FIFO element";
}

TEST(FdcanTest, FiltersAgainstThePerMessageRamLists) {
    fil::stm32g4::FdcanMessageRam ram;
    fil::stm32g4::FdcanPeripheral controller(1U, ram);
    writeWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset, 0U);

    constexpr std::uint32_t accepted_identifier = 0x456U;
    ram.storeWord(0U, fil::stm32g4::FdcanMessageRam::standardFilterOffset,
                  (2U << 30U) | (1U << 27U) | (accepted_identifier << 16U) | 0x7ffU);
    writeWord(controller, fil::stm32g4::FdcanPeripheral::rxgfcOffset,
              (1U << 16U) | (2U << 4U) | (2U << 2U));

    fil::devices::CanFrame rejected;
    rejected.id = 0x123U;
    rejected.dlc = 1U;
    EXPECT_TRUE(!controller.receiveFrame(rejected, 10U))
        << "non-matching standard identifier is rejected by RXGFC";

    fil::devices::CanFrame accepted = rejected;
    accepted.id = accepted_identifier;
    EXPECT_TRUE(controller.receiveFrame(accepted, 11U))
        << "classic-mask standard filter accepts its configured identifier";
    EXPECT_TRUE((readWord(controller, fil::stm32g4::FdcanPeripheral::rxf0sOffset) & 0x0fU) == 1U)
        << "accepted filtered frame occupies RX FIFO 0";
}

TEST(FdcanTest, GatesPendingEventsThroughInterruptRegisters) {
    fil::stm32g4::FdcanMessageRam ram;
    fil::stm32g4::FdcanPeripheral controller(2U, ram);
    writeWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset, 0U);

    std::vector<std::uint32_t> interrupts;
    controller.setInterruptCallback([&](const std::uint32_t line) { interrupts.push_back(line); });

    fil::devices::CanFrame frame;
    frame.id = 0x55U;
    frame.dlc = 1U;
    frame.data[0] = 0xaaU;
    EXPECT_TRUE(controller.receiveFrame(frame, 1U))
        << "receives a frame while interrupt output is disabled";
    EXPECT_TRUE(interrupts.empty()) << "IR alone does not bypass IE and ILE";

    writeWord(controller, fil::stm32g4::FdcanPeripheral::ieOffset,
              fil::stm32g4::FdcanPeripheral::interruptRxFifo0New);
    writeWord(controller, fil::stm32g4::FdcanPeripheral::ilsOffset, 1U);
    EXPECT_TRUE(interrupts.empty()) << "disabled line suppresses an enabled pending event";
    writeWord(controller, fil::stm32g4::FdcanPeripheral::ileOffset, 1U << 1U);
    EXPECT_TRUE(interrupts == std::vector<std::uint32_t>({1U}))
        << "enabling ILE asserts the ILS-selected line for a pending event";
}

TEST(FdcanTest, FdcanStandardRangeFifoFullAndAck) {
    fil::stm32g4::FdcanMessageRam ram;
    fil::stm32g4::FdcanPeripheral can(1U, ram);
    ASSERT_TRUE(can.write(fil::stm32g4::FdcanPeripheral::cccrOffset, fil::mem::AccessSize::word, 0U, write_context));
    // Standard range filter (type 0/action FIFO0), identifiers [0x120, 0x12f].
    ram.storeWord(0U, fil::stm32g4::FdcanMessageRam::standardFilterOffset,
                  (1U << 27U) | (0x120U << 16U) | 0x12fU);
    ASSERT_TRUE(can.write(fil::stm32g4::FdcanPeripheral::rxgfcOffset, fil::mem::AccessSize::word, (1U << 16U) | (1U << 4U), write_context));
    fil::devices::CanFrame frame;
    frame.id = 0x128U; frame.fd = true; frame.dlc = 9U;
    for (std::uint32_t i = 0; i < 12U; ++i) frame.data[i] = static_cast<std::uint8_t>(i + 1U);
    ASSERT_TRUE(can.receiveFrame(frame, 1U));
    EXPECT_EQ(ram.loadWord(0U, fil::stm32g4::FdcanMessageRam::rxFifo0Offset + 8U), 0x04030201U);
    EXPECT_EQ(ram.loadWord(0U, fil::stm32g4::FdcanMessageRam::rxFifo0Offset + 12U), 0x08070605U);
    frame.id = 0x130U;
    EXPECT_FALSE(can.receiveFrame(frame, 2U)); // outside inclusive range
    frame.id = 0x128U;
    ASSERT_TRUE(can.receiveFrame(frame, 3U));
    ASSERT_TRUE(can.receiveFrame(frame, 4U));
    EXPECT_FALSE(can.receiveFrame(frame, 5U)); // full, overwrite disabled
    EXPECT_NE(can.peekRegister(fil::stm32g4::FdcanPeripheral::rxf0sOffset) & (1U << 25U), 0U);
    // Release indices 0..1 with an acknowledge; remaining slot accepts another.
    ASSERT_TRUE(can.write(fil::stm32g4::FdcanPeripheral::rxf0aOffset, fil::mem::AccessSize::word, 1U, write_context));
    EXPECT_EQ(can.peekRegister(fil::stm32g4::FdcanPeripheral::rxf0sOffset) & 0xfU, 1U);
    // RXF0 overwrite mode (RXGFC.F0OM) retains the newest three messages.
    ASSERT_TRUE(can.write(fil::stm32g4::FdcanPeripheral::rxgfcOffset, fil::mem::AccessSize::word,
                          (1U << 16U) | (1U << 4U) | (1U << 9U), write_context));
    ASSERT_TRUE(can.receiveFrame(frame, 6U));
    ASSERT_TRUE(can.receiveFrame(frame, 7U));
    ASSERT_EQ(can.peekRegister(fil::stm32g4::FdcanPeripheral::rxf0sOffset) & 0xfU, 3U);
    frame.data[0] = 0xeeU;
    ASSERT_TRUE(can.receiveFrame(frame, 8U));
    const std::uint32_t overwritten_element = fil::stm32g4::FdcanMessageRam::rxFifo0Offset +
        2U * fil::stm32g4::FdcanMessageRam::rxFifoElementSize;
    EXPECT_EQ(ram.loadWord(0U, overwritten_element + 8U) & 0xffU, 0xeeU);
    const std::uint32_t fifo_status = can.peekRegister(fil::stm32g4::FdcanPeripheral::rxf0sOffset);
    EXPECT_EQ(fifo_status & 0xfU, 3U);
    EXPECT_NE(fifo_status & (1U << 25U), 0U);
}

TEST(FdcanTest, FdcanExtendedRangeFilterAndNonmatchReject) {
    fil::stm32g4::FdcanMessageRam ram;
    fil::stm32g4::FdcanPeripheral can(1U, ram);
    ASSERT_TRUE(can.write(fil::stm32g4::FdcanPeripheral::cccrOffset, fil::mem::AccessSize::word, 0U, write_context));
    constexpr std::uint32_t low = 0x1234000U;
    constexpr std::uint32_t high = 0x12340ffU;
    const std::uint32_t offset = fil::stm32g4::FdcanMessageRam::extendedFilterOffset;
    ram.storeWord(0U, offset, (1U << 29U) | low); // FIFO0 action + ID1
    ram.storeWord(0U, offset + 4U, high); // range filter (type 0) + ID2
    ASSERT_TRUE(can.write(fil::stm32g4::FdcanPeripheral::rxgfcOffset, fil::mem::AccessSize::word,
                          (1U << 24U) | (1U << 2U), write_context)); // one XID filter, reject nonmatch
    fil::devices::CanFrame frame;
    frame.extended = true;
    frame.id = 0x1234080U;
    frame.dlc = 1U;
    EXPECT_TRUE(can.receiveFrame(frame, 1U));
    frame.id = 0x1234100U;
    EXPECT_FALSE(can.receiveFrame(frame, 2U));
}

TEST(FdcanTest, FdcanStandardDualIdAndMaskFiltersAcceptOnlyMatches) {
    struct Case { std::uint32_t type; std::uint32_t first; std::uint32_t second; std::uint32_t accept; std::uint32_t reject; };
    constexpr std::array cases{
        Case{1U, 0x123U, 0x456U, 0x456U, 0x124U}, // dual ID
        Case{2U, 0x520U, 0x7f0U, 0x52aU, 0x531U}, // classic mask
    };
    for (const Case entry : cases) {
        fil::stm32g4::FdcanMessageRam ram;
        fil::stm32g4::FdcanPeripheral can(1U, ram);
        ASSERT_TRUE(can.write(fil::stm32g4::FdcanPeripheral::cccrOffset, fil::mem::AccessSize::word, 0U, write_context));
        const std::uint32_t filter = (entry.type << 30U) | (1U << 27U) |
            (entry.first << 16U) | entry.second;
        ram.storeWord(0U, fil::stm32g4::FdcanMessageRam::standardFilterOffset, filter);
        ASSERT_TRUE(can.write(fil::stm32g4::FdcanPeripheral::rxgfcOffset, fil::mem::AccessSize::word,
                              (1U << 16U) | (1U << 4U), write_context));
        fil::devices::CanFrame frame;
        frame.id = entry.accept; frame.dlc = 0U;
        EXPECT_TRUE(can.receiveFrame(frame, 1U)) << "standard filter type=" << entry.type;
        frame.id = entry.reject;
        EXPECT_FALSE(can.receiveFrame(frame, 2U)) << "standard filter type=" << entry.type;
    }
}

TEST(FdcanTest, FdcanExtendedDualIdAndMaskFiltersAcceptOnlyMatches) {
    struct Case { std::uint32_t type; std::uint32_t first; std::uint32_t second; std::uint32_t accept; std::uint32_t reject; };
    constexpr std::array cases{
        Case{1U, 0x123456U, 0x234567U, 0x234567U, 0x123457U},
        Case{2U, 0x1234500U, 0x1fffff00U, 0x12345a5U, 0x12344a5U},
    };
    for (const Case entry : cases) {
        fil::stm32g4::FdcanMessageRam ram;
        fil::stm32g4::FdcanPeripheral can(1U, ram);
        ASSERT_TRUE(can.write(fil::stm32g4::FdcanPeripheral::cccrOffset, fil::mem::AccessSize::word, 0U, write_context));
        const std::uint32_t offset = fil::stm32g4::FdcanMessageRam::extendedFilterOffset;
        ram.storeWord(0U, offset, (1U << 29U) | entry.first);
        ram.storeWord(0U, offset + 4U, (entry.type << 30U) | entry.second);
        ASSERT_TRUE(can.write(fil::stm32g4::FdcanPeripheral::rxgfcOffset, fil::mem::AccessSize::word,
                              (1U << 24U) | (1U << 2U), write_context));
        fil::devices::CanFrame frame;
        frame.extended = true; frame.id = entry.accept;
        EXPECT_TRUE(can.receiveFrame(frame, 1U)) << "extended filter type=" << entry.type;
        frame.id = entry.reject;
        EXPECT_FALSE(can.receiveFrame(frame, 2U)) << "extended filter type=" << entry.type;
    }
}

TEST(FdcanTest, FdcanTransmitBuffersTrackRequestsAndIgnoreInvalidBits) {
    fil::stm32g4::FdcanMessageRam ram;
    fil::stm32g4::FdcanPeripheral can(1U, ram);
    ASSERT_TRUE(can.write(fil::stm32g4::FdcanPeripheral::cccrOffset, fil::mem::AccessSize::word, 0U, write_context));
    EXPECT_EQ(can.peekRegister(fil::stm32g4::FdcanPeripheral::txfqsOffset) & 0x7U, 3U);
    ASSERT_TRUE(can.write(fil::stm32g4::FdcanPeripheral::txbarOffset, fil::mem::AccessSize::word, 1U << 3U, write_context));
    EXPECT_EQ(can.peekRegister(0x0c8U), 0U);
    for (std::uint32_t buffer = 0; buffer < 3U; ++buffer) {
        const std::uint32_t element = fil::stm32g4::FdcanMessageRam::txFifoOffset +
            buffer * fil::stm32g4::FdcanMessageRam::txFifoElementSize;
        ram.storeWord(0U, element, (0x100U + buffer) << 18U);
        ram.storeWord(0U, element + 4U, 1U << 16U);
        ram.storeWord(0U, element + 8U, 0xa0U + buffer);
        ASSERT_TRUE(can.write(fil::stm32g4::FdcanPeripheral::txbarOffset, fil::mem::AccessSize::word, 1U << buffer, write_context));
        EXPECT_EQ(can.peekRegister(0x0c8U), 0U);
        EXPECT_NE(can.peekRegister(fil::stm32g4::FdcanPeripheral::txbtoOffset) & (1U << buffer), 0U);
    }
    EXPECT_EQ(can.peekRegister(fil::stm32g4::FdcanPeripheral::txbtoOffset) & 0x7U, 0x7U);
    EXPECT_EQ(can.peekRegister(fil::stm32g4::FdcanPeripheral::txfqsOffset) & 0x7U, 3U);
}

} // namespace
