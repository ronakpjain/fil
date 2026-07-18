#include "fil/stm32g4/fdcan.hpp"
#include "../test_support.hpp"

#include <cstdint>
#include <vector>

namespace {

std::uint32_t readWord(fil::mem::MmioDevice& device, const std::uint32_t offset) {
    auto result = device.read(offset, fil::mem::AccessSize::word, {});
    fil::test::check(result.hasValue(), "reads FDCAN word");
    return result ? static_cast<std::uint32_t>(result.value()) : 0U;
}

void writeWord(fil::mem::MmioDevice& device, const std::uint32_t offset,
               const std::uint32_t value) {
    fil::test::check(device.write(offset, fil::mem::AccessSize::word, value, {}).hasValue(),
                     "writes FDCAN word");
}

void modelsClockStopAndInitTransitions() {
    fil::stm32g4::FdcanMessageRam ram;
    fil::stm32g4::FdcanPeripheral controller(3U, ram);

    fil::test::check(readWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset) ==
                         fil::stm32g4::FdcanPeripheral::cccrInit,
                     "FDCAN resets in initialization mode");

    writeWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset, 0U);
    fil::test::check(controller.operational(), "clearing INIT starts the FDCAN core");

    writeWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset,
              fil::stm32g4::FdcanPeripheral::cccrCsr);
    const std::uint32_t stopped = readWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset);
    fil::test::check((stopped & (fil::stm32g4::FdcanPeripheral::cccrCsr |
                                 fil::stm32g4::FdcanPeripheral::cccrCsa |
                                 fil::stm32g4::FdcanPeripheral::cccrInit)) ==
                         (fil::stm32g4::FdcanPeripheral::cccrCsr |
                          fil::stm32g4::FdcanPeripheral::cccrCsa |
                          fil::stm32g4::FdcanPeripheral::cccrInit),
                     "clock-stop request immediately acknowledges and enters INIT");

    writeWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset,
              stopped & ~fil::stm32g4::FdcanPeripheral::cccrCsr);
    const std::uint32_t awakened = readWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset);
    fil::test::check((awakened & fil::stm32g4::FdcanPeripheral::cccrCsa) == 0U &&
                         (awakened & fil::stm32g4::FdcanPeripheral::cccrInit) != 0U,
                     "clearing CSR clears CSA but leaves the core in INIT");

    writeWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset,
              fil::stm32g4::FdcanPeripheral::cccrInit | fil::stm32g4::FdcanPeripheral::cccrCce);
    writeWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset, 0U);
    fil::test::check((readWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset) &
                      fil::stm32g4::FdcanPeripheral::cccrCce) == 0U,
                     "leaving INIT automatically clears CCE");
}

void exposesSharedMessageRamAsMmio() {
    fil::stm32g4::FdcanMessageRam ram;
    const std::uint32_t second_slice_word = fil::stm32g4::FdcanMessageRam::controllerStride +
                                            fil::stm32g4::FdcanMessageRam::txFifoOffset + 8U;
    writeWord(ram, second_slice_word, 0xa1b2c3d4U);
    fil::test::check(readWord(ram, second_slice_word) == 0xa1b2c3d4U,
                     "message RAM MMIO preserves little-endian controller-slice data");
    fil::test::check(ram.loadWord(1U, fil::stm32g4::FdcanMessageRam::txFifoOffset + 8U) ==
                         0xa1b2c3d4U,
                     "message RAM MMIO and controller view share storage");
    fil::test::check(
        !ram.read(fil::stm32g4::FdcanMessageRam::sizeBytes, fil::mem::AccessSize::word, {}),
        "message RAM faults accesses beyond all three slices");
}

void transmitsAndReceivesThroughMessageRam() {
    fil::stm32g4::FdcanMessageRam ram;
    fil::devices::VirtualCanBus bus("vehicle");
    fil::stm32g4::FdcanPeripheral sender(1U, ram);
    fil::stm32g4::FdcanPeripheral receiver(2U, ram);
    fil::test::check(sender.attachBus(bus, "sender").hasValue(), "attaches FDCAN sender");
    fil::test::check(receiver.attachBus(bus, "receiver").hasValue(), "attaches FDCAN receiver");

    writeWord(sender, fil::stm32g4::FdcanPeripheral::cccrOffset, 0U);
    writeWord(receiver, fil::stm32g4::FdcanPeripheral::cccrOffset, 0U);

    std::vector<unsigned int> sender_interrupts;
    std::vector<unsigned int> receiver_interrupts;
    sender.setInterruptCallback(
        [&](const unsigned int line) { sender_interrupts.push_back(line); });
    receiver.setInterruptCallback(
        [&](const unsigned int line) { receiver_interrupts.push_back(line); });

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

    fil::test::check(receiver_interrupts == std::vector<unsigned int>({0U}),
                     "RX FIFO new-message interrupt uses line 0");
    fil::test::check(sender_interrupts == std::vector<unsigned int>({1U}),
                     "TX-complete interrupt follows ILS to line 1");

    const std::uint32_t rx_status = readWord(receiver, fil::stm32g4::FdcanPeripheral::rxf0sOffset);
    fil::test::check((rx_status & 0x0fU) == 1U && ((rx_status >> 16U) & 0x3U) == 1U,
                     "RXF0S exposes fill level and next put index");
    const std::uint32_t rx_offset = fil::stm32g4::FdcanMessageRam::rxFifo0Offset;
    fil::test::check(ram.loadWord(1U, rx_offset) == identifier << 18U,
                     "received standard identifier is encoded in message RAM");
    fil::test::check(ram.loadWord(1U, rx_offset + 8U) == 0x44332211U,
                     "received payload is copied into message RAM");

    const std::uint32_t tx_status = readWord(sender, fil::stm32g4::FdcanPeripheral::txfqsOffset);
    fil::test::check((tx_status & 0x7U) == 3U && ((tx_status >> 16U) & 0x3U) == 1U,
                     "completed TX leaves three slots free and advances TFQPI");
    fil::test::check((readWord(sender, fil::stm32g4::FdcanPeripheral::txbtoOffset) & 1U) != 0U,
                     "TXBTO records the transmitted FIFO element");

    writeWord(receiver, fil::stm32g4::FdcanPeripheral::irOffset,
              fil::stm32g4::FdcanPeripheral::interruptRxFifo0New);
    writeWord(sender, fil::stm32g4::FdcanPeripheral::irOffset,
              fil::stm32g4::FdcanPeripheral::interruptTransmissionComplete);
    fil::test::check((readWord(receiver, fil::stm32g4::FdcanPeripheral::irOffset) &
                      fil::stm32g4::FdcanPeripheral::interruptRxFifo0New) == 0U &&
                         (readWord(sender, fil::stm32g4::FdcanPeripheral::irOffset) &
                          fil::stm32g4::FdcanPeripheral::interruptTransmissionComplete) == 0U,
                     "IR implements write-one-to-clear semantics");

    writeWord(receiver, fil::stm32g4::FdcanPeripheral::rxf0aOffset, 0U);
    fil::test::check((readWord(receiver, fil::stm32g4::FdcanPeripheral::rxf0sOffset) & 0x0fU) == 0U,
                     "RXF0A releases the acknowledged FIFO element");
}

void filtersAgainstThePerMessageRamLists() {
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
    fil::test::check(!controller.receiveFrame(rejected, 10U),
                     "non-matching standard identifier is rejected by RXGFC");

    fil::devices::CanFrame accepted = rejected;
    accepted.id = accepted_identifier;
    fil::test::check(controller.receiveFrame(accepted, 11U),
                     "classic-mask standard filter accepts its configured identifier");
    fil::test::check((readWord(controller, fil::stm32g4::FdcanPeripheral::rxf0sOffset) & 0x0fU) ==
                         1U,
                     "accepted filtered frame occupies RX FIFO 0");
}

void gatesPendingEventsThroughInterruptRegisters() {
    fil::stm32g4::FdcanMessageRam ram;
    fil::stm32g4::FdcanPeripheral controller(2U, ram);
    writeWord(controller, fil::stm32g4::FdcanPeripheral::cccrOffset, 0U);

    std::vector<unsigned int> interrupts;
    controller.setInterruptCallback([&](const unsigned int line) { interrupts.push_back(line); });

    fil::devices::CanFrame frame;
    frame.id = 0x55U;
    frame.dlc = 1U;
    frame.data[0] = 0xaaU;
    fil::test::check(controller.receiveFrame(frame, 1U),
                     "receives a frame while interrupt output is disabled");
    fil::test::check(interrupts.empty(), "IR alone does not bypass IE and ILE");

    writeWord(controller, fil::stm32g4::FdcanPeripheral::ieOffset,
              fil::stm32g4::FdcanPeripheral::interruptRxFifo0New);
    writeWord(controller, fil::stm32g4::FdcanPeripheral::ilsOffset, 1U);
    fil::test::check(interrupts.empty(), "disabled line suppresses an enabled pending event");
    writeWord(controller, fil::stm32g4::FdcanPeripheral::ileOffset, 1U << 1U);
    fil::test::check(interrupts == std::vector<unsigned int>({1U}),
                     "enabling ILE asserts the ILS-selected line for a pending event");
}

} // namespace

void runFdcanTests() {
    modelsClockStopAndInitTransitions();
    exposesSharedMessageRamAsMmio();
    transmitsAndReceivesThroughMessageRam();
    filtersAgainstThePerMessageRamLists();
    gatesPendingEventsThroughInterruptRegisters();
}

#ifdef FIL_FDCAN_TEST_MAIN
int main() {
    runFdcanTests();
    return fil::test::failures == 0 ? 0 : 1;
}
#endif
