#include "fil/devices/can_bus.hpp"
#include "fil/sim/world.hpp"
#include "fil/stm32g4/fdcan.hpp"
#include "fil/stm32g4/stm32g4.hpp"
#include "../test_support.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>

namespace {

class TempCanWorldConfig {
public:
    TempCanWorldConfig()
        : root_(std::filesystem::temp_directory_path() / "fil-can-end-to-end-tests") {
        std::error_code error;
        std::filesystem::remove_all(root_, error);
        std::filesystem::create_directories(root_, error);

        const std::filesystem::path mcu =
            std::filesystem::path(FIL_SOURCE_DIR) / "configs/mcus/stm32g474retx.json";
        const std::filesystem::path elf =
            std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";
        board_path_ = root_ / "receiver.json";
        std::ofstream board(board_path_, std::ios::binary);
        board << "{\n"
              << "  \"schema_version\": 1,\n"
              << "  \"name\": \"receiver\",\n"
              << "  \"mcu\": \"" << mcu.string() << "\",\n"
              << "  \"elf\": \"" << elf.string() << "\",\n"
              << "  \"vector_base\": \"0x08000000\",\n"
              << "  \"can\": {\"FDCAN1\": {\"bus\": \"vehicle\", \"loopback\": false}}\n"
              << "}\n";
    }

    ~TempCanWorldConfig() {
        std::error_code error;
        std::filesystem::remove_all(root_, error);
    }

    [[nodiscard]] fil::config::NetworkConfig network() const {
        fil::config::NetworkConfig config;
        config.name = "can-end-to-end";
        config.source_path = root_ / "network.json";
        config.buses.push_back({"vehicle", 500000U});
        config.board_paths.push_back(board_path_);
        return config;
    }

private:
    std::filesystem::path root_;
    std::filesystem::path board_path_;
};

std::uint32_t readRegister(fil::stm32g4::FdcanPeripheral& controller,
                           const std::uint32_t offset) {
    const auto value = controller.read(offset, fil::mem::AccessSize::word, {});
    fil::test::check(value.hasValue(), "reads an FDCAN register");
    return value ? static_cast<std::uint32_t>(value.value()) : 0U;
}

void writeRegister(fil::stm32g4::FdcanPeripheral& controller, const std::uint32_t offset,
                   const std::uint32_t value) {
    fil::test::check(
        controller.write(offset, fil::mem::AccessSize::word, value, {}).hasValue(),
        "writes an FDCAN register"
    );
}

bool hasTraceField(
    const fil::sim::TraceRecord& record,
    const std::string_view key,
    const std::string_view value
) {
    for (const auto& [field_key, field_value] : record.fields) {
        if (field_key == key && field_value == value) return true;
    }
    return false;
}

void scheduledExternalFrameReachesMessageRamAndNvic() {
    TempCanWorldConfig files;
    auto world = fil::sim::World::load(files.network());
    fil::test::check(world.hasValue(), "loads a CAN end-to-end world");
    if (!world) return;

    fil::sim::Board* board = world.value()->board("receiver");
    fil::devices::VirtualCanBus* bus = world.value()->canBus("vehicle");
    fil::test::check(board != nullptr && bus != nullptr, "exposes the receiver board and CAN bus");
    if (board == nullptr || bus == nullptr) return;

    fil::stm32g4::FdcanPeripheral* receiver = board->peripherals().fdcan("FDCAN1");
    fil::test::check(receiver != nullptr, "exposes the receiver FDCAN instance");
    if (receiver == nullptr) return;

    writeRegister(*receiver, fil::stm32g4::FdcanPeripheral::ieOffset,
                  fil::stm32g4::FdcanPeripheral::interruptRxFifo0New);
    writeRegister(*receiver, fil::stm32g4::FdcanPeripheral::ileOffset, 1U);
    writeRegister(*receiver, fil::stm32g4::FdcanPeripheral::cccrOffset, 0U);
    fil::test::check(receiver->operational(), "starts the receiving FDCAN core");

    fil::devices::CanFrame frame;
    frame.id = 0x5a3U;
    frame.fd = true;
    frame.brs = true;
    frame.dlc = 9U;
    for (std::uint8_t index = 0; index < 12U; ++index) frame.data[index] = index;

    constexpr fil::sim::SimTimeNs injection_time = 250U;
    bool injection_succeeded = false;
    static_cast<void>(world.value()->eventLoop().scheduleAt(
        injection_time,
        [&] {
            const auto injected = bus->inject(frame, world.value()->eventLoop().now());
            injection_succeeded = injected.hasValue();
        }
    ));

    fil::test::check(
        (readRegister(*receiver, fil::stm32g4::FdcanPeripheral::rxf0sOffset) & 0x0fU) == 0U,
        "does not deliver a scheduled frame before its timestamp"
    );
    const auto events = world.value()->eventLoop().runDueEvents(injection_time);
    fil::test::check(
        events.events_executed == 1U && events.stopped_at == injection_time && injection_succeeded,
        "delivers the external frame at the scheduled world time"
    );

    const std::uint32_t status =
        readRegister(*receiver, fil::stm32g4::FdcanPeripheral::rxf0sOffset);
    fil::test::check(
        (status & 0x0fU) == 1U && ((status >> 16U) & 0x3U) == 1U,
        "advances the operational receiver's RX FIFO 0 state"
    );
    fil::test::check(
        (readRegister(*receiver, fil::stm32g4::FdcanPeripheral::irOffset)
         & fil::stm32g4::FdcanPeripheral::interruptRxFifo0New) != 0U,
        "raises the FDCAN RX FIFO 0 new-message interrupt flag"
    );

    constexpr std::uint32_t element =
        fil::stm32g4::FdcanMessageRam::baseAddress
        + fil::stm32g4::FdcanMessageRam::rxFifo0Offset;
    const auto header0 = board->memory().read32(element);
    const auto header1 = board->memory().read32(element + 4U);
    const auto payload0 = board->memory().read32(element + 8U);
    const auto payload1 = board->memory().read32(element + 12U);
    const auto payload2 = board->memory().read32(element + 16U);
    fil::test::check(
        header0 && header1 && payload0 && payload1 && payload2,
        "reads the received element through the board's CPU-visible message RAM"
    );
    if (header0 && header1 && payload0 && payload1 && payload2) {
        fil::test::check(
            header0.value() == (frame.id << 18U)
                && ((header1.value() >> 16U) & 0x0fU) == frame.dlc
                && (header1.value() & (1U << 20U)) != 0U
                && (header1.value() & (1U << 21U)) != 0U,
            "stores the injected identifier, encoded DLC, BRS, and FD flags in the RX header"
        );
        fil::test::check(
            payload0.value() == 0x03020100U && payload1.value() == 0x07060504U
                && payload2.value() == 0x0b0a0908U,
            "stores all 12 bytes of the injected CAN-FD payload in message RAM"
        );
    }

    // FDCAN1 interrupt line 0 is STM32G4 external IRQ 21. It remains pending
    // because this test deliberately leaves the NVIC enable bit clear, so no
    // firmware exception entry can consume it while inspecting the result.
    const auto nvic_pending = board->memory().read32(0xe000e200U);
    fil::test::check(
        nvic_pending && (nvic_pending.value() & (1U << 21U)) != 0U,
        "propagates FDCAN1 line 0 into the NVIC pending register"
    );

    bool traced_external_transmit = false;
    bool traced_receiver_delivery = false;
    bool traced_controller_receive = false;
    for (const fil::sim::TraceRecord& record : world.value()->trace().records()) {
        if (record.time_ns != injection_time) continue;
        const bool fd_length = hasTraceField(record, "dlc", "9")
            && hasTraceField(record, "length", "12");
        traced_external_transmit |=
            record.type == "can_tx" && record.source == "vehicle/external" && fd_length;
        traced_receiver_delivery |=
            record.type == "can_rx" && record.source == "vehicle/receiver.FDCAN1" && fd_length;
        traced_controller_receive |=
            record.type == "can_rx" && record.source == "receiver.FDCAN1" && fd_length;
    }
    fil::test::check(
        traced_external_transmit && traced_receiver_delivery && traced_controller_receive,
        "CAN-FD bus and controller traces use encoded DLC, decoded length, and qualified sources"
    );
}

} // namespace

int main() {
    scheduledExternalFrameReachesMessageRamAndNvic();
    if (fil::test::failures != 0) {
        return 1;
    }
    return 0;
}
