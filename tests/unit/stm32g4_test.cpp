#include "fil/cortexm/system_control.hpp"
#include "fil/stm32g4/stm32g4.hpp"
#include "../test_support.hpp"

#include <filesystem>
#include <fstream>

namespace {

void routesIntegratedPeripherals() {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true, 16'000'000U);
    fil::test::check(mcu.hasValue(), "constructs integrated STM32G4 peripheral map");
    if (!mcu) return;

    fil::test::check(mcu.value()->adc("ADC1")->sampleHistoryEnabled(),
                     "integrated ADC diagnostics default to enabled");
    mcu.value()->setAdcDiagnosticsEnabled(false);
    fil::test::check(!mcu.value()->adc("ADC1")->sampleHistoryEnabled()
                         && !mcu.value()->adc("ADC4")->sampleHistoryEnabled(),
                     "integrated ADC diagnostics toggle applies to every ADC");

    auto rcc_write = mcu.value()->router().write(0x00021000U, fil::mem::AccessSize::word, 1U << 8U, {});
    auto rcc_read = mcu.value()->router().read(0x00021000U, fil::mem::AccessSize::word, {});
    fil::test::check(rcc_write && rcc_read && (rcc_read.value() & (1U << 10U)) != 0, "routes RCC and asserts HSI ready");
    auto hse_enable = mcu.value()->router().write(
        0x00021000U, fil::mem::AccessSize::word, 1U << 16U, {}
    );
    auto hse_select = mcu.value()->router().write(
        0x00021008U, fil::mem::AccessSize::word, 2U, {}
    );
    fil::test::check(
        hse_enable && hse_select && mcu.value()->rcc().systemClockHz() == 16'000'000U,
        "uses the configured HSE frequency"
    );

    fil::config::BoardConfig board;
    board.gpio.push_back({"PB7", "output", false, true});
    const auto uart_log = std::filesystem::temp_directory_path() / "fil-stm32g4-uart.bin";
    board.usart.push_back({"USART1", uart_log, {0x42U}});
    board.spi.push_back({"SPI1", "echo"});
    fil::test::check(mcu.value()->configure(board).hasValue(), "applies board device configuration");
    auto gpio_write = mcu.value()->router().write(0x08000418U, fil::mem::AccessSize::word, 1U << 7U, {});
    fil::test::check(gpio_write && mcu.value()->gpio("GPIOB")->output(7), "routes GPIO BSRR side effect");
    auto uart_read = mcu.value()->router().read(0x00013824U, fil::mem::AccessSize::word, {});
    fil::test::check(uart_read && (uart_read.value() & 0xffU) == 0x42U, "routes scripted USART RX byte");
    auto uart_write = mcu.value()->router().write(0x00013828U, fil::mem::AccessSize::word, 0x5aU, {});
    std::ifstream uart_input(uart_log, std::ios::binary);
    const int logged_byte = uart_input.get();
    fil::test::check(uart_write && logged_byte == 0x5a, "writes configured USART TX byte log");
    std::error_code remove_error;
    std::filesystem::remove(uart_log, remove_error);
    auto fdcan_read = mcu.value()->router().read(
        fil::stm32g4::FdcanPeripheral::baseAddresses[0] - 0x40000000U +
            fil::stm32g4::FdcanPeripheral::cccrOffset,
        fil::mem::AccessSize::word,
        {}
    );
    fil::test::check(
        fdcan_read && mcu.value()->fdcan("FDCAN1") != nullptr,
        "routes and exposes FDCAN controllers"
    );
    auto ram_write = mcu.value()->router().write(
        fil::stm32g4::FdcanMessageRam::baseAddress - 0x40000000U,
        fil::mem::AccessSize::word,
        0x12345678U,
        {}
    );
    auto ram_read = mcu.value()->router().read(
        fil::stm32g4::FdcanMessageRam::baseAddress - 0x40000000U,
        fil::mem::AccessSize::word,
        {}
    );
    fil::test::check(ram_write && ram_read && ram_read.value() == 0x12345678U,
                     "routes shared FDCAN message RAM");
}

} // namespace

void runStm32G4Tests() {
    routesIntegratedPeripherals();
}
