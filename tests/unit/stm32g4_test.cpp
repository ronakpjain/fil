#include "fil/cortexm/system_control.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace {

TEST(Stm32G4Test, FdcanInterruptRependsUntilSourceCleared) {
    using Fdcan = fil::stm32g4::FdcanPeripheral;
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true);
    ASSERT_TRUE(mcu);
    auto* fdcan = mcu.value()->fdcan("FDCAN1");
    ASSERT_NE(fdcan, nullptr);
    ASSERT_TRUE(system.write(0xe100U, fil::mem::AccessSize::word, 1U << 21U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::cccrOffset, fil::mem::AccessSize::word, 0U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::ieOffset, fil::mem::AccessSize::word,
                            Fdcan::interruptTransmissionComplete, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::ileOffset, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::txbtieOffset, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::txbarOffset, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_EQ(system.nextPending(0U, 0U, 0U), 37U);

    system.enter(37U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
    system.leave(37U); // ISR did not acknowledge the peripheral source.
    EXPECT_TRUE(system.hasEnabledPending());
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 37U);

    system.enter(37U);
    ASSERT_TRUE(fdcan->write(Fdcan::irOffset, fil::mem::AccessSize::word,
                            Fdcan::interruptTransmissionComplete, {}));
    system.leave(37U);
    EXPECT_FALSE(system.hasEnabledPending());
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, TimerInterruptRependsUntilSourceCleared) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true);
    ASSERT_TRUE(mcu);
    auto& timer = mcu.value()->router(); // TIM2 is at peripheral offset zero.
    ASSERT_TRUE(system.write(0xe100U, fil::mem::AccessSize::word, 1U << 28U, {}));
    ASSERT_TRUE(timer.write(0x0cU, fil::mem::AccessSize::word, 1U, {})); // DIER.UIE
    ASSERT_TRUE(timer.write(0x14U, fil::mem::AccessSize::word, 1U, {})); // EGR.UG
    ASSERT_EQ(system.nextPending(0U, 0U, 0U), 44U);
    system.enter(44U);
    system.leave(44U);
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 44U);
    system.enter(44U);
    ASSERT_TRUE(timer.write(0x10U, fil::mem::AccessSize::word, 0U, {})); // Clear SR.UIF.
    system.leave(44U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, UsartInterruptStopsRependingAfterReceiveConsumed) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system);
    ASSERT_TRUE(mcu);
    auto* usart = mcu.value()->usart("USART1");
    ASSERT_NE(usart, nullptr);
    ASSERT_TRUE(system.write(0xe104U, fil::mem::AccessSize::word, 1U << 5U, {}));
    ASSERT_TRUE(usart->write(0U, fil::mem::AccessSize::word, 1U | (1U << 5U), {}));
    usart->injectRx(0x42U);
    ASSERT_EQ(system.nextPending(0U, 0U, 0U), 53U);
    system.enter(53U);
    system.leave(53U);
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 53U);
    system.enter(53U);
    const auto received = usart->read(0x24U, fil::mem::AccessSize::word, {});
    ASSERT_TRUE(received);
    EXPECT_EQ(received.value(), 0x42U);
    system.leave(53U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, SharedTimerIrqRemainsAssertedUntilAllSourcesClear) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system);
    ASSERT_TRUE(mcu);
    auto& router = mcu.value()->router();
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(system.write(0xe100U, word, 1U << 25U, {}));
    // TIM1 update and TIM16 share IRQ25. Both have UIE/UIF set.
    for (const std::uint32_t base : {0x12c00U, 0x14400U}) {
        ASSERT_TRUE(router.write(base + 0x0cU, word, 1U, {}));
        ASSERT_TRUE(router.write(base + 0x14U, word, 1U, {}));
    }
    ASSERT_EQ(system.nextPending(0U, 0U, 0U), 41U);
    system.enter(41U);
    ASSERT_TRUE(router.write(0x12c10U, word, 0U, {})); // Clear only TIM1.
    system.leave(41U);
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 41U);
    system.enter(41U);
    ASSERT_TRUE(router.write(0x1440cU, word, 0U, {})); // Disable TIM16 interrupt.
    system.leave(41U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
    ASSERT_TRUE(router.write(0x1440cU, word, 1U, {})); // UIF still set: assert again.
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 41U);
    system.enter(41U);
    mcu.value()->reset();
    system.leave(41U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, FdcanInterruptLineRoutingAndDisableUpdateNvicLevels) {
    using Fdcan = fil::stm32g4::FdcanPeripheral;
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system);
    ASSERT_TRUE(mcu);
    auto* fdcan = mcu.value()->fdcan("FDCAN1");
    ASSERT_NE(fdcan, nullptr);
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(system.write(0xe100U, word, (1U << 21U) | (1U << 22U), {}));
    ASSERT_TRUE(fdcan->write(Fdcan::cccrOffset, word, 0U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::ieOffset, word, Fdcan::interruptTransmissionComplete, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::ileOffset, word, 3U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::txbtieOffset, word, 1U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::txbarOffset, word, 1U, {}));
    ASSERT_EQ(system.nextPending(0U, 0U, 0U), 37U);
    system.enter(37U);
    ASSERT_TRUE(fdcan->write(Fdcan::ilsOffset, word, 1U << 2U, {}));
    system.leave(37U);
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 38U);
    system.enter(38U);
    ASSERT_TRUE(fdcan->write(Fdcan::ileOffset, word, 0U, {}));
    system.leave(38U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
    ASSERT_TRUE(fdcan->write(Fdcan::ileOffset, word, 2U, {}));
    EXPECT_EQ(system.nextPending(0U, 0U, 0U), 38U);
    system.enter(38U);
    fdcan->reset();
    system.leave(38U);
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, ResetDoesNotRelatchStaleSharedInterruptSources) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system);
    ASSERT_TRUE(mcu);
    constexpr auto word = fil::mem::AccessSize::word;
    for (const std::uint32_t base : {0x12c00U, 0x14400U}) {
        ASSERT_TRUE(mcu.value()->router().write(base + 0x0cU, word, 1U, {}));
        ASSERT_TRUE(mcu.value()->router().write(base + 0x14U, word, 1U, {}));
    }
    // Board::reset resets SystemControl before the peripherals.
    system.reset(0x08000000U);
    mcu.value()->reset();
    ASSERT_TRUE(system.write(0xe100U, word, 1U << 25U, {}));
    EXPECT_FALSE(system.hasEnabledPending());
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, DestructionDeassertsPeripheralInterruptLines) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system);
    ASSERT_TRUE(mcu);
    ASSERT_TRUE(system.write(0xe100U, fil::mem::AccessSize::word, 1U << 28U, {}));
    ASSERT_TRUE(mcu.value()->router().write(0x0cU, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_TRUE(mcu.value()->router().write(0x14U, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_EQ(system.nextPending(0U, 0U, 0U), 44U);
    system.enter(44U);
    mcu.value().reset();
    system.leave(44U);
    EXPECT_FALSE(system.hasEnabledPending());
    EXPECT_FALSE(system.nextPending(0U, 0U, 0U));
}

TEST(Stm32G4Test, RoutesIntegratedPeripherals) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true, 16'000'000U);
    EXPECT_TRUE(mcu.hasValue()) << "constructs integrated STM32G4 peripheral map";
    if (!mcu) return;

    EXPECT_TRUE(mcu.value()->adc("ADC1")->sampleHistoryEnabled())
        << "integrated ADC diagnostics default to enabled";
    mcu.value()->setAdcDiagnosticsEnabled(false);
    EXPECT_TRUE(!mcu.value()->adc("ADC1")->sampleHistoryEnabled() &&
                !mcu.value()->adc("ADC4")->sampleHistoryEnabled())
        << "integrated ADC diagnostics toggle applies to every ADC";

    auto rcc_write = mcu.value()->router().write(0x00021000U, fil::mem::AccessSize::word, 1U << 8U, {});
    auto rcc_read = mcu.value()->router().read(0x00021000U, fil::mem::AccessSize::word, {});
    EXPECT_TRUE(rcc_write && rcc_read && (rcc_read.value() & (1U << 10U)) != 0)
        << "routes RCC and asserts HSI ready";
    auto hse_enable = mcu.value()->router().write(
        0x00021000U, fil::mem::AccessSize::word, 1U << 16U, {}
    );
    auto hse_select = mcu.value()->router().write(
        0x00021008U, fil::mem::AccessSize::word, 2U, {}
    );
    EXPECT_TRUE(hse_enable && hse_select && mcu.value()->rcc().systemClockHz() == 16'000'000U)
        << "uses the configured HSE frequency";

    fil::config::BoardConfig board;
    board.gpio.push_back({"PB7", "output", false, true});
    const auto uart_log = std::filesystem::temp_directory_path() / "fil-stm32g4-uart.bin";
    board.usart.push_back({"USART1", uart_log, {0x42U}});
    board.spi.push_back({"SPI1", "echo"});
    EXPECT_TRUE(mcu.value()->configure(board).hasValue()) << "applies board device configuration";
    auto gpio_write = mcu.value()->router().write(0x08000418U, fil::mem::AccessSize::word, 1U << 7U, {});
    EXPECT_TRUE(gpio_write && mcu.value()->gpio("GPIOB")->output(7))
        << "routes GPIO BSRR side effect";
    auto uart_read = mcu.value()->router().read(0x00013824U, fil::mem::AccessSize::word, {});
    EXPECT_TRUE(uart_read && (uart_read.value() & 0xffU) == 0x42U)
        << "routes scripted USART RX byte";
    auto uart_write = mcu.value()->router().write(0x00013828U, fil::mem::AccessSize::word, 0x5aU, {});
    std::ifstream uart_input(uart_log, std::ios::binary);
    const int logged_byte = uart_input.get();
    EXPECT_TRUE(uart_write && logged_byte == 0x5a) << "writes configured USART TX byte log";
    std::error_code remove_error;
    std::filesystem::remove(uart_log, remove_error);
    auto fdcan_read = mcu.value()->router().read(
        fil::stm32g4::FdcanPeripheral::baseAddresses[0] - 0x40000000U +
            fil::stm32g4::FdcanPeripheral::cccrOffset,
        fil::mem::AccessSize::word,
        {}
    );
    EXPECT_TRUE(fdcan_read && mcu.value()->fdcan("FDCAN1") != nullptr)
        << "routes and exposes FDCAN controllers";
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
    EXPECT_TRUE(ram_write && ram_read && ram_read.value() == 0x12345678U)
        << "routes shared FDCAN message RAM";
}

} // namespace
