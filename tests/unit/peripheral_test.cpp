#include "fil/mem/memory_bus.hpp"
#include "fil/stm32g4/peripheral.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr fil::mem::AccessContext read_context{fil::mem::AccessType::data_read, 0};
constexpr fil::mem::AccessContext write_context{fil::mem::AccessType::data_write, 0};

TEST(PeripheralTest, MergesRegisterByteLanesAndRejectsOutOfBlockWrites) {
    fil::stm32g4::RegisterPeripheral registers("register-contract", 8U);
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(registers.write(0U, word, 0x11223344U, write_context));
    ASSERT_TRUE(registers.write(4U, word, 0x55667788U, write_context));
    ASSERT_TRUE(registers.write(1U, fil::mem::AccessSize::byte, 0xabU, write_context));
    ASSERT_TRUE(registers.write(2U, fil::mem::AccessSize::halfword, 0xcdefU, write_context));
    EXPECT_EQ(registers.peekRegister(0U), 0xcdefab44U);
    // The generic backing supports cross-register lanes; individual device
    // access restrictions need their own tests rather than inheriting this rule.
    ASSERT_TRUE(registers.write(3U, fil::mem::AccessSize::halfword, 0x9876U, write_context));
    EXPECT_EQ(registers.peekRegister(0U), 0x76efab44U);
    EXPECT_EQ(registers.peekRegister(4U), 0x55667798U);
    EXPECT_EQ(registers.read(3U, fil::mem::AccessSize::halfword, read_context).value(), 0x9876U);
    EXPECT_FALSE(registers.write(7U, word, 0U, write_context));
    EXPECT_FALSE(registers.read(8U, word, read_context));
    EXPECT_EQ(registers.peekRegister(4U), 0x55667798U);
    registers.reset();
    EXPECT_EQ(registers.peekRegister(0U), 0U);
    EXPECT_EQ(registers.peekRegister(4U), 0U);
}

TEST(PeripheralTest, StoresRegistersAndUnknownMmio) {
    fil::stm32g4::PwrPeripheral pwr;
    EXPECT_TRUE(pwr.write(0, fil::mem::AccessSize::word, 0x11223344U, write_context).hasValue())
        << "writes a register word";
    EXPECT_TRUE(pwr.write(1, fil::mem::AccessSize::byte, 0xaaU, write_context).hasValue())
        << "merges a partial register write";
    const auto merged = pwr.read(0, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(merged && merged.value() == 0x1122aa44U)
        << "register storage merges little-endian byte writes";
    pwr.reset();
    const auto reset = pwr.read(0, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(reset && reset.value() == (1U << 9U))
        << "register reset restores STM32G4 voltage-scaling range 1";

    fil::stm32g4::UnknownMmioDevice unknown("fallback", 0x40000000U);
    EXPECT_TRUE(unknown.write(3, fil::mem::AccessSize::halfword, 0xabcdU, write_context).hasValue())
        << "lenient unknown MMIO accepts writes";
    const auto unknown_read = unknown.read(3, fil::mem::AccessSize::halfword, read_context);
    EXPECT_TRUE(unknown_read && unknown_read.value() == 0xabcdU)
        << "unknown MMIO preserves sparse written bytes";
    unknown.setStrict(true);
    const auto strict = unknown.read(0, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(!strict && strict.fault().address == 0x40000000U)
        << "strict unknown MMIO returns an absolute device fault";
}

TEST(PeripheralTest, ModelsRangeOneBoostControl) {
    fil::stm32g4::PwrPeripheral pwr;
    const auto normal = pwr.read(0x80U, fil::mem::AccessSize::word, read_context);
    ASSERT_TRUE(normal);
    EXPECT_EQ(normal.value(), 1U << 8U);
    ASSERT_TRUE(pwr.write(0x80U, fil::mem::AccessSize::word,
                          normal.value() & ~(1U << 8U), write_context));
    const auto boost = pwr.read(0x80U, fil::mem::AccessSize::word, read_context);
    ASSERT_TRUE(boost);
    EXPECT_EQ(boost.value(), 0U);
    pwr.reset();
    const auto reset = pwr.read(0x80U, fil::mem::AccessSize::word, read_context);
    ASSERT_TRUE(reset);
    EXPECT_EQ(reset.value(), 1U << 8U);
    EXPECT_FALSE(pwr.read(0x84U, fil::mem::AccessSize::word, read_context));
}

TEST(PeripheralTest, ModelsClockFlashAndGpioStartup) {
    fil::stm32g4::RccPeripheral rcc;
    EXPECT_TRUE(rcc.write(0, fil::mem::AccessSize::word, (1U << 16U) | (1U << 24U), write_context)
            .hasValue())
        << "writes RCC CR";
    const auto clock_ready = rcc.read(0, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(clock_ready &&
                (clock_ready.value() & ((1U << 17U) | (1U << 25U))) == ((1U << 17U) | (1U << 25U)))
        << "RCC reflects HSE and PLL ready bits";
    EXPECT_TRUE(rcc.write(8, fil::mem::AccessSize::word, 3U, write_context).hasValue())
        << "selects PLL system clock";
    const auto selected = rcc.read(8, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(selected && (selected.value() & 0x0fU) == 0x0fU) << "RCC reflects SW in SWS";

    fil::stm32g4::FlashPeripheral flash;
    const auto locked = flash.read(0x14, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(locked && (locked.value() & (1U << 31U)) != 0U) << "FLASH control starts locked";
    EXPECT_TRUE(
        flash.write(0x08, fil::mem::AccessSize::word, 0x45670123U, write_context).hasValue())
        << "accepts first FLASH key";
    EXPECT_TRUE(
        flash.write(0x08, fil::mem::AccessSize::word, 0xcdef89abU, write_context).hasValue())
        << "accepts second FLASH key";
    const auto unlocked = flash.read(0x14, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(unlocked && (unlocked.value() & (1U << 31U)) == 0U)
        << "FLASH key sequence clears LOCK";

    fil::sim::EventLoop loop;
    fil::stm32g4::GpioPeripheral gpio("GPIOC", &loop);
    std::vector<fil::stm32g4::GpioTransition> callbacks;
    gpio.setOutputCallback([&](const std::uint32_t pin, const bool high, const fil::sim::SimTimeNs time) {
        callbacks.push_back({time, pin, high});
    });
    EXPECT_TRUE(
        gpio.write(0, fil::mem::AccessSize::word, 1U << (13U * 2U), write_context).hasValue())
        << "configures GPIO output mode";
    EXPECT_TRUE(gpio.write(0x18, fil::mem::AccessSize::word, 1U << 13U, write_context).hasValue())
        << "GPIO BSRR sets an output";
    const auto output = gpio.read(0x14, fil::mem::AccessSize::word, read_context);
    const auto mirrored = gpio.read(0x10, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(output && (output.value() & (1U << 13U)) != 0U) << "GPIO stores ODR state";
    EXPECT_TRUE(mirrored && (mirrored.value() & (1U << 13U)) != 0U)
        << "GPIO IDR mirrors an undriven output";
    gpio.setInput(13, false);
    const auto driven = gpio.read(0x10, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(driven && (driven.value() & (1U << 13U)) == 0U)
        << "external GPIO input overrides output mirroring";
    EXPECT_TRUE(callbacks.size() == 1 && callbacks.front().pin == 13U)
        << "GPIO reports output transitions once";
}

TEST(PeripheralTest, ModelsUsartAndSpiDataPaths) {
    fil::sim::EventLoop loop;
    fil::sim::TraceRecorder trace;
    fil::stm32g4::UsartPeripheral usart("USART1", &loop, &trace);
    std::int32_t usart_interrupts = 0;
    usart.setInterruptCallback([&]() { ++usart_interrupts; });
    EXPECT_TRUE(usart
            .write(0, fil::mem::AccessSize::word, 1U | (1U << 2U) | (1U << 3U) | (1U << 5U),
                write_context)
            .hasValue())
        << "enables USART RX interrupt";
    usart.injectRx(0x42);
    const auto status = usart.read(0x1c, fil::mem::AccessSize::word, read_context);
    const auto received = usart.read(0x24, fil::mem::AccessSize::byte, read_context);
    EXPECT_TRUE(status && (status.value() & (1U << 5U)) != 0U)
        << "USART sets RXNE for queued input";
    EXPECT_TRUE(received && received.value() == 0x42U) << "USART RDR pops scripted input";
    EXPECT_TRUE(usart.write(0x28, fil::mem::AccessSize::byte, 'A', write_context).hasValue())
        << "USART accepts a transmit byte";
    EXPECT_TRUE(usart.txLog().size() == 1 && usart.txLog().front().value == 'A')
        << "USART records transmitted bytes";
    EXPECT_TRUE(usart_interrupts > 0) << "USART signals enabled receive interrupts";

    fil::stm32g4::SpiPeripheral spi("SPI1", &loop, &trace);
    spi.setEcho(true);
    EXPECT_TRUE(spi.write(0x0c, fil::mem::AccessSize::halfword, 0x1234U, write_context).hasValue())
        << "SPI accepts a frame";
    const auto response = spi.read(0x0c, fil::mem::AccessSize::halfword, read_context);
    EXPECT_TRUE(response && response.value() == 0x1234U)
        << "SPI echo device returns the transmitted frame";
    EXPECT_TRUE(spi.transferLog().size() == 1) << "SPI records complete transactions";
    EXPECT_TRUE(trace.records().size() >= 3) << "USART and SPI emit device-facing trace records";
}

TEST(PeripheralTest, SignalsUsartDmaRequestsForTxAndRx) {
    fil::sim::EventLoop loop;
    fil::sim::TraceRecorder trace;
    fil::stm32g4::UsartPeripheral usart("USART1", &loop, &trace);
    std::int32_t tx_requests = 0;
    std::int32_t rx_requests = 0;
    usart.setDmaRequestCallback([&](const bool transmit) {
        if (transmit) ++tx_requests;
        else ++rx_requests;
    });
    // No DMA enabled: RX data must not request DMA.
    usart.injectRx(0x42);
    EXPECT_EQ(tx_requests, 0);
    EXPECT_EQ(rx_requests, 0);
    // Enable DMAR (CR3.6): existing RXNE should request once, and RDR pop
    // with remaining data should request again.
    EXPECT_TRUE(usart.write(0x08, fil::mem::AccessSize::word, 1U << 6U, write_context).hasValue())
        << "enables USART RX DMA";
    EXPECT_EQ(rx_requests, 1) << "RX DMA requested for queued input";
    EXPECT_EQ(tx_requests, 0);
    usart.injectRx(0x43);
    EXPECT_EQ(rx_requests, 2) << "RX DMA requested for newly injected byte";
    const auto first = usart.read(0x24, fil::mem::AccessSize::byte, read_context);
    EXPECT_TRUE(first && first.value() == 0x42U);
    EXPECT_EQ(rx_requests, 3) << "RX DMA requested after RDR pop with data remaining";
    // Enable DMAT (CR3.7): TXE ready should request TX DMA once. RX also
    // re-requests once because one byte (0x43) still remains queued.
    EXPECT_TRUE(usart.write(0x08, fil::mem::AccessSize::word, (1U << 6U) | (1U << 7U), write_context).hasValue())
        << "enables USART TX DMA";
    EXPECT_EQ(tx_requests, 1) << "TX DMA requested when DMAT enabled with TXE ready";
    EXPECT_EQ(rx_requests, 4) << "RX DMA re-requested for remaining queued byte on CR3 update";
    // Suppress during burst: no callbacks while suppressed.
    usart.setDmaSuppress(true);
    usart.injectRx(0x44);
    EXPECT_EQ(rx_requests, 4) << "DMA suppressed during burst";
    usart.setDmaSuppress(false);
}

TEST(PeripheralTest, DrivesTimerAndAdcFromSimulatedTime) {
    fil::sim::EventLoop loop;
    fil::stm32g4::TimerPeripheral timer("TIM1", 1000000, &loop);
    std::int32_t timer_interrupts = 0;
    timer.setInterruptCallback([&]() { ++timer_interrupts; });
    EXPECT_TRUE(timer.write(0x2c, fil::mem::AccessSize::word, 9, write_context).hasValue())
        << "sets timer auto-reload";
    EXPECT_TRUE(timer.write(0x0c, fil::mem::AccessSize::word, 1, write_context).hasValue())
        << "enables timer update interrupt";
    EXPECT_TRUE(timer.write(0x00, fil::mem::AccessSize::word, 1, write_context).hasValue())
        << "starts timer";
    EXPECT_TRUE(loop.runDueEvents(9999).events_executed == 0)
        << "timer does not update before its period";
    EXPECT_TRUE(loop.runDueEvents(10000).events_executed == 1)
        << "timer schedules an update at its exact period";
    const auto timer_status = timer.read(0x10, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(timer_status && (timer_status.value() & 1U) != 0U && timer_interrupts == 1)
        << "timer sets UIF and signals UIE";

    fil::stm32g4::AdcPeripheral adc("ADC1", &loop);
    adc.setConversionDelay(5'000);
    adc.setChannelValue(2, 2048);
    EXPECT_TRUE(adc.write(0x30, fil::mem::AccessSize::word, 2U << 6U, write_context).hasValue())
        << "selects ADC channel";
    EXPECT_TRUE(
        adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context).hasValue())
        << "enables and starts ADC";
    EXPECT_TRUE(loop.runDueEvents(14999).events_executed == 0)
        << "ADC conversion waits for configured delay";
    EXPECT_TRUE(loop.runDueEvents(15000).events_executed == 1)
        << "ADC completes conversion deterministically";
    const auto adc_status = adc.read(0, fil::mem::AccessSize::word, read_context);
    const auto adc_data = adc.read(0x40, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(adc_status && (adc_status.value() & ((1U << 2U) | (1U << 3U))) != 0U)
        << "ADC sets EOC/EOS";
    EXPECT_TRUE(adc_data && adc_data.value() == 2048) << "ADC DR returns configured channel value";
}

TEST(PeripheralTest, SequencesAdcChannelsWithRegisterDerivedTiming) {
    fil::sim::EventLoop loop;
    fil::stm32g4::AdcPeripheral adc("ADC1", &loop);
    adc.setInputClockHz(16'000'000U);
    adc.setChannelValue(2, 2002U);
    adc.setChannelValue(3, 3003U);
    const std::uint32_t sequence = 1U | (2U << 6U) | (3U << 12U);
    EXPECT_TRUE(adc.write(0x14, fil::mem::AccessSize::word, (7U << 6U) | (7U << 9U), write_context)
                    .hasValue() &&
                adc.write(0x30, fil::mem::AccessSize::word, sequence, write_context).hasValue())
        << "configures a two-rank ADC sequence with 640.5-cycle sampling";
    EXPECT_TRUE(
        adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context).hasValue())
        << "starts a register-timed ADC sequence";
    EXPECT_TRUE(loop.runDueEvents(40'812U).events_executed == 0U)
        << "first ADC rank waits for sample and conversion cycles";
    EXPECT_TRUE(loop.runDueEvents(40'813U).events_executed == 1U && adc.samples().size() == 1U &&
                adc.samples()[0].channel == 2U)
        << "first ADC rank completes at its clock-derived deadline";
    EXPECT_TRUE(loop.runDueEvents(81'626U).events_executed == 1U && adc.samples().size() == 2U &&
                adc.samples()[1].channel == 3U)
        << "ADC advances through the configured channel sequence";
    const auto data = adc.read(0x40, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(data && data.value() == 3003U)
        << "ADC data register contains the final sequence rank";
}

TEST(PeripheralTest, LazilySynchronizesUnobservedContinuousAdc) {
    fil::sim::EventLoop loop;
    fil::sim::TraceRecorder trace;
    trace.setEnabled(false);
    fil::stm32g4::AdcPeripheral adc("ADC1", &loop, &trace);
    adc.setSampleHistoryEnabled(false);
    adc.setConversionDelay(5'000);
    std::vector<fil::sim::SimTimeNs> provider_times;
    adc.setChannelProvider([&](const std::uint32_t, const fil::sim::SimTimeNs now) {
        provider_times.push_back(now);
        return static_cast<std::uint16_t>(now / 5'000U);
    });

    EXPECT_TRUE(adc.write(0x0c, fil::mem::AccessSize::word, 1U << 13U, write_context).hasValue())
        << "enables ADC continuous mode for lazy conversion test";
    EXPECT_TRUE(
        adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context).hasValue())
        << "starts unobserved continuous ADC conversion";
    EXPECT_TRUE(loop.pending() == 0)
        << "unobserved continuous ADC does not enqueue conversion callbacks";
    EXPECT_TRUE(loop.runDueEvents(25'000).events_executed == 0)
        << "virtual time crosses lazy ADC conversions without callbacks";
    EXPECT_TRUE(provider_times.empty())
        << "lazy ADC defers provider sampling until firmware observes registers";

    const auto status = adc.read(0x00, fil::mem::AccessSize::word, read_context);
    const auto data = adc.read(0x40, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(status && (status.value() & ((1U << 2U) | (1U << 3U))) != 0U)
        << "lazy ADC synchronizes EOC and EOS on MMIO access";
    EXPECT_TRUE(
        data && data.value() == 5U && provider_times == std::vector<fil::sim::SimTimeNs>{25'000})
        << "lazy ADC samples only the most recent due conversion timestamp";
    EXPECT_TRUE(adc.samples().empty() && trace.records().empty())
        << "lazy ADC does not synthesize disabled history or trace diagnostics";

    adc.setSampleHistoryEnabled(true);
    EXPECT_TRUE(loop.pending() == 1)
        << "enabling ADC history restores the exact next conversion event";
    EXPECT_TRUE(loop.runDueEvents(29'999).events_executed == 0)
        << "observable ADC retains its original next deadline";
    EXPECT_TRUE(loop.runDueEvents(30'000).events_executed == 1)
        << "observable continuous ADC fires at the exact next deadline";
    EXPECT_TRUE(adc.samples().size() == 1 && adc.samples().front().time_ns == 30'000)
        << "newly enabled ADC history begins with future conversions only";
}

TEST(PeripheralTest, PreservesObservableAndSingleShotAdcEvents) {
    fil::sim::EventLoop interrupt_loop;
    fil::sim::TraceRecorder interrupt_trace;
    interrupt_trace.setEnabled(false);
    fil::stm32g4::AdcPeripheral interrupt_adc("ADC1", &interrupt_loop, &interrupt_trace);
    interrupt_adc.setConversionDelay(5'000);
    interrupt_adc.setSampleHistoryEnabled(false);
    std::int32_t interrupts = 0;
    interrupt_adc.setInterruptCallback([&]() { ++interrupts; });
    EXPECT_TRUE(
        interrupt_adc.write(0x04, fil::mem::AccessSize::word, 1U << 2U, write_context).hasValue())
        << "enables ADC EOC interrupt";
    EXPECT_TRUE(
        interrupt_adc.write(0x0c, fil::mem::AccessSize::word, 1U << 13U, write_context).hasValue())
        << "enables interrupt-observed continuous ADC mode";
    EXPECT_TRUE(
        interrupt_adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context)
            .hasValue())
        << "starts interrupt-observed continuous ADC";
    EXPECT_TRUE(interrupt_loop.pending() == 1 &&
                interrupt_loop.runDueEvents(5'000).events_executed == 1 && interrupts == 1)
        << "enabled EOC interrupt retains per-conversion scheduling";

    fil::sim::EventLoop single_loop;
    fil::sim::TraceRecorder single_trace;
    single_trace.setEnabled(false);
    fil::stm32g4::AdcPeripheral single_adc("ADC2", &single_loop, &single_trace);
    single_adc.setConversionDelay(5'000);
    single_adc.setSampleHistoryEnabled(false);
    single_adc.setChannelValue(0, 1234);
    EXPECT_TRUE(single_adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context)
            .hasValue())
        << "starts unobserved single-shot ADC";
    EXPECT_TRUE(single_loop.pending() == 1 && single_loop.runDueEvents(5'000).events_executed == 1)
        << "single-shot ADC always retains its exact completion event";
    const auto single_data = single_adc.read(0x40, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(single_data && single_data.value() == 1234U)
        << "unobserved single-shot ADC still materializes DR";
}

TEST(PeripheralTest, ReportsPeripheralInterruptLevelsAndRependsOnEnable) {
    std::vector<std::pair<std::uint32_t, bool>> levels;

    fil::stm32g4::TimerPeripheral timer("TIM1");
    timer.setInterruptLevelCallback([&](const std::uint32_t line, const bool asserted) {
        levels.emplace_back(line, asserted);
    });
    levels.clear();
    EXPECT_TRUE(timer.write(0x14, fil::mem::AccessSize::word, 1U, write_context).hasValue());
    EXPECT_TRUE(timer.write(0x0c, fil::mem::AccessSize::word, 1U, write_context).hasValue());
    ASSERT_EQ(levels.size(), 1U);
    EXPECT_TRUE(levels.back() == std::make_pair(0U, true));
    EXPECT_TRUE(timer.write(0x10, fil::mem::AccessSize::word, 0U, write_context).hasValue());
    ASSERT_EQ(levels.size(), 2U);
    EXPECT_TRUE(levels.back() == std::make_pair(0U, false));

    fil::stm32g4::SpiPeripheral spi;
    levels.clear();
    spi.setInterruptLevelCallback([&](const std::uint32_t line, const bool asserted) {
        levels.emplace_back(line, asserted);
    });
    levels.clear();
    EXPECT_TRUE(spi.write(0x04, fil::mem::AccessSize::word, 1U << 7U, write_context).hasValue());
    ASSERT_EQ(levels.size(), 1U);
    EXPECT_TRUE(levels.back() == std::make_pair(0U, true));
    EXPECT_TRUE(spi.write(0x04, fil::mem::AccessSize::word, 0U, write_context).hasValue());
    ASSERT_EQ(levels.size(), 2U);
    EXPECT_TRUE(levels.back() == std::make_pair(0U, false));

    fil::stm32g4::AdcPeripheral adc;
    levels.clear();
    adc.setInterruptLevelCallback([&](const std::uint32_t line, const bool asserted) {
        levels.emplace_back(line, asserted);
    });
    levels.clear();
    EXPECT_TRUE(adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context).hasValue());
    EXPECT_TRUE(adc.write(0x04, fil::mem::AccessSize::word, 1U << 2U, write_context).hasValue());
    ASSERT_EQ(levels.size(), 1U);
    EXPECT_TRUE(levels.back() == std::make_pair(0U, true));
    EXPECT_TRUE(adc.write(0x04, fil::mem::AccessSize::word, 0U, write_context).hasValue());
    ASSERT_EQ(levels.size(), 2U);
    EXPECT_TRUE(levels.back() == std::make_pair(0U, false));

    fil::stm32g4::DmaPeripheral dma("DMA1", 1);
    levels.clear();
    dma.setInterruptLevelCallback([&](const std::uint32_t line, const bool asserted) {
        levels.emplace_back(line, asserted);
    });
    levels.clear();
    EXPECT_TRUE(dma.write(0x0c, fil::mem::AccessSize::word, 1U, write_context).hasValue());
    EXPECT_TRUE(dma.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 3U), write_context).hasValue());
    EXPECT_TRUE(dma.request(1U));
    ASSERT_EQ(levels.size(), 1U);
    EXPECT_TRUE(levels.back() == std::make_pair(0U, true));
    EXPECT_TRUE(dma.write(0x04, fil::mem::AccessSize::word, 1U << 3U, write_context).hasValue());
    ASSERT_EQ(levels.size(), 2U);
    EXPECT_TRUE(levels.back() == std::make_pair(0U, false));
}

TEST(PeripheralTest, AdcInterruptLevelMatchesEnabledStatusBits) {
    fil::sim::EventLoop events;
    fil::stm32g4::AdcPeripheral adc("ADC1", &events);
    adc.setConversionDelay(100U);
    std::vector<bool> levels;
    adc.setInterruptLevelCallback([&](const std::uint32_t line, const bool asserted) {
        if (line == 0U) levels.push_back(asserted);
    });
    EXPECT_EQ(levels, std::vector<bool>{false});
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(adc.write(0x30U, word, 1U, {})); // Two ranks.
    ASSERT_TRUE(adc.write(0x04U, word, 1U << 3U, {})); // Only EOSIE, not EOCIE.
    ASSERT_TRUE(adc.write(0x08U, word, 1U | (1U << 2U), {}));
    EXPECT_EQ(events.runDueEvents(100U).events_executed, 1U);
    EXPECT_EQ(levels, std::vector<bool>{false}); // First rank: EOC, but no EOS.
    ASSERT_TRUE(adc.write(0x04U, word, 1U << 3U, {}));
    EXPECT_EQ(levels, std::vector<bool>{false}); // IER write must not confuse EOC/EOS.
    EXPECT_EQ(events.runDueEvents(200U).events_executed, 1U);
    EXPECT_EQ(levels, (std::vector<bool>{false, true}));
    ASSERT_TRUE(adc.write(0U, word, 1U << 3U, {})); // W1C EOS leaves EOC set.
    EXPECT_EQ(levels, (std::vector<bool>{false, true, false}));
    ASSERT_TRUE(adc.write(0x04U, word, 1U << 2U, {})); // Enable already-set EOC.
    EXPECT_EQ(levels, (std::vector<bool>{false, true, false, true}));
    ASSERT_TRUE(adc.read(0x40U, word, {})); // DR read acknowledges conversion.
    EXPECT_EQ(levels, (std::vector<bool>{false, true, false, true, false}));
}

TEST(PeripheralTest, DmaOwnerLocalAdcTransferRejectsPublicObserversAndSubstitutions) {
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapRam(0x20000000U, 0x100U, "dma-local-ram"));
    fil::sim::TraceRecorder trace;
    fil::stm32g4::DmaPeripheral dma("DMA1", 7U, &memory, nullptr, &trace);
    constexpr auto word = fil::mem::AccessSize::word;
    constexpr std::uint32_t source = 0x50000040U;
    const auto trusted_generation = dma.interruptLevelCallbackGeneration();
    ASSERT_TRUE(dma.write(0x0cU, word, 2U, {}));
    ASSERT_TRUE(dma.write(0x10U, word, source, {}));
    ASSERT_TRUE(dma.write(0x14U, word, 0x20000000U, {}));
    // Halfword peripheral-to-memory, with a writable-RAM destination.
    ASSERT_TRUE(dma.write(0x08U, word, 1U | (1U << 8U) | (1U << 10U), {}));

    EXPECT_TRUE(dma.ownerLocalAdcTransferSafe(1U, &memory, source, trusted_generation))
        << "diagnostic history alone does not prevent trusted owner-local DMA";
    dma.setTransferHistoryEnabled(false);
    EXPECT_TRUE(dma.ownerLocalAdcTransferSafe(1U, &memory, source, trusted_generation));

    dma.setInterruptCallback([](std::uint32_t) {});
    EXPECT_FALSE(dma.ownerLocalAdcTransferSafe(1U, &memory, source, trusted_generation))
        << "legacy DMA interrupt callback is arbitrary user code";
    dma.setInterruptCallback({});

    dma.setInterruptLevelCallback([](std::uint32_t, bool) {});
    EXPECT_FALSE(dma.ownerLocalAdcTransferSafe(1U, &memory, source, trusted_generation))
        << "public IRQ-level callback replacement invalidates the trusted generation";
    dma.setInterruptLevelCallback({});
    const auto refreshed_generation = dma.interruptLevelCallbackGeneration();
    ASSERT_NE(refreshed_generation, trusted_generation);

    trace.setObserver([](const fil::sim::TraceRecord&) {});
    EXPECT_FALSE(dma.ownerLocalAdcTransferSafe(1U, &memory, source, refreshed_generation))
        << "a real trace observer is an external callback even without retained history";
    trace.setObserver({});
    EXPECT_TRUE(dma.ownerLocalAdcTransferSafe(1U, &memory, source, refreshed_generation));

    fil::mem::MemoryBus replacement;
    ASSERT_TRUE(replacement.mapRam(0x20000000U, 0x100U, "replacement-ram"));
    EXPECT_FALSE(dma.ownerLocalAdcTransferSafe(1U, &replacement, source, refreshed_generation))
        << "the exact MemoryBus identity is part of the transfer certificate";
    dma.setMemory(&replacement);
    EXPECT_FALSE(dma.ownerLocalAdcTransferSafe(1U, &memory, source, refreshed_generation));
    EXPECT_TRUE(dma.ownerLocalAdcTransferSafe(1U, &replacement, source, refreshed_generation));
}

TEST(PeripheralTest, DmaGlobalFlagClearDeassertsOnlySelectedChannel) {
    fil::stm32g4::DmaPeripheral dma("DMA1", 8U);
    std::uint32_t levels = 0U;
    dma.setInterruptLevelCallback([&](const std::uint32_t line, const bool asserted) {
        if (asserted) levels |= 1U << line;
        else levels &= ~(1U << line);
    });
    constexpr auto word = fil::mem::AccessSize::word;
    for (const std::uint32_t channel : {1U, 8U}) {
        const std::uint32_t base = 0x08U + (channel - 1U) * 0x14U;
        ASSERT_TRUE(dma.write(base + 4U, word, 1U, {}));
        ASSERT_TRUE(dma.write(base, word, 1U, {}));
        ASSERT_TRUE(dma.request(channel)); // No memory attached: TEIF, interrupt disabled.
        EXPECT_EQ(levels & (1U << (channel - 1U)), 0U);
        ASSERT_TRUE(dma.write(base, word, 1U << 3U, {})); // Enable TEIE after flag set.
        EXPECT_NE(levels & (1U << (channel - 1U)), 0U);
    }
    EXPECT_EQ(levels, 0x81U);
    ASSERT_TRUE(dma.write(0x04U, word, 1U, {})); // CGIF1 clears TEIF1 as well.
    EXPECT_EQ(levels, 0x80U);
    EXPECT_EQ(dma.peekRegister(0U), 0x90000000U);
    ASSERT_TRUE(dma.write(0x07U, fil::mem::AccessSize::byte, 0x10U, {})); // CGIF8.
    EXPECT_EQ(levels, 0U);
    EXPECT_EQ(dma.peekRegister(0U), 0U);
}

TEST(PeripheralTest, UsartTransmissionCompleteCanBeAcknowledged) {
    fil::stm32g4::UsartPeripheral usart;
    std::vector<bool> levels;
    usart.setInterruptLevelCallback([&](const std::uint32_t line, const bool asserted) {
        if (line == 0U) levels.push_back(asserted);
    });
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(usart.write(0U, word, 1U | (1U << 6U), {})); // TCIE and UE.
    EXPECT_EQ(levels, (std::vector<bool>{false, true}));
    ASSERT_TRUE(usart.write(0x20U, word, 1U << 6U, {})); // TCCF.
    EXPECT_EQ(levels, (std::vector<bool>{false, true, false}));
    ASSERT_TRUE(usart.read(0x1cU, word, {})); // Status read must not reassert TC.
    EXPECT_EQ(levels, (std::vector<bool>{false, true, false}));
    ASSERT_TRUE(usart.write(0x28U, word, 0x42U, {}));
    EXPECT_EQ(levels, (std::vector<bool>{false, true, false, true}));
    usart.reset();
    EXPECT_EQ(levels, (std::vector<bool>{false, true, false, true, false}));
}

TEST(PeripheralTest, CompletesDmaAndWatchdogSideEffects) {
    fil::mem::MemoryBus memory;
    EXPECT_TRUE(memory.mapRam(0x20000000U, 64, "dma-ram").hasValue()) << "maps DMA test RAM";
    EXPECT_TRUE(memory.write32(0x20000000U, 0x11223344U).hasValue()) << "initializes DMA source";
    fil::stm32g4::DmaPeripheral dma("DMA1", 7, &memory);
    std::int32_t dma_interrupts = 0;
    dma.setInterruptCallback([&](const std::uint32_t channel) {
        if (channel == 1U) ++dma_interrupts;
    });
    EXPECT_TRUE(dma.write(0x0c, fil::mem::AccessSize::word, 1, write_context).hasValue())
        << "sets DMA item count";
    EXPECT_TRUE(dma.write(0x10, fil::mem::AccessSize::word, 0x20000020U, write_context).hasValue())
        << "sets DMA peripheral address";
    EXPECT_TRUE(dma.write(0x14, fil::mem::AccessSize::word, 0x20000000U, write_context).hasValue())
        << "sets DMA memory address";
    const std::uint32_t control = 1U | (1U << 1U) | (1U << 4U) | (2U << 8U) | (2U << 10U);
    EXPECT_TRUE(dma.write(0x08, fil::mem::AccessSize::word, control, write_context).hasValue())
        << "enables a request-driven DMA channel";
    EXPECT_TRUE(dma.request(1)) << "peripheral request services the DMA channel";
    const auto copied = memory.read32(0x20000020U);
    const auto dma_status = dma.read(0, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(copied && copied.value() == 0x11223344U)
        << "DMA copies configured data through MemoryBus";
    EXPECT_TRUE(dma_status && (dma_status.value() & (1U << 1U)) != 0U)
        << "DMA sets channel transfer-complete flag";
    EXPECT_TRUE(dma_interrupts == 1 && dma.transferLog().front().success)
        << "DMA signals completion callback";
    dma.setTransferHistoryEnabled(false);
    EXPECT_TRUE(dma.write(0x0c, fil::mem::AccessSize::word, 1, write_context).hasValue() &&
                dma.write(0x08, fil::mem::AccessSize::word, control, write_context).hasValue() &&
                dma.request(1) && dma.transferLog().size() == 1U)
        << "DMA can service requests without retaining diagnostic history";

    fil::stm32g4::DmamuxPeripheral dmamux;
    const std::uint64_t initial_routing = dmamux.routingGeneration();
    EXPECT_TRUE(dmamux.write(0, fil::mem::AccessSize::word, 36, write_context).hasValue())
        << "stores DMAMUX request selection";
    EXPECT_TRUE(dmamux.requestForChannel(0) == 36 && dmamux.routingGeneration() > initial_routing)
        << "DMAMUX exposes and versions the selected request";

    fil::sim::EventLoop loop;
    fil::stm32g4::IwdgPeripheral watchdog(true, &loop);
    std::int32_t resets = 0;
    watchdog.setResetCallback([&]() { ++resets; });
    EXPECT_TRUE(watchdog.write(0, fil::mem::AccessSize::word, 0x5555U, write_context).hasValue())
        << "unlocks watchdog registers";
    EXPECT_TRUE(watchdog.write(8, fil::mem::AccessSize::word, 0, write_context).hasValue())
        << "sets watchdog reload counter";
    EXPECT_TRUE(watchdog.write(0, fil::mem::AccessSize::word, 0xccccU, write_context).hasValue())
        << "starts watchdog";
    EXPECT_TRUE(loop.runDueEvents(124999).events_executed == 0)
        << "watchdog remains armed before timeout";
    EXPECT_TRUE(loop.runDueEvents(125000).events_executed == 1 && resets == 1)
        << "watchdog requests reset at deterministic timeout";
}

TEST(PeripheralTest, DecimatesContinuousAdcScansWithoutDriftingSchedule) {
    fil::sim::EventLoop loop;
    fil::sim::TraceRecorder trace;
    trace.setEnabled(false);
    fil::stm32g4::AdcPeripheral adc("ADC1", &loop, &trace);
    adc.setConversionDelay(5'000);
    adc.setDecimation(4);
    EXPECT_TRUE(adc.decimation() == 4U) << "decimation factor is retained";
    adc.setChannelProvider([&](const std::uint32_t, const fil::sim::SimTimeNs now) {
        return static_cast<std::uint16_t>(now / 5'000U);
    });
    std::int32_t interrupts = 0;
    adc.setInterruptCallback([&]() { ++interrupts; });
    EXPECT_TRUE(
        adc.write(0x04, fil::mem::AccessSize::word, (1U << 2U) | (1U << 3U), write_context)
                .hasValue() &&
            adc.write(0x0c, fil::mem::AccessSize::word, 1U << 13U, write_context).hasValue())
        << "enables EOC/EOS interrupts and continuous mode";
    EXPECT_TRUE(
        adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context).hasValue())
        << "starts decimated continuous ADC";

    EXPECT_TRUE(loop.runDueEvents(5'000).events_executed == 1 && interrupts == 1)
        << "kept scan zero materializes on schedule";
    const auto first = adc.read(0x40, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(first && first.value() == 1U) << "DR holds scan zero value";
    // DR acknowledges EOC only; EOS requires its explicit W1C acknowledgement.
    ASSERT_TRUE(adc.write(0U, fil::mem::AccessSize::word, 1U << 3U, write_context));
    EXPECT_TRUE(loop.runDueEvents(19'999).events_executed == 0)
        << "skipped scans enqueue no per-conversion events";
    EXPECT_TRUE(loop.runDueEvents(20'000).events_executed == 1 && interrupts == 1)
        << "decimation gap lands without side effects";
    const auto flags = adc.read(0, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(flags && (flags.value() & ((1U << 2U) | (1U << 3U))) == 0U)
        << "skipped scans raise no EOC/EOS flags";
    EXPECT_TRUE(loop.runDueEvents(25'000).events_executed == 1 && interrupts == 2)
        << "kept scan four materializes without phase drift";
    const auto kept = adc.read(0x40, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(kept && kept.value() == 5U)
        << "kept scans observe exact scheduled timestamps";
    EXPECT_TRUE(adc.samples().size() == 2U && adc.samples()[1].time_ns == 25'000)
        << "sample history records kept scans only";
}

TEST(PeripheralTest, DecimationSkipsMultiRankScansAsWholeScans) {
    fil::sim::EventLoop loop;
    fil::sim::TraceRecorder trace;
    trace.setEnabled(false);
    fil::stm32g4::AdcPeripheral adc("ADC1", &loop, &trace);
    adc.setConversionDelay(5'000);
    adc.setDecimation(2);
    adc.setChannelValue(2, 2002U);
    adc.setChannelValue(3, 3003U);
    std::vector<std::uint32_t> converted;
    adc.setSampleCallback([&](const fil::stm32g4::AdcSample& sample) {
        converted.push_back(sample.channel);
    });
    const std::uint32_t sequence = 1U | (2U << 6U) | (3U << 12U);
    EXPECT_TRUE(
        adc.write(0x0c, fil::mem::AccessSize::word, 1U << 13U, write_context).hasValue() &&
        adc.write(0x30, fil::mem::AccessSize::word, sequence, write_context).hasValue())
        << "configures continuous two-rank ADC sequence";
    EXPECT_TRUE(
        adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context).hasValue())
        << "starts decimated ADC sequence";

    EXPECT_TRUE(loop.runDueEvents(10'000).events_executed == 2 && converted.size() == 2U)
        << "kept scan converts every rank in order";
    EXPECT_TRUE((converted == std::vector<std::uint32_t>{2U, 3U}))
        << "rank order matches the configured sequence";
    EXPECT_TRUE(loop.runDueEvents(20'000).events_executed == 1 && converted.size() == 2U)
        << "skipped scan jumps in one event without DMA requests";
    EXPECT_TRUE(loop.runDueEvents(30'000).events_executed == 2 && converted.size() == 4U)
        << "next kept scan resumes on schedule";
    EXPECT_TRUE((converted == std::vector<std::uint32_t>{2U, 3U, 2U, 3U}))
        << "decimation preserves multi-rank DMA alignment";
    const auto data = adc.read(0x40, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(data && data.value() == 3003U) << "DR holds the last kept rank value";
}

TEST(PeripheralTest, DecimationNeverSkipsSingleShotAdc) {
    fil::sim::EventLoop loop;
    fil::sim::TraceRecorder trace;
    trace.setEnabled(false);
    fil::stm32g4::AdcPeripheral adc("ADC1", &loop, &trace);
    adc.setConversionDelay(5'000);
    adc.setDecimation(4);
    adc.setChannelValue(0, 1234);
    std::int32_t interrupts = 0;
    adc.setInterruptCallback([&]() { ++interrupts; });
    EXPECT_TRUE(
        adc.write(0x04, fil::mem::AccessSize::word, 1U << 2U, write_context).hasValue())
        << "enables single-shot EOC interrupt";
    for (std::int32_t shot = 0; shot < 3; ++shot) {
        EXPECT_TRUE(
            adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context)
                .hasValue())
            << "starts single-shot ADC";
        EXPECT_TRUE(loop.runDueEvents(loop.now() + 5'000).events_executed == 1)
            << "single-shot conversion keeps its exact event";
    }
    EXPECT_TRUE(interrupts == 3) << "decimation never skips single-shot conversions";
    const auto data = adc.read(0x40, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(data && data.value() == 1234U) << "single-shot DR is unaffected";
}

TEST(PeripheralTest, DecimationFactorZeroMeansOne) {
    fil::stm32g4::AdcPeripheral adc;
    adc.setDecimation(0);
    EXPECT_TRUE(adc.decimation() == 1U) << "zero decimation clamps to exact mode";
}

// RM0440 §7.4.1: ON is software controlled; RDY is read-only hardware status.
TEST(PeripheralTest, RccReadyFlagsMirrorEnableAndHonorAbsentHse) {
    fil::stm32g4::RccPeripheral present(true), absent(false);
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(present.write(0, word, (1U << 8U) | (1U << 16U) | (1U << 24U), write_context));
    auto cr = present.read(0, word, read_context);
    ASSERT_TRUE(cr);
    EXPECT_EQ(cr.value() & ((1U << 8U) | (1U << 10U) | (1U << 16U) | (1U << 17U) |
                            (1U << 24U) | (1U << 25U)),
              (1U << 8U) | (1U << 10U) | (1U << 16U) | (1U << 17U) |
                  (1U << 24U) | (1U << 25U));
    ASSERT_TRUE(present.write(0, word, 0, write_context));
    cr = present.read(0, word, read_context);
    ASSERT_TRUE(cr);
    EXPECT_EQ(cr.value() & ((1U << 10U) | (1U << 17U) | (1U << 25U)), 0U);
    ASSERT_TRUE(absent.write(0, word, 1U << 16U, write_context));
    cr = absent.read(0, word, read_context);
    ASSERT_TRUE(cr);
    EXPECT_EQ(cr.value() & (1U << 16U), 1U << 16U);
    EXPECT_EQ(cr.value() & (1U << 17U), 0U);
}

// RM0440 §7.4.27–29: ON is writable; LSERDY/LSIRDY/HSI48RDY are status fields.
TEST(PeripheralTest, RccLowSpeedAndRecoveryOscillatorsMirrorOnAndHardwareReady) {
    fil::stm32g4::RccPeripheral rcc;
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(rcc.write(0x90, word, 1U, write_context));
    ASSERT_TRUE(rcc.write(0x94, word, 1U, write_context));
    ASSERT_TRUE(rcc.write(0x98, word, 1U, write_context));
    EXPECT_EQ(rcc.read(0x90, word, read_context).value() & 3U, 3U);
    EXPECT_EQ(rcc.read(0x94, word, read_context).value() & 3U, 3U);
    EXPECT_EQ(rcc.read(0x98, word, read_context).value() & 3U, 3U);
    // Writing RDY alone cannot keep it high when its matching ON control is zero.
    ASSERT_TRUE(rcc.write(0x90, word, 2U, write_context));
    ASSERT_TRUE(rcc.write(0x94, word, 2U, write_context));
    ASSERT_TRUE(rcc.write(0x98, word, 2U, write_context));
    EXPECT_EQ(rcc.read(0x90, word, read_context).value() & 3U, 0U);
    EXPECT_EQ(rcc.read(0x94, word, read_context).value() & 3U, 0U);
    EXPECT_EQ(rcc.read(0x98, word, read_context).value() & 3U, 0U);
    ASSERT_TRUE(rcc.write(0x90, word, 1U, write_context));
    EXPECT_EQ(rcc.read(0x90, word, read_context).value() & 3U, 3U);
}

// RM0440 §7.2.4, §7.4.3–4: actual SW encodings are HSI16/HSE/PLL for 01/10/11.
TEST(PeripheralTest, RccSysclkPllCalculationIsExplicitlyPermissive) {
    fil::stm32g4::RccPeripheral rcc(true, 8'000'000U);
    constexpr auto word = fil::mem::AccessSize::word;
    EXPECT_EQ(rcc.systemClockHz(), 16'000'000U);
    ASSERT_TRUE(rcc.write(0x0c, word, 3U | (20U << 8U) | (1U << 25U), write_context));
    ASSERT_TRUE(rcc.write(0, word, 1U << 24U, write_context)); // PLLON; model makes PLLRDY immediate.
    ASSERT_TRUE(rcc.write(0x08, word, 3U, write_context));
    EXPECT_EQ(rcc.systemClockHz(), 40'000'000U); // HSE(8 MHz) * N(20) / R(4).
    ASSERT_TRUE(rcc.write(0x08, word, 2U, write_context));
    EXPECT_EQ(rcc.systemClockHz(), 8'000'000U);
    ASSERT_TRUE(rcc.write(0x08, word, 1U, write_context));
    EXPECT_EQ(rcc.systemClockHz(), 16'000'000U);

    fil::stm32g4::RccPeripheral no_hse(false, 8'000'000U);
    ASSERT_TRUE(no_hse.write(0x08, word, 2U, write_context));
    EXPECT_EQ(no_hse.systemClockHz(), 16'000'000U); // permissive fallback, not RM ready-switch behavior.
}

// RM0440 §7.4.6–7: CICR write-one-to-clear corresponding CIFR flags.
TEST(PeripheralTest, RccCicrClearsOnlyWrittenFlagsAndResetRestoresStartup) {
    fil::stm32g4::RccPeripheral rcc;
    constexpr auto word = fil::mem::AccessSize::word;
    ASSERT_TRUE(rcc.write(0x1c, word, 0x3fU, write_context));
    ASSERT_TRUE(rcc.write(0x20, word, 1U << 3U, write_context));
    auto flags = rcc.read(0x1c, word, read_context);
    ASSERT_TRUE(flags);
    EXPECT_EQ(flags.value(), 0x37U);
    rcc.reset();
    EXPECT_EQ(rcc.read(0x1c, word, read_context).value(), 0U);
    EXPECT_EQ(rcc.read(0, word, read_context).value() & ((1U << 8U) | (1U << 10U)),
              (1U << 8U) | (1U << 10U));
}

// RM0440 §6.4.1, §6.4.9, §6.4.22: reset VOS=01, VOSF read-only, CR5.R1MODE.
TEST(PeripheralTest, PwrResetVoltageRangeAndBoostFieldAreStable) {
    fil::stm32g4::PwrPeripheral pwr;
    constexpr auto word = fil::mem::AccessSize::word;
    EXPECT_EQ(pwr.read(0, word, read_context).value() & (3U << 9U), 1U << 9U);
    EXPECT_EQ(pwr.read(0x14, word, read_context).value() & (1U << 10U), 0U);
    ASSERT_TRUE(pwr.write(0, word, 2U << 9U, write_context));
    EXPECT_EQ(pwr.read(0, word, read_context).value() & (3U << 9U), 2U << 9U);
    EXPECT_EQ(pwr.read(0x14, word, read_context).value() & (1U << 10U), 0U);
    ASSERT_TRUE(pwr.write(0x80, word, 0U, write_context));
    EXPECT_EQ(pwr.read(0x80, word, read_context).value() & (1U << 8U), 0U);
    pwr.reset();
    EXPECT_EQ(pwr.read(0, word, read_context).value(), 1U << 9U);
    EXPECT_EQ(pwr.read(0x80, word, read_context).value(), 1U << 8U);
    // VOSF is deliberately simplified to always complete immediately; no regulator delay modeled.
}

// RM0440 §9.3.5, §9.4.5–7, §9.4.11: set wins simultaneous BSRR set/reset.
TEST(PeripheralTest, GpioBsrrPriorityBrrSubwordAndExternalRelease) {
    fil::stm32g4::GpioPeripheral gpio("GPIOC");
    constexpr auto word = fil::mem::AccessSize::word;
    constexpr auto half = fil::mem::AccessSize::halfword;
    constexpr auto byte = fil::mem::AccessSize::byte;
    ASSERT_TRUE(gpio.write(0x14, half, 0U, write_context));
    ASSERT_TRUE(gpio.write(0x18, word, (1U << 2U) | (1U << (16U + 2U)) | (1U << 4U), write_context));
    EXPECT_EQ(gpio.read(0x14, word, read_context).value(), (1U << 2U) | (1U << 4U));
    ASSERT_TRUE(gpio.write(0x28, byte, 1U << 4U, write_context));
    EXPECT_EQ(gpio.read(0x14, word, read_context).value(), 1U << 2U);
    EXPECT_EQ(gpio.read(0x18, word, read_context).value(), 0U);
    ASSERT_TRUE(gpio.write(0, word, 1U << 2U, write_context)); // pin 1 output
    ASSERT_TRUE(gpio.write(0x14, word, 1U << 1U, write_context));
    gpio.setInput(1, false);
    EXPECT_EQ(gpio.read(0x10, word, read_context).value() & (1U << 1U), 0U);
    gpio.releaseInput(1);
    EXPECT_EQ(gpio.read(0x10, word, read_context).value() & (1U << 1U), 1U << 1U);
    EXPECT_EQ(gpio.read(0x10, word, read_context).value() & 0xffff0000U, 0U);
}

// RM0440 §9.4.1: GPIOA=ABFFFFFF, GPIOB=FFFFFEBF, ports C–G=FFFFFFFF.
TEST(PeripheralTest, GpioResetRestoresPortSpecificDebugPinModes) {
    fil::stm32g4::GpioPeripheral port_a("GPIOA"), port_b("GPIOB"), port_c("GPIOC");
    constexpr auto word = fil::mem::AccessSize::word;
    EXPECT_EQ(port_a.read(0, word, read_context).value(), 0xabffffffU);
    EXPECT_EQ(port_b.read(0, word, read_context).value(), 0xfffffebfU);
    EXPECT_EQ(port_c.read(0, word, read_context).value(), 0xffffffffU);
    port_a.reset();
    EXPECT_EQ(port_a.read(0, word, read_context).value(), 0xabffffffU);
}

TEST(PeripheralTest, AdcControlFlagsAndStatusClearing) {
    fil::sim::EventLoop loop;
    fil::stm32g4::AdcPeripheral adc("ADC1", &loop);
    adc.setConversionDelay(10U);
    EXPECT_TRUE(adc.write(0x08, fil::mem::AccessSize::word, (1U << 31U), write_context));
    EXPECT_EQ(adc.read(0x08, fil::mem::AccessSize::word, read_context).value() & (1U << 31U), 0U)
        << "ADCAL is modeled as immediate calibration completion";
    EXPECT_TRUE(adc.write(0x08, fil::mem::AccessSize::word, 1U, write_context));
    EXPECT_NE(adc.read(0x00, fil::mem::AccessSize::word, read_context).value() & 1U, 0U)
        << "ADEN immediately reports ADRDY in this model";

    EXPECT_TRUE(adc.write(0x04, fil::mem::AccessSize::word, (1U << 2U) | (1U << 3U), write_context));
    EXPECT_TRUE(adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context));
    ASSERT_EQ(loop.runDueEvents(10U).events_executed, 1U);
    EXPECT_NE(adc.read(0x00, fil::mem::AccessSize::word, read_context).value() & 0x0cU, 0U);
    EXPECT_TRUE(adc.write(0x00, fil::mem::AccessSize::word, 1U << 3U, write_context));
    EXPECT_EQ(adc.read(0x00, fil::mem::AccessSize::word, read_context).value() & 0x0cU, 1U << 2U)
        << "clearing EOS leaves EOC asserted";
    EXPECT_TRUE(adc.write(0x00, fil::mem::AccessSize::word, 1U << 2U, write_context));
    EXPECT_EQ(adc.read(0x00, fil::mem::AccessSize::word, read_context).value() & 0x0cU, 0U)
        << "clearing EOC leaves EOS clear";

    EXPECT_TRUE(adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context));
    ASSERT_EQ(loop.runDueEvents(20U).events_executed, 1U);
    EXPECT_NE(adc.read(0x00, fil::mem::AccessSize::word, read_context).value() & 0x0cU, 0U);
    EXPECT_EQ(adc.read(0x40, fil::mem::AccessSize::word, read_context).value(), 0U);
    EXPECT_EQ(adc.read(0x00, fil::mem::AccessSize::word, read_context).value() & 0x0cU, 1U << 3U)
        << "a DR read clears EOC but not EOS; EOS requires ISR W1C";
    EXPECT_TRUE(adc.write(0x00, fil::mem::AccessSize::word, 1U << 3U, write_context));
    EXPECT_EQ(adc.read(0x00, fil::mem::AccessSize::word, read_context).value() & 0x0cU, 0U);

    EXPECT_TRUE(adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context));
    EXPECT_TRUE(adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 1U), write_context));
    EXPECT_EQ(adc.read(0x08, fil::mem::AccessSize::word, read_context).value() & 7U, 0U);
    EXPECT_EQ(adc.read(0x00, fil::mem::AccessSize::word, read_context).value() & 1U, 0U)
        << "ADDIS clears ADEN/ADSTART and ADRDY, and cancels conversion";
    EXPECT_EQ(loop.pending(), 0U);
}

TEST(PeripheralTest, AdcSamplingSelectorsAndResolutionTimingCoverSmprBoundary) {
    constexpr std::array<std::uint32_t, 8> sample_half_cycles{5U, 13U, 25U, 49U, 95U, 185U, 495U, 1281U};
    constexpr std::array<std::uint32_t, 4> conversion_half_cycles{25U, 21U, 17U, 13U};
    for (std::uint32_t resolution = 0; resolution < 4U; ++resolution) {
        for (std::uint32_t selector = 0; selector < 8U; ++selector) {
            fil::sim::EventLoop loop;
            fil::stm32g4::AdcPeripheral adc("ADC1", &loop);
            adc.setInputClockHz(16'000'000U);
            std::vector<fil::stm32g4::AdcSample> samples;
            adc.setSampleCallback([&](const fil::stm32g4::AdcSample& sample) { samples.push_back(sample); });
            const std::uint32_t exact_half_cycles = sample_half_cycles[selector] + conversion_half_cycles[resolution];
            const std::uint32_t delay_ns = (exact_half_cycles * 500U + 15U) / 16U;
            ASSERT_TRUE(adc.write(0x0c, fil::mem::AccessSize::word, resolution << 3U, write_context));
            ASSERT_TRUE(adc.write(0x14, fil::mem::AccessSize::word,
                selector << 27U, write_context)); // CH9
            ASSERT_TRUE(adc.write(0x18, fil::mem::AccessSize::word, selector, write_context)); // CH10
            ASSERT_TRUE(adc.write(0x30, fil::mem::AccessSize::word, 1U | (9U << 6U) | (10U << 12U), write_context));
            ASSERT_TRUE(adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context));
            ASSERT_EQ(loop.runDueEvents(delay_ns - 1U).events_executed, 0U)
                << "selector=" << selector << " RES=" << resolution;
            ASSERT_EQ(loop.runDueEvents(delay_ns).events_executed, 1U)
                << "selector=" << selector << " RES=" << resolution;
            ASSERT_EQ(samples.size(), 1U);
            EXPECT_EQ(samples[0].channel, 9U);
            EXPECT_EQ(samples[0].time_ns, delay_ns);
            ASSERT_EQ(loop.runDueEvents(delay_ns * 2U).events_executed, 1U);
            ASSERT_EQ(samples.size(), 2U);
            EXPECT_EQ(samples[1].channel, 10U)
                << "validates SMPR1 channel 9 / SMPR2 channel 10 boundary";
            EXPECT_EQ(samples[1].time_ns, delay_ns * 2U);
        }
    }
}

// RM0440 21.7.6: SMPPLUS affects the shortest selection in both SMPR banks.
TEST(PeripheralTest, AdcSamplePlusAddsOneCycleToShortestSample) {
    for (const std::uint32_t channel : {0U, 9U, 10U, 19U}) {
        fil::sim::EventLoop loop;
        fil::stm32g4::AdcPeripheral adc("ADC1", &loop);
        adc.setInputClockHz(16'000'000U);
        ASSERT_TRUE(adc.write(0x30U, fil::mem::AccessSize::word, channel << 6U, write_context));
        ASSERT_TRUE(adc.write(0x14U, fil::mem::AccessSize::word, 1U << 31U, write_context));
        ASSERT_TRUE(adc.write(0x08U, fil::mem::AccessSize::word, 5U, write_context));
        EXPECT_EQ(loop.runDueEvents(999U).events_executed, 0U);
        EXPECT_EQ(loop.runDueEvents(1'000U).events_executed, 1U)
            << "SMPPLUS adds one sample cycle for channel " << channel;
    }
}

TEST(PeripheralTest, AdcRegularSequenceCoversAllSixteenRanks) {
    fil::sim::EventLoop loop;
    fil::stm32g4::AdcPeripheral adc("ADC1", &loop);
    adc.setConversionDelay(1U);
    std::vector<std::uint32_t> seen;
    std::vector<std::uint32_t> flags;
    adc.setSampleCallback([&](const fil::stm32g4::AdcSample& sample) {
        seen.push_back(sample.channel);
        flags.push_back(static_cast<std::uint32_t>(adc.read(0x00, fil::mem::AccessSize::word, read_context).value()) & 0x0cU);
    });
    std::array<std::uint32_t, 4> sqr{};
    sqr[0] = 15U; // L = 15 means sixteen ranks.
    for (std::uint32_t rank = 0; rank < 16U; ++rank) {
        const std::uint32_t channel = rank;
        const std::uint32_t reg = rank < 4U ? 0U : rank < 9U ? 1U : rank < 14U ? 2U : 3U;
        const std::uint32_t shift = rank < 4U ? 6U + rank * 6U
            : rank < 9U ? (rank - 4U) * 6U
            : rank < 14U ? (rank - 9U) * 6U : (rank - 14U) * 6U;
        sqr[reg] |= channel << shift;
        adc.setChannelValue(channel, static_cast<std::uint16_t>(0x100U + channel));
    }
    for (std::uint32_t i = 0; i < 4U; ++i)
        ASSERT_TRUE(adc.write(0x30U + i * 4U, fil::mem::AccessSize::word, sqr[i], write_context));
    ASSERT_TRUE(adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context));
    ASSERT_EQ(loop.runDueEvents(16U).events_executed, 16U);
    ASSERT_EQ(seen.size(), 16U);
    ASSERT_EQ(flags.size(), 16U);
    for (std::uint32_t i = 0; i < 16U; ++i) {
        EXPECT_EQ(seen[i], i);
        EXPECT_EQ(flags[i], i == 15U ? 0x0cU : 0x04U)
            << "EOC marks each rank; EOS only marks the end of sequence";
    }
    EXPECT_NE(adc.read(0x00, fil::mem::AccessSize::word, read_context).value() & (1U << 3U), 0U)
        << "EOS occurs at final rank";
    EXPECT_EQ(adc.read(0x40, fil::mem::AccessSize::word, read_context).value(), 0x10fU);
}

TEST(PeripheralTest, TimerForcedUpdateStatusAndOnePulseRestart) {
    fil::sim::EventLoop loop;
    fil::stm32g4::TimerPeripheral timer("TIM2", 1'000'000U, &loop);
    std::uint32_t callbacks = 0;
    timer.setUpdateCallback([&](fil::sim::SimTimeNs) { ++callbacks; });
    ASSERT_TRUE(timer.write(0x2c, fil::mem::AccessSize::word, 9U, write_context));
    ASSERT_TRUE(timer.write(0x28, fil::mem::AccessSize::word, 1U, write_context));
    ASSERT_TRUE(timer.write(0x0c, fil::mem::AccessSize::word, 1U, write_context));
    ASSERT_TRUE(timer.write(0x00, fil::mem::AccessSize::word, 1U | (1U << 3U), write_context));
    ASSERT_EQ(loop.runDueEvents(19'999U).events_executed, 0U);
    ASSERT_EQ(timer.read(0x24, fil::mem::AccessSize::word, read_context).value(), 9U);
    ASSERT_EQ(loop.runDueEvents(20'000U).events_executed, 1U);
    EXPECT_EQ(callbacks, 1U);
    EXPECT_EQ(timer.read(0x24, fil::mem::AccessSize::word, read_context).value(), 0U);
    EXPECT_EQ(timer.read(0x00, fil::mem::AccessSize::word, read_context).value() & 1U, 0U)
        << "OPM clears CEN on update";
    ASSERT_TRUE(timer.write(0x10, fil::mem::AccessSize::word, 0U, write_context));
    EXPECT_EQ(timer.read(0x10, fil::mem::AccessSize::word, read_context).value() & 1U, 0U)
        << "UIF uses write-zero-to-clear semantics";
    ASSERT_TRUE(timer.write(0x14, fil::mem::AccessSize::word, 1U, write_context));
    EXPECT_EQ(callbacks, 2U);
    EXPECT_NE(timer.read(0x10, fil::mem::AccessSize::word, read_context).value() & 1U, 0U)
        << "EGR.UG generates an update while stopped";
}

TEST(PeripheralTest, IwdgLockedWritesAndExactReloadDeadline) {
    fil::sim::EventLoop loop;
    fil::stm32g4::IwdgPeripheral watchdog(true, &loop);
    std::uint32_t resets = 0;
    watchdog.setResetCallback([&]() { ++resets; });
    ASSERT_TRUE(watchdog.write(0x04, fil::mem::AccessSize::word, 0U, write_context));
    ASSERT_TRUE(watchdog.write(0x08, fil::mem::AccessSize::word, 1U, write_context));
    EXPECT_EQ(watchdog.read(0x08, fil::mem::AccessSize::word, read_context).value(), 0x0fffU)
        << "PR/RLR writes are ignored until key 0x5555";
    ASSERT_TRUE(watchdog.write(0x00, fil::mem::AccessSize::word, 0x5555U, write_context));
    ASSERT_TRUE(watchdog.write(0x04, fil::mem::AccessSize::word, 0U, write_context));
    ASSERT_TRUE(watchdog.write(0x08, fil::mem::AccessSize::word, 1U, write_context));
    ASSERT_TRUE(watchdog.write(0x00, fil::mem::AccessSize::word, 0xccccU, write_context));
    EXPECT_TRUE(watchdog.running());
    ASSERT_EQ(loop.runDueEvents(249'999U).events_executed, 0U);
    ASSERT_TRUE(watchdog.write(0x00, fil::mem::AccessSize::word, 0xaaaaU, write_context));
    ASSERT_EQ(loop.runDueEvents(499'998U).events_executed, 0U);
    EXPECT_EQ(loop.runDueEvents(499'999U).events_executed, 1U);
    EXPECT_EQ(resets, 1U);
}

TEST(PeripheralTest, TimerPrescalerClockChangeCntWriteAndStopRestartRephase) {
    fil::sim::EventLoop loop;
    fil::stm32g4::TimerPeripheral timer("TIM2", 1'000'000U, &loop);
    ASSERT_TRUE(timer.write(0x2c, fil::mem::AccessSize::word, 99U, write_context));
    ASSERT_TRUE(timer.write(0x00, fil::mem::AccessSize::word, 1U, write_context));
    ASSERT_EQ(loop.runDueEvents(20'000U).events_executed, 0U);
    ASSERT_EQ(timer.read(0x24, fil::mem::AccessSize::word, read_context).value(), 20U);

    ASSERT_TRUE(timer.write(0x28, fil::mem::AccessSize::word, 1U, write_context));
    ASSERT_EQ(timer.read(0x24, fil::mem::AccessSize::word, read_context).value(), 20U)
        << "PSC update captures CNT without resetting it";
    ASSERT_EQ(loop.runDueEvents(30'000U).events_executed, 0U);
    ASSERT_EQ(timer.read(0x24, fil::mem::AccessSize::word, read_context).value(), 25U);

    timer.setInputClockHz(2'000'000U);
    ASSERT_EQ(timer.read(0x24, fil::mem::AccessSize::word, read_context).value(), 25U)
        << "clock change preserves current count";
    ASSERT_EQ(loop.runDueEvents(40'000U).events_executed, 0U);
    ASSERT_TRUE(timer.write(0x24, fil::mem::AccessSize::word, 10U, write_context));
    ASSERT_EQ(timer.read(0x24, fil::mem::AccessSize::word, read_context).value(), 10U)
        << "software CNT write is visible immediately";

    ASSERT_TRUE(timer.write(0x00, fil::mem::AccessSize::word, 0U, write_context));
    ASSERT_EQ(loop.runDueEvents(60'000U).events_executed, 0U);
    EXPECT_EQ(timer.read(0x24, fil::mem::AccessSize::word, read_context).value(), 10U)
        << "stopped counter holds CNT";
    ASSERT_TRUE(timer.write(0x00, fil::mem::AccessSize::word, 1U, write_context));
    ASSERT_EQ(loop.runDueEvents(149'999U).events_executed, 0U);
    EXPECT_EQ(loop.runDueEvents(150'000U).events_executed, 1U)
        << "restart schedules the remaining ARR ticks at the new PSC/clock";
}

TEST(PeripheralTest, IwdgPrescalerCodesProduceNominalTimeoutDeadlines) {
    for (std::uint32_t code = 0; code <= 7U; ++code) {
        fil::sim::EventLoop loop;
        fil::stm32g4::IwdgPeripheral watchdog(true, &loop);
        std::uint32_t resets = 0;
        watchdog.setResetCallback([&]() { ++resets; });
        ASSERT_TRUE(watchdog.write(0x00, fil::mem::AccessSize::word, 0x5555U, write_context));
        ASSERT_TRUE(watchdog.write(0x04, fil::mem::AccessSize::word, code, write_context));
        ASSERT_TRUE(watchdog.write(0x08, fil::mem::AccessSize::word, 0U, write_context));
        ASSERT_TRUE(watchdog.write(0x00, fil::mem::AccessSize::word, 0xccccU, write_context));
        EXPECT_EQ(watchdog.read(0x0c, fil::mem::AccessSize::word, read_context).value(), 0U)
            << "the modeled IWDG status register reports no synchronization flags";
        const std::uint64_t deadline_ns = 125'000U << std::min(code, 6U);
        EXPECT_EQ(loop.runDueEvents(deadline_ns - 1U).events_executed, 0U) << "PR=" << code;
        EXPECT_EQ(loop.runDueEvents(deadline_ns).events_executed, 1U) << "PR=" << code;
        EXPECT_EQ(resets, 1U) << "PR=" << code;
    }
}

TEST(PeripheralTest, WwdgCounterAndCfrPrescalerSetTimeoutDeadline) {
    fil::sim::EventLoop loop;
    fil::stm32g4::WwdgPeripheral watchdog(true, 16'000'000U, &loop);
    std::uint32_t resets = 0;
    watchdog.setResetCallback([&]() { ++resets; });
    // T[6:0]=0x41 leaves two modeled decrement periods before underflow.
    ASSERT_TRUE(watchdog.write(0x04, fil::mem::AccessSize::word, 1U << 7U, write_context));
    ASSERT_TRUE(watchdog.write(0x00, fil::mem::AccessSize::word, (1U << 7U) | 0x41U, write_context));
    ASSERT_EQ(loop.runDueEvents(1'023'999U).events_executed, 0U);
    EXPECT_EQ(loop.runDueEvents(1'024'000U).events_executed, 1U);
    EXPECT_EQ(resets, 1U);

    fil::sim::EventLoop refresh_loop;
    fil::stm32g4::WwdgPeripheral refreshed(true, 16'000'000U, &refresh_loop);
    std::uint32_t refresh_resets = 0;
    refreshed.setResetCallback([&]() { ++refresh_resets; });
    ASSERT_TRUE(refreshed.write(0x00, fil::mem::AccessSize::word, (1U << 7U) | 0x41U, write_context));
    ASSERT_EQ(refresh_loop.runDueEvents(100'000U).events_executed, 0U);
    ASSERT_TRUE(refreshed.write(0x00, fil::mem::AccessSize::word, (1U << 7U) | 0x41U, write_context));
    EXPECT_EQ(refresh_loop.runDueEvents(511'999U).events_executed, 0U)
        << "a WDGA CR write refreshes the model timeout deadline";
    EXPECT_EQ(refresh_loop.runDueEvents(612'000U).events_executed, 1U);
    EXPECT_EQ(refresh_resets, 1U);

    fil::sim::EventLoop stop_loop;
    fil::stm32g4::WwdgPeripheral stopped(true, 16'000'000U, &stop_loop);
    std::uint32_t stopped_resets = 0;
    stopped.setResetCallback([&]() { ++stopped_resets; });
    ASSERT_TRUE(stopped.write(0x00, fil::mem::AccessSize::word, (1U << 7U) | 0x41U, write_context));
    ASSERT_TRUE(stopped.write(0x00, fil::mem::AccessSize::word, 0x41U, write_context));
    EXPECT_EQ(stop_loop.runDueEvents(1'000'000U).events_executed, 0U);
    EXPECT_EQ(stopped_resets, 0U) << "clearing WDGA cancels the scheduled timeout";

    fil::sim::EventLoop disabled_loop;
    fil::stm32g4::WwdgPeripheral no_reset(false, 16'000'000U, &disabled_loop);
    no_reset.setResetCallback([&]() { ++resets; });
    ASSERT_TRUE(no_reset.write(0x00, fil::mem::AccessSize::word, (1U << 7U) | 0x40U, write_context));
    EXPECT_EQ(disabled_loop.runDueEvents(256'000U).events_executed, 1U);
    EXPECT_EQ(resets, 1U) << "timeout requests no reset when reset option is disabled";
}


TEST(PeripheralTest, UsartAcknowledgementsIdleRefillAndInterruptGates) {
    fil::sim::EventLoop loop;
    fil::stm32g4::UsartPeripheral uart("USART1", &loop);
    ASSERT_TRUE(uart.write(0, fil::mem::AccessSize::word, 1U | (1U << 2U) | (1U << 3U), write_context)); // UE, RE, TE
    auto status = uart.read(0x1c, fil::mem::AccessSize::word, read_context);
    ASSERT_TRUE(status);
    EXPECT_NE(status.value() & ((1U << 21U) | (1U << 22U)), 0U);
    EXPECT_EQ(status.value() & ((1U << 21U) | (1U << 22U)), (1U << 21U) | (1U << 22U));

    uart.setIdleGap(10U);
    uart.injectRx(0x31U);
    ASSERT_EQ(loop.runDueEvents(10U).events_executed, 1U);
    status = uart.read(0x1c, fil::mem::AccessSize::word, read_context);
    ASSERT_TRUE(status);
    EXPECT_NE(status.value() & (1U << 4U), 0U); // IDLE
    ASSERT_TRUE(uart.write(0x20, fil::mem::AccessSize::word, 1U << 4U, write_context)); // IDLECF
    EXPECT_EQ(uart.peekRegister(0x1c) & (1U << 4U), 0U);
    ASSERT_TRUE(uart.write(0x18, fil::mem::AccessSize::word, 1U << 3U, write_context)); // RXFRQ
    EXPECT_FALSE(uart.hasRxData());
    EXPECT_EQ(uart.peekRegister(0x1c) & (1U << 5U), 0U);

    unsigned int provider_calls = 0;
    uart.setRxProvider([&](std::uint64_t) -> std::optional<std::uint8_t> {
        return provider_calls++ == 0U ? std::optional<std::uint8_t>{0x5aU} : std::nullopt;
    });
    status = uart.read(0x1c, fil::mem::AccessSize::word, read_context); // status observation refills an empty RDR
    ASSERT_TRUE(status);
    EXPECT_NE(status.value() & (1U << 5U), 0U);
    auto received = uart.read(0x24, fil::mem::AccessSize::byte, read_context);
    ASSERT_TRUE(received);
    EXPECT_EQ(received.value(), 0x5aU);

}

TEST(PeripheralTest, UsartInterruptEnablesIndependentlyAssertAndDeassert) {
    const std::array<std::pair<std::uint32_t, std::uint32_t>, 4> gates{{
        {1U << 4U, 1U << 4U}, {1U << 5U, 1U << 5U},
        {1U << 6U, 1U << 6U}, {1U << 7U, 1U << 7U},
    }};
    for (const auto [enable, flag] : gates) {
        fil::sim::EventLoop loop;
        fil::stm32g4::UsartPeripheral uart("USART1", &loop);
        std::vector<bool> levels;
        uart.setInterruptLevelCallback([&](std::uint32_t line, bool asserted) {
            if (line == 0U) levels.push_back(asserted);
        });
        levels.clear();
        ASSERT_TRUE(uart.write(0U, fil::mem::AccessSize::word, 1U, write_context)); // UE, no source enable
        ASSERT_TRUE(uart.write(0x20U, fil::mem::AccessSize::word, 0xffffffffU, write_context)); // clear TC/IDLE/etc.
        ASSERT_TRUE(uart.write(0U, fil::mem::AccessSize::word, 1U | enable, write_context));
        if (flag == (1U << 4U)) {
            uart.setIdleGap(10U);
            uart.injectRx(0x11U);
            ASSERT_EQ(loop.runDueEvents(10U).events_executed, 1U);
        } else if (flag == (1U << 5U)) {
            uart.injectRx(0x11U);
        } else if (flag == (1U << 6U)) {
            ASSERT_TRUE(uart.write(0x28U, fil::mem::AccessSize::byte, 0x55U, write_context));
        }
        ASSERT_EQ(levels, (std::vector<bool>{true})) << "CR1 enable " << enable << " asserts only its pending source";
        ASSERT_TRUE(uart.write(0U, fil::mem::AccessSize::word, 1U, write_context)); // disable only interrupt gate; status remains
        EXPECT_EQ(levels, (std::vector<bool>{true, false}));
        EXPECT_NE(uart.peekRegister(0x1cU) & flag, 0U) << "status survives interrupt masking";
    }
}

TEST(PeripheralTest, SpiConsumesFramesAndSignalsReceiveDma) {
    fil::stm32g4::SpiPeripheral spi("SPI1");
    std::vector<bool> requests;
    spi.setDmaRequestCallback([&](bool tx) { requests.push_back(tx); });
    std::vector<std::uint8_t> seen;
    spi.setTransferCallback([&](std::span<const std::uint8_t> tx, std::uint64_t) {
        seen.assign(tx.begin(), tx.end());
        return std::vector<std::uint8_t>{0xa5U, 0x5aU};
    });
    ASSERT_TRUE(spi.write(0x04, fil::mem::AccessSize::word, 1U, write_context)); // RXDMAEN
    ASSERT_TRUE(spi.write(0x0c, fil::mem::AccessSize::halfword, 0x1234U, write_context));
    EXPECT_EQ(seen, (std::vector<std::uint8_t>{0x34U, 0x12U}));
    ASSERT_TRUE(spi.read(0x08, fil::mem::AccessSize::word, read_context));
    EXPECT_NE(spi.peekRegister(0x08) & 1U, 0U);
    auto rx = spi.read(0x0c, fil::mem::AccessSize::halfword, read_context);
    ASSERT_TRUE(rx);
    EXPECT_EQ(rx.value(), 0x5aa5U);
    EXPECT_EQ(spi.peekRegister(0x08) & 1U, 0U);
    EXPECT_EQ(requests, (std::vector<bool>{false}));
    EXPECT_EQ(spi.transferLog().size(), 1U);
}

TEST(PeripheralTest, DmaWidthsIncrementsCircularAndFailureFlags) {
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 0x100U, "dma-contract"));
    ASSERT_TRUE(bus.write16(0x20000000U, 0x1234U));
    ASSERT_TRUE(bus.write16(0x20000002U, 0x5678U));
    fil::stm32g4::DmaPeripheral dma("DMA1", 2U, &bus);
    // Channel 1: memory-to-peripheral, halfword widths, both address increments.
    ASSERT_TRUE(dma.write(0x0c, fil::mem::AccessSize::word, 2U, write_context));
    ASSERT_TRUE(dma.write(0x10, fil::mem::AccessSize::word, 0x20000020U, write_context));
    ASSERT_TRUE(dma.write(0x14, fil::mem::AccessSize::word, 0x20000000U, write_context));
    const std::uint32_t ccr = 1U | (1U << 4U) | (1U << 5U) | (1U << 6U) | (1U << 7U) |
                              (1U << 8U) | (1U << 10U);
    ASSERT_TRUE(dma.write(0x08, fil::mem::AccessSize::word, ccr, write_context));
    ASSERT_TRUE(dma.request(1U));
    ASSERT_TRUE(dma.request(1U));
    EXPECT_EQ(bus.read16(0x20000020U).value(), 0x1234U);
    EXPECT_EQ(bus.read16(0x20000022U).value(), 0x5678U);
    // CIRC was selected in CCR: terminal count reloads and EN remains asserted.
    EXPECT_EQ(dma.peekRegister(0x0c), 2U);
    EXPECT_NE(dma.peekRegister(0x08) & 1U, 0U);
    EXPECT_NE(dma.peekRegister(0) & (1U << 1U), 0U);

    fil::stm32g4::DmaPeripheral circular("DMA1-CIRC", 1U, &bus);
    ASSERT_TRUE(circular.write(0x0c, fil::mem::AccessSize::word, 1U, write_context));
    ASSERT_TRUE(circular.write(0x10, fil::mem::AccessSize::word, 0x20000030U, write_context));
    ASSERT_TRUE(circular.write(0x14, fil::mem::AccessSize::word, 0x20000000U, write_context));
    ASSERT_TRUE(circular.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 5U), write_context)); // CIRC, fil::mem::AccessSize::byte widths
    ASSERT_TRUE(circular.request(1U));
    EXPECT_EQ(circular.peekRegister(0x0c), 1U); // CNDTR reloads at TC
    EXPECT_NE(circular.peekRegister(0x08) & 1U, 0U); // EN remains set
    EXPECT_TRUE(circular.request(1U)); // next circular cycle

    // No attached memory is a modeled transfer error: TEIF, disable, TEIE IRQ.
    fil::stm32g4::DmaPeripheral failing("DMA2", 1U);
    std::vector<bool> irq;
    failing.setInterruptLevelCallback([&](std::uint32_t, bool value) { irq.push_back(value); });
    ASSERT_TRUE(failing.write(0x0c, fil::mem::AccessSize::word, 1U, write_context));
    ASSERT_TRUE(failing.write(0x10, fil::mem::AccessSize::word, 0x50000000U, write_context));
    ASSERT_TRUE(failing.write(0x14, fil::mem::AccessSize::word, 0x20000000U, write_context));
    ASSERT_TRUE(failing.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 3U), write_context));
    ASSERT_TRUE(failing.request(1U));
    EXPECT_NE(failing.peekRegister(0) & (1U << 3U), 0U);
    EXPECT_EQ(failing.peekRegister(0x08) & 1U, 0U);
    ASSERT_TRUE(failing.write(0x04, fil::mem::AccessSize::word, 1U << 3U, write_context)); // CTEIF1
    EXPECT_EQ(failing.peekRegister(0), 0U);
    EXPECT_FALSE(irq.empty());
    EXPECT_FALSE(irq.back());
}

TEST(PeripheralTest, DmaByteHalfwordWordAndBothDirectionsHonorIncrementBits) {
    struct WidthCase { std::uint32_t selector; std::uint32_t width; };
    constexpr std::array widths{WidthCase{0U, 1U}, WidthCase{1U, 2U}, WidthCase{2U, 4U}};
    constexpr std::array increment_modes{
        std::pair{false, false}, std::pair{true, false},
        std::pair{false, true}, std::pair{true, true},
    };
    for (const WidthCase width : widths) {
        for (const bool memory_to_peripheral : {false, true}) {
            for (const auto [pinc, minc] : increment_modes) {
                fil::mem::MemoryBus bus;
                ASSERT_TRUE(bus.mapRam(0x20000000U, 0x100U, "dma-width-matrix"));
                fil::stm32g4::DmaPeripheral dma("DMA", 1U, &bus);
                const std::uint32_t peripheral = 0x20000040U;
                const std::uint32_t memory = 0x20000000U;
                const std::array<std::uint32_t, 2> values{0x12345678U, 0x9abcdef0U};
                const bool input_increment = memory_to_peripheral ? minc : pinc;
                const auto writeItem = [&](const std::uint32_t address, const std::uint32_t value) {
                    if (width.width == 1U) return bus.write8(address, static_cast<std::uint8_t>(value));
                    if (width.width == 2U) return bus.write16(address, static_cast<std::uint16_t>(value));
                    return bus.write32(address, value);
                };
                for (std::uint32_t i = 0; i < 2U; ++i) {
                    const std::uint32_t input = memory_to_peripheral ? memory : peripheral;
                    ASSERT_TRUE(writeItem(input + (input_increment ? i * width.width : 0U), values[i]));
                }
                ASSERT_TRUE(dma.write(0x0cU, fil::mem::AccessSize::word, 2U, write_context));
                ASSERT_TRUE(dma.write(0x10U, fil::mem::AccessSize::word, peripheral, write_context));
                ASSERT_TRUE(dma.write(0x14U, fil::mem::AccessSize::word, memory, write_context));
                std::uint32_t ccr = 1U | (width.selector << 8U) | (width.selector << 10U);
                if (memory_to_peripheral) ccr |= 1U << 4U;
                if (pinc) ccr |= 1U << 6U;
                if (minc) ccr |= 1U << 7U;
                ASSERT_TRUE(dma.write(0x08U, fil::mem::AccessSize::word, ccr, write_context));
                ASSERT_TRUE(dma.request(1U));
                ASSERT_TRUE(dma.request(1U));
                const std::uint32_t output = memory_to_peripheral ? peripheral : memory;
                const bool output_increment = memory_to_peripheral ? pinc : minc;
                const std::uint32_t expected_items = output_increment ? 2U : 1U;
                for (std::uint32_t i = 0; i < expected_items; ++i) {
                    const std::uint32_t index = output_increment ? i : 0U;
                    auto actual = width.width == 1U
                        ? fil::mem::MemoryResult<std::uint64_t>{bus.read8(output + index * width.width).value()}
                        : width.width == 2U
                            ? fil::mem::MemoryResult<std::uint64_t>{bus.read16(output + index * width.width).value()}
                            : fil::mem::MemoryResult<std::uint64_t>{bus.read32(output + index * width.width).value()};
                    ASSERT_TRUE(actual);
                    const std::uint32_t source_index = output_increment && input_increment ? i : 1U;
                    const std::uint64_t width_mask = width.width == 1U ? 0xffU
                        : width.width == 2U ? 0xffffU : 0xffffffffU;
                    EXPECT_EQ(actual.value(), values[source_index] & width_mask)
                        << "width=" << width.width << " DIR=" << memory_to_peripheral
                        << " PINC=" << pinc << " MINC=" << minc;
                }
            }
        }
    }
}

TEST(PeripheralTest, DmamuxRequestSelectorAndReadOnlyClearStatus) {
    fil::stm32g4::DmamuxPeripheral mux("DMAMUX", 2U);
    ASSERT_TRUE(mux.write(0x00, fil::mem::AccessSize::word, 0x55U, write_context));
    EXPECT_EQ(mux.requestForChannel(0U), 0x55U);
    const auto generation = mux.routingGeneration();
    ASSERT_TRUE(mux.write(0x00, fil::mem::AccessSize::word, 0x55U, write_context));
    EXPECT_EQ(mux.routingGeneration(), generation);
    // CSR is read-only, with no modeled source that can raise SOFx.
    ASSERT_TRUE(mux.write(0x80, fil::mem::AccessSize::word, 1U, write_context));
    EXPECT_EQ(mux.peekRegister(0x80), 0U);
}

} // namespace

/** @brief Runs STM32G4 peripheral foundation unit tests. */
