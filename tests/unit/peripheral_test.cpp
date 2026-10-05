#include "fil/mem/memory_bus.hpp"
#include "fil/stm32g4/peripheral.hpp"

#include <gtest/gtest.h>

#include <cstdint>
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

} // namespace

/** @brief Runs STM32G4 peripheral foundation unit tests. */
