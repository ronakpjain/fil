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
    gpio.setOutputCallback([&](const unsigned int pin, const bool high, const fil::sim::SimTimeNs time) {
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
    int usart_interrupts = 0;
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

TEST(PeripheralTest, DrivesTimerAndAdcFromSimulatedTime) {
    fil::sim::EventLoop loop;
    fil::stm32g4::TimerPeripheral timer("TIM1", 1000000, &loop);
    int timer_interrupts = 0;
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
    adc.setChannelProvider([&](const unsigned int, const fil::sim::SimTimeNs now) {
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
    int interrupts = 0;
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
    std::vector<std::pair<unsigned int, bool>> levels;

    fil::stm32g4::TimerPeripheral timer("TIM1");
    timer.setInterruptLevelCallback([&](const unsigned int line, const bool asserted) {
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
    spi.setInterruptLevelCallback([&](const unsigned int line, const bool asserted) {
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
    adc.setInterruptLevelCallback([&](const unsigned int line, const bool asserted) {
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
    dma.setInterruptLevelCallback([&](const unsigned int line, const bool asserted) {
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
    adc.setInterruptLevelCallback([&](const unsigned int line, const bool asserted) {
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

TEST(PeripheralTest, DmaGlobalFlagClearDeassertsOnlySelectedChannel) {
    fil::stm32g4::DmaPeripheral dma("DMA1", 8U);
    std::uint32_t levels = 0U;
    dma.setInterruptLevelCallback([&](const unsigned int line, const bool asserted) {
        if (asserted) levels |= 1U << line;
        else levels &= ~(1U << line);
    });
    constexpr auto word = fil::mem::AccessSize::word;
    for (const unsigned int channel : {1U, 8U}) {
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
    usart.setInterruptLevelCallback([&](const unsigned int line, const bool asserted) {
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
    int dma_interrupts = 0;
    dma.setInterruptCallback([&](const unsigned int channel) {
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
    int resets = 0;
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

} // namespace

/** @brief Runs STM32G4 peripheral foundation unit tests. */
