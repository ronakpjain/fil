#include "fil/mem/memory_bus.hpp"
#include "fil/stm32g4/peripheral.hpp"
#include "../test_support.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace {

constexpr fil::mem::AccessContext read_context{fil::mem::AccessType::data_read, 0};
constexpr fil::mem::AccessContext write_context{fil::mem::AccessType::data_write, 0};

void storesRegistersAndUnknownMmio() {
    fil::stm32g4::PwrPeripheral pwr;
    fil::test::check(pwr.write(0, fil::mem::AccessSize::word, 0x11223344U, write_context).hasValue(), "writes a register word");
    fil::test::check(pwr.write(1, fil::mem::AccessSize::byte, 0xaaU, write_context).hasValue(), "merges a partial register write");
    const auto merged = pwr.read(0, fil::mem::AccessSize::word, read_context);
    fil::test::check(merged && merged.value() == 0x1122aa44U, "register storage merges little-endian byte writes");
    pwr.reset();
    const auto reset = pwr.read(0, fil::mem::AccessSize::word, read_context);
    fil::test::check(reset && reset.value() == 0, "register reset restores deterministic values");

    fil::stm32g4::UnknownMmioDevice unknown("fallback", 0x40000000U);
    fil::test::check(unknown.write(3, fil::mem::AccessSize::halfword, 0xabcdU, write_context).hasValue(), "lenient unknown MMIO accepts writes");
    const auto unknown_read = unknown.read(3, fil::mem::AccessSize::halfword, read_context);
    fil::test::check(unknown_read && unknown_read.value() == 0xabcdU, "unknown MMIO preserves sparse written bytes");
    unknown.setStrict(true);
    const auto strict = unknown.read(0, fil::mem::AccessSize::word, read_context);
    fil::test::check(!strict && strict.fault().address == 0x40000000U, "strict unknown MMIO returns an absolute device fault");
}

void journalsTransactionalRegisters() {
    fil::stm32g4::PwrPeripheral pwr;
    fil::test::check(
        pwr.write(0, fil::mem::AccessSize::word, 0x11U, write_context).hasValue(),
        "seeds transactional register state"
    );
    pwr.beginTransaction();
    static_cast<void>(pwr.write(0, fil::mem::AccessSize::word, 0x22U, write_context));
    static_cast<void>(pwr.write(0, fil::mem::AccessSize::word, 0x33U, write_context));
    pwr.rollbackTransaction();
    const auto rolled_back = pwr.read(0, fil::mem::AccessSize::word, read_context);
    fil::test::check(
        rolled_back && rolled_back.value() == 0x11U,
        "register COW restores the first pre-transaction value"
    );

    pwr.beginTransaction();
    static_cast<void>(pwr.write(0, fil::mem::AccessSize::word, 0x44U, write_context));
    pwr.commitTransaction();
    const auto committed = pwr.read(0, fil::mem::AccessSize::word, read_context);
    fil::test::check(
        committed && committed.value() == 0x44U,
        "register COW retains committed mutations"
    );
}

void modelsClockFlashAndGpioStartup() {
    fil::stm32g4::RccPeripheral rcc;
    fil::test::check(rcc.write(0, fil::mem::AccessSize::word, (1U << 16U) | (1U << 24U), write_context).hasValue(), "writes RCC CR");
    const auto clock_ready = rcc.read(0, fil::mem::AccessSize::word, read_context);
    fil::test::check(
        clock_ready && (clock_ready.value() & ((1U << 17U) | (1U << 25U))) == ((1U << 17U) | (1U << 25U)),
        "RCC reflects HSE and PLL ready bits"
    );
    fil::test::check(rcc.write(8, fil::mem::AccessSize::word, 3U, write_context).hasValue(), "selects PLL system clock");
    const auto selected = rcc.read(8, fil::mem::AccessSize::word, read_context);
    fil::test::check(selected && (selected.value() & 0x0fU) == 0x0fU, "RCC reflects SW in SWS");

    fil::stm32g4::FlashPeripheral flash;
    const auto locked = flash.read(0x14, fil::mem::AccessSize::word, read_context);
    fil::test::check(locked && (locked.value() & (1U << 31U)) != 0U, "FLASH control starts locked");
    fil::test::check(flash.write(0x08, fil::mem::AccessSize::word, 0x45670123U, write_context).hasValue(), "accepts first FLASH key");
    fil::test::check(flash.write(0x08, fil::mem::AccessSize::word, 0xcdef89abU, write_context).hasValue(), "accepts second FLASH key");
    const auto unlocked = flash.read(0x14, fil::mem::AccessSize::word, read_context);
    fil::test::check(unlocked && (unlocked.value() & (1U << 31U)) == 0U, "FLASH key sequence clears LOCK");

    fil::sim::EventLoop loop;
    fil::stm32g4::GpioPeripheral gpio("GPIOC", &loop);
    std::vector<fil::stm32g4::GpioTransition> callbacks;
    gpio.setOutputCallback([&](const unsigned int pin, const bool high, const fil::sim::SimTimeNs time) {
        callbacks.push_back({time, pin, high});
    });
    fil::test::check(gpio.write(0, fil::mem::AccessSize::word, 1U << (13U * 2U), write_context).hasValue(), "configures GPIO output mode");
    fil::test::check(gpio.write(0x18, fil::mem::AccessSize::word, 1U << 13U, write_context).hasValue(), "GPIO BSRR sets an output");
    const auto output = gpio.read(0x14, fil::mem::AccessSize::word, read_context);
    const auto mirrored = gpio.read(0x10, fil::mem::AccessSize::word, read_context);
    fil::test::check(output && (output.value() & (1U << 13U)) != 0U, "GPIO stores ODR state");
    fil::test::check(mirrored && (mirrored.value() & (1U << 13U)) != 0U, "GPIO IDR mirrors an undriven output");
    gpio.setInput(13, false);
    const auto driven = gpio.read(0x10, fil::mem::AccessSize::word, read_context);
    fil::test::check(driven && (driven.value() & (1U << 13U)) == 0U, "external GPIO input overrides output mirroring");
    fil::test::check(callbacks.size() == 1 && callbacks.front().pin == 13U, "GPIO reports output transitions once");
}

void modelsUsartAndSpiDataPaths() {
    fil::sim::EventLoop loop;
    fil::sim::TraceRecorder trace;
    fil::stm32g4::UsartPeripheral usart("USART1", &loop, &trace);
    int usart_interrupts = 0;
    usart.setInterruptCallback([&]() { ++usart_interrupts; });
    fil::test::check(usart.write(0, fil::mem::AccessSize::word, 1U | (1U << 2U) | (1U << 3U) | (1U << 5U), write_context).hasValue(), "enables USART RX interrupt");
    usart.injectRx(0x42);
    const auto status = usart.read(0x1c, fil::mem::AccessSize::word, read_context);
    const auto received = usart.read(0x24, fil::mem::AccessSize::byte, read_context);
    fil::test::check(status && (status.value() & (1U << 5U)) != 0U, "USART sets RXNE for queued input");
    fil::test::check(received && received.value() == 0x42U, "USART RDR pops scripted input");
    fil::test::check(usart.write(0x28, fil::mem::AccessSize::byte, 'A', write_context).hasValue(), "USART accepts a transmit byte");
    fil::test::check(usart.txLog().size() == 1 && usart.txLog().front().value == 'A', "USART records transmitted bytes");
    fil::test::check(usart_interrupts > 0, "USART signals enabled receive interrupts");

    fil::stm32g4::SpiPeripheral spi("SPI1", &loop, &trace);
    spi.setEcho(true);
    fil::test::check(spi.write(0x0c, fil::mem::AccessSize::halfword, 0x1234U, write_context).hasValue(), "SPI accepts a frame");
    const auto response = spi.read(0x0c, fil::mem::AccessSize::halfword, read_context);
    fil::test::check(response && response.value() == 0x1234U, "SPI echo device returns the transmitted frame");
    fil::test::check(spi.transferLog().size() == 1, "SPI records complete transactions");
    fil::test::check(trace.records().size() >= 3, "USART and SPI emit device-facing trace records");
}

void drivesTimerAndAdcFromSimulatedTime() {
    fil::sim::EventLoop loop;
    fil::stm32g4::TimerPeripheral timer("TIM1", 1000000, &loop);
    int timer_interrupts = 0;
    timer.setInterruptCallback([&]() { ++timer_interrupts; });
    fil::test::check(timer.write(0x2c, fil::mem::AccessSize::word, 9, write_context).hasValue(), "sets timer auto-reload");
    fil::test::check(timer.write(0x0c, fil::mem::AccessSize::word, 1, write_context).hasValue(), "enables timer update interrupt");
    fil::test::check(timer.write(0x00, fil::mem::AccessSize::word, 1, write_context).hasValue(), "starts timer");
    fil::test::check(loop.runDueEvents(9999).events_executed == 0, "timer does not update before its period");
    fil::test::check(loop.runDueEvents(10000).events_executed == 1, "timer schedules an update at its exact period");
    const auto timer_status = timer.read(0x10, fil::mem::AccessSize::word, read_context);
    fil::test::check(timer_status && (timer_status.value() & 1U) != 0U && timer_interrupts == 1, "timer sets UIF and signals UIE");

    fil::stm32g4::AdcPeripheral adc("ADC1", &loop);
    adc.setConversionDelay(5'000);
    adc.setChannelValue(2, 2048);
    fil::test::check(adc.write(0x30, fil::mem::AccessSize::word, 2U << 6U, write_context).hasValue(), "selects ADC channel");
    fil::test::check(adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context).hasValue(), "enables and starts ADC");
    fil::test::check(loop.runDueEvents(14999).events_executed == 0, "ADC conversion waits for configured delay");
    fil::test::check(loop.runDueEvents(15000).events_executed == 1, "ADC completes conversion deterministically");
    const auto adc_status = adc.read(0, fil::mem::AccessSize::word, read_context);
    const auto adc_data = adc.read(0x40, fil::mem::AccessSize::word, read_context);
    fil::test::check(adc_status && (adc_status.value() & ((1U << 2U) | (1U << 3U))) != 0U, "ADC sets EOC/EOS");
    fil::test::check(adc_data && adc_data.value() == 2048, "ADC DR returns configured channel value");
}

void sequencesAdcChannelsWithRegisterDerivedTiming() {
    fil::sim::EventLoop loop;
    fil::stm32g4::AdcPeripheral adc("ADC1", &loop);
    adc.setInputClockHz(16'000'000U);
    adc.setChannelValue(2, 2002U);
    adc.setChannelValue(3, 3003U);
    const std::uint32_t sequence = 1U | (2U << 6U) | (3U << 12U);
    fil::test::check(
        adc.write(0x14, fil::mem::AccessSize::word, (7U << 6U) | (7U << 9U),
                  write_context).hasValue()
            && adc.write(0x30, fil::mem::AccessSize::word, sequence, write_context).hasValue(),
        "configures a two-rank ADC sequence with 640.5-cycle sampling"
    );
    fil::test::check(
        adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U),
                  write_context).hasValue(),
        "starts a register-timed ADC sequence"
    );
    fil::test::check(loop.runDueEvents(40'812U).events_executed == 0U,
                     "first ADC rank waits for sample and conversion cycles");
    fil::test::check(loop.runDueEvents(40'813U).events_executed == 1U
                         && adc.samples().size() == 1U
                         && adc.samples()[0].channel == 2U,
                     "first ADC rank completes at its clock-derived deadline");
    fil::test::check(loop.runDueEvents(81'626U).events_executed == 1U
                         && adc.samples().size() == 2U
                         && adc.samples()[1].channel == 3U,
                     "ADC advances through the configured channel sequence");
    const auto data = adc.read(0x40, fil::mem::AccessSize::word, read_context);
    fil::test::check(data && data.value() == 3003U,
                     "ADC data register contains the final sequence rank");
}

void lazilySynchronizesUnobservedContinuousAdc() {
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

    fil::test::check(adc.write(0x0c, fil::mem::AccessSize::word, 1U << 13U,
                               write_context).hasValue(),
                     "enables ADC continuous mode for lazy conversion test");
    fil::test::check(adc.write(0x08, fil::mem::AccessSize::word, 1U | (1U << 2U),
                               write_context).hasValue(),
                     "starts unobserved continuous ADC conversion");
    fil::test::check(loop.pending() == 0,
                     "unobserved continuous ADC does not enqueue conversion callbacks");
    fil::test::check(loop.runDueEvents(25'000).events_executed == 0,
                     "virtual time crosses lazy ADC conversions without callbacks");
    fil::test::check(provider_times.empty(),
                     "lazy ADC defers provider sampling until firmware observes registers");

    const auto status = adc.read(0x00, fil::mem::AccessSize::word, read_context);
    const auto data = adc.read(0x40, fil::mem::AccessSize::word, read_context);
    fil::test::check(status && (status.value() & ((1U << 2U) | (1U << 3U))) != 0U,
                     "lazy ADC synchronizes EOC and EOS on MMIO access");
    fil::test::check(data && data.value() == 5U && provider_times == std::vector<fil::sim::SimTimeNs>{25'000},
                     "lazy ADC samples only the most recent due conversion timestamp");
    fil::test::check(adc.samples().empty() && trace.records().empty(),
                     "lazy ADC does not synthesize disabled history or trace diagnostics");

    adc.setSampleHistoryEnabled(true);
    fil::test::check(loop.pending() == 1,
                     "enabling ADC history restores the exact next conversion event");
    fil::test::check(loop.runDueEvents(29'999).events_executed == 0,
                     "observable ADC retains its original next deadline");
    fil::test::check(loop.runDueEvents(30'000).events_executed == 1,
                     "observable continuous ADC fires at the exact next deadline");
    fil::test::check(adc.samples().size() == 1 && adc.samples().front().time_ns == 30'000,
                     "newly enabled ADC history begins with future conversions only");
}

void overridesConfiguredAdcProvider() {
    fil::sim::EventLoop loop;
    fil::sim::TraceRecorder trace;
    fil::stm32g4::AdcPeripheral adc("ADC1", &loop, &trace);
    adc.setChannelProvider([](const unsigned int, const fil::sim::SimTimeNs) {
        return std::uint16_t{1000U};
    });
    adc.overrideChannelValue(0, 2345U);
    adc.setConversionDelay(1U);

    fil::test::check(adc.write(0x08, fil::mem::AccessSize::word,
                               1U | (1U << 2U), write_context).hasValue(),
                     "starts ADC conversion with a live override");
    fil::test::check(loop.runDueEvents(1U).events_executed == 1,
                     "completes ADC conversion with a live override");
    const auto result = adc.read(0x40, fil::mem::AccessSize::word, read_context);
    fil::test::check(result && result.value() == 2345U,
                     "live ADC override takes precedence over configured provider");
}

void preservesObservableAndSingleShotAdcEvents() {
    fil::sim::EventLoop interrupt_loop;
    fil::sim::TraceRecorder interrupt_trace;
    interrupt_trace.setEnabled(false);
    fil::stm32g4::AdcPeripheral interrupt_adc("ADC1", &interrupt_loop, &interrupt_trace);
    interrupt_adc.setConversionDelay(5'000);
    interrupt_adc.setSampleHistoryEnabled(false);
    int interrupts = 0;
    interrupt_adc.setInterruptCallback([&]() { ++interrupts; });
    fil::test::check(interrupt_adc.write(0x04, fil::mem::AccessSize::word, 1U << 2U,
                                         write_context).hasValue(),
                     "enables ADC EOC interrupt");
    fil::test::check(interrupt_adc.write(0x0c, fil::mem::AccessSize::word, 1U << 13U,
                                         write_context).hasValue(),
                     "enables interrupt-observed continuous ADC mode");
    fil::test::check(interrupt_adc.write(0x08, fil::mem::AccessSize::word,
                                         1U | (1U << 2U), write_context).hasValue(),
                     "starts interrupt-observed continuous ADC");
    fil::test::check(interrupt_loop.pending() == 1
                         && interrupt_loop.runDueEvents(5'000).events_executed == 1
                         && interrupts == 1,
                     "enabled EOC interrupt retains per-conversion scheduling");

    fil::sim::EventLoop single_loop;
    fil::sim::TraceRecorder single_trace;
    single_trace.setEnabled(false);
    fil::stm32g4::AdcPeripheral single_adc("ADC2", &single_loop, &single_trace);
    single_adc.setConversionDelay(5'000);
    single_adc.setSampleHistoryEnabled(false);
    single_adc.setChannelValue(0, 1234);
    fil::test::check(single_adc.write(0x08, fil::mem::AccessSize::word,
                                      1U | (1U << 2U), write_context).hasValue(),
                     "starts unobserved single-shot ADC");
    fil::test::check(single_loop.pending() == 1
                         && single_loop.runDueEvents(5'000).events_executed == 1,
                     "single-shot ADC always retains its exact completion event");
    const auto single_data = single_adc.read(0x40, fil::mem::AccessSize::word, read_context);
    fil::test::check(single_data && single_data.value() == 1234U,
                     "unobserved single-shot ADC still materializes DR");
}

void completesDmaAndWatchdogSideEffects() {
    fil::mem::MemoryBus memory;
    fil::test::check(memory.mapRam(0x20000000U, 64, "dma-ram").hasValue(), "maps DMA test RAM");
    fil::test::check(memory.write32(0x20000000U, 0x11223344U).hasValue(), "initializes DMA source");
    fil::stm32g4::DmaPeripheral dma("DMA1", 7, &memory);
    int dma_interrupts = 0;
    dma.setInterruptCallback([&](const unsigned int channel) {
        if (channel == 1U) ++dma_interrupts;
    });
    fil::test::check(dma.write(0x0c, fil::mem::AccessSize::word, 1, write_context).hasValue(), "sets DMA item count");
    fil::test::check(dma.write(0x10, fil::mem::AccessSize::word, 0x20000020U, write_context).hasValue(), "sets DMA peripheral address");
    fil::test::check(dma.write(0x14, fil::mem::AccessSize::word, 0x20000000U, write_context).hasValue(), "sets DMA memory address");
    const std::uint32_t control = 1U | (1U << 1U) | (1U << 4U) | (2U << 8U) | (2U << 10U);
    fil::test::check(dma.write(0x08, fil::mem::AccessSize::word, control, write_context).hasValue(), "enables a request-driven DMA channel");
    fil::test::check(dma.request(1), "peripheral request services the DMA channel");
    const auto copied = memory.read32(0x20000020U);
    const auto dma_status = dma.read(0, fil::mem::AccessSize::word, read_context);
    fil::test::check(copied && copied.value() == 0x11223344U, "DMA copies configured data through MemoryBus");
    fil::test::check(dma_status && (dma_status.value() & (1U << 1U)) != 0U, "DMA sets channel transfer-complete flag");
    fil::test::check(dma_interrupts == 1 && dma.transferLog().front().success, "DMA signals completion callback");
    dma.setTransferHistoryEnabled(false);
    fil::test::check(dma.write(0x0c, fil::mem::AccessSize::word, 1, write_context).hasValue()
                         && dma.write(0x08, fil::mem::AccessSize::word, control,
                                      write_context).hasValue()
                         && dma.request(1) && dma.transferLog().size() == 1U,
                     "DMA can service requests without retaining diagnostic history");

    fil::stm32g4::DmamuxPeripheral dmamux;
    const std::uint64_t initial_routing = dmamux.routingGeneration();
    fil::test::check(dmamux.write(0, fil::mem::AccessSize::word, 36, write_context).hasValue(), "stores DMAMUX request selection");
    fil::test::check(dmamux.requestForChannel(0) == 36
                         && dmamux.routingGeneration() > initial_routing,
                     "DMAMUX exposes and versions the selected request");

    fil::sim::EventLoop loop;
    fil::stm32g4::IwdgPeripheral watchdog(true, &loop);
    int resets = 0;
    watchdog.setResetCallback([&]() { ++resets; });
    fil::test::check(watchdog.write(0, fil::mem::AccessSize::word, 0x5555U, write_context).hasValue(), "unlocks watchdog registers");
    fil::test::check(watchdog.write(8, fil::mem::AccessSize::word, 0, write_context).hasValue(), "sets watchdog reload counter");
    fil::test::check(watchdog.write(0, fil::mem::AccessSize::word, 0xccccU, write_context).hasValue(), "starts watchdog");
    fil::test::check(loop.runDueEvents(124999).events_executed == 0, "watchdog remains armed before timeout");
    fil::test::check(loop.runDueEvents(125000).events_executed == 1 && resets == 1, "watchdog requests reset at deterministic timeout");
}

} // namespace

/** @brief Runs STM32G4 peripheral foundation unit tests. */
void runPeripheralTests() {
    storesRegistersAndUnknownMmio();
    journalsTransactionalRegisters();
    modelsClockFlashAndGpioStartup();
    modelsUsartAndSpiDataPaths();
    drivesTimerAndAdcFromSimulatedTime();
    sequencesAdcChannelsWithRegisterDerivedTiming();
    lazilySynchronizesUnobservedContinuousAdc();
    overridesConfiguredAdcProvider();
    preservesObservableAndSingleShotAdcEvents();
    completesDmaAndWatchdogSideEffects();
}
