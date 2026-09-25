#include "fil/stm32g4/stm32g4.hpp"

#include "fil/cortexm/system_control.hpp"

#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <string>
#include <utility>

namespace fil::stm32g4 {
namespace {

constexpr std::uint32_t peripheral_base = 0x40000000U;

Error configError(std::string message) {
    return Error{ErrorCategory::config, std::move(message), std::nullopt};
}

template <typename T>
T* named(std::vector<std::unique_ptr<T>>& devices, const std::string_view name) noexcept {
    for (auto& device : devices) {
        if (device->name() == name) return device.get();
    }
    return nullptr;
}

} // namespace

Stm32G4::Stm32G4(
    sim::EventLoop& event_loop,
    sim::TraceRecorder& trace,
    cortexm::SystemControl& system,
    const bool hse_present,
    const std::uint64_t hse_hz
) : system_(system),
    router_(peripheral_base, 0x20000000U, "stm32g4-peripherals", true, 0),
    rcc_(hse_present, hse_hz, &event_loop, &trace),
    flash_(&event_loop, &trace),
    crc_(&event_loop, &trace),
    pwr_(&event_loop, &trace),
    dma1_("DMA1", 8, nullptr, &event_loop, &trace),
    dma2_("DMA2", 8, nullptr, &event_loop, &trace),
    dmamux_("DMAMUX", 16, &event_loop, &trace),
    iwdg_(false, &event_loop, &trace),
    wwdg_(false, 16000000U, &event_loop, &trace),
    syscfg_("SYSCFG", &event_loop, &trace),
    exti_("EXTI", &event_loop, &trace) {
    constexpr std::array<std::string_view, 7> gpio_names{
        "GPIOA", "GPIOB", "GPIOC", "GPIOD", "GPIOE", "GPIOF", "GPIOG",
    };
    for (const auto name : gpio_names) gpio_.push_back(std::make_unique<GpioPeripheral>(std::string(name), &event_loop, &trace));

    constexpr std::array<std::string_view, 3> usart_names{"USART1", "USART2", "USART3"};
    for (const auto name : usart_names) usart_.push_back(std::make_unique<UsartPeripheral>(std::string(name), &event_loop, &trace));

    constexpr std::array<std::string_view, 12> timer_names{
        "TIM2", "TIM3", "TIM4", "TIM6", "TIM7", "TIM1",
        "TIM8", "TIM15", "TIM16", "TIM17", "TIM20", "TIM5",
    };
    for (const auto name : timer_names) timers_.push_back(std::make_unique<TimerPeripheral>(std::string(name), 16000000U, &event_loop, &trace));

    constexpr std::array<std::string_view, 4> adc_names{"ADC1", "ADC2", "ADC3", "ADC4"};
    for (const auto name : adc_names) adc_.push_back(std::make_unique<AdcPeripheral>(std::string(name), &event_loop, &trace));

    constexpr std::array<std::string_view, 3> spi_names{"SPI1", "SPI2", "SPI3"};
    for (const auto name : spi_names) spi_.push_back(std::make_unique<SpiPeripheral>(std::string(name), &event_loop, &trace));

    for (unsigned int instance = 1U; instance <= FdcanMessageRam::controllerCount; ++instance) {
        fdcan_.push_back(std::make_unique<FdcanPeripheral>(instance, fdcan_message_ram_, &event_loop, &trace));
    }

    stubs_.push_back(std::make_unique<UnknownMmioDevice>("ADC12_COMMON", 0x50000300U, false, 0, &event_loop, &trace));
    stubs_.push_back(std::make_unique<UnknownMmioDevice>("ADC345_COMMON", 0x50000700U, false, 0, &event_loop, &trace));
    exti_.setSyscfg(&syscfg_);
}

Stm32G4::~Stm32G4() {
    clearInterruptLines();
}

Result<std::unique_ptr<Stm32G4>> Stm32G4::create(
    sim::EventLoop& event_loop,
    sim::TraceRecorder& trace,
    cortexm::SystemControl& system,
    const bool hse_present,
    const std::uint64_t hse_hz
) {
    auto mcu = std::unique_ptr<Stm32G4>(
        new Stm32G4(event_loop, trace, system, hse_present, hse_hz)
    );
    auto mapped = mcu->mapDevices();
    if (!mapped) return mapped.error();
    mcu->wireInterrupts();
    return mcu;
}

Result<void> Stm32G4::map(const std::uint32_t absolute_address, RegisterPeripheral& device) {
    return router_.map(absolute_address - peripheral_base, device.size(), device, std::string(device.name()));
}

Result<void> Stm32G4::mapDevices() {
    if (mapped_) return Error{ErrorCategory::internal, "STM32G4 peripherals already mapped", std::nullopt};
    auto checked = [&](const std::uint32_t address, RegisterPeripheral& device) -> Result<void> {
        auto result = map(address, device);
        if (!result) return result.error();
        return {};
    };

    for (auto entry : std::array<std::pair<std::uint32_t, RegisterPeripheral*>, 11>{
        std::pair{0x40021000U, static_cast<RegisterPeripheral*>(&rcc_)},
        {0x40022000U, &flash_}, {0x40023000U, &crc_}, {0x40007000U, &pwr_},
        {0x40020000U, &dma1_}, {0x40020400U, &dma2_}, {0x40020800U, &dmamux_},
        {0x40003000U, &iwdg_}, {0x40002c00U, &wwdg_},
        {0x40010000U, &syscfg_}, {0x40010400U, &exti_},
    }) {
        auto result = checked(entry.first, *entry.second);
        if (!result) return result.error();
    }

    for (std::size_t index = 0; index < gpio_.size(); ++index) {
        auto result = checked(0x48000000U + static_cast<std::uint32_t>(index) * 0x400U, *gpio_[index]);
        if (!result) return result.error();
    }
    constexpr std::array<std::uint32_t, 3> usart_bases{0x40013800U, 0x40004400U, 0x40004800U};
    for (std::size_t index = 0; index < usart_.size(); ++index) {
        auto result = checked(usart_bases[index], *usart_[index]);
        if (!result) return result.error();
    }
    constexpr std::array<std::uint32_t, 12> timer_bases{
        0x40000000U, 0x40000400U, 0x40000800U, 0x40001000U, 0x40001400U,
        0x40012c00U, 0x40013400U, 0x40014000U, 0x40014400U, 0x40014800U,
        0x40015000U, 0x40000c00U,
    };
    for (std::size_t index = 0; index < timers_.size(); ++index) {
        auto result = checked(timer_bases[index], *timers_[index]);
        if (!result) return result.error();
    }
    constexpr std::array<std::uint32_t, 4> adc_bases{0x50000000U, 0x50000100U, 0x50000400U, 0x50000500U};
    for (std::size_t index = 0; index < adc_.size(); ++index) {
        auto result = checked(adc_bases[index], *adc_[index]);
        if (!result) return result.error();
    }
    constexpr std::array<std::uint32_t, 3> spi_bases{0x40013000U, 0x40003800U, 0x40003c00U};
    for (std::size_t index = 0; index < spi_.size(); ++index) {
        auto result = checked(spi_bases[index], *spi_[index]);
        if (!result) return result.error();
    }
    for (std::size_t index = 0; index < fdcan_.size(); ++index) {
        auto result = checked(FdcanPeripheral::baseAddresses[index], *fdcan_[index]);
        if (!result) return result.error();
    }
    {
        auto result = router_.map(
            FdcanMessageRam::baseAddress - peripheral_base,
            FdcanMessageRam::sizeBytes,
            fdcan_message_ram_,
            std::string(fdcan_message_ram_.name())
        );
        if (!result) return result.error();
    }
    constexpr std::array<std::pair<std::uint32_t, std::uint32_t>, 2> stub_ranges{
        std::pair{0x50000300U, 0x100U}, {0x50000700U, 0x100U},
    };
    for (std::size_t index = 0; index < stubs_.size(); ++index) {
        auto result = router_.map(
            stub_ranges[index].first - peripheral_base, stub_ranges[index].second,
            *stubs_[index], std::string(stubs_[index]->name())
        );
        if (!result) return result.error();
    }
    mapped_ = true;
    return {};
}

void Stm32G4::wireInterrupts() {
    unsigned int source = 0U;
    const auto connect = [this, &source](RegisterPeripheral& peripheral, const auto irqs) {
        const std::uint32_t source_bit = std::uint32_t{1} << source++;
        peripheral.setInterruptLevelCallback(
            [this, irqs, source_bit](const unsigned int line, const bool asserted) {
                if (line >= irqs.size()) return;
                const std::uint16_t irq = irqs[line];
                auto& sources = irq_sources_[irq];
                if (asserted) sources |= source_bit;
                else sources &= ~source_bit;
                system_.setInterruptLine(irq, sources != 0U);
            }
        );
    };
    constexpr std::array<std::uint16_t, 3> usart_irqs{37, 38, 39};
    for (std::size_t index = 0; index < usart_.size(); ++index) {
        connect(*usart_[index], std::array{usart_irqs[index]});
    }
    constexpr std::array<std::uint16_t, 3> spi_irqs{35, 36, 51};
    for (std::size_t index = 0; index < spi_.size(); ++index) {
        connect(*spi_[index], std::array{spi_irqs[index]});
    }
    constexpr std::array<std::uint16_t, 12> timer_irqs{28, 29, 30, 54, 55, 25, 44, 24, 25, 26, 78, 50};
    for (std::size_t index = 0; index < timers_.size(); ++index) {
        connect(*timers_[index], std::array{timer_irqs[index]});
    }
    constexpr std::array<std::uint8_t, 4> adc_dma_requests{5U, 36U, 37U, 38U};
    for (std::size_t index = 0; index < adc_.size(); ++index) {
        connect(*adc_[index], std::array<std::uint16_t, 1>{18U});
        adc_[index]->setSampleCallback(
            [this, request = adc_dma_requests[index]](const AdcSample&) {
                serviceDmaRequest(request);
            }
        );
    }
    constexpr std::array<std::array<std::uint16_t, 2>, 3> fdcan_irqs{{
        {{21U, 22U}}, {{86U, 87U}}, {{88U, 89U}},
    }};
    for (std::size_t index = 0; index < fdcan_.size(); ++index) {
        connect(*fdcan_[index], fdcan_irqs[index]);
    }
    connect(dma1_, std::array<std::uint16_t, 8>{11U, 12U, 13U, 14U, 15U, 16U, 17U, 96U});
    connect(dma2_, std::array<std::uint16_t, 8>{56U, 57U, 58U, 59U, 60U, 97U, 98U, 99U});
    connect(exti_, std::array<std::uint16_t, 7>{6U, 7U, 8U, 9U, 10U, 23U, 40U});
    for (std::size_t port = 0; port < gpio_.size(); ++port) {
        gpio_[port]->setEdgeCallback([this, port](const unsigned int pin, const bool high, const sim::SimTimeNs) {
            exti_.notifyGpioEdge(static_cast<unsigned int>(port), pin, high);
        });
    }
    // RM0440 Table 91 DMAMUX requests: USART1-3 RX 24/26/28 TX 25/27/29,
    // SPI1-3 RX 10/12/14 TX 11/13/15. Bursts drain the channel synchronously
    // with suppression so TDR/DR writes do not recurse.
    constexpr std::array<std::uint8_t, 3> usart_tx_requests{25U, 27U, 29U};
    constexpr std::array<std::uint8_t, 3> usart_rx_requests{24U, 26U, 28U};
    for (std::size_t i = 0; i < usart_.size() && i < 3U; ++i) {
        usart_[i]->setDmaRequestCallback(
            [this, usart = usart_[i].get(), tx = usart_tx_requests[i], rx = usart_rx_requests[i]](const bool transmit) {
                serviceUsartDma(usart, tx, rx, transmit);
            }
        );
    }
    constexpr std::array<std::uint8_t, 3> spi_tx_requests{11U, 13U, 15U};
    constexpr std::array<std::uint8_t, 3> spi_rx_requests{10U, 12U, 14U};
    for (std::size_t i = 0; i < spi_.size() && i < 3U; ++i) {
        spi_[i]->setDmaRequestCallback(
            [this, spi = spi_[i].get(), tx = spi_tx_requests[i], rx = spi_rx_requests[i]](const bool transmit) {
                serviceSpiDma(spi, tx, rx, transmit);
            }
        );
    }
    const auto dma_enable_trigger = [this](const unsigned int) { serviceAllSerialDma(); };
    dma1_.setEnableCallback(dma_enable_trigger);
    dma2_.setEnableCallback(dma_enable_trigger);
    rcc_.setClockChangedCallback([this](const std::uint64_t frequency) {
        for (auto& timer : timers_) timer->setInputClockHz(frequency);
        for (auto& adc : adc_) adc->setInputClockHz(frequency);
    });
    iwdg_.setResetCallback([this] { reset_requested_ = true; });
    wwdg_.setResetCallback([this] { reset_requested_ = true; });
}

void Stm32G4::serviceDmaRequest(const std::uint8_t request) {
    static_cast<void>(serviceDmaRequestOnce(request));
}

bool Stm32G4::serviceDmaRequestOnce(const std::uint8_t request) {
    const std::uint64_t generation = dmamux_.routingGeneration();
    if (dma_route_generation_ != generation) {
        dma_request_routes_.fill(0U);
        for (unsigned int channel = 0U; channel < 16U; ++channel) {
            const std::uint8_t selected = dmamux_.requestForChannel(channel);
            dma_request_routes_[selected] |= static_cast<std::uint16_t>(1U << channel);
        }
        dma_route_generation_ = generation;
    }

    bool progressed = false;
    std::uint16_t routes = dma_request_routes_[request];
    while (routes != 0U) {
        const auto mux_channel = static_cast<unsigned int>(std::countr_zero(routes));
        routes &= static_cast<std::uint16_t>(routes - 1U);
        bool ok = false;
        if (mux_channel < 8U) {
            ok = dma1_.request(mux_channel + 1U);
        } else {
            ok = dma2_.request(mux_channel - 7U);
        }
        progressed = progressed || ok;
    }
    return progressed;
}

void Stm32G4::serviceUsartDma(
    UsartPeripheral* const usart,
    const std::uint8_t tx_request,
    const std::uint8_t rx_request,
    const bool transmit
) {
    if (usart == nullptr) return;
    if (transmit) {
        // TX (memory->periph to TDR): TXE is always ready in this instantaneous
        // model, so drain the DMA channel synchronously. Suppress re-triggering
        // during the burst; TDR writes do not re-trigger TX DMA by design.
        usart->setDmaSuppress(true);
        for (unsigned int i = 0; i < 65536U; ++i) {
            if (!serviceDmaRequestOnce(tx_request)) break;
        }
        usart->setDmaSuppress(false);
    } else {
        // RX (periph->memory from RDR): only transfer while data is available,
        // otherwise DMA would consume zeros. Each RDR pop may reveal more data.
        usart->setDmaSuppress(true);
        for (unsigned int i = 0; i < 65536U; ++i) {
            if (!usart->hasRxData()) break;
            if (!serviceDmaRequestOnce(rx_request)) break;
        }
        usart->setDmaSuppress(false);
    }
}

void Stm32G4::serviceSpiDma(
    SpiPeripheral* const spi,
    const std::uint8_t tx_request,
    const std::uint8_t rx_request,
    const bool transmit
) {
    if (spi == nullptr) return;
    spi->setDmaSuppress(true);
    if (transmit) {
        for (unsigned int i = 0; i < 65536U; ++i) {
            if (!serviceDmaRequestOnce(tx_request)) break;
        }
        // Full-duplex: each TX byte generated an RX byte; drain RX now that
        // TX burst is complete and suppression will be lifted for the RX loop
        // below (still suppressed here, so use direct Once with hasRxData).
        for (unsigned int i = 0; i < 65536U; ++i) {
            if (!spi->hasRxData()) break;
            if (!serviceDmaRequestOnce(rx_request)) break;
        }
    } else {
        for (unsigned int i = 0; i < 65536U; ++i) {
            if (!spi->hasRxData()) break;
            if (!serviceDmaRequestOnce(rx_request)) break;
        }
    }
    spi->setDmaSuppress(false);
}

void Stm32G4::serviceAllSerialDma() {
    // Called on DMA channel enable: a peripheral already ready (TXE/RXNE with
    // DMAT/DMAR) should start even though its enable edge already passed.
    // Bursts are safe no-ops when the peripheral or DMA channel is not ready.
    constexpr std::array<std::uint8_t, 3> usart_tx{25U, 27U, 29U};
    constexpr std::array<std::uint8_t, 3> usart_rx{24U, 26U, 28U};
    for (std::size_t i = 0; i < usart_.size() && i < 3U; ++i) {
        if (usart_[i]->dmaTxEnabled()) serviceUsartDma(usart_[i].get(), usart_tx[i], usart_rx[i], true);
        if (usart_[i]->dmaRxEnabled() && usart_[i]->hasRxData())
            serviceUsartDma(usart_[i].get(), usart_tx[i], usart_rx[i], false);
    }
    constexpr std::array<std::uint8_t, 3> spi_tx{11U, 13U, 15U};
    constexpr std::array<std::uint8_t, 3> spi_rx{10U, 12U, 14U};
    for (std::size_t i = 0; i < spi_.size() && i < 3U; ++i) {
        // TXE is always ready; RX only when data available.
        serviceSpiDma(spi_[i].get(), spi_tx[i], spi_rx[i], true);
        if (spi_[i]->hasRxData()) serviceSpiDma(spi_[i].get(), spi_tx[i], spi_rx[i], false);
    }
}

void Stm32G4::attachMemory(mem::MemoryBus& memory) noexcept {
    dma1_.setMemory(&memory);
    dma2_.setMemory(&memory);

    std::uint32_t flash_base = 0x08000000U;
    std::uint32_t flash_size = 0U;
    for (const auto& region : memory.regions()) {
        if (region.name == "flash") {
            flash_base = region.base;
            flash_size = region.size;
        }
    }
    if (flash_size == 0U) {
        flash_size = 512U * 1024U;
    }
    // STM32G4 default OPTR.DBANK=0: one 512 KiB bank of 4 KiB pages.
    flash_.setEraseGeometry(flash_base, 4096U, flash_size);
    flash_.setPageEraseCallback(
        [&memory](const std::uint32_t page_base, const std::uint32_t page_size) {
            const std::vector<std::uint8_t> erased(page_size, 0xffU);
            return memory.loadBytes(page_base, erased);
        }
    );
}

GpioPeripheral* Stm32G4::gpio(const std::string_view name) noexcept { return named(gpio_, name); }
UsartPeripheral* Stm32G4::usart(const std::string_view name) noexcept { return named(usart_, name); }
AdcPeripheral* Stm32G4::adc(const std::string_view name) noexcept { return named(adc_, name); }
SpiPeripheral* Stm32G4::spi(const std::string_view name) noexcept { return named(spi_, name); }
FdcanPeripheral* Stm32G4::fdcan(const std::string_view name) noexcept { return named(fdcan_, name); }

Result<void> Stm32G4::configure(const config::BoardConfig& board) {
    for (auto& device : usart_) device->setTxCallback({});
    usart_tx_files_.clear();
    for (const config::GpioPinConfig& pin : board.gpio) {
        if (pin.pin.size() < 3 || pin.pin[0] != 'P' || pin.pin[1] < 'A' || pin.pin[1] > 'G') {
            return configError("invalid GPIO pin name '" + pin.pin + "'");
        }
        unsigned int number = 0;
        const auto conversion = std::from_chars(pin.pin.data() + 2, pin.pin.data() + pin.pin.size(), number);
        if (conversion.ec != std::errc{} || conversion.ptr != pin.pin.data() + pin.pin.size() || number > 15U) {
            return configError("invalid GPIO pin number in '" + pin.pin + "'");
        }
        GpioPeripheral* port = gpio_[static_cast<std::size_t>(pin.pin[1] - 'A')].get();
        port->setInput(number, pin.value);
    }

    for (const config::UsartConfig& config : board.usart) {
        UsartPeripheral* device = usart(config.instance);
        if (device == nullptr) return configError("unknown USART instance '" + config.instance + "'");
        device->injectRx(config.scripted_rx);
        if (config.tx_log) {
            auto output = std::make_unique<std::ofstream>(
                *config.tx_log, std::ios::binary | std::ios::trunc
            );
            if (!*output) {
                return configError("unable to open USART TX log '" + config.tx_log->string() + "'");
            }
            std::ofstream* sink = output.get();
            device->setTxCallback([sink](const std::uint8_t byte, const sim::SimTimeNs) {
                sink->put(static_cast<char>(byte));
                sink->flush();
            });
            usart_tx_files_.push_back(std::move(output));
        }
    }
    for (const config::SpiConfig& config : board.spi) {
        SpiPeripheral* device = spi(config.instance);
        if (device == nullptr) return configError("unknown SPI instance '" + config.instance + "'");
        if (config.device == "echo") device->setEcho(true);
        else if (config.device != "zero") return configError("unknown SPI device model '" + config.device + "'");
    }
    for (const config::CanControllerConfig& config : board.can) {
        if (fdcan(config.instance) == nullptr) {
            return configError("unknown FDCAN instance '" + config.instance + "'");
        }
    }
    for (const config::AdcConfig& config : board.adc) {
        AdcPeripheral* device = adc(config.instance);
        if (device == nullptr) return configError("unknown ADC instance '" + config.instance + "'");
        for (const auto& [channel, source] : config.channels) {
            if (source.kind == config::AdcChannelConfig::Kind::constant) device->setChannelValue(channel, source.value);
        }
        const auto channels = config.channels;
        device->setChannelProvider([channels](const unsigned int channel, const sim::SimTimeNs now) {
            const auto found = channels.find(static_cast<std::uint8_t>(channel));
            if (found == channels.end()) return std::uint16_t{0};
            const auto& source = found->second;
            if (source.kind == config::AdcChannelConfig::Kind::constant) return source.value;
            const double period_ns = static_cast<double>(source.period_ms) * 1000000.0;
            const double phase = static_cast<double>(now % static_cast<std::uint64_t>(period_ns)) / period_ns;
            const double normalized = (std::sin(phase * 2.0 * std::numbers::pi) + 1.0) * 0.5;
            const double span = static_cast<double>(source.maximum - source.minimum);
            return static_cast<std::uint16_t>(static_cast<double>(source.minimum) + normalized * span + 0.5);
        });
    }
    return {};
}

void Stm32G4::setAdcDiagnosticsEnabled(const bool enabled) {
    for (auto& device : adc_) device->setSampleHistoryEnabled(enabled);
    dma1_.setTransferHistoryEnabled(enabled);
    dma2_.setTransferHistoryEnabled(enabled);
}

void Stm32G4::setTraceSourcePrefix(const std::string_view prefix) {
    for (RegisterPeripheral* device : std::array<RegisterPeripheral*, 11>{
             &rcc_, &flash_, &crc_, &pwr_, &dma1_, &dma2_, &dmamux_, &iwdg_, &wwdg_,
             &syscfg_, &exti_,
         }) {
        device->setTraceSourcePrefix(prefix);
    }
    const auto qualify = [prefix](auto& devices) {
        for (auto& device : devices) device->setTraceSourcePrefix(prefix);
    };
    qualify(gpio_);
    qualify(usart_);
    qualify(timers_);
    qualify(adc_);
    qualify(spi_);
    qualify(fdcan_);
    qualify(stubs_);
}

void Stm32G4::clearInterruptLines() {
    for (std::uint16_t irq = 0U; irq < irq_sources_.size(); ++irq) {
        if (irq_sources_[irq] != 0U) system_.setInterruptLine(irq, false);
    }
    irq_sources_.fill(0U);
}

void Stm32G4::reset() {
    // Drop shared-source state together, before individual devices reset. This
    // avoids reasserting stale sources when SystemControl has already reset.
    clearInterruptLines();
    rcc_.reset();
    flash_.reset();
    crc_.reset();
    pwr_.reset();
    dma1_.reset();
    dma2_.reset();
    dmamux_.reset();
    dma_route_generation_ = 0U;
    iwdg_.reset();
    wwdg_.reset();
    syscfg_.reset();
    exti_.reset();
    for (auto& device : gpio_) device->reset();
    for (auto& device : usart_) device->reset();
    for (auto& device : timers_) device->reset();
    for (auto& device : adc_) device->reset();
    for (auto& device : spi_) device->reset();
    fdcan_message_ram_.reset();
    for (auto& device : fdcan_) device->reset();
    reset_requested_ = false;
}

bool Stm32G4::consumeResetRequest() noexcept {
    const bool requested = reset_requested_;
    reset_requested_ = false;
    return requested;
}

} // namespace fil::stm32g4
