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
    wwdg_(false, 16000000U, &event_loop, &trace) {
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

    stubs_.push_back(std::make_unique<UnknownMmioDevice>("SYSCFG", 0x40010000U, false, 0, &event_loop, &trace));
    stubs_.push_back(std::make_unique<UnknownMmioDevice>("EXTI", 0x40010400U, false, 0, &event_loop, &trace));
    stubs_.push_back(std::make_unique<UnknownMmioDevice>("ADC12_COMMON", 0x50000300U, false, 0, &event_loop, &trace));
    stubs_.push_back(std::make_unique<UnknownMmioDevice>("ADC345_COMMON", 0x50000700U, false, 0, &event_loop, &trace));
}

Stm32G4::~Stm32G4() = default;

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

    for (auto entry : std::array<std::pair<std::uint32_t, RegisterPeripheral*>, 9>{
        std::pair{0x40021000U, static_cast<RegisterPeripheral*>(&rcc_)},
        {0x40022000U, &flash_}, {0x40023000U, &crc_}, {0x40007000U, &pwr_},
        {0x40020000U, &dma1_}, {0x40020400U, &dma2_}, {0x40020800U, &dmamux_},
        {0x40003000U, &iwdg_}, {0x40002c00U, &wwdg_},
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
    constexpr std::array<std::pair<std::uint32_t, std::uint32_t>, 4> stub_ranges{
        std::pair{0x40010000U, 0x400U}, {0x40010400U, 0x400U},
        {0x50000300U, 0x100U}, {0x50000700U, 0x100U},
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
    constexpr std::array<std::uint16_t, 3> usart_irqs{37, 38, 39};
    for (std::size_t index = 0; index < usart_.size(); ++index) {
        usart_[index]->setInterruptCallback([this, irq = usart_irqs[index]] { system_.pend(irq + 16U); });
    }
    constexpr std::array<std::uint16_t, 3> spi_irqs{35, 36, 51};
    for (std::size_t index = 0; index < spi_.size(); ++index) {
        spi_[index]->setInterruptCallback([this, irq = spi_irqs[index]] { system_.pend(irq + 16U); });
    }
    constexpr std::array<std::uint16_t, 12> timer_irqs{28, 29, 30, 54, 55, 25, 44, 24, 25, 26, 78, 50};
    for (std::size_t index = 0; index < timers_.size(); ++index) {
        timers_[index]->setInterruptCallback([this, irq = timer_irqs[index]] { system_.pend(irq + 16U); });
    }
    constexpr std::array<std::uint8_t, 4> adc_dma_requests{5U, 36U, 37U, 38U};
    for (std::size_t index = 0; index < adc_.size(); ++index) {
        adc_[index]->setInterruptCallback([this] { system_.pend(18U + 16U); });
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
        fdcan_[index]->setInterruptCallback([this, irqs = fdcan_irqs[index]](const unsigned int line) {
            if (line < irqs.size()) system_.pend(static_cast<std::uint16_t>(irqs[line] + 16U));
        });
    }
    dma1_.setInterruptCallback([this](const unsigned int channel) {
        const std::uint16_t irq = channel == 8U ? 96U : static_cast<std::uint16_t>(10U + channel);
        system_.pend(irq + 16U);
    });
    dma2_.setInterruptCallback([this](const unsigned int channel) {
        const std::uint16_t irq = channel <= 5U
            ? static_cast<std::uint16_t>(55U + channel)
            : static_cast<std::uint16_t>(91U + channel);
        system_.pend(irq + 16U);
    });
    rcc_.setClockChangedCallback([this](const std::uint64_t frequency) {
        for (auto& timer : timers_) timer->setInputClockHz(frequency);
        for (auto& adc : adc_) adc->setInputClockHz(frequency);
    });
    iwdg_.setResetCallback([this] { reset_requested_ = true; });
    wwdg_.setResetCallback([this] { reset_requested_ = true; });
}

void Stm32G4::serviceDmaRequest(const std::uint8_t request) {
    const std::uint64_t generation = dmamux_.routingGeneration();
    if (dma_route_generation_ != generation) {
        dma_request_routes_.fill(0U);
        for (unsigned int channel = 0U; channel < 16U; ++channel) {
            const std::uint8_t selected = dmamux_.requestForChannel(channel);
            dma_request_routes_[selected] |= static_cast<std::uint16_t>(1U << channel);
        }
        dma_route_generation_ = generation;
    }

    std::uint16_t routes = dma_request_routes_[request];
    while (routes != 0U) {
        const auto mux_channel = static_cast<unsigned int>(std::countr_zero(routes));
        routes &= static_cast<std::uint16_t>(routes - 1U);
        if (mux_channel < 8U) {
            static_cast<void>(dma1_.request(mux_channel + 1U));
        } else {
            static_cast<void>(dma2_.request(mux_channel - 7U));
        }
    }
}

void Stm32G4::attachMemory(mem::MemoryBus& memory) noexcept {
    dma1_.setMemory(&memory);
    dma2_.setMemory(&memory);
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
    for (RegisterPeripheral* device : std::array<RegisterPeripheral*, 9>{
             &rcc_, &flash_, &crc_, &pwr_, &dma1_, &dma2_, &dmamux_, &iwdg_, &wwdg_,
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

void Stm32G4::reset() {
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
