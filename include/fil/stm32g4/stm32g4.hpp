#pragma once

/** @file stm32g4.hpp
 *  @brief Integrated STM32G474 peripheral aperture and board-config binding.
 */

#include "fil/common/result.hpp"
#include "fil/config/config.hpp"
#include "fil/mem/mmio_router.hpp"
#include "fil/stm32g4/fdcan.hpp"
#include "fil/stm32g4/peripheral.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <fstream>
#include <string_view>
#include <vector>

namespace fil::cortexm {
class SystemControl;
}
namespace fil::mem {
class MemoryBus;
}

namespace fil::stm32g4 {

/**
 * @brief Owns and routes the peripheral models for one STM32G474 instance.
 *
 * Device lifetimes are stable after construction, allowing MmioRouter to hold
 * non-owning pointers. Peripheral interrupt levels are ORed for shared IRQs
 * and drive the caller-owned Cortex-M NVIC model.
 */
class Stm32G4 {
public:
    /** @brief Constructs, routes, and IRQ-wires a complete peripheral set. */
    [[nodiscard]] static Result<std::unique_ptr<Stm32G4>> create(
        sim::EventLoop& event_loop,
        sim::TraceRecorder& trace,
        cortexm::SystemControl& system,
        bool hse_present = true,
        std::uint64_t hse_hz = 8'000'000U
    );

    ~Stm32G4();
    Stm32G4(const Stm32G4&) = delete;
    Stm32G4& operator=(const Stm32G4&) = delete;

    /** @brief Gets the device mapped across the STM32 peripheral aperture. */
    [[nodiscard]] mem::MmioDevice& mmio() noexcept { return router_; }
    [[nodiscard]] mem::MmioRouter& router() noexcept { return router_; }

    /** @brief Installs the finalized memory bus used by DMA transfers. */
    void attachMemory(mem::MemoryBus& memory) noexcept;

    /** @brief Applies external pin, UART, ADC, and SPI behavior from board config. */
    [[nodiscard]] Result<void> configure(const config::BoardConfig& board);

    /** @brief Enables or disables retained per-conversion ADC sample history on every ADC. */
    void setAdcDiagnosticsEnabled(bool enabled);

    /** @brief Qualifies every peripheral trace source as `prefix.device`. */
    void setTraceSourcePrefix(std::string_view prefix);

    /** @brief Resets register-backed devices while retaining host attachments. */
    void reset();

    [[nodiscard]] RccPeripheral& rcc() noexcept { return rcc_; }
    [[nodiscard]] FlashPeripheral& flash() noexcept { return flash_; }
    [[nodiscard]] CrcPeripheral& crc() noexcept { return crc_; }
    [[nodiscard]] GpioPeripheral* gpio(std::string_view name) noexcept;
    [[nodiscard]] UsartPeripheral* usart(std::string_view name) noexcept;
    [[nodiscard]] AdcPeripheral* adc(std::string_view name) noexcept;
    [[nodiscard]] SpiPeripheral* spi(std::string_view name) noexcept;
    [[nodiscard]] FdcanPeripheral* fdcan(std::string_view name) noexcept;
    [[nodiscard]] bool resetRequested() const noexcept { return reset_requested_; }
    [[nodiscard]] bool consumeResetRequest() noexcept;

private:
    Stm32G4(
        sim::EventLoop& event_loop,
        sim::TraceRecorder& trace,
        cortexm::SystemControl& system,
        bool hse_present,
        std::uint64_t hse_hz
    );

    [[nodiscard]] Result<void> mapDevices();
    void wireInterrupts();
    void clearInterruptLines();
    void serviceDmaRequest(std::uint8_t request);
    [[nodiscard]] Result<void> map(
        std::uint32_t absolute_address,
        RegisterPeripheral& device
    );

    cortexm::SystemControl& system_;
    mem::MmioRouter router_;
    std::array<std::uint32_t, 240> irq_sources_{};

    RccPeripheral rcc_;
    FlashPeripheral flash_;
    CrcPeripheral crc_;
    PwrPeripheral pwr_;
    DmaPeripheral dma1_;
    DmaPeripheral dma2_;
    DmamuxPeripheral dmamux_;
    std::array<std::uint16_t, 128> dma_request_routes_{};
    std::uint64_t dma_route_generation_{0U};
    IwdgPeripheral iwdg_;
    WwdgPeripheral wwdg_;
    FdcanMessageRam fdcan_message_ram_;
    std::vector<std::unique_ptr<GpioPeripheral>> gpio_;
    std::vector<std::unique_ptr<UsartPeripheral>> usart_;
    std::vector<std::unique_ptr<TimerPeripheral>> timers_;
    std::vector<std::unique_ptr<AdcPeripheral>> adc_;
    std::vector<std::unique_ptr<SpiPeripheral>> spi_;
    std::vector<std::unique_ptr<FdcanPeripheral>> fdcan_;
    std::vector<std::unique_ptr<std::ofstream>> usart_tx_files_;
    std::vector<std::unique_ptr<UnknownMmioDevice>> stubs_;
    bool mapped_{false};
    bool reset_requested_{false};
};

} // namespace fil::stm32g4
