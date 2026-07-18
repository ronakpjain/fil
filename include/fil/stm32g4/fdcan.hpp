#pragma once

/** @file fdcan.hpp
 *  @brief STM32G4 FDCAN controller and shared message-RAM models.
 */

#include "fil/devices/can_bus.hpp"
#include "fil/stm32g4/peripheral.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

namespace fil::stm32g4 {

class FdcanPeripheral;

/**
 * @brief CPU-visible message RAM shared by all three STM32G4 FDCAN cores.
 *
 * STM32G4 hardware assigns one fixed 0x350-byte slice per core. Unlike the
 * configurable message-RAM layout used by some other M_CAN integrations, the
 * G4 register block exposes no SIDFC, XIDFC, RXF0C, RXESC, or TXESC registers;
 * its official HAL uses these same fixed offsets, counts, and 64-byte element
 * sizes. RXGFC still selects how many of the reserved standard and extended
 * filter slots are active. Element sizes include their two header words.
 */
class FdcanMessageRam final : public mem::MmioDevice {
public:
    static constexpr std::uint32_t baseAddress = 0x4000a400U;
    static constexpr unsigned int controllerCount = 3U;
    static constexpr std::uint32_t controllerStride = 0x350U;
    static constexpr std::uint32_t sizeBytes = controllerCount * controllerStride;

    static constexpr std::uint32_t standardFilterOffset = 0x000U;
    static constexpr std::uint32_t standardFilterCount = 28U;
    static constexpr std::uint32_t standardFilterElementSize = 4U;
    static constexpr std::uint32_t extendedFilterOffset = 0x070U;
    static constexpr std::uint32_t extendedFilterCount = 8U;
    static constexpr std::uint32_t extendedFilterElementSize = 8U;
    static constexpr std::uint32_t rxFifo0Offset = 0x0b0U;
    static constexpr std::uint32_t rxFifoElementCount = 3U;
    static constexpr std::uint32_t rxFifoElementSize = 72U;
    static constexpr std::uint32_t rxFifo1Offset = 0x188U;
    static constexpr std::uint32_t txEventFifoOffset = 0x260U;
    static constexpr std::uint32_t txEventElementCount = 3U;
    static constexpr std::uint32_t txEventElementSize = 8U;
    static constexpr std::uint32_t txFifoOffset = 0x278U;
    static constexpr std::uint32_t txFifoElementCount = 3U;
    static constexpr std::uint32_t txFifoElementSize = 72U;

    explicit FdcanMessageRam(std::string name = "FDCAN message RAM");

    [[nodiscard]] mem::MemoryResult<std::uint64_t> read(std::uint32_t offset, mem::AccessSize size,
                                                        const mem::AccessContext& context) override;

    [[nodiscard]] mem::MemoryResult<std::uint64_t>
    write(std::uint32_t offset, mem::AccessSize size, std::uint64_t value,
          const mem::AccessContext& context) override;

    [[nodiscard]] std::string_view name() const noexcept override { return name_; }

    /** @brief Clears all three controller slices. */
    void reset() noexcept;

    /** @brief Reads one little-endian word from a zero-based controller slice. */
    [[nodiscard]] std::uint32_t loadWord(unsigned int controller_index,
                                         std::uint32_t controller_offset) const noexcept;

    /** @brief Writes one little-endian word in a zero-based controller slice. */
    void storeWord(unsigned int controller_index, std::uint32_t controller_offset,
                   std::uint32_t value) noexcept;

private:
    friend class FdcanPeripheral;

    [[nodiscard]] mem::BusFault fault(std::uint32_t offset, mem::AccessSize size,
                                      const mem::AccessContext& context, std::string message) const;
    [[nodiscard]] static std::optional<std::uint32_t>
    absoluteOffset(unsigned int controller_index, std::uint32_t controller_offset,
                   std::uint32_t width) noexcept;
    void clearRange(unsigned int controller_index, std::uint32_t offset,
                    std::uint32_t size) noexcept;

    std::string name_;
    std::array<std::uint8_t, sizeBytes> bytes_{};
};

static_assert(FdcanMessageRam::standardFilterOffset
                  + FdcanMessageRam::standardFilterCount
                        * FdcanMessageRam::standardFilterElementSize
              == FdcanMessageRam::extendedFilterOffset);
static_assert(FdcanMessageRam::extendedFilterOffset
                  + FdcanMessageRam::extendedFilterCount
                        * FdcanMessageRam::extendedFilterElementSize
              == FdcanMessageRam::rxFifo0Offset);
static_assert(FdcanMessageRam::rxFifo0Offset
                  + FdcanMessageRam::rxFifoElementCount
                        * FdcanMessageRam::rxFifoElementSize
              == FdcanMessageRam::rxFifo1Offset);
static_assert(FdcanMessageRam::rxFifo1Offset
                  + FdcanMessageRam::rxFifoElementCount
                        * FdcanMessageRam::rxFifoElementSize
              == FdcanMessageRam::txEventFifoOffset);
static_assert(FdcanMessageRam::txEventFifoOffset
                  + FdcanMessageRam::txEventElementCount
                        * FdcanMessageRam::txEventElementSize
              == FdcanMessageRam::txFifoOffset);
static_assert(FdcanMessageRam::txFifoOffset
                  + FdcanMessageRam::txFifoElementCount
                        * FdcanMessageRam::txFifoElementSize
              == FdcanMessageRam::controllerStride);

/** @brief Register-level model of one of the three STM32G4 FDCAN cores. */
class FdcanPeripheral final : public RegisterPeripheral {
public:
    [[nodiscard]] bool transactionalAccessSafe(
        std::uint32_t offset, mem::AccessSize size, bool
    ) const noexcept override {
        return offset == 0x01cU && mem::byteCount(size) <= 4U;
    }

    [[nodiscard]] mem::MmioDomain domain(
        std::uint32_t offset, mem::AccessSize size
    ) const noexcept override {
        static_cast<void>(size);
        return offset == 0x01cU
            ? mem::MmioDomain::board_local : mem::MmioDomain::shared;
    }

    enum class Instance : unsigned int {
        fdcan1 = 1U,
        fdcan2 = 2U,
        fdcan3 = 3U,
    };

    using InterruptCallback = std::function<void(unsigned int line)>;
    using LineInterruptCallback = std::function<void()>;

    static constexpr std::uint32_t registerBlockSize = 0x100U;
    static constexpr std::array<std::uint32_t, 3> baseAddresses{
        0x40006400U,
        0x40006800U,
        0x40006c00U,
    };

    static constexpr std::uint32_t cccrOffset = 0x018U;
    static constexpr std::uint32_t irOffset = 0x050U;
    static constexpr std::uint32_t ieOffset = 0x054U;
    static constexpr std::uint32_t ilsOffset = 0x058U;
    static constexpr std::uint32_t ileOffset = 0x05cU;
    static constexpr std::uint32_t rxgfcOffset = 0x080U;
    static constexpr std::uint32_t rxf0sOffset = 0x090U;
    static constexpr std::uint32_t rxf0aOffset = 0x094U;
    static constexpr std::uint32_t txfqsOffset = 0x0c4U;
    static constexpr std::uint32_t txbarOffset = 0x0ccU;
    static constexpr std::uint32_t txbtoOffset = 0x0d4U;
    static constexpr std::uint32_t txbtieOffset = 0x0dcU;

    static constexpr std::uint32_t cccrInit = 1U << 0U;
    static constexpr std::uint32_t cccrCce = 1U << 1U;
    static constexpr std::uint32_t cccrCsa = 1U << 3U;
    static constexpr std::uint32_t cccrCsr = 1U << 4U;
    static constexpr std::uint32_t interruptRxFifo0New = 1U << 0U;
    static constexpr std::uint32_t interruptRxFifo0Full = 1U << 1U;
    static constexpr std::uint32_t interruptRxFifo0Lost = 1U << 2U;
    static constexpr std::uint32_t interruptTransmissionComplete = 1U << 7U;

    explicit FdcanPeripheral(Instance instance, FdcanMessageRam& message_ram,
                             sim::EventLoop* event_loop = nullptr,
                             sim::TraceRecorder* trace = nullptr);

    explicit FdcanPeripheral(unsigned int instance_number, FdcanMessageRam& message_ram,
                             sim::EventLoop* event_loop = nullptr,
                             sim::TraceRecorder* trace = nullptr);

    FdcanPeripheral(std::string name, unsigned int instance_number, FdcanMessageRam& message_ram,
                    sim::EventLoop* event_loop = nullptr, sim::TraceRecorder* trace = nullptr);

    ~FdcanPeripheral() override;

    /** @brief Attaches this core to a deterministic virtual CAN bus. */
    [[nodiscard]] Result<void> attachBus(devices::VirtualCanBus& bus, std::string node_name,
                                         bool loopback = false);

    /** @brief Detaches this core from its current virtual CAN bus. */
    void detachBus() noexcept;

    [[nodiscard]] bool busAttached() const noexcept { return bus_ != nullptr; }
    [[nodiscard]] unsigned int instanceNumber() const noexcept { return instance_index_ + 1U; }
    [[nodiscard]] unsigned int controllerIndex() const noexcept { return instance_index_; }

    /** @brief Installs one callback carrying the asserted FDCAN interrupt line (0
     * or 1). */
    void setInterruptCallback(InterruptCallback callback);

    /** @brief Installs a callback for one individual FDCAN interrupt line. */
    void setInterruptLineCallback(unsigned int line, LineInterruptCallback callback);

    /** @brief Injects a bus frame through this core's filters and RX FIFO 0. */
    [[nodiscard]] bool receiveFrame(const devices::CanFrame& frame, std::uint64_t time_ns);

    /** @brief Reports whether INIT and clock-stop mode are both inactive. */
    [[nodiscard]] bool operational() const noexcept;

protected:
    [[nodiscard]] std::uint32_t loadRegister(std::uint32_t word_offset,
                                             const mem::AccessContext& context) override;

    void storeRegister(std::uint32_t word_offset, std::uint32_t previous, std::uint32_t value,
                       std::uint32_t write_mask, const mem::AccessContext& context) override;

    void onReset() override;

private:
    struct FilterDecision {
        bool accepted{false};
        bool non_matching{true};
        std::uint8_t index{0x7fU};
    };

    [[nodiscard]] FilterDecision filter(const devices::CanFrame& frame) const noexcept;
    [[nodiscard]] bool transmitBuffer(unsigned int buffer_index);
    void acknowledgeRxFifo0(unsigned int acknowledged_index);
    void updateRxFifo0Status() noexcept;
    void updateTxFifoStatus() noexcept;
    void setInterruptFlags(std::uint32_t flags);
    void updateInterruptLines();
    [[nodiscard]] bool interruptLinePending(unsigned int line) const noexcept;

    FdcanMessageRam& message_ram_;
    unsigned int instance_index_{0};
    devices::VirtualCanBus* bus_{nullptr};
    devices::VirtualCanBus::NodeId bus_node_id_{0};
    std::uint8_t rx_fill_level_{0};
    std::uint8_t rx_get_index_{0};
    std::uint8_t rx_put_index_{0};
    bool rx_message_lost_{false};
    std::uint8_t tx_get_index_{0};
    std::uint8_t tx_put_index_{0};
    InterruptCallback interrupt_callback_;
    std::array<LineInterruptCallback, 2> line_interrupt_callbacks_{};
    std::array<bool, 2> interrupt_line_asserted_{};
};

using FdcanController = FdcanPeripheral;
using FdcanMessageRamDevice = FdcanMessageRam;

} // namespace fil::stm32g4
