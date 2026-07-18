#pragma once

/** @file mmio_router.hpp
 *  @brief Deterministic sub-range routing and lenient fallback for MMIO windows.
 */

#include "fil/common/result.hpp"
#include "fil/mem/memory_bus.hpp"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace fil::mem {

/** @brief Aggregate information for an address handled by the lenient fallback. */
struct UnknownMmioAccess {
    std::uint32_t address{0}; ///< Absolute target address.
    std::uint64_t reads{0};  ///< Number of reads at this address.
    std::uint64_t writes{0}; ///< Number of writes at this address.
};

/**
 * @brief Routes one large MMIO window to non-overlapping child devices.
 *
 * Mapping a peripheral aperture as one MemoryBus region makes otherwise
 * unknown registers deterministic without preventing specific models from
 * being installed inside the aperture. Unknown addresses can either return a
 * configured value and remember aggregate counts, or produce a device fault.
 */
class MmioRouter final : public MmioDevice {
public:
    /** @brief Creates a router for an absolute target window. */
    explicit MmioRouter(
        std::uint32_t absolute_base,
        std::uint32_t window_size,
        std::string name,
        bool lenient = true,
        std::uint64_t unknown_read_value = 0
    );

    /** @brief Adds a caller-owned child device at a byte offset in the window. */
    [[nodiscard]] Result<void> map(
        std::uint32_t offset,
        std::uint32_t size,
        MmioDevice& device,
        std::string name
    );

    /** @brief Switches between fallback reads/writes and strict device faults. */
    void setLenient(bool value) noexcept { lenient_ = value; }

    /** @brief Sets the unmasked value returned by future unknown reads. */
    void setUnknownReadValue(std::uint64_t value) noexcept { unknown_read_value_ = value; }

    /** @brief Gets stable, address-sorted aggregate unknown-access records. */
    [[nodiscard]] std::vector<UnknownMmioAccess> unknownAccesses() const;

    /** @brief Clears aggregate unknown-access records. */
    void clearUnknownAccesses() noexcept { unknown_.clear(); }

    [[nodiscard]] MemoryResult<std::uint64_t> read(
        std::uint32_t offset,
        AccessSize size,
        const AccessContext& context
    ) override;

    [[nodiscard]] MemoryResult<std::uint64_t> write(
        std::uint32_t offset,
        AccessSize size,
        std::uint64_t value,
        const AccessContext& context
    ) override;

    [[nodiscard]] std::string_view name() const noexcept override { return name_; }

    [[nodiscard]] bool transactionalAccessSafe(
        std::uint32_t offset, AccessSize size, bool write
    ) const noexcept override;

    [[nodiscard]] MmioDomain domain(
        std::uint32_t offset, AccessSize size
    ) const noexcept override;

private:
    struct Route {
        std::uint32_t offset{0};
        std::uint32_t size{0};
        MmioDevice* device{nullptr};
        std::string name;
    };

    [[nodiscard]] const Route* find(std::uint32_t offset, std::uint32_t width) const;
    [[nodiscard]] UnknownMmioAccess& record(std::uint32_t absolute_address);
    [[nodiscard]] BusFault fault(
        std::uint32_t offset,
        AccessSize size,
        const AccessContext& context,
        std::string message
    ) const;

    std::uint32_t absolute_base_{0};
    std::uint32_t window_size_{0};
    std::string name_;
    bool lenient_{true};
    std::uint64_t unknown_read_value_{0};
    std::vector<Route> routes_;
    std::vector<UnknownMmioAccess> unknown_;
};

} // namespace fil::mem
