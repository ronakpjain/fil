#pragma once

/** @file address.hpp
 *  @brief Target memory access types, widths, and structured bus faults.
 */

#include <cstdint>
#include <string>
#include <string_view>

namespace fil::mem {

/** @brief Purpose of a target memory access. */
enum class AccessType {
    instruction_fetch, ///< CPU instruction fetch requiring execute permission.
    data_read,         ///< CPU or peripheral data read.
    data_write,        ///< CPU or peripheral data write.
    debug,             ///< Non-architectural debugger/inspection access.
};

/** @brief Width of one indivisible target bus operation. */
enum class AccessSize : std::uint8_t {
    byte = 1,      ///< 8-bit operation.
    halfword = 2,  ///< 16-bit operation.
    word = 4,      ///< 32-bit operation.
    doubleword = 8 ///< 64-bit operation.
};

/** @brief Reason a memory bus access could not complete. */
enum class BusFaultReason {
    address_overflow, ///< Address plus width wrapped target address space.
    unmapped,         ///< No region contains the requested address.
    cross_region,     ///< A single operation spans multiple regions.
    read_protected,   ///< Region does not permit reads.
    write_protected,  ///< Region does not permit writes.
    execute_protected, ///< Region does not permit instruction fetches.
    alias_cycle,      ///< Alias translation exceeded the recursion limit.
    device_error,     ///< An MMIO device rejected the operation.
    synchronization_required, ///< Worker reached cross-board MMIO before commit.
};

/** @brief Metadata attached to every target memory access. */
struct AccessContext {
    AccessType type{AccessType::data_read}; ///< Access purpose and permission class.
    std::uint32_t pc{0};                   ///< CPU instruction address causing the access.
};

/** @brief Structured target bus fault suitable for architectural fault handling. */
struct BusFault {
    BusFaultReason reason{BusFaultReason::unmapped}; ///< Machine-readable failure reason.
    std::uint32_t address{0};                        ///< First requested target address.
    AccessSize size{AccessSize::byte};               ///< Requested indivisible width.
    AccessContext context;                           ///< Access type and originating PC.
    std::string region;                              ///< Region/device name when known.
    std::string message;                             ///< Human-readable explanation.
};

/**
 * @brief Converts an access width to bytes.
 * @param size Access width.
 * @return Width in bytes.
 */
[[nodiscard]] constexpr std::uint32_t byteCount(const AccessSize size) noexcept {
    return static_cast<std::uint32_t>(size);
}

/**
 * @brief Returns a stable name for an access type.
 * @param type Access purpose.
 * @return Static lowercase name.
 */
[[nodiscard]] std::string_view accessTypeName(AccessType type) noexcept;

/**
 * @brief Formats a bus fault for diagnostics.
 * @param fault Fault to format.
 * @return Human-readable single-line description.
 */
[[nodiscard]] std::string formatBusFault(const BusFault& fault);

} // namespace fil::mem
