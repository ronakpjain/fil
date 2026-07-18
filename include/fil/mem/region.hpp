#pragma once

/** @file region.hpp
 *  @brief Public memory-region metadata used for inspection and diagnostics.
 */

#include <cstdint>
#include <string>

namespace fil::mem {

/** @brief Backing behavior assigned to an address range. */
enum class RegionKind {
    ram,   ///< Mutable zero-initialized byte storage.
    rom,   ///< Read-only storage initialized to erased bytes.
    alias, ///< Address translation into another mapped region.
    mmio,  ///< Indivisible dispatch to a peripheral device.
};

/** @brief Immutable description of one mapped target address range. */
struct MemoryRegionInfo {
    std::uint32_t base{0};       ///< First mapped address.
    std::uint32_t size{0};       ///< Region size in bytes.
    RegionKind kind{RegionKind::ram}; ///< Region backing behavior.
    std::string name;            ///< Stable diagnostic name.
    bool readable{false};        ///< Data reads are permitted.
    bool writable{false};        ///< Data writes are permitted.
    bool executable{false};      ///< Instruction fetches are permitted.
};

} // namespace fil::mem
