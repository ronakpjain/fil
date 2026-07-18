#pragma once

/** @file mcu_map.hpp
 *  @brief Construction and reset-vector validation for an STM32 MCU memory map.
 */

#include "fil/common/result.hpp"
#include "fil/config/config.hpp"
#include "fil/elf/elf_loader.hpp"
#include "fil/mem/memory_bus.hpp"

namespace fil::mem {

/** @brief Caller-owned MMIO windows installed into a standard STM32 map. */
struct McuMmioWindows {
    MmioDevice& peripherals;   ///< STM32 peripheral aperture at `0x40000000`.
    MmioDevice& system_control; ///< Cortex-M system aperture at `0xe0000000`.
};

/**
 * @brief Builds the deterministic reset memory image for one MCU and ELF.
 *
 * The resulting map contains flash, SRAM, CCM SRAM, the flash boot alias,
 * system-memory stubs, STM32 peripheral MMIO, and Cortex-M system MMIO. ELF
 * bytes are materialized at physical/load addresses, leaving split-VMA SRAM
 * zero-filled for firmware startup code.
 */
[[nodiscard]] Result<MemoryBus> buildMcuMemoryMap(
    const config::McuConfig& mcu,
    const elf::ElfImage& image,
    McuMmioWindows windows,
    bool executable_sram = false
);

/** @brief Validates reset vectors against an already constructed address map. */
[[nodiscard]] Result<void> validateResetVectors(
    const elf::ElfImage& image,
    const MemoryBus& memory
);

} // namespace fil::mem
