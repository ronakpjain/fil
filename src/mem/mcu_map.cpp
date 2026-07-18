#include "fil/mem/mcu_map.hpp"

#include <cstdint>
#include <string>
#include <utility>

namespace fil::mem {
namespace {

Error configFailure(std::string message) {
    return Error{ErrorCategory::config, std::move(message), std::nullopt};
}

bool isWritableStackAddress(const MemoryBus& memory, const std::uint32_t address) {
    for (const MemoryRegionInfo& region : memory.regions()) {
        if (!region.writable || region.kind == RegionKind::mmio) {
            continue;
        }
        const std::uint64_t begin = region.base;
        const std::uint64_t end = begin + region.size;
        // An initial full-descending stack pointer may point one byte past RAM.
        if (address >= begin && address <= end) {
            return true;
        }
    }
    return false;
}

bool isExecutableAddress(const MemoryBus& memory, const std::uint32_t address) {
    for (const MemoryRegionInfo& region : memory.regions()) {
        const std::uint64_t end = static_cast<std::uint64_t>(region.base) + region.size;
        if (region.executable && address >= region.base && address < end) {
            return true;
        }
    }
    return false;
}

} // namespace

Result<void> validateResetVectors(const elf::ElfImage& image, const MemoryBus& memory) {
    if ((image.vectorBase() & 0x7fU) != 0) {
        return configFailure("vector-table base must be 128-byte aligned");
    }
    if ((image.initialMsp() & 0x7U) != 0) {
        return configFailure("initial MSP must be 8-byte aligned");
    }
    if (!isWritableStackAddress(memory, image.initialMsp())) {
        return configFailure("initial MSP does not lie in or immediately above writable memory");
    }
    if ((image.resetHandler() & 1U) == 0) {
        return configFailure("reset handler does not select Thumb state");
    }
    if (!isExecutableAddress(memory, image.resetHandler() & ~1U)) {
        return configFailure("reset handler does not resolve to executable memory");
    }
    return {};
}

Result<MemoryBus> buildMcuMemoryMap(
    const config::McuConfig& mcu,
    const elf::ElfImage& image,
    const McuMmioWindows windows,
    const bool executable_sram
) {
    MemoryBus memory;
    auto flash = memory.mapRom(mcu.flash_base, mcu.flash_size, "flash", true);
    if (!flash) return flash.error();
    auto ccm = memory.mapRam(
        mcu.ccm_sram_base, mcu.ccm_sram_size, "ccm-sram", executable_sram
    );
    if (!ccm) return ccm.error();
    auto sram = memory.mapRam(mcu.sram_base, mcu.sram_size, "sram", executable_sram);
    if (!sram) return sram.error();
    auto system_memory = memory.mapRom(0x1fff0000U, 0x00010000U, "system-memory", false);
    if (!system_memory) return system_memory.error();
    auto boot_alias = memory.mapAlias(0x00000000U, mcu.flash_base, mcu.flash_size, "boot-alias");
    if (!boot_alias) return boot_alias.error();
    auto peripherals = memory.mapMmio(
        0x40000000U, 0x20000000U, windows.peripherals, "stm32-peripherals"
    );
    if (!peripherals) return peripherals.error();
    auto system_control = memory.mapMmio(
        0xe0000000U, 0x00100000U, windows.system_control, "cortexm-system"
    );
    if (!system_control) return system_control.error();
    auto loaded = memory.materialize(image);
    if (!loaded) return loaded.error();
    auto vectors = validateResetVectors(image, memory);
    if (!vectors) return vectors.error();
    return memory;
}

} // namespace fil::mem
