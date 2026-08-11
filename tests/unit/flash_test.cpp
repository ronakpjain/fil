#include "fil/cortexm/system_control.hpp"
#include "fil/mem/memory_bus.hpp"
#include "fil/stm32g4/peripheral.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace {

constexpr fil::mem::AccessContext read_context{fil::mem::AccessType::data_read, 0};
constexpr fil::mem::AccessContext write_context{fil::mem::AccessType::data_write, 0};

constexpr std::uint32_t flash_keyr = 0x08;
constexpr std::uint32_t flash_sr = 0x10;
constexpr std::uint32_t flash_cr = 0x14;

std::vector<std::uint8_t> erasedPage(const std::uint32_t size) {
    return std::vector<std::uint8_t>(size, 0xffU);
}

/** @brief Wires a FLASH peripheral to a memory backing exactly like Stm32G4 does. */
void attachEraseToMemory(
    fil::stm32g4::FlashPeripheral& flash,
    fil::mem::MemoryBus& memory,
    const std::uint32_t flash_base = 0x08000000U,
    const std::uint32_t page_size = 4096U,
    const std::uint32_t bank_size = 512U * 1024U
) {
    flash.setEraseGeometry(flash_base, page_size, bank_size);
    flash.setPageEraseCallback(
        [&memory](const std::uint32_t page_base, const std::uint32_t page_size) {
            return memory.loadBytes(page_base, erasedPage(page_size));
        }
    );
}

void unlockFlash(fil::stm32g4::FlashPeripheral& flash) {
    ASSERT_TRUE(flash.write(flash_keyr, fil::mem::AccessSize::word, 0x45670123U, write_context)
            .hasValue());
    ASSERT_TRUE(flash.write(flash_keyr, fil::mem::AccessSize::word, 0xcdef89abU, write_context)
            .hasValue());
}

/** @brief Verifies mapped flash is erased-initialized, writable, and executable. */
TEST(FlashRegionTest, MappedFlashIsWritableAndExecutable) {
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapFlash(0x08000000U, 16U * 1024U, "flash", true).hasValue())
        << "maps writable flash";

    const auto erased = memory.read32(0x08000000U);
    EXPECT_TRUE(erased && erased.value() == 0xffffffffU)
        << "flash backing initializes to erased bytes";
    const auto region = memory.regions();
    const auto flash = std::find_if(region.begin(), region.end(), [](const auto& info) {
        return info.name == "flash";
    });
    ASSERT_TRUE(flash != region.end());
    EXPECT_TRUE(flash->writable && flash->executable)
        << "flash region is writable and executable";

    EXPECT_TRUE(memory.write32(0x08000000U, 0xdeadbeefU).hasValue())
        << "firmware programming writes flash words";
    const auto programmed = memory.read32(0x08000000U);
    EXPECT_TRUE(programmed && programmed.value() == 0xdeadbeefU)
        << "reads back programmed words";
    const auto fetched = memory.read16(
        0x08000000U,
        fil::mem::AccessContext{fil::mem::AccessType::instruction_fetch, 0x08000000U}
    );
    EXPECT_TRUE(fetched && fetched.value() == 0xbeefU)
        << "instruction fetches read programmed flash";
}

/** @brief Verifies the FLASH CR PER/PNB/BKER/STRT sequence erases one mapped page. */
TEST(FlashPeripheralTest, ErasesSelectedPageThroughMappedBacking) {
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapFlash(0x08000000U, 16U * 1024U, "flash", true).hasValue());

    fil::stm32g4::FlashPeripheral flash;
    attachEraseToMemory(flash, memory, 0x08000000U, 4096U, 16U * 1024U);

    // Program page 0, 1, and 2 with distinct words.
    ASSERT_TRUE(memory.write32(0x08000000U, 0x11111111U).hasValue());
    ASSERT_TRUE(memory.write32(0x08001000U, 0x22222222U).hasValue());
    ASSERT_TRUE(memory.write32(0x08002000U, 0x33333333U).hasValue());

    unlockFlash(flash);

    // Erase page 2 (PNB = 2): CR = PER | (2 << 3) | STRT.
    constexpr std::uint32_t per = 1U << 1U;
    constexpr std::uint32_t strt = 1U << 16U;
    EXPECT_TRUE(
        flash.write(flash_cr, fil::mem::AccessSize::word, per | (2U << 3U) | strt, write_context)
            .hasValue())
        << "starts a page erase through FLASH_CR";

    const auto status = flash.read(flash_sr, fil::mem::AccessSize::word, read_context);
    ASSERT_TRUE(status.hasValue());
    EXPECT_TRUE((status.value() & (1U << 0U)) != 0U)
        << "FLASH_SR reports EOP after the erase";
    EXPECT_TRUE((status.value() & (1U << 16U)) == 0U)
        << "FLASH_SR reports BSY clear so PHAL_FLASH_erase returns success";

    const auto erased = memory.read32(0x08002000U);
    EXPECT_TRUE(erased && erased.value() == 0xffffffffU) << "selected page is erased";
    const auto page0 = memory.read32(0x08000000U);
    EXPECT_TRUE(page0 && page0.value() == 0x11111111U) << "other pages retain program data";
    const auto page1 = memory.read32(0x08001000U);
    EXPECT_TRUE(page1 && page1.value() == 0x22222222U) << "page 1 retains program data";
}

/** @brief Verifies erased flash can be reprogrammed after an erase. */
TEST(FlashPeripheralTest, ReprogramsErasedPages) {
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapFlash(0x08000000U, 8U * 1024U, "flash", true).hasValue());

    fil::stm32g4::FlashPeripheral flash;
    attachEraseToMemory(flash, memory, 0x08000000U, 4096U, 8U * 1024U);
    unlockFlash(flash);

    constexpr std::uint32_t per = 1U << 1U;
    constexpr std::uint32_t strt = 1U << 16U;
    ASSERT_TRUE(memory.write32(0x08000000U, 0xcafebabeU).hasValue());
    EXPECT_TRUE(
        flash.write(flash_cr, fil::mem::AccessSize::word, per | strt, write_context).hasValue())
        << "erases page 0";
    const auto after_erase = memory.read32(0x08000000U);
    ASSERT_TRUE(after_erase && after_erase.value() == 0xffffffffU);

    EXPECT_TRUE(memory.write32(0x08000000U, 0xfeedfaceU).hasValue())
        << "reprograms the erased page";
    const auto reprogrammed = memory.read32(0x08000000U);
    EXPECT_TRUE(reprogrammed && reprogrammed.value() == 0xfeedfaceU)
        << "read back reprogrammed flash";
}

/** @brief Verifies the erase sequence is ignored while FLASH_CR stays locked. */
TEST(FlashPeripheralTest, IgnoresEraseWhileLocked) {
    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapFlash(0x08000000U, 8U * 1024U, "flash", true).hasValue());

    fil::stm32g4::FlashPeripheral flash;
    attachEraseToMemory(flash, memory, 0x08000000U, 4096U, 8U * 1024U);
    ASSERT_TRUE(memory.write32(0x08000000U, 0x12345678U).hasValue());

    constexpr std::uint32_t per = 1U << 1U;
    constexpr std::uint32_t strt = 1U << 16U;
    EXPECT_TRUE(
        flash.write(flash_cr, fil::mem::AccessSize::word, per | strt, write_context).hasValue());
    const auto control = flash.read(flash_cr, fil::mem::AccessSize::word, read_context);
    ASSERT_TRUE(control.hasValue());
    EXPECT_TRUE((control.value() & (1U << 31U)) != 0U)
        << "locked CR rejects erase control writes";
    const auto preserved = memory.read32(0x08000000U);
    EXPECT_TRUE(preserved && preserved.value() == 0x12345678U)
        << "locked flash is not erased";
}

/** @brief Verifies the integrated STM32G4 wires flash erase to memory. */
TEST(FlashIntegrationTest, RoutesEraseThroughStm32G4) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true, 16'000'000U);
    ASSERT_TRUE(mcu.hasValue()) << "constructs the integrated STM32G4 peripheral map";

    fil::mem::MemoryBus memory;
    ASSERT_TRUE(memory.mapFlash(0x08000000U, 512U * 1024U, "flash", true).hasValue())
        << "maps a full-size STM32G474 flash";
    mcu.value()->attachMemory(memory);

    // Unlock FLASH through the routed KEYR.
    EXPECT_TRUE(
        mcu.value()->router()
            .write(0x00022008U, fil::mem::AccessSize::word, 0x45670123U, write_context)
            .hasValue());
    EXPECT_TRUE(
        mcu.value()->router()
            .write(0x00022008U, fil::mem::AccessSize::word, 0xcdef89abU, write_context)
            .hasValue());

    // Program and erase page 0 through the routed FLASH_CR.
    ASSERT_TRUE(memory.write32(0x08000000U, 0x5a5a5a5aU).hasValue());
    constexpr std::uint32_t per = 1U << 1U;
    constexpr std::uint32_t strt = 1U << 16U;
    EXPECT_TRUE(
        mcu.value()->router()
            .write(0x00022014U, fil::mem::AccessSize::word, per | strt, write_context)
            .hasValue())
        << "routes a page-erase start";
    const auto status =
        mcu.value()->router().read(0x00022010U, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(status && (status.value() & (1U << 0U)) != 0U)
        << "integrated FLASH_SR reports EOP";
    const auto page = memory.read32(0x08000000U);
    EXPECT_TRUE(page && page.value() == 0xffffffffU)
        << "integrated erase clears the mapped flash page";
}

} // namespace
