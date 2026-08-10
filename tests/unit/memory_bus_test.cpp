#include "fil/elf/elf_loader.hpp"
#include "fil/mem/memory_bus.hpp"
#include "fil/mem/mcu_map.hpp"
#include "fil/mem/mmio_router.hpp"
#include "../test_support.hpp"

#include <cstdint>
#include <filesystem>
#include <string_view>
#include <vector>

namespace {

class RecordingMmio final : public fil::mem::MmioDevice {
public:
    /** @brief Records one MMIO read and returns a fixed pattern. */
    fil::mem::MemoryResult<std::uint64_t> read(
        const std::uint32_t offset,
        const fil::mem::AccessSize size,
        const fil::mem::AccessContext& context
    ) override {
        ++read_count;
        last_offset = offset;
        last_size = size;
        last_context = context;
        return std::uint64_t{0x8877665544332211ULL};
    }

    /** @brief Records one MMIO write. */
    fil::mem::MemoryResult<std::uint64_t> write(
        const std::uint32_t offset,
        const fil::mem::AccessSize size,
        const std::uint64_t value,
        const fil::mem::AccessContext& context
    ) override {
        ++write_count;
        last_offset = offset;
        last_size = size;
        last_value = value;
        last_context = context;
        return std::uint64_t{0};
    }

    /** @brief Gets the test device name. */
    std::string_view name() const noexcept override { return "recording-mmio"; }

    fil::mem::MmioDomain domain(
        std::uint32_t, fil::mem::AccessSize
    ) const noexcept override {
        return shared ? fil::mem::MmioDomain::shared : fil::mem::MmioDomain::board_local;
    }

    bool shared{false};
    int read_count{0};
    int write_count{0};
    std::uint32_t last_offset{0};
    fil::mem::AccessSize last_size{fil::mem::AccessSize::byte};
    std::uint64_t last_value{0};
    fil::mem::AccessContext last_context;
};

/** @brief Deterministic no-op MMIO window used by map-construction tests. */
class ZeroMmio final : public fil::mem::MmioDevice {
public:
    fil::mem::MemoryResult<std::uint64_t> read(
        std::uint32_t,
        fil::mem::AccessSize,
        const fil::mem::AccessContext&
    ) override { return std::uint64_t{0}; }
    fil::mem::MemoryResult<std::uint64_t> write(
        std::uint32_t,
        fil::mem::AccessSize,
        std::uint64_t,
        const fil::mem::AccessContext&
    ) override { return std::uint64_t{0}; }
    std::string_view name() const noexcept override { return "zero-mmio"; }
};

/** @brief Creates a representative flash/RAM/boot-alias address map. */
fil::mem::MemoryBus basicBus() {
    fil::mem::MemoryBus bus;
    fil::test::check(bus.mapRom(0x08000000U, 32, "flash").hasValue(), "maps flash");
    fil::test::check(bus.mapRam(0x20000000U, 32, "sram").hasValue(), "maps SRAM");
    fil::test::check(bus.mapAlias(0x00000000U, 0x08000000U, 32, "boot-alias").hasValue(), "maps boot alias");
    return bus;
}

/** @brief Verifies little-endian aligned, unaligned, and alias accesses. */
void readsAndWritesBackedMemory() {
    auto bus = basicBus();
    const std::vector<std::uint8_t> flash = {0x10, 0x21, 0x32, 0x43, 0x54, 0x65};
    fil::test::check(bus.loadBytes(0x08000000U, flash).hasValue(), "loads bytes into ROM during setup");

    const auto word = bus.read32(0x08000000U);
    const auto unaligned = bus.read32(0x08000001U);
    const auto alias = bus.read16(0x00000002U);
    fil::test::check(word && word.value() == 0x43322110U, "reads little-endian ROM word");
    fil::test::check(unaligned && unaligned.value() == 0x54433221U, "composes unaligned ROM word");
    fil::test::check(alias && alias.value() == 0x4332U, "translates boot alias reads");

    fil::test::check(bus.write32(0x20000001U, 0xa1b2c3d4U).hasValue(), "writes unaligned SRAM word");
    const auto ram = bus.read32(0x20000001U);
    fil::test::check(ram && ram.value() == 0xa1b2c3d4U, "reads back unaligned SRAM word");

    const auto rom_write = bus.write8(0x08000000U, 0);
    fil::test::check(
        !rom_write && rom_write.fault().reason == fil::mem::BusFaultReason::write_protected,
        "rejects CPU writes to ROM"
    );
}

/** @brief Verifies precise range and permission faults. */
void reportsStructuredFaults() {
    auto bus = basicBus();
    const auto unmapped = bus.read32(0x60000000U);
    const auto crossing = bus.read16(0x2000001fU);
    const auto overflow = bus.read32(0xfffffffeU);
    const auto execute_ram = bus.read16(
        0x20000000U,
        fil::mem::AccessContext{fil::mem::AccessType::instruction_fetch, 0x08000100U}
    );
    const auto execute_flash = bus.read16(
        0x08000000U,
        fil::mem::AccessContext{fil::mem::AccessType::instruction_fetch, 0x08000100U}
    );

    fil::test::check(!unmapped && unmapped.fault().reason == fil::mem::BusFaultReason::unmapped, "reports unmapped reads");
    fil::test::check(!crossing && crossing.fault().reason == fil::mem::BusFaultReason::cross_region, "reports cross-region accesses");
    fil::test::check(!overflow && overflow.fault().reason == fil::mem::BusFaultReason::address_overflow, "reports address overflow");
    fil::test::check(!execute_ram && execute_ram.fault().reason == fil::mem::BusFaultReason::execute_protected, "enforces execute permission");
    fil::test::check(execute_flash.hasValue(), "allows instruction fetch from executable flash");
    fil::test::check(
        !execute_ram && fil::mem::formatBusFault(execute_ram.fault()).find("pc=0x08000100") != std::string::npos,
        "fault formatting includes originating PC"
    );
}

/** @brief Verifies MMIO accesses retain width and dispatch exactly once. */
void dispatchesMmioIndivisibly() {
    fil::mem::MemoryBus bus;
    RecordingMmio device;
    fil::test::check(bus.mapMmio(0x40001000U, 16, device, "test-device").hasValue(), "maps MMIO device");

    const auto read = bus.read32(0x40001004U);
    fil::test::check(read && read.value() == 0x44332211U, "masks MMIO read to requested width");
    fil::test::check(device.read_count == 1, "dispatches one MMIO read");
    fil::test::check(device.last_offset == 4 && device.last_size == fil::mem::AccessSize::word, "preserves MMIO offset and width");

    const auto write = bus.write16(0x40001002U, 0xabcdU);
    fil::test::check(write.hasValue() && device.write_count == 1, "dispatches one MMIO write");
    fil::test::check(device.last_offset == 2 && device.last_value == 0xabcdU, "preserves MMIO write value");

    const auto crossing = bus.read32(0x4000100eU);
    fil::test::check(!crossing && device.read_count == 1, "rejects cross-boundary MMIO before dispatch");
}

/** @brief Verifies compact memory results preserve values and deep-copy faults. */
void copiesMemoryResultsSafely() {
    fil::mem::MemoryResult<std::uint32_t> success{0x12345678U};
    const auto copied_success = success;
    fil::test::check(copied_success && copied_success.value() == success.value(),
                     "copies successful memory results");

    fil::mem::BusFault fault;
    fault.reason = fil::mem::BusFaultReason::device_error;
    fault.address = 0x40000010U;
    fault.region = "device";
    fault.message = "rejected";
    fil::mem::MemoryResult<std::uint32_t> failure{fault};
    auto copied_failure = failure;
    copied_failure.fault().region = "copy";
    fil::test::check(!failure && !copied_failure
                         && failure.fault().region == "device"
                         && copied_failure.fault().region == "copy",
                     "deep-copies memory faults");

    success = failure;
    fil::test::check(!success
                         && success.fault().reason
                             == fil::mem::BusFaultReason::device_error,
                     "copy-assigns failed memory results");
}

/** @brief Verifies the region cache rechecks mappings that share one slot. */
void handlesRegionCacheCollisions() {
    fil::mem::MemoryBus bus;
    fil::test::check(bus.mapRam(0x20000000U, 16U, "low").hasValue()
                         && bus.mapRam(0x20ff0000U, 16U, "high").hasValue(),
                     "maps two regions in one dispatch-cache slot");
    static_cast<void>(bus.write32(0x20000000U, 0x11223344U));
    static_cast<void>(bus.write32(0x20ff0000U, 0xaabbccddU));

    bool values_match = true;
    for (unsigned int iteration = 0; iteration < 4U; ++iteration) {
        const auto low = bus.read32(0x20000000U);
        const auto high = bus.read32(0x20ff0000U);
        values_match = values_match
            && low && low.value() == 0x11223344U
            && high && high.value() == 0xaabbccddU;
    }
    fil::test::check(values_match,
                     "cache collisions fall back to the authoritative region map");
}

/** @brief Verifies map-time overlap, wraparound, and alias validation. */
void validatesRegionMaps() {
    fil::mem::MemoryBus bus;
    fil::test::check(bus.mapRam(0x20000000U, 32, "ram").hasValue(), "maps initial region");
    fil::test::check(!bus.mapRom(0x20000010U, 32, "overlap"), "rejects overlapping regions");
    fil::test::check(!bus.mapRam(0xfffffff0U, 32, "wrapping"), "rejects wrapping regions");
    fil::test::check(!bus.mapAlias(0, 0x30000000U, 16, "bad-alias"), "rejects aliases to unmapped targets");
}

/** @brief Verifies exact-loop side-effect checkpoints accept only restored RAM. */
void tracksReversibleLoopMemoryEffects() {
    fil::mem::MemoryBus bus;
    RecordingMmio device;
    fil::test::check(bus.mapRam(0x20000000U, 16U, "ram").hasValue()
                         && bus.mapMmio(0x40000000U, 16U, device, "device").hasValue(),
                     "maps checkpoint test regions");

    const auto restored_checkpoint = bus.sideEffectCheckpoint();
    static_cast<void>(bus.write8(0x20000000U, 0x5aU));
    static_cast<void>(bus.write8(0x20000000U, 0x00U));
    fil::test::check(bus.sideEffectsRestoredSince(restored_checkpoint),
                     "accepts temporary RAM changes restored to checkpoint bytes");

    const auto idempotent_checkpoint = bus.sideEffectCheckpoint();
    static_cast<void>(bus.write32(0x20000004U, 0U));
    fil::test::check(bus.sideEffectsRestoredSince(idempotent_checkpoint),
                     "accepts idempotent backed-memory writes");

    const auto rollback_checkpoint = bus.sideEffectCheckpoint();
    static_cast<void>(bus.write8(0x20000008U, 0x11U));
    static_cast<void>(bus.write8(0x20000008U, 0x22U));
    fil::test::check(bus.restoreSideEffects(rollback_checkpoint),
                     "reverses speculative backed-memory mutations");
    const auto rolled_back = bus.read8(0x20000008U);
    fil::test::check(rolled_back && rolled_back.value() == 0U
                         && bus.sideEffectsRestoredSince(rollback_checkpoint),
                     "rollback restores bytes and checkpoint sequence");

    static_cast<void>(bus.read32(
        0x20000000U, {fil::mem::AccessType::data_read, 0x08000100U}
    ));
    const auto footprint = bus.takeReadFootprint();
    const auto external_checkpoint = bus.sideEffectCheckpoint();
    static_cast<void>(bus.write8(
        0x2000000cU, 0x33U, {fil::mem::AccessType::data_write, 0U}
    ));
    fil::test::check(bus.sideEffectsCompatibleSince(external_checkpoint, footprint),
                     "ignores external writes outside a proven loop read footprint");
    static_cast<void>(bus.write8(
        0x20000000U, 0x44U, {fil::mem::AccessType::data_write, 0U}
    ));
    fil::test::check(!bus.sideEffectsCompatibleSince(external_checkpoint, footprint),
                     "invalidates a proof when external DMA overlaps a loop read");

    const auto mmio_checkpoint = bus.sideEffectCheckpoint();
    static_cast<void>(bus.read32(0x40000000U));
    fil::test::check(!bus.sideEffectsRestoredSince(mmio_checkpoint)
                         && !bus.mmioUnchangedSince(mmio_checkpoint)
                         && !bus.restoreSideEffects(mmio_checkpoint),
                     "rejects rollback after an MMIO access");

    const auto expired_checkpoint = bus.sideEffectCheckpoint();
    for (std::size_t index = 0; index < 8193U; ++index) {
        static_cast<void>(bus.write8(
            0x2000000fU, static_cast<std::uint8_t>((index & 1U) + 1U)
        ));
    }
    fil::test::check(!bus.canRestoreSideEffects(expired_checkpoint)
                         && !bus.sideEffectsRestoredSince(expired_checkpoint)
                         && !bus.restoreSideEffects(expired_checkpoint),
                     "fails closed when a checkpoint exceeds the mutation journal");
}

void trapsSharedMmioBeforeDeviceSideEffects() {
    fil::mem::MemoryBus bus;
    fil::mem::MmioRouter router(0x40000000U, 0x1000U, "peripherals");
    RecordingMmio shared;
    shared.shared = true;
    fil::test::check(router.map(0x100U, 0x20U, shared, "shared-child").hasValue()
                         && bus.mapMmio(0x40000000U, 0x1000U, router,
                                        "peripherals").hasValue(),
                     "maps shared MMIO through a router");

    bus.setSharedMmioTrapping(true);
    const auto checkpoint = bus.sideEffectCheckpoint();
    const auto trapped = bus.read32(
        0x40000100U, {fil::mem::AccessType::data_read, 0x08000100U}
    );
    fil::test::check(!trapped
                         && trapped.fault().reason
                             == fil::mem::BusFaultReason::synchronization_required
                         && shared.read_count == 0
                         && bus.sideEffectsRestoredSince(checkpoint),
                     "shared MMIO trap stops before device or journal side effects");

    bus.setSharedMmioTrapping(false);
    const auto committed = bus.read32(0x40000100U);
    fil::test::check(committed && shared.read_count == 1,
                     "coordinator can disable trapping and commit shared MMIO");
}

/** @brief Verifies sub-device routing and aggregate lenient unknown handling. */
void routesMmioWindows() {
    fil::mem::MemoryBus bus;
    fil::mem::MmioRouter router(0x40000000U, 0x1000U, "peripherals");
    RecordingMmio device;
    fil::test::check(router.map(0x100U, 0x20U, device, "child").hasValue(), "maps MMIO child route");
    fil::test::check(!router.map(0x110U, 0x20U, device, "overlap"), "rejects overlapping MMIO child route");
    fil::test::check(bus.mapMmio(0x40000000U, 0x1000U, router, "peripherals").hasValue(), "maps routed MMIO window");

    const auto routed = bus.read32(0x40000104U);
    const auto unknown_read = bus.read32(0x40000200U);
    const auto unknown_write = bus.write16(0x40000200U, 0x1234U);
    fil::test::check(routed && device.last_offset == 4U, "routes access with child-relative offset");
    fil::test::check(unknown_read && unknown_read.value() == 0U && unknown_write, "handles unknown MMIO leniently");
    const auto unknown = router.unknownAccesses();
    fil::test::check(
        unknown.size() == 1 && unknown[0].address == 0x40000200U
            && unknown[0].reads == 1 && unknown[0].writes == 1,
        "aggregates unknown MMIO counts by absolute address"
    );

    router.setLenient(false);
    const auto strict = bus.read32(0x40000300U);
    fil::test::check(
        !strict && strict.fault().reason == fil::mem::BusFaultReason::device_error,
        "faults on unknown MMIO in strict mode"
    );
}

/** @brief Verifies ELF bytes load at physical addresses without initializing runtime SRAM. */
void materializesElfLoadImage() {
    const auto fixture = std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";
    const auto image = fil::elf::load(fixture);
    fil::test::check(image.hasValue(), "loads ELF fixture for memory materialization");
    if (!image) return;

    fil::mem::MemoryBus bus;
    fil::test::check(bus.mapRom(0x08000000U, 64, "flash").hasValue(), "maps fixture flash");
    fil::test::check(bus.mapRam(0x20000000U, 64, "sram").hasValue(), "maps fixture SRAM");
    fil::test::check(bus.materialize(image.value()).hasValue(), "materializes ELF image");

    const auto vector = bus.read32(0x08000000U);
    const auto initializer = bus.read32(0x08000018U);
    const auto runtime_data = bus.read32(0x20000000U);
    const auto erased = bus.read8(0x08000030U);
    fil::test::check(vector && vector.value() == 0x20020000U, "loads vector bytes into flash");
    fil::test::check(initializer && initializer.value() == 0x12345678U, "loads .data initializer at physical flash address");
    fil::test::check(runtime_data && runtime_data.value() == 0, "leaves .data runtime SRAM at reset value");
    fil::test::check(erased && erased.value() == 0xffU, "leaves unloaded ROM bytes erased");
}

/** @brief Verifies config-driven STM32 map construction and reset validation. */
void buildsMcuResetMap() {
    const auto fixture = std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";
    const auto image = fil::elf::load(fixture);
    if (!image) {
        fil::test::check(false, "loads ELF fixture for MCU map");
        return;
    }
    const fil::config::McuConfig mcu{
        1, "fixture-mcu", 0x08000000U, 0x1000U,
        0x20000000U, 0x20000U, 0x10000000U, 0x8000U,
    };
    ZeroMmio peripherals;
    ZeroMmio system;
    auto memory = fil::mem::buildMcuMemoryMap(
        mcu, image.value(), {peripherals, system}
    );
    fil::test::check(memory.hasValue(), "builds standard config-driven MCU memory map");
    if (!memory) return;
    const auto regions = memory.value().regions();
    fil::test::check(regions.size() == 7, "maps all standard memory windows");
    const auto aliased_vector = memory.value().read32(0x00000000U);
    fil::test::check(
        aliased_vector && aliased_vector.value() == image.value().initialMsp(),
        "boot alias exposes loaded vector table"
    );
}

} // namespace

/** @brief Runs all memory bus unit-test cases. */
void runMemoryBusTests() {
    readsAndWritesBackedMemory();
    reportsStructuredFaults();
    dispatchesMmioIndivisibly();
    copiesMemoryResultsSafely();
    handlesRegionCacheCollisions();
    validatesRegionMaps();
    tracksReversibleLoopMemoryEffects();
    trapsSharedMmioBeforeDeviceSideEffects();
    routesMmioWindows();
    materializesElfLoadImage();
    buildsMcuResetMap();
}
