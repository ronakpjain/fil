#include "fil/elf/elf_loader.hpp"
#include "fil/mem/memory_bus.hpp"
#include "fil/mem/mcu_map.hpp"
#include "fil/mem/mmio_router.hpp"

#include <gtest/gtest.h>

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
    EXPECT_TRUE(bus.mapRom(0x08000000U, 32, "flash").hasValue()) << "maps flash";
    EXPECT_TRUE(bus.mapRam(0x20000000U, 32, "sram").hasValue()) << "maps SRAM";
    EXPECT_TRUE(bus.mapAlias(0x00000000U, 0x08000000U, 32, "boot-alias").hasValue())
        << "maps boot alias";
    return bus;
}

/** @brief Verifies little-endian aligned, unaligned, and alias accesses. */
TEST(MemoryBusTest, ReadsAndWritesBackedMemory) {
    auto bus = basicBus();
    const std::vector<std::uint8_t> flash = {0x10, 0x21, 0x32, 0x43, 0x54, 0x65};
    EXPECT_TRUE(bus.loadBytes(0x08000000U, flash).hasValue())
        << "loads bytes into ROM during setup";

    const auto word = bus.read32(0x08000000U);
    const auto unaligned = bus.read32(0x08000001U);
    const auto alias = bus.read16(0x00000002U);
    EXPECT_TRUE(word && word.value() == 0x43322110U) << "reads little-endian ROM word";
    EXPECT_TRUE(unaligned && unaligned.value() == 0x54433221U) << "composes unaligned ROM word";
    EXPECT_TRUE(alias && alias.value() == 0x4332U) << "translates boot alias reads";

    EXPECT_TRUE(bus.write32(0x20000001U, 0xa1b2c3d4U).hasValue()) << "writes unaligned SRAM word";
    const auto ram = bus.read32(0x20000001U);
    EXPECT_TRUE(ram && ram.value() == 0xa1b2c3d4U) << "reads back unaligned SRAM word";

    const auto rom_write = bus.write8(0x08000000U, 0);
    EXPECT_TRUE(!rom_write && rom_write.fault().reason == fil::mem::BusFaultReason::write_protected)
        << "rejects CPU writes to ROM";
}

/** @brief Verifies precise range and permission faults. */
TEST(MemoryBusTest, ReportsStructuredFaults) {
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

    EXPECT_TRUE(!unmapped && unmapped.fault().reason == fil::mem::BusFaultReason::unmapped)
        << "reports unmapped reads";
    EXPECT_TRUE(!crossing && crossing.fault().reason == fil::mem::BusFaultReason::cross_region)
        << "reports cross-region accesses";
    EXPECT_TRUE(!overflow && overflow.fault().reason == fil::mem::BusFaultReason::address_overflow)
        << "reports address overflow";
    EXPECT_TRUE(
        !execute_ram && execute_ram.fault().reason == fil::mem::BusFaultReason::execute_protected)
        << "enforces execute permission";
    EXPECT_TRUE(execute_flash.hasValue()) << "allows instruction fetch from executable flash";
    EXPECT_TRUE(
        !execute_ram &&
        fil::mem::formatBusFault(execute_ram.fault()).find("pc=0x08000100") != std::string::npos)
        << "fault formatting includes originating PC";
}

/** @brief Verifies MMIO accesses retain width and dispatch exactly once. */
TEST(MemoryBusTest, DispatchesMmioIndivisibly) {
    fil::mem::MemoryBus bus;
    RecordingMmio device;
    EXPECT_TRUE(bus.mapMmio(0x40001000U, 16, device, "test-device").hasValue())
        << "maps MMIO device";

    const auto read = bus.read32(0x40001004U);
    EXPECT_TRUE(read && read.value() == 0x44332211U) << "masks MMIO read to requested width";
    EXPECT_TRUE(device.read_count == 1) << "dispatches one MMIO read";
    EXPECT_TRUE(device.last_offset == 4 && device.last_size == fil::mem::AccessSize::word)
        << "preserves MMIO offset and width";

    const auto write = bus.write16(0x40001002U, 0xabcdU);
    EXPECT_TRUE(write.hasValue() && device.write_count == 1) << "dispatches one MMIO write";
    EXPECT_TRUE(device.last_offset == 2 && device.last_value == 0xabcdU)
        << "preserves MMIO write value";

    const auto crossing = bus.read32(0x4000100eU);
    EXPECT_TRUE(!crossing && device.read_count == 1)
        << "rejects cross-boundary MMIO before dispatch";
}

/** @brief Verifies explicit little-endian byte decoding for every access width. */
TEST(MemoryBusTest, DecodesLittleEndianBytesAtEveryWidth) {
    constexpr std::uint8_t bytes[] = {0x10U, 0x32U, 0x54U, 0x76U,
                                      0x98U, 0xbaU, 0xdcU, 0xfeU};
    EXPECT_TRUE(fil::mem::detail::decodeLittleEndianBytes(bytes, 1U) == 0x10U);
    EXPECT_TRUE(fil::mem::detail::decodeLittleEndianBytes(bytes, 2U) == 0x3210U);
    EXPECT_TRUE(fil::mem::detail::decodeLittleEndianBytes(bytes, 4U) == 0x76543210U);
    EXPECT_TRUE(fil::mem::detail::decodeLittleEndianBytes(bytes, 8U) == 0xfedcba9876543210ULL);
}

/** @brief Verifies shared access-width helpers cover every MMIO width. */
TEST(MemoryBusTest, ValidatesAccessWidthUtilities) {
    using fil::mem::AccessSize;
    EXPECT_TRUE(fil::mem::validAccessSize(AccessSize::byte) &&
                fil::mem::validAccessSize(AccessSize::halfword) &&
                fil::mem::validAccessSize(AccessSize::word) &&
                fil::mem::validAccessSize(AccessSize::doubleword) &&
                !fil::mem::validAccessSize(static_cast<AccessSize>(3U)))
        << "recognizes only supported access widths";
    EXPECT_TRUE(fil::mem::accessWidthMask(AccessSize::byte) == 0xffU &&
                fil::mem::accessWidthMask(AccessSize::halfword) == 0xffffU &&
                fil::mem::accessWidthMask(AccessSize::word) == 0xffffffffU &&
                fil::mem::accessWidthMask(AccessSize::doubleword) == 0xffffffffffffffffULL)
        << "builds width masks without full-width shifts";
}

/** @brief Verifies compact memory results preserve values and deep-copy faults. */
TEST(MemoryBusTest, CopiesMemoryResultsSafely) {
    fil::mem::MemoryResult<std::uint32_t> success{0x12345678U};
    const auto copied_success = success;
    EXPECT_TRUE(copied_success && copied_success.value() == success.value())
        << "copies successful memory results";

    fil::mem::BusFault fault;
    fault.reason = fil::mem::BusFaultReason::device_error;
    fault.address = 0x40000010U;
    fault.region = "device";
    fault.message = "rejected";
    fil::mem::MemoryResult<std::uint32_t> failure{fault};
    auto copied_failure = failure;
    copied_failure.fault().region = "copy";
    EXPECT_TRUE(!failure && !copied_failure && failure.fault().region == "device" &&
                copied_failure.fault().region == "copy")
        << "deep-copies memory faults";

    success = failure;
    EXPECT_TRUE(!success && success.fault().reason == fil::mem::BusFaultReason::device_error)
        << "copy-assigns failed memory results";
}

/** @brief Verifies the region cache rechecks mappings that share one slot. */
TEST(MemoryBusTest, HandlesRegionCacheCollisions) {
    fil::mem::MemoryBus bus;
    EXPECT_TRUE(bus.mapRam(0x20000000U, 16U, "low").hasValue() &&
                bus.mapRam(0x20ff0000U, 16U, "high").hasValue())
        << "maps two regions in one dispatch-cache slot";
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
    EXPECT_TRUE(values_match) << "cache collisions fall back to the authoritative region map";
}

/** @brief Verifies map-time overlap, wraparound, and alias validation. */
TEST(MemoryBusTest, ValidatesRegionMaps) {
    fil::mem::MemoryBus bus;
    EXPECT_TRUE(bus.mapRam(0x20000000U, 32, "ram").hasValue()) << "maps initial region";
    EXPECT_TRUE(!bus.mapRom(0x20000010U, 32, "overlap")) << "rejects overlapping regions";
    EXPECT_TRUE(!bus.mapRam(0xfffffff0U, 32, "wrapping")) << "rejects wrapping regions";
    EXPECT_TRUE(!bus.mapAlias(0, 0x30000000U, 16, "bad-alias"))
        << "rejects aliases to unmapped targets";
}

/** @brief Verifies exact-loop side-effect checkpoints accept only restored RAM. */
TEST(MemoryBusTest, TracksReversibleLoopMemoryEffects) {
    fil::mem::MemoryBus bus;
    RecordingMmio device;
    EXPECT_TRUE(bus.mapRam(0x20000000U, 16U, "ram").hasValue() &&
                bus.mapMmio(0x40000000U, 16U, device, "device").hasValue())
        << "maps checkpoint test regions";

    const auto restored_checkpoint = bus.sideEffectCheckpoint();
    static_cast<void>(bus.write8(0x20000000U, 0x5aU));
    static_cast<void>(bus.write8(0x20000000U, 0x00U));
    EXPECT_TRUE(bus.sideEffectsRestoredSince(restored_checkpoint))
        << "accepts temporary RAM changes restored to checkpoint bytes";

    const auto idempotent_checkpoint = bus.sideEffectCheckpoint();
    static_cast<void>(bus.write32(0x20000004U, 0U));
    EXPECT_TRUE(bus.sideEffectsRestoredSince(idempotent_checkpoint))
        << "accepts idempotent backed-memory writes";

    const auto rollback_checkpoint = bus.sideEffectCheckpoint();
    static_cast<void>(bus.write8(0x20000008U, 0x11U));
    static_cast<void>(bus.write8(0x20000008U, 0x22U));
    EXPECT_TRUE(bus.restoreSideEffects(rollback_checkpoint))
        << "reverses speculative backed-memory mutations";
    const auto rolled_back = bus.read8(0x20000008U);
    EXPECT_TRUE(rolled_back && rolled_back.value() == 0U &&
                bus.sideEffectsRestoredSince(rollback_checkpoint))
        << "rollback restores bytes and checkpoint sequence";

    static_cast<void>(bus.read32(
        0x20000000U, {fil::mem::AccessType::data_read, 0x08000100U}
    ));
    const auto footprint = bus.takeReadFootprint();
    const auto external_checkpoint = bus.sideEffectCheckpoint();
    static_cast<void>(bus.write8(
        0x2000000cU, 0x33U, {fil::mem::AccessType::data_write, 0U}
    ));
    EXPECT_TRUE(bus.sideEffectsCompatibleSince(external_checkpoint, footprint))
        << "ignores external writes outside a proven loop read footprint";
    static_cast<void>(bus.write8(
        0x20000000U, 0x44U, {fil::mem::AccessType::data_write, 0U}
    ));
    EXPECT_TRUE(!bus.sideEffectsCompatibleSince(external_checkpoint, footprint))
        << "invalidates a proof when external DMA overlaps a loop read";

    const auto mmio_checkpoint = bus.sideEffectCheckpoint();
    static_cast<void>(bus.read32(0x40000000U));
    EXPECT_TRUE(!bus.sideEffectsRestoredSince(mmio_checkpoint) &&
                !bus.mmioUnchangedSince(mmio_checkpoint) &&
                !bus.restoreSideEffects(mmio_checkpoint))
        << "rejects rollback after an MMIO access";

    const auto expired_checkpoint = bus.sideEffectCheckpoint();
    for (std::size_t index = 0; index < 8193U; ++index) {
        static_cast<void>(bus.write8(
            0x2000000fU, static_cast<std::uint8_t>((index & 1U) + 1U)
        ));
    }
    EXPECT_TRUE(!bus.canRestoreSideEffects(expired_checkpoint) &&
                !bus.sideEffectsRestoredSince(expired_checkpoint) &&
                !bus.restoreSideEffects(expired_checkpoint))
        << "fails closed when a checkpoint exceeds the mutation journal";
}

TEST(MemoryBusTest, OptionalTrackingPreservesArchitecturalMemoryAndFailsClosed) {
    using fil::mem::AccessContext;
    using fil::mem::AccessType;
    using fil::mem::MemoryBus;

    MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 32U, "ram").hasValue());
    ASSERT_TRUE(bus.mapRam(0x20000100U, 32U, "executable", true).hasValue());
    RecordingMmio device;
    ASSERT_TRUE(bus.mapMmio(0x40000000U, 16U, device, "device").hasValue());
    ASSERT_TRUE(bus.write32(0x20000000U, 0x12345678U));
    ASSERT_TRUE(bus.write32(0x20000100U, 0xabcdef01U));

    const auto execution_before = bus.executionGeneration();
    const auto effects_before = bus.sideEffectGeneration();
    bus.setReadFootprintTracking(false);
    const auto data = bus.read32(0x20000000U, {AccessType::data_read, 0x08000100U});
    const auto instruction = bus.read16(
        0x20000100U, {AccessType::instruction_fetch, 0x08000102U});
    const auto fault = bus.read32(0x60000000U, {AccessType::data_read, 0x08000104U});
    ASSERT_TRUE(data && instruction && !fault);
    EXPECT_EQ(data.value(), 0x12345678U);
    EXPECT_EQ(instruction.value(), 0xabcdef01U & 0xffffU);
    EXPECT_EQ(fault.fault().reason, fil::mem::BusFaultReason::unmapped);
    const auto missed_reads = bus.takeReadFootprint();
    EXPECT_FALSE(missed_reads.complete);
    EXPECT_EQ(missed_reads.words, MemoryBus::ReadFootprint{}.words);
    EXPECT_EQ(bus.executionGeneration(), execution_before);
    EXPECT_EQ(bus.sideEffectGeneration(), effects_before);

    bus.setReadFootprintTracking(true);
    static_cast<void>(bus.read32(0x20000000U, {AccessType::data_read, 0x08000100U}));
    const auto complete_reads = bus.takeReadFootprint();
    EXPECT_TRUE(complete_reads.complete);
    EXPECT_NE(complete_reads.words, MemoryBus::ReadFootprint{}.words);

    // A checkpoint spanning disabled journaling cannot restore or certify a
    // write, even after tracking is re-enabled.
    const auto checkpoint = bus.sideEffectCheckpoint();
    const auto footprint = complete_reads;
    bus.setWriteJournalTracking(false);
    const auto before_ram_write = bus.sideEffectGeneration();
    const auto before_exec_write = bus.executionGeneration();
    ASSERT_TRUE(bus.write32(0x20000000U, 0x87654321U));
    ASSERT_TRUE(bus.tryFastWriteWords(0x20000100U, 1U,
        std::array<std::uint32_t, 1>{0x10203040U}.data(), {}));
    EXPECT_EQ(bus.sideEffectGeneration(), before_ram_write + 8U);
    EXPECT_EQ(bus.executionGeneration(), before_exec_write + 1U);
    EXPECT_EQ(bus.read32(0x20000000U).value(), 0x87654321U);
    EXPECT_FALSE(bus.canRestoreSideEffects(checkpoint));
    EXPECT_FALSE(bus.restoreSideEffects(checkpoint));
    EXPECT_FALSE(bus.sideEffectsRestoredSince(checkpoint));
    EXPECT_FALSE(bus.sideEffectsCompatibleSince(checkpoint, footprint));

    bus.setWriteJournalTracking(true);
    EXPECT_FALSE(bus.canRestoreSideEffects(checkpoint));
    EXPECT_EQ(bus.read32(0x20000100U).value(), 0x10203040U);
    EXPECT_EQ(bus.executionGeneration(), before_exec_write + 1U);

    // A read made while tracking is disabled marks the resulting footprint
    // incomplete; such a footprint cannot approve overlapping external writes.
    bus.setReadFootprintTracking(false);
    static_cast<void>(bus.read32(0x20000000U, {AccessType::data_read, 0x08000100U}));
    const auto incomplete = bus.takeReadFootprint();
    bus.setReadFootprintTracking(true);
    const auto external_checkpoint = bus.sideEffectCheckpoint();
    ASSERT_TRUE(bus.write8(0x20000000U, 0x55U, {AccessType::data_write, 0U}));
    EXPECT_FALSE(bus.sideEffectsCompatibleSince(external_checkpoint, incomplete));
}

TEST(MemoryBusTest, UnjournaledWordStoresMatchBytewiseArchitecturalEffects) {
    using fil::mem::AccessSize;
    using fil::mem::MemoryBus;

    MemoryBus tracked;
    MemoryBus unjournaled;
    ASSERT_TRUE(tracked.mapRam(0x20000000U, 32U, "ram").hasValue());
    ASSERT_TRUE(unjournaled.mapRam(0x20000000U, 32U, "ram").hasValue());
    ASSERT_TRUE(tracked.mapRam(0x20000100U, 32U, "exec", true).hasValue());
    ASSERT_TRUE(unjournaled.mapRam(0x20000100U, 32U, "exec", true).hasValue());
    ASSERT_TRUE(tracked.write32(0x20000000U, 0x11223344U));
    ASSERT_TRUE(unjournaled.write32(0x20000000U, 0x11223344U));
    unjournaled.setWriteJournalTracking(false);

    // Exercise 0..4 changed bytes through both the dedicated 32-bit and
    // width-generic word fast paths, including unaligned backing addresses.
    for (std::uint32_t changed = 0U; changed <= 4U; ++changed) {
        const std::uint32_t old_value = tracked.read32(0x20000001U).value();
        std::uint32_t new_value = old_value;
        for (std::uint32_t byte = 0U; byte < changed; ++byte) {
            new_value ^= 0xffU << (byte * 8U);
        }
        const auto tracked_checkpoint = tracked.sideEffectCheckpoint();
        const auto fast_checkpoint = unjournaled.sideEffectCheckpoint();
        const auto tracked_effects = tracked.sideEffectGeneration();
        const auto fast_effects = unjournaled.sideEffectGeneration();
        const auto tracked_exec = tracked.executionGeneration();
        const auto fast_exec = unjournaled.executionGeneration();
        if ((changed & 1U) == 0U) {
            ASSERT_TRUE(tracked.tryFastWrite32(0x20000001U, new_value, {}));
            ASSERT_TRUE(unjournaled.tryFastWrite32(0x20000001U, new_value, {}));
        } else {
            ASSERT_TRUE(tracked.tryFastWrite(0x20000001U, AccessSize::word, new_value, {}));
            ASSERT_TRUE(unjournaled.tryFastWrite(0x20000001U, AccessSize::word, new_value, {}));
        }
        EXPECT_EQ(tracked.read32(0x20000001U).value(), new_value);
        EXPECT_EQ(unjournaled.read32(0x20000001U).value(), new_value);
        EXPECT_EQ(tracked.sideEffectGeneration() - tracked_effects, changed);
        EXPECT_EQ(unjournaled.sideEffectGeneration() - fast_effects, changed);
        EXPECT_EQ(tracked.sideEffectCheckpoint().mutation_sequence
                      - tracked_checkpoint.mutation_sequence, changed);
        EXPECT_EQ(unjournaled.sideEffectCheckpoint().mutation_sequence
                      - fast_checkpoint.mutation_sequence, changed);
        EXPECT_EQ(tracked.executionGeneration(), tracked_exec);
        EXPECT_EQ(unjournaled.executionGeneration(), fast_exec);
    }

    // Executable stores still invalidate code even when their bytes are
    // unchanged, while byte mutation counters remain unchanged.
    const auto exec_before = unjournaled.executionGeneration();
    const auto tracked_exec_before = tracked.executionGeneration();
    const auto seq_before = unjournaled.sideEffectCheckpoint().mutation_sequence;
    const auto tracked_seq_before = tracked.sideEffectCheckpoint().mutation_sequence;
    const auto effects_before = unjournaled.sideEffectGeneration();
    const auto tracked_effects_before = tracked.sideEffectGeneration();
    ASSERT_TRUE(unjournaled.tryFastWrite32(0x20000100U, 0U, {}));
    ASSERT_TRUE(tracked.tryFastWrite32(0x20000100U, 0U, {}));
    EXPECT_EQ(unjournaled.executionGeneration(), exec_before + 1U);
    EXPECT_EQ(tracked.executionGeneration(), tracked_exec_before + 1U);
    EXPECT_EQ(unjournaled.sideEffectCheckpoint().mutation_sequence, seq_before);
    EXPECT_EQ(tracked.sideEffectCheckpoint().mutation_sequence, tracked_seq_before);
    EXPECT_EQ(unjournaled.sideEffectGeneration(), effects_before);
    EXPECT_EQ(tracked.sideEffectGeneration(), tracked_effects_before);

    // Checked range rejection remains before the word helper and has no state
    // effects; disabling journaling does not widen a backing access.
    const auto crossing_seq = unjournaled.sideEffectCheckpoint().mutation_sequence;
    const auto crossing_effects = unjournaled.sideEffectGeneration();
    EXPECT_FALSE(unjournaled.tryFastWrite32(0x2000001eU, 0xdeadbeefU, {}));
    EXPECT_EQ(unjournaled.sideEffectCheckpoint().mutation_sequence, crossing_seq);
    EXPECT_EQ(unjournaled.sideEffectGeneration(), crossing_effects);

    const auto stale_checkpoint = unjournaled.sideEffectCheckpoint();
    unjournaled.setWriteJournalTracking(true);
    EXPECT_FALSE(unjournaled.canRestoreSideEffects(stale_checkpoint));
    const auto rollback_checkpoint = unjournaled.sideEffectCheckpoint();
    const auto original_word = unjournaled.read32(0x20000004U).value();
    ASSERT_TRUE(unjournaled.write32(0x20000004U, 0xaabbccddU));
    ASSERT_TRUE(unjournaled.restoreSideEffects(rollback_checkpoint));
    EXPECT_EQ(unjournaled.read32(0x20000004U).value(), original_word);
}

TEST(MemoryBusTest, StoreFootprintScopeTracksIdempotentAndAllStorePaths) {
    using fil::mem::AccessContext;
    using fil::mem::AccessType;
    using fil::mem::MemoryBus;

    MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 64U, "ram").hasValue());
    ASSERT_TRUE(bus.mapAlias(0x00000000U, 0x20000000U, 64U, "alias").hasValue());
    const AccessContext cpu_store{AccessType::data_write, 0x08000100U};
    EXPECT_EQ(bus.takeReadFootprint().words, MemoryBus::ReadFootprint{}.words);
    {
        MemoryBus::StoreFootprintScope tracking(bus);
        ASSERT_TRUE(bus.write8(0x20000000U, 0U, cpu_store)); // idempotent
        auto footprint = bus.takeReadFootprint();
        EXPECT_NE(footprint.words, MemoryBus::ReadFootprint{}.words);

        ASSERT_TRUE(bus.write32(0x20000001U, 0x11223344U, cpu_store));
        footprint = bus.takeReadFootprint();
        EXPECT_NE(footprint.words, MemoryBus::ReadFootprint{}.words);

        const std::uint32_t words[] = {0x01020304U, 0x05060708U};
        ASSERT_TRUE(bus.tryFastWriteWords(0x20000008U, 2U, words, cpu_store));
        footprint = bus.takeReadFootprint();
        EXPECT_NE(footprint.words, MemoryBus::ReadFootprint{}.words);

        ASSERT_TRUE(bus.write32(0x0000000cU, 0xaabbccddU, cpu_store));
        footprint = bus.takeReadFootprint();
        EXPECT_NE(footprint.words, MemoryBus::ReadFootprint{}.words)
            << "alias stores track canonical backing words";

        ASSERT_TRUE(bus.write32(0x20000010U, 0U, {AccessType::data_write, 0U}));
        EXPECT_EQ(bus.takeReadFootprint().words, MemoryBus::ReadFootprint{}.words)
            << "PC-zero stores retain existing non-CPU origin convention";
    }
    ASSERT_TRUE(bus.write8(0x20000014U, 0U, cpu_store));
    EXPECT_EQ(bus.takeReadFootprint().words, MemoryBus::ReadFootprint{}.words)
        << "store tracking is restored after scope exit";
}

TEST(MemoryBusTest, StoreCertificatesRejectChangesToEveryTouchedCanonicalWord) {
    using fil::mem::AccessContext;
    using fil::mem::AccessType;
    using fil::mem::MemoryBus;
    constexpr std::uint32_t base = 0x20000000U;
    const AccessContext cpu_store{AccessType::data_write, 0x08000100U};
    for (unsigned path = 0U; path < 7U; ++path) {
        SCOPED_TRACE(path);
        MemoryBus bus;
        ASSERT_TRUE(bus.mapRam(base, 64U, "ram"));
        ASSERT_TRUE(bus.mapAlias(0U, base, 64U, "alias"));
        std::uint32_t first = 0U;
        std::uint32_t last = 0U;
        {
            MemoryBus::StoreFootprintScope outer(bus);
            {
                MemoryBus::StoreFootprintScope inner(bus);
                const std::uint32_t words[] = {0U, 0U, 0U};
                switch (path) {
                case 0U: ASSERT_TRUE(bus.write8(base, 0U, cpu_store)); break;
                case 1U:
                    ASSERT_TRUE(bus.tryFastWrite(base + 3U, fil::mem::AccessSize::halfword, 0U, cpu_store));
                    last = 4U; break;
                case 2U: ASSERT_TRUE(bus.write32(base + 1U, 0U, cpu_store)); last = 4U; break;
                case 3U: ASSERT_TRUE(bus.write64(base + 2U, 0U, cpu_store)); last = 8U; break;
                case 4U:
                    ASSERT_TRUE(bus.tryFastWriteWords(base + 8U, 3U, words, cpu_store));
                    first = 8U; last = 16U; break;
                case 5U:
                    ASSERT_TRUE(bus.write64(11U, 0U, cpu_store));
                    first = 8U; last = 16U; break;
                case 6U:
                    bus.setWriteJournalTracking(false);
                    ASSERT_TRUE(bus.tryFastWrite32(base, 0U, cpu_store));
                    bus.setWriteJournalTracking(true); break;
                }
            }
            // Nested scope exit must preserve the outer setting.
            ASSERT_TRUE(bus.write8(base + first, 0U, cpu_store));
        }
        const auto footprint = bus.takeReadFootprint();
        for (auto word = first; word <= last; word += 4U) {
            const auto checkpoint = bus.sideEffectCheckpoint();
            ASSERT_TRUE(bus.write8(base + word, 1U, {AccessType::data_write, 0U}));
            EXPECT_FALSE(bus.ramInputsCompatibleSince(checkpoint, footprint));
        }
        ASSERT_TRUE(bus.write8(base + 32U, 0U, cpu_store));
        EXPECT_EQ(bus.takeReadFootprint().words, MemoryBus::ReadFootprint{}.words);
    }
    MemoryBus disabled;
    ASSERT_TRUE(disabled.mapRam(base, 8U, "ram"));
    disabled.setReadFootprintTracking(false);
    const auto checkpoint = disabled.sideEffectCheckpoint();
    {
        MemoryBus::StoreFootprintScope scope(disabled);
        ASSERT_TRUE(disabled.write8(base, 0U, cpu_store));
    }
    EXPECT_FALSE(disabled.ramInputsCompatibleSince(checkpoint, disabled.takeReadFootprint()));
}

TEST(MemoryBusTest, RestorationTokenRejectsJournalSequenceAba) {
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 8U, "ram"));
    ASSERT_TRUE(bus.read8(0x20000000U, {fil::mem::AccessType::data_read, 0x08000100U}));
    const auto footprint = bus.takeReadFootprint();
    const auto initial = bus.sideEffectCheckpoint();
    ASSERT_TRUE(bus.write8(0x20000000U, 1U));
    const auto admission = bus.sideEffectCheckpoint();
    const auto token = bus.restorationGeneration();
    ASSERT_TRUE(bus.restoreSideEffects(initial));
    ASSERT_TRUE(bus.write8(0x20000000U, 2U));
    EXPECT_EQ(bus.sideEffectCheckpoint().mutation_sequence, admission.mutation_sequence);
    EXPECT_TRUE(bus.ramInputsCompatibleSince(admission, footprint))
        << "sequence compatibility alone cannot detect a different transaction branch";
    EXPECT_NE(bus.restorationGeneration(), token)
        << "certificate reuse must also validate the monotonic restoration token";
}

TEST(MemoryBusTest, RamInputCompatibilityIsScopedAndDoesNotRelaxTransactionChecks) {
    using fil::mem::AccessContext;
    using fil::mem::AccessType;
    using fil::mem::MemoryBus;

    MemoryBus bus;
    RecordingMmio device;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 32U, "ram").hasValue());
    ASSERT_TRUE(bus.mapMmio(0x40000000U, 16U, device, "mmio").hasValue());
    static_cast<void>(bus.read32(0x20000000U,
        {AccessType::data_read, 0x08000100U}));
    const auto footprint = bus.takeReadFootprint();

    auto checkpoint = bus.sideEffectCheckpoint();
    ASSERT_TRUE(bus.write8(0x20000008U, 1U, {AccessType::data_write, 0U}));
    EXPECT_TRUE(bus.ramInputsCompatibleSince(checkpoint, footprint));
    checkpoint = bus.sideEffectCheckpoint();
    ASSERT_TRUE(bus.write8(0x20000000U, 1U, {AccessType::data_write, 0U}));
    EXPECT_FALSE(bus.ramInputsCompatibleSince(checkpoint, footprint));

    checkpoint = bus.sideEffectCheckpoint();
    ASSERT_TRUE(bus.write8(0x20000000U, 2U, {AccessType::data_write, 0U}));
    ASSERT_TRUE(bus.restoreSideEffects(checkpoint));
    const auto fresh_admission = bus.sideEffectCheckpoint();
    EXPECT_TRUE(bus.ramInputsCompatibleSince(fresh_admission, footprint))
        << "overlap restored before fresh admission is compatible";

    const auto mmio_checkpoint = bus.sideEffectCheckpoint();
    static_cast<void>(bus.read32(0x40000000U));
    EXPECT_TRUE(bus.ramInputsCompatibleSince(mmio_checkpoint, footprint));
    EXPECT_FALSE(bus.sideEffectsCompatibleSince(mmio_checkpoint, footprint));
    EXPECT_FALSE(bus.sideEffectsRestoredSince(mmio_checkpoint));
    EXPECT_FALSE(bus.canRestoreSideEffects(mmio_checkpoint));
}

TEST(MemoryBusTest, RestoreAdvancesMonotonicGeneration) {
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 16U, "ram").hasValue());
    const auto initial = bus.restorationGeneration();
    const auto checkpoint = bus.sideEffectCheckpoint();
    ASSERT_TRUE(bus.write8(0x20000000U, 0x5aU));
    ASSERT_TRUE(bus.restoreSideEffects(checkpoint));
    EXPECT_EQ(bus.restorationGeneration(), initial + 1U);
    const auto no_op_checkpoint = bus.sideEffectCheckpoint();
    ASSERT_TRUE(bus.restoreSideEffects(no_op_checkpoint));
    EXPECT_EQ(bus.restorationGeneration(), initial + 2U);
}

TEST(MemoryBusTest, CpuReadFootprintsCoverEveryTouchedWord) {
    using fil::mem::AccessContext;
    using fil::mem::AccessType;
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 32U, "ram").hasValue());
    const AccessContext cpu_read{AccessType::data_read, 0x08000100U};

    ASSERT_TRUE(bus.read32(0x20000003U, cpu_read)); // crosses word boundary
    const auto unaligned = bus.takeReadFootprint();
    const auto first_word = bus.sideEffectCheckpoint();
    ASSERT_TRUE(bus.write8(0x20000000U, 1U, {AccessType::data_write, 0U}));
    EXPECT_FALSE(bus.ramInputsCompatibleSince(first_word, unaligned));
    const auto second_word = bus.sideEffectCheckpoint();
    ASSERT_TRUE(bus.write8(0x20000004U, 1U, {AccessType::data_write, 0U}));
    EXPECT_FALSE(bus.ramInputsCompatibleSince(second_word, unaligned));

    ASSERT_TRUE(bus.read64(0x20000008U, cpu_read));
    const auto wide = bus.takeReadFootprint();
    const auto wide_checkpoint = bus.sideEffectCheckpoint();
    ASSERT_TRUE(bus.write8(0x2000000cU, 1U, {AccessType::data_write, 0U}));
    EXPECT_FALSE(bus.ramInputsCompatibleSince(wide_checkpoint, wide));
}

TEST(MemoryBusTest, TrackingScopeRestoresSettingsOnExit) {
    fil::mem::MemoryBus bus;
    bus.setReadFootprintTracking(false);
    {
        fil::mem::MemoryBus::TrackingScope tracking(bus, true, false);
        EXPECT_TRUE(bus.readFootprintTracking());
        EXPECT_FALSE(bus.writeJournalTracking());
    }
    EXPECT_FALSE(bus.readFootprintTracking());
    EXPECT_TRUE(bus.writeJournalTracking());
}

TEST(MemoryBusTest, ReversibleRamOnlyRejectsNonRamStoresBeforeMutation) {
    using fil::mem::AccessContext;
    using fil::mem::AccessSize;
    using fil::mem::AccessType;
    using fil::mem::BusFaultReason;
    using fil::mem::MemoryBus;

    MemoryBus bus;
    RecordingMmio device;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 16U, "ordinary-ram").hasValue());
    ASSERT_TRUE(bus.mapRam(0x20000100U, 16U, "executable-ram", true).hasValue());
    ASSERT_TRUE(bus.mapFlash(0x08000000U, 16U, "writable-flash").hasValue());
    ASSERT_TRUE(bus.mapRom(0x08000100U, 16U, "readonly-rom").hasValue());
    ASSERT_TRUE(bus.mapAlias(0x00000000U, 0x20000000U, 16U, "ram-alias").hasValue());
    ASSERT_TRUE(bus.mapAlias(0x00000100U, 0x08000000U, 16U, "flash-alias").hasValue());
    ASSERT_TRUE(bus.mapAlias(0x00000200U, 0x08000100U, 16U, "readonly-alias").hasValue());
    ASSERT_TRUE(bus.mapMmio(0x40000000U, 16U, device, "device").hasValue());

    bus.setReversibleRamOnly(true);
    EXPECT_TRUE(bus.reversibleRamOnly());
    const auto checkpoint = bus.sideEffectCheckpoint();
    EXPECT_TRUE(bus.write32(0x00000000U, 0x11223344U).hasValue())
        << "alias-resolved ordinary RAM remains writable";
    EXPECT_TRUE(bus.read32(0x20000000U).value() == 0x11223344U)
        << "RAM alias mutates its backing bytes";

    const auto reject = [](const auto& result) {
        return !result && result.fault().reason == BusFaultReason::synchronization_required;
    };
    EXPECT_TRUE(reject(bus.write32(0x20000100U, 0x55667788U)))
        << "rejects executable RAM";
    EXPECT_TRUE(reject(bus.write32(0x08000000U, 0xaabbccddU)))
        << "rejects writable flash";
    EXPECT_TRUE(reject(bus.write32(0x00000100U, 0xaabbccddU)))
        << "checks the backing kind after alias translation";
    const auto readonly = bus.write32(0x00000200U, 0xaabbccddU);
    EXPECT_TRUE(!readonly && readonly.fault().reason == BusFaultReason::write_protected)
        << "preserves backing write protection through aliases in reversible mode";
    EXPECT_TRUE(reject(bus.write64(0x40000000U, 0x123456789abcdef0ULL)))
        << "generic-width fallback rejects MMIO before dispatch";
    EXPECT_TRUE(device.write_count == 0) << "MMIO device receives no rejected store";

    std::uint32_t fast_value = 0xaabbccddU;
    EXPECT_TRUE(!bus.tryFastWrite32(0x08000000U, fast_value, {}))
        << "fast 32-bit path declines restricted flash";
    EXPECT_TRUE(!bus.tryFastWrite(0x20000100U, AccessSize::byte, 0x5aU, {}))
        << "generic fast path declines executable RAM";
    const std::uint32_t words[] = {0x01020304U, 0x05060708U};
    EXPECT_TRUE(!bus.tryFastWriteWords(0x2000000cU, 2U, words, {}))
        << "bulk fast path rejects a range crossing the RAM boundary";
    EXPECT_TRUE(bus.read32(0x2000000cU).value() == 0U)
        << "crossing bulk write leaves its in-range prefix unchanged";

    EXPECT_TRUE(bus.restoreSideEffects(checkpoint))
        << "rollback reverses the permitted RAM alias mutation";
    EXPECT_TRUE(bus.read32(0x20000000U).value() == 0U)
        << "rollback restores the original RAM bytes";
    EXPECT_TRUE(bus.canRestoreSideEffects(bus.sideEffectCheckpoint()))
        << "checkpoint restoration remains available";
    static_assert(MemoryBus::max_reversible_ram_mutations == 1024U);

    bus.setReversibleRamOnly(false);
    EXPECT_TRUE(!bus.reversibleRamOnly());
    EXPECT_TRUE(bus.write32(0x08000000U, 0x12345678U).hasValue())
        << "disabling restriction restores ordinary writable-flash semantics";
}

TEST(MemoryBusTest, TrapsSharedMmioBeforeDeviceSideEffects) {
    fil::mem::MemoryBus bus;
    fil::mem::MmioRouter router(0x40000000U, 0x1000U, "peripherals");
    RecordingMmio shared;
    shared.shared = true;
    EXPECT_TRUE(router.map(0x100U, 0x20U, shared, "shared-child").hasValue() &&
                bus.mapMmio(0x40000000U, 0x1000U, router, "peripherals").hasValue())
        << "maps shared MMIO through a router";

    bus.setSharedMmioTrapping(true);
    const auto checkpoint = bus.sideEffectCheckpoint();
    const auto trapped = bus.read32(
        0x40000100U, {fil::mem::AccessType::data_read, 0x08000100U}
    );
    EXPECT_TRUE(!trapped &&
                trapped.fault().reason == fil::mem::BusFaultReason::synchronization_required &&
                shared.read_count == 0 && bus.sideEffectsRestoredSince(checkpoint))
        << "shared MMIO trap stops before device or journal side effects";

    bus.setSharedMmioTrapping(false);
    const auto committed = bus.read32(0x40000100U);
    EXPECT_TRUE(committed && shared.read_count == 1)
        << "coordinator can disable trapping and commit shared MMIO";
}

/** @brief Verifies sub-device routing and aggregate lenient unknown handling. */
TEST(MemoryBusTest, RoutesMmioWindows) {
    fil::mem::MemoryBus bus;
    fil::mem::MmioRouter router(0x40000000U, 0x1000U, "peripherals");
    RecordingMmio device;
    EXPECT_TRUE(router.map(0x100U, 0x20U, device, "child").hasValue()) << "maps MMIO child route";
    EXPECT_TRUE(!router.map(0x110U, 0x20U, device, "overlap"))
        << "rejects overlapping MMIO child route";
    EXPECT_TRUE(bus.mapMmio(0x40000000U, 0x1000U, router, "peripherals").hasValue())
        << "maps routed MMIO window";

    const auto routed = bus.read32(0x40000104U);
    const auto unknown_read = bus.read32(0x40000200U);
    const auto unknown_write = bus.write16(0x40000200U, 0x1234U);
    EXPECT_TRUE(routed && device.last_offset == 4U) << "routes access with child-relative offset";
    EXPECT_TRUE(unknown_read && unknown_read.value() == 0U && unknown_write)
        << "handles unknown MMIO leniently";
    const auto unknown = router.unknownAccesses();
    EXPECT_TRUE(unknown.size() == 1 && unknown[0].address == 0x40000200U && unknown[0].reads == 1 &&
                unknown[0].writes == 1)
        << "aggregates unknown MMIO counts by absolute address";

    router.setLenient(false);
    const auto strict = bus.read32(0x40000300U);
    EXPECT_TRUE(!strict && strict.fault().reason == fil::mem::BusFaultReason::device_error)
        << "faults on unknown MMIO in strict mode";
}

/** @brief Verifies ELF bytes load at physical addresses without initializing runtime SRAM. */
TEST(MemoryBusTest, MaterializesElfLoadImage) {
    const auto fixture = std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";
    const auto image = fil::elf::load(fixture);
    EXPECT_TRUE(image.hasValue()) << "loads ELF fixture for memory materialization";
    if (!image) return;

    fil::mem::MemoryBus bus;
    EXPECT_TRUE(bus.mapRom(0x08000000U, 64, "flash").hasValue()) << "maps fixture flash";
    EXPECT_TRUE(bus.mapRam(0x20000000U, 64, "sram").hasValue()) << "maps fixture SRAM";
    EXPECT_TRUE(bus.materialize(image.value()).hasValue()) << "materializes ELF image";

    const auto vector = bus.read32(0x08000000U);
    const auto initializer = bus.read32(0x08000018U);
    const auto runtime_data = bus.read32(0x20000000U);
    const auto erased = bus.read8(0x08000030U);
    EXPECT_TRUE(vector && vector.value() == 0x20020000U) << "loads vector bytes into flash";
    EXPECT_TRUE(initializer && initializer.value() == 0x12345678U)
        << "loads .data initializer at physical flash address";
    EXPECT_TRUE(runtime_data && runtime_data.value() == 0)
        << "leaves .data runtime SRAM at reset value";
    EXPECT_TRUE(erased && erased.value() == 0xffU) << "leaves unloaded ROM bytes erased";
}

/** @brief Verifies config-driven STM32 map construction and reset validation. */
TEST(MemoryBusTest, BuildsMcuResetMap) {
    const auto fixture = std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";
    const auto image = fil::elf::load(fixture);
    if (!image) {
        EXPECT_TRUE(false) << "loads ELF fixture for MCU map";
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
    EXPECT_TRUE(memory.hasValue()) << "builds standard config-driven MCU memory map";
    if (!memory) return;
    const auto regions = memory.value().regions();
    EXPECT_TRUE(regions.size() == 7) << "maps all standard memory windows";
    const auto aliased_vector = memory.value().read32(0x00000000U);
    EXPECT_TRUE(aliased_vector && aliased_vector.value() == image.value().initialMsp())
        << "boot alias exposes loaded vector table";
}

} // namespace

/** @brief Runs all memory bus unit-test cases. */
