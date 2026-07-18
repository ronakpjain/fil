#pragma once

/** @file memory_bus.hpp
 *  @brief Firmware-agnostic target address map and MMIO dispatch.
 */

#include "fil/common/result.hpp"
#include "fil/mem/address.hpp"
#include "fil/mem/region.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace fil::elf {
class ElfImage;
}

namespace fil::mem {

/**
 * @brief Holds either a completed memory value or a structured BusFault.
 * @tparam T Successful access value type.
 */
template <typename T>
class [[nodiscard]] MemoryResult {
public:
    /** @brief Constructs a successful access result. @param value Access value. */
    MemoryResult(T value) : value_(std::move(value)) {}

    /** @brief Constructs a failed access result. @param fault Bus fault details. */
    MemoryResult(BusFault fault)
        : fault_(std::make_unique<BusFault>(std::move(fault))) {}

    MemoryResult(const MemoryResult& other)
        : value_(other.value_),
          fault_(other.fault_ ? std::make_unique<BusFault>(*other.fault_) : nullptr) {}
    MemoryResult& operator=(const MemoryResult& other) {
        if (this == &other) return *this;
        value_ = other.value_;
        fault_ = other.fault_ ? std::make_unique<BusFault>(*other.fault_) : nullptr;
        return *this;
    }
    MemoryResult(MemoryResult&&) noexcept = default;
    MemoryResult& operator=(MemoryResult&&) noexcept = default;

    /** @brief Tests whether the access succeeded. @return True when a value is present. */
    [[nodiscard]] bool hasValue() const noexcept { return fault_ == nullptr; }

    /** @brief Tests whether the access succeeded. @return True when a value is present. */
    [[nodiscard]] explicit operator bool() const noexcept { return hasValue(); }

    /** @brief Gets the successful value. @return Mutable value reference. */
    [[nodiscard]] T& value() & { assert(hasValue()); return value_; }

    /** @brief Gets the successful value. @return Const value reference. */
    [[nodiscard]] const T& value() const& { assert(hasValue()); return value_; }

    /** @brief Gets the access fault. @return Mutable fault reference. */
    [[nodiscard]] BusFault& fault() & {
        assert(fault_ != nullptr);
        return *fault_;
    }

    /** @brief Gets the access fault. @return Const fault reference. */
    [[nodiscard]] const BusFault& fault() const& {
        assert(fault_ != nullptr);
        return *fault_;
    }

private:
    T value_{};
    std::unique_ptr<BusFault> fault_;
};

/** @brief Interface implemented by target peripheral register blocks. */
/** @brief Visibility of an MMIO access to independently executing board lanes. */
enum class MmioDomain {
    board_local,
    shared,
};

class MmioDevice {
public:
    /** @brief Allows polymorphic destruction through the interface. */
    virtual ~MmioDevice() = default;

    /**
     * @brief Performs one indivisible peripheral register read.
     * @param offset Byte offset from the mapped MMIO base.
     * @param size Original CPU access width.
     * @param context Access metadata.
     * @return Register value in the low bits, or a device fault.
     */
    [[nodiscard]] virtual MemoryResult<std::uint64_t> read(
        std::uint32_t offset,
        AccessSize size,
        const AccessContext& context
    ) = 0;

    /**
     * @brief Performs one indivisible peripheral register write.
     * @param offset Byte offset from the mapped MMIO base.
     * @param size Original CPU access width.
     * @param value Value in the low width-appropriate bits.
     * @param context Access metadata.
     * @return Zero on success, or a device fault.
     */
    [[nodiscard]] virtual MemoryResult<std::uint64_t> write(
        std::uint32_t offset,
        AccessSize size,
        std::uint64_t value,
        const AccessContext& context
    ) = 0;

    /** @brief Gets the device's diagnostic name. @return Static or device-owned name. */
    [[nodiscard]] virtual std::string_view name() const noexcept = 0;

    /** Whether a board-local transaction journal covers this access. */
    [[nodiscard]] virtual bool transactionalAccessSafe(
        std::uint32_t offset, AccessSize size, bool write
    ) const noexcept {
        static_cast<void>(offset);
        static_cast<void>(size);
        static_cast<void>(write);
        return false;
    }

    /** @brief Classifies an access for conservative multi-board synchronization. */
    [[nodiscard]] virtual MmioDomain domain(
        std::uint32_t offset, AccessSize size
    ) const noexcept {
        static_cast<void>(offset);
        static_cast<void>(size);
        return MmioDomain::board_local;
    }
};

/**
 * @brief Non-overlapping 32-bit target address map.
 *
 * RAM and ROM accesses are little-endian and may be unaligned when wholly
 * contained in one region. MMIO accesses retain their original width and are
 * dispatched exactly once. Alias regions translate to another mapped address.
 */
class MemoryBus {
public:
    struct SideEffectCheckpoint {
        std::uint64_t mutation_sequence{0};
        std::uint64_t mmio_generation{0};
    };
    struct ReadFootprint {
        static constexpr std::size_t word_count = 4U;
        std::array<std::uint64_t, word_count> words{};
    };
    /** @brief Constructs an empty address map. */
    MemoryBus();

    /** @brief Destroys mapped region storage; MMIO devices remain caller-owned. */
    ~MemoryBus();

    MemoryBus(const MemoryBus&) = delete;
    MemoryBus& operator=(const MemoryBus&) = delete;
    MemoryBus(MemoryBus&&) noexcept;
    MemoryBus& operator=(MemoryBus&&) noexcept;

    /** @brief Maps zero-initialized RAM. */
    [[nodiscard]] Result<void> mapRam(
        std::uint32_t base, std::uint32_t size, std::string name, bool executable = false
    );

    /** @brief Maps erased (`0xff`) read-only storage. */
    [[nodiscard]] Result<void> mapRom(
        std::uint32_t base, std::uint32_t size, std::string name, bool executable = true
    );

    /** @brief Maps an address range that translates to a target range. */
    [[nodiscard]] Result<void> mapAlias(
        std::uint32_t alias_base,
        std::uint32_t target_base,
        std::uint32_t size,
        std::string name
    );

    /** @brief Maps a caller-owned peripheral device. */
    [[nodiscard]] Result<void> mapMmio(
        std::uint32_t base,
        std::uint32_t size,
        MmioDevice& device,
        std::string name
    );

    /**
     * @brief Copies bytes into RAM or ROM without applying CPU write permissions.
     * @param address First target address.
     * @param bytes Source bytes.
     * @return Success or a map/range error.
     */
    [[nodiscard]] Result<void> loadBytes(std::uint32_t address, std::span<const std::uint8_t> bytes);

    /**
     * @brief Materializes all file-backed ELF segments at physical load addresses.
     * @param image Parsed ELF image.
     * @return Success when load and runtime ranges are mapped appropriately.
     */
    [[nodiscard]] Result<void> materialize(const elf::ElfImage& image);

    /** @brief Reads an 8-bit little-endian value. */
    [[nodiscard]] MemoryResult<std::uint8_t> read8(std::uint32_t address, AccessContext context = {}) const;
    /** @brief Reads a 16-bit little-endian value. */
    [[nodiscard]] MemoryResult<std::uint16_t> read16(std::uint32_t address, AccessContext context = {}) const;
    /** @brief Reads a 32-bit little-endian value. */
    [[nodiscard]] MemoryResult<std::uint32_t> read32(std::uint32_t address, AccessContext context = {}) const;
    /** @brief Reads a 64-bit little-endian value. */
    [[nodiscard]] MemoryResult<std::uint64_t> read64(std::uint32_t address, AccessContext context = {}) const;

    /** @brief Writes an 8-bit little-endian value. */
    [[nodiscard]] MemoryResult<std::uint64_t> write8(
        std::uint32_t address, std::uint8_t value, AccessContext context = {AccessType::data_write, 0}
    );
    /** @brief Writes a 16-bit little-endian value. */
    [[nodiscard]] MemoryResult<std::uint64_t> write16(
        std::uint32_t address, std::uint16_t value, AccessContext context = {AccessType::data_write, 0}
    );
    /** @brief Writes a 32-bit little-endian value. */
    [[nodiscard]] MemoryResult<std::uint64_t> write32(
        std::uint32_t address, std::uint32_t value, AccessContext context = {AccessType::data_write, 0}
    );
    /** @brief Writes a 64-bit little-endian value. */
    [[nodiscard]] MemoryResult<std::uint64_t> write64(
        std::uint32_t address, std::uint64_t value, AccessContext context = {AccessType::data_write, 0}
    );

    /** @brief Gets mapped ranges sorted by base address. */
    [[nodiscard]] std::vector<MemoryRegionInfo> regions() const;

    /** @brief Makes shared MMIO return a side-effect-free worker synchronization fault. */
    void setSharedMmioTrapping(bool enabled) noexcept { trap_shared_mmio_ = enabled; }

    /** @brief Whether shared MMIO accesses currently stop before device dispatch. */
    [[nodiscard]] bool sharedMmioTrapping() const noexcept { return trap_shared_mmio_; }

    /** @brief Makes every MMIO access yield, for fully reversible worker windows. */
    void setAllMmioTrapping(bool enabled) noexcept { trap_all_mmio_ = enabled; }

    /** @brief Whether every MMIO access currently yields before dispatch. */
    [[nodiscard]] bool allMmioTrapping() const noexcept { return trap_all_mmio_; }

    /** @brief Whether any MMIO trap mode requires restartable CPU execution. */
    [[nodiscard]] bool mmioTrapping() const noexcept {
        return trap_shared_mmio_ || trap_all_mmio_;
    }

    /**
     * @brief Gets the generation of executable backing storage.
     *
     * The value changes whenever an executable mapping is created or its
     * directly backed bytes are modified. CPU instruction caches can use it as
     * a conservative self-modifying-code invalidation boundary.
     */
    [[nodiscard]] std::uint64_t executionGeneration() const noexcept {
        return execution_generation_;
    }

    /**
     * @brief Generation of externally observable data side effects.
     *
     * Changes on every MMIO read/write and every value-changing write to
     * directly backed memory. Plain RAM/ROM reads and idempotent backed writes
     * leave it unchanged so exact-state idle loops can be proven safely.
     */
    [[nodiscard]] std::uint64_t sideEffectGeneration() const noexcept {
        return side_effect_generation_;
    }

    /** @brief Captures a marker for exact reversible-memory/MMIO loop proof. */
    [[nodiscard]] SideEffectCheckpoint sideEffectCheckpoint() const noexcept;

    /** @brief True when no MMIO occurred and all changed backed bytes were restored. */
    [[nodiscard]] bool sideEffectsRestoredSince(SideEffectCheckpoint checkpoint) const;

    /** @brief Returns and clears CPU backed-memory reads since the previous take. */
    [[nodiscard]] ReadFootprint takeReadFootprint() noexcept;
    [[nodiscard]] ReadFootprint readFootprint() const noexcept { return read_footprint_; }
    void restoreReadFootprint(const ReadFootprint& footprint) noexcept {
        read_footprint_ = footprint;
    }

    /** @brief Checks restored CPU writes and external writes against a read footprint. */
    [[nodiscard]] bool sideEffectsCompatibleSince(
        SideEffectCheckpoint checkpoint, const ReadFootprint& footprint
    ) const;

    /** @brief Whether backed-memory state can be transactionally restored. */
    [[nodiscard]] bool canRestoreSideEffects(SideEffectCheckpoint checkpoint) const noexcept;

    /** @brief Reverses backed-memory writes when no MMIO occurred since a checkpoint. */
    [[nodiscard]] bool restoreSideEffects(SideEffectCheckpoint checkpoint);

    /** @brief True when no MMIO read or write occurred since the checkpoint. */
    [[nodiscard]] bool mmioUnchangedSince(SideEffectCheckpoint checkpoint) const noexcept {
        return checkpoint.mmio_generation == mmio_generation_;
    }

private:
    struct Region;
    struct BackedMutation {
        std::uint64_t sequence{0};
        std::uint32_t address{0};
        std::uint8_t old_value{0};
        bool external{false};
    };

    static void addReadFootprint(ReadFootprint& footprint, std::uint32_t address) noexcept;
    [[nodiscard]] static bool footprintContains(
        const ReadFootprint& footprint, std::uint32_t address
    ) noexcept;

    [[nodiscard]] MemoryResult<std::uint64_t> read(
        std::uint32_t address, AccessSize size, const AccessContext& context, unsigned int alias_depth
    ) const;
    [[nodiscard]] MemoryResult<std::uint64_t> write(
        std::uint32_t address,
        AccessSize size,
        std::uint64_t value,
        const AccessContext& context,
        unsigned int alias_depth
    );
    [[nodiscard]] Result<void> addRegion(std::unique_ptr<Region> region);
    [[nodiscard]] const Region* find(std::uint32_t address) const;
    [[nodiscard]] Region* find(std::uint32_t address);

    std::vector<std::unique_ptr<Region>> regions_;
    mutable std::array<const Region*, 256> region_lookup_cache_{};
    std::uint64_t execution_generation_{1};
    mutable std::uint64_t side_effect_generation_{1};
    mutable std::uint64_t mmio_generation_{1};
    std::uint64_t mutation_sequence_{0};
    static constexpr std::size_t mutation_journal_capacity = 8192U;
    std::array<BackedMutation, mutation_journal_capacity> mutation_journal_{};
    mutable ReadFootprint read_footprint_{};
    bool trap_shared_mmio_{false};
    bool trap_all_mmio_{false};
};

} // namespace fil::mem
