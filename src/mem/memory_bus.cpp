#include "fil/mem/memory_bus.hpp"

#include "fil/elf/elf_loader.hpp"

#include <algorithm>
#include <bit>
#include <cstring>
#include <limits>

namespace fil::mem {
namespace {

constexpr unsigned int maximum_alias_depth = 8;

/** @brief Checks a nonempty target range without 32-bit wraparound. */
bool validRange(const std::uint32_t base, const std::uint32_t size) {
    return size != 0 && base <= std::numeric_limits<std::uint32_t>::max() - (size - 1U);
}

/** @brief Creates a map-construction error. */
Error mapError(std::string message) {
    return Error{ErrorCategory::invalid_argument, std::move(message), std::nullopt};
}

/** @brief Creates a structured bus fault for one access. */
BusFault makeFault(
    const BusFaultReason reason,
    const std::uint32_t address,
    const AccessSize size,
    const AccessContext& context,
    std::string region,
    std::string message
) {
    return BusFault{reason, address, size, context, std::move(region), std::move(message)};
}

} // namespace

struct MemoryBus::Region {
    MemoryRegionInfo info;
    std::vector<std::uint8_t> bytes;
    std::uint32_t alias_target{0};
    MmioDevice* device{nullptr};

    /** @brief Gets the first address after this region in widened arithmetic. */
    [[nodiscard]] std::uint64_t end() const {
        return static_cast<std::uint64_t>(info.base) + info.size;
    }

    /** @brief Tests whether one address lies in this region. */
    [[nodiscard]] bool contains(const std::uint32_t address) const {
        return address >= info.base && static_cast<std::uint64_t>(address) < end();
    }

    /** @brief Tests whether an entire non-wrapping range lies in this region. */
    [[nodiscard]] bool containsRange(const std::uint32_t address, const std::uint32_t size) const {
        return address >= info.base
            && static_cast<std::uint64_t>(address) + size <= end();
    }
};

MemoryBus::MemoryBus() = default;
MemoryBus::~MemoryBus() = default;
MemoryBus::MemoryBus(MemoryBus&&) noexcept = default;
MemoryBus& MemoryBus::operator=(MemoryBus&&) noexcept = default;

Result<void> MemoryBus::addRegion(std::unique_ptr<Region> region) {
    if (!validRange(region->info.base, region->info.size)) {
        return mapError("memory region '" + region->info.name + "' has an empty or wrapping range");
    }

    const std::uint64_t candidate_begin = region->info.base;
    const std::uint64_t candidate_end = region->end();
    for (const auto& existing : regions_) {
        const std::uint64_t overlap_begin = std::max(candidate_begin, static_cast<std::uint64_t>(existing->info.base));
        const std::uint64_t overlap_end = std::min(candidate_end, existing->end());
        if (overlap_begin < overlap_end) {
            return mapError(
                "memory region '" + region->info.name + "' overlaps region '" + existing->info.name + "'"
            );
        }
    }

    regions_.push_back(std::move(region));
    std::sort(regions_.begin(), regions_.end(), [](const auto& left, const auto& right) {
        return left->info.base < right->info.base;
    });
    ++execution_generation_;
    return {};
}

Result<void> MemoryBus::mapRam(
    const std::uint32_t base,
    const std::uint32_t size,
    std::string name,
    const bool executable
) {
    auto region = std::make_unique<Region>();
    region->info = MemoryRegionInfo{base, size, RegionKind::ram, std::move(name), true, true, executable};
    if (validRange(base, size)) {
        region->bytes.assign(size, 0);
    }
    return addRegion(std::move(region));
}

Result<void> MemoryBus::mapRom(
    const std::uint32_t base,
    const std::uint32_t size,
    std::string name,
    const bool executable
) {
    auto region = std::make_unique<Region>();
    region->info = MemoryRegionInfo{base, size, RegionKind::rom, std::move(name), true, false, executable};
    if (validRange(base, size)) {
        region->bytes.assign(size, 0xffU);
    }
    return addRegion(std::move(region));
}

Result<void> MemoryBus::mapFlash(
    const std::uint32_t base,
    const std::uint32_t size,
    std::string name,
    const bool executable
) {
    auto region = std::make_unique<Region>();
    region->info = MemoryRegionInfo{base, size, RegionKind::flash, std::move(name), true, true, executable};
    if (validRange(base, size)) {
        region->bytes.assign(size, 0xffU);
    }
    return addRegion(std::move(region));
}

Result<void> MemoryBus::mapAlias(
    const std::uint32_t alias_base,
    const std::uint32_t target_base,
    const std::uint32_t size,
    std::string name
) {
    if (!validRange(target_base, size)) {
        return mapError("alias '" + name + "' has an empty or wrapping target range");
    }
    const Region* target = find(target_base);
    if (target == nullptr || !target->containsRange(target_base, size)) {
        return mapError("alias '" + name + "' target range is not contained in one mapped region");
    }

    auto region = std::make_unique<Region>();
    region->info = MemoryRegionInfo{
        alias_base, size, RegionKind::alias, std::move(name),
        target->info.readable, target->info.writable, target->info.executable,
    };
    region->alias_target = target_base;
    return addRegion(std::move(region));
}

Result<void> MemoryBus::mapMmio(
    const std::uint32_t base,
    const std::uint32_t size,
    MmioDevice& device,
    std::string name
) {
    auto region = std::make_unique<Region>();
    region->info = MemoryRegionInfo{base, size, RegionKind::mmio, std::move(name), true, true, false};
    region->device = &device;
    return addRegion(std::move(region));
}

const MemoryBus::Region* MemoryBus::find(const std::uint32_t address) const {
    const std::size_t cache_index = address >> 24U;
    const Region* cached = region_lookup_cache_[cache_index];
    if (cached != nullptr && cached->contains(address)) {
        return cached;
    }
    for (const auto& region : regions_) {
        if (region->contains(address)) {
            region_lookup_cache_[cache_index] = region.get();
            return region.get();
        }
        if (region->info.base > address) {
            break;
        }
    }
    region_lookup_cache_[cache_index] = nullptr;
    return nullptr;
}

MemoryBus::Region* MemoryBus::find(const std::uint32_t address) {
    return const_cast<Region*>(std::as_const(*this).find(address));
}

Result<void> MemoryBus::loadBytes(
    const std::uint32_t address,
    const std::span<const std::uint8_t> bytes
) {
    if (bytes.empty()) {
        return {};
    }
    if (bytes.size() > std::numeric_limits<std::uint32_t>::max()
        || !validRange(address, static_cast<std::uint32_t>(bytes.size()))) {
        return mapError("load byte range wraps target address space");
    }
    Region* region = find(address);
    if (region == nullptr || !region->containsRange(address, static_cast<std::uint32_t>(bytes.size()))) {
        return mapError("load byte range is not contained in one mapped region");
    }
    if (region->info.kind != RegionKind::ram && region->info.kind != RegionKind::rom
        && region->info.kind != RegionKind::flash) {
        return mapError("load bytes require directly backed RAM, ROM, or flash, not region '" + region->info.name + "'");
    }
    const std::size_t offset = address - region->info.base;
    for (std::size_t index = 0; index < bytes.size(); ++index) {
        if (region->bytes[offset + index] != bytes[index]) {
            ++mutation_sequence_;
            mutation_journal_[(mutation_sequence_ - 1U) % mutation_journal_capacity] = BackedMutation{
                mutation_sequence_, address + static_cast<std::uint32_t>(index),
                region->bytes[offset + index], true,
            };
            ++side_effect_generation_;
        }
    }
    std::copy(bytes.begin(), bytes.end(), region->bytes.begin() + static_cast<std::ptrdiff_t>(offset));
    if (region->info.executable) {
        ++execution_generation_;
    }
    return {};
}

Result<void> MemoryBus::materialize(const elf::ElfImage& image) {
    for (const elf::ElfLoadSegment& segment : image.segments()) {
        if (segment.memory_size != 0) {
            const Region* runtime = find(segment.virtual_address);
            if (runtime == nullptr || !runtime->containsRange(segment.virtual_address, segment.memory_size)) {
                return mapError("ELF runtime segment is not contained in one mapped memory region");
            }
        }
        if (segment.file_size == 0) {
            continue;
        }
        auto bytes = image.readLoadImage(segment.physical_address, segment.file_size);
        if (!bytes) {
            return bytes.error();
        }
        auto loaded = loadBytes(segment.physical_address, bytes.value());
        if (!loaded) {
            return loaded.error();
        }
    }
    return {};
}

MemoryResult<std::uint64_t> MemoryBus::read(
    const std::uint32_t address,
    const AccessSize size,
    const AccessContext& context,
    const unsigned int alias_depth
) const {
    const std::uint32_t width = byteCount(size);
    if (!validRange(address, width)) {
        return makeFault(
            BusFaultReason::address_overflow, address, size, context, {},
            "memory read wraps target address space"
        );
    }
    const Region* region = find(address);
    if (region == nullptr) {
        return makeFault(BusFaultReason::unmapped, address, size, context, {}, "read from unmapped memory");
    }
    if (!region->containsRange(address, width)) {
        return makeFault(
            BusFaultReason::cross_region, address, size, context, region->info.name,
            "memory read crosses a region boundary"
        );
    }
    if (!region->info.readable) {
        return makeFault(
            BusFaultReason::read_protected, address, size, context, region->info.name,
            "memory region is not readable"
        );
    }
    if (context.type == AccessType::instruction_fetch && !region->info.executable) {
        return makeFault(
            BusFaultReason::execute_protected, address, size, context, region->info.name,
            "memory region is not executable"
        );
    }

    if (region->info.kind == RegionKind::alias) {
        if (alias_depth >= maximum_alias_depth) {
            return makeFault(
                BusFaultReason::alias_cycle, address, size, context, region->info.name,
                "alias translation exceeded its recursion limit"
            );
        }
        const std::uint32_t translated = region->alias_target + (address - region->info.base);
        return read(translated, size, context, alias_depth + 1U);
    }
    if (region->info.kind == RegionKind::mmio) {
        const std::uint32_t offset = address - region->info.base;
        if (trap_all_mmio_ || (trap_shared_mmio_
            && region->device->domain(offset, size) == MmioDomain::shared)) {
            return makeFault(
                BusFaultReason::synchronization_required, address, size, context,
                region->info.name, "MMIO access requires coordinator synchronization"
            );
        }
        ++side_effect_generation_;
        ++mmio_generation_;
        auto result = region->device->read(offset, size, context);
        if (!result && result.fault().region.empty()) {
            result.fault().region = region->info.name;
        }
        if (result) {
            result.value() &= accessWidthMask(size);
        }
        return result;
    }

    if (context.type == AccessType::data_read && context.pc != 0U) {
        addReadFootprint(read_footprint_, address);
    }
    const std::size_t offset = address - region->info.base;
    std::uint64_t value = 0;
    for (std::uint32_t index = 0; index < width; ++index) {
        value |= static_cast<std::uint64_t>(region->bytes[offset + index]) << (index * 8U);
    }
    return value;
}

MemoryResult<std::uint64_t> MemoryBus::write(
    const std::uint32_t address,
    const AccessSize size,
    const std::uint64_t value,
    const AccessContext& context,
    const unsigned int alias_depth
) {
    const std::uint32_t width = byteCount(size);
    if (!validRange(address, width)) {
        return makeFault(
            BusFaultReason::address_overflow, address, size, context, {},
            "memory write wraps target address space"
        );
    }
    Region* region = find(address);
    if (region == nullptr) {
        return makeFault(BusFaultReason::unmapped, address, size, context, {}, "write to unmapped memory");
    }
    if (!region->containsRange(address, width)) {
        return makeFault(
            BusFaultReason::cross_region, address, size, context, region->info.name,
            "memory write crosses a region boundary"
        );
    }
    if (!region->info.writable) {
        return makeFault(
            BusFaultReason::write_protected, address, size, context, region->info.name,
            "memory region is not writable"
        );
    }

    if (region->info.kind == RegionKind::alias) {
        if (alias_depth >= maximum_alias_depth) {
            return makeFault(
                BusFaultReason::alias_cycle, address, size, context, region->info.name,
                "alias translation exceeded its recursion limit"
            );
        }
        const std::uint32_t translated = region->alias_target + (address - region->info.base);
        return write(translated, size, value, context, alias_depth + 1U);
    }
    if (region->info.kind == RegionKind::mmio) {
        const std::uint32_t offset = address - region->info.base;
        if (trap_all_mmio_ || (trap_shared_mmio_
            && region->device->domain(offset, size) == MmioDomain::shared)) {
            return makeFault(
                BusFaultReason::synchronization_required, address, size, context,
                region->info.name, "MMIO access requires coordinator synchronization"
            );
        }
        ++side_effect_generation_;
        ++mmio_generation_;
        auto result = region->device->write(
            offset, size, value & accessWidthMask(size), context
        );
        if (!result && result.fault().region.empty()) {
            result.fault().region = region->info.name;
        }
        return result;
    }

    const std::size_t offset = address - region->info.base;
    for (std::uint32_t index = 0; index < width; ++index) {
        const auto byte = static_cast<std::uint8_t>(value >> (index * 8U));
        if (region->bytes[offset + index] != byte) {
            ++mutation_sequence_;
            mutation_journal_[(mutation_sequence_ - 1U) % mutation_journal_capacity] = BackedMutation{
                mutation_sequence_, address + index, region->bytes[offset + index],
                context.pc == 0U,
            };
            ++side_effect_generation_;
        }
        region->bytes[offset + index] = byte;
    }
    if (region->info.executable) {
        ++execution_generation_;
    }
    return std::uint64_t{0};
}

MemoryResult<std::uint8_t> MemoryBus::read8(
    const std::uint32_t address,
    const AccessContext context
) const {
    auto result = read(address, AccessSize::byte, context, 0);
    if (!result) return result.fault();
    return static_cast<std::uint8_t>(result.value());
}

MemoryResult<std::uint16_t> MemoryBus::read16(
    const std::uint32_t address,
    const AccessContext context
) const {
    auto result = read(address, AccessSize::halfword, context, 0);
    if (!result) return result.fault();
    return static_cast<std::uint16_t>(result.value());
}

MemoryResult<std::uint32_t> MemoryBus::read32(
    const std::uint32_t address,
    const AccessContext context
) const {
    const Region* region = find(address);
    if (region != nullptr
        && (region->info.kind == RegionKind::ram
            || region->info.kind == RegionKind::rom
            || region->info.kind == RegionKind::flash)
        && region->info.readable
        && region->containsRange(address, sizeof(std::uint32_t))
        && (context.type != AccessType::instruction_fetch
            || region->info.executable)) {
        if (context.type == AccessType::data_read && context.pc != 0U) {
            addReadFootprint(read_footprint_, address);
        }
        std::uint32_t value = 0U;
        const std::size_t offset = address - region->info.base;
        std::memcpy(&value, region->bytes.data() + offset, sizeof(value));
        if constexpr (std::endian::native == std::endian::big) {
            value = ((value & 0x000000ffU) << 24U)
                | ((value & 0x0000ff00U) << 8U)
                | ((value & 0x00ff0000U) >> 8U)
                | ((value & 0xff000000U) >> 24U);
        }
        return value;
    }

    auto result = read(address, AccessSize::word, context, 0);
    if (!result) return result.fault();
    return static_cast<std::uint32_t>(result.value());
}

MemoryResult<std::uint64_t> MemoryBus::read64(
    const std::uint32_t address,
    const AccessContext context
) const {
    return read(address, AccessSize::doubleword, context, 0);
}

MemoryResult<std::uint64_t> MemoryBus::write8(
    const std::uint32_t address,
    const std::uint8_t value,
    const AccessContext context
) {
    return write(address, AccessSize::byte, value, context, 0);
}

MemoryResult<std::uint64_t> MemoryBus::write16(
    const std::uint32_t address,
    const std::uint16_t value,
    const AccessContext context
) {
    return write(address, AccessSize::halfword, value, context, 0);
}

MemoryResult<std::uint64_t> MemoryBus::write32(
    const std::uint32_t address,
    const std::uint32_t value,
    const AccessContext context
) {
    Region* region = find(address);
    if (region != nullptr
        && (region->info.kind == RegionKind::ram
            || region->info.kind == RegionKind::rom
            || region->info.kind == RegionKind::flash)
        && region->info.writable
        && region->containsRange(address, sizeof(value))) {
        const std::size_t offset = address - region->info.base;
        for (std::uint32_t index = 0U; index < sizeof(value); ++index) {
            const auto byte = static_cast<std::uint8_t>(value >> (index * 8U));
            if (region->bytes[offset + index] == byte) continue;
            ++mutation_sequence_;
            mutation_journal_[(mutation_sequence_ - 1U) % mutation_journal_capacity] =
                BackedMutation{
                    mutation_sequence_, address + index,
                    region->bytes[offset + index], context.pc == 0U,
                };
            ++side_effect_generation_;
            region->bytes[offset + index] = byte;
        }
        if (region->info.executable) ++execution_generation_;
        return std::uint64_t{0};
    }
    return write(address, AccessSize::word, value, context, 0);
}

MemoryResult<std::uint64_t> MemoryBus::write64(
    const std::uint32_t address,
    const std::uint64_t value,
    const AccessContext context
) {
    return write(address, AccessSize::doubleword, value, context, 0);
}

std::vector<MemoryRegionInfo> MemoryBus::regions() const {
    std::vector<MemoryRegionInfo> result;
    result.reserve(regions_.size());
    for (const auto& region : regions_) {
        result.push_back(region->info);
    }
    return result;
}

void MemoryBus::addReadFootprint(
    ReadFootprint& footprint, const std::uint32_t address
) noexcept {
    const std::uint32_t word = address >> 2U;
    const std::uint32_t first = word * 0x9e3779b1U;
    const std::uint32_t second = (word ^ (word >> 16U)) * 0x85ebca6bU;
    constexpr std::size_t bit_count = ReadFootprint::word_count * 64U;
    for (const std::uint32_t mixed : {first, second}) {
        const std::size_t bit = mixed & (bit_count - 1U);
        footprint.words[bit >> 6U] |= std::uint64_t{1U} << (bit & 63U);
    }
}

bool MemoryBus::footprintContains(
    const ReadFootprint& footprint, const std::uint32_t address
) noexcept {
    const std::uint32_t word = address >> 2U;
    const std::uint32_t first = word * 0x9e3779b1U;
    const std::uint32_t second = (word ^ (word >> 16U)) * 0x85ebca6bU;
    constexpr std::size_t bit_count = ReadFootprint::word_count * 64U;
    for (const std::uint32_t mixed : {first, second}) {
        const std::size_t bit = mixed & (bit_count - 1U);
        if ((footprint.words[bit >> 6U] & (std::uint64_t{1U} << (bit & 63U))) == 0U) {
            return false;
        }
    }
    return true;
}

MemoryBus::ReadFootprint MemoryBus::takeReadFootprint() noexcept {
    ReadFootprint result = read_footprint_;
    read_footprint_ = {};
    return result;
}

MemoryBus::SideEffectCheckpoint MemoryBus::sideEffectCheckpoint() const noexcept {
    return SideEffectCheckpoint{mutation_sequence_, mmio_generation_};
}

bool MemoryBus::sideEffectsRestoredSince(const SideEffectCheckpoint checkpoint) const {
    if (checkpoint.mmio_generation != mmio_generation_) return false;
    if (checkpoint.mutation_sequence == mutation_sequence_) return true;
    if (mutation_sequence_ - checkpoint.mutation_sequence > mutation_journal_capacity) {
        return false;
    }

    for (std::uint64_t sequence = checkpoint.mutation_sequence + 1U;
         sequence <= mutation_sequence_; ++sequence) {
        const BackedMutation& current =
            mutation_journal_[(sequence - 1U) % mutation_journal_capacity];
        if (current.sequence != sequence) return false;
        bool first_for_address = true;
        for (std::uint64_t prior_sequence = checkpoint.mutation_sequence + 1U;
             prior_sequence < sequence; ++prior_sequence) {
            const BackedMutation& prior =
                mutation_journal_[(prior_sequence - 1U) % mutation_journal_capacity];
            if (prior.address == current.address) {
                first_for_address = false;
                break;
            }
        }
        if (!first_for_address) continue;
        const Region* region = find(current.address);
        if (region == nullptr || region->info.kind == RegionKind::mmio
            || region->info.kind == RegionKind::alias) {
            return false;
        }
        const std::size_t offset = current.address - region->info.base;
        if (region->bytes[offset] != current.old_value) return false;
    }
    return true;
}

bool MemoryBus::sideEffectsCompatibleSince(
    const SideEffectCheckpoint checkpoint, const ReadFootprint& footprint
) const {
    if (checkpoint.mmio_generation != mmio_generation_) return false;
    if (checkpoint.mutation_sequence == mutation_sequence_) return true;
    if (checkpoint.mutation_sequence > mutation_sequence_
        || mutation_sequence_ - checkpoint.mutation_sequence
            > mutation_journal_capacity) {
        return false;
    }

    for (std::uint64_t sequence = checkpoint.mutation_sequence + 1U;
         sequence <= mutation_sequence_; ++sequence) {
        const BackedMutation& first =
            mutation_journal_[(sequence - 1U) % mutation_journal_capacity];
        if (first.sequence != sequence) return false;
        bool first_for_address = true;
        bool only_external = first.external;
        for (std::uint64_t prior = checkpoint.mutation_sequence + 1U;
             prior < sequence; ++prior) {
            if (mutation_journal_[(prior - 1U) % mutation_journal_capacity].address
                == first.address) {
                first_for_address = false;
                break;
            }
        }
        if (!first_for_address) continue;
        for (std::uint64_t later = sequence + 1U; later <= mutation_sequence_; ++later) {
            const BackedMutation& mutation =
                mutation_journal_[(later - 1U) % mutation_journal_capacity];
            if (mutation.address == first.address) only_external &= mutation.external;
        }
        const Region* const region = find(first.address);
        if (region == nullptr || region->info.kind == RegionKind::mmio
            || region->info.kind == RegionKind::alias) return false;
        const std::size_t offset = first.address - region->info.base;
        if (region->bytes[offset] != first.old_value
            && (!only_external || footprintContains(footprint, first.address))) {
            return false;
        }
    }
    return true;
}

bool MemoryBus::canRestoreSideEffects(
    const SideEffectCheckpoint checkpoint
) const noexcept {
    return checkpoint.mmio_generation == mmio_generation_
        && checkpoint.mutation_sequence <= mutation_sequence_
        && mutation_sequence_ - checkpoint.mutation_sequence
            <= mutation_journal_capacity;
}

bool MemoryBus::restoreSideEffects(const SideEffectCheckpoint checkpoint) {
    if (!canRestoreSideEffects(checkpoint)) return false;

    bool executable_changed = false;
    for (std::uint64_t sequence = mutation_sequence_;
         sequence > checkpoint.mutation_sequence; --sequence) {
        BackedMutation& mutation =
            mutation_journal_[(sequence - 1U) % mutation_journal_capacity];
        if (mutation.sequence != sequence) return false;
        Region* const region = find(mutation.address);
        if (region == nullptr || region->info.kind == RegionKind::mmio
            || region->info.kind == RegionKind::alias) {
            return false;
        }
        const std::size_t offset = mutation.address - region->info.base;
        region->bytes[offset] = mutation.old_value;
        executable_changed = executable_changed || region->info.executable;
        mutation = {};
    }
    mutation_sequence_ = checkpoint.mutation_sequence;
    ++side_effect_generation_;
    if (executable_changed) ++execution_generation_;
    return true;
}

} // namespace fil::mem
