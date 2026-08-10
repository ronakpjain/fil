#include "fil/elf/elf_loader.hpp"

#include "fil/common/format.hpp"
#include "fil/common/numeric.hpp"

#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <string_view>
#include <utility>

namespace fil::elf {
namespace {

constexpr std::uint16_t elf_type_executable = 2;
constexpr std::uint16_t elf_machine_arm = 40;
constexpr std::uint32_t program_type_load = 1;
constexpr std::uint32_t section_type_symbol_table = 2;
constexpr std::uint32_t section_type_dynamic_symbols = 11;
constexpr std::uint8_t symbol_type_function = 2;

/// @brief Creates a source-associated ELF parse error.
Error elfError(const std::filesystem::path& path, std::string message) {
    return Error{ErrorCategory::parse, std::move(message), SourceContext{path, 0, 0}};
}

/// @brief Checks a target address range without 32-bit wraparound.
bool addressRangeFits(const std::uint32_t address, const std::uint32_t size) {
    return size == 0 || address <= std::numeric_limits<std::uint32_t>::max() - (size - 1U);
}

// ELF structures cannot be cast from the byte buffer: host alignment,
// endianness, and struct padding are not part of the file format. Reader keeps
// every primitive access little-endian and bounds checked.
class Reader {
public:
    /// @brief Creates a checked reader over immutable ELF bytes.
    Reader(const std::span<const std::uint8_t> bytes, const std::filesystem::path& path)
        : bytes_(bytes), path_(path) {}

    /// @brief Reads one byte at a file offset.
    Result<std::uint8_t> u8(const std::size_t offset) const {
        if (!rangeFits(offset, 1, bytes_.size())) {
            return outOfBounds(offset, 1);
        }
        return bytes_[offset];
    }

    /// @brief Reads a little-endian 16-bit value at a file offset.
    Result<std::uint16_t> u16(const std::size_t offset) const {
        if (!rangeFits(offset, 2, bytes_.size())) {
            return outOfBounds(offset, 2);
        }
        return static_cast<std::uint16_t>(bytes_[offset])
            | static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes_[offset + 1]) << 8U);
    }

    /// @brief Reads a little-endian 32-bit value at a file offset.
    Result<std::uint32_t> u32(const std::size_t offset) const {
        if (!rangeFits(offset, 4, bytes_.size())) {
            return outOfBounds(offset, 4);
        }
        return static_cast<std::uint32_t>(bytes_[offset])
            | (static_cast<std::uint32_t>(bytes_[offset + 1]) << 8U)
            | (static_cast<std::uint32_t>(bytes_[offset + 2]) << 16U)
            | (static_cast<std::uint32_t>(bytes_[offset + 3]) << 24U);
    }

    /// @brief Returns a checked view over a file byte range.
    Result<std::span<const std::uint8_t>> bytes(const std::size_t offset, const std::size_t size) const {
        if (!rangeFits(offset, size, bytes_.size())) {
            return outOfBounds(offset, size);
        }
        return bytes_.subspan(offset, size);
    }

private:
    /// @brief Builds a diagnostic for an invalid file range.
    Error outOfBounds(const std::size_t offset, const std::size_t size) const {
        return elfError(
            path_,
            "ELF read outside file at offset " + std::to_string(offset)
                + " for " + std::to_string(size) + " bytes"
        );
    }

    std::span<const std::uint8_t> bytes_;
    const std::filesystem::path& path_;
};

struct Section {
    std::uint32_t name_offset{0};
    std::uint32_t type{0};
    std::uint32_t offset{0};
    std::uint32_t size{0};
    std::uint32_t link{0};
    std::uint32_t entry_size{0};
    std::string name;
};

/// @brief Reads an entire ELF file into owned byte storage.
Result<std::vector<std::uint8_t>> readFile(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return Error{ErrorCategory::io, "unable to open ELF file", SourceContext{path, 0, 0}};
    }
    input.seekg(0, std::ios::end);
    const std::streamoff end = input.tellg();
    if (end < 0) {
        return Error{ErrorCategory::io, "unable to determine ELF file size", SourceContext{path, 0, 0}};
    }
    if (static_cast<std::uintmax_t>(end) > std::numeric_limits<std::size_t>::max()) {
        return Error{ErrorCategory::io, "ELF file is too large for this host", SourceContext{path, 0, 0}};
    }
    input.seekg(0, std::ios::beg);
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(end));
    if (!bytes.empty()) {
        input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    }
    if (!input) {
        return Error{ErrorCategory::io, "failed while reading ELF file", SourceContext{path, 0, 0}};
    }
    return bytes;
}

/// @brief Reads a null-terminated string from an ELF string table.
Result<std::string> stringAt(
    const std::span<const std::uint8_t> table,
    const std::uint32_t offset,
    const std::filesystem::path& path,
    const std::string_view description
) {
    if (offset >= table.size()) {
        return elfError(path, std::string(description) + " string offset is outside its string table");
    }
    std::size_t end = offset;
    while (end < table.size() && table[end] != 0) {
        ++end;
    }
    if (end == table.size()) {
        return elfError(path, std::string(description) + " string is not null terminated");
    }
    return std::string(
        reinterpret_cast<const char*>(table.data() + offset),
        end - static_cast<std::size_t>(offset)
    );
}

/// @brief Parses and names all ELF32 section headers.
Result<std::vector<Section>> parseSections(
    const Reader& reader,
    const std::span<const std::uint8_t> file,
    const std::filesystem::path& path,
    const std::uint32_t table_offset,
    const std::uint16_t entry_size,
    const std::uint16_t count,
    const std::uint16_t names_index
) {
    if (count == 0) {
        return std::vector<Section>{};
    }
    if (entry_size < 40) {
        return elfError(path, "ELF section-header entry is smaller than ELF32 requires");
    }
    const std::uint64_t table_size = static_cast<std::uint64_t>(entry_size) * count;
    if (table_size > std::numeric_limits<std::size_t>::max()
        || !rangeFits(table_offset, static_cast<std::size_t>(table_size), file.size())) {
        return elfError(path, "ELF section-header table is outside the file");
    }
    if (names_index >= count) {
        return elfError(path, "ELF section-name string-table index is out of range");
    }

    std::vector<Section> sections;
    sections.reserve(count);
    for (std::uint16_t index = 0; index < count; ++index) {
        const std::size_t offset = static_cast<std::size_t>(table_offset)
            + static_cast<std::size_t>(index) * entry_size;
        auto name = reader.u32(offset);
        auto type = reader.u32(offset + 4);
        auto section_offset = reader.u32(offset + 16);
        auto size = reader.u32(offset + 20);
        auto link = reader.u32(offset + 24);
        auto section_entry_size = reader.u32(offset + 36);
        if (!name) return name.error();
        if (!type) return type.error();
        if (!section_offset) return section_offset.error();
        if (!size) return size.error();
        if (!link) return link.error();
        if (!section_entry_size) return section_entry_size.error();
        if (!rangeFits(section_offset.value(), size.value(), file.size())) {
            return elfError(path, "ELF section " + std::to_string(index) + " is outside the file");
        }
        sections.push_back(Section{
            name.value(), type.value(), section_offset.value(), size.value(), link.value(),
            section_entry_size.value(), {},
        });
    }

    const Section& names = sections[names_index];
    auto names_table = reader.bytes(names.offset, names.size);
    if (!names_table) {
        return names_table.error();
    }
    for (Section& section : sections) {
        auto name = stringAt(names_table.value(), section.name_offset, path, "section name");
        if (!name) {
            return name.error();
        }
        section.name = std::move(name).value();
    }
    return sections;
}

/// @brief Parses linked symbol and string tables into sorted symbols.
Result<std::vector<ElfSymbol>> parseSymbols(
    const Reader& reader,
    const std::vector<Section>& sections,
    const std::filesystem::path& path
) {
    std::vector<ElfSymbol> symbols;
    for (const Section& section : sections) {
        if (section.type != section_type_symbol_table && section.type != section_type_dynamic_symbols) {
            continue;
        }
        if (section.entry_size < 16 || section.size % section.entry_size != 0) {
            return elfError(path, "symbol table '" + section.name + "' has an invalid entry size");
        }
        if (section.link >= sections.size()) {
            return elfError(path, "symbol table '" + section.name + "' has an invalid string-table link");
        }
        const Section& strings = sections[section.link];
        auto string_table = reader.bytes(strings.offset, strings.size);
        if (!string_table) {
            return string_table.error();
        }

        const std::uint32_t count = section.size / section.entry_size;
        for (std::uint32_t index = 0; index < count; ++index) {
            const std::size_t offset = static_cast<std::size_t>(section.offset)
                + static_cast<std::size_t>(index) * section.entry_size;
            auto name_offset = reader.u32(offset);
            auto value = reader.u32(offset + 4);
            auto size = reader.u32(offset + 8);
            auto info = reader.u8(offset + 12);
            if (!name_offset) return name_offset.error();
            if (!value) return value.error();
            if (!size) return size.error();
            if (!info) return info.error();
            if (name_offset.value() == 0) {
                continue;
            }
            auto name = stringAt(string_table.value(), name_offset.value(), path, "symbol name");
            if (!name) {
                return name.error();
            }
            if (name.value().empty()) {
                continue;
            }
            if (name.value().size() >= 2U && name.value()[0] == '$'
                && (name.value()[1] == 'a' || name.value()[1] == 'd' || name.value()[1] == 't')
                && (name.value().size() == 2U || name.value()[2] == '.')) {
                continue; // ARM mapping symbols are disassembly metadata, not user diagnostics.
            }
            const bool function = (info.value() & 0x0fU) == symbol_type_function;
            symbols.push_back(ElfSymbol{
                function ? (value.value() & ~1U) : value.value(), size.value(),
                std::move(name).value(), function,
            });
        }
    }

    std::sort(symbols.begin(), symbols.end(), [](const ElfSymbol& left, const ElfSymbol& right) {
        if (left.address != right.address) return left.address < right.address;
        if (left.function != right.function) return left.function > right.function;
        return left.name < right.name;
    });
    symbols.erase(
        std::unique(symbols.begin(), symbols.end(), [](const ElfSymbol& left, const ElfSymbol& right) {
            return left.address == right.address && left.name == right.name;
        }),
        symbols.end()
    );
    return symbols;
}

/// @brief Reads a known-in-bounds little-endian 32-bit value.
std::uint32_t rawU32(const std::span<const std::uint8_t> bytes, const std::size_t offset) {
    return static_cast<std::uint32_t>(bytes[offset])
        | (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U)
        | (static_cast<std::uint32_t>(bytes[offset + 2]) << 16U)
        | (static_cast<std::uint32_t>(bytes[offset + 3]) << 24U);
}

/// @brief Decodes one bounded unsigned LEB128 value and advances the cursor.
bool readUleb(
    const std::span<const std::uint8_t> bytes,
    std::size_t& cursor,
    const std::size_t end,
    std::uint32_t& result
) {
    result = 0;
    unsigned int shift = 0;
    while (cursor < end && shift < 35U) {
        const std::uint8_t byte = bytes[cursor++];
        result |= static_cast<std::uint32_t>(byte & 0x7fU) << shift;
        if ((byte & 0x80U) == 0) {
            return true;
        }
        shift += 7U;
    }
    return false;
}

/// @brief Advances past a bounded null-terminated string.
bool skipCString(const std::span<const std::uint8_t> bytes, std::size_t& cursor, const std::size_t end) {
    while (cursor < end) {
        if (bytes[cursor++] == 0) {
            return true;
        }
    }
    return false;
}

/// @brief Identifies ARM build attributes whose payload is a string.
bool isStringAttribute(const std::uint32_t tag) {
    return tag == 4 || tag == 5 || tag == 65 || tag == 67;
}

/// @brief Records recognized numeric ARM attributes.
void recordAttribute(ArmAttributes& attributes, const std::uint32_t tag, const std::uint32_t value) {
    switch (tag) {
    case 6:
        attributes.cpu_arch = value;
        break;
    case 9:
        attributes.thumb_isa_use = value;
        break;
    case 10:
        attributes.fp_arch = value;
        break;
    case 27:
        attributes.hard_fp_use = value;
        break;
    case 28:
        attributes.vfp_args = value;
        break;
    case 34:
        attributes.unaligned_access = value;
        break;
    default:
        break;
    }
}

/**
 * @brief Parses recognized values from the nested `.ARM.attributes` format.
 *
 * Vendor sections contain tagged subsections, which contain ULEB128 tag/value
 * pairs. Unknown values are consumed but not exposed by the initial public API.
 */
void parseArmAttributes(
    const std::span<const std::uint8_t> bytes,
    ArmAttributes& attributes
) {
    if (bytes.empty() || bytes.front() != static_cast<std::uint8_t>('A')) {
        return;
    }
    std::size_t cursor = 1;
    while (rangeFits(cursor, 4, bytes.size())) {
        const std::size_t vendor_start = cursor;
        const std::uint32_t vendor_size = rawU32(bytes, cursor);
        if (vendor_size < 5 || !rangeFits(vendor_start, vendor_size, bytes.size())) {
            return;
        }
        const std::size_t vendor_end = vendor_start + vendor_size;
        cursor += 4;
        const std::size_t vendor_name_start = cursor;
        if (!skipCString(bytes, cursor, vendor_end)) {
            return;
        }
        const std::string_view vendor(
            reinterpret_cast<const char*>(bytes.data() + vendor_name_start),
            cursor - vendor_name_start - 1
        );
        if (vendor != "aeabi") {
            cursor = vendor_end;
            continue;
        }

        while (cursor < vendor_end) {
            const std::size_t subsection_start = cursor;
            std::uint32_t subsection_tag = 0;
            if (!readUleb(bytes, cursor, vendor_end, subsection_tag)
                || !rangeFits(cursor, 4, vendor_end)) {
                return;
            }
            const std::uint32_t subsection_size = rawU32(bytes, cursor);
            if (subsection_size < cursor + 4 - subsection_start
                || !rangeFits(subsection_start, subsection_size, vendor_end)) {
                return;
            }
            const std::size_t subsection_end = subsection_start + subsection_size;
            cursor += 4;
            if (subsection_tag != 1) {
                cursor = subsection_end;
                continue;
            }

            while (cursor < subsection_end) {
                std::uint32_t tag = 0;
                if (!readUleb(bytes, cursor, subsection_end, tag)) {
                    return;
                }
                if (isStringAttribute(tag)) {
                    const std::size_t string_start = cursor;
                    if (!skipCString(bytes, cursor, subsection_end)) {
                        return;
                    }
                    if (tag == 5) {
                        attributes.cpu_name = std::string(
                            reinterpret_cast<const char*>(bytes.data() + string_start),
                            cursor - string_start - 1
                        );
                    }
                } else if (tag == 32) {
                    std::uint32_t compatibility = 0;
                    if (!readUleb(bytes, cursor, subsection_end, compatibility)
                        || !skipCString(bytes, cursor, subsection_end)) {
                        return;
                    }
                } else {
                    std::uint32_t value = 0;
                    if (!readUleb(bytes, cursor, subsection_end, value)) {
                        return;
                    }
                    recordAttribute(attributes, tag, value);
                }
            }
        }
        cursor = vendor_end;
    }
}

/**
 * @brief Validates that overlapping PT_LOAD records contain identical bytes.
 *
 * Linkers may emit adjacent or overlapping records; differing bytes at the same
 * physical address would make the load image ambiguous and are rejected.
 */
Result<void> validateFileBackedOverlap(
    const ElfLoadSegment& candidate,
    const std::vector<ElfLoadSegment>& existing,
    const std::span<const std::uint8_t> file,
    const std::filesystem::path& path
) {
    const std::uint64_t candidate_end = static_cast<std::uint64_t>(candidate.physical_address)
        + candidate.file_size;
    for (std::size_t index = 0; index < existing.size(); ++index) {
        const ElfLoadSegment& other = existing[index];
        const std::uint64_t other_end = static_cast<std::uint64_t>(other.physical_address)
            + other.file_size;
        const std::uint64_t overlap_start = std::max<std::uint64_t>(candidate.physical_address, other.physical_address);
        const std::uint64_t overlap_end = std::min(candidate_end, other_end);
        if (overlap_start >= overlap_end) {
            continue;
        }
        for (std::uint64_t address = overlap_start; address < overlap_end; ++address) {
            const std::size_t candidate_offset = candidate.file_offset
                + static_cast<std::size_t>(address - candidate.physical_address);
            const std::size_t other_offset = other.file_offset
                + static_cast<std::size_t>(address - other.physical_address);
            if (file[candidate_offset] != file[other_offset]) {
                return elfError(
                    path,
                    "PT_LOAD file ranges overlap with different bytes at load address "
                        + hexValue(address) + " (new segment and segment "
                        + std::to_string(index) + ")"
                );
            }
        }
    }
    return {};
}

/// @brief Formats ELF segment permission flags as RWX text.
std::string permissions(const ElfLoadSegment& segment) {
    std::string result = "---";
    if (segment.readable) result[0] = 'R';
    if (segment.writable) result[1] = 'W';
    if (segment.executable) result[2] = 'X';
    return result;
}

} // namespace

Result<ElfImage> load(
    const std::filesystem::path& path,
    const std::optional<std::uint32_t> vector_base
) {
    auto bytes = readFile(path);
    if (!bytes) {
        return bytes.error();
    }
    if (bytes.value().size() < 52) {
        return elfError(path, "file is smaller than an ELF32 header");
    }

    Reader reader(bytes.value(), path);
    const auto& file = bytes.value();
    if (file[0] != 0x7fU || file[1] != 'E' || file[2] != 'L' || file[3] != 'F') {
        return elfError(path, "invalid ELF magic");
    }
    if (file[4] != 1) return elfError(path, "only ELF32 files are supported");
    if (file[5] != 1) return elfError(path, "only little-endian ELF files are supported");
    if (file[6] != 1) return elfError(path, "unsupported ELF identification version");

    auto type = reader.u16(16);
    auto machine = reader.u16(18);
    auto version = reader.u32(20);
    auto entry = reader.u32(24);
    auto program_offset = reader.u32(28);
    auto section_offset = reader.u32(32);
    auto flags = reader.u32(36);
    auto header_size = reader.u16(40);
    auto program_entry_size = reader.u16(42);
    auto program_count = reader.u16(44);
    auto section_entry_size = reader.u16(46);
    auto section_count = reader.u16(48);
    auto section_names = reader.u16(50);
    if (!type) return type.error();
    if (!machine) return machine.error();
    if (!version) return version.error();
    if (!entry) return entry.error();
    if (!program_offset) return program_offset.error();
    if (!section_offset) return section_offset.error();
    if (!flags) return flags.error();
    if (!header_size) return header_size.error();
    if (!program_entry_size) return program_entry_size.error();
    if (!program_count) return program_count.error();
    if (!section_entry_size) return section_entry_size.error();
    if (!section_count) return section_count.error();
    if (!section_names) return section_names.error();

    if (type.value() != elf_type_executable) return elfError(path, "ELF is not an executable image");
    if (machine.value() != elf_machine_arm) return elfError(path, "ELF machine is not ARM");
    if (version.value() != 1) return elfError(path, "unsupported ELF header version");
    if (header_size.value() < 52) return elfError(path, "ELF header size is smaller than ELF32 requires");
    if (program_count.value() != 0 && program_entry_size.value() < 32) {
        return elfError(path, "ELF program-header entry is smaller than ELF32 requires");
    }
    const std::uint64_t program_table_size = static_cast<std::uint64_t>(program_entry_size.value())
        * program_count.value();
    if (program_table_size > std::numeric_limits<std::size_t>::max()
        || !rangeFits(program_offset.value(), static_cast<std::size_t>(program_table_size), file.size())) {
        return elfError(path, "ELF program-header table is outside the file");
    }

    ElfImage image;
    image.path_ = path;
    image.file_bytes_ = std::move(bytes).value();
    image.entry_point_ = entry.value();
    image.flags_ = flags.value();

    // Program headers are authoritative for loading. Sections are parsed later
    // only for names, symbols, and attributes; stripped ELFs remain loadable.
    for (std::uint16_t index = 0; index < program_count.value(); ++index) {
        const std::size_t offset = static_cast<std::size_t>(program_offset.value())
            + static_cast<std::size_t>(index) * program_entry_size.value();
        auto segment_type = reader.u32(offset);
        if (!segment_type) return segment_type.error();
        if (segment_type.value() != program_type_load) {
            continue;
        }
        auto file_offset = reader.u32(offset + 4);
        auto virtual_address = reader.u32(offset + 8);
        auto physical_address = reader.u32(offset + 12);
        auto file_size = reader.u32(offset + 16);
        auto memory_size = reader.u32(offset + 20);
        auto segment_flags = reader.u32(offset + 24);
        if (!file_offset) return file_offset.error();
        if (!virtual_address) return virtual_address.error();
        if (!physical_address) return physical_address.error();
        if (!file_size) return file_size.error();
        if (!memory_size) return memory_size.error();
        if (!segment_flags) return segment_flags.error();
        if (file_size.value() > memory_size.value()) {
            return elfError(path, "PT_LOAD " + std::to_string(index) + " has p_filesz greater than p_memsz");
        }
        if (!rangeFits(file_offset.value(), file_size.value(), image.file_bytes_.size())) {
            return elfError(path, "PT_LOAD " + std::to_string(index) + " file range is outside the ELF");
        }
        if (!addressRangeFits(physical_address.value(), file_size.value())
            || !addressRangeFits(virtual_address.value(), memory_size.value())) {
            return elfError(path, "PT_LOAD " + std::to_string(index) + " address range wraps 32-bit space");
        }

        // Preserve p_paddr and p_vaddr separately. In a typical MCU image the
        // .data bytes are stored at p_paddr in flash and copied to p_vaddr SRAM
        // by reset code; pre-populating SRAM here would hide startup bugs.
        ElfLoadSegment segment{
            virtual_address.value(), physical_address.value(), memory_size.value(), file_size.value(),
            file_offset.value(), (segment_flags.value() & 4U) != 0, (segment_flags.value() & 2U) != 0,
            (segment_flags.value() & 1U) != 0,
        };
        auto overlap = validateFileBackedOverlap(segment, image.segments_, image.file_bytes_, path);
        if (!overlap) return overlap.error();
        image.segments_.push_back(segment);
    }
    if (image.segments_.empty()) {
        return elfError(path, "ELF contains no PT_LOAD segments");
    }
    std::sort(image.segments_.begin(), image.segments_.end(), [](const auto& left, const auto& right) {
        if (left.physical_address != right.physical_address) {
            return left.physical_address < right.physical_address;
        }
        return left.virtual_address < right.virtual_address;
    });

    // Without an override, the first executable file-backed segment is the
    // least surprising vector-table candidate and supports offset applications.
    image.vector_base_ = vector_base.value_or([&image] {
        for (const ElfLoadSegment& segment : image.segments_) {
            if (segment.executable && segment.file_size >= 8) {
                return segment.physical_address;
            }
        }
        return image.segments_.front().physical_address;
    }());
    auto vectors = image.readLoadImage(image.vector_base_, 8);
    if (!vectors) {
        return elfError(path, "vector table is not contained in a file-backed PT_LOAD segment: " + vectors.error().message);
    }
    const auto vector_bytes = std::span<const std::uint8_t>(vectors.value());
    image.initial_msp_ = rawU32(vector_bytes, 0);
    image.reset_handler_ = rawU32(vector_bytes, 4);

    auto sections = parseSections(
        reader, image.file_bytes_, path, section_offset.value(), section_entry_size.value(),
        section_count.value(), section_names.value()
    );
    if (!sections) {
        return sections.error();
    }
    auto symbols = parseSymbols(reader, sections.value(), path);
    if (!symbols) {
        return symbols.error();
    }
    image.symbols_ = std::move(symbols).value();
    for (const Section& section : sections.value()) {
        if (section.name == ".ARM.attributes") {
            auto attribute_bytes = reader.bytes(section.offset, section.size);
            if (!attribute_bytes) return attribute_bytes.error();
            parseArmAttributes(attribute_bytes.value(), image.attributes_);
        }
    }

    return image;
}

Result<std::vector<std::uint8_t>> ElfImage::readLoadImage(
    const std::uint32_t address,
    const std::uint32_t size
) const {
    if (!addressRangeFits(address, size)) {
        return Error{ErrorCategory::invalid_argument, "requested load-image range wraps 32-bit space", std::nullopt};
    }
    for (const ElfLoadSegment& segment : segments_) {
        const std::uint64_t segment_end = static_cast<std::uint64_t>(segment.physical_address)
            + segment.file_size;
        const std::uint64_t requested_end = static_cast<std::uint64_t>(address) + size;
        if (address < segment.physical_address || requested_end > segment_end) {
            continue;
        }
        const std::size_t offset = segment.file_offset
            + static_cast<std::size_t>(address - segment.physical_address);
        return std::vector<std::uint8_t>(
            file_bytes_.begin() + static_cast<std::ptrdiff_t>(offset),
            file_bytes_.begin() + static_cast<std::ptrdiff_t>(offset + size)
        );
    }
    return Error{ErrorCategory::invalid_argument, "requested range is not file-backed load memory", std::nullopt};
}

const ElfSymbol* ElfImage::symbolAtOrBefore(const std::uint32_t address) const noexcept {
    // Symbols are sorted during loading, so lookup stays logarithmic even for
    // large debug builds with many local symbols.
    const auto iterator = std::upper_bound(
        symbols_.begin(), symbols_.end(), address,
        [](const std::uint32_t value, const ElfSymbol& symbol) { return value < symbol.address; }
    );
    if (iterator == symbols_.begin()) {
        return nullptr;
    }
    return &*std::prev(iterator);
}

std::string inspect(const ElfImage& image, const std::filesystem::path& path) {
    std::ostringstream output;
    output << "ELF: " << path.string() << '\n'
           << "class: ELF32\n"
           << "endian: little\n"
           << "machine: ARM\n"
           << "entry: " << hex32(image.entryPoint()) << '\n'
           << "abi: EABI" << ((image.flags() >> 24U) & 0xffU)
           << (image.hardFloatAbi() ? " hard-float" : " soft-float-or-unspecified") << '\n'
           << "load segments:\n";
    for (const ElfLoadSegment& segment : image.segments()) {
        output << "  vaddr=" << hex32(segment.virtual_address)
               << " paddr=" << hex32(segment.physical_address)
               << " mem=" << segment.memory_size
               << " file=" << segment.file_size
               << " offset=" << hex32(segment.file_offset)
               << " " << permissions(segment) << '\n';
    }
    output << "vector table: " << hex32(image.vectorBase()) << '\n'
           << "initial MSP: " << hex32(image.initialMsp()) << '\n'
           << "reset handler: " << hex32(image.resetHandler()) << '\n'
           << "symbols: " << image.symbols().size() << '\n';

    const ArmAttributes& attributes = image.armAttributes();
    if (attributes.cpu_name.has_value()) output << "ARM CPU name: " << *attributes.cpu_name << '\n';
    if (attributes.cpu_arch.has_value()) output << "ARM CPU arch tag: " << *attributes.cpu_arch << '\n';
    if (attributes.thumb_isa_use.has_value()) output << "Thumb ISA tag: " << *attributes.thumb_isa_use << '\n';
    if (attributes.fp_arch.has_value()) output << "FP arch tag: " << *attributes.fp_arch << '\n';
    if (attributes.hard_fp_use.has_value()) output << "HardFP use tag: " << *attributes.hard_fp_use << '\n';
    if (attributes.vfp_args.has_value()) output << "VFP args tag: " << *attributes.vfp_args << '\n';
    if (attributes.unaligned_access.has_value()) output << "Unaligned access tag: " << *attributes.unaligned_access << '\n';
    return output.str();
}

} // namespace fil::elf
