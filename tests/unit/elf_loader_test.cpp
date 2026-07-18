#include "fil/elf/elf_loader.hpp"
#include "../test_support.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <vector>

namespace {

const std::filesystem::path fixture_path =
    std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";

/// @brief Reads the committed synthetic ELF fixture.
std::vector<std::uint8_t> fixtureBytes() {
    std::ifstream input(fixture_path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

/// @brief Writes a mutated ELF image for a negative parser test.
std::filesystem::path writeTemporaryElf(
    const std::string& name,
    const std::vector<std::uint8_t>& bytes
) {
    const auto directory = std::filesystem::temp_directory_path() / "fil-elf-tests";
    std::filesystem::create_directories(directory);
    const auto path = directory / name;
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    return path;
}

/// @brief Writes a little-endian 32-bit value into fixture bytes.
void writeU32(std::vector<std::uint8_t>& bytes, const std::size_t offset, const std::uint32_t value) {
    bytes[offset] = static_cast<std::uint8_t>(value);
    bytes[offset + 1] = static_cast<std::uint8_t>(value >> 8U);
    bytes[offset + 2] = static_cast<std::uint8_t>(value >> 16U);
    bytes[offset + 3] = static_cast<std::uint8_t>(value >> 24U);
}

/// @brief Verifies split physical/load and virtual/runtime addresses.
void loadsSplitAddressImage() {
    const auto image = fil::elf::load(fixture_path);
    fil::test::check(image.hasValue(), "loads synthetic ELF fixture");
    if (!image) {
        return;
    }

    fil::test::check(image.value().entryPoint() == 0x08000009U, "reads ELF entry point");
    fil::test::check(image.value().initialMsp() == 0x20020000U, "reads initial MSP");
    fil::test::check(image.value().resetHandler() == 0x08000009U, "reads reset vector");
    fil::test::check(image.value().hardFloatAbi(), "reads hard-float EABI flag");
    fil::test::check(image.value().segments().size() == 3, "reads all PT_LOAD segments");

    const auto& data_segment = image.value().segments()[1];
    fil::test::check(data_segment.physical_address == 0x08000018U, "preserves .data load address");
    fil::test::check(data_segment.virtual_address == 0x20000000U, "preserves .data runtime address");
    const auto initializer = image.value().readLoadImage(0x08000018U, 4);
    fil::test::check(
        initializer && initializer.value() == std::vector<std::uint8_t>({0x78, 0x56, 0x34, 0x12}),
        "reads .data initializer from flash load address"
    );
    fil::test::check(
        !image.value().readLoadImage(0x20000000U, 4),
        "does not confuse .data runtime address with load address"
    );
}

/// @brief Verifies symbol and ARM build-attribute parsing.
void loadsSymbolsAndAttributes() {
    const auto image = fil::elf::load(fixture_path);
    if (!image) {
        fil::test::check(false, "fixture is available for metadata test");
        return;
    }

    const fil::elf::ElfSymbol* reset = image.value().symbolAtOrBefore(0x0800000aU);
    fil::test::check(reset != nullptr && reset->name == "Reset_Handler", "finds nearest function symbol");
    fil::test::check(reset != nullptr && reset->function, "preserves function symbol type");
    fil::test::check(reset != nullptr && reset->address == 0x08000008U,
                     "normalizes the Thumb bit in function symbol addresses");
    fil::test::check(
        image.value().armAttributes().cpu_name == std::optional<std::string>("Cortex-M4"),
        "reads ARM CPU name attribute"
    );
    fil::test::check(image.value().armAttributes().thumb_isa_use == 2U, "reads Thumb-2 attribute");
    fil::test::check(image.value().armAttributes().fp_arch == 6U, "reads FPv4-D16 attribute");
    fil::test::check(image.value().armAttributes().vfp_args == 1U, "reads VFP argument attribute");
}

/// @brief Verifies malformed and ambiguous ELF images are rejected.
void rejectsMalformedImages() {
    auto bad_magic_bytes = fixtureBytes();
    bad_magic_bytes[0] = 0;
    const auto bad_magic = fil::elf::load(writeTemporaryElf("bad-magic.elf", bad_magic_bytes));

    auto bad_sizes_bytes = fixtureBytes();
    // ELF32 e_phoff is at 28; p_filesz/p_memsz are at +16/+20 in the first entry.
    const std::uint32_t program_headers = static_cast<std::uint32_t>(bad_sizes_bytes[28])
        | (static_cast<std::uint32_t>(bad_sizes_bytes[29]) << 8U)
        | (static_cast<std::uint32_t>(bad_sizes_bytes[30]) << 16U)
        | (static_cast<std::uint32_t>(bad_sizes_bytes[31]) << 24U);
    writeU32(bad_sizes_bytes, program_headers + 16U, 64U);
    writeU32(bad_sizes_bytes, program_headers + 20U, 32U);
    const auto bad_sizes = fil::elf::load(writeTemporaryElf("bad-sizes.elf", bad_sizes_bytes));

    auto overlap_bytes = fixtureBytes();
    writeU32(overlap_bytes, program_headers + 32U + 12U, 0x08000000U);
    const auto overlap = fil::elf::load(writeTemporaryElf("overlap.elf", overlap_bytes));

    auto wrap_bytes = fixtureBytes();
    writeU32(wrap_bytes, program_headers + 32U + 12U, 0xfffffffeU);
    const auto wrap = fil::elf::load(writeTemporaryElf("wrap.elf", wrap_bytes));

    std::vector<std::uint8_t> truncated(20, 0);
    const auto short_file = fil::elf::load(writeTemporaryElf("truncated.elf", truncated));
    const auto bad_vector = fil::elf::load(fixture_path, 0x08010000U);

    fil::test::check(!bad_magic && bad_magic.error().message.find("magic") != std::string::npos, "rejects bad ELF magic");
    fil::test::check(!bad_sizes && bad_sizes.error().message.find("p_filesz") != std::string::npos, "rejects p_filesz greater than p_memsz");
    fil::test::check(!overlap && overlap.error().message.find("overlap") != std::string::npos, "rejects conflicting load-address overlap");
    fil::test::check(!wrap && wrap.error().message.find("wraps") != std::string::npos, "rejects target address wraparound");
    fil::test::check(!short_file && short_file.error().message.find("smaller") != std::string::npos, "rejects truncated ELF headers");
    fil::test::check(!bad_vector && bad_vector.error().message.find("vector table") != std::string::npos, "rejects vector bases outside load memory");

    std::error_code error;
    std::filesystem::remove_all(std::filesystem::temp_directory_path() / "fil-elf-tests", error);
}

} // namespace

/// @brief Runs all ELF loader unit-test cases.
void runElfLoaderTests() {
    loadsSplitAddressImage();
    loadsSymbolsAndAttributes();
    rejectsMalformedImages();
}
