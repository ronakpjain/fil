#include "fil/sim/board.hpp"
#include "../test_support.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string_view>

namespace {

std::optional<std::uint32_t> hardFloatSymbol(
    const fil::elf::ElfImage& image,
    const std::string_view name
) {
    for (const auto& symbol : image.symbols()) {
        if (symbol.name == name) return symbol.address;
    }
    return std::nullopt;
}

std::optional<std::uint32_t> readHardFloatWord(
    fil::sim::Board& board,
    const std::string_view name
) {
    const auto address = hardFloatSymbol(board.image(), name);
    if (!address) return std::nullopt;
    const auto value = board.memory().read32(*address);
    if (!value) return std::nullopt;
    return value.value();
}

void executesCompiledHardFloatFirmware() {
    fil::config::BoardConfig config;
    config.name = "hard-float-firmware";
    config.mcu_path = std::filesystem::path(FIL_SOURCE_DIR)
        / "configs/mcus/stm32g474retx.json";
    config.elf_path = std::filesystem::path(FIL_SOURCE_DIR)
        / "tests/fixtures/elf/hard_float.elf";
    config.vector_base = 0x08000000U;

    auto loaded = fil::sim::Board::load(config, true);
    fil::test::check(loaded.hasValue(), "loads compiled Cortex-M4F hard-float fixture");
    if (!loaded) return;
    auto& board = *loaded.value();

    const auto& image = board.image();
    const auto& attributes = image.armAttributes();
    fil::test::check(image.hardFloatAbi(), "fixture ELF declares the hard-float ABI");
    fil::test::check(attributes.fp_arch == 6U,
                     "fixture declares the VFPv4-D16 architecture");
    fil::test::check(attributes.hard_fp_use == 1U && attributes.vfp_args == 1U,
                     "fixture declares scalar HardFP use and VFP argument passing");

    const auto results = hardFloatSymbol(image, "hard_float_results");
    fil::test::check(results.has_value(), "hard-float fixture exposes its result array");
    if (!results) return;

    fil::sim::BoardRunOptions options;
    options.max_instructions = 128U;
    options.duration_ns = 0U;
    options.detect_spin = false;
    const auto result = board.run(options);

    fil::test::check(result.reason != fil::sim::BoardStopReason::unimplemented_instruction,
                     "compiled hard-float firmware uses no unsupported instruction");
    fil::test::check(result.reason == fil::sim::BoardStopReason::breakpoint,
                     "compiled hard-float firmware completes and stops at BKPT");

    const std::array<std::uint32_t, 4> expected_float_bits{
        0x41700000U, // (1.5 + 2.25) * 4.0 = 15.0
        0x40100000U, // 9.0 / 4.0 = 2.25
        0xc2140000U, // signed -37 converted to float
        0x42280000U, // unsigned 42 converted to float
    };
    for (std::size_t index = 0; index < expected_float_bits.size(); ++index) {
        const auto value = board.memory().read32(
            *results + static_cast<std::uint32_t>(index * sizeof(std::uint32_t))
        );
        fil::test::check(value && value.value() == expected_float_bits[index],
                         "compiled scalar VFP result has exact binary32 bits");
    }
    fil::test::check(readHardFloatWord(board, "hard_signed_result") == 0xfffffff4U,
                     "float-to-signed conversion truncates -12.75 to -12");
    fil::test::check(readHardFloatWord(board, "hard_unsigned_result") == 255U,
                     "float-to-unsigned conversion truncates 255.75 to 255");
    fil::test::check(readHardFloatWord(board, "hard_float_complete") == 0xf00dcafeU,
                     "hard-float main reaches its completion sentinel");
}

} // namespace

void runHardFloatFirmwareTests() {
    executesCompiledHardFloatFirmware();
}
