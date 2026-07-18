#include "fil/sim/board.hpp"
#include "../test_support.hpp"

#include <filesystem>
#include <optional>
#include <string_view>

namespace {

std::optional<std::uint32_t> symbolAddress(
    const fil::elf::ElfImage& image,
    const std::string_view name
) {
    for (const auto& symbol : image.symbols()) {
        if (symbol.name == name) return symbol.address;
    }
    return std::nullopt;
}

void copiesDataZerosBssAndCallsMain() {
    fil::config::BoardConfig config;
    config.name = "startup-runtime";
    config.mcu_path = std::filesystem::path(FIL_SOURCE_DIR)
        / "configs/mcus/stm32g474retx.json";
    config.elf_path = std::filesystem::path(FIL_SOURCE_DIR)
        / "tests/fixtures/elf/startup_runtime.elf";
    config.vector_base = 0x08000000U;
    auto board = fil::sim::Board::load(config);
    fil::test::check(board.hasValue(), "loads synthetic C-runtime startup fixture");
    if (!board) return;

    const auto data = symbolAddress(board.value()->image(), "startup_data");
    const auto bss = symbolAddress(board.value()->image(), "startup_bss");
    const auto sentinel = symbolAddress(board.value()->image(), "startup_sentinel");
    fil::test::check(data && bss && sentinel, "startup fixture exposes verification symbols");
    if (!data || !bss || !sentinel) return;

    const auto initial_data = board.value()->memory().read32(*data);
    fil::test::check(initial_data && initial_data.value() == 0U,
                     "runtime .data begins at reset value before startup copy");
    for (std::uint32_t offset = 0; offset < 16U; offset += 4U) {
        fil::test::check(
            board.value()->memory().write32(*bss + offset, 0xa5a5a5a5U).hasValue(),
            "pre-dirties startup .bss"
        );
    }
    fil::test::check(
        board.value()->memory().write32(*sentinel, 0x11111111U).hasValue(),
        "pre-dirties startup sentinel"
    );

    fil::sim::BoardRunOptions options;
    options.max_instructions = 200U;
    options.duration_ns = 0U;
    options.detect_spin = false;
    const auto result = board.value()->run(options);
    fil::test::check(result.reason == fil::sim::BoardStopReason::breakpoint,
                     "synthetic startup reaches main and stops at BKPT");
    fil::test::check(result.instructions == 61U,
                     "synthetic startup executes the exact expected loop path");
    const std::uint32_t expected_data[]{0x12345678U, 0xa5a55a5aU, 0x0badc0deU};
    for (std::uint32_t index = 0; index < 3U; ++index) {
        const auto value = board.value()->memory().read32(*data + index * 4U);
        fil::test::check(value && value.value() == expected_data[index],
                         "startup copies one .data initializer from its flash LMA");
    }
    for (std::uint32_t offset = 0; offset < 16U; offset += 4U) {
        const auto value = board.value()->memory().read32(*bss + offset);
        fil::test::check(value && value.value() == 0U,
                         "startup zeros one pre-dirtied .bss word");
    }
    const auto final_sentinel = board.value()->memory().read32(*sentinel);
    fil::test::check(final_sentinel && final_sentinel.value() == 0xfeedbeefU,
                     "main writes the expected startup sentinel");
}

} // namespace

void runStartupRuntimeTests() {
    copiesDataZerosBssAndCallsMain();
}
