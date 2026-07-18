#include "fil/sim/board.hpp"
#include "../test_support.hpp"

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

std::optional<std::uint32_t> schedulerSymbol(
    const fil::elf::ElfImage& image,
    const std::string_view name
) {
    for (const auto& symbol : image.symbols()) {
        if (symbol.name == name) return symbol.address;
    }
    return std::nullopt;
}

std::optional<std::uint32_t> readSchedulerWord(
    fil::sim::Board& board,
    const std::string_view name
) {
    const auto address = schedulerSymbol(board.image(), name);
    if (!address) return std::nullopt;
    const auto value = board.memory().read32(*address);
    if (!value) return std::nullopt;
    return value.value();
}

void startsPspTaskAndDeliversSysTick() {
    fil::config::BoardConfig config;
    config.name = "scheduler-tick";
    config.mcu_path = std::filesystem::path(FIL_SOURCE_DIR)
        / "configs/mcus/stm32g474retx.json";
    config.elf_path = std::filesystem::path(FIL_SOURCE_DIR)
        / "tests/fixtures/elf/scheduler_tick.elf";
    config.vector_base = 0x08000000U;

    auto loaded = fil::sim::Board::load(config, true);
    fil::test::check(loaded.hasValue(), "loads hermetic scheduler/tick firmware fixture");
    if (!loaded) return;
    auto& board = *loaded.value();

    const auto task_stack_top = schedulerSymbol(board.image(), "task_stack_top");
    fil::test::check(task_stack_top.has_value(), "scheduler fixture exposes its PSP stack top");
    if (!task_stack_top) return;

    fil::sim::BoardRunOptions options;
    options.max_instructions = 200U;
    options.duration_ns = 0U;
    options.detect_spin = false;
    const auto result = board.run(options);

    fil::test::check(result.reason == fil::sim::BoardStopReason::breakpoint,
                     "PSP task resumes after SysTick and reaches its BKPT");
    fil::test::check(result.instructions == 102U,
                     "scheduler fixture follows the exact bounded instruction path");
    fil::test::check(readSchedulerWord(board, "svc_count") == 1U,
                     "SVC handler runs exactly once");
    fil::test::check(readSchedulerWord(board, "pendsv_count") == 1U,
                     "PendSV handler performs exactly one initial context restore");
    fil::test::check(readSchedulerWord(board, "tick_count") == 1U,
                     "SysTick handler runs exactly once");
    fil::test::check(readSchedulerWord(board, "task_started") == 0x51c00001U,
                     "PendSV starts the synthetic task");
    fil::test::check(readSchedulerWord(board, "task_resumed") == 0x51c00002U,
                     "the synthetic task observes and resumes after its tick");
    fil::test::check(readSchedulerWord(board, "task_entry_psp") == *task_stack_top,
                     "the task begins on the restored PSP");
    fil::test::check(readSchedulerWord(board, "systick_frame_psp") == *task_stack_top - 32U,
                     "SysTick stacks its basic frame on the task PSP");
    fil::test::check(readSchedulerWord(board, "task_resumed_psp") == *task_stack_top,
                     "SysTick return restores the task PSP exactly");

    const auto& state = board.cpu().state();
    fil::test::check(state.ipsr() == 0U && (state.control & 2U) != 0U,
                     "BKPT is reached in PSP-selected Thread mode");
    fil::test::check(state.psp == *task_stack_top && state.r[13] == *task_stack_top,
                     "final architectural SP view selects the task PSP");

    std::vector<std::pair<std::string, std::string>> exception_trace;
    for (const auto& record : board.trace().records()) {
        if (record.type != "exception_enter" && record.type != "exception_return") continue;
        const std::string_view wanted = record.type == "exception_enter" ? "exception" : "exc_return";
        for (const auto& [key, value] : record.fields) {
            if (key == wanted) exception_trace.emplace_back(record.type, value);
        }
    }
    const std::array<std::pair<std::string_view, std::string_view>, 6> expected_trace{{
        {"exception_enter", "11"},
        {"exception_return", "0xfffffff9"},
        {"exception_enter", "14"},
        {"exception_return", "0xfffffffd"},
        {"exception_enter", "15"},
        {"exception_return", "0xfffffffd"},
    }};
    fil::test::check(exception_trace.size() == expected_trace.size(),
                     "scheduler fixture emits exactly three entries and three returns");
    if (exception_trace.size() == expected_trace.size()) {
        for (std::size_t index = 0; index < expected_trace.size(); ++index) {
            fil::test::check(
                exception_trace[index].first == expected_trace[index].first
                    && exception_trace[index].second == expected_trace[index].second,
                "scheduler exception trace preserves SVC/PendSV/SysTick order"
            );
        }
    }
}

} // namespace

void runSchedulerTickTests() {
    startsPspTaskAndDeliversSysTick();
}
