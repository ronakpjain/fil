#include "fil/sim/board.hpp"
#include "fil/sim/world.hpp"
#include "fil/stm32g4/stm32g4.hpp"
#include "../fixture_support.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

class TempWorldTimeConfigs {
public:
    [[nodiscard]] std::filesystem::path writeBoard(
        const std::string_view file_name,
        const std::string_view board_name
    ) const {
        const auto fixture = fil::test::fixtureBoardConfig(board_name);
        std::ostringstream json;
        json << "{\n"
             << "  \"schema_version\": 1,\n"
             << "  \"name\": \"" << board_name << "\",\n"
             << "  \"mcu\": \"" << fixture.mcu_path.string() << "\",\n"
             << "  \"elf\": \"" << fixture.elf_path.string() << "\",\n"
             << "  \"vector_base\": \"0x08000000\"\n"
             << "}\n";
        return directory_.write(file_name, json.str());
    }

    [[nodiscard]] fil::config::NetworkConfig network(
        std::vector<std::filesystem::path> boards
    ) const {
        fil::config::NetworkConfig config;
        config.name = "world-time-fixture";
        config.source_path = directory_.root() / "network.json";
        config.board_paths = std::move(boards);
        return config;
    }

private:
    fil::test::TemporaryDirectory directory_{"fil-world-time-tests"};
};

fil::sim::WorldRunOptions runOptions(const fil::sim::SimTimeNs duration_ns) {
    fil::sim::WorldRunOptions options;
    options.max_instructions_per_board = 10U;
    options.duration_ns = duration_ns;
    options.instruction_quantum = 7U;
    options.detect_spin = false;
    return options;
}

bool installIdleLoop(
    fil::sim::World& world,
    const std::string_view board_name,
    const std::size_t nop_count
) {
    fil::sim::Board* board = world.board(board_name);
    if (board == nullptr || nop_count > 16U) return false;
    const std::uint32_t start = board->cpu().state().r[15] & ~1U;
    std::vector<std::uint8_t> code;
    code.reserve((nop_count + 1U) * 2U);
    for (std::size_t index = 0; index < nop_count; ++index) {
        code.push_back(0x00U);
        code.push_back(0xbfU); // NOP
    }
    const auto backwards_halfwords = -static_cast<std::int32_t>(nop_count) - 2;
    const auto branch = static_cast<std::uint16_t>(
        0xe000U | (static_cast<std::uint32_t>(backwards_halfwords) & 0x07ffU)
    );
    code.push_back(static_cast<std::uint8_t>(branch));
    code.push_back(static_cast<std::uint8_t>(branch >> 8U));
    return board->memory().loadBytes(start, code).hasValue();
}

bool selectExtremePllClock(fil::sim::World& world, const std::string_view board_name) {
    fil::sim::Board* board = world.board(board_name);
    if (board == nullptr) return false;
    auto& rcc = board->peripherals().rcc();
    const std::uint32_t pll_config = 2U | (127U << 8U); // 16 MHz * 127 / 2.
    const fil::mem::AccessContext context{fil::mem::AccessType::data_write, 0U};
    return rcc.write(0x0cU, fil::mem::AccessSize::word, pll_config, context).hasValue()
        && rcc.write(0x08U, fil::mem::AccessSize::word, 3U, context).hasValue()
        && rcc.systemClockHz() == 1'016'000'000U;
}

bool configureSysTickIdleLoop(
    fil::sim::World& world,
    const std::string_view board_name,
    const std::size_t nop_count
) {
    if (!installIdleLoop(world, board_name, nop_count)) return false;
    fil::sim::Board* board = world.board(board_name);
    if (board == nullptr) return false;
    const std::uint32_t loop_start = board->cpu().state().r[15] & ~1U;
    const std::uint32_t handler = loop_start + 0x80U;
    const std::vector<std::uint8_t> handler_code{
        0x01U, 0x36U, // ADDS r6, #1
        0x70U, 0x47U, // BX lr (EXC_RETURN)
    };
    const std::uint32_t vector = handler | 1U;
    const std::vector<std::uint8_t> vector_bytes{
        static_cast<std::uint8_t>(vector),
        static_cast<std::uint8_t>(vector >> 8U),
        static_cast<std::uint8_t>(vector >> 16U),
        static_cast<std::uint8_t>(vector >> 24U),
    };
    if (!board->memory().loadBytes(handler, handler_code)
        || !board->memory().loadBytes(board->image().vectorBase() + 15U * 4U, vector_bytes)) {
        return false;
    }
    return board->memory().write32(0xe000e014U, 31U).hasValue()
        && board->memory().write32(0xe000e018U, 0U).hasValue()
        && board->memory().write32(0xe000e010U, 3U).hasValue();
}

TEST(WorldTimeTest, BoardCountDoesNotScaleGlobalTime) {
    TempWorldTimeConfigs files;
    const auto alpha = files.writeBoard("alpha.json", "alpha");
    const auto beta = files.writeBoard("beta.json", "beta");
    const auto gamma = files.writeBoard("gamma.json", "gamma");

    auto one = fil::sim::World::load(files.network({alpha}));
    auto three = fil::sim::World::load(files.network({alpha, beta, gamma}));
    EXPECT_TRUE(one.hasValue() && three.hasValue())
        << "loads one- and three-board virtual-time worlds";
    if (!one || !three) return;

    const auto one_result = one.value()->run(runOptions(0U));
    const auto three_result = three.value()->run(runOptions(0U));
    EXPECT_TRUE(one_result.hasValue() && three_result.hasValue())
        << "runs one- and three-board virtual-time worlds";
    if (!one_result || !three_result) return;

    EXPECT_TRUE(one_result.value().end_time_ns == three_result.value().end_time_ns)
        << "adding identical boards does not multiply shared simulated time";

    fil::config::BoardConfig standalone_config;
    standalone_config.name = "standalone";
    standalone_config.mcu_path =
        std::filesystem::path(FIL_SOURCE_DIR) / "configs/mcus/stm32g474retx.json";
    standalone_config.elf_path =
        std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";
    standalone_config.vector_base = 0x08000000U;
    auto standalone = fil::sim::Board::load(standalone_config);
    fil::sim::BoardRunOptions standalone_options;
    standalone_options.max_instructions = 10U;
    standalone_options.duration_ns = 0U;
    standalone_options.detect_spin = false;
    const auto standalone_result = standalone
        ? standalone.value()->run(standalone_options) : fil::sim::BoardRunResult{};
    EXPECT_TRUE(standalone.hasValue() &&
                standalone_result.reason == fil::sim::BoardStopReason::breakpoint &&
                standalone_result.time_ns == one_result.value().end_time_ns)
        << "standalone Board timing remains identical to a one-board world";
    EXPECT_TRUE(one_result.value().boards[0].result.instructions == 3U &&
                one_result.value().boards[0].result.cycles == 3U)
        << "single-board world executes the complete fixture cycle count";
    bool every_board_matches = true;
    for (const fil::sim::WorldBoardRunResult& board : three_result.value().boards) {
        every_board_matches = every_board_matches
            && board.result.instructions == one_result.value().boards[0].result.instructions
            && board.result.cycles == one_result.value().boards[0].result.cycles
            && board.result.time_ns == one_result.value().end_time_ns;
    }
    EXPECT_TRUE(every_board_matches)
        << "each concurrent board receives the same CPU progress and completion time";
    EXPECT_TRUE(three_result.value().cycles == 3U * one_result.value().cycles)
        << "world cycle totals remain the sum of independent board cycles";
}

TEST(WorldTimeTest, TimeBudgetProgressIsIndependentOfBoardCount) {
    TempWorldTimeConfigs files;
    const auto alpha = files.writeBoard("alpha.json", "alpha");
    const auto beta = files.writeBoard("beta.json", "beta");
    const auto gamma = files.writeBoard("gamma.json", "gamma");

    auto one = fil::sim::World::load(files.network({alpha}));
    auto three = fil::sim::World::load(files.network({alpha, beta, gamma}));
    if (!one || !three) {
        EXPECT_TRUE(false) << "loads worlds for concurrent time-budget test";
        return;
    }

    const auto one_result = one.value()->run(runOptions(100U));
    const auto three_result = three.value()->run(runOptions(100U));
    EXPECT_TRUE(one_result.hasValue() && three_result.hasValue())
        << "runs concurrent worlds to a sub-instruction time deadline";
    if (!one_result || !three_result) return;

    EXPECT_TRUE(one_result.value().reason == fil::sim::WorldStopReason::time_budget &&
                three_result.value().reason == fil::sim::WorldStopReason::time_budget)
        << "one- and three-board worlds reach the same time-budget boundary";
    EXPECT_TRUE(one_result.value().end_time_ns == three_result.value().end_time_ns)
        << "atomic deadline overshoot is independent of board count";
    const auto& reference = one_result.value().boards[0].result;
    bool equal_progress = reference.instructions == 2U && reference.cycles == 2U;
    for (const fil::sim::WorldBoardRunResult& board : three_result.value().boards) {
        equal_progress = equal_progress
            && board.result.reason == fil::sim::BoardStopReason::time_budget
            && board.result.instructions == reference.instructions
            && board.result.cycles == reference.cycles;
    }
    EXPECT_TRUE(equal_progress)
        << "time limiting does not divide per-board instructions by board count";
}

TEST(WorldTimeTest, ConcurrentScheduleIsByteDeterministic) {
    TempWorldTimeConfigs files;
    const auto alpha = files.writeBoard("alpha.json", "alpha");
    const auto beta = files.writeBoard("beta.json", "beta");
    const auto config = files.network({alpha, beta});
    auto first = fil::sim::World::load(config);
    auto second = fil::sim::World::load(config);
    if (!first || !second) {
        EXPECT_TRUE(false) << "loads repeated worlds for virtual-time determinism test";
        return;
    }

    auto options = runOptions(0U);
    options.trace_instructions = true;
    const auto first_result = first.value()->run(options);
    const auto second_result = second.value()->run(options);
    std::ostringstream first_trace;
    std::ostringstream second_trace;
    first.value()->trace().writeJsonLines(first_trace);
    second.value()->trace().writeJsonLines(second_trace);
    EXPECT_TRUE(first_result.hasValue() && second_result.hasValue())
        << "runs repeated concurrent schedules";
    EXPECT_TRUE(first_trace.str() == second_trace.str())
        << "concurrent schedules produce byte-identical traces";

    std::vector<std::string> starts;
    for (const fil::sim::TraceRecord& record : first.value()->trace().records()) {
        if (record.type == "instr") {
            starts.push_back(std::to_string(record.time_ns) + ":" + record.source);
        }
    }
    EXPECT_TRUE((starts ==
                 std::vector<std::string>{
                     "0:alpha",
                     "0:beta",
                     "62:alpha",
                     "62:beta",
                     "125:alpha",
                     "125:beta",
                 }))
        << "same-time CPU starts use stable configuration order at each virtual frontier";
}

TEST(WorldTimeTest, SharedEventsKeepTheirExactDeadlines) {
    TempWorldTimeConfigs files;
    const auto alpha = files.writeBoard("alpha.json", "alpha");
    const auto beta = files.writeBoard("beta.json", "beta");
    auto world = fil::sim::World::load(files.network({alpha, beta}));
    if (!world) {
        EXPECT_TRUE(false) << "loads world for shared event-deadline test";
        return;
    }

    fil::sim::SimTimeNs callback_time = 0U;
    static_cast<void>(world.value()->eventLoop().scheduleAt(100U, [&] {
        callback_time = world.value()->eventLoop().now();
        world.value()->trace().record(callback_time, "test", "deadline");
    }));
    auto options = runOptions(0U);
    options.trace_instructions = true;
    const auto result = world.value()->run(options);
    EXPECT_TRUE(result.hasValue() && callback_time == 100U)
        << "shared callbacks execute at their exact timestamp between CPU frontiers";

    std::size_t deadline_index = world.value()->trace().records().size();
    std::size_t next_instruction_index = world.value()->trace().records().size();
    for (std::size_t index = 0; index < world.value()->trace().records().size(); ++index) {
        const fil::sim::TraceRecord& record = world.value()->trace().records()[index];
        if (record.type == "deadline") deadline_index = index;
        if (record.type == "instr" && record.time_ns == 125U
            && next_instruction_index == world.value()->trace().records().size()) {
            next_instruction_index = index;
        }
    }
    EXPECT_TRUE(deadline_index < next_instruction_index)
        << "due events settle before either CPU starts at the next completion frontier";
}

TEST(WorldTimeTest, AcceleratedPhaseMismatchedLoopsMatchExactExecution) {
    TempWorldTimeConfigs files;
    const auto alpha = files.writeBoard("alpha.json", "alpha");
    const auto beta = files.writeBoard("beta.json", "beta");
    const auto config = files.network({alpha, beta});
    auto exact = fil::sim::World::load(config);
    auto accelerated = fil::sim::World::load(config);
    if (!exact || !accelerated) {
        EXPECT_TRUE(false) << "loads worlds for accelerated-loop equivalence";
        return;
    }
    EXPECT_TRUE(installIdleLoop(*exact.value(), "alpha", 0U) &&
                installIdleLoop(*exact.value(), "beta", 1U) &&
                installIdleLoop(*accelerated.value(), "alpha", 0U) &&
                installIdleLoop(*accelerated.value(), "beta", 1U) &&
                selectExtremePllClock(*exact.value(), "alpha") &&
                selectExtremePllClock(*exact.value(), "beta") &&
                selectExtremePllClock(*accelerated.value(), "alpha") &&
                selectExtremePllClock(*accelerated.value(), "beta"))
        << "installs phase-mismatched loops with sub-nanosecond target cycles";

    const auto schedule_wake = [](fil::sim::World& world) {
        static_cast<void>(world.eventLoop().scheduleAt(62U, [&world] {
            world.board("alpha")->cpu().state().r[4] = 0x12345678U;
            world.board("beta")->cpu().state().r[5] = 0x87654321U;
            world.trace().record(world.eventLoop().now(), "test", "wake");
        }));
    };
    schedule_wake(*exact.value());
    schedule_wake(*accelerated.value());

    auto exact_options = runOptions(150U);
    exact_options.max_instructions_per_board = 1'000U;
    exact_options.enable_loop_batching = false;
    auto accelerated_options = exact_options;
    accelerated_options.enable_loop_batching = true;
    const auto exact_result = exact.value()->run(exact_options);
    const auto accelerated_result = accelerated.value()->run(accelerated_options);
    EXPECT_TRUE(exact_result.hasValue() && accelerated_result.hasValue())
        << "runs exact and accelerated loop worlds";
    if (!exact_result || !accelerated_result) return;

    EXPECT_TRUE(exact_result.value().reason == accelerated_result.value().reason &&
                exact_result.value().end_time_ns == accelerated_result.value().end_time_ns &&
                exact_result.value().instructions == accelerated_result.value().instructions &&
                exact_result.value().cycles == accelerated_result.value().cycles)
        << "loop batching preserves world stop boundary and logical counters";
    EXPECT_TRUE(fil::cpu::bitwiseEqual(exact.value()->board("alpha")->cpu().state(),
                    accelerated.value()->board("alpha")->cpu().state()) &&
                fil::cpu::bitwiseEqual(exact.value()->board("beta")->cpu().state(),
                    accelerated.value()->board("beta")->cpu().state()))
        << "loop batching preserves full CPU state across a scheduled wakeup";
    EXPECT_TRUE(exact.value()->trace().jsonLines() == accelerated.value()->trace().jsonLines())
        << "loop batching preserves observable callback and trace ordering";
}

TEST(WorldTimeTest, TransactionalSlicesMatchExactIdleExecution) {
    TempWorldTimeConfigs files;
    const auto alpha = files.writeBoard("txn-alpha.json", "txn-alpha");
    const auto beta = files.writeBoard("txn-beta.json", "txn-beta");
    const auto config = files.network({alpha, beta});
    auto exact = fil::sim::World::load(config);
    auto transactional = fil::sim::World::load(config);
    if (!exact || !transactional) {
        EXPECT_TRUE(false) << "loads worlds for transactional equivalence";
        return;
    }
    exact.value()->setDiagnosticsEnabled(false);
    transactional.value()->setDiagnosticsEnabled(false);
    EXPECT_TRUE(installIdleLoop(*exact.value(), "txn-alpha", 0U) &&
                installIdleLoop(*exact.value(), "txn-beta", 0U) &&
                installIdleLoop(*transactional.value(), "txn-alpha", 0U) &&
                installIdleLoop(*transactional.value(), "txn-beta", 0U))
        << "installs MMIO-free transactional loop fixtures";

    auto exact_options = runOptions(100'000U);
    exact_options.max_instructions_per_board = 10'000U;
    exact_options.enable_loop_batching = false;
    exact_options.enable_transactional_slices = false;
    auto transactional_options = exact_options;
    transactional_options.enable_transactional_slices = true;
    const auto exact_result = exact.value()->run(exact_options);
    const auto transactional_result = transactional.value()->run(transactional_options);
    EXPECT_TRUE(exact_result && transactional_result &&
                transactional_result.value().transactional_commits > 0U)
        << "transactional scheduler commits MMIO-free lane epochs";
    if (!exact_result || !transactional_result) return;
    EXPECT_TRUE(exact_result.value().end_time_ns == transactional_result.value().end_time_ns &&
                exact_result.value().instructions == transactional_result.value().instructions &&
                exact_result.value().cycles == transactional_result.value().cycles &&
                fil::cpu::bitwiseEqual(exact.value()->board("txn-alpha")->cpu().state(),
                    transactional.value()->board("txn-alpha")->cpu().state()) &&
                fil::cpu::bitwiseEqual(exact.value()->board("txn-beta")->cpu().state(),
                    transactional.value()->board("txn-beta")->cpu().state()))
        << "transactional slices preserve virtual time counters and CPU state";
}

TEST(WorldTimeTest, AcceleratedLoopsPreserveSysTickAndExceptionEntry) {
    TempWorldTimeConfigs files;
    const auto alpha = files.writeBoard("tick-alpha.json", "tick-alpha");
    const auto beta = files.writeBoard("tick-beta.json", "tick-beta");
    const auto config = files.network({alpha, beta});
    auto exact = fil::sim::World::load(config);
    auto accelerated = fil::sim::World::load(config);
    if (!exact || !accelerated) {
        EXPECT_TRUE(false) << "loads worlds for SysTick batching equivalence";
        return;
    }
    EXPECT_TRUE(configureSysTickIdleLoop(*exact.value(), "tick-alpha", 1U) &&
                configureSysTickIdleLoop(*exact.value(), "tick-beta", 2U) &&
                configureSysTickIdleLoop(*accelerated.value(), "tick-alpha", 1U) &&
                configureSysTickIdleLoop(*accelerated.value(), "tick-beta", 2U))
        << "installs phase-mismatched idle loops with a real SysTick handler";

    auto exact_options = runOptions(10'000U);
    exact_options.max_instructions_per_board = 1'000U;
    exact_options.enable_loop_batching = false;
    auto accelerated_options = exact_options;
    accelerated_options.enable_loop_batching = true;
    const auto exact_result = exact.value()->run(exact_options);
    const auto accelerated_result = accelerated.value()->run(accelerated_options);
    EXPECT_TRUE(exact_result.hasValue() && accelerated_result.hasValue())
        << "runs exact and accelerated SysTick worlds";
    if (!exact_result || !accelerated_result) return;

    EXPECT_TRUE(exact_result.value().reason == accelerated_result.value().reason &&
                exact_result.value().end_time_ns == accelerated_result.value().end_time_ns &&
                exact_result.value().instructions == accelerated_result.value().instructions &&
                exact_result.value().cycles == accelerated_result.value().cycles)
        << "SysTick-capped batching preserves stop boundary and logical counters";
    EXPECT_TRUE(accelerated_result.value().rounds < exact_result.value().rounds)
        << "SysTick equivalence exercises a real accelerated path";
    EXPECT_TRUE(fil::cpu::bitwiseEqual(exact.value()->board("tick-alpha")->cpu().state(),
                    accelerated.value()->board("tick-alpha")->cpu().state()) &&
                fil::cpu::bitwiseEqual(exact.value()->board("tick-beta")->cpu().state(),
                    accelerated.value()->board("tick-beta")->cpu().state()) &&
                exact.value()->board("tick-alpha")->cpu().state().r[6] != 0U)
        << "batching preserves SysTick handler execution and final CPU state";
    EXPECT_TRUE(exact.value()->trace().jsonLines() == accelerated.value()->trace().jsonLines())
        << "batching preserves SysTick exception-entry and return trace ordering";
}

} // namespace
