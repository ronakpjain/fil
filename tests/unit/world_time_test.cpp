#include "fil/sim/board.hpp"
#include "fil/sim/world.hpp"
#include "fil/stm32g4/stm32g4.hpp"
#include "../fixture_support.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <limits>
#include <sstream>
#include <stdexcept>
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
    // Real-timing model (docs/real_timing_audit.md): the split-image fixture
    // executes LDR, LDR, BKPT from flash at 16 MHz/0 WS, costing 2+2+1 = 5
    // pipeline cycles for 3 instructions. The 1-CPI equality no longer holds.
    EXPECT_TRUE(one_result.value().boards[0].result.instructions == 3U &&
                one_result.value().boards[0].result.cycles == 5U)
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
    // Real-timing model: the first fixture LDR costs 2 cycles (125 ns at
    // 16 MHz), so the 100 ns budget is exhausted atomically after 1
    // instruction / 2 cycles instead of the old 2 instructions / 2 cycles.
    bool equal_progress = reference.instructions == 1U && reference.cycles == 2U;
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
    // Real-timing model: each fixture LDR costs 2 cycles (125 ns at
    // 16 MHz), so same-time frontiers land on 125 ns boundaries instead of
    // the old 62.5 ns single-cycle grid. Order per frontier is unchanged.
    EXPECT_TRUE((starts ==
                 std::vector<std::string>{
                     "0:alpha",
                     "0:beta",
                     "125:alpha",
                     "125:beta",
                     "250:alpha",
                     "250:beta",
                 }))
        << "same-time CPU starts use stable configuration order at each virtual frontier";
}

TEST(WorldTimeTest, LockstepRequiresAProvenPureBlock) {
    TempWorldTimeConfigs files;
    const auto config = files.network({
        files.writeBoard("single-alpha.json", "single-alpha"),
        files.writeBoard("single-beta.json", "single-beta")});
    auto lockstep = fil::sim::World::load(config);
    auto exact = fil::sim::World::load(config);
    ASSERT_TRUE(lockstep && exact);

    auto options = runOptions(0U);
    options.max_instructions_per_board = 10U;
    options.enable_jit = false;
    const auto lockstep_result = lockstep.value()->run(options);
    options.detect_spin = true; // Disables bursts; retains exact dispatch timing.
    const auto exact_result = exact.value()->run(options);
    ASSERT_TRUE(lockstep_result && exact_result);
    EXPECT_EQ(lockstep_result.value().instructions, exact_result.value().instructions);
    EXPECT_EQ(lockstep_result.value().cycles, exact_result.value().cycles);
    EXPECT_EQ(lockstep_result.value().end_time_ns, exact_result.value().end_time_ns);
    EXPECT_EQ(lockstep_result.value().lockstep_bursts, 0U)
        << "scalar per-instruction predictions do not prove actual completion time";
    EXPECT_EQ(lockstep.value()->trace().jsonLines(), exact.value()->trace().jsonLines());
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

void warmJit(fil::sim::Board& board) {
    const auto initial = board.cpu().state();
    for (unsigned int i = 0U; i < 60U; ++i) {
        board.cpu().state() = initial;
        static_cast<void>(board.cpu().tryStepJitBlock());
    }
    board.cpu().state() = initial;
}

TEST(WorldTimeTest, CompactFrontierMatchesGeneralSchedulerAcrossPaddingBoundary) {
    for (const std::size_t count : {1U, 6U, 8U, 9U}) {
        SCOPED_TRACE(count);
        TempWorldTimeConfigs files;
        std::vector<std::filesystem::path> paths;
        std::vector<std::string> names;
        for (std::size_t i = 0U; i < count; ++i) {
            names.push_back("lane-" + std::to_string(i));
            paths.push_back(files.writeBoard(names.back() + ".json", names.back()));
        }
        const auto config = files.network(paths);
        auto compact = fil::sim::World::load(config);
        auto general = fil::sim::World::load(config);
        ASSERT_TRUE(compact && general);
        for (auto* world : {compact.value().get(), general.value().get()}) {
            for (std::size_t i = 0U; i < count; ++i) {
                ASSERT_TRUE(installIdleLoop(*world, names[i], static_cast<unsigned int>(i % 3U + 1U)));
            }
            static_cast<void>(world->eventLoop().scheduleAt(125U, [world, names] {
                for (const auto& name : names) {
                    world->trace().record(world->eventLoop().now(), name, "frontier-observe",
                        {{"pc", std::to_string(world->board(name)->cpu().state().r[15])}});
                }
                static_cast<void>(world->eventLoop().scheduleAt(125U, [world] {
                    EXPECT_EQ(world->eventLoop().now(), 125U);
                    world->trace().record(world->eventLoop().now(), "test", "frontier-tie");
                }));
            }));
        }
        auto options = runOptions(10'000U);
        options.max_instructions_per_board = 103U;
        options.enable_loop_batching = false;
        const auto result = compact.value()->run(options);
        options.enable_loop_batching = true;
        const auto reference = general.value()->run(options);
        ASSERT_TRUE(result && reference);
        EXPECT_EQ(result.value().reason, reference.value().reason);
        EXPECT_EQ(result.value().instructions, reference.value().instructions);
        EXPECT_EQ(result.value().cycles, reference.value().cycles);
        EXPECT_EQ(result.value().end_time_ns, reference.value().end_time_ns);
        EXPECT_EQ(compact.value()->trace().jsonLines(), general.value()->trace().jsonLines());
        for (const auto& name : names) {
            EXPECT_TRUE(fil::cpu::bitwiseEqual(compact.value()->board(name)->cpu().state(),
                general.value()->board(name)->cpu().state()));
        }
    }
}

TEST(WorldTimeTest, CompactQuantumMatchesGeneralAtSaturatedClock) {
    TempWorldTimeConfigs files;
    const auto config = files.network({
        files.writeBoard("saturated-alpha.json", "saturated-alpha"),
        files.writeBoard("saturated-beta.json", "saturated-beta")});
    auto compact = fil::sim::World::load(config);
    auto general = fil::sim::World::load(config);
    ASSERT_TRUE(compact && general);
    for (auto* world : {compact.value().get(), general.value().get()}) {
        ASSERT_TRUE(installIdleLoop(*world, "saturated-alpha", 1U));
        ASSERT_TRUE(installIdleLoop(*world, "saturated-beta", 3U));
        static_cast<void>(world->eventLoop().runDueEvents(
            std::numeric_limits<fil::sim::SimTimeNs>::max()));
    }
    auto options = runOptions(0U);
    options.max_instructions_per_board = 11U;
    options.instruction_quantum = 1U;
    options.enable_loop_batching = false;
    const auto result = compact.value()->run(options);
    options.enable_loop_batching = true;
    const auto reference = general.value()->run(options);
    ASSERT_TRUE(result && reference);
    EXPECT_EQ(result.value().reason, reference.value().reason);
    EXPECT_EQ(result.value().instructions, reference.value().instructions);
    EXPECT_EQ(result.value().cycles, reference.value().cycles);
    EXPECT_EQ(result.value().end_time_ns, reference.value().end_time_ns);
    EXPECT_EQ(result.value().dispatches, result.value().instructions);
    for (const auto name : {"saturated-alpha", "saturated-beta"}) {
        EXPECT_TRUE(fil::cpu::bitwiseEqual(compact.value()->board(name)->cpu().state(),
            general.value()->board(name)->cpu().state()));
    }
}

TEST(WorldTimeTest, SynchronizedPureJitBlocksMatchInterpreter) {
    TempWorldTimeConfigs files;
    const auto config = files.network({
        files.writeBoard("sync-alpha.json", "sync-alpha"),
        files.writeBoard("sync-beta.json", "sync-beta")});
    auto exact = fil::sim::World::load(config);
    auto jit = fil::sim::World::load(config);
    ASSERT_TRUE(exact && jit);
    for (const auto name : {"sync-alpha", "sync-beta"}) {
        ASSERT_TRUE(installIdleLoop(*exact.value(), name, 6U));
        ASSERT_TRUE(installIdleLoop(*jit.value(), name, 6U));
    }
    const auto schedule = [](fil::sim::World& world) {
        static_cast<void>(world.eventLoop().scheduleAt(100U, [&world] {
            world.trace().record(world.eventLoop().now(), "test", "synchronized-wake",
                {{"pc", std::to_string(world.board("sync-alpha")->cpu().state().r[15])}});
            static_cast<void>(world.eventLoop().scheduleAt(150U, [&world] {
                world.board("sync-beta")->cpu().state().r[0] = 42U;
                world.trace().record(world.eventLoop().now(), "test", "synchronized-followup",
                    {{"pc", std::to_string(world.board("sync-beta")->cpu().state().r[15])}});
            }));
        }));
    };
    schedule(*exact.value());
    schedule(*jit.value());
    auto options = runOptions(1'000'000U);
    options.max_instructions_per_board = 5'003U;
    options.enable_loop_batching = false;
    options.enable_jit = false;
    const auto reference = exact.value()->run(options);
    options.enable_jit = true;
    const auto result = jit.value()->run(options);
    ASSERT_TRUE(reference && result);
    EXPECT_EQ(result.value().reason, reference.value().reason);
    EXPECT_EQ(result.value().instructions, reference.value().instructions);
    EXPECT_EQ(result.value().cycles, reference.value().cycles);
    EXPECT_EQ(result.value().end_time_ns, reference.value().end_time_ns);
    EXPECT_EQ(result.value().dispatches, reference.value().dispatches);
    EXPECT_EQ(result.value().exact_dispatches, result.value().instructions);
    EXPECT_EQ(jit.value()->board("sync-alpha")->cpu().jitStats().block_executions, 0U);
    EXPECT_EQ(jit.value()->trace().jsonLines(), exact.value()->trace().jsonLines());
    for (const auto name : {"sync-alpha", "sync-beta"}) {
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.value()->board(name)->cpu().state(),
            exact.value()->board(name)->cpu().state()));
    }
}

TEST(WorldTimeTest, SynchronizedJitAdmitsDifferentPurePrefixLengths) {
    TempWorldTimeConfigs files;
    const auto config = files.network({
        files.writeBoard("prefix-long.json", "prefix-long"),
        files.writeBoard("prefix-short.json", "prefix-short")});
    auto exact = fil::sim::World::load(config);
    auto jit = fil::sim::World::load(config);
    ASSERT_TRUE(exact && jit);
    for (auto* world : {exact.value().get(), jit.value().get()}) {
        // Five one-cycle NOPs cost the same as one NOP plus a taken branch
        // (one + four cycles), but the two lanes need different prefix lengths.
        ASSERT_TRUE(installIdleLoop(*world, "prefix-long", 5U));
        ASSERT_TRUE(installIdleLoop(*world, "prefix-short", 1U));
    }
    warmJit(*jit.value()->board("prefix-long"));
    warmJit(*jit.value()->board("prefix-short"));
    ASSERT_TRUE(jit.value()->board("prefix-long")->cpu().jitBlockReady());
    ASSERT_TRUE(jit.value()->board("prefix-short")->cpu().jitBlockReady());
    const auto long_preview = jit.value()->board("prefix-long")->cpu().peekJitBlock();
    const auto short_preview = jit.value()->board("prefix-short")->cpu().peekJitBlock();
    ASSERT_TRUE(long_preview && short_preview);
    EXPECT_FALSE(jit.value()->eventLoop().nextScheduledTime().has_value());
    EXPECT_EQ(long_preview->count, 6U);
    EXPECT_EQ(short_preview->count, 2U);
    EXPECT_EQ(long_preview->max_cycles, 9U);
    EXPECT_EQ(short_preview->max_cycles, 5U);
    EXPECT_TRUE(long_preview->cycles_exact && short_preview->cycles_exact);

    auto options = runOptions(0U);
    options.max_instructions_per_board = 100U;
    options.enable_loop_batching = false;
    options.enable_jit = false;
    const auto reference = exact.value()->run(options);
    options.enable_jit = true;
    const auto result = jit.value()->run(options);
    ASSERT_TRUE(reference && result);
    EXPECT_EQ(result.value().reason, reference.value().reason);
    EXPECT_EQ(result.value().instructions, reference.value().instructions);
    EXPECT_EQ(result.value().cycles, reference.value().cycles);
    EXPECT_EQ(result.value().end_time_ns, reference.value().end_time_ns);
    EXPECT_EQ(result.value().dispatches, reference.value().dispatches)
        << "no-loop-batching dispatches one instruction per board even with JIT";
    EXPECT_EQ(result.value().exact_dispatches, result.value().instructions);
    EXPECT_EQ(jit.value()->trace().jsonLines(), exact.value()->trace().jsonLines());
    for (const auto name : {"prefix-long", "prefix-short"}) {
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.value()->board(name)->cpu().state(),
            exact.value()->board(name)->cpu().state()));
    }
}

TEST(WorldTimeTest, SynchronizedJitMatchesMemoryStepToPureBatch) {
    constexpr std::uint32_t data_address = 0x20000000U;
    TempWorldTimeConfigs files;
    const auto config = files.network({
        files.writeBoard("memory-step.json", "memory-step"),
        files.writeBoard("pure-batch.json", "pure-batch")});
    auto exact = fil::sim::World::load(config);
    auto jit = fil::sim::World::load(config);
    ASSERT_TRUE(exact && jit);
    for (auto* world : {exact.value().get(), jit.value().get()}) {
        auto* memory_board = world->board("memory-step");
        ASSERT_NE(memory_board, nullptr);
        const auto start = memory_board->cpu().state().r[15] & ~1U;
        // LDR r0,[r1]; B start. The LDR is one exact two-cycle RAM access.
        const std::vector<std::uint8_t> memory_code{0x08U, 0x68U, 0xfdU, 0xe7U};
        ASSERT_TRUE(memory_board->memory().loadBytes(start, memory_code).hasValue());
        ASSERT_TRUE(memory_board->memory().write32(data_address, 0x12345678U).hasValue());
        memory_board->cpu().state().r[1] = data_address;
        ASSERT_TRUE(installIdleLoop(*world, "pure-batch", 2U));
    }
    warmJit(*jit.value()->board("memory-step"));
    warmJit(*jit.value()->board("pure-batch"));

    auto options = runOptions(0U);
    options.max_instructions_per_board = 100U;
    options.enable_loop_batching = false;
    options.enable_jit = false;
    const auto reference = exact.value()->run(options);
    options.enable_jit = true;
    const auto result = jit.value()->run(options);
    ASSERT_TRUE(reference && result);
    EXPECT_EQ(result.value().instructions, reference.value().instructions);
    EXPECT_EQ(result.value().cycles, reference.value().cycles);
    EXPECT_EQ(result.value().end_time_ns, reference.value().end_time_ns);
    EXPECT_GE(result.value().dispatches, reference.value().dispatches)
        << "a memory step must not share a frontier with another lane's pure batch";
    EXPECT_EQ(jit.value()->board("memory-step")->cpu().state().r[0], 0x12345678U);
    EXPECT_EQ(jit.value()->trace().jsonLines(), exact.value()->trace().jsonLines());
    for (const auto name : {"memory-step", "pure-batch"}) {
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.value()->board(name)->cpu().state(),
            exact.value()->board(name)->cpu().state()));
    }
}

TEST(WorldTimeTest, JitPreservesDivergentLaneClocksAndBlockCosts) {
    TempWorldTimeConfigs files;
    const auto config = files.network({
        files.writeBoard("mixed-alpha.json", "mixed-alpha"),
        files.writeBoard("mixed-beta.json", "mixed-beta")});
    auto exact = fil::sim::World::load(config);
    auto jit = fil::sim::World::load(config);
    ASSERT_TRUE(exact && jit);
    for (auto* world : {exact.value().get(), jit.value().get()}) {
        ASSERT_TRUE(installIdleLoop(*world, "mixed-alpha", 6U));
        ASSERT_TRUE(installIdleLoop(*world, "mixed-beta", 4U));
        ASSERT_TRUE(selectExtremePllClock(*world, "mixed-beta"));
    }
    warmJit(*jit.value()->board("mixed-alpha"));
    warmJit(*jit.value()->board("mixed-beta"));
    auto options = runOptions(100'001U);
    options.max_instructions_per_board = 501U;
    options.enable_loop_batching = false;
    options.enable_jit = false;
    const auto reference = exact.value()->run(options);
    options.enable_jit = true;
    const auto result = jit.value()->run(options);
    ASSERT_TRUE(reference && result);
    EXPECT_EQ(result.value().instructions, reference.value().instructions);
    EXPECT_EQ(result.value().cycles, reference.value().cycles);
    EXPECT_EQ(result.value().end_time_ns, reference.value().end_time_ns);
    for (const auto name : {"mixed-alpha", "mixed-beta"}) {
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.value()->board(name)->cpu().state(),
            exact.value()->board(name)->cpu().state()));
    }
    EXPECT_EQ(jit.value()->trace().jsonLines(), exact.value()->trace().jsonLines());
}

TEST(WorldTimeTest, StandaloneJitBlocksPreserveSysTickBoundaries) {
    TempWorldTimeConfigs files;
    const auto config = files.network({files.writeBoard("jit-tick.json", "jit-tick")});
    auto exact = fil::sim::World::load(config);
    auto jit = fil::sim::World::load(config);
    ASSERT_TRUE(exact && jit);
    ASSERT_TRUE(configureSysTickIdleLoop(*exact.value(), "jit-tick", 6U));
    ASSERT_TRUE(configureSysTickIdleLoop(*jit.value(), "jit-tick", 6U));
    warmJit(*jit.value()->board("jit-tick"));
    fil::sim::BoardRunOptions options;
    options.max_instructions = 1'000U;
    options.duration_ns = 10'001U;
    options.enable_loop_batching = false;
    options.enable_jit = false;
    const auto reference = exact.value()->board("jit-tick")->run(options);
    options.enable_jit = true;
    const auto result = jit.value()->board("jit-tick")->run(options);
    EXPECT_EQ(result.instructions, reference.instructions);
    EXPECT_EQ(result.cycles, reference.cycles);
    EXPECT_EQ(result.time_ns, reference.time_ns);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.value()->board("jit-tick")->cpu().state(),
        exact.value()->board("jit-tick")->cpu().state()));
    EXPECT_GT(jit.value()->board("jit-tick")->cpu().state().r[6], 0U);
    EXPECT_EQ(jit.value()->trace().jsonLines(), exact.value()->trace().jsonLines());
}

TEST(WorldTimeTest, JitPreservesSmallBudgetsEventsAndSysTick) {
    TempWorldTimeConfigs files;
    const auto config = files.network({
        files.writeBoard("jit-alpha.json", "jit-alpha"),
        files.writeBoard("jit-beta.json", "jit-beta")});
    for (const bool tick : {false, true}) {
        auto exact = fil::sim::World::load(config);
        auto jit = fil::sim::World::load(config);
        ASSERT_TRUE(exact && jit);
        for (const auto name : {"jit-alpha", "jit-beta"}) {
            if (tick) {
                ASSERT_TRUE(configureSysTickIdleLoop(*exact.value(), name, 6U));
                ASSERT_TRUE(configureSysTickIdleLoop(*jit.value(), name, 6U));
            } else {
                ASSERT_TRUE(installIdleLoop(*exact.value(), name, 6U));
                ASSERT_TRUE(installIdleLoop(*jit.value(), name, 6U));
            }
            warmJit(*jit.value()->board(name));
            ASSERT_TRUE(jit.value()->board(name)->cpu().jitBlockReady());
        }
        const auto schedule = [](fil::sim::World& world) {
            static_cast<void>(world.eventLoop().scheduleAt(100U, [&world] {
                world.board("jit-alpha")->cpu().state().r[0] += 1U;
                world.trace().record(world.eventLoop().now(), "test", "jit-boundary",
                    {{"beta_pc", std::to_string(world.board("jit-beta")->cpu().state().r[15])}});
                // A callback can add a new boundary after dispatch. Lanes
                // must not have executed an entire block across this event.
                static_cast<void>(world.eventLoop().scheduleAt(150U, [&world] {
                    world.board("jit-beta")->cpu().state().r[0] += 2U;
                    world.trace().record(world.eventLoop().now(), "test", "jit-new-boundary",
                        {{"beta_pc", std::to_string(world.board("jit-beta")->cpu().state().r[15])}});
                }));
            }));
        };
        schedule(*exact.value());
        schedule(*jit.value());
        auto options = runOptions(tick ? 10'001U : 100'000U);
        options.max_instructions_per_board = tick ? 1'000U : 3U;
        options.enable_loop_batching = false;
        options.enable_jit = false;
        const auto reference = exact.value()->run(options);
        options.enable_jit = true;
        const auto result = jit.value()->run(options);
        ASSERT_TRUE(reference && result);
        EXPECT_EQ(result.value().reason, reference.value().reason);
        EXPECT_EQ(result.value().instructions, reference.value().instructions);
        EXPECT_EQ(result.value().cycles, reference.value().cycles);
        EXPECT_EQ(result.value().end_time_ns, reference.value().end_time_ns);
        for (const auto name : {"jit-alpha", "jit-beta"}) {
            EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.value()->board(name)->cpu().state(),
                exact.value()->board(name)->cpu().state()));
        }
        EXPECT_EQ(jit.value()->trace().jsonLines(), exact.value()->trace().jsonLines());
        EXPECT_GT(jit.value()->board("jit-alpha")->cpu().jitStats().block_executions, 0U);
        if (tick) EXPECT_GT(jit.value()->board("jit-alpha")->cpu().state().r[6], 0U);
        else for (const auto& board : result.value().boards) {
            EXPECT_EQ(board.result.instructions, 3U);
        }
    }
}

TEST(WorldTimeTest, ExactSchedulerRestoresMemoryTrackingWhenCallbacksThrow) {
    TempWorldTimeConfigs files;
    auto world = fil::sim::World::load(files.network({
        files.writeBoard("tracking-alpha.json", "tracking-alpha")}));
    ASSERT_TRUE(world);
    ASSERT_TRUE(installIdleLoop(*world.value(), "tracking-alpha", 2U));
    auto& memory = world.value()->board("tracking-alpha")->memory();
    ASSERT_TRUE(memory.readFootprintTracking());
    ASSERT_TRUE(memory.writeJournalTracking());
    static_cast<void>(world.value()->eventLoop().scheduleAt(1U, [&memory] {
        EXPECT_FALSE(memory.readFootprintTracking());
        EXPECT_FALSE(memory.writeJournalTracking());
        ASSERT_TRUE(memory.write32(0x20000000U, 123U).hasValue());
        throw std::runtime_error("test callback failure");
    }));
    auto options = runOptions(1'000U);
    options.enable_loop_batching = false;
    EXPECT_THROW(static_cast<void>(world.value()->run(options)), std::runtime_error);
    EXPECT_TRUE(memory.readFootprintTracking());
    EXPECT_TRUE(memory.writeJournalTracking());
    ASSERT_TRUE(memory.read32(0x20000000U).hasValue());
    EXPECT_EQ(memory.read32(0x20000000U).value(), 123U);
}

TEST(WorldTimeTest, CompactExactSchedulerMatchesGeneralWithEventsAndZeroTimeSteps) {
    TempWorldTimeConfigs files;
    const auto config = files.network({
        files.writeBoard("compact-alpha.json", "compact-alpha"),
        files.writeBoard("compact-beta.json", "compact-beta")});
    for (const bool extreme_clock : {false, true}) {
        for (const bool jit : {false, true}) {
            auto exact = fil::sim::World::load(config);
            auto general = fil::sim::World::load(config);
            ASSERT_TRUE(exact && general);
            for (auto* world : {exact.value().get(), general.value().get()}) {
                ASSERT_TRUE(installIdleLoop(*world, "compact-alpha", 2U));
                ASSERT_TRUE(installIdleLoop(*world, "compact-beta", 5U));
                if (extreme_clock) {
                    ASSERT_TRUE(selectExtremePllClock(*world, "compact-alpha"));
                    ASSERT_TRUE(selectExtremePllClock(*world, "compact-beta"));
                }
                static_cast<void>(world->eventLoop().scheduleAt(100U, [world] {
                    world->trace().record(world->eventLoop().now(), "test", "observation",
                        {{"alpha_pc", std::to_string(world->board("compact-alpha")->cpu().state().r[15])},
                         {"beta_pc", std::to_string(world->board("compact-beta")->cpu().state().r[15])}});
                    static_cast<void>(world->eventLoop().scheduleAt(101U, [world] {
                        world->board("compact-beta")->cpu().state().r[0] += 2U;
                        world->trace().record(world->eventLoop().now(), "test", "nested-observation",
                            {{"alpha_pc", std::to_string(world->board("compact-alpha")->cpu().state().r[15])}});
                    }));
                }));
            }
            auto options = runOptions(1'001U);
            options.max_instructions_per_board = 5'000U;
            options.instruction_quantum = 1U;
            options.enable_loop_batching = false;
            options.enable_jit = jit;
            const auto candidate = exact.value()->run(options);
            // Spin observation selects the original general scheduler, with
            // batching disabled and an unreachable spin-stop threshold.
            options.detect_spin = true;
            options.spin_threshold = std::numeric_limits<std::uint64_t>::max();
            const auto reference = general.value()->run(options);
            ASSERT_TRUE(candidate && reference);
            EXPECT_EQ(candidate.value().reason, reference.value().reason);
            EXPECT_EQ(candidate.value().end_time_ns, reference.value().end_time_ns);
            EXPECT_EQ(candidate.value().instructions, reference.value().instructions);
            EXPECT_EQ(candidate.value().cycles, reference.value().cycles);
            EXPECT_EQ(candidate.value().rounds, reference.value().rounds);
            EXPECT_EQ(candidate.value().dispatches, reference.value().dispatches);
            EXPECT_EQ(candidate.value().event_callbacks, reference.value().event_callbacks);
            EXPECT_EQ(exact.value()->trace().jsonLines(), general.value()->trace().jsonLines());
            for (std::size_t index = 0U; index < candidate.value().boards.size(); ++index) {
                const auto& actual = candidate.value().boards[index];
                const auto& expected = reference.value().boards[index];
                EXPECT_EQ(actual.result.instructions, expected.result.instructions);
                EXPECT_EQ(actual.result.cycles, expected.result.cycles);
                EXPECT_EQ(actual.result.time_ns, expected.result.time_ns);
                EXPECT_EQ(actual.result.diagnostic.raw, expected.result.diagnostic.raw);
                EXPECT_TRUE(fil::cpu::bitwiseEqual(exact.value()->board(actual.name)->cpu().state(),
                    general.value()->board(expected.name)->cpu().state()));
            }
        }
    }
}

TEST(WorldTimeTest, CompactExactSchedulerPreservesFailureDrainingAndBudgets) {
    TempWorldTimeConfigs files;
    const auto config = files.network({
        files.writeBoard("drain-alpha.json", "drain-alpha"),
        files.writeBoard("drain-beta.json", "drain-beta")});
    for (const bool stop_on_failure : {false, true}) {
        for (const auto budget : {0U, 1U, 7U}) {
            auto exact = fil::sim::World::load(config);
            auto general = fil::sim::World::load(config);
            ASSERT_TRUE(exact && general);
            for (auto* world : {exact.value().get(), general.value().get()}) {
                ASSERT_TRUE(installIdleLoop(*world, "drain-alpha", 0U));
                ASSERT_TRUE(installIdleLoop(*world, "drain-beta", 5U));
                static_cast<void>(world->eventLoop().scheduleAt(150U, [world] {
                    // Replace alpha's next branch with an undefined instruction
                    // while beta still has an instruction in flight.
                    auto* board = world->board("drain-alpha");
                    ASSERT_TRUE(board->memory().loadBytes(board->cpu().state().r[15],
                        std::vector<std::uint8_t>{0xffU, 0xffU, 0xffU, 0xffU}).hasValue());
                }));
            }
            auto options = runOptions(10'000U);
            options.max_instructions_per_board = budget;
            options.enable_loop_batching = false;
            options.enable_jit = true;
            options.stop_on_board_failure = stop_on_failure;
            const auto candidate = exact.value()->run(options);
            options.detect_spin = true;
            options.spin_threshold = std::numeric_limits<std::uint64_t>::max();
            const auto reference = general.value()->run(options);
            ASSERT_TRUE(candidate && reference);
            EXPECT_EQ(candidate.value().reason, reference.value().reason);
            EXPECT_EQ(candidate.value().end_time_ns, reference.value().end_time_ns);
            EXPECT_EQ(candidate.value().instructions, reference.value().instructions);
            EXPECT_EQ(candidate.value().cycles, reference.value().cycles);
            EXPECT_EQ(exact.value()->trace().jsonLines(), general.value()->trace().jsonLines());
            for (std::size_t index = 0U; index < candidate.value().boards.size(); ++index) {
                const auto& actual = candidate.value().boards[index];
                const auto& expected = reference.value().boards[index];
                EXPECT_EQ(actual.terminal, expected.terminal);
                EXPECT_EQ(actual.result.reason, expected.result.reason);
                EXPECT_EQ(actual.result.instructions, expected.result.instructions);
                EXPECT_EQ(actual.result.cycles, expected.result.cycles);
                EXPECT_EQ(actual.result.time_ns, expected.result.time_ns);
                EXPECT_EQ(actual.result.diagnostic.instruction_address,
                    expected.result.diagnostic.instruction_address);
                EXPECT_EQ(actual.result.diagnostic.raw, expected.result.diagnostic.raw);
                EXPECT_TRUE(fil::cpu::bitwiseEqual(exact.value()->board(actual.name)->cpu().state(),
                    general.value()->board(expected.name)->cpu().state()));
            }
        }
    }
}

} // namespace
