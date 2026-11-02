#include "fil/devices/can_bus.hpp"
#include "fil/sim/board.hpp"
#include "fil/sim/world.hpp"
#include "fil/stm32g4/stm32g4.hpp"
#include "../fixture_support.hpp"
#include "../../src/sim/lane_workers.hpp"

#include <cstdint>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
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

// These scheduler vectors specify a zero-wait-state fixture, independently
// of the device's FLASH_ACR reset value.
bool configureZeroWaitFixture(fil::sim::World& world) {
    for (const auto name : {"alpha", "beta", "gamma"}) {
        auto* board = world.board(name);
        if (board != nullptr && !board->memory().write32(0x40022000U, 0U)) return false;
    }
    return true;
}

fil::sim::WorldRunOptions runOptions(const fil::sim::SimTimeNs duration_ns) {
    fil::sim::WorldRunOptions options;
    options.cross_board_window_ns = 0U; // These vectors specify exact cross-board timing.
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

TEST(WorldTimeTest, CapsuleFailedConditionalPreservesInstructionBudgetCuts) {
    TempWorldTimeConfigs files;
    const auto alpha = files.writeBoard("alpha.json", "alpha");
    // CMP; failed BEQ; five independent increments; B back to CMP. The
    // terminal instruction budget cuts a warmed kernel after the failed B.
    const std::vector<std::uint8_t> code{
        0x00U, 0x28U, 0x00U, 0xd0U, 0x01U, 0x31U, 0x01U, 0x32U,
        0x01U, 0x33U, 0x01U, 0x34U, 0x01U, 0x35U, 0xf7U, 0xe7U,
    };
    for (const std::uint64_t tail : {1U, 2U, 3U, 4U, 5U, 6U, 7U}) {
        SCOPED_TRACE(tail);
        auto exact = fil::sim::World::load(files.network({alpha}));
        auto capsules = fil::sim::World::load(files.network({alpha}));
        ASSERT_TRUE(exact && capsules);
        for (auto* world : {exact.value().get(), capsules.value().get()}) {
            ASSERT_TRUE(configureZeroWaitFixture(*world));
            auto& board = *world->board("alpha");
            const auto start = board.cpu().state().r[15];
            ASSERT_TRUE(board.memory().loadBytes(start, code));
            board.cpu().state().r[0] = 1U; // BEQ never taken.
        }
        auto options = runOptions(1'000'000U);
        options.max_instructions_per_board = 2'000U + tail;
        options.enable_ram_capsules = false;
        const auto reference = exact.value()->run(options);
        options.enable_ram_capsules = true;
        const auto actual = capsules.value()->run(options);
        ASSERT_TRUE(reference && actual);
        EXPECT_GT(actual.value().deferred_prefixes, 0U);
#if defined(FIL_ENABLE_MICRO_JIT) && FIL_ENABLE_MICRO_JIT
        if (fil::cpu::MicroJit::hostSupported()) {
            const auto stats = capsules.value()->board("alpha")->cpu().jitStats();
            // Admission is host-performance dependent; either path must match
            // exact dispatch. Direct emitter tests force native prefix cuts.
            EXPECT_GT(stats.micro_admitted + stats.micro_rejected, 0U);
        }
#endif
        EXPECT_EQ(actual.value().instructions, reference.value().instructions);
        EXPECT_EQ(actual.value().cycles, reference.value().cycles);
        EXPECT_EQ(actual.value().end_time_ns, reference.value().end_time_ns);
        EXPECT_TRUE(fil::cpu::bitwiseEqual(capsules.value()->board("alpha")->cpu().state(),
                                         exact.value()->board("alpha")->cpu().state()));
        EXPECT_EQ(capsules.value()->trace().jsonLines(), exact.value()->trace().jsonLines());
    }
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
    ASSERT_TRUE(configureZeroWaitFixture(*one.value()));
    ASSERT_TRUE(configureZeroWaitFixture(*three.value()));

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
    ASSERT_TRUE(standalone);
    ASSERT_TRUE(standalone.value()->memory().write32(0x40022000U, 0U));
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

    ASSERT_TRUE(configureZeroWaitFixture(*one.value()));
    ASSERT_TRUE(configureZeroWaitFixture(*three.value()));
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
    ASSERT_TRUE(configureZeroWaitFixture(*first.value()));
    ASSERT_TRUE(configureZeroWaitFixture(*second.value()));

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
    for (std::uint32_t i = 0U; i < 60U; ++i) {
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
                ASSERT_TRUE(installIdleLoop(*world, names[i], static_cast<std::uint32_t>(i % 3U + 1U)));
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
    options.enable_ram_capsules = false;
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
    options.enable_ram_capsules = false;
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
    options.enable_ram_capsules = false;
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
    options.enable_ram_capsules = false;
    EXPECT_THROW(static_cast<void>(world.value()->run(options)), std::runtime_error);
    EXPECT_TRUE(memory.readFootprintTracking());
    EXPECT_TRUE(memory.writeJournalTracking());
    ASSERT_TRUE(memory.read32(0x20000000U).hasValue());
    EXPECT_EQ(memory.read32(0x20000000U).value(), 123U);
}

TEST(WorldTimeTest, IndependentSchedulerRestoresTrackingWhenCallbacksThrow) {
    for (const bool previous_tracking : {false, true}) {
        TempWorldTimeConfigs files;
        auto world = fil::sim::World::load(files.network({files.writeBoard("alpha.json", "alpha")}));
        ASSERT_TRUE(world);
        ASSERT_TRUE(installIdleLoop(*world.value(), "alpha", 2U));
        auto& memory = world.value()->board("alpha")->memory();
        memory.setReadFootprintTracking(previous_tracking);
        memory.setWriteJournalTracking(false);
        {
            auto owner = world.value()->eventLoop().useOwner(0U);
            static_cast<void>(world.value()->eventLoop().scheduleAt(1U, [&memory] {
                EXPECT_FALSE(memory.readFootprintTracking());
                EXPECT_FALSE(memory.writeJournalTracking());
                throw std::runtime_error("owner callback failure");
            }));
        }
        auto options = runOptions(1'000U);
        options.cross_board_window_ns = 100'000U;
        EXPECT_THROW(static_cast<void>(world.value()->run(options)), std::runtime_error);
        EXPECT_EQ(memory.readFootprintTracking(), previous_tracking);
        EXPECT_FALSE(memory.writeJournalTracking());
    }
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
            options.enable_ram_capsules = false;
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

TEST(WorldTimeTest, AdaptiveLaneSlotPolicyUsesMeasuredWorkAndSafeFallbacks) {
    using fil::sim::detail::laneActiveExecutionSlots;
    EXPECT_EQ(laneActiveExecutionSlots({}, 8U), 0U);
    EXPECT_EQ(laneActiveExecutionSlots({10U, 10U, 10U, 10U}, 8U), 4U);
    EXPECT_EQ(laneActiveExecutionSlots({100U, 100U, 10U, 10U, 10U, 10U}, 8U), 3U);
    EXPECT_EQ(laneActiveExecutionSlots({100U, 1U, 1U, 1U}, 3U), 2U);
    EXPECT_EQ(laneActiveExecutionSlots({1U, 2U, 3U}, 2U), 2U);
    EXPECT_EQ(laneActiveExecutionSlots({0U, 100U}, 8U), 8U);
    EXPECT_EQ(laneActiveExecutionSlots({UINT64_MAX, 1U}, 8U), 8U);
    EXPECT_EQ(laneActiveExecutionSlots({100U, 100U}, 0U), 0U);
}

TEST(WorldTimeTest, PersistentLaneWorkersJoinEveryWindowAndPropagateExceptions) {
    std::vector<std::size_t> counts(3U, 0U);
    bool fail = false;
    fil::sim::detail::LaneWorkers workers(counts.size(), [&](const auto index) {
        ++counts[index];
        if (fail && index == 1U) throw std::runtime_error("lane failure");
    });
    for (std::size_t window = 0U; window < 10U; ++window) workers.run();
    EXPECT_EQ(counts, (std::vector<std::size_t>{10U, 10U, 10U}));
    fail = true;
    EXPECT_THROW(workers.run(), std::runtime_error);
    EXPECT_EQ(counts, (std::vector<std::size_t>{11U, 11U, 11U}));
    fail = false;
    workers.run();
    EXPECT_EQ(counts, (std::vector<std::size_t>{12U, 12U, 12U}));
}

TEST(WorldTimeTest, PersistentLaneWorkersWakeAfterUnbalancedWindows) {
    std::vector<std::size_t> counts(3U, 0U);
    {
        fil::sim::detail::LaneWorkers workers(counts.size(), [&](const auto index) {
            // Give the coordinator and lighter lanes time to park, then
            // verify all participants finish every subsequent window.
            if (index == 0U) std::this_thread::sleep_for(std::chrono::milliseconds{1});
            ++counts[index];
        });
        for (std::size_t window = 0U; window < 32U; ++window) workers.run();
        EXPECT_EQ(counts, (std::vector<std::size_t>{32U, 32U, 32U}));
    } // Destruction must also wake idle workers before joining them.
    fil::sim::detail::LaneWorkers empty(0U, [](auto) { ADD_FAILURE(); });
    empty.run();
    std::size_t single_count = 0U;
    fil::sim::detail::LaneWorkers single(1U, [&](auto) { ++single_count; });
    for (std::size_t window = 0U; window < 32U; ++window) single.run();
    EXPECT_EQ(single_count, 32U);
    for (const auto count : {0U, 1U, 3U}) {
        fil::sim::detail::LaneWorkers idle(count, [](auto) { ADD_FAILURE(); });
    } // Also join workers that may not yet have observed startup completion.
}

TEST(WorldTimeTest, QueuedLaneWorkersJoinEveryConsumerBeforeTheNextWindow) {
    for (const auto jobs : {0U, 1U, 6U}) {
        for (const auto background : {0U, 1U, 3U, 8U}) {
            SCOPED_TRACE(jobs);
            SCOPED_TRACE(background);
            std::vector<std::atomic<std::size_t>> counts(jobs);
            std::size_t epoch = 0U;
            const auto coordinator = std::this_thread::get_id();
            fil::sim::detail::LaneWorkers workers(jobs, [&](const auto index) {
                EXPECT_EQ(counts[index].fetch_add(1U, std::memory_order_relaxed), epoch);
                if (background == 0U) EXPECT_EQ(std::this_thread::get_id(), coordinator);
                if (index + 1U == jobs && epoch % 31U == 0U)
                    std::this_thread::sleep_for(std::chrono::microseconds{100});
                else if (epoch % 7U == 0U) std::this_thread::yield();
            }, background);
            for (; epoch < 256U; ++epoch) {
                workers.run();
                for (const auto& count : counts)
                    EXPECT_EQ(count.load(std::memory_order_relaxed), epoch + 1U);
            }
        }
    }
}

TEST(WorldTimeTest, QueuedLaneWorkersDrainAllFailuresAndClearThemBeforeRethrow) {
    for (const auto background : {0U, 1U, 3U, 8U}) {
        SCOPED_TRACE(background);
        std::vector<std::size_t> counts(6U, 0U);
        bool fail = true;
        fil::sim::detail::LaneWorkers workers(counts.size(), [&](const auto index) {
            ++counts[index];
            if (fail && (index == 1U || index == 4U))
                throw std::runtime_error(std::to_string(index));
        }, background);
        for (unsigned invocation = 0U; invocation < 16U; ++invocation) {
            fail = true;
            try {
                workers.run();
                FAIL() << "expected both failing jobs to be captured";
            } catch (const std::runtime_error& error) {
                EXPECT_STREQ(error.what(), "1");
            }
            for (const auto count : counts) EXPECT_EQ(count, 2U * invocation + 1U);
            fail = false;
            EXPECT_NO_THROW(workers.run());
            for (const auto count : counts) EXPECT_EQ(count, 2U * invocation + 2U);
        }
    }
}

TEST(WorldTimeTest, QueuedLaneWorkersRestoreCoordinatorOwnerDomainsAfterExceptions) {
    for (const auto background : {0U, 1U, 3U}) {
        SCOPED_TRACE(background);
        fil::sim::EventLoop events;
        auto domains = events.useLaneDomains(6U);
        std::vector<std::size_t> counts(6U, 0U);
        std::vector<fil::devices::DeferredCanOutbox> outboxes(6U);
        std::atomic<bool> background_failed{false};
        const auto coordinator = std::this_thread::get_id();
        bool fail = true;
        std::uint64_t at = 1U;
        {
            fil::sim::detail::LaneWorkers workers(counts.size(), [&](const auto index) {
                const auto id = static_cast<fil::sim::EventOwner>(index);
                auto lane = events.useLaneDomain(id);
                auto owner = events.useOwner(id);
                EXPECT_FALSE(fil::devices::DeferredCanScope::active());
                fil::devices::DeferredCanScope communications(outboxes[index],
                    static_cast<std::uint32_t>(index), events, id);
                EXPECT_TRUE(fil::devices::DeferredCanScope::active());
                EXPECT_EQ(fil::devices::DeferredCanScope::boardIndex(), index);
                EXPECT_EQ(fil::devices::DeferredCanScope::nextSequence(), 0U);
                EXPECT_EQ(events.activeOwner(), id);
                static_cast<void>(events.scheduleAt(at, [&, index] { ++counts[index]; }));
                EXPECT_FALSE(events.runOwnedEvents(id, at).same_time_limit_hit);
                if (fail && background != 0U) {
                    if (std::this_thread::get_id() == coordinator) {
                        background_failed.wait(false, std::memory_order_acquire);
                    } else {
                        background_failed.store(true, std::memory_order_release);
                        background_failed.notify_one();
                        throw std::runtime_error("background owner failure");
                    }
                } else if (fail && index == 2U) throw std::runtime_error("coordinator owner failure");
            }, background);
            EXPECT_THROW(workers.run(), std::runtime_error);
            if (background != 0U) EXPECT_TRUE(background_failed.load(std::memory_order_acquire));
            EXPECT_FALSE(fil::devices::DeferredCanScope::active());
            EXPECT_EQ(events.activeOwner(), fil::sim::shared_event_owner);
            EXPECT_NO_THROW(static_cast<void>(events.now()));
            fail = false;
            ++at;
            EXPECT_NO_THROW(workers.run());
            EXPECT_FALSE(fil::devices::DeferredCanScope::active());
            EXPECT_EQ(events.activeOwner(), fil::sim::shared_event_owner);
            EXPECT_NO_THROW(static_cast<void>(events.runSharedEvents(at)));
        }
        domains.close();
        for (const auto count : counts) EXPECT_EQ(count, 2U);
    }
}

TEST(WorldTimeTest, ParallelOwnerWindowsMatchSerialOwnerExecution) {
    TempWorldTimeConfigs files;
    const auto alpha = files.writeBoard("alpha.json", "alpha");
    const auto beta = files.writeBoard("beta.json", "beta");
    auto parallel = fil::sim::World::load(files.network({alpha, beta}));
    auto serial = fil::sim::World::load(files.network({alpha, beta}));
    ASSERT_TRUE(parallel && serial);
    for (auto* world : {parallel.value().get(), serial.value().get()}) {
        ASSERT_TRUE(configureZeroWaitFixture(*world));
        ASSERT_TRUE(configureSysTickIdleLoop(*world, "alpha", 3U));
        ASSERT_TRUE(configureSysTickIdleLoop(*world, "beta", 7U));
        world->setDiagnosticsEnabled(false);
    }
    auto options = runOptions(100'000U);
    options.max_instructions_per_board = 10'001U;
    options.cross_board_window_ns = 10'000U;
    for (unsigned invocation = 0U; invocation < 3U; ++invocation) {
        SCOPED_TRACE(invocation);
        options.enable_parallel_lanes = true;
        const auto actual = parallel.value()->run(options);
        options.enable_parallel_lanes = false;
        const auto expected = serial.value()->run(options);
        ASSERT_TRUE(actual && expected);
        if (std::thread::hardware_concurrency() > 1U) EXPECT_EQ(actual.value().parallel_lanes, 2U);
        EXPECT_EQ(expected.value().parallel_lanes, 0U);
        EXPECT_EQ(actual.value().instructions, expected.value().instructions);
        EXPECT_EQ(actual.value().cycles, expected.value().cycles);
        EXPECT_EQ(actual.value().event_callbacks, expected.value().event_callbacks);
        EXPECT_EQ(actual.value().end_time_ns, expected.value().end_time_ns);
        for (const auto name : {"alpha", "beta"}) {
            EXPECT_TRUE(fil::cpu::bitwiseEqual(parallel.value()->board(name)->cpu().state(),
                                              serial.value()->board(name)->cpu().state()));
            EXPECT_EQ(parallel.value()->board(name)->eventLoop().now(name == std::string_view{"alpha"} ? 0U : 1U),
                      serial.value()->board(name)->eventLoop().now(name == std::string_view{"alpha"} ? 0U : 1U));
        }
    }
}

TEST(WorldTimeTest, OwnerRamRunnerMatchesCheckedMmioAndInterruptsAfterWarmup) {
    TempWorldTimeConfigs files;
    const auto config = files.network({files.writeBoard("alpha.json", "alpha"),
                                       files.writeBoard("beta.json", "beta")});
    auto fast = fil::sim::World::load(config);
    auto checked = fil::sim::World::load(config);
    ASSERT_TRUE(fast && checked);
    for (auto* world : {fast.value().get(), checked.value().get()}) {
        ASSERT_TRUE(configureZeroWaitFixture(*world));
        for (const auto name : {"alpha", "beta"}) {
            ASSERT_TRUE(configureSysTickIdleLoop(*world, name, 2U));
            auto& board = *world->board(name);
            const auto start = board.cpu().state().r[15];
            const std::vector<std::uint8_t> code{
                0x08U, 0x68U, // ldr r0, [r1]
                0x01U, 0x30U, // adds r0, #1
                0x18U, 0x60U, // str r0, [r3]
                0x22U, 0x68U, // ldr r2, [r4] (live DWT_CYCCNT MMIO)
                0xfaU, 0xe7U, // b start
            };
            ASSERT_TRUE(board.memory().loadBytes(start, code));
            ASSERT_TRUE(board.memory().write32(0x20000040U, 0U));
            ASSERT_TRUE(board.memory().write32(0xe0001000U, 1U));
            board.cpu().state().r[1] = 0x20000040U;
            board.cpu().state().r[3] = 0x20000040U;
            board.cpu().state().r[4] = 0xe0001004U;
        }
        world->setDiagnosticsEnabled(false);
    }
    std::array<std::vector<std::uint32_t>, 2U> fast_observations, checked_observations;
    for (std::size_t lane = 0U; lane < 2U; ++lane) {
        const auto name = lane == 0U ? "alpha" : "beta";
        for (const bool candidate : {false, true}) {
            auto& world = candidate ? *fast.value() : *checked.value();
            auto* board = world.board(name);
            auto& observations = candidate ? fast_observations[lane] : checked_observations[lane];
            auto owner = world.eventLoop().useOwner(static_cast<fil::sim::EventOwner>(lane));
            for (const auto at : {17U, 100U, 1'999U, 3'125U, 7'777U, 100'000U, 100'001U}) {
                static_cast<void>(world.eventLoop().scheduleAt(at, [board, &observations] {
                    observations.push_back(board->cpu().state().r[0]);
                }));
            }
        }
    }
    auto options = runOptions(1'000'000U);
    options.cross_board_window_ns = 100'000U;
    for (const auto budget : {3U, 10'003U, 4'097U, 1U, 32'000U}) {
        SCOPED_TRACE(budget);
        options.max_instructions_per_board = budget;
        options.enable_jit = true;
        const auto actual = fast.value()->run(options);
        options.enable_jit = false;
        const auto expected = checked.value()->run(options);
        ASSERT_TRUE(actual && expected);
        EXPECT_EQ(actual.value().reason, expected.value().reason);
        EXPECT_EQ(actual.value().instructions, expected.value().instructions);
        EXPECT_EQ(actual.value().cycles, expected.value().cycles);
        EXPECT_EQ(actual.value().end_time_ns, expected.value().end_time_ns);
        EXPECT_EQ(actual.value().event_callbacks, expected.value().event_callbacks);
        EXPECT_EQ(fast_observations, checked_observations);
        for (const auto name : {"alpha", "beta"}) {
            auto& a = *fast.value()->board(name);
            auto& b = *checked.value()->board(name);
            EXPECT_TRUE(fil::cpu::bitwiseEqual(a.cpu().state(), b.cpu().state()));
            for (const auto& region : a.memory().regions()) {
                if (region.kind != fil::mem::RegionKind::ram) continue;
                for (std::uint32_t offset = 0U; offset < region.size; ++offset) {
                    const auto address = region.base + offset;
                    ASSERT_EQ(a.memory().read8(address).value(), b.memory().read8(address).value())
                        << "RAM address " << address;
                }
            }
        }
    }
}

TEST(WorldTimeTest, IndependentExecutionUsesLiveRamWithoutReplayOrJournaling) {
    for (const bool previous_tracking : {false, true}) {
        TempWorldTimeConfigs files;
        auto world = fil::sim::World::load(files.network({files.writeBoard("alpha.json", "alpha")}));
        ASSERT_TRUE(world);
        ASSERT_TRUE(configureZeroWaitFixture(*world.value()));
        auto& board = *world.value()->board("alpha");
        auto& memory = board.memory();
        const auto start = board.cpu().state().r[15];
        const std::vector<std::uint8_t> code{
            0x08U, 0x68U, // ldr r0, [r1]
            0x00U, 0x28U, // cmp r0, #0
            0xfcU, 0xe7U, // b start
        };
        ASSERT_TRUE(memory.loadBytes(start, code));
        ASSERT_TRUE(memory.write32(0x20000040U, 0U));
        board.cpu().state().r[1] = 0x20000040U;
        memory.setReadFootprintTracking(previous_tracking);
        world.value()->setDiagnosticsEnabled(false);
        bool observed = false;
        {
            auto owner = world.value()->eventLoop().useOwner(0U);
            static_cast<void>(world.value()->eventLoop().scheduleAt(5'000'000U, [&] {
                observed = true;
                EXPECT_FALSE(memory.readFootprintTracking());
                EXPECT_FALSE(memory.writeJournalTracking());
                // Execution must observe the real external write immediately.
                EXPECT_TRUE(memory.write32(0x20000040U, 5U));
            }));
        }
        auto options = runOptions(20'000'000U);
        options.max_instructions_per_board = 1'000'000U;
        options.cross_board_window_ns = 100'000U;
        const auto result = world.value()->run(options);
        ASSERT_TRUE(result);
        EXPECT_TRUE(observed);
        EXPECT_EQ(board.cpu().state().r[0], 5U);
        EXPECT_EQ(result.value().memoized_prefixes, 0U);
        EXPECT_EQ(memory.readFootprintTracking(), previous_tracking);
        EXPECT_TRUE(memory.writeJournalTracking());
    }
}

TEST(WorldTimeTest, OwnerMmioReadsKeepUnchangedEventHorizonAndExactClock) {
    for (const auto flash_acr : {0U, 0x102U}) {
        SCOPED_TRACE(flash_acr);
        TempWorldTimeConfigs files;
        const auto config = files.network({files.writeBoard("alpha.json", "alpha")});
        auto exact = fil::sim::World::load(config);
        auto jit = fil::sim::World::load(config);
        ASSERT_TRUE(exact && jit);
        for (auto* world : {exact.value().get(), jit.value().get()}) {
            ASSERT_TRUE(configureZeroWaitFixture(*world));
            auto& board = *world->board("alpha");
            const auto start = board.cpu().state().r[15];
            const std::vector<std::uint8_t> code{
                0x01U, 0x30U, // adds r0, #1
                0x01U, 0x30U, // adds r0, #1
                0x0aU, 0x68U, // ldr r2, [r1] (DWT_CYCCNT)
                0xfbU, 0xe7U, // b start
            };
            ASSERT_TRUE(board.memory().loadBytes(start, code));
            ASSERT_TRUE(board.memory().write32(0xe0001000U, 1U));
            ASSERT_TRUE(board.memory().write32(0x40022000U, flash_acr));
            board.cpu().state().r[1] = 0xe0001004U;
            // This event is already part of admission; a read must not stop a
            // trace merely because the same future event remains scheduled.
            auto owner = world->eventLoop().useOwner(0U);
            static_cast<void>(world->eventLoop().scheduleAt(10'000'000U, [] {}));
            world->setDiagnosticsEnabled(false);
        }
        auto options = runOptions(1'000'000U);
        options.max_instructions_per_board = 20'003U;
        options.cross_board_window_ns = 100'000U;
        options.enable_jit = false;
        const auto reference = exact.value()->run(options);
        options.enable_jit = true;
        const auto result = jit.value()->run(options);
        ASSERT_TRUE(reference && result);
        EXPECT_EQ(result.value().reason, reference.value().reason);
        EXPECT_EQ(result.value().instructions, reference.value().instructions);
        EXPECT_EQ(result.value().cycles, reference.value().cycles);
        EXPECT_EQ(result.value().end_time_ns, reference.value().end_time_ns);
        auto& a = *jit.value()->board("alpha");
        auto& b = *exact.value()->board("alpha");
        EXPECT_TRUE(fil::cpu::bitwiseEqual(a.cpu().state(), b.cpu().state()));
        EXPECT_EQ(a.cpu().state().r, b.cpu().state().r);
        EXPECT_EQ(a.cpu().state().xpsr, b.cpu().state().xpsr);
        const auto stats = a.cpu().jitStats();
        if (stats.trace_executions != 0U)
            EXPECT_GT(stats.trace_instructions, 8U * stats.trace_executions)
                << "unchanged horizon allows traces across multiple MMIO reads";
    }
}

TEST(WorldTimeTest, IndependentWindowsPreserveLocalInterruptsAndInstructionBudgets) {
    TempWorldTimeConfigs files;
    const auto alpha = files.writeBoard("alpha.json", "alpha");
    const auto beta = files.writeBoard("beta.json", "beta");
    for (const auto window : {1'000U, 10'000U}) {
        SCOPED_TRACE(window);
        auto independent = fil::sim::World::load(files.network({alpha, beta}));
        auto lanes_reference = fil::sim::World::load(files.network({alpha, beta}));
        auto lockstep = fil::sim::World::load(files.network({alpha, beta}));
        ASSERT_TRUE(independent && lanes_reference && lockstep);
        for (auto* world : {independent.value().get(), lanes_reference.value().get(),
                            lockstep.value().get()}) {
            ASSERT_TRUE(configureZeroWaitFixture(*world));
            ASSERT_TRUE(configureSysTickIdleLoop(*world, "alpha", 3U));
            ASSERT_TRUE(configureSysTickIdleLoop(*world, "beta", 7U));
        }
        auto options = runOptions(100'000U);
        options.max_instructions_per_board = 1'001U;
        options.cross_board_window_ns = window;
        options.enable_parallel_lanes = true;
        const auto actual = independent.value()->run(options);
        // The bounded window itself is only allowed to change *when* work is
        // attributed relative to the exact lockstep frontier, never what a board
        // retires for a given start. Executing the identical window serially must
        // therefore reproduce the lane result byte for byte.
        options.enable_parallel_lanes = false;
        const auto expected = lanes_reference.value()->run(options);
        ASSERT_TRUE(actual && expected);
        EXPECT_EQ(actual.value().reason, expected.value().reason);
        EXPECT_EQ(actual.value().instructions, expected.value().instructions);
        EXPECT_EQ(actual.value().cycles, expected.value().cycles);
        EXPECT_EQ(actual.value().end_time_ns, expected.value().end_time_ns);
        for (const auto name : {"alpha", "beta"}) {
            EXPECT_TRUE(fil::cpu::bitwiseEqual(independent.value()->board(name)->cpu().state(),
                                              lanes_reference.value()->board(name)->cpu().state()));
        }
        // The window-0, capsules-disabled world is a different scheduler: it
        // offers the memoized idle-loop certificates at every instruction
        // boundary, so it fast-forwards idle time that the batched committed-span
        // executor in the independent path reaches later. Byte equality with it is
        // not an invariant of this path; the vectors above that do require exact
        // parity are ParallelOwnerWindowsMatchSerialOwnerExecution and
        // OwnerRamRunnerMatchesCheckedMmioAndInterruptsAfterWarmup. What must
        // hold here is that the bounded-window path stays inside every per-board
        // instruction budget, only stops on a normal budget reason, makes
        // progress on every board, and reports the sum of its board results.
        options.cross_board_window_ns = 0U;
        options.enable_ram_capsules = false;
        const auto baseline = lockstep.value()->run(options);
        ASSERT_TRUE(baseline);
        for (const auto* result : {&actual.value(), &baseline.value()}) {
            EXPECT_NE(result->reason, fil::sim::WorldStopReason::board_failure);
            EXPECT_NE(result->reason, fil::sim::WorldStopReason::all_boards_stopped);
            ASSERT_EQ(result->boards.size(), 2U);
            std::uint64_t reported = 0U;
            for (const auto& board : result->boards) {
                EXPECT_LE(board.result.instructions, options.max_instructions_per_board);
                reported += board.result.instructions;
            }
            EXPECT_EQ(reported, result->instructions);
            EXPECT_GT(result->instructions, 0U);
        }
        // The instruction budget is per board, so a windowed run that stops on it
        // must have retired it on both boards; a time-budget stop must not have
        // exceeded it anywhere.
        if (actual.value().reason == fil::sim::WorldStopReason::instruction_budget) {
            for (const auto& board : actual.value().boards) {
                EXPECT_EQ(board.result.instructions, options.max_instructions_per_board);
            }
        }
        for (const auto name : {"alpha", "beta"}) {
            EXPECT_GT(independent.value()->board(name)->cpu().state().r[6], 0U);
        }
    }
}

TEST(WorldTimeTest, IndependentSliceServicesRendezvousInterruptBeforeForeground) {
    TempWorldTimeConfigs files;
    const auto alpha = files.writeBoard("alpha.json", "alpha");
    auto world = fil::sim::World::load(files.network({alpha}));
    ASSERT_TRUE(world);
    ASSERT_TRUE(configureZeroWaitFixture(*world.value()));
    ASSERT_TRUE(configureSysTickIdleLoop(*world.value(), "alpha", 3U));
    auto& board = *world.value()->board("alpha");
    ASSERT_TRUE(board.memory().write32(0xe000e010U, 0U)); // Only the rendezvous pends SysTick.
    bool observed = false;
    auto& loop = world.value()->eventLoop();
    static_cast<void>(loop.scheduleAt(250U, [&] {
        ASSERT_TRUE(board.memory().write32(0xe000ed04U, 1U << 26U));
        auto owner = loop.useOwner(0U);
        static_cast<void>(loop.scheduleAfter(1U, [&] {
            observed = true;
            EXPECT_EQ(board.cpu().state().ipsr(), 15U);
            EXPECT_EQ(board.cpu().state().r[6], 1U);
        }));
    }));
    auto options = runOptions(2'000U);
    options.max_instructions_per_board = 100'000U;
    options.cross_board_window_ns = 1'000U;
    ASSERT_TRUE(world.value()->run(options));
    EXPECT_TRUE(observed);
}

TEST(WorldTimeTest, IndependentFailureStopsBeforeLaterLanesAndSharedCallbacks) {
    TempWorldTimeConfigs files;
    const auto alpha = files.writeBoard("alpha.json", "alpha");
    const auto beta = files.writeBoard("beta.json", "beta");
    auto world = fil::sim::World::load(files.network({alpha, beta}));
    ASSERT_TRUE(world);
    ASSERT_TRUE(configureZeroWaitFixture(*world.value()));
    auto& board = *world.value()->board("alpha");
    const std::vector<std::uint8_t> udf{0x00U, 0xdeU};
    ASSERT_TRUE(board.memory().loadBytes(board.cpu().state().r[15], udf));
    const auto beta_before = world.value()->board("beta")->cpu().state();
    bool called = false;
    static_cast<void>(world.value()->eventLoop().scheduleAt(250U, [&] { called = true; }));
    auto options = runOptions(10'000U);
    options.cross_board_window_ns = 1'000U;
    // The "later lanes never run" guarantee is specific to the serial
    // independent scheduler; parallel lanes execute their window concurrently
    // and observe a peer failure only at the next rendezvous.
    options.enable_parallel_lanes = false;
    const auto result = world.value()->run(options);
    ASSERT_TRUE(result);
    EXPECT_EQ(result.value().parallel_lanes, 0U);
    EXPECT_EQ(result.value().reason, fil::sim::WorldStopReason::board_failure);
    EXPECT_FALSE(called);
    EXPECT_EQ(result.value().boards[1].result.instructions, 0U);
    EXPECT_EQ(result.value().boards[1].result.reason, fil::sim::BoardStopReason::synchronization_required);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(beta_before, world.value()->board("beta")->cpu().state()));
}

TEST(WorldTimeTest, IndependentWindowsDeliverSharedEffectsAtNextRendezvous) {
    TempWorldTimeConfigs files;
    const auto alpha = files.writeBoard("alpha.json", "alpha");
    auto world = fil::sim::World::load(files.network({alpha}));
    ASSERT_TRUE(world);
    ASSERT_TRUE(configureZeroWaitFixture(*world.value()));
    ASSERT_TRUE(installIdleLoop(*world.value(), "alpha", 3U));
    auto& loop = world.value()->eventLoop();
    fil::sim::SimTimeNs observed_local = 0U;
    fil::sim::SimTimeNs observed_shared = 0U;
    static_cast<void>(loop.scheduleAt(250U, [&] {
        observed_shared = loop.now();
        observed_local = loop.now(0U);
        world.value()->board("alpha")->cpu().state().r[7] = 42U;
    }));
    auto options = runOptions(2'000U);
    options.max_instructions_per_board = 100'000U;
    options.cross_board_window_ns = 1'000U;
    const auto result = world.value()->run(options);
    ASSERT_TRUE(result);
    EXPECT_EQ(result.value().reason, fil::sim::WorldStopReason::time_budget);
    EXPECT_EQ(result.value().end_time_ns, 2'000U);
    EXPECT_EQ(observed_shared, 250U);
    EXPECT_GE(observed_local, 1'000U);
    EXPECT_LE(observed_local, 1'250U); // One window plus an ordinary in-flight branch.
    EXPECT_EQ(world.value()->board("alpha")->cpu().state().r[7], 42U);
    const auto before = world.value()->board("alpha")->cpu().state();
    options.max_instructions_per_board = 0U;
    const auto zero = world.value()->run(options);
    ASSERT_TRUE(zero);
    EXPECT_EQ(zero.value().instructions, 0U);
    EXPECT_EQ(zero.value().reason, fil::sim::WorldStopReason::instruction_budget);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(before, world.value()->board("alpha")->cpu().state()));
}

} // namespace
