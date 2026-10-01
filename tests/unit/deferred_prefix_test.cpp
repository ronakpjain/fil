#include "fil/sim/world.hpp"
#include "fil/stm32g4/fdcan.hpp"
#include "fil/stm32g4/stm32g4.hpp"
#include "../fixture_support.hpp"

#include <gtest/gtest.h>
#include <sstream>
#include <vector>

namespace {

class DeferredNetwork {
public:
    fil::config::NetworkConfig config(bool receiver_first) {
        fil::config::NetworkConfig result;
        result.name = "deferred-test";
        result.source_path = files_.root() / "network.json";
        result.buses.push_back({"vehicle", 500000U});
        for (const auto& name : receiver_first
                ? std::vector<std::string>{"receiver", "sender"}
                : std::vector<std::string>{"sender", "receiver"}) {
            const auto fixture = fil::test::fixtureBoardConfig(name);
            std::ostringstream board;
            board << "{\"schema_version\":1,\"name\":\"" << name
                  << "\",\"mcu\":\"" << fixture.mcu_path.string()
                  << "\",\"elf\":\"" << fixture.elf_path.string()
                  << "\",\"vector_base\":\"0x08000000\","
                  << "\"can\":{\"FDCAN1\":{\"bus\":\"vehicle\",\"loopback\":false}}}";
            result.board_paths.push_back(files_.write(name + ".json", board.str()));
        }
        return result;
    }
private:
    fil::test::TemporaryDirectory files_{"fil-deferred-tests"};
};

void installCode(fil::sim::Board& board, const std::vector<std::uint8_t>& code) {
    ASSERT_TRUE(board.memory().loadBytes(board.cpu().state().r[15], code));
    // Prepare the entry without executing it, so the first interval exercises
    // deferred execution rather than only warming the instruction cache.
    for (unsigned i = 0; i < 64U; ++i) static_cast<void>(board.cpu().prepareJitBlock());
}

void configureReceiver(fil::sim::Board& board) {
    auto* fdcan = board.peripherals().fdcan("FDCAN1");
    ASSERT_NE(fdcan, nullptr);
    using Fdcan = fil::stm32g4::FdcanPeripheral;
    ASSERT_TRUE(fdcan->write(Fdcan::ieOffset, fil::mem::AccessSize::word,
                            Fdcan::interruptRxFifo0New, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::ileOffset, fil::mem::AccessSize::word, 1U, {}));
    ASSERT_TRUE(fdcan->write(Fdcan::cccrOffset, fil::mem::AccessSize::word, 0U, {}));
    ASSERT_TRUE(board.memory().write32(0xe000e100U, 1U << 21U));
    const auto handler = board.cpu().state().r[15] + 0x100U;
    const std::vector<std::uint8_t> handler_code{0x01U, 0x36U, 0x70U, 0x47U};
    ASSERT_TRUE(board.memory().loadBytes(handler, handler_code));
    const auto vector = handler | 1U;
    const std::vector<std::uint8_t> bytes{
        static_cast<std::uint8_t>(vector), static_cast<std::uint8_t>(vector >> 8U),
        static_cast<std::uint8_t>(vector >> 16U), static_cast<std::uint8_t>(vector >> 24U)};
    ASSERT_TRUE(board.memory().loadBytes(board.image().vectorBase() + 37U * 4U, bytes));
}

TEST(DeferredPrefixTest, WarmIdleCallPeriodEveryCutMatchesExactCpuMemoryAndTimers) {
    // A small FreeRTOS-style idle call: the callee saves/restores R3/LR and
    // polls a private-RAM deletion count, then the caller polls the ready count.
    const std::vector<std::uint8_t> code{
        0x08U,0xb5U, 0x00U,0xf0U,0x05U,0xf8U, 0x07U,0x4bU,
        0x1bU,0x68U, 0x01U,0x2bU, 0xf9U,0xd9U, 0xfeU,0xe7U,
        0x08U,0xb5U, 0x00U,0xe0U, 0x00U,0xbfU, 0x02U,0x4bU,
        0x1bU,0x68U, 0x00U,0x2bU, 0xfaU,0xd1U, 0x08U,0xbdU,
        0x40U,0x00U,0x00U,0x20U, 0x44U,0x00U,0x00U,0x20U};
    for (std::size_t count = 1U; count <= 36U; ++count) {
        SCOPED_TRACE(count);
        auto exact = fil::sim::Board::load(fil::test::fixtureBoardConfig());
        auto speculative = fil::sim::Board::load(fil::test::fixtureBoardConfig());
        ASSERT_TRUE(exact && speculative);
        fil::sim::BoardRunOptions options;
        options.duration_ns = 0U;
        options.enable_loop_batching = false;
        options.enable_jit = true;
        options.max_instructions = 13U;
        for (auto* board : {exact.value().get(), speculative.value().get()}) {
            board->cpu().state().r[3] = 1U;
            board->cpu().setNativeSingleInstructionJitEnabled(false);
            ASSERT_TRUE(board->memory().write32(0x20000040U, 0U));
            ASSERT_TRUE(board->memory().write32(0x20000044U, 1U));
            ASSERT_TRUE(board->memory().write32(0xe0001000U, 1U));
            installCode(*board, code);
            const auto warmed = board->run(options);
            ASSERT_EQ(warmed.instructions, 13U);
        }
        const auto start = exact.value()->eventLoop().now();
        const auto stack = exact.value()->cpu().state().r[13];
        const auto entry = speculative.value()->cpu().state();
        const auto base = entry.r[15] - 2U;
        for (const auto offset : {2U, 6U, 8U, 10U, 12U, 16U, 18U, 22U, 24U, 26U, 28U, 30U}) {
            speculative.value()->cpu().state().r[15] = base + offset;
            for (unsigned i = 0U; i < 64U; ++i) static_cast<void>(speculative.value()->cpu().prepareJitBlock(true));
        }
        speculative.value()->cpu().state() = entry;
        const auto prefix = speculative.value()->prepareReversibleRamPrefix(36U);
        ASSERT_TRUE(prefix);
        ASSERT_EQ(prefix->count, 36U);
        ASSERT_TRUE(prefix->period_certificate);
        const auto committed = speculative.value()->materializeReversibleRamPrefix(*prefix, count);
        options.max_instructions = count;
        const auto reference = exact.value()->run(options);
        ASSERT_EQ(reference.instructions, count);
        EXPECT_EQ(committed.cpu_result.instructions, count);
        EXPECT_EQ(committed.cpu_result.cycles, reference.cycles);
        EXPECT_EQ(committed.elapsed_ns, reference.time_ns - start);
        EXPECT_TRUE(fil::cpu::bitwiseEqual(speculative.value()->cpu().state(), exact.value()->cpu().state()));
        for (const auto address : {0x20000040U, 0x20000044U, stack - 8U, stack - 4U, stack, stack + 4U}) {
            const auto expected = exact.value()->memory().read32(address);
            const auto actual = speculative.value()->memory().read32(address);
            ASSERT_TRUE(expected && actual);
            EXPECT_EQ(actual.value(), expected.value());
        }
        const auto expected_counter = exact.value()->memory().read32(0xe0001004U);
        const auto actual_counter = speculative.value()->memory().read32(0xe0001004U);
        ASSERT_TRUE(expected_counter && actual_counter);
        EXPECT_EQ(actual_counter.value(), expected_counter.value());

        // Now admit from the just-materialized phase through the immutable
        // certificate, with a varying partial cut at every loop iteration.
        const auto hit_start = exact.value()->eventLoop().now();
        const auto hit_prefix = speculative.value()->prepareReversibleRamPrefix(36U);
        ASSERT_TRUE(hit_prefix);
        ASSERT_GT(hit_prefix->memoized_count, 0U);
        const std::size_t hit_cut = 1U + ((count - 1U) % hit_prefix->memoized_count);
        const auto hit_commit = speculative.value()->materializeReversibleRamPrefix(
            *hit_prefix, hit_cut);
        options.max_instructions = hit_cut;
        const auto hit_reference = exact.value()->run(options);
        ASSERT_EQ(hit_reference.instructions, hit_cut);
        EXPECT_EQ(hit_commit.cpu_result.cycles, hit_reference.cycles);
        EXPECT_EQ(hit_commit.elapsed_ns, hit_reference.time_ns - hit_start);
        EXPECT_TRUE(fil::cpu::bitwiseEqual(speculative.value()->cpu().state(),
                                          exact.value()->cpu().state()));
        EXPECT_EQ(speculative.value()->memory().read32(0x20000040U).value(),
                  exact.value()->memory().read32(0x20000040U).value());
        EXPECT_EQ(speculative.value()->memory().read32(0x20000044U).value(),
                  exact.value()->memory().read32(0x20000044U).value());
        EXPECT_EQ(speculative.value()->memory().read32(0xe0001004U).value(),
                  exact.value()->memory().read32(0xe0001004U).value());
    }
}

TEST(DeferredPrefixTest, IdempotentPeriodAdmissionReusesRotatedPhasesWithoutExecution) {
    const std::vector<std::uint8_t> code{
        0x08U,0xb5U, 0x00U,0xf0U,0x05U,0xf8U, 0x07U,0x4bU,
        0x1bU,0x68U, 0x01U,0x2bU, 0xf9U,0xd9U, 0xfeU,0xe7U,
        0x08U,0xb5U, 0x00U,0xe0U, 0x00U,0xbfU, 0x02U,0x4bU,
        0x1bU,0x68U, 0x00U,0x2bU, 0xfaU,0xd1U, 0x08U,0xbdU,
        0x40U,0x00U,0x00U,0x20U, 0x44U,0x00U,0x00U,0x20U};
    auto exact = fil::sim::Board::load(fil::test::fixtureBoardConfig());
    auto cached = fil::sim::Board::load(fil::test::fixtureBoardConfig());
    ASSERT_TRUE(exact && cached);
    fil::sim::BoardRunOptions warm;
    warm.duration_ns = 0U;
    warm.enable_loop_batching = false;
    warm.enable_jit = true;
    warm.max_instructions = 13U;
    for (auto* board : {exact.value().get(), cached.value().get()}) {
        board->cpu().state().r[3] = 1U;
        board->cpu().setNativeSingleInstructionJitEnabled(false);
        ASSERT_TRUE(board->memory().write32(0x20000040U, 0U));
        ASSERT_TRUE(board->memory().write32(0x20000044U, 1U));
        ASSERT_TRUE(board->memory().write32(0xe0001000U, 1U));
        installCode(*board, code);
        ASSERT_EQ(board->run(warm).instructions, 13U);
    }
    const auto entry = cached.value()->cpu().state();
    const auto base = entry.r[15] - 2U;
    for (const auto offset : {2U, 6U, 8U, 10U, 12U, 16U, 18U, 22U, 24U, 26U, 28U, 30U}) {
        cached.value()->cpu().state().r[15] = base + offset;
        for (unsigned i = 0U; i < 64U; ++i) {
            static_cast<void>(cached.value()->cpu().prepareJitBlock(true));
        }
    }
    cached.value()->cpu().state() = entry;
    const auto initial = cached.value()->prepareReversibleRamPrefix(36U);
    ASSERT_TRUE(initial);
    ASSERT_TRUE(initial->period_certificate);
    static_cast<void>(cached.value()->materializeReversibleRamPrefix(*initial, 1U));
    warm.max_instructions = 1U;
    ASSERT_EQ(exact.value()->run(warm).instructions, 1U);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cached.value()->cpu().state(), exact.value()->cpu().state()));

    const auto start = exact.value()->eventLoop().now();
    const auto repeated = cached.value()->prepareReversibleRamPrefix(12U);
    ASSERT_TRUE(repeated);
    EXPECT_GT(repeated->memoized_count, 0U);
    EXPECT_NE(repeated->period_phase, 0U);
    const std::uint8_t memoized = repeated->memoized_count;
    const auto result = cached.value()->materializeReversibleRamPrefix(*repeated, memoized);
    warm.max_instructions = memoized;
    const auto reference = exact.value()->run(warm);
    ASSERT_EQ(reference.instructions, memoized);
    EXPECT_EQ(result.cpu_result.cycles, reference.cycles);
    EXPECT_EQ(result.elapsed_ns, reference.time_ns - start);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cached.value()->cpu().state(), exact.value()->cpu().state()));
    for (const auto address : {0x20000040U, 0x20000044U}) {
        EXPECT_EQ(cached.value()->memory().read32(address).value(),
                  exact.value()->memory().read32(address).value());
    }
    EXPECT_EQ(cached.value()->memory().read32(0xe0001004U).value(),
              exact.value()->memory().read32(0xe0001004U).value());

    // A successful admission rebases validation: unrelated external writes
    // remain compatible, while a write to any period input invalidates reuse.
    ASSERT_TRUE(cached.value()->memory().write32(0x20000060U, 0x12345678U));
    const auto unrelated = cached.value()->prepareReversibleRamPrefix(12U);
    ASSERT_TRUE(unrelated);
    EXPECT_GT(unrelated->memoized_count, 0U);
    ASSERT_TRUE(cached.value()->memory().write32(0x20000040U, 1U));
    const auto changed_input = cached.value()->prepareReversibleRamPrefix(12U);
    ASSERT_TRUE(changed_input);
    EXPECT_EQ(changed_input->memoized_count, 0U);
}

TEST(DeferredPrefixTest, RestorationInvalidatesImmutablePeriodAdmission) {
    auto board = fil::sim::Board::load(fil::test::fixtureBoardConfig());
    ASSERT_TRUE(board);
    const std::vector<std::uint8_t> code{
        0x08U,0xb5U, 0x00U,0xf0U,0x05U,0xf8U, 0x07U,0x4bU,
        0x1bU,0x68U, 0x01U,0x2bU, 0xf9U,0xd9U, 0xfeU,0xe7U,
        0x08U,0xb5U, 0x00U,0xe0U, 0x00U,0xbfU, 0x02U,0x4bU,
        0x1bU,0x68U, 0x00U,0x2bU, 0xfaU,0xd1U, 0x08U,0xbdU,
        0x40U,0x00U,0x00U,0x20U, 0x44U,0x00U,0x00U,0x20U};
    board.value()->cpu().state().r[3] = 1U;
    board.value()->cpu().setNativeSingleInstructionJitEnabled(false);
    ASSERT_TRUE(board.value()->memory().write32(0x20000040U, 0U));
    ASSERT_TRUE(board.value()->memory().write32(0x20000044U, 1U));
    ASSERT_TRUE(board.value()->memory().write32(0xe0001000U, 1U));
    installCode(*board.value(), code);
    fil::sim::BoardRunOptions warm;
    warm.duration_ns = 0U;
    warm.enable_loop_batching = false;
    warm.enable_jit = true;
    warm.max_instructions = 13U;
    ASSERT_EQ(board.value()->run(warm).instructions, 13U);
    const auto entry = board.value()->cpu().state();
    const auto base = entry.r[15] - 2U;
    for (const auto offset : {2U, 6U, 8U, 10U, 12U, 16U, 18U, 22U, 24U, 26U, 28U, 30U}) {
        board.value()->cpu().state().r[15] = base + offset;
        for (unsigned i = 0U; i < 64U; ++i) {
            static_cast<void>(board.value()->cpu().prepareJitBlock(true));
        }
    }
    board.value()->cpu().state() = entry;
    const auto certificate = board.value()->prepareReversibleRamPrefix(36U);
    ASSERT_TRUE(certificate && certificate->period_certificate);
    const auto checkpoint = board.value()->memory().sideEffectCheckpoint();
    ASSERT_TRUE(board.value()->memory().restoreSideEffects(checkpoint));
    const auto after_rewind = board.value()->prepareReversibleRamPrefix(12U);
    ASSERT_TRUE(after_rewind);
    EXPECT_EQ(after_rewind->memoized_count, 0U);
    const auto transaction = board.value()->captureTransaction(fil::sim::shared_event_owner);
    ASSERT_TRUE(board.value()->restoreTransaction(transaction));
    const auto after_transaction_restore = board.value()->prepareReversibleRamPrefix(12U);
    ASSERT_TRUE(after_transaction_restore);
    EXPECT_EQ(after_transaction_restore->memoized_count, 0U);
    ASSERT_TRUE(board.value()->reset());
    EXPECT_THROW(static_cast<void>(board.value()->materializeReversibleRamPrefix(*certificate, 1U)),
                 std::logic_error);
}

TEST(DeferredPrefixTest, ReversibleRamEveryCutMatchesExactCpuMemoryAndTimers) {
    for (std::size_t count = 1U; count <= 6U; ++count) {
        auto exact = fil::sim::Board::load(fil::test::fixtureBoardConfig());
        auto speculative = fil::sim::Board::load(fil::test::fixtureBoardConfig());
        ASSERT_TRUE(exact && speculative);
        const std::vector<std::uint8_t> code{
            0x01U,0x20U, 0x08U,0x60U, 0x01U,0x30U,
            0x48U,0x60U, 0x01U,0x30U, 0x88U,0x60U, 0xfeU,0xe7U};
        constexpr std::uint32_t ram = 0x20000040U;
        for (auto* board : {exact.value().get(), speculative.value().get()}) {
            board->cpu().state().r[1] = ram;
            board->cpu().setNativeSingleInstructionJitEnabled(false);
            ASSERT_TRUE(board->memory().write32(0xe0001000U, 1U)); // DWT cycle counter
            installCode(*board, code);
        }
        const auto prefix = speculative.value()->prepareReversibleRamPrefix(6U);
        ASSERT_TRUE(prefix);
        ASSERT_EQ(prefix->count, 6U);
        EXPECT_FALSE(prefix->period_certificate) << "non-idempotent RAM writes cannot certify a period";
        EXPECT_EQ(speculative.value()->eventLoop().now(), prefix->start_time_ns);
        EXPECT_FALSE(speculative.value()->memory().mmioTrapping());
        EXPECT_FALSE(speculative.value()->memory().reversibleRamOnly());
        const auto committed = speculative.value()->materializeReversibleRamPrefix(*prefix, count);
        fil::sim::BoardRunOptions options;
        options.max_instructions = count;
        options.duration_ns = 0U;
        options.enable_loop_batching = false;
        const auto reference = exact.value()->run(options);
        ASSERT_EQ(reference.instructions, count);
        EXPECT_EQ(committed.cpu_result.instructions, count);
        EXPECT_EQ(committed.cpu_result.cycles, reference.cycles);
        EXPECT_EQ(committed.elapsed_ns, reference.time_ns);
        EXPECT_TRUE(fil::cpu::bitwiseEqual(speculative.value()->cpu().state(), exact.value()->cpu().state()));
        for (const auto offset : {0U, 4U, 8U}) {
            const auto expected = exact.value()->memory().read32(ram + offset);
            const auto actual = speculative.value()->memory().read32(ram + offset);
            ASSERT_TRUE(expected && actual);
            EXPECT_EQ(actual.value(), expected.value());
        }
        const auto counter = speculative.value()->memory().read32(0xe0001004U);
        ASSERT_TRUE(counter);
        EXPECT_EQ(counter.value(), reference.cycles) << "timers must be charged only at materialization";
    }
}

TEST(DeferredPrefixTest, ReversibleRamRejectsExecutableStoresWithoutEffects) {
    auto board = fil::sim::Board::load(fil::test::fixtureBoardConfig());
    ASSERT_TRUE(board);
    constexpr std::uint32_t ram = 0x21000000U;
    ASSERT_TRUE(board.value()->memory().mapRam(ram, 64U, "executable", true));
    board.value()->cpu().state().r[1] = ram;
    installCode(*board.value(), {0x01U,0x20U, 0x08U,0x60U, 0x00U,0xbfU, 0xfeU,0xe7U});
    const auto before = board.value()->cpu().state();
    EXPECT_FALSE(board.value()->prepareReversibleRamPrefix(4U));
    EXPECT_TRUE(fil::cpu::bitwiseEqual(board.value()->cpu().state(), before));
    const auto value = board.value()->memory().read32(ram);
    ASSERT_TRUE(value);
    EXPECT_EQ(value.value(), 0U);
}

TEST(DeferredPrefixTest, FailureTruncatesOtherCapsulesBeforeDraining) {
    for (const bool receiver_first : {false, true}) {
        for (const bool stop_on_failure : {false, true}) {
            DeferredNetwork files;
            const auto config = files.config(receiver_first);
            auto exact = fil::sim::World::load(config, true);
            auto candidate = fil::sim::World::load(config, true);
            ASSERT_TRUE(exact && candidate);
            for (auto* world : {exact.value().get(), candidate.value().get()}) {
                installCode(*world->board("sender"), {0x00U,0xdeU}); // UDF
                auto& receiver = *world->board("receiver");
                receiver.cpu().state().r[1] = 0x20000040U;
                installCode(receiver, {0x01U,0x20U, 0x08U,0x60U, 0x01U,0x30U,
                                       0x48U,0x60U, 0x01U,0x30U, 0x88U,0x60U,
                                       0x00U,0xbfU, 0x00U,0xbfU, 0xfeU,0xe7U});
            }
            fil::sim::WorldRunOptions options;
            options.duration_ns = 10000U;
            options.max_instructions_per_board = 1000U;
            options.enable_loop_batching = false;
            options.stop_on_board_failure = stop_on_failure;
            const auto expected = exact.value()->run(options);
            options.enable_jit = true;
            options.enable_ram_capsules = true;
            const auto actual = candidate.value()->run(options);
            ASSERT_TRUE(expected && actual);
            EXPECT_EQ(actual.value().reason, expected.value().reason);
            EXPECT_EQ(actual.value().end_time_ns, expected.value().end_time_ns);
            EXPECT_EQ(actual.value().instructions, expected.value().instructions);
            EXPECT_EQ(actual.value().cycles, expected.value().cycles);
            for (const auto name : {"sender", "receiver"}) {
                EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.value()->board(name)->cpu().state(),
                                                 exact.value()->board(name)->cpu().state()));
            }
            for (const auto offset : {0U, 4U, 8U}) {
                const auto expected_word = exact.value()->board("receiver")->memory().read32(0x20000040U + offset);
                const auto actual_word = candidate.value()->board("receiver")->memory().read32(0x20000040U + offset);
                ASSERT_TRUE(expected_word && actual_word);
                EXPECT_EQ(actual_word.value(), expected_word.value());
            }
            std::ostringstream expected_trace, actual_trace;
            exact.value()->trace().writeJsonLines(expected_trace);
            candidate.value()->trace().writeJsonLines(actual_trace);
            EXPECT_EQ(actual_trace.str(), expected_trace.str());
        }
    }
}

TEST(DeferredPrefixTest, NativeDisabledSingleStepPreservesDivideLatency) {
    for (const std::uint32_t divisor : {0U, 1U, 2U, 8U, 0x80U, 0xffffffffU}) {
        auto exact = fil::sim::Board::load(fil::test::fixtureBoardConfig());
        auto jit = fil::sim::Board::load(fil::test::fixtureBoardConfig());
        ASSERT_TRUE(exact);
        ASSERT_TRUE(jit);
        const std::vector<std::uint8_t> code{0x93U, 0xfbU, 0xf2U, 0xf1U}; // SDIV r1,r3,r2
        for (auto* board : {exact.value().get(), jit.value().get()}) {
            ASSERT_TRUE(board->memory().loadBytes(board->cpu().state().r[15], code));
            board->cpu().state().r[2] = divisor;
            board->cpu().state().r[3] = 1234567U;
            board->cpu().setNativeSingleInstructionJitEnabled(false);
        }
        const auto expected = exact.value()->cpu().stepFast();
        const auto actual = jit.value()->cpu().stepJitFast();
        EXPECT_EQ(actual.cycles, expected.cycles) << "divisor=" << divisor;
        EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.value()->cpu().state(), exact.value()->cpu().state()));
    }
}

TEST(DeferredPrefixTest, SynchronousCanAtHiddenCompletionPreservesBoardOrder) {
    for (const bool receiver_first : {false, true}) {
        DeferredNetwork files;
        const auto config = files.config(receiver_first);
        auto reference = fil::sim::World::load(config, true);
        auto candidate = fil::sim::World::load(config, true);
        ASSERT_TRUE(reference);
        ASSERT_TRUE(candidate);
        for (auto* world : {reference.value().get(), candidate.value().get()}) {
            auto& receiver = *world->board("receiver");
            auto& sender = *world->board("sender");
            configureReceiver(receiver);
            // Eight NOPs then a backward branch: hidden pure completions occur
            // at every cycle while a later sender performs a synchronous TX.
            installCode(receiver, {0x00U,0xbfU, 0x00U,0xbfU, 0x00U,0xbfU, 0x00U,0xbfU,
                                   0x00U,0xbfU, 0x00U,0xbfU, 0x00U,0xbfU, 0x00U,0xbfU,
                                   0xf6U,0xe7U});
            auto* tx = sender.peripherals().fdcan("FDCAN1");
            ASSERT_NE(tx, nullptr);
            ASSERT_TRUE(tx->write(fil::stm32g4::FdcanPeripheral::cccrOffset,
                                 fil::mem::AccessSize::word, 0U, {}));
            using Ram = fil::stm32g4::FdcanMessageRam;
            ASSERT_TRUE(sender.memory().write32(Ram::baseAddress + Ram::txFifoOffset, 0x123U << 18U));
            ASSERT_TRUE(sender.memory().write32(Ram::baseAddress + Ram::txFifoOffset + 4U, 0U));
            sender.cpu().state().r[0] = 1U;
            sender.cpu().state().r[1] = fil::stm32g4::FdcanPeripheral::baseAddresses[0]
                + fil::stm32g4::FdcanPeripheral::txbarOffset;
            // Three NOPs, STR r0,[r1], then an idle branch.
            installCode(sender, {0x00U,0xbfU, 0x00U,0xbfU, 0x00U,0xbfU, 0x08U,0x60U,
                                 0xfeU,0xe7U});
        }
        fil::sim::WorldRunOptions options;
        options.duration_ns = 10000U;
        options.max_instructions_per_board = 1000U;
        options.enable_loop_batching = false;
        const auto expected = reference.value()->run(options);
        options.enable_jit = true;
        options.enable_deferred_prefixes = true;
        const auto actual = candidate.value()->run(options);
        ASSERT_TRUE(expected);
        ASSERT_TRUE(actual);
        EXPECT_GT(actual.value().deferred_prefixes, 0U);
        EXPECT_GT(actual.value().deferred_truncations, 0U);
        EXPECT_GT(reference.value()->board("receiver")->cpu().state().r[6], 0U)
            << "the synchronous transmission must actually enter the receiver IRQ";
        EXPECT_EQ(actual.value().instructions, expected.value().instructions);
        EXPECT_EQ(actual.value().cycles, expected.value().cycles);
        EXPECT_EQ(actual.value().end_time_ns, expected.value().end_time_ns);
        for (const auto name : {"sender", "receiver"}) {
            EXPECT_TRUE(fil::cpu::bitwiseEqual(candidate.value()->board(name)->cpu().state(),
                                             reference.value()->board(name)->cpu().state()));
        }
        std::ostringstream expected_trace, actual_trace;
        reference.value()->trace().writeJsonLines(expected_trace);
        candidate.value()->trace().writeJsonLines(actual_trace);
        EXPECT_EQ(actual_trace.str(), expected_trace.str());
    }
}

} // namespace
