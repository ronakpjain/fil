#include "fil/cpu/cortex_m4.hpp"
#include "fil/cpu/instruction.hpp"
#include "fil/sim/board.hpp"
#include "fil/stm32g4/peripheral.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>

namespace {

constexpr fil::mem::AccessContext read_context{fil::mem::AccessType::data_read, 0};
constexpr fil::mem::AccessContext write_context{fil::mem::AccessType::data_write, 0};

fil::cpu::DecodedInstruction decodedAs(const fil::cpu::InstrKind kind) {
    fil::cpu::DecodedInstruction instruction;
    instruction.kind = kind;
    return instruction;
}

TEST(RealTimingTest, BasePipelineCyclesMatchDocumentedClasses) {
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::mov)), 1U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::add)), 1U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::nop)), 1U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::cmp)), 1U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::mul)), 1U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::mla)), 2U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::mls)), 2U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::umull)), 5U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::smull)), 5U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::ldr)), 2U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::strb)), 2U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::ldrd)), 3U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::strd)), 3U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::b)), 2U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::bl)), 2U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::cbz)), 1U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::mrs)), 2U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::svc)), 2U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::vdiv)), 14U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::vsqrt)), 14U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::vfma)), 3U);
    EXPECT_EQ(fil::cpu::basePipelineCycles(decodedAs(fil::cpu::InstrKind::vldr)), 2U);

    auto push_two = decodedAs(fil::cpu::InstrKind::push);
    push_two.register_list = 0x0003U;
    EXPECT_EQ(fil::cpu::basePipelineCycles(push_two), 3U)
        << "PUSH costs 1 + population count";

    auto ldm_full = decodedAs(fil::cpu::InstrKind::ldm);
    ldm_full.register_list = 0xffffU;
    EXPECT_EQ(fil::cpu::basePipelineCycles(ldm_full), 17U)
        << "16-register LDM costs 1 + 16";
}

TEST(RealTimingTest, DivideCyclesTerminateEarlyOnSmallDivisors) {
    EXPECT_EQ(fil::cpu::divideCycles(0U), 2U);
    EXPECT_EQ(fil::cpu::divideCycles(1U), 3U);
    EXPECT_EQ(fil::cpu::divideCycles(0x80000000U), 12U);
    EXPECT_TRUE(fil::cpu::divideCycles(0xffU) < fil::cpu::divideCycles(0xffffff00U))
        << "wider divisors cost more cycles";
}

TEST(RealTimingTest, TakenBranchPenaltyMatchesRefillModel) {
    EXPECT_EQ(fil::cpu::takenBranchPenalty(fil::cpu::InstrKind::b), 2U);
    EXPECT_EQ(fil::cpu::takenBranchPenalty(fil::cpu::InstrKind::bl), 2U);
    EXPECT_EQ(fil::cpu::takenBranchPenalty(fil::cpu::InstrKind::ldr), 2U);
    EXPECT_EQ(fil::cpu::takenBranchPenalty(fil::cpu::InstrKind::cbz), 1U);
    EXPECT_EQ(fil::cpu::takenBranchPenalty(fil::cpu::InstrKind::cbnz), 1U);
}

// RM0440 3.7.1: FLASH_ACR reset is 0x00040601 (LATENCY=1, caches enabled).
TEST(RealTimingTest, FlashResetEnablesCachesAndOneWaitState) {
    fil::stm32g4::FlashPeripheral flash;
    EXPECT_EQ(flash.waitStates(), 1U);
    EXPECT_FALSE(flash.prefetchEnabled());
    EXPECT_TRUE(flash.instructionCacheEnabled());
    EXPECT_EQ(flash.fetchStallCycles(0x08000000U, false), 1U);
}

TEST(RealTimingTest, FlashWaitStateTableMatchesDatasheet) {
    using fil::stm32g4::FlashPeripheral;
    EXPECT_EQ(FlashPeripheral::requiredWaitStates(16'000'000U), 0U);
    EXPECT_EQ(FlashPeripheral::requiredWaitStates(30'000'000U), 0U);
    EXPECT_EQ(FlashPeripheral::requiredWaitStates(60'000'000U), 1U);
    EXPECT_EQ(FlashPeripheral::requiredWaitStates(90'000'000U), 2U);
    EXPECT_EQ(FlashPeripheral::requiredWaitStates(120'000'000U), 3U);
    EXPECT_EQ(FlashPeripheral::requiredWaitStates(170'000'000U), 4U);
}

TEST(RealTimingTest, FlashAcrProgramsLatencyAndPrefetch) {
    fil::stm32g4::FlashPeripheral flash;
    const std::uint64_t generation = flash.acrGeneration();
    ASSERT_TRUE(flash.write(0x00U, fil::mem::AccessSize::word, 0x4U | (1U << 8U), write_context)
                    .hasValue());
    EXPECT_EQ(flash.waitStates(), 4U);
    EXPECT_TRUE(flash.prefetchEnabled());
    EXPECT_TRUE(flash.acrGeneration() != generation)
        << "ACR changes bump the loop-proof generation";

    // Taken-branch (non-sequential) flash fetch pays full LATENCY.
    EXPECT_EQ(flash.fetchStallCycles(0x08001000U, false), 4U);
    // Sequential fetch with prefetch enabled hits the ART buffer.
    EXPECT_EQ(flash.fetchStallCycles(0x08001002U, true), 0U);
    // SRAM execution never stalls.
    EXPECT_EQ(flash.fetchStallCycles(0x20000000U, false), 0U);
    // Boot alias inherits flash timing.
    EXPECT_EQ(flash.fetchStallCycles(0x00000100U, false), 4U);

    const auto acr = flash.read(0x00U, fil::mem::AccessSize::word, read_context);
    ASSERT_TRUE(acr.hasValue());
    EXPECT_EQ(acr.value() & 0x7U, 4U);
}

TEST(RealTimingTest, FlashStallWithoutArtHitsEveryFetch) {
    fil::stm32g4::FlashPeripheral flash;
    ASSERT_TRUE(flash.write(0x00U, fil::mem::AccessSize::word, 0x2U, write_context).hasValue());
    EXPECT_EQ(flash.fetchStallCycles(0x08000000U, true), 2U)
        << "sequential fetch without prefetch/cache still pays LATENCY";
}

TEST(RealTimingTest, SplitImageCostsFiveCyclesForThreeInstructions) {
    fil::config::BoardConfig config;
    config.name = "real-timing-spot";
    config.mcu_path = std::filesystem::path(FIL_SOURCE_DIR) / "configs/mcus/stm32g474retx.json";
    config.elf_path = std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";
    config.vector_base = 0x08000000U;
    auto loaded = fil::sim::Board::load(config, true);
    ASSERT_TRUE(loaded.hasValue());
    ASSERT_TRUE(loaded.value()->memory().write32(0x40022000U, 0U));
    fil::sim::BoardRunOptions options;
    options.max_instructions = 10U;
    options.duration_ns = 0U;
    const auto result = loaded.value()->run(options);
    EXPECT_EQ(result.instructions, 3U);
    // LDR(2) + LDR(2) + BKPT(1) at 16 MHz / 0 WS: no fetch stalls.
    EXPECT_EQ(result.cycles, 5U);
    EXPECT_GE(result.cycles, result.instructions);
    // 5 cycles at 16 MHz = 312.5 ns, truncated with fraction preserved.
    EXPECT_EQ(result.time_ns, 312U);
}

TEST(RealTimingTest, Pll170MHzProgramRequiresFourWaitStates) {
    fil::config::BoardConfig config;
    config.name = "real-timing-pll";
    config.mcu_path = std::filesystem::path(FIL_SOURCE_DIR) / "configs/mcus/stm32g474retx.json";
    config.elf_path = std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";
    config.vector_base = 0x08000000U;
    auto loaded = fil::sim::Board::load(config, true);
    ASSERT_TRUE(loaded.hasValue());
    auto& board = *loaded.value();
    // HSE 16 MHz, M=4, N=85, R=2 -> (16/4)*85/2 = 170 MHz.
    const std::uint32_t pll_config = 3U | (3U << 4U) | (85U << 8U) | (0U << 25U);
    const fil::mem::AccessContext context{fil::mem::AccessType::data_write, 0U};
    ASSERT_TRUE(board.peripherals().rcc().write(0x0cU, fil::mem::AccessSize::word, pll_config, context)
                    .hasValue());
    ASSERT_TRUE(board.peripherals().rcc().write(0x08U, fil::mem::AccessSize::word, 3U, context)
                    .hasValue());
    EXPECT_EQ(board.peripherals().rcc().systemClockHz(), 170'000'000U);
    EXPECT_EQ(fil::stm32g4::FlashPeripheral::requiredWaitStates(170'000'000U), 4U);
}

} // namespace
