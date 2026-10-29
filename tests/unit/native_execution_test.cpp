#include "fil/cpu/native_execution.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace {

TEST(NativeExecutionTest, ScalarHelpersUseFastRamPathsAndExplicitInstructionPc) {
    fil::mem::MemoryBus helper_bus;
    fil::mem::MemoryBus direct_bus;
    ASSERT_TRUE(helper_bus.mapRam(0x20000000U, 64U, "ram"));
    ASSERT_TRUE(direct_bus.mapRam(0x20000000U, 64U, "ram"));
    helper_bus.setReadFootprintTracking(true);
    direct_bus.setReadFootprintTracking(true);
    const auto context = [] (fil::mem::MemoryBus& bus) {
        return fil::cpu::NativeExecutionContext{nullptr, &bus, 8U, 100U};
    };
    auto native = context(helper_bus);
    constexpr std::uint32_t pc = 0x08001234U;
    constexpr std::uint32_t address = 0x20000004U;

    ASSERT_TRUE(fil::cpu::nativeWrite32(&native, pc, address, 0x12345678U));
    ASSERT_TRUE(direct_bus.tryFastWrite32(
        address, 0x12345678U, {fil::mem::AccessType::data_write, pc}));
    std::uint32_t value = 0;
    ASSERT_TRUE(fil::cpu::nativeRead32(&native, pc, address, &value));
    std::uint32_t direct_value = 0;
    ASSERT_TRUE(direct_bus.tryFastRead32(
        address, direct_value, {fil::mem::AccessType::data_read, pc}));
    EXPECT_EQ(value, 0x12345678U);
    EXPECT_EQ(value, direct_value);
    EXPECT_EQ(helper_bus.sideEffectCheckpoint().mutation_sequence,
              direct_bus.sideEffectCheckpoint().mutation_sequence);
    EXPECT_EQ(helper_bus.executionGeneration(), direct_bus.executionGeneration());
    EXPECT_EQ(helper_bus.readFootprint().words, direct_bus.readFootprint().words);
    EXPECT_EQ(helper_bus.readFootprint().complete, direct_bus.readFootprint().complete);
}

TEST(NativeExecutionTest, DeclinesFaultsAndMmioWithoutDispatchOrOutputMutation) {
    class Mmio final : public fil::mem::MmioDevice {
    public:
        fil::mem::MemoryResult<std::uint64_t> read(
            std::uint32_t, fil::mem::AccessSize, const fil::mem::AccessContext&) override {
            ++calls;
            return std::uint64_t{0};
        }
        fil::mem::MemoryResult<std::uint64_t> write(
            std::uint32_t, fil::mem::AccessSize, std::uint64_t,
            const fil::mem::AccessContext&) override {
            ++calls;
            return std::uint64_t{0};
        }
        std::string_view name() const noexcept override { return "test-mmio"; }
        int calls{0};
    } mmio;
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 4U, "ram"));
    ASSERT_TRUE(bus.mapMmio(0x40000000U, 4U, mmio, "mmio"));
    fil::cpu::NativeExecutionContext context{nullptr, &bus, 1U, 1U};
    std::uint32_t output = 0xaabbccddU;
    EXPECT_FALSE(fil::cpu::nativeRead32(&context, 0x08000000U, 0x40000000U, &output));
    EXPECT_EQ(output, 0xaabbccddU);
    EXPECT_EQ(mmio.calls, 0);
    EXPECT_FALSE(fil::cpu::nativeRead32(&context, 0x08000000U, 0x20000004U, &output));
    EXPECT_EQ(output, 0xaabbccddU);
    EXPECT_FALSE(fil::cpu::nativeRead32(nullptr, 0U, 0U, &output));
    EXPECT_FALSE(fil::cpu::nativeRead32(&context, 0U, 0U, nullptr));
}

TEST(NativeExecutionTest, StoreRequiresNonExecutableWritableRam) {
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 8U, "ram"));
    ASSERT_TRUE(bus.mapRam(0x20000100U, 8U, "code", true));
    ASSERT_TRUE(bus.mapRom(0x08000000U, 8U, "rom"));
    fil::cpu::CpuState cpu;
    cpu.r[0] = 0xfeedfaceU;
    cpu.xpsr = fil::cpu::xpsr_t | fil::cpu::xpsr_n | fil::cpu::xpsr_c;
    fil::cpu::NativeExecutionContext context{&cpu, &bus, 1U, 1U};
    EXPECT_TRUE(fil::cpu::nativeWrite32(&context, 0x08000000U, 0x20000000U, 1U));
    EXPECT_EQ(cpu.r[0], 0xfeedfaceU);
    EXPECT_EQ(cpu.xpsr, fil::cpu::xpsr_t | fil::cpu::xpsr_n | fil::cpu::xpsr_c);
    EXPECT_FALSE(fil::cpu::nativeWrite32(&context, 0x08000000U, 0x20000100U, 2U));
    EXPECT_FALSE(fil::cpu::nativeWrite32(&context, 0x08000000U, 0x08000000U, 3U));
    EXPECT_FALSE(fil::cpu::nativeWrite32(&context, 0x08000000U, 0x20000008U, 4U));
}

} // namespace
