#include "fil/cpu/native_execution.hpp"
#include "fil/cpu/native_stack.hpp"

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

TEST(NativeExecutionTest, CheckedStackTransferOrdersWordsAndKeepsDeclinesAtomic) {
    using namespace fil::cpu;
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 64U, "ram"));
    CpuState cpu; cpu.msp = 0x20000030U; cpu.r[0] = 0x11111111U; cpu.r[3] = 0x33333333U;
    cpu.xpsr = xpsr_t;
    NativeExecutionContext context{&cpu, &bus, 1U, 10U};
    EXPECT_EQ(nativeStackTransfer(&context, 0x08000000U, 0x9U, 1U, 0x08000002U), NativeStackResult::success);
    EXPECT_EQ(cpu.readRegister(13U), 0x20000028U);
    EXPECT_EQ(bus.read32(0x20000028U).value(), 0x11111111U);
    EXPECT_EQ(bus.read32(0x2000002cU).value(), 0x33333333U);
    cpu.r[0] = 0U; cpu.r[3] = 0U;
    EXPECT_EQ(nativeStackTransfer(&context, 0x08000002U, 0x9U, 0U, 0x08000004U), NativeStackResult::success);
    EXPECT_EQ(cpu.r[0], 0x11111111U); EXPECT_EQ(cpu.r[3], 0x33333333U);
    EXPECT_EQ(cpu.readRegister(13U), 0x20000030U);

    // A valid Thumb POP PC whose target differs from the guarded edge is a
    // branch mismatch, with CPU and read-footprint state fully rolled back.
    ASSERT_TRUE(bus.write32(0x20000030U, 0x08000101U));
    const auto before = bus.readFootprint();
    const CpuState snapshot = cpu;
    EXPECT_EQ(nativeStackTransfer(&context, 0x08000004U, 0x8000U, 0U, 0x08000200U), NativeStackResult::branch_mismatch);
    EXPECT_TRUE(bitwiseEqual(cpu, snapshot));
    EXPECT_EQ(bus.readFootprint().words, before.words);
    EXPECT_EQ(bus.readFootprint().complete, before.complete);
    ASSERT_TRUE(bus.write32(0x20000030U, 0xfffffff9U));
    EXPECT_EQ(nativeStackTransfer(&context, 0x08000004U, 0x8000U, 0U, 0xfffffff8U), NativeStackResult::memory_declined);
    EXPECT_TRUE(bitwiseEqual(cpu, snapshot));
}

TEST(NativeExecutionTest, StackMasksArchitecturalSpStoreFootprintsAndRollback) {
    using namespace fil::cpu;
    for (unsigned mode = 0U; mode < 3U; ++mode) {
        fil::mem::MemoryBus bus;
        constexpr std::uint32_t ram = 0x20000000U;
        ASSERT_TRUE(bus.mapRam(ram, 128U, "stack"));
        CpuState state;
        state.msp = ram + 64U; state.psp = ram + 96U;
        state.r[13] = ram + 32U;
        state.control = mode == 0U ? 0U : 2U;
        state.xpsr |= mode == 2U ? 3U : 0U;
        state.r[0] = 0x11223344U; state.r[2] = 0xaabbccddU;
        const auto start = state.readRegister(13U) - 8U;
        NativeExecutionContext context{&state, &bus, 1U, 100U};
        const auto before = bus.sideEffectCheckpoint();
        {
            fil::mem::MemoryBus::StoreFootprintScope capture(bus);
            ASSERT_EQ(nativeStackTransfer(&context, 0x08000000U, 5U, 1U, 0x08000002U),
                      NativeStackResult::success);
        }
        const auto footprint = bus.readFootprint();
        EXPECT_TRUE(footprint.complete);
        bool nonempty = false;
        for (const auto word : footprint.words) nonempty |= word != 0U;
        EXPECT_TRUE(nonempty);
        EXPECT_EQ(state.readRegister(13U), start);
        EXPECT_EQ(state.r[13], start);
        EXPECT_EQ(bus.read32(start).value(), 0x11223344U);
        EXPECT_EQ(bus.read32(start + 4U).value(), 0xaabbccddU);
        EXPECT_EQ(bus.sideEffectCheckpoint().mutation_sequence - before.mutation_sequence, 8U);
        ASSERT_TRUE(bus.restoreSideEffects(before));
        EXPECT_EQ(bus.read32(start).value(), 0U);
        EXPECT_EQ(bus.read32(start + 4U).value(), 0U);
        const auto saved = state;
        const auto restored = bus.sideEffectCheckpoint();
        const auto reads = bus.readFootprint();
        for (const auto malformed : {0x10000U, 0xffff0000U}) {
            EXPECT_EQ(nativeStackTransfer(&context, 0x08000000U, malformed, 1U, 0x08000002U),
                      NativeStackResult::memory_declined);
            EXPECT_TRUE(bitwiseEqual(state, saved));
            EXPECT_EQ(bus.sideEffectCheckpoint().mutation_sequence, restored.mutation_sequence);
            EXPECT_EQ(bus.readFootprint().words, reads.words);
        }
    }
}

TEST(NativeExecutionTest, StackPushDeclinesWholeExecutableOrMmioSpan) {
    using namespace fil::cpu;
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(0x20000000U, 8U, "ram"));
    ASSERT_TRUE(bus.mapRam(0x20000100U, 8U, "code", true));
    CpuState cpu; cpu.msp = 0x20000004U; cpu.r[0] = 1U; cpu.r[1] = 2U;
    NativeExecutionContext context{&cpu, &bus, 1U, 1U};
    const CpuState before = cpu;
    EXPECT_EQ(nativeStackTransfer(&context, 0x08000000U, 3U, 1U, 0x08000002U), NativeStackResult::memory_declined);
    EXPECT_TRUE(bitwiseEqual(cpu, before));
    EXPECT_EQ(bus.read32(0x20000000U).value(), 0U);
    cpu.msp = 0x20000108U;
    EXPECT_EQ(nativeStackTransfer(&context, 0x08000000U, 3U, 1U, 0x08000002U), NativeStackResult::memory_declined);
    EXPECT_EQ(cpu.readRegister(13U), 0x20000108U);
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
