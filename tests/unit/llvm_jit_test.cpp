#include "fil/cpu/cortex_m4.hpp"
#include "fil/cpu/decoder.hpp"
#include "fil/cpu/llvm_jit.hpp"
#include "fil/mem/memory_bus.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

using fil::cpu::NativeJitInstruction;
using fil::cpu::NativeJitKernel;
using fil::cpu::LlvmJit;

NativeJitInstruction narrow(const std::uint16_t raw, const std::uint32_t pc) {
    const auto decoded = fil::cpu::decode16(raw);
    EXPECT_TRUE(decoded.has_value()) << std::hex << raw;
    if (!decoded) return {};
    return NativeJitInstruction{*decoded, pc, 2U};
}

NativeJitInstruction wide(const std::uint16_t first, const std::uint16_t second,
                          const std::uint32_t pc) {
    const auto decoded = fil::cpu::decode32(first, second);
    EXPECT_TRUE(decoded.has_value()) << std::hex << first << ' ' << second;
    if (!decoded) return {};
    return NativeJitInstruction{*decoded, pc, 4U};
}

std::unique_ptr<LlvmJit> compiler() {
    std::string error;
    auto jit = LlvmJit::create(error);
    EXPECT_NE(jit, nullptr) << error;
    return jit;
}

std::shared_ptr<const NativeJitKernel> compile(
    LlvmJit& jit, const std::vector<NativeJitInstruction>& instructions) {
    std::string error;
    auto kernel = jit.compile(instructions, error);
    EXPECT_NE(kernel, nullptr) << error;
    return kernel;
}

} // namespace

TEST(LlvmJitAvailabilityTest, CortexAvailabilityMatchesBuildMacro) {
#if FIL_HAS_LLVM_JIT
    EXPECT_TRUE(fil::cpu::CortexM4::nativeJitAvailable());
#else
    EXPECT_FALSE(fil::cpu::CortexM4::nativeJitAvailable());
#endif
}

#if FIL_HAS_LLVM_JIT
namespace {
constexpr std::uint32_t entry = 0x08000000U;

std::vector<NativeJitInstruction> arithmeticProgram() {
    return {narrow(0x2001U, entry),       // MOVS r0,#1
            narrow(0x3001U, entry + 2U),   // ADDS r0,#1
            narrow(0x3801U, entry + 4U),   // SUBS r0,#1
            narrow(0x2801U, entry + 6U),   // CMP r0,#1
            narrow(0xBF00U, entry + 8U),    // NOP
            narrow(0xE7FFU, entry + 10U)}; // B to its fallthrough
}

bool equalState(const fil::cpu::CpuState& lhs, const fil::cpu::CpuState& rhs) {
    return fil::cpu::bitwiseEqual(lhs, rhs);
}
} // namespace

TEST(LlvmJitTest, SupportsOnlyConservativeDecodedInstructionForms) {
    const std::array supported{
        narrow(0xBF00U, entry), narrow(0x2001U, entry + 2U),
        narrow(0x3001U, entry + 4U), narrow(0x3801U, entry + 6U),
        narrow(0x2801U, entry + 8U),
        wide(0xF110U, 0x0F01U, entry + 10U), // CMN r0,#1
        wide(0xF240U, 0x0001U, entry + 14U), // MOVW r0,#1
        wide(0xF2C0U, 0x0001U, entry + 18U), // MOVT r0,#1
    };
    for (const auto& instruction : supported) {
        EXPECT_TRUE(LlvmJit::supports(instruction))
            << "kind=" << static_cast<std::uint32_t>(instruction.decoded.kind)
            << " raw=" << std::hex << instruction.decoded.raw;
    }

    auto conditional = narrow(0xD100U, entry);
    auto memory = narrow(0x6808U, entry);
    auto it = narrow(0xBF08U, entry);
    auto bad_reg = narrow(0x4687U, entry); // MOV involving PC
    EXPECT_FALSE(LlvmJit::supports(conditional));
    EXPECT_FALSE(LlvmJit::supports(memory));
    EXPECT_FALSE(LlvmJit::supports(it));
    EXPECT_FALSE(LlvmJit::supports(bad_reg));
}

TEST(LlvmJitTest, DifferentialArithmeticPrefixesPreserveUnrelatedState) {
    auto jit = compiler();
    ASSERT_NE(jit, nullptr);
    const auto program = arithmeticProgram();
    for (const auto& instruction : program) ASSERT_TRUE(LlvmJit::supports(instruction));
    const auto kernel = compile(*jit, program);
    ASSERT_NE(kernel, nullptr);

    constexpr std::array<std::uint32_t, 5> edges{
        0U, 1U, 0x7fffffffU, 0x80000000U, 0xffffffffU};
    for (std::size_t trial = 0; trial < 48U; ++trial) {
        fil::cpu::CpuState initial{};
        initial.r[15] = entry;
        initial.instruction_address = entry;
        initial.xpsr = fil::cpu::xpsr_t
            | (((trial & 1U) != 0U) ? fil::cpu::xpsr_c : 0U)
            | (((trial & 2U) != 0U) ? fil::cpu::xpsr_v : 0U);
        initial.fpscr = 0xa5c30000U ^ static_cast<std::uint32_t>(trial);
        for (std::size_t i = 0; i < initial.s.size(); ++i) {
            initial.s[i] = std::bit_cast<float>(0x7fc00000U
                | (static_cast<std::uint32_t>(i) << 8U)
                | static_cast<std::uint32_t>(trial));
        }
        for (std::size_t i = 0; i < 13U; ++i)
            initial.r[i] = static_cast<std::uint32_t>(trial * 0x9e3779b9U + i * 17U);
        initial.r[0] = edges[trial % edges.size()];
        initial.msp = 0x20000100U;
        initial.psp = 0x20000200U;
        initial.control = 2U;
        initial.primask = 3U;

        for (std::size_t limit = 0; limit <= program.size(); ++limit) {
            auto actual = initial;
            kernel->execute(actual, limit);
            fil::mem::MemoryBus bus;
            ASSERT_TRUE(bus.mapRam(entry, 256U, "llvm-jit-oracle", true).hasValue());
            std::vector<std::uint8_t> bytes;
            for (const auto& instruction : program) {
                const std::uint16_t first = instruction.size == 4U
                    ? static_cast<std::uint16_t>(instruction.decoded.raw >> 16U)
                    : static_cast<std::uint16_t>(instruction.decoded.raw);
                bytes.push_back(static_cast<std::uint8_t>(first));
                bytes.push_back(static_cast<std::uint8_t>(first >> 8U));
                if (instruction.size == 4U) {
                    const auto second = static_cast<std::uint16_t>(instruction.decoded.raw);
                    bytes.push_back(static_cast<std::uint8_t>(second));
                    bytes.push_back(static_cast<std::uint8_t>(second >> 8U));
                }
            }
            ASSERT_TRUE(bus.loadBytes(entry, bytes).hasValue());
            fil::cpu::CortexM4 reference(bus);
            reference.state() = initial;
            for (std::size_t i = 0; i < limit; ++i) {
                ASSERT_EQ(reference.stepFast().reason, fil::cpu::StopReason::step_complete);
            }
            EXPECT_TRUE(equalState(actual, reference.state()))
                << "trial=" << trial << " limit=" << limit;
        }
    }
}

TEST(LlvmJitTest, BranchPrefixesMatchInterpreterForFallthroughAndTakenTargets) {
    auto jit = compiler();
    ASSERT_NE(jit, nullptr);
    const std::array programs{
        std::vector{narrow(0xBF00U, entry), narrow(0xE7FFU, entry + 2U)}, // B target=fallthrough
        std::vector{narrow(0xBF00U, entry), narrow(0xE7FDU, entry + 2U)}, // B to entry
        std::vector{narrow(0xBF00U, entry), wide(0xF000U, 0xB800U, entry + 2U)}, // B.W offset 0
        std::vector{narrow(0xBF00U, entry), wide(0xF000U, 0xF800U, entry + 2U)}, // BL offset 0
    };
    for (const auto& program : programs) {
        for (const auto& instruction : program) ASSERT_TRUE(LlvmJit::supports(instruction));
        const auto kernel = compile(*jit, program);
        ASSERT_NE(kernel, nullptr);
        fil::mem::MemoryBus bus;
        ASSERT_TRUE(bus.mapRam(entry, 256U, "llvm-branch-oracle", true).hasValue());
        std::vector<std::uint8_t> bytes;
        for (const auto& instruction : program) {
            const auto first = instruction.size == 4U
                ? static_cast<std::uint16_t>(instruction.decoded.raw >> 16U)
                : static_cast<std::uint16_t>(instruction.decoded.raw);
            bytes.push_back(static_cast<std::uint8_t>(first));
            bytes.push_back(static_cast<std::uint8_t>(first >> 8U));
            if (instruction.size == 4U) {
                const auto second = static_cast<std::uint16_t>(instruction.decoded.raw);
                bytes.push_back(static_cast<std::uint8_t>(second));
                bytes.push_back(static_cast<std::uint8_t>(second >> 8U));
            }
        }
        ASSERT_TRUE(bus.loadBytes(entry, bytes).hasValue());
        for (const std::size_t limit : std::array<std::size_t, 3>{0U, 1U, program.size()}) {
            fil::cpu::CpuState initial{};
            initial.r[15] = entry;
            initial.instruction_address = entry;
            initial.r[14] = 0x12345678U;
            initial.xpsr = fil::cpu::xpsr_t | fil::cpu::xpsr_c;
            auto actual = initial;
            kernel->execute(actual, limit);
            fil::cpu::CortexM4 reference(bus);
            reference.state() = initial;
            for (std::size_t i = 0; i < limit; ++i) {
                ASSERT_EQ(reference.stepFast().reason, fil::cpu::StopReason::step_complete);
            }
            EXPECT_TRUE(equalState(actual, reference.state()))
                << "branch raw=" << std::hex << program.back().decoded.raw << " limit=" << limit;
        }
    }
}

TEST(LlvmJitTest, RejectsMalformedBlocksAndKeepsKernelAliveAfterCompilerDestruction) {
    std::string error;
    auto jit = LlvmJit::create(error);
    ASSERT_NE(jit, nullptr) << error;
    const auto valid = narrow(0xBF00U, entry);
    auto gap = narrow(0x2001U, entry + 4U);
    std::array noncontiguous{valid, gap};
    EXPECT_EQ(jit->compile(noncontiguous, error), nullptr);
    auto branch = narrow(0xE000U, entry + 2U);
    std::array branch_not_final{branch, narrow(0xBF00U, entry + 4U)};
    EXPECT_EQ(jit->compile(branch_not_final, error), nullptr);
    auto conditional = narrow(0xD100U, entry + 2U);
    std::array conditional_block{valid, conditional};
    EXPECT_EQ(jit->compile(conditional_block, error), nullptr);
    auto memory = narrow(0x6808U, entry + 2U);
    std::array memory_block{valid, memory};
    EXPECT_EQ(jit->compile(memory_block, error), nullptr);

    auto kernel = compile(*jit, std::vector{valid, narrow(0x2001U, entry + 2U)});
    ASSERT_NE(kernel, nullptr);
    jit.reset();
    fil::cpu::CpuState state{};
    state.r[15] = entry;
    kernel->execute(state, 2U);
    EXPECT_EQ(state.r[0], 1U);
    for (std::int32_t guard = 0; guard < 5; ++guard) {
        fil::cpu::CpuState guarded{};
        guarded.r[15] = entry;
        if (guard == 0) guarded.setItState(0x08U);
        if (guard == 1) guarded.pending_exception = 3U;
        if (guard == 2) guarded.pending_exc_return = 0xfffffff9U;
        if (guard == 3) guarded.thumb = false;
        if (guard == 4) guarded.halted = true;
        const auto before = guarded;
        kernel->execute(guarded, 2U);
        EXPECT_TRUE(equalState(guarded, before)) << "guard=" << guard;
    }
}

TEST(LlvmJitIntegrationTest, CompiledBranchFaultOnNextDispatchMatchesInterpreter) {
    constexpr std::uint32_t ram = 0x20000000U;
    fil::mem::MemoryBus native_bus;
    fil::mem::MemoryBus reference_bus;
    ASSERT_TRUE(native_bus.mapRam(ram, 256U, "llvm-branch-fault-native", true).hasValue());
    ASSERT_TRUE(reference_bus.mapRam(ram, 256U, "llvm-branch-fault-reference", true).hasValue());
    const std::array<std::uint8_t, 4> code{0x00U, 0xBFU, 0x7dU, 0xE0U}; // NOP; B outside RAM
    ASSERT_TRUE(native_bus.loadBytes(ram, code).hasValue());
    ASSERT_TRUE(reference_bus.loadBytes(ram, code).hasValue());
    fil::cpu::CortexM4 native(native_bus);
    fil::cpu::CortexM4 reference(reference_bus);
    fil::cpu::CpuState initial{};
    initial.r[15] = ram;
    initial.instruction_address = ram;
    initial.msp = ram + 0xf0U;
    initial.r[13] = initial.msp;
    native.state() = initial;
    reference.state() = initial;
    std::optional<fil::cpu::CortexM4::JitStepOutcome> branch;
    for (std::int32_t i = 0; i < 1024 && native.jitStats().native_executions == 0U; ++i) {
        native.state() = initial;
        branch = native.tryStepJitBlock(2U);
    }
    ASSERT_GT(native.jitStats().native_executions, 0U) << native.nativeJitError()
        << " compilations=" << native.jitStats().native_compilations
        << " failures=" << native.jitStats().native_compilation_failures
        << " block compilations=" << native.jitStats().compilations;
    ASSERT_TRUE(branch.has_value());
    ASSERT_EQ(reference.stepFast().reason, fil::cpu::StopReason::step_complete);
    ASSERT_EQ(reference.stepFast().reason, fil::cpu::StopReason::step_complete);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(native.state(), reference.state()));
    const auto expected = reference.stepFast();
    const auto actual = native.stepJitFast();
    EXPECT_EQ(actual.reason, expected.reason);
    EXPECT_EQ(actual.instruction_address, expected.instruction_address);
    EXPECT_EQ(actual.raw, expected.raw);
    EXPECT_EQ(actual.instruction_size, expected.instruction_size);
    EXPECT_EQ(actual.cycles, expected.cycles);
    EXPECT_EQ(native.lastDiagnostic().instruction_address,
              reference.lastDiagnostic().instruction_address);
    EXPECT_EQ(native.lastDiagnostic().raw, reference.lastDiagnostic().raw);
    EXPECT_EQ(native.lastDiagnostic().message, reference.lastDiagnostic().message);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(native.state(), reference.state()));
}

TEST(LlvmJitIntegrationTest, CompiledBranchReportsExactCyclesAndFallthroughState) {
    constexpr std::uint32_t ram = 0x20000000U;
    fil::mem::MemoryBus native_bus;
    fil::mem::MemoryBus reference_bus;
    ASSERT_TRUE(native_bus.mapRam(ram, 256U, "llvm-branch-native", true).hasValue());
    ASSERT_TRUE(reference_bus.mapRam(ram, 256U, "llvm-branch-reference", true).hasValue());
    const std::array<std::uint8_t, 8> code{
        0x00U, 0xBFU, 0x00U, 0xF0U, 0x00U, 0xF8U, 0x00U, 0xBEU}; // NOP; BL +0; BKPT
    ASSERT_TRUE(native_bus.loadBytes(ram, code).hasValue());
    ASSERT_TRUE(reference_bus.loadBytes(ram, code).hasValue());
    fil::cpu::CortexM4 native(native_bus);
    fil::cpu::CortexM4 reference(reference_bus);
    fil::cpu::CpuState initial{};
    initial.r[15] = ram;
    initial.instruction_address = ram;
    initial.r[14] = 0xfeedfaceU;
    initial.msp = ram + 0xf0U;
    initial.r[13] = initial.msp;
    native.state() = initial;
    reference.state() = initial;
    std::uint16_t cycles = 0U;
    for (std::int32_t i = 0; i < 2; ++i) {
        const auto step = reference.stepFast();
        ASSERT_EQ(step.reason, fil::cpu::StopReason::step_complete);
        cycles = static_cast<std::uint16_t>(cycles + step.cycles);
    }
    std::optional<fil::cpu::CortexM4::JitStepOutcome> outcome;
    for (std::int32_t i = 0; i < 1024 && native.jitStats().native_executions == 0U; ++i) {
        native.state() = initial;
        outcome = native.tryStepJitBlock(2U);
    }
    ASSERT_GT(native.jitStats().native_compilations, 0U) << native.nativeJitError();
    ASSERT_TRUE(outcome.has_value());
    EXPECT_EQ(outcome->count, 2U);
    EXPECT_EQ(outcome->result.cycles, cycles);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(native.state(), reference.state()));
    EXPECT_EQ(native.state().r[14], ram + 7U);
    EXPECT_EQ(native.state().r[15], ram + 6U);
}

TEST(LlvmJitIntegrationTest, SingleInstructionNativeHotPathInvalidatesAfterExecutableWrite) {
    constexpr std::uint32_t ram = 0x20000000U;
    fil::mem::MemoryBus jit_bus;
    fil::mem::MemoryBus reference_bus;
    ASSERT_TRUE(jit_bus.mapRam(ram, 256U, "llvm-single-native", true).hasValue());
    ASSERT_TRUE(reference_bus.mapRam(ram, 256U, "llvm-single-reference", true).hasValue());
    const std::array<std::uint8_t, 2> original{0x01U, 0x20U}; // MOVS r0,#1
    const std::array<std::uint8_t, 2> replacement{0x09U, 0x20U}; // MOVS r0,#9
    ASSERT_TRUE(jit_bus.loadBytes(ram, original).hasValue());
    ASSERT_TRUE(reference_bus.loadBytes(ram, original).hasValue());
    fil::cpu::CortexM4 jit(jit_bus);
    fil::cpu::CortexM4 reference(reference_bus);
    fil::cpu::CpuState initial{};
    initial.r[15] = ram;
    initial.instruction_address = ram;
    initial.msp = ram + 0xf0U;
    initial.r[13] = initial.msp;
    std::optional<fil::cpu::FastStepResult> last;
    for (std::int32_t i = 0; i < 20000 && jit.jitStats().native_executions == 0U; ++i) {
        jit.state() = initial;
        last = jit.stepJitFast();
    }
    ASSERT_TRUE(last.has_value());
    ASSERT_GT(jit.jitStats().native_compilations, 0U) << jit.nativeJitError();
    ASSERT_GT(jit.jitStats().native_executions, 0U) << jit.nativeJitError();
    EXPECT_EQ(jit.state().r[0], 1U);

    ASSERT_TRUE(jit_bus.loadBytes(ram, replacement).hasValue());
    ASSERT_TRUE(reference_bus.loadBytes(ram, replacement).hasValue());
    const auto executions_before = jit.jitStats().native_executions;
    const auto compilations_before = jit.jitStats().native_compilations;
    jit.state() = initial;
    reference.state() = initial;
    const auto actual = jit.stepJitFast();
    const auto expected = reference.stepFast();
    EXPECT_EQ(actual.reason, expected.reason);
    EXPECT_EQ(actual.raw, expected.raw);
    EXPECT_EQ(actual.instruction_size, expected.instruction_size);
    EXPECT_EQ(actual.cycles, expected.cycles);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(jit.state(), reference.state()));
    for (std::int32_t i = 0; i < 20000 && jit.jitStats().native_executions == executions_before; ++i) {
        jit.state() = initial;
        static_cast<void>(jit.stepJitFast());
    }
    EXPECT_GT(jit.jitStats().native_compilations, compilations_before)
        << "executable RAM generation change must discard stale native code";
    EXPECT_GT(jit.jitStats().native_executions, executions_before);
    EXPECT_EQ(jit.state().r[0], 9U);
}

TEST(LlvmJitIntegrationTest, HotCappedNativeBlockMatchesInterpreterAndTracksNativeExecution) {
    constexpr std::uint32_t ram = 0x20000000U;
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(ram, 256U, "llvm-jit-test", true).hasValue());
    const std::array<std::uint8_t, 10> code{0x01U, 0x20U, 0x01U, 0x30U,
                                            0x00U, 0xBFU, 0x00U, 0xBFU,
                                            0x00U, 0xBEU}; // BKPT ends lookahead
    ASSERT_TRUE(bus.loadBytes(ram, code).hasValue());
    fil::cpu::CortexM4 cpu(bus);
    fil::cpu::CpuState initial{};
    initial.r[15] = ram;
    initial.instruction_address = ram;
    initial.msp = ram + 0xf0U;
    initial.r[13] = initial.msp;
    auto reference_bus = fil::mem::MemoryBus{};
    ASSERT_TRUE(reference_bus.mapRam(ram, 256U, "llvm-jit-reference", true).hasValue());
    ASSERT_TRUE(reference_bus.loadBytes(ram, code).hasValue());
    fil::cpu::CortexM4 reference(reference_bus);
    reference.state() = initial;
    std::uint16_t reference_cycles = 0U;
    for (std::int32_t i = 0; i < 2; ++i) {
        const auto step = reference.stepFast();
        ASSERT_EQ(step.reason, fil::cpu::StopReason::step_complete);
        reference_cycles = static_cast<std::uint16_t>(reference_cycles + step.cycles);
    }

    std::optional<fil::cpu::CortexM4::JitStepOutcome> result;
    for (std::int32_t i = 0; i < 1024; ++i) {
        cpu.state() = initial;
        result = cpu.tryStepJitBlock(2U);
        if (cpu.jitStats().native_executions != 0U) break;
    }
    ASSERT_GT(cpu.jitStats().native_compilations, 0U) << cpu.nativeJitError();
    ASSERT_GT(cpu.jitStats().native_executions, 0U) << cpu.nativeJitError();
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->count, 2U);
    EXPECT_GT(cpu.jitStats().native_instructions, 0U);
    EXPECT_EQ(cpu.jitStats().native_instructions, 2U);
    EXPECT_EQ(result->result.cycles, reference_cycles);
    EXPECT_EQ(cpu.state().r[0], 2U);
    EXPECT_EQ(cpu.state().r[15], ram + 4U);
    EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));

    // A ready native block must still obey the CPU's architectural entry guards.
    for (std::int32_t guard = 0; guard < 4; ++guard) {
        cpu.state() = initial;
        if (guard == 0) cpu.state().setItState(0x08U);
        if (guard == 1) cpu.state().pending_exception = 3U;
        if (guard == 2) cpu.state().thumb = false;
        if (guard == 3) cpu.state().halted = true;
        EXPECT_FALSE(cpu.tryStepJitBlock(1U).has_value()) << "guard=" << guard;
    }
}
TEST(LlvmJitIntegrationTest, MixedBlockNativePrefixPreservesSuffixAndLimits) {
    constexpr std::uint32_t ram = 0x20000000U;
    for (const bool unsupported_first : {false, true}) {
        fil::mem::MemoryBus bus;
        ASSERT_TRUE(bus.mapRam(ram, 256U, "mixed-native", true).hasValue());
        // Four supported ops; register ADDS (not lowered); NOP; BKPT.
        std::array<std::uint8_t, 14> code{0x01, 0x20, 0x01, 0x30,
            0x00, 0xbf, 0x00, 0xbf, 0x40, 0x18, 0x00, 0xbf, 0x00, 0xbe};
        if (unsupported_first) {
            code[0] = 0x40; code[1] = 0x18;
        }
        ASSERT_TRUE(bus.loadBytes(ram, code).hasValue());
        fil::cpu::CortexM4 cpu(bus), reference(bus);
        fil::cpu::CpuState initial{};
        initial.r[15] = ram;
        initial.r[1] = 7U;
        for (std::int32_t warm = 0; warm < 1024; ++warm) {
            cpu.state() = initial;
            static_cast<void>(cpu.tryStepJitBlock(6U));
        }
        EXPECT_EQ(cpu.jitStats().native_compilations, unsupported_first ? 0U : 1U);
        for (std::size_t limit = 1; limit <= 6; ++limit) {
            cpu.state() = reference.state() = initial;
            std::uint16_t cycles = 0;
            for (std::size_t i = 0; i < limit; ++i) {
                const auto step = reference.stepFast();
                ASSERT_EQ(step.reason, fil::cpu::StopReason::step_complete);
                cycles = static_cast<std::uint16_t>(cycles + step.cycles);
            }
            const auto before = cpu.jitStats();
            const auto result = cpu.tryStepJitBlock(limit);
            ASSERT_TRUE(result);
            EXPECT_EQ(result->count, limit);
            EXPECT_EQ(result->result.instructions, limit);
            EXPECT_EQ(result->result.cycles, cycles);
            EXPECT_EQ(result->result.instruction_address, reference.state().instruction_address);
            EXPECT_TRUE(fil::cpu::bitwiseEqual(cpu.state(), reference.state()));
            EXPECT_EQ(result->result.instruction_size, 2U);
            EXPECT_EQ(result->result.raw, static_cast<std::uint32_t>(code[2 * (limit - 1)])
                | (static_cast<std::uint32_t>(code[2 * (limit - 1) + 1]) << 8U));
            for (std::size_t i = 0; i < limit; ++i) {
                EXPECT_EQ(result->pcs[i], ram + 2U * i);
                EXPECT_EQ(result->sizes[i], 2U);
            }
            EXPECT_EQ(cpu.jitStats().native_instructions - before.native_instructions,
                unsupported_first ? 0U : std::min<std::size_t>(limit, 4U));
            EXPECT_EQ(cpu.jitStats().block_instructions - before.block_instructions, limit);
        }
    }
}
TEST(LlvmJitIntegrationTest, NativeBlockAdmissionRequiresActualMultiInstructionUse) {
    constexpr std::uint32_t ram = 0x20000000U;
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(ram, 256U, "native-admission", true).hasValue());
    const std::array<std::uint8_t, 10> code{1, 0x20, 1, 0x30,
        0, 0xbf, 0, 0xbf, 0, 0xbe};
    ASSERT_TRUE(bus.loadBytes(ram, code).hasValue());
    fil::cpu::CortexM4 cpu(bus);
    fil::cpu::CpuState initial{};
    initial.r[15] = ram;
    cpu.state() = initial;
    for (std::int32_t i = 0; i < 2000; ++i) {
        static_cast<void>(cpu.prepareJitBlock());
        static_cast<void>(cpu.peekJitBlock(4U));
        cpu.state() = initial;
        static_cast<void>(cpu.tryStepJitBlock(1U));
    }
    EXPECT_EQ(cpu.jitStats().native_compilations, 0U);
    for (std::int32_t i = 0; i < 512; ++i) {
        cpu.state() = initial;
        ASSERT_TRUE(cpu.tryStepJitBlock(4U));
    }
    EXPECT_EQ(cpu.jitStats().native_compilations, 0U);
    cpu.state() = initial;
    ASSERT_TRUE(cpu.tryStepJitBlock(4U));
    EXPECT_EQ(cpu.jitStats().native_compilations, 1U);
    EXPECT_EQ(cpu.jitStats().native_instructions, 4U);
}

TEST(LlvmJitIntegrationTest, EvictedNativeBlockRewarmsWithoutStaleEntryPoint) {
    constexpr std::uint32_t ram = 0x20000000U;
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(ram, 1024U, "mixed-lru", true).hasValue());
    std::array<std::uint8_t, 164> code{};
    const std::array<std::uint8_t, 10> block{1, 0x20, 1, 0x30,
        0, 0xbf, 0, 0xbf, 0, 0xbe};
    std::copy(block.begin(), block.end(), code.begin());
    for (std::size_t i = 0; i < 33; ++i) {
        code[32 + i * 4] = static_cast<std::uint8_t>(i + 1);
        code[33 + i * 4] = 0x20;
        code[34 + i * 4] = 0; code[35 + i * 4] = 0xbe;
    }
    ASSERT_TRUE(bus.loadBytes(ram, code).hasValue());
    fil::cpu::CortexM4 cpu(bus);
    const auto step_single = [&](std::size_t site) {
        cpu.state() = fil::cpu::CpuState{};
        cpu.state().r[15] = ram + 32U + static_cast<std::uint32_t>(site * 4U);
        EXPECT_EQ(cpu.stepJitFast().reason, fil::cpu::StopReason::step_complete);
        EXPECT_EQ(cpu.state().r[0], site + 1U);
    };
    const auto step_block = [&] {
        cpu.state() = fil::cpu::CpuState{};
        cpu.state().r[15] = ram;
        const auto result = cpu.tryStepJitBlock(4U);
        if (result) {
            EXPECT_EQ(result->count, 4U);
            EXPECT_EQ(cpu.state().r[0], 2U);
        }
    };
    for (std::int32_t i = 0; i < 600; ++i) step_block();
    ASSERT_EQ(cpu.jitStats().native_compilations, 1U);
    for (std::size_t site = 0; site < 31; ++site) {
        const auto before = cpu.jitStats().native_compilations;
        for (std::int32_t i = 0; i < 20000 && cpu.jitStats().native_compilations == before; ++i) {
            step_single(site);
            if (i % 100 == 0) step_block(); // Block touch must affect LRU.
        }
        ASSERT_EQ(cpu.jitStats().native_compilations, before + 1U);
    }
    ASSERT_EQ(cpu.jitStats().native_evictions, 0U);
    auto before = cpu.jitStats();
    step_block();
    EXPECT_EQ(cpu.jitStats().native_instructions, before.native_instructions + 4U);
    // Keep every single resident recent, but stop touching the block.
    for (std::int32_t i = 0; i < 20000 && cpu.jitStats().native_compilations == before.native_compilations; ++i) {
        step_single(31);
        if (i % 100 == 0) for (std::size_t site = 0; site < 31; ++site) step_single(site);
    }
    ASSERT_EQ(cpu.jitStats().native_evictions, 1U);
    before = cpu.jitStats();
    step_block();
    EXPECT_EQ(cpu.jitStats().native_instructions, before.native_instructions);
    for (std::int32_t i = 0; i < 20000 && cpu.jitStats().native_compilations == before.native_compilations; ++i) step_block();
    EXPECT_EQ(cpu.jitStats().native_compilations, before.native_compilations + 1U);
    EXPECT_EQ(cpu.jitStats().native_evictions, 2U);
    before = cpu.jitStats();
    step_block();
    EXPECT_EQ(cpu.jitStats().native_instructions, before.native_instructions + 4U);
}

TEST(LlvmJitIntegrationTest, LruRetainsRecentKernelAndEvictedSitesRewarm) {
    constexpr std::uint32_t ram = 0x20000000U;
    fil::mem::MemoryBus bus;
    ASSERT_TRUE(bus.mapRam(ram, 512U, "lru-native", true).hasValue());
    std::array<std::uint8_t, 132> code{};
    for (std::size_t i = 0; i < 33; ++i) {
        code[i * 4] = static_cast<std::uint8_t>(i + 1);
        code[i * 4 + 1] = 0x20; // MOVS r0,#(i+1)
        code[i * 4 + 2] = 0x00; code[i * 4 + 3] = 0xbe;
    }
    ASSERT_TRUE(bus.loadBytes(ram, code).hasValue());
    fil::cpu::CortexM4 cpu(bus);
    const auto step_site = [&](std::size_t site) {
        cpu.state() = fil::cpu::CpuState{};
        cpu.state().r[15] = ram + static_cast<std::uint32_t>(site * 4);
        const auto result = cpu.stepJitFast();
        EXPECT_EQ(result.reason, fil::cpu::StopReason::step_complete);
        EXPECT_EQ(cpu.state().r[0], site + 1U);
    };
    for (std::size_t site = 0; site < 33; ++site) {
        const auto before = cpu.jitStats().native_compilations;
        for (std::int32_t warm = 0; warm < 20000 && cpu.jitStats().native_compilations == before; ++warm) {
            step_site(site);
            if (site != 0 && warm % 100 == 0) step_site(0); // Keep anchor MRU.
        }
        ASSERT_EQ(cpu.jitStats().native_compilations, before + 1U) << "site=" << site;
    }
    ASSERT_EQ(cpu.jitStats().native_evictions, 1U);
    auto before = cpu.jitStats();
    step_site(0);
    EXPECT_EQ(cpu.jitStats().native_executions, before.native_executions + 1U);
    EXPECT_EQ(cpu.jitStats().native_compilations, before.native_compilations);
    // Site one is LRU, not the recently touched anchor. Its raw pointer must
    // have been cleared before its ORC resource was removed.
    before = cpu.jitStats();
    step_site(1);
    EXPECT_EQ(cpu.jitStats().native_executions, before.native_executions);
    for (std::int32_t warm = 0; warm < 20000 && cpu.jitStats().native_compilations == before.native_compilations; ++warm) {
        step_site(1);
        if (warm % 100 == 0) step_site(0);
    }
    EXPECT_EQ(cpu.jitStats().native_compilations, before.native_compilations + 1U);
    EXPECT_EQ(cpu.jitStats().native_evictions, 2U);
}
#endif // FIL_HAS_LLVM_JIT
