#include "fil/cpu/jit.hpp"
#if defined(FIL_HAS_LLVM_JIT)
#include "fil/cpu/jit_llvm.hpp"
#endif
#include "../test_support.hpp"

#include <array>

void runJitTests() {
    using fil::cpu::DecodedInstruction;
    using fil::cpu::InstrKind;
    using fil::cpu::JitBoundary;
    using fil::cpu::classifyJitBoundary;
    using fil::cpu::planJitBlock;

    DecodedInstruction instruction;
    instruction.kind = InstrKind::add;
    instruction.rd = 0U;
    fil::test::check(
        classifyJitBoundary(instruction) == JitBoundary::none,
        "JIT accepts pure integer ALU instructions"
    );

    instruction.rd = 15U;
    fil::test::check(
        classifyJitBoundary(instruction) == JitBoundary::control_flow,
        "JIT ends a block when an ALU instruction writes PC"
    );

    instruction.kind = InstrKind::ldr;
    instruction.rd = 0U;
    fil::test::check(
        classifyJitBoundary(instruction) == JitBoundary::memory,
        "JIT exits before memory and MMIO dispatch"
    );

    instruction.kind = InstrKind::b;
    fil::test::check(
        classifyJitBoundary(instruction) == JitBoundary::control_flow,
        "JIT ends a block at a branch"
    );

    instruction.kind = InstrKind::svc;
    fil::test::check(
        classifyJitBoundary(instruction) == JitBoundary::exception_or_system,
        "JIT exits before exception-producing instructions"
    );

    instruction.kind = InstrKind::vadd;
    fil::test::check(
        classifyJitBoundary(instruction) == JitBoundary::floating_point,
        "initial JIT tier leaves FP to the interpreter"
    );

    std::array<DecodedInstruction, 4> sequence{};
    sequence[0].kind = InstrKind::movw;
    sequence[1].kind = InstrKind::add;
    sequence[2].kind = InstrKind::cmp;
    sequence[3].kind = InstrKind::ldr;
    const auto plan = planJitBlock(sequence);
    fil::test::check(
        plan.translated_instructions == 3U && plan.boundary == JitBoundary::memory,
        "JIT planner selects the maximal pure prefix"
    );
    fil::test::check(
        planJitBlock(sequence, 2U).translated_instructions == 2U,
        "JIT planner honors its compilation cap"
    );

#if defined(FIL_HAS_LLVM_JIT)
    std::array<DecodedInstruction, 2> native{};
    native[0].kind = InstrKind::movw;
    native[0].rd = 0U;
    native[0].imm = 5U;
    native[1].kind = InstrKind::add;
    native[1].form = fil::cpu::OperandForm::immediate;
    native[1].rd = 1U;
    native[1].rn = 0U;
    native[1].imm = 7U;
    fil::cpu::LlvmJitEngine engine;
    const auto compiled = engine.compile(native);
    std::array<std::uint32_t, 16> registers{};
    std::uint32_t xpsr = 0U;
    fil::test::check(
        compiled.function(registers.data(), &xpsr) == 2U
            && registers[0] == 5U && registers[1] == 12U,
        "LLVM ORC executes a compiled pure-integer block"
    );
#endif
}
