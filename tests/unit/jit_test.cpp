#include "fil/cpu/jit.hpp"
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
}
