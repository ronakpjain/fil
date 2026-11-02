#pragma once

#include "fil/cpu/native_ir.hpp"
#include "fil/cpu/native_stack_plan.hpp"
#include "fil/cpu/micro_jit.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace fil::cpu {

struct NativeTraceOp {
    DecodedInstruction decoded{};
    NativeInst native{};
    std::uint32_t raw{0};
    std::uint32_t pc{0};
    std::uint32_t expected_next_pc{0};
    std::uint16_t execution_cycles{0};
    std::uint8_t size{0};
    bool expected_taken{false};
    NativeStackPlan stack_plan{};
};

struct NativeTracePlan {
    std::vector<NativeTraceOp> ops;
    std::uint64_t generation{0};
    NativeFunction function{nullptr};
    static constexpr std::size_t max_ops = 64U;
};

/** Allocation-free portable reference execution of an already validated plan. */
void interpretNativeTrace(const NativeTracePlan& plan, NativeExecutionContext& context) noexcept;

/** Lower/validate atomically. Inputs' derived native/stack metadata is ignored. */
[[nodiscard]] bool buildNativeTrace(std::span<const NativeTraceOp> sources,
                                    NativeTracePlan& out, std::string& error);

} // namespace fil::cpu
