#pragma once

#include "fil/cpu/native_ir.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string>

namespace fil::cpu {

struct CpuState;

enum class RegionFlow : std::uint8_t { next, jump, conditional, call, indirect, checked };

struct NativeRegionOp {
    NativeInst instruction{};
    std::uint32_t pc{0U}, raw{0U};
    std::uint32_t fallthrough{0U}, target{0U};
    std::uint16_t fallthrough_cycles{0U}, taken_cycles{0U};
    std::uint8_t size{0U};
    RegionFlow flow{RegionFlow::checked};
    // Edge 0 is fallthrough for next/conditional, target for jump/call.
    // Edge 1 is the taken edge for conditional, return continuation for call.
    static constexpr std::uint16_t external = std::numeric_limits<std::uint16_t>::max();
    std::array<std::uint16_t, 2> edges{external, external};
};

/** A bounded control-flow graph compiled from firmware, without branch profiles
 * or loop unrolling. Backedges refer to existing nodes. Capacity cuts become
 * explicit external edges; unsupported instructions remain checked exits.
 */
struct NativeRegionPlan {
    static constexpr std::size_t max_ops = 256U;
    std::array<NativeRegionOp, max_ops> ops{};
    std::uint16_t count{0U};
    std::uint32_t entry_pc{0U};
    bool capacity_limited{false};
};

/** Build from one immutable executable backing span. Failure leaves out alone. */
[[nodiscard]] bool buildNativeRegion(std::span<const std::uint8_t> bytes,
    std::uint32_t base, std::uint32_t entry_pc, NativeRegionPlan& out, std::string& error);

enum class NativeRegionExit : std::uint32_t {
    none, instruction_budget, cycle_budget, checked_instruction, external_edge, invalid_state
};

/** Entry/exit ABI for a compiled control-flow region. Counters restart at zero
 * per invocation; instruction/fetch state is materialized only at an exit.
 * The caller supplies stable flash fetch costs for the executable backing.
 */
struct NativeRegionContext {
    CpuState* cpu_state{nullptr};
    std::uint32_t instruction_limit{0U};
    std::uint64_t cycle_limit{0U};
    std::uint32_t sequential_fetch_stall{0U}, nonsequential_fetch_stall{0U};
    std::uint32_t fetch_end{0U};
    bool have_fetch{false};
    std::uint32_t retired_count{0U};
    std::uint64_t retired_cycles{0U};
    NativeRegionExit exit_reason{NativeRegionExit::none};
};

using NativeRegionFunction = void (*)(NativeRegionContext*);

/** Checked once before entering the compiled address switch. Any compiled
 * instruction can be a resumption point; an uncompiled PC exits unchanged.
 */
[[nodiscard]] bool nativeRegionEntryValid(NativeRegionContext*) noexcept;

} // namespace fil::cpu
