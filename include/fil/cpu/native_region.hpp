#pragma once

#include "fil/cpu/native_ir.hpp"
#include "fil/cpu/native_stack_plan.hpp"
#include "fil/mem/memory_bus.hpp"

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
    NativeStackPlan stack{};
    // Region-only addressing. Legacy reversible stencils retain their original
    // restricted NativeInst contract. Writeback commits after a guarded access.
    struct Address {
        std::uint8_t rm{0U}, shift{0U};
        bool register_offset{false}, add{true}, index{true}, writeback{false};
    } address{};
    struct PredicatedPair {
        std::array<std::uint32_t, 2U> values{};
        std::array<std::uint8_t, 2U> registers{};
        Condition condition{Condition::al};
        bool valid{false};
    } predicated_pair{};
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

/** Decode every aligned entry in a bounded code window, including entries
 * after checked instructions and the second halfword of wide instructions.
 * The backing span may extend beyond the window for a final wide fetch.
 */
[[nodiscard]] bool buildNativeRegionWindow(std::span<const std::uint8_t> bytes,
    std::uint32_t base, std::uint32_t window_pc, std::uint32_t window_bytes,
    NativeRegionPlan& out, std::string& error);

enum class NativeRegionExit : std::uint32_t {
    none, instruction_budget, cycle_budget, checked_instruction, external_edge, invalid_state
};

/** Entry/exit ABI for a compiled control-flow region. Counters restart at zero
 * per invocation; instruction/fetch state is materialized only at an exit.
 * The caller supplies stable flash fetch costs for the executable backing.
 */
struct NativeRegionContext {
    using CheckedStep = bool (*)(NativeRegionContext*) noexcept;
    CpuState* cpu_state{nullptr};
    std::uint32_t instruction_limit{0U};
    std::uint64_t cycle_limit{0U};
    std::uint32_t sequential_fetch_stall{0U}, nonsequential_fetch_stall{0U};
    std::uint32_t fetch_end{0U};
    bool have_fetch{false};
    std::uint32_t retired_count{0U};
    std::uint64_t retired_cycles{0U};
    NativeRegionExit exit_reason{NativeRegionExit::none};
    // Called only after complete architectural and memory publication at a
    // checked exit. May retire checked work, update credit/fetch state, and
    // request native resumption. Must catch exceptions and retain all code,
    // CPU and backing storage until this invocation returns.
    CheckedStep checked_step{nullptr};
    void* checked_context{nullptr};
    std::uint32_t checked_count{0U};
    // Invocation-scoped bindings to authoritative backed memory, not retained
    // lookup results. Publish with MemoryBus's checked binding APIs while its
    // mapping/tracking modes stay fixed. Empty bindings force checked exits.
    mem::MemoryBus::DirectReadWindow ram_read{}, code_read{};
    mem::MemoryBus::DirectWriteWindow ram_write{};
};

using NativeRegionFunction = void (*)(NativeRegionContext*);

/** Eagerly linked executable image. Tables describe every code window;
 * execution never inserts entries, records target history, or evicts code.
 * The compiling MicroJit must outlive this object and every invocation.
 */
class NativeProgram {
public:
    static constexpr std::uint32_t window_bytes = NativeRegionPlan::max_ops * 2U;
    static constexpr std::size_t max_windows = 1024U;
    NativeProgram(const NativeProgram&) = delete;
    NativeProgram& operator=(const NativeProgram&) = delete;
    [[nodiscard]] NativeRegionFunction entryFor(std::uint32_t pc) const noexcept {
        if ((pc & 1U) != 0U || pc < base_ || pc - base_ >= size_) return nullptr;
        return entries_[(pc - base_) / window_bytes];
    }
    [[nodiscard]] std::uint32_t base() const noexcept { return base_; }
    [[nodiscard]] std::uint32_t size() const noexcept { return size_; }

private:
    friend class MicroJit;
    NativeProgram() = default;
    std::uint32_t base_{0U}, size_{0U};
    std::array<NativeRegionFunction, max_windows> entries_{};
    std::array<void*, max_windows> resumes_{};
};

/** Checked once before entering the compiled address switch. Any compiled
 * instruction can be a resumption point; an uncompiled PC exits unchanged.
 */
[[nodiscard]] bool nativeRegionEntryValid(NativeRegionContext*) noexcept;

/** Invoke a checked service and validate progress before native resumption.
 * Unlike entry validation this retains cumulative retirement accounting.
 */
[[nodiscard]] bool nativeRegionContinue(NativeRegionContext*) noexcept;

} // namespace fil::cpu
