#pragma once

/** @file micro_jit_ir.hpp
 *  @brief Bespoke FilIR: tiny guest IR for the LLVM-free micro-JIT.
 *
 *  FilIR replaces LLVM IR for straight-line Cortex-M integer kernels.
 *  One DecodedInstruction lowers to exactly one MicroInst (no optimizer,
 *  no modules, no linker). The stencil emitter (phase 2) memcpy's one
 *  host template per MicroInst; this header + micro_jit_ir.cpp own
 *  lowering, validation, cost model, and a reference interpreter used
 *  for differential testing against the existing executor.
 *
 *  Scope mirrors the proven LLVM subset plus shifted register forms:
 *  NOP, MOV/MOVW/MOVT immediate, ADD/SUB/CMP/CMN immediate, unshifted
 *  AND/ORR/EOR/BIC and ADD/SUB, shifted ADD/SUB, LSL/LSR/ASR/ROR/RRX,
 *  and a terminal unconditional B/BL. Everything else (memory/MMIO,
 *  ADC/SBC, SP/PC operands, IT, system, FP) stays on cached handlers.
 */

#include "fil/cpu/instruction.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace fil::cpu {

/** Guest IR opcodes; one stencil per opcode in the emitter. */
enum class MicroOp : std::uint8_t {
    nop,
    mov_imm,
    movw,
    movt,
    add_imm,
    sub_imm,
    cmp_imm,
    cmn_imm,
    add_reg,
    sub_reg,
    and_reg,
    orr_reg,
    eor_reg,
    bic_reg,
    add_shift,
    sub_shift,
    lsl,
    lsr,
    asr,
    ror,
    rrx,
    b,
    bl,
    b_cond,
    cbz,
    cbnz,
};

/** Lowered guest instruction with normalized operands. */
struct MicroInst {
    MicroOp op{MicroOp::nop};
    std::uint8_t rd{0};
    std::uint8_t rn{0};
    std::uint8_t rm{0};
    std::uint32_t imm{0};
    Condition condition{Condition::al};
    ShiftType shift{ShiftType::lsl};
    std::uint8_t shift_amount{0};
    bool set_flags{false};
    /// Shifter carry for flag-setting MOV immediate (else retained C).
    bool carry_valid{false};
    bool carry{false};
    /// True for immediate-specified shifts (value=rm); else value=rn, amount=rm&0xff.
    bool shift_immediate{true};
    std::int32_t branch_offset{0};
    std::uint32_t pc{0};
    std::uint8_t size{0};
};

/** Already-decoded guest instruction with address/width (LLVM-free alias). */
struct MicroSource {
    DecodedInstruction decoded{};
    std::uint32_t pc{0};
    std::uint8_t size{0};
};

/** Why lowering declined a guest instruction. */
enum class MicroLowerReason : std::uint8_t {
    ok,
    bad_condition,
    bad_size,
    bad_pc,
    unsupported_kind,
    bad_shape,
    bad_register,
    bad_immediate,
    branch_not_terminal,
};

/** Result of lowering one instruction or one block. */
struct MicroLowerResult {
    bool ok{false};
    MicroLowerReason reason{MicroLowerReason::ok};
    std::string detail;
};

/** Straight-line FilIR block: 1:1 with the admitted guest prefix. */
struct MicroBlock {
    std::vector<MicroInst> ops;
    bool ends_with_branch{false};
    std::uint32_t entry_pc{0};
    static constexpr std::size_t max_ops = 16U;
};

/** Minimal integer CPU for the FilIR reference interpreter (r0-r12, LR, PC, xPSR). */
struct MicroCpu {
    std::array<std::uint32_t, 13> r{};
    std::uint32_t lr{0};
    std::uint32_t pc{0};
    std::uint32_t xpsr{1U << 24U};
};

[[nodiscard]] MicroLowerResult tryLowerMicroInst(const MicroSource& source, MicroInst& out) noexcept;

/** Lower a straight-line prefix; B/BL may only appear as the final op. */
[[nodiscard]] MicroLowerResult buildMicroBlock(
    std::span<const MicroSource> sources, MicroBlock& out) noexcept;

/** Static cost: one per op, branch costs 2 (taken-refill estimate). */
[[nodiscard]] std::uint64_t microBlockCost(const MicroBlock& block) noexcept;

/** Reference interpreter: executes up to max_instructions, returns retired count. */
std::size_t interpretMicroBlock(const MicroBlock& block, MicroCpu& cpu, std::size_t max_instructions) noexcept;

[[nodiscard]] const char* toString(MicroOp op) noexcept;
[[nodiscard]] const char* toString(MicroLowerReason reason) noexcept;

} // namespace fil::cpu
