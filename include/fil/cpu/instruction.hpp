#pragma once

/** @file instruction.hpp
 *  @brief Semantic Thumb instruction representation and architectural helpers.
 */

#include <cstdint>

namespace fil::cpu {

/** @brief Tests whether a value is an ARMv7-M EXC_RETURN token. */
[[nodiscard]] constexpr bool isExceptionReturn(const std::uint32_t value) noexcept {
    switch (value) {
    case 0xfffffff1U:
    case 0xfffffff9U:
    case 0xfffffffdU:
    case 0xffffffe1U:
    case 0xffffffe9U:
    case 0xffffffedU:
        return true;
    default:
        return false;
    }
}

/** @brief ARM condition-code encodings. */
enum class Condition : std::uint8_t {
    eq = 0x0,
    ne = 0x1,
    cs = 0x2,
    cc = 0x3,
    mi = 0x4,
    pl = 0x5,
    vs = 0x6,
    vc = 0x7,
    hi = 0x8,
    ls = 0x9,
    ge = 0xa,
    lt = 0xb,
    gt = 0xc,
    le = 0xd,
    al = 0xe,
    nv = 0xf,
};

/** @brief Shift operations shared by decoder and executor. */
enum class ShiftType : std::uint8_t {
    lsl,
    lsr,
    asr,
    ror,
    rrx,
};

/** @brief Source form carried by a semantic decoded instruction. */
enum class OperandForm : std::uint8_t {
    none,
    immediate,
    register_value,
    literal,
    register_list,
};

/** @brief Instruction operations currently understood by the Cortex-M4 core. */
enum class InstrKind : std::uint8_t {
    undefined,
    mov,
    movw,
    movt,
    add,
    adc,
    sub,
    sbc,
    rsb,
    cmp,
    cmn,
    tst,
    and_,
    orr,
    eor,
    bic,
    mvn,
    orn,
    mul,
    mla,
    mls,
    umull,
    udiv,
    clz,
    bfc,
    ubfx,
    sxtb,
    sxth,
    uxtb,
    uxth,
    rev,
    rev16,
    revsh,
    sdiv,
    smull,
    uadd8,
    sel,
    lsl,
    lsr,
    asr,
    ror,
    rrx,
    ldr,
    str,
    ldrb,
    strb,
    ldrh,
    strh,
    ldrsb,
    ldrsh,
    ldrd,
    strd,
    ldm,
    stm,
    push,
    pop,
    b,
    bl,
    bx,
    blx,
    cbz,
    cbnz,
    it,
    nop,
    dmb,
    dsb,
    isb,
    svc,
    mrs,
    msr,
    cps,
    wfi,
    wfe,
    sev,
    vstm,
    vldm,
    vldr,
    vstr,
    vmov_core_to_single,
    vmov_single_to_core,
    vmov_single,
    vmov_immediate,
    vcvt_f32_s32,
    vcvt_f32_u32,
    vcvt_s32_f32,
    vcvt_u32_f32,
    vadd,
    vsub,
    vmul,
    vnmul,
    vdiv,
    vfma,
    vfms,
    vfnms,
    vcmp,
    vneg,
    vabs,
    vsqrt,
    vmrs,
    bkpt,
};

/** @brief Decoder output with operands normalized for instruction handlers. */
struct DecodedInstruction {
    InstrKind kind{InstrKind::undefined};
    Condition condition{Condition::al};
    OperandForm form{OperandForm::none};
    std::uint8_t rd{0};
    std::uint8_t rn{0};
    std::uint8_t rm{0};
    std::uint8_t ra{0};
    std::uint32_t imm{0};
    std::int32_t branch_offset{0};
    std::uint16_t register_list{0};
    ShiftType shift_type{ShiftType::lsl};
    std::uint8_t shift_amount{0};
    bool set_flags{false};
    bool index{true};
    bool add{true};
    bool writeback{false};
    bool immediate_carry_valid{false};
    bool immediate_carry{false};
    bool fp_double{false};
    bool is_32bit{false};
    std::uint32_t raw{0};
};

/** @brief Result of an add-with-carry operation, including all APSR flags. */
struct AddResult {
    std::uint32_t value{0};
    bool n{false};
    bool z{false};
    bool c{false};
    bool v{false};
};

/** @brief Result of a shift operation and its carry output. */
struct ShiftResult {
    std::uint32_t value{0};
    bool carry{false};
};

/** @brief Performs ARM AddWithCarry without signed-overflow undefined behavior. */
[[nodiscard]] AddResult addWithCarry(
    std::uint32_t x,
    std::uint32_t y,
    bool carry_in
) noexcept;

/** @brief Performs an ARM shift and returns the resulting carry bit. */
[[nodiscard]] ShiftResult shiftC(
    std::uint32_t value,
    ShiftType type,
    std::uint32_t amount,
    bool old_carry
) noexcept;

/** @brief Evaluates one ARM condition against an xPSR value. */
[[nodiscard]] bool conditionPasses(Condition condition, std::uint32_t xpsr) noexcept;

/** @brief Returns whether an IT state denotes an active block. */
[[nodiscard]] constexpr bool inItBlock(const std::uint8_t it_state) noexcept {
    return (it_state & 0x0fU) != 0;
}

/** @brief Returns whether the current instruction is last in an IT block. */
[[nodiscard]] constexpr bool lastInItBlock(const std::uint8_t it_state) noexcept {
    return inItBlock(it_state) && (it_state & 0x07U) == 0;
}

/** @brief Gets the condition applying to the current IT-block slot. */
[[nodiscard]] constexpr Condition currentItCondition(const std::uint8_t it_state) noexcept {
    return static_cast<Condition>((it_state >> 4U) & 0x0fU);
}

/** @brief Advances the compact architectural IT state by one instruction. */
[[nodiscard]] std::uint8_t advanceItState(std::uint8_t it_state) noexcept;

} // namespace fil::cpu
