#include "fil/cpu/decoder.hpp"

#include <array>
#include <bit>
#include <cstddef>

namespace fil::cpu {
namespace {

template <unsigned int Bits>
[[nodiscard]] constexpr std::int32_t signExtend(const std::uint32_t value) noexcept {
    static_assert(Bits > 0 && Bits < 32);
    constexpr std::uint32_t sign = std::uint32_t{1} << (Bits - 1U);
    constexpr std::uint32_t mask = (std::uint32_t{1} << Bits) - 1U;
    const std::uint32_t narrowed = value & mask;
    if ((narrowed & sign) == 0) {
        return static_cast<std::int32_t>(narrowed);
    }
    return static_cast<std::int32_t>(
        static_cast<std::int64_t>(narrowed) - (std::int64_t{1} << Bits)
    );
}

[[nodiscard]] DecodedInstruction base16(
    const std::uint16_t raw,
    const InstrKind kind,
    const OperandForm form = OperandForm::none
) noexcept {
    DecodedInstruction result;
    result.kind = kind;
    result.form = form;
    result.raw = raw;
    return result;
}

[[nodiscard]] DecodedInstruction base32(
    const std::uint16_t first,
    const std::uint16_t second,
    const InstrKind kind,
    const OperandForm form = OperandForm::none
) noexcept {
    auto result = base16(first, kind, form);
    result.is_32bit = true;
    result.raw = (static_cast<std::uint32_t>(first) << 16U) | second;
    return result;
}

struct ExpandedImmediate {
    std::uint32_t value{0};
    bool carry_valid{false};
    bool carry{false};
};

[[nodiscard]] std::optional<ExpandedImmediate> thumbExpandImmediate(
    const std::uint16_t imm12
) noexcept {
    const std::uint32_t imm8 = imm12 & 0xffU;
    if ((imm12 & 0xc00U) == 0U) {
        switch ((imm12 >> 8U) & 0x3U) {
        case 0U: return ExpandedImmediate{imm8, false, false};
        case 1U:
            if (imm8 == 0U) return std::nullopt;
            return ExpandedImmediate{(imm8 << 16U) | imm8, false, false};
        case 2U:
            if (imm8 == 0U) return std::nullopt;
            return ExpandedImmediate{(imm8 << 24U) | (imm8 << 8U), false, false};
        case 3U:
            if (imm8 == 0U) return std::nullopt;
            return ExpandedImmediate{imm8 * 0x01010101U, false, false};
        default: return std::nullopt;
        }
    }

    const std::uint32_t unrotated = 0x80U | (imm12 & 0x7fU);
    const auto rotation = static_cast<int>((imm12 >> 7U) & 0x1fU);
    const std::uint32_t value = std::rotr(unrotated, rotation);
    return ExpandedImmediate{value, true, (value & 0x80000000U) != 0U};
}

[[nodiscard]] std::optional<DecodedInstruction> decodeShiftImmediate(const std::uint16_t raw) {
    const auto selector = static_cast<std::uint8_t>((raw >> 11U) & 0x3U);
    InstrKind kind = InstrKind::lsl;
    ShiftType type = ShiftType::lsl;
    if (selector == 1U) {
        kind = InstrKind::lsr;
        type = ShiftType::lsr;
    } else if (selector == 2U) {
        kind = InstrKind::asr;
        type = ShiftType::asr;
    } else if (selector != 0U) {
        return std::nullopt;
    }

    auto result = base16(raw, kind, OperandForm::immediate);
    result.rd = static_cast<std::uint8_t>(raw & 0x7U);
    result.rm = static_cast<std::uint8_t>((raw >> 3U) & 0x7U);
    result.shift_type = type;
    result.shift_amount = static_cast<std::uint8_t>((raw >> 6U) & 0x1fU);
    if (type != ShiftType::lsl && result.shift_amount == 0U) {
        result.shift_amount = 32U;
    }
    result.set_flags = true;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeAddSubtract(const std::uint16_t raw) {
    const bool immediate = (raw & 0x0400U) != 0;
    const bool subtract = (raw & 0x0200U) != 0;
    auto result = base16(
        raw,
        subtract ? InstrKind::sub : InstrKind::add,
        immediate ? OperandForm::immediate : OperandForm::register_value
    );
    result.rd = static_cast<std::uint8_t>(raw & 0x7U);
    result.rn = static_cast<std::uint8_t>((raw >> 3U) & 0x7U);
    if (immediate) {
        result.imm = (raw >> 6U) & 0x7U;
    } else {
        result.rm = static_cast<std::uint8_t>((raw >> 6U) & 0x7U);
    }
    result.set_flags = true;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeImmediate8(const std::uint16_t raw) {
    const auto operation = static_cast<std::uint8_t>((raw >> 11U) & 0x3U);
    InstrKind kind = InstrKind::mov;
    if (operation == 1U) kind = InstrKind::cmp;
    if (operation == 2U) kind = InstrKind::add;
    if (operation == 3U) kind = InstrKind::sub;

    auto result = base16(raw, kind, OperandForm::immediate);
    result.rd = static_cast<std::uint8_t>((raw >> 8U) & 0x7U);
    result.rn = result.rd;
    result.imm = raw & 0xffU;
    result.set_flags = true;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeDataProcessing(const std::uint16_t raw) {
    static constexpr std::array<InstrKind, 16> operations = {
        InstrKind::and_, InstrKind::eor, InstrKind::lsl, InstrKind::lsr,
        InstrKind::asr, InstrKind::adc, InstrKind::sbc, InstrKind::ror,
        InstrKind::tst, InstrKind::rsb, InstrKind::cmp, InstrKind::cmn,
        InstrKind::orr, InstrKind::mul, InstrKind::bic, InstrKind::mvn,
    };
    const auto opcode = static_cast<std::uint8_t>((raw >> 6U) & 0x0fU);
    auto result = base16(raw, operations[opcode], OperandForm::register_value);
    result.rd = static_cast<std::uint8_t>(raw & 0x7U);
    result.rn = result.rd;
    result.rm = static_cast<std::uint8_t>((raw >> 3U) & 0x7U);
    result.set_flags = true;
    result.shift_type = operations[opcode] == InstrKind::lsr ? ShiftType::lsr
        : operations[opcode] == InstrKind::asr ? ShiftType::asr
        : operations[opcode] == InstrKind::ror ? ShiftType::ror
        : ShiftType::lsl;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeSpecialData(const std::uint16_t raw) {
    const auto operation = static_cast<std::uint8_t>((raw >> 8U) & 0x3U);
    const auto rd = static_cast<std::uint8_t>((raw & 0x7U) | ((raw >> 4U) & 0x8U));
    const auto rm = static_cast<std::uint8_t>((raw >> 3U) & 0x0fU);

    if (operation == 3U) {
        if ((raw & 0x7U) != 0U) return std::nullopt;
        auto result = base16(
            raw,
            (raw & 0x0080U) != 0 ? InstrKind::blx : InstrKind::bx,
            OperandForm::register_value
        );
        result.rm = rm;
        return result;
    }

    const InstrKind kind = operation == 0U ? InstrKind::add
        : operation == 1U ? InstrKind::cmp : InstrKind::mov;
    auto result = base16(raw, kind, OperandForm::register_value);
    result.rd = rd;
    result.rn = rd;
    result.rm = rm;
    result.set_flags = operation == 1U;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeLiteralLoad(const std::uint16_t raw) {
    auto result = base16(raw, InstrKind::ldr, OperandForm::literal);
    result.rd = static_cast<std::uint8_t>((raw >> 8U) & 0x7U);
    result.rn = 15U;
    result.imm = (raw & 0xffU) << 2U;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeRegisterTransfer(const std::uint16_t raw) {
    static constexpr std::array<InstrKind, 8> operations = {
        InstrKind::str, InstrKind::strh, InstrKind::strb, InstrKind::ldrsb,
        InstrKind::ldr, InstrKind::ldrh, InstrKind::ldrb, InstrKind::ldrsh,
    };
    const auto opcode = static_cast<std::uint8_t>((raw >> 9U) & 0x7U);
    auto result = base16(raw, operations[opcode], OperandForm::register_value);
    result.rd = static_cast<std::uint8_t>(raw & 0x7U);
    result.rn = static_cast<std::uint8_t>((raw >> 3U) & 0x7U);
    result.rm = static_cast<std::uint8_t>((raw >> 6U) & 0x7U);
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeImmediateTransfer(const std::uint16_t raw) {
    const std::uint16_t family = raw & 0xf000U;
    const bool load = (raw & 0x0800U) != 0;
    InstrKind kind = load ? InstrKind::ldr : InstrKind::str;
    std::uint32_t scale = 4U;
    if (family == 0x7000U) {
        kind = load ? InstrKind::ldrb : InstrKind::strb;
        scale = 1U;
    } else if (family == 0x8000U) {
        kind = load ? InstrKind::ldrh : InstrKind::strh;
        scale = 2U;
    }
    auto result = base16(raw, kind, OperandForm::immediate);
    result.rd = static_cast<std::uint8_t>(raw & 0x7U);
    result.rn = static_cast<std::uint8_t>((raw >> 3U) & 0x7U);
    result.imm = ((raw >> 6U) & 0x1fU) * scale;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeSpTransfer(const std::uint16_t raw) {
    auto result = base16(
        raw,
        (raw & 0x0800U) != 0 ? InstrKind::ldr : InstrKind::str,
        OperandForm::immediate
    );
    result.rd = static_cast<std::uint8_t>((raw >> 8U) & 0x7U);
    result.rn = 13U;
    result.imm = (raw & 0xffU) << 2U;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeAddressGeneration(const std::uint16_t raw) {
    auto result = base16(raw, InstrKind::add, OperandForm::immediate);
    result.rd = static_cast<std::uint8_t>((raw >> 8U) & 0x7U);
    result.rn = (raw & 0x0800U) != 0 ? 13U : 15U;
    result.imm = (raw & 0xffU) << 2U;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeMiscellaneous(const std::uint16_t raw) {
    if ((raw & 0xff00U) == 0xba00U) {
        InstrKind kind = InstrKind::undefined;
        const auto operation = static_cast<std::uint8_t>((raw >> 6U) & 0x3U);
        if (operation == 0U) kind = InstrKind::rev;
        if (operation == 1U) kind = InstrKind::rev16;
        if (operation == 3U) kind = InstrKind::revsh;
        if (kind == InstrKind::undefined) return std::nullopt;
        auto result = base16(raw, kind, OperandForm::register_value);
        result.rd = static_cast<std::uint8_t>(raw & 0x7U);
        result.rm = static_cast<std::uint8_t>((raw >> 3U) & 0x7U);
        return result;
    }
    if ((raw & 0xff00U) == 0xb200U) {
        static constexpr std::array<InstrKind, 4> operations = {
            InstrKind::sxth, InstrKind::sxtb, InstrKind::uxth, InstrKind::uxtb,
        };
        auto result = base16(
            raw, operations[(raw >> 6U) & 0x3U], OperandForm::register_value
        );
        result.rd = static_cast<std::uint8_t>(raw & 0x7U);
        result.rm = static_cast<std::uint8_t>((raw >> 3U) & 0x7U);
        return result;
    }
    if ((raw & 0xff00U) == 0xbe00U) {
        auto result = base16(raw, InstrKind::bkpt, OperandForm::immediate);
        result.imm = raw & 0xffU;
        return result;
    }
    if ((raw & 0xff00U) == 0xbf00U) {
        if (raw == 0xbf20U) return base16(raw, InstrKind::wfe);
        if (raw == 0xbf30U) return base16(raw, InstrKind::wfi);
        if (raw == 0xbf40U) return base16(raw, InstrKind::sev);
        const auto mask = static_cast<std::uint8_t>(raw & 0x0fU);
        const auto first_condition = static_cast<std::uint8_t>((raw >> 4U) & 0x0fU);
        if (mask == 0U) {
            if (raw == 0xbf00U) return base16(raw, InstrKind::nop);
            return std::nullopt;
        }
        if (first_condition >= static_cast<std::uint8_t>(Condition::al)) return std::nullopt;
        auto result = base16(raw, InstrKind::it, OperandForm::immediate);
        result.condition = static_cast<Condition>(first_condition);
        result.imm = (static_cast<std::uint32_t>(first_condition) << 4U) | mask;
        return result;
    }
    if ((raw & 0xffe8U) == 0xb660U) {
        const std::uint8_t affected = static_cast<std::uint8_t>(raw & 0x7U);
        if (affected == 0U || (affected & 0x4U) != 0U) return std::nullopt;
        auto result = base16(raw, InstrKind::cps, OperandForm::immediate);
        result.imm = affected;
        result.add = (raw & 0x0010U) == 0U;
        return result;
    }
    if ((raw & 0xf500U) == 0xb100U) {
        auto result = base16(
            raw,
            (raw & 0x0800U) != 0 ? InstrKind::cbnz : InstrKind::cbz,
            OperandForm::immediate
        );
        result.rn = static_cast<std::uint8_t>(raw & 0x7U);
        result.imm = ((raw >> 3U) & 0x1fU) << 1U;
        result.imm |= ((raw >> 9U) & 0x1U) << 6U;
        return result;
    }
    if ((raw & 0xfe00U) == 0xb400U) {
        auto result = base16(raw, InstrKind::push, OperandForm::register_list);
        result.register_list = static_cast<std::uint16_t>(raw & 0xffU);
        if ((raw & 0x0100U) != 0) result.register_list |= std::uint16_t{1} << 14U;
        result.rn = 13U;
        result.writeback = true;
        result.add = false;
        result.index = true;
        if (result.register_list == 0U) return std::nullopt;
        return result;
    }
    if ((raw & 0xfe00U) == 0xbc00U) {
        auto result = base16(raw, InstrKind::pop, OperandForm::register_list);
        result.register_list = static_cast<std::uint16_t>(raw & 0xffU);
        if ((raw & 0x0100U) != 0) result.register_list |= std::uint16_t{1} << 15U;
        result.rn = 13U;
        result.writeback = true;
        result.add = true;
        result.index = false;
        if (result.register_list == 0U) return std::nullopt;
        return result;
    }
    if ((raw & 0xff00U) == 0xb000U) {
        auto result = base16(
            raw,
            (raw & 0x0080U) != 0 ? InstrKind::sub : InstrKind::add,
            OperandForm::immediate
        );
        result.rd = 13U;
        result.rn = 13U;
        result.imm = (raw & 0x7fU) << 2U;
        return result;
    }
    return std::nullopt;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeMultiple(const std::uint16_t raw) {
    auto result = base16(
        raw,
        (raw & 0x0800U) != 0 ? InstrKind::ldm : InstrKind::stm,
        OperandForm::register_list
    );
    result.rn = static_cast<std::uint8_t>((raw >> 8U) & 0x7U);
    result.register_list = static_cast<std::uint16_t>(raw & 0xffU);
    if (result.register_list == 0U) return std::nullopt;
    result.writeback = result.kind == InstrKind::stm
        || (result.register_list & (std::uint16_t{1} << result.rn)) == 0U;
    result.add = true;
    result.index = false;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeConditionalBranch(const std::uint16_t raw) {
    const auto condition = static_cast<std::uint8_t>((raw >> 8U) & 0x0fU);
    if (condition == static_cast<std::uint8_t>(Condition::nv)) {
        auto result = base16(raw, InstrKind::svc, OperandForm::immediate);
        result.imm = raw & 0xffU;
        return result;
    }
    if (condition >= static_cast<std::uint8_t>(Condition::al)) return std::nullopt;
    auto result = base16(raw, InstrKind::b, OperandForm::immediate);
    result.condition = static_cast<Condition>(condition);
    result.branch_offset = signExtend<9>((raw & 0xffU) << 1U);
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeUnconditionalBranch(const std::uint16_t raw) {
    auto result = base16(raw, InstrKind::b, OperandForm::immediate);
    result.branch_offset = signExtend<12>((raw & 0x07ffU) << 1U);
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeMovWide(
    const std::uint16_t first,
    const std::uint16_t second
) {
    const bool top = (first & 0x0080U) != 0;
    auto result = base32(first, second, top ? InstrKind::movt : InstrKind::movw, OperandForm::immediate);
    result.rd = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
    result.imm = (static_cast<std::uint32_t>(first & 0x000fU) << 12U)
        | (static_cast<std::uint32_t>((first >> 10U) & 0x1U) << 11U)
        | (static_cast<std::uint32_t>((second >> 12U) & 0x7U) << 8U)
        | (second & 0x00ffU);
    if (result.rd == 15U) return std::nullopt;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeAddSubWideImmediate(
    const std::uint16_t first,
    const std::uint16_t second
) {
    const bool subtract = (first & 0x0080U) != 0U;
    auto result = base32(
        first, second, subtract ? InstrKind::sub : InstrKind::add,
        OperandForm::immediate
    );
    result.rn = static_cast<std::uint8_t>(first & 0x0fU);
    result.rd = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
    result.imm = ((first & 0x0400U) << 1U)
        | ((second & 0x7000U) >> 4U) | (second & 0x00ffU);
    if (result.rd == 15U) return std::nullopt;
    return result;
}

[[nodiscard]] ShiftType decodeImmediateShiftType(
    const std::uint8_t encoded,
    const std::uint8_t amount
) noexcept {
    if (encoded == 0U) return ShiftType::lsl;
    if (encoded == 1U) return ShiftType::lsr;
    if (encoded == 2U) return ShiftType::asr;
    return amount == 0U ? ShiftType::rrx : ShiftType::ror;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeBranchWide(
    std::uint16_t first,
    std::uint16_t second
);

[[nodiscard]] std::optional<DecodedInstruction> decodeDataProcessingImmediate(
    const std::uint16_t first,
    const std::uint16_t second
) {
    const auto opcode = static_cast<std::uint8_t>((first >> 5U) & 0x0fU);
    const bool set_flags = (first & 0x0010U) != 0U;
    const auto rn = static_cast<std::uint8_t>(first & 0x0fU);
    const auto rd = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
    const std::uint16_t imm12 = static_cast<std::uint16_t>(
        ((first & 0x0400U) << 1U) | ((second & 0x7000U) >> 4U) | (second & 0x00ffU)
    );
    const auto expanded = thumbExpandImmediate(imm12);
    if (!expanded) return std::nullopt;

    InstrKind kind = InstrKind::undefined;
    switch (opcode) {
    case 0U: kind = InstrKind::and_; break;
    case 1U: kind = InstrKind::bic; break;
    case 2U: kind = InstrKind::orr; break;
    case 3U: kind = InstrKind::orn; break;
    case 4U: kind = InstrKind::eor; break;
    case 8U: kind = InstrKind::add; break;
    case 10U: kind = InstrKind::adc; break;
    case 11U: kind = InstrKind::sbc; break;
    case 13U: kind = InstrKind::sub; break;
    case 14U: kind = InstrKind::rsb; break;
    default: return std::nullopt;
    }

    if (set_flags && rd == 15U) {
        if (kind == InstrKind::and_) kind = InstrKind::tst;
        else if (kind == InstrKind::add) kind = InstrKind::cmn;
        else if (kind == InstrKind::sub) kind = InstrKind::cmp;
        else return std::nullopt;
    } else if (rn == 15U) {
        if (kind == InstrKind::orr) kind = InstrKind::mov;
        else if (kind == InstrKind::orn) kind = InstrKind::mvn;
        else return std::nullopt;
    }

    if (rd == 15U && kind != InstrKind::tst && kind != InstrKind::cmp
        && kind != InstrKind::cmn) {
        return std::nullopt;
    }
    if (rn == 15U && kind != InstrKind::mov && kind != InstrKind::mvn) {
        return std::nullopt;
    }

    auto result = base32(first, second, kind, OperandForm::immediate);
    result.rd = rd;
    result.rn = rn;
    result.imm = expanded->value;
    result.set_flags = set_flags;
    result.immediate_carry_valid = expanded->carry_valid;
    result.immediate_carry = expanded->carry;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeDataProcessingRegister(
    const std::uint16_t first,
    const std::uint16_t second
) {
    const auto opcode = static_cast<std::uint8_t>((first >> 5U) & 0x0fU);
    const bool set_flags = (first & 0x0010U) != 0U;
    const auto rn = static_cast<std::uint8_t>(first & 0x0fU);
    const auto rd = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
    const auto rm = static_cast<std::uint8_t>(second & 0x0fU);
    const auto shift_encoding = static_cast<std::uint8_t>((second >> 4U) & 0x3U);
    const auto amount = static_cast<std::uint8_t>(
        ((second >> 10U) & 0x1cU) | ((second >> 6U) & 0x3U)
    );

    InstrKind kind = InstrKind::undefined;
    switch (opcode) {
    case 0U: kind = InstrKind::and_; break;
    case 1U: kind = InstrKind::bic; break;
    case 2U: kind = InstrKind::orr; break;
    case 3U: kind = InstrKind::orn; break;
    case 4U: kind = InstrKind::eor; break;
    case 8U: kind = InstrKind::add; break;
    case 10U: kind = InstrKind::adc; break;
    case 11U: kind = InstrKind::sbc; break;
    case 13U: kind = InstrKind::sub; break;
    case 14U: kind = InstrKind::rsb; break;
    default: return std::nullopt;
    }

    if (set_flags && rd == 15U) {
        if (kind == InstrKind::and_) kind = InstrKind::tst;
        else if (kind == InstrKind::add) kind = InstrKind::cmn;
        else if (kind == InstrKind::sub) kind = InstrKind::cmp;
        else return std::nullopt;
    } else if (rn == 15U) {
        if (kind == InstrKind::orr) kind = InstrKind::mov;
        else if (kind == InstrKind::orn) kind = InstrKind::mvn;
        else return std::nullopt;
    }

    if (rm == 15U || (rd == 15U && kind != InstrKind::tst && kind != InstrKind::cmp
        && kind != InstrKind::cmn)) {
        return std::nullopt;
    }
    if (rn == 15U && kind != InstrKind::mov && kind != InstrKind::mvn) {
        return std::nullopt;
    }

    auto result = base32(first, second, kind, OperandForm::register_value);
    result.rd = rd;
    result.rn = rn;
    result.rm = rm;
    result.shift_amount = amount;
    result.shift_type = decodeImmediateShiftType(shift_encoding, amount);
    result.set_flags = set_flags;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeRegisterShift(
    const std::uint16_t first,
    const std::uint16_t second
) {
    const auto rn = static_cast<std::uint8_t>(first & 0x0fU);
    const auto rd = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
    const auto rm = static_cast<std::uint8_t>(second & 0x0fU);
    if (rn == 15U || rd == 15U || rm == 15U) return std::nullopt;

    static constexpr std::array<InstrKind, 4> kinds = {
        InstrKind::lsl, InstrKind::lsr, InstrKind::asr, InstrKind::ror,
    };
    static constexpr std::array<ShiftType, 4> shifts = {
        ShiftType::lsl, ShiftType::lsr, ShiftType::asr, ShiftType::ror,
    };
    const auto operation = static_cast<std::uint8_t>((first >> 5U) & 0x3U);
    auto result = base32(first, second, kinds[operation], OperandForm::register_value);
    result.rd = rd;
    result.rn = rn;
    result.rm = rm;
    result.shift_type = shifts[operation];
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeExtendWide(
    const std::uint16_t first,
    const std::uint16_t second
) {
    InstrKind kind = InstrKind::undefined;
    switch ((first >> 4U) & 0x7U) {
    case 0U: kind = InstrKind::sxth; break;
    case 1U: kind = InstrKind::uxth; break;
    case 4U: kind = InstrKind::sxtb; break;
    case 5U: kind = InstrKind::uxtb; break;
    default: return std::nullopt;
    }
    auto result = base32(first, second, kind, OperandForm::register_value);
    result.rd = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
    result.rm = static_cast<std::uint8_t>(second & 0x0fU);
    result.shift_amount = static_cast<std::uint8_t>(((second >> 4U) & 0x3U) * 8U);
    if (result.rd == 15U || result.rm == 15U) return std::nullopt;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeMultiply(
    const std::uint16_t first,
    const std::uint16_t second
) {
    const auto rn = static_cast<std::uint8_t>(first & 0x0fU);
    const auto rm = static_cast<std::uint8_t>(second & 0x0fU);
    const auto rd = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
    const auto ra = static_cast<std::uint8_t>((second >> 12U) & 0x0fU);
    if (rn == 15U || rm == 15U || rd == 15U) return std::nullopt;
    const bool subtract = (second & 0x0010U) != 0U;
    if (subtract && ra == 15U) return std::nullopt;

    InstrKind kind = subtract ? InstrKind::mls : InstrKind::mla;
    if (!subtract && ra == 15U) kind = InstrKind::mul;
    auto result = base32(first, second, kind, OperandForm::register_value);
    result.rd = rd;
    result.rn = rn;
    result.rm = rm;
    result.ra = ra;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeUmull(
    const std::uint16_t first,
    const std::uint16_t second
) {
    auto result = base32(first, second, InstrKind::umull, OperandForm::register_value);
    result.rn = static_cast<std::uint8_t>(first & 0x0fU);
    result.rm = static_cast<std::uint8_t>(second & 0x0fU);
    result.rd = static_cast<std::uint8_t>((second >> 12U) & 0x0fU);
    result.ra = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
    if (result.rn == 15U || result.rm == 15U || result.rd == 15U || result.ra == 15U
        || result.rd == result.ra) {
        return std::nullopt;
    }
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeSmull(
    const std::uint16_t first,
    const std::uint16_t second
) {
    auto result = decodeUmull(first, second);
    if (result) result->kind = InstrKind::smull;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeUdiv(
    const std::uint16_t first,
    const std::uint16_t second
) {
    auto result = base32(first, second, InstrKind::udiv, OperandForm::register_value);
    result.rn = static_cast<std::uint8_t>(first & 0x0fU);
    result.rd = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
    result.rm = static_cast<std::uint8_t>(second & 0x0fU);
    if (result.rn == 15U || result.rd == 15U || result.rm == 15U) return std::nullopt;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeSdiv(
    const std::uint16_t first,
    const std::uint16_t second
) {
    auto result = decodeUdiv(first, second);
    if (result) result->kind = InstrKind::sdiv;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeParallelAddSelect(
    const std::uint16_t first,
    const std::uint16_t second
) {
    const bool select = (first & 0x0020U) != 0U;
    auto result = base32(
        first, second, select ? InstrKind::sel : InstrKind::uadd8,
        OperandForm::register_value
    );
    result.rn = static_cast<std::uint8_t>(first & 0x0fU);
    result.rd = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
    result.rm = static_cast<std::uint8_t>(second & 0x0fU);
    if (result.rn == 15U || result.rd == 15U || result.rm == 15U) return std::nullopt;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeClz(
    const std::uint16_t first,
    const std::uint16_t second
) {
    const auto rm = static_cast<std::uint8_t>(first & 0x0fU);
    if ((second & 0x0fU) != rm) return std::nullopt;
    auto result = base32(first, second, InstrKind::clz, OperandForm::register_value);
    result.rd = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
    result.rm = rm;
    if (result.rd == 15U || result.rm == 15U) return std::nullopt;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeUbfx(
    const std::uint16_t first,
    const std::uint16_t second
) {
    const auto lsb = static_cast<std::uint8_t>(
        ((second >> 10U) & 0x1cU) | ((second >> 6U) & 0x3U)
    );
    const auto width = static_cast<std::uint8_t>((second & 0x1fU) + 1U);
    if (static_cast<unsigned int>(lsb) + width > 32U) return std::nullopt;
    auto result = base32(first, second, InstrKind::ubfx, OperandForm::immediate);
    result.rn = static_cast<std::uint8_t>(first & 0x0fU);
    result.rd = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
    result.shift_amount = lsb;
    result.imm = width;
    if (result.rn == 15U || result.rd == 15U) return std::nullopt;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeMrs(
    const std::uint16_t first,
    const std::uint16_t second
) {
    auto result = base32(first, second, InstrKind::mrs, OperandForm::immediate);
    result.rd = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
    result.imm = second & 0xffU;
    const bool supported = result.imm == 5U || result.imm == 8U || result.imm == 9U
        || (result.imm >= 16U && result.imm <= 20U);
    if (result.rd == 15U || !supported) return std::nullopt;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeMsr(
    const std::uint16_t first,
    const std::uint16_t second
) {
    auto result = base32(first, second, InstrKind::msr, OperandForm::register_value);
    result.rn = static_cast<std::uint8_t>(first & 0x0fU);
    result.imm = second & 0xffU;
    const bool supported = result.imm == 8U || result.imm == 9U
        || (result.imm >= 16U && result.imm <= 20U);
    if (result.rn == 15U || !supported) return std::nullopt;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeBarrier(
    const std::uint16_t first,
    const std::uint16_t second
) {
    InstrKind kind = InstrKind::undefined;
    if ((second & 0x00f0U) == 0x0040U) kind = InstrKind::dsb;
    if ((second & 0x00f0U) == 0x0050U) kind = InstrKind::dmb;
    if ((second & 0x00f0U) == 0x0060U) kind = InstrKind::isb;
    if (kind == InstrKind::undefined) return std::nullopt;
    auto result = base32(first, second, kind, OperandForm::immediate);
    result.imm = second & 0x0fU;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeNopWide(
    const std::uint16_t first,
    const std::uint16_t second
) {
    return base32(first, second, InstrKind::nop);
}

[[nodiscard]] std::optional<DecodedInstruction> decodeBranchOrSystem(
    const std::uint16_t first,
    const std::uint16_t second
) {
    const std::uint32_t raw = (static_cast<std::uint32_t>(first) << 16U) | second;
    if ((raw & 0xfffff000U) == 0xf3ef8000U) return decodeMrs(first, second);
    if ((raw & 0xfff0ff00U) == 0xf3808800U) return decodeMsr(first, second);
    if ((raw & 0xfffffff0U) == 0xf3bf8f40U
        || (raw & 0xfffffff0U) == 0xf3bf8f50U
        || (raw & 0xfffffff0U) == 0xf3bf8f60U) {
        return decodeBarrier(first, second);
    }
    if (raw == 0xf3af8000U) return decodeNopWide(first, second);
    return decodeBranchWide(first, second);
}

[[nodiscard]] std::optional<DecodedInstruction> decodeBranchWide(
    const std::uint16_t first,
    const std::uint16_t second
) {
    const bool link = (second & 0x4000U) != 0;
    const bool conditional = (second & 0x1000U) == 0;
    auto result = base32(first, second, link ? InstrKind::bl : InstrKind::b, OperandForm::immediate);
    const std::uint32_t s = (first >> 10U) & 1U;
    const std::uint32_t j1 = (second >> 13U) & 1U;
    const std::uint32_t j2 = (second >> 11U) & 1U;
    if (conditional) {
        const auto condition = static_cast<std::uint8_t>((first >> 6U) & 0x0fU);
        if (condition >= static_cast<std::uint8_t>(Condition::al) || link) return std::nullopt;
        result.condition = static_cast<Condition>(condition);
        const std::uint32_t encoded = (s << 20U) | (j2 << 19U) | (j1 << 18U)
            | (static_cast<std::uint32_t>(first & 0x003fU) << 12U)
            | (static_cast<std::uint32_t>(second & 0x07ffU) << 1U);
        result.branch_offset = signExtend<21>(encoded);
        return result;
    }

    const std::uint32_t i1 = (~(j1 ^ s)) & 1U;
    const std::uint32_t i2 = (~(j2 ^ s)) & 1U;
    const std::uint32_t encoded = (s << 24U) | (i1 << 23U) | (i2 << 22U)
        | (static_cast<std::uint32_t>(first & 0x03ffU) << 12U)
        | (static_cast<std::uint32_t>(second & 0x07ffU) << 1U);
    result.branch_offset = signExtend<25>(encoded);
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeWideTransfer(
    const std::uint16_t first,
    const std::uint16_t second
) {
    InstrKind kind = InstrKind::undefined;
    const std::uint16_t op = first & 0xfff0U;
    const bool literal = (first & 0xff7fU) == 0xf85fU;
    const bool literal_negative = literal && (first & 0x0080U) == 0U;
    if (literal) {
        kind = InstrKind::ldr;
    } else {
        switch (op) {
        case 0xf8c0U: kind = InstrKind::str; break;
        case 0xf8d0U: kind = InstrKind::ldr; break;
        case 0xf880U: kind = InstrKind::strb; break;
        case 0xf890U: kind = InstrKind::ldrb; break;
        case 0xf8a0U: kind = InstrKind::strh; break;
        case 0xf8b0U: kind = InstrKind::ldrh; break;
        case 0xf990U: kind = InstrKind::ldrsb; break;
        case 0xf9b0U: kind = InstrKind::ldrsh; break;
        case 0xf840U: kind = InstrKind::str; break;
        case 0xf850U: kind = InstrKind::ldr; break;
        case 0xf800U: kind = InstrKind::strb; break;
        case 0xf810U: kind = InstrKind::ldrb; break;
        case 0xf820U: kind = InstrKind::strh; break;
        case 0xf830U: kind = InstrKind::ldrh; break;
        case 0xf910U: kind = InstrKind::ldrsb; break;
        case 0xf930U: kind = InstrKind::ldrsh; break;
        default: return std::nullopt;
        }
    }

    auto result = base32(first, second, kind, OperandForm::immediate);
    result.rn = static_cast<std::uint8_t>(first & 0x0fU);
    result.rd = static_cast<std::uint8_t>((second >> 12U) & 0x0fU);
    result.imm = second & 0x0fffU;

    if (literal || (result.rn == 15U && kind != InstrKind::str
        && kind != InstrKind::strb && kind != InstrKind::strh)) {
        result.form = OperandForm::literal;
        result.rn = 15U;
        result.add = !literal_negative;
        result.imm = second & 0x0fffU;
    } else if (op == 0xf840U || op == 0xf850U || op == 0xf800U || op == 0xf810U
        || op == 0xf820U || op == 0xf830U || op == 0xf910U || op == 0xf930U) {
        if ((second & 0x0800U) != 0) {
            result.form = OperandForm::immediate;
            result.index = (second & 0x0400U) != 0;
            result.add = (second & 0x0200U) != 0;
            result.writeback = (second & 0x0100U) != 0;
            result.imm = second & 0x00ffU;
            if (!result.index && !result.writeback) return std::nullopt;
        } else {
            if ((second & 0x0fc0U) != 0U) return std::nullopt;
            result.form = OperandForm::register_value;
            result.rm = static_cast<std::uint8_t>(second & 0x0fU);
            result.shift_type = ShiftType::lsl;
            result.shift_amount = static_cast<std::uint8_t>((second >> 4U) & 0x3U);
            result.imm = 0;
        }
    }

    const bool store = kind == InstrKind::str || kind == InstrKind::strb || kind == InstrKind::strh;
    if ((store && (result.rn == 15U || result.rd == 15U))
        || (!store && result.rd == 15U && kind != InstrKind::ldr)) {
        return std::nullopt;
    }
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeWideMultiple(
    const std::uint16_t first,
    const std::uint16_t second
) {
    if ((first & 0xfe40U) == 0xe840U) {
        auto result = base32(
            first, second,
            (first & 0x0010U) != 0U ? InstrKind::ldrd : InstrKind::strd,
            OperandForm::immediate
        );
        result.rn = static_cast<std::uint8_t>(first & 0x0fU);
        result.rd = static_cast<std::uint8_t>((second >> 12U) & 0x0fU);
        result.ra = static_cast<std::uint8_t>((second >> 8U) & 0x0fU);
        result.imm = (second & 0xffU) << 2U;
        result.index = (first & 0x0100U) != 0U;
        result.add = (first & 0x0080U) != 0U;
        result.writeback = (first & 0x0020U) != 0U;
        if ((!result.index && !result.writeback) || result.rn == 15U
            || result.rd == 15U || result.ra == 15U || result.rd == result.ra
            || (result.writeback && (result.rn == result.rd || result.rn == result.ra))) {
            return std::nullopt;
        }
        return result;
    }

    const std::uint16_t operation = first & 0xffd0U;
    const bool load = operation == 0xe890U || operation == 0xe910U;
    const bool decrement_before = operation == 0xe900U || operation == 0xe910U;
    if (operation != 0xe880U && operation != 0xe890U
        && operation != 0xe900U && operation != 0xe910U) {
        return std::nullopt;
    }
    const auto rn = static_cast<std::uint8_t>(first & 0x0fU);
    const bool writeback = (first & 0x0020U) != 0;
    if (rn == 15U || second == 0U || (second & (std::uint16_t{1} << 13U)) != 0U) {
        return std::nullopt;
    }
    if (!load && (second & (std::uint16_t{1} << 15U)) != 0U) return std::nullopt;

    InstrKind kind = load ? InstrKind::ldm : InstrKind::stm;
    if (rn == 13U && writeback && decrement_before && !load) kind = InstrKind::push;
    if (rn == 13U && writeback && !decrement_before && load) kind = InstrKind::pop;
    auto result = base32(first, second, kind, OperandForm::register_list);
    result.rn = rn;
    result.register_list = second;
    result.writeback = writeback;
    result.add = !decrement_before;
    result.index = decrement_before;
    return result;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeVfpMultiple(
    const std::uint16_t first,
    const std::uint16_t second
) {
    const bool single_precision = (second & 0x0f00U) == 0x0a00U;
    const bool double_precision = (second & 0x0f00U) == 0x0b00U;
    if (!single_precision && !double_precision) return std::nullopt;

    const bool pre_index = (first & 0x0100U) != 0U;
    const bool writeback = (first & 0x0020U) != 0U;
    const bool d_bit = (first & 0x0040U) != 0U;
    if (pre_index && !writeback) {
        auto result = base32(
            first, second,
            (first & 0x0010U) != 0U ? InstrKind::vldr : InstrKind::vstr,
            OperandForm::immediate
        );
        result.rn = static_cast<std::uint8_t>(first & 0x0fU);
        result.add = (first & 0x0080U) != 0U;
        result.imm = (second & 0xffU) << 2U;
        result.fp_double = double_precision;
        if (single_precision) {
            result.rd = static_cast<std::uint8_t>(
                (((second >> 12U) & 0x0fU) << 1U) | (d_bit ? 1U : 0U)
            );
        } else {
            if (d_bit) return std::nullopt;
            result.rd = static_cast<std::uint8_t>(((second >> 12U) & 0x0fU) << 1U);
        }
        if (result.rn == 15U) result.form = OperandForm::literal;
        if (result.rd + (result.fp_double ? 2U : 1U) > 32U) return std::nullopt;
        return result;
    }

    const bool load = (first & 0x0010U) != 0U;
    const bool increment_after = (first & 0x0180U) == 0x0080U;
    const bool decrement_before = (first & 0x0180U) == 0x0100U;
    if (!increment_after && !decrement_before) return std::nullopt;

    auto result = base32(
        first, second, load ? InstrKind::vldm : InstrKind::vstm,
        OperandForm::register_list
    );
    result.rn = static_cast<std::uint8_t>(first & 0x0fU);
    if (single_precision) {
        result.rd = static_cast<std::uint8_t>(
            (((second >> 12U) & 0x0fU) << 1U) | (d_bit ? 1U : 0U)
        );
    } else {
        if (d_bit) return std::nullopt;
        result.rd = static_cast<std::uint8_t>(((second >> 12U) & 0x0fU) << 1U);
        result.fp_double = true;
    }
    result.imm = second & 0xffU;
    result.add = increment_after;
    result.index = decrement_before;
    result.writeback = (first & 0x0020U) != 0U;
    if (result.rn == 15U || result.imm == 0U
        || result.rd + result.imm > 32U) {
        return std::nullopt;
    }
    return result;
}

[[nodiscard]] std::uint8_t vfpSd(
    const std::uint16_t first,
    const std::uint16_t second
) noexcept {
    return static_cast<std::uint8_t>(
        (((second >> 12U) & 0x0fU) << 1U) | ((first >> 6U) & 0x1U)
    );
}

[[nodiscard]] std::uint8_t vfpSn(
    const std::uint16_t first,
    const std::uint16_t second
) noexcept {
    return static_cast<std::uint8_t>(
        ((first & 0x0fU) << 1U) | ((second >> 7U) & 0x1U)
    );
}

[[nodiscard]] std::uint8_t vfpSm(const std::uint16_t second) noexcept {
    return static_cast<std::uint8_t>(
        ((second & 0x0fU) << 1U) | ((second >> 5U) & 0x1U)
    );
}

[[nodiscard]] std::uint32_t vfpExpandImmediate(const std::uint8_t imm8) noexcept {
    const std::uint32_t sign = static_cast<std::uint32_t>(imm8 & 0x80U) << 24U;
    const bool exponent_bit = (imm8 & 0x40U) != 0U;
    const std::uint32_t exponent_head = exponent_bit ? 0U : 0x40000000U;
    const std::uint32_t exponent_repeated = exponent_bit ? 0x3e000000U : 0U;
    const std::uint32_t fraction = static_cast<std::uint32_t>(imm8 & 0x3fU) << 19U;
    return sign | exponent_head | exponent_repeated | fraction;
}

[[nodiscard]] std::optional<DecodedInstruction> decodeVfpDataProcessing(
    const std::uint16_t first,
    const std::uint16_t second
) {
    if ((first & 0xffe0U) == 0xee00U && (second & 0x0f7fU) == 0x0a10U) {
        const bool to_core = (first & 0x0010U) != 0U;
        const auto core_register = static_cast<std::uint8_t>((second >> 12U) & 0x0fU);
        const auto single_register = static_cast<std::uint8_t>(
            ((first & 0x0fU) << 1U) | ((second >> 7U) & 0x1U)
        );
        if (core_register == 15U) return std::nullopt;
        auto result = base32(
            first, second,
            to_core ? InstrKind::vmov_single_to_core : InstrKind::vmov_core_to_single,
            OperandForm::register_value
        );
        result.rd = to_core ? core_register : single_register;
        result.rn = to_core ? single_register : core_register;
        return result;
    }

    if (first == 0xeef1U && (second & 0x0fffU) == 0x0a10U) {
        if ((second >> 12U) != 15U) return std::nullopt;
        return base32(first, second, InstrKind::vmrs);
    }

    const std::uint16_t normalized_first = first & 0xffbfU;
    const std::uint16_t binary_first = first & 0xffb0U;
    if (binary_first == 0xeeb0U && (second & 0x0ff0U) == 0x0a00U) {
        auto result = base32(first, second, InstrKind::vmov_immediate, OperandForm::immediate);
        result.rd = vfpSd(first, second);
        const auto imm8 = static_cast<std::uint8_t>(((first & 0x0fU) << 4U) | (second & 0x0fU));
        result.imm = vfpExpandImmediate(imm8);
        return result;
    }

    const std::uint16_t unary_shape = second & 0x0fd0U;
    if (unary_shape == 0x0a40U || unary_shape == 0x0ac0U) {
        InstrKind kind = InstrKind::undefined;
        if (normalized_first == 0xeeb0U) {
            kind = unary_shape == 0x0ac0U ? InstrKind::vabs : InstrKind::vmov_single;
        }
        if (normalized_first == 0xeeb1U) {
            kind = unary_shape == 0x0ac0U ? InstrKind::vsqrt : InstrKind::vneg;
        }
        if (normalized_first == 0xeeb8U) {
            kind = unary_shape == 0x0ac0U
                ? InstrKind::vcvt_f32_s32 : InstrKind::vcvt_f32_u32;
        }
        if (normalized_first == 0xeebcU && unary_shape == 0x0ac0U) {
            kind = InstrKind::vcvt_u32_f32;
        }
        if (normalized_first == 0xeebdU && unary_shape == 0x0ac0U) {
            kind = InstrKind::vcvt_s32_f32;
        }
        if (kind != InstrKind::undefined) {
            auto result = base32(first, second, kind, OperandForm::register_value);
            result.rd = vfpSd(first, second);
            result.rm = vfpSm(second);
            return result;
        }
    }

    if ((normalized_first == 0xeeb4U || normalized_first == 0xeeb5U)
        && (second & 0x0f50U) == 0x0a40U) {
        auto result = base32(
            first, second, InstrKind::vcmp,
            normalized_first == 0xeeb5U ? OperandForm::immediate : OperandForm::register_value
        );
        result.rn = vfpSd(first, second);
        result.rm = vfpSm(second);
        return result;
    }

    InstrKind binary_kind = InstrKind::undefined;
    if (binary_first == 0xee30U) {
        binary_kind = (second & 0x0040U) != 0U ? InstrKind::vsub : InstrKind::vadd;
    } else if (binary_first == 0xee20U) {
        binary_kind = (second & 0x0040U) != 0U ? InstrKind::vnmul : InstrKind::vmul;
    } else if (binary_first == 0xee80U) {
        binary_kind = InstrKind::vdiv;
    } else if (binary_first == 0xeea0U) {
        binary_kind = (second & 0x0040U) != 0U ? InstrKind::vfms : InstrKind::vfma;
    } else if (binary_first == 0xee90U && (second & 0x0040U) == 0U) {
        binary_kind = InstrKind::vfnms;
    }
    if (binary_kind != InstrKind::undefined && (second & 0x0f10U) == 0x0a00U) {
        auto result = base32(first, second, binary_kind, OperandForm::register_value);
        result.rd = vfpSd(first, second);
        result.rn = vfpSn(first, second);
        result.rm = vfpSm(second);
        return result;
    }

    return std::nullopt;
}

constexpr std::array<DecodePattern16, 18> patterns16 = {{
    {0xf800U, 0x0000U, decodeShiftImmediate},
    {0xf800U, 0x0800U, decodeShiftImmediate},
    {0xf800U, 0x1000U, decodeShiftImmediate},
    {0xf800U, 0x1800U, decodeAddSubtract},
    {0xe000U, 0x2000U, decodeImmediate8},
    {0xfc00U, 0x4000U, decodeDataProcessing},
    {0xfc00U, 0x4400U, decodeSpecialData},
    {0xf800U, 0x4800U, decodeLiteralLoad},
    {0xf000U, 0x5000U, decodeRegisterTransfer},
    {0xf000U, 0x6000U, decodeImmediateTransfer},
    {0xf000U, 0x7000U, decodeImmediateTransfer},
    {0xf000U, 0x8000U, decodeImmediateTransfer},
    {0xf000U, 0x9000U, decodeSpTransfer},
    {0xf000U, 0xa000U, decodeAddressGeneration},
    {0xf000U, 0xb000U, decodeMiscellaneous},
    {0xf000U, 0xc000U, decodeMultiple},
    {0xf000U, 0xd000U, decodeConditionalBranch},
    {0xf800U, 0xe000U, decodeUnconditionalBranch},
}};

constexpr std::array<DecodePattern32, 25> patterns32 = {{
    {0xfbf08000U, 0xf2000000U, decodeAddSubWideImmediate},
    {0xfbf08000U, 0xf2a00000U, decodeAddSubWideImmediate},
    {0xfbf08000U, 0xf2400000U, decodeMovWide},
    {0xfbf08000U, 0xf2c00000U, decodeMovWide},
    {0xfa008000U, 0xf0000000U, decodeDataProcessingImmediate},
    {0xf800d000U, 0xf0008000U, decodeBranchOrSystem},
    {0xf800d000U, 0xf0009000U, decodeBranchWide},
    {0xf800d000U, 0xf000d000U, decodeBranchWide},
    {0xfe008000U, 0xea000000U, decodeDataProcessingRegister},
    {0xff80f0f0U, 0xfa00f000U, decodeRegisterShift},
    {0xff8ff0c0U, 0xfa0ff080U, decodeExtendWide},
    {0xfff000f0U, 0xfb000000U, decodeMultiply},
    {0xfff000f0U, 0xfb000010U, decodeMultiply},
    {0xfff000f0U, 0xfb800000U, decodeSmull},
    {0xfff000f0U, 0xfba00000U, decodeUmull},
    {0xfff0f0f0U, 0xfb90f0f0U, decodeSdiv},
    {0xfff0f0f0U, 0xfbb0f0f0U, decodeUdiv},
    {0xfff0f0f0U, 0xfa80f040U, decodeParallelAddSelect},
    {0xfff0f0f0U, 0xfaa0f080U, decodeParallelAddSelect},
    {0xfff0f0f0U, 0xfab0f080U, decodeClz},
    {0xfff08020U, 0xf3c00000U, decodeUbfx},
    {0xfe000000U, 0xf8000000U, decodeWideTransfer},
    {0xfe000000U, 0xe8000000U, decodeWideMultiple},
    {0xfe000000U, 0xec000000U, decodeVfpMultiple},
    {0xff000000U, 0xee000000U, decodeVfpDataProcessing},
}};

template <typename Pattern>
[[nodiscard]] bool tableHasNoOverlaps(const std::span<const Pattern> table) noexcept {
    for (std::size_t left = 0; left < table.size(); ++left) {
        if ((table[left].value & table[left].mask) != table[left].value
            || table[left].decode == nullptr) {
            return false;
        }
        for (std::size_t right = left + 1U; right < table.size(); ++right) {
            const auto common = table[left].mask & table[right].mask;
            if ((table[left].value & common) == (table[right].value & common)) {
                return false;
            }
        }
    }
    return true;
}

} // namespace

std::optional<DecodedInstruction> decode16(const std::uint16_t halfword) noexcept {
    if (is32BitThumbPrefix(halfword)) return std::nullopt;
    for (const auto& pattern : patterns16) {
        if ((halfword & pattern.mask) == pattern.value) return pattern.decode(halfword);
    }
    return std::nullopt;
}

std::optional<DecodedInstruction> decode32(
    const std::uint16_t first,
    const std::uint16_t second
) noexcept {
    if (!is32BitThumbPrefix(first)) return std::nullopt;
    const std::uint32_t raw = (static_cast<std::uint32_t>(first) << 16U) | second;
    for (const auto& pattern : patterns32) {
        if ((raw & pattern.mask) == pattern.value) return pattern.decode(first, second);
    }
    return std::nullopt;
}

std::span<const DecodePattern16> decodePatterns16() noexcept {
    return patterns16;
}

std::span<const DecodePattern32> decodePatterns32() noexcept {
    return patterns32;
}

bool decoderTablesHaveNoOverlaps() noexcept {
    return tableHasNoOverlaps<DecodePattern16>(patterns16)
        && tableHasNoOverlaps<DecodePattern32>(patterns32);
}

} // namespace fil::cpu
