#include "fil/cpu/cortex_m4.hpp"

#include "fil/mem/memory_bus.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string_view>

namespace fil::cpu {
namespace {

[[nodiscard]] bool carryFlag(const CpuState& state) noexcept {
    return (state.xpsr & xpsr_c) != 0;
}

void setNz(CpuState& state, const std::uint32_t value) noexcept {
    state.xpsr &= ~(xpsr_n | xpsr_z);
    if ((value & 0x80000000U) != 0) state.xpsr |= xpsr_n;
    if (value == 0) state.xpsr |= xpsr_z;
}

void setNzc(CpuState& state, const std::uint32_t value, const bool carry) noexcept {
    setNz(state, value);
    state.xpsr &= ~xpsr_c;
    if (carry) state.xpsr |= xpsr_c;
}

void setNzcv(CpuState& state, const AddResult& result) noexcept {
    state.xpsr &= ~(xpsr_n | xpsr_z | xpsr_c | xpsr_v);
    if (result.n) state.xpsr |= xpsr_n;
    if (result.z) state.xpsr |= xpsr_z;
    if (result.c) state.xpsr |= xpsr_c;
    if (result.v) state.xpsr |= xpsr_v;
}

[[nodiscard]] bool isStore(const InstrKind kind) noexcept {
    return kind == InstrKind::str || kind == InstrKind::strb || kind == InstrKind::strh
        || kind == InstrKind::strd;
}

[[nodiscard]] std::uint32_t branchTarget(
    const CpuState& state,
    const std::int32_t offset
) noexcept {
    const auto wide = static_cast<std::int64_t>(state.currentInstrAddr()) + 4 + offset;
    return static_cast<std::uint32_t>(wide);
}

[[nodiscard]] std::uint32_t floatToSignedIntegerBits(const float value) noexcept {
    if (!std::isfinite(value)) return 0x80000000U;
    const double truncated = std::trunc(static_cast<double>(value));
    if (truncated < static_cast<double>(std::numeric_limits<std::int32_t>::min())
        || truncated > static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
        return 0x80000000U;
    }
    return std::bit_cast<std::uint32_t>(static_cast<std::int32_t>(truncated));
}

[[nodiscard]] std::uint32_t floatToUnsignedIntegerBits(const float value) noexcept {
    if (!std::isfinite(value) || value <= 0.0F) return 0U;
    const double truncated = std::trunc(static_cast<double>(value));
    if (truncated >= static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
        return std::numeric_limits<std::uint32_t>::max();
    }
    return static_cast<std::uint32_t>(truncated);
}

} // namespace

AddResult addWithCarry(
    const std::uint32_t x,
    const std::uint32_t y,
    const bool carry_in
) noexcept {
    const std::uint64_t unsigned_sum = static_cast<std::uint64_t>(x)
        + static_cast<std::uint64_t>(y) + (carry_in ? 1U : 0U);
    const std::uint32_t value = static_cast<std::uint32_t>(unsigned_sum);
    const bool overflow = ((~(x ^ y) & (x ^ value)) & 0x80000000U) != 0;
    return AddResult{
        value,
        (value & 0x80000000U) != 0,
        value == 0,
        (unsigned_sum >> 32U) != 0,
        overflow,
    };
}

ShiftResult shiftC(
    const std::uint32_t value,
    const ShiftType type,
    const std::uint32_t amount,
    const bool old_carry
) noexcept {
    switch (type) {
    case ShiftType::lsl:
        if (amount == 0U) return {value, old_carry};
        if (amount < 32U) return {value << amount, ((value >> (32U - amount)) & 1U) != 0};
        if (amount == 32U) return {0, (value & 1U) != 0};
        return {0, false};
    case ShiftType::lsr:
        if (amount == 0U) return {value, old_carry};
        if (amount < 32U) return {value >> amount, ((value >> (amount - 1U)) & 1U) != 0};
        if (amount == 32U) return {0, (value & 0x80000000U) != 0};
        return {0, false};
    case ShiftType::asr:
        if (amount == 0U) return {value, old_carry};
        if (amount < 32U) {
            const std::uint32_t shifted = value >> amount;
            const std::uint32_t fill = (value & 0x80000000U) != 0
                ? (~std::uint32_t{0} << (32U - amount)) : 0U;
            return {shifted | fill, ((value >> (amount - 1U)) & 1U) != 0};
        }
        return {
            (value & 0x80000000U) != 0 ? 0xffffffffU : 0U,
            (value & 0x80000000U) != 0,
        };
    case ShiftType::ror: {
        if (amount == 0U) return {value, old_carry};
        const std::uint32_t rotation = amount & 31U;
        if (rotation == 0U) return {value, (value & 0x80000000U) != 0};
        const std::uint32_t shifted = (value >> rotation) | (value << (32U - rotation));
        return {shifted, (shifted & 0x80000000U) != 0};
    }
    case ShiftType::rrx:
        return {
            (old_carry ? 0x80000000U : 0U) | (value >> 1U),
            (value & 1U) != 0,
        };
    }
    return {value, old_carry};
}

bool conditionPasses(const Condition condition, const std::uint32_t xpsr) noexcept {
    const bool n = (xpsr & xpsr_n) != 0;
    const bool z = (xpsr & xpsr_z) != 0;
    const bool c = (xpsr & xpsr_c) != 0;
    const bool v = (xpsr & xpsr_v) != 0;
    switch (condition) {
    case Condition::eq: return z;
    case Condition::ne: return !z;
    case Condition::cs: return c;
    case Condition::cc: return !c;
    case Condition::mi: return n;
    case Condition::pl: return !n;
    case Condition::vs: return v;
    case Condition::vc: return !v;
    case Condition::hi: return c && !z;
    case Condition::ls: return !c || z;
    case Condition::ge: return n == v;
    case Condition::lt: return n != v;
    case Condition::gt: return !z && n == v;
    case Condition::le: return z || n != v;
    case Condition::al: return true;
    case Condition::nv: return false;
    }
    return false;
}

std::uint8_t advanceItState(const std::uint8_t it_state) noexcept {
    if ((it_state & 0x07U) == 0U) return 0;
    const std::uint32_t widened = it_state;
    return static_cast<std::uint8_t>((widened & 0xe0U) | ((widened << 1U) & 0x1fU));
}

StopReason CortexM4::execute(
    const DecodedInstruction& instruction,
    DiagnosticSnapshot& diagnostic
) {
    const auto failInvalid = [&](const std::string_view message) {
        state_.halted = true;
        diagnostic.bus_fault.reset();
        diagnostic.message = message;
        return StopReason::invalid_state;
    };
    const auto failBus = [&](const mem::BusFault& fault, const std::string_view message) {
        diagnostic.bus_fault = fault;
        diagnostic.message = message;
        return fault.reason == mem::BusFaultReason::synchronization_required
            ? StopReason::synchronization_required : StopReason::bus_fault;
    };
    const auto writeAluResult = [&](const std::uint8_t rd, const std::uint32_t value) {
        if (rd == 15U) {
            state_.r[15] = value & ~std::uint32_t{1};
        } else {
            state_.writeRegister(rd, value);
        }
    };
    const auto shiftedOperand = [&]() {
        if (instruction.form == OperandForm::immediate) {
            return ShiftResult{
                instruction.imm,
                instruction.immediate_carry_valid
                    ? instruction.immediate_carry : carryFlag(state_),
            };
        }
        return shiftC(
            state_.readRegister(instruction.rm), instruction.shift_type,
            instruction.shift_amount, carryFlag(state_)
        );
    };
    const auto operand2 = [&]() {
        return shiftedOperand().value;
    };

    switch (instruction.kind) {
    case InstrKind::mov: {
        const ShiftResult shifted = shiftedOperand();
        const std::uint32_t value = shifted.value;
        if (instruction.rd == 15U) {
            if (!state_.branchWritePc(value)) return failInvalid("MOV to PC selected non-Thumb state");
        } else {
            state_.writeRegister(instruction.rd, value);
        }
        if (instruction.set_flags) setNzc(state_, value, shifted.carry);
        return StopReason::step_complete;
    }
    case InstrKind::movw:
        state_.writeRegister(instruction.rd, instruction.imm);
        return StopReason::step_complete;
    case InstrKind::movt: {
        const std::uint32_t value = (state_.readRegister(instruction.rd) & 0x0000ffffU)
            | (instruction.imm << 16U);
        state_.writeRegister(instruction.rd, value);
        return StopReason::step_complete;
    }
    case InstrKind::add:
    case InstrKind::adc:
    case InstrKind::sub:
    case InstrKind::sbc: {
        std::uint32_t left = state_.readRegister(instruction.rn);
        if (instruction.rn == 15U && instruction.form == OperandForm::immediate) {
            left &= ~std::uint32_t{3};
        }
        const std::uint32_t right = operand2();
        const bool subtract = instruction.kind == InstrKind::sub || instruction.kind == InstrKind::sbc;
        const bool carry_in = instruction.kind == InstrKind::adc
            ? carryFlag(state_) : instruction.kind == InstrKind::sbc ? carryFlag(state_) : subtract;
        const AddResult result = addWithCarry(left, subtract ? ~right : right, carry_in);
        writeAluResult(instruction.rd, result.value);
        if (instruction.set_flags) setNzcv(state_, result);
        return StopReason::step_complete;
    }
    case InstrKind::rsb: {
        const std::uint32_t left = instruction.form == OperandForm::immediate
            ? instruction.imm : instruction.is_32bit ? operand2() : 0U;
        const std::uint32_t right = instruction.form == OperandForm::immediate
            ? state_.readRegister(instruction.rn)
            : instruction.is_32bit ? state_.readRegister(instruction.rn)
                                   : state_.readRegister(instruction.rm);
        const AddResult result = addWithCarry(left, ~right, true);
        state_.writeRegister(instruction.rd, result.value);
        if (instruction.set_flags) setNzcv(state_, result);
        return StopReason::step_complete;
    }
    case InstrKind::cmp:
    case InstrKind::cmn: {
        const std::uint32_t left = state_.readRegister(instruction.rn);
        const std::uint32_t right = operand2();
        const AddResult result = instruction.kind == InstrKind::cmp
            ? addWithCarry(left, ~right, true) : addWithCarry(left, right, false);
        setNzcv(state_, result);
        return StopReason::step_complete;
    }
    case InstrKind::tst: {
        const ShiftResult shifted = shiftedOperand();
        const std::uint32_t value = state_.readRegister(instruction.rn) & shifted.value;
        setNzc(state_, value, shifted.carry);
        return StopReason::step_complete;
    }
    case InstrKind::and_:
    case InstrKind::orr:
    case InstrKind::eor:
    case InstrKind::bic:
    case InstrKind::mvn:
    case InstrKind::orn: {
        const std::uint32_t left = state_.readRegister(instruction.rn);
        const ShiftResult shifted = shiftedOperand();
        const std::uint32_t right = shifted.value;
        std::uint32_t value = 0;
        if (instruction.kind == InstrKind::and_) value = left & right;
        if (instruction.kind == InstrKind::orr) value = left | right;
        if (instruction.kind == InstrKind::eor) value = left ^ right;
        if (instruction.kind == InstrKind::bic) value = left & ~right;
        if (instruction.kind == InstrKind::mvn) value = ~right;
        if (instruction.kind == InstrKind::orn) value = left | ~right;
        state_.writeRegister(instruction.rd, value);
        if (instruction.set_flags) setNzc(state_, value, shifted.carry);
        return StopReason::step_complete;
    }
    case InstrKind::mul: {
        const std::uint64_t product = static_cast<std::uint64_t>(state_.readRegister(instruction.rn))
            * state_.readRegister(instruction.rm);
        const std::uint32_t value = static_cast<std::uint32_t>(product);
        state_.writeRegister(instruction.rd, value);
        if (instruction.set_flags) setNz(state_, value);
        return StopReason::step_complete;
    }
    case InstrKind::mla:
    case InstrKind::mls: {
        const std::uint32_t product = state_.readRegister(instruction.rn)
            * state_.readRegister(instruction.rm);
        const std::uint32_t addend = state_.readRegister(instruction.ra);
        const std::uint32_t value = instruction.kind == InstrKind::mla
            ? product + addend : addend - product;
        state_.writeRegister(instruction.rd, value);
        return StopReason::step_complete;
    }
    case InstrKind::umull: {
        const std::uint64_t product = static_cast<std::uint64_t>(
            state_.readRegister(instruction.rn)
        ) * state_.readRegister(instruction.rm);
        state_.writeRegister(instruction.rd, static_cast<std::uint32_t>(product));
        state_.writeRegister(instruction.ra, static_cast<std::uint32_t>(product >> 32U));
        return StopReason::step_complete;
    }
    case InstrKind::smull: {
        const std::int64_t product = static_cast<std::int64_t>(
            std::bit_cast<std::int32_t>(state_.readRegister(instruction.rn))
        ) * std::bit_cast<std::int32_t>(state_.readRegister(instruction.rm));
        const std::uint64_t bits = std::bit_cast<std::uint64_t>(product);
        state_.writeRegister(instruction.rd, static_cast<std::uint32_t>(bits));
        state_.writeRegister(instruction.ra, static_cast<std::uint32_t>(bits >> 32U));
        return StopReason::step_complete;
    }
    case InstrKind::udiv: {
        const std::uint32_t divisor = state_.readRegister(instruction.rm);
        const std::uint32_t value = divisor == 0U
            ? 0U : state_.readRegister(instruction.rn) / divisor;
        state_.writeRegister(instruction.rd, value);
        return StopReason::step_complete;
    }
    case InstrKind::sdiv: {
        const std::int32_t dividend = std::bit_cast<std::int32_t>(
            state_.readRegister(instruction.rn)
        );
        const std::int32_t divisor = std::bit_cast<std::int32_t>(
            state_.readRegister(instruction.rm)
        );
        std::int32_t quotient = 0;
        if (divisor != 0) {
            if (dividend == std::numeric_limits<std::int32_t>::min() && divisor == -1) {
                quotient = std::numeric_limits<std::int32_t>::min();
            } else {
                quotient = dividend / divisor;
            }
        }
        state_.writeRegister(instruction.rd, std::bit_cast<std::uint32_t>(quotient));
        return StopReason::step_complete;
    }
    case InstrKind::uadd8: {
        const std::uint32_t left = state_.readRegister(instruction.rn);
        const std::uint32_t right = state_.readRegister(instruction.rm);
        std::uint32_t value = 0U;
        std::uint32_t ge = 0U;
        for (std::uint32_t lane = 0U; lane < 4U; ++lane) {
            const std::uint32_t shift = lane * 8U;
            const std::uint32_t sum = ((left >> shift) & 0xffU)
                + ((right >> shift) & 0xffU);
            value |= (sum & 0xffU) << shift;
            if (sum >= 0x100U) ge |= std::uint32_t{1} << (16U + lane);
        }
        state_.writeRegister(instruction.rd, value);
        state_.xpsr = (state_.xpsr & ~xpsr_ge_mask) | ge;
        return StopReason::step_complete;
    }
    case InstrKind::sel: {
        const std::uint32_t left = state_.readRegister(instruction.rn);
        const std::uint32_t right = state_.readRegister(instruction.rm);
        std::uint32_t value = 0U;
        for (std::uint32_t lane = 0U; lane < 4U; ++lane) {
            const std::uint32_t shift = lane * 8U;
            const bool use_left = (state_.xpsr & (std::uint32_t{1} << (16U + lane))) != 0U;
            value |= ((use_left ? left : right) >> shift & 0xffU) << shift;
        }
        state_.writeRegister(instruction.rd, value);
        return StopReason::step_complete;
    }
    case InstrKind::clz:
        state_.writeRegister(
            instruction.rd,
            static_cast<std::uint32_t>(std::countl_zero(state_.readRegister(instruction.rm)))
        );
        return StopReason::step_complete;
    case InstrKind::ubfx: {
        const std::uint32_t width = instruction.imm;
        const std::uint32_t mask = width == 32U
            ? 0xffffffffU : (std::uint32_t{1} << width) - 1U;
        state_.writeRegister(
            instruction.rd,
            (state_.readRegister(instruction.rn) >> instruction.shift_amount) & mask
        );
        return StopReason::step_complete;
    }
    case InstrKind::uxtb: {
        const std::uint32_t source = std::rotr(
            state_.readRegister(instruction.rm), static_cast<int>(instruction.shift_amount)
        );
        state_.writeRegister(instruction.rd, source & 0xffU);
        return StopReason::step_complete;
    }
    case InstrKind::uxth: {
        const std::uint32_t source = std::rotr(
            state_.readRegister(instruction.rm), static_cast<int>(instruction.shift_amount)
        );
        state_.writeRegister(instruction.rd, source & 0xffffU);
        return StopReason::step_complete;
    }
    case InstrKind::sxtb: {
        std::uint32_t value = std::rotr(
            state_.readRegister(instruction.rm), static_cast<int>(instruction.shift_amount)
        ) & 0xffU;
        if ((value & 0x80U) != 0U) value |= 0xffffff00U;
        state_.writeRegister(instruction.rd, value);
        return StopReason::step_complete;
    }
    case InstrKind::sxth: {
        std::uint32_t value = std::rotr(
            state_.readRegister(instruction.rm), static_cast<int>(instruction.shift_amount)
        ) & 0xffffU;
        if ((value & 0x8000U) != 0U) value |= 0xffff0000U;
        state_.writeRegister(instruction.rd, value);
        return StopReason::step_complete;
    }
    case InstrKind::rev: {
        const std::uint32_t value = state_.readRegister(instruction.rm);
        state_.writeRegister(
            instruction.rd,
            ((value & 0x000000ffU) << 24U) | ((value & 0x0000ff00U) << 8U)
                | ((value & 0x00ff0000U) >> 8U) | ((value & 0xff000000U) >> 24U)
        );
        return StopReason::step_complete;
    }
    case InstrKind::rev16: {
        const std::uint32_t value = state_.readRegister(instruction.rm);
        state_.writeRegister(
            instruction.rd,
            ((value & 0x00ff00ffU) << 8U) | ((value & 0xff00ff00U) >> 8U)
        );
        return StopReason::step_complete;
    }
    case InstrKind::revsh: {
        const std::uint32_t source = state_.readRegister(instruction.rm);
        std::uint32_t value = ((source & 0xffU) << 8U) | ((source >> 8U) & 0xffU);
        if ((value & 0x8000U) != 0U) value |= 0xffff0000U;
        state_.writeRegister(instruction.rd, value);
        return StopReason::step_complete;
    }
    case InstrKind::lsl:
    case InstrKind::lsr:
    case InstrKind::asr:
    case InstrKind::ror:
    case InstrKind::rrx: {
        const bool immediate = instruction.form == OperandForm::immediate;
        const std::uint32_t value = immediate
            ? state_.readRegister(instruction.rm) : state_.readRegister(instruction.rn);
        const std::uint32_t amount = immediate ? instruction.shift_amount
            : (state_.readRegister(instruction.rm) & 0xffU);
        const ShiftResult result = shiftC(value, instruction.shift_type, amount, carryFlag(state_));
        state_.writeRegister(instruction.rd, result.value);
        if (instruction.set_flags) setNzc(state_, result.value, result.carry);
        return StopReason::step_complete;
    }
    case InstrKind::ldr: {
        const std::uint32_t base = instruction.rn == 15U
            ? state_.architecturalPcForRead() & ~std::uint32_t{3}
            : state_.readRegister(instruction.rn);
        std::uint32_t offset = instruction.imm;
        if (instruction.form == OperandForm::register_value) {
            offset = shiftC(
                state_.readRegister(instruction.rm), instruction.shift_type,
                instruction.shift_amount, carryFlag(state_)
            ).value;
        }
        const std::uint32_t offset_address = instruction.add
            ? base + offset : base - offset;
        const std::uint32_t address = instruction.index ? offset_address : base;
        const mem::AccessContext context{
            mem::AccessType::data_read, state_.currentInstrAddr()
        };
        const auto result = memory_.read32(address, context);
        if (!result) return failBus(result.fault(), "word load failed");
        if (instruction.rd == 15U) {
            if (!state_.branchWritePc(result.value())) {
                return failInvalid("load to PC selected non-Thumb state");
            }
        } else {
            state_.writeRegister(instruction.rd, result.value());
        }
        if (instruction.writeback) {
            state_.writeRegister(instruction.rn, offset_address);
        }
        return StopReason::step_complete;
    }
    case InstrKind::str:
    case InstrKind::ldrb:
    case InstrKind::strb:
    case InstrKind::ldrh:
    case InstrKind::strh:
    case InstrKind::ldrsb:
    case InstrKind::ldrsh: {
        std::uint32_t base = instruction.rn == 15U
            ? state_.architecturalPcForRead() & ~std::uint32_t{3}
            : state_.readRegister(instruction.rn);
        std::uint32_t offset = instruction.imm;
        if (instruction.form == OperandForm::register_value) {
            offset = shiftC(
                state_.readRegister(instruction.rm), instruction.shift_type,
                instruction.shift_amount, carryFlag(state_)
            ).value;
        }
        const std::uint32_t offset_address = instruction.add ? base + offset : base - offset;
        const std::uint32_t address = instruction.index ? offset_address : base;
        const mem::AccessContext context{
            isStore(instruction.kind) ? mem::AccessType::data_write : mem::AccessType::data_read,
            state_.currentInstrAddr(),
        };

        if (isStore(instruction.kind)) {
            const std::uint32_t value = state_.readRegister(instruction.rd);
            if (instruction.kind == InstrKind::str) {
                const auto result = memory_.write32(address, value, context);
                if (!result) return failBus(result.fault(), "word store failed");
            } else if (instruction.kind == InstrKind::strb) {
                const auto result = memory_.write8(address, static_cast<std::uint8_t>(value), context);
                if (!result) return failBus(result.fault(), "byte store failed");
            } else {
                const auto result = memory_.write16(address, static_cast<std::uint16_t>(value), context);
                if (!result) return failBus(result.fault(), "halfword store failed");
            }
        } else {
            std::uint32_t value = 0;
            if (instruction.kind == InstrKind::ldrb
                || instruction.kind == InstrKind::ldrsb) {
                const auto result = memory_.read8(address, context);
                if (!result) return failBus(result.fault(), "byte load failed");
                value = result.value();
                if (instruction.kind == InstrKind::ldrsb && (value & 0x80U) != 0) value |= 0xffffff00U;
            } else {
                const auto result = memory_.read16(address, context);
                if (!result) return failBus(result.fault(), "halfword load failed");
                value = result.value();
                if (instruction.kind == InstrKind::ldrsh && (value & 0x8000U) != 0) value |= 0xffff0000U;
            }

            if (instruction.rd == 15U) {
                if (!state_.branchWritePc(value)) return failInvalid("load to PC selected non-Thumb state");
            } else {
                state_.writeRegister(instruction.rd, value);
            }
        }
        if (instruction.writeback) state_.writeRegister(instruction.rn, offset_address);
        return StopReason::step_complete;
    }
    case InstrKind::ldrd:
    case InstrKind::strd: {
        const bool load = instruction.kind == InstrKind::ldrd;
        const std::uint32_t base = state_.readRegister(instruction.rn);
        const std::uint32_t offset_address = instruction.add
            ? base + instruction.imm : base - instruction.imm;
        const std::uint32_t address = instruction.index ? offset_address : base;
        const mem::AccessContext context{
            load ? mem::AccessType::data_read : mem::AccessType::data_write,
            state_.currentInstrAddr(),
        };
        if (load) {
            const auto first = memory_.read32(address, context);
            if (!first) return failBus(first.fault(), "first doubleword load failed");
            const auto second = memory_.read32(address + 4U, context);
            if (!second) return failBus(second.fault(), "second doubleword load failed");
            state_.writeRegister(instruction.rd, first.value());
            state_.writeRegister(instruction.ra, second.value());
        } else {
            const auto first = memory_.write32(
                address, state_.readRegister(instruction.rd), context
            );
            if (!first) return failBus(first.fault(), "first doubleword store failed");
            const auto second = memory_.write32(
                address + 4U, state_.readRegister(instruction.ra), context
            );
            if (!second) return failBus(second.fault(), "second doubleword store failed");
        }
        if (instruction.writeback) state_.writeRegister(instruction.rn, offset_address);
        return StopReason::step_complete;
    }
    case InstrKind::push:
    case InstrKind::pop:
    case InstrKind::ldm:
    case InstrKind::stm: {
        const bool load = instruction.kind == InstrKind::pop || instruction.kind == InstrKind::ldm;
        const std::uint32_t count = static_cast<std::uint32_t>(std::popcount(instruction.register_list));
        const std::uint32_t original_base = state_.readRegister(instruction.rn);
        const bool decrement_before = !instruction.add && instruction.index;
        const std::uint32_t start = decrement_before
            ? original_base - count * 4U : original_base;
        const mem::AccessContext context{
            load ? mem::AccessType::data_read : mem::AccessType::data_write,
            state_.currentInstrAddr(),
        };
        std::array<std::uint32_t, 16> loaded{};
        std::uint32_t address = start;
        for (std::uint8_t reg = 0; reg < 16U; ++reg) {
            if ((instruction.register_list & (std::uint16_t{1} << reg)) == 0U) continue;
            if (load) {
                const auto value = memory_.read32(address, context);
                if (!value) return failBus(value.fault(), "multiple-register load failed");
                loaded[reg] = value.value();
            } else {
                const auto stored = memory_.write32(address, state_.readRegister(reg), context);
                if (!stored) return failBus(stored.fault(), "multiple-register store failed");
            }
            address += 4U;
        }

        if (load) {
            for (std::uint8_t reg = 0; reg < 15U; ++reg) {
                if ((instruction.register_list & (std::uint16_t{1} << reg)) != 0U) {
                    state_.writeRegister(reg, loaded[reg]);
                }
            }
        }
        if (instruction.writeback) {
            const std::uint32_t final_base = decrement_before
                ? start : original_base + count * 4U;
            state_.writeRegister(instruction.rn, final_base);
        }
        if (load && (instruction.register_list & (std::uint16_t{1} << 15U)) != 0U) {
            if (isExceptionReturn(loaded[15])) {
                state_.pending_exc_return = loaded[15];
            } else if (!state_.branchWritePc(loaded[15])) {
                return failInvalid("POP/LDM selected non-Thumb state");
            }
        }
        return StopReason::step_complete;
    }
    case InstrKind::b:
        state_.r[15] = branchTarget(state_, instruction.branch_offset) & ~std::uint32_t{1};
        return StopReason::step_complete;
    case InstrKind::bl:
        state_.r[14] = state_.r[15] | 1U;
        state_.r[15] = branchTarget(state_, instruction.branch_offset) & ~std::uint32_t{1};
        return StopReason::step_complete;
    case InstrKind::bx:
    case InstrKind::blx: {
        const std::uint32_t target = state_.readRegister(instruction.rm);
        if (instruction.kind == InstrKind::blx) state_.r[14] = state_.r[15] | 1U;
        if (instruction.kind == InstrKind::bx && isExceptionReturn(target)) {
            state_.pending_exc_return = target;
            return StopReason::step_complete;
        }
        if (!state_.branchWritePc(target)) return failInvalid("branch exchange selected non-Thumb state");
        return StopReason::step_complete;
    }
    case InstrKind::cbz:
    case InstrKind::cbnz: {
        const bool zero = state_.readRegister(instruction.rn) == 0U;
        const bool take = instruction.kind == InstrKind::cbz ? zero : !zero;
        if (take) {
            state_.r[15] = state_.architecturalPcForRead() + instruction.imm;
            state_.r[15] &= ~std::uint32_t{1};
        }
        return StopReason::step_complete;
    }
    case InstrKind::vldr:
    case InstrKind::vstr: {
        const bool load = instruction.kind == InstrKind::vldr;
        const std::uint32_t base = instruction.rn == 15U
            ? state_.architecturalPcForRead() & ~std::uint32_t{3}
            : state_.readRegister(instruction.rn);
        const std::uint32_t address = instruction.add
            ? base + instruction.imm : base - instruction.imm;
        const std::uint32_t words = instruction.fp_double ? 2U : 1U;
        const mem::AccessContext context{
            load ? mem::AccessType::data_read : mem::AccessType::data_write,
            state_.currentInstrAddr(),
        };
        for (std::uint32_t word = 0; word < words; ++word) {
            const auto fp_register = static_cast<std::size_t>(instruction.rd) + word;
            if (load) {
                const auto value = memory_.read32(address + word * 4U, context);
                if (!value) return failBus(value.fault(), "VFP scalar load failed");
                state_.s[fp_register] = std::bit_cast<float>(value.value());
            } else {
                const auto stored = memory_.write32(
                    address + word * 4U,
                    std::bit_cast<std::uint32_t>(state_.s[fp_register]), context
                );
                if (!stored) return failBus(stored.fault(), "VFP scalar store failed");
            }
        }
        return StopReason::step_complete;
    }
    case InstrKind::vmov_core_to_single:
        state_.s[instruction.rd] = std::bit_cast<float>(state_.readRegister(instruction.rn));
        return StopReason::step_complete;
    case InstrKind::vmov_single_to_core:
        state_.writeRegister(
            instruction.rd, std::bit_cast<std::uint32_t>(state_.s[instruction.rn])
        );
        return StopReason::step_complete;
    case InstrKind::vmov_single:
        state_.s[instruction.rd] = state_.s[instruction.rm];
        return StopReason::step_complete;
    case InstrKind::vmov_immediate:
        state_.s[instruction.rd] = std::bit_cast<float>(instruction.imm);
        return StopReason::step_complete;
    case InstrKind::vcvt_f32_s32: {
        const std::int32_t value = std::bit_cast<std::int32_t>(
            std::bit_cast<std::uint32_t>(state_.s[instruction.rm])
        );
        state_.s[instruction.rd] = static_cast<float>(value);
        return StopReason::step_complete;
    }
    case InstrKind::vcvt_f32_u32:
        state_.s[instruction.rd] = static_cast<float>(
            std::bit_cast<std::uint32_t>(state_.s[instruction.rm])
        );
        return StopReason::step_complete;
    case InstrKind::vcvt_s32_f32:
        state_.s[instruction.rd] = std::bit_cast<float>(
            floatToSignedIntegerBits(state_.s[instruction.rm])
        );
        return StopReason::step_complete;
    case InstrKind::vcvt_u32_f32:
        state_.s[instruction.rd] = std::bit_cast<float>(
            floatToUnsignedIntegerBits(state_.s[instruction.rm])
        );
        return StopReason::step_complete;
    case InstrKind::vadd:
        state_.s[instruction.rd] = state_.s[instruction.rn] + state_.s[instruction.rm];
        return StopReason::step_complete;
    case InstrKind::vsub:
        state_.s[instruction.rd] = state_.s[instruction.rn] - state_.s[instruction.rm];
        return StopReason::step_complete;
    case InstrKind::vmul:
        state_.s[instruction.rd] = state_.s[instruction.rn] * state_.s[instruction.rm];
        return StopReason::step_complete;
    case InstrKind::vnmul:
        state_.s[instruction.rd] = -(state_.s[instruction.rn] * state_.s[instruction.rm]);
        return StopReason::step_complete;
    case InstrKind::vdiv:
        state_.s[instruction.rd] = state_.s[instruction.rn] / state_.s[instruction.rm];
        return StopReason::step_complete;
    case InstrKind::vfma:
        state_.s[instruction.rd] = std::fma(
            state_.s[instruction.rn], state_.s[instruction.rm], state_.s[instruction.rd]
        );
        return StopReason::step_complete;
    case InstrKind::vfms:
        state_.s[instruction.rd] = std::fma(
            -state_.s[instruction.rn], state_.s[instruction.rm], state_.s[instruction.rd]
        );
        return StopReason::step_complete;
    case InstrKind::vfnms:
        state_.s[instruction.rd] = std::fma(
            state_.s[instruction.rn], state_.s[instruction.rm], -state_.s[instruction.rd]
        );
        return StopReason::step_complete;
    case InstrKind::vneg:
        state_.s[instruction.rd] = -state_.s[instruction.rm];
        return StopReason::step_complete;
    case InstrKind::vabs:
        state_.s[instruction.rd] = std::fabs(state_.s[instruction.rm]);
        return StopReason::step_complete;
    case InstrKind::vsqrt:
        state_.s[instruction.rd] = std::sqrt(state_.s[instruction.rm]);
        return StopReason::step_complete;
    case InstrKind::vcmp: {
        const float left = state_.s[instruction.rn];
        const float right = instruction.form == OperandForm::immediate
            ? 0.0F : state_.s[instruction.rm];
        state_.fpscr &= 0x0fffffffU;
        if (std::isnan(left) || std::isnan(right)) {
            state_.fpscr |= xpsr_c | xpsr_v;
        } else if (left == right) {
            state_.fpscr |= xpsr_z | xpsr_c;
        } else if (left < right) {
            state_.fpscr |= xpsr_n;
        } else {
            state_.fpscr |= xpsr_c;
        }
        return StopReason::step_complete;
    }
    case InstrKind::vmrs:
        state_.xpsr = (state_.xpsr & ~(xpsr_n | xpsr_z | xpsr_c | xpsr_v))
            | (state_.fpscr & (xpsr_n | xpsr_z | xpsr_c | xpsr_v));
        return StopReason::step_complete;
    case InstrKind::vstm:
    case InstrKind::vldm: {
        const bool load = instruction.kind == InstrKind::vldm;
        const std::uint32_t count = instruction.imm;
        const std::uint32_t original_base = state_.readRegister(instruction.rn);
        const bool decrement_before = !instruction.add && instruction.index;
        const std::uint32_t start = decrement_before
            ? original_base - count * 4U : original_base;
        const mem::AccessContext context{
            load ? mem::AccessType::data_read : mem::AccessType::data_write,
            state_.currentInstrAddr(),
        };
        std::uint32_t address = start;
        for (std::uint32_t offset = 0; offset < count; ++offset) {
            const auto fp_register = static_cast<std::size_t>(instruction.rd) + offset;
            if (load) {
                const auto value = memory_.read32(address, context);
                if (!value) return failBus(value.fault(), "VFP multiple load failed");
                state_.s[fp_register] = std::bit_cast<float>(value.value());
            } else {
                const auto stored = memory_.write32(
                    address, std::bit_cast<std::uint32_t>(state_.s[fp_register]), context
                );
                if (!stored) return failBus(stored.fault(), "VFP multiple store failed");
            }
            address += 4U;
        }
        if (instruction.writeback) {
            state_.writeRegister(
                instruction.rn,
                decrement_before ? start : original_base + count * 4U
            );
        }
        return StopReason::step_complete;
    }
    case InstrKind::mrs: {
        std::uint32_t value = 0U;
        switch (instruction.imm) {
        case 5U: value = state_.ipsr(); break;
        case 8U: value = state_.msp; break;
        case 9U: value = state_.psp; break;
        case 16U: value = state_.primask & 1U; break;
        case 17U:
        case 18U: value = state_.basepri & 0xffU; break;
        case 19U: value = state_.faultmask & 1U; break;
        case 20U: value = state_.control & 0x7U; break;
        default:
            diagnostic.bus_fault.reset();
            diagnostic.message = "unsupported MRS special register";
            return StopReason::undefined_instruction;
        }
        state_.writeRegister(instruction.rd, value);
        return StopReason::step_complete;
    }
    case InstrKind::msr: {
        const std::uint32_t value = state_.readRegister(instruction.rn);
        switch (instruction.imm) {
        case 8U:
            state_.msp = value & ~std::uint32_t{3};
            if (&state_.activeSp() == &state_.msp) state_.r[13] = state_.msp;
            break;
        case 9U:
            state_.psp = value & ~std::uint32_t{3};
            if (&state_.activeSp() == &state_.psp) state_.r[13] = state_.psp;
            break;
        case 16U: state_.primask = value & 1U; break;
        case 17U: state_.basepri = value & 0xffU; break;
        case 18U: {
            const std::uint32_t requested = value & 0xffU;
            if (requested != 0U && (state_.basepri == 0U || requested < state_.basepri)) {
                state_.basepri = requested;
            }
            break;
        }
        case 19U: state_.faultmask = value & 1U; break;
        case 20U: state_.control = value & 0x7U; break;
        default:
            diagnostic.bus_fault.reset();
            diagnostic.message = "unsupported MSR special register";
            return StopReason::undefined_instruction;
        }
        return StopReason::step_complete;
    }
    case InstrKind::cps:
        if ((instruction.imm & 0x2U) != 0U) state_.primask = instruction.add ? 0U : 1U;
        if ((instruction.imm & 0x1U) != 0U) state_.faultmask = instruction.add ? 0U : 1U;
        return StopReason::step_complete;
    case InstrKind::svc:
        state_.pending_exception = 11U;
        return StopReason::step_complete;
    case InstrKind::it:
        if (inItBlock(state_.it_state)) return failInvalid("nested IT block is invalid");
        state_.setItState(static_cast<std::uint8_t>(instruction.imm));
        return StopReason::step_complete;
    case InstrKind::nop:
    case InstrKind::dmb:
    case InstrKind::dsb:
    case InstrKind::isb:
    case InstrKind::wfi:
    case InstrKind::wfe:
    case InstrKind::sev:
        return StopReason::step_complete;
    case InstrKind::bkpt:
        state_.halted = true;
        diagnostic.bus_fault.reset();
        diagnostic.message = "BKPT #" + std::to_string(instruction.imm);
        return StopReason::breakpoint;
    case InstrKind::undefined:
        diagnostic.bus_fault.reset();
        diagnostic.message = "decoded instruction has no executor";
        return StopReason::undefined_instruction;
    }
    diagnostic.bus_fault.reset();
    diagnostic.message = "decoded instruction kind is outside executor table";
    return StopReason::undefined_instruction;
}

} // namespace fil::cpu
