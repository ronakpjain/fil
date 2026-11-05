#include "registers.hpp"

#include "fil/cpu/cortex_m4.hpp"
#include "fil/sim/board.hpp"
#include "rsp.hpp"

#include <algorithm>
#include <bit>

namespace fil::debug::detail {
namespace {

constexpr std::string_view target_description = R"XML(<?xml version="1.0"?>
<!DOCTYPE target SYSTEM "gdb-target.dtd">
<target>
  <architecture>arm</architecture>
  <feature name="org.gnu.gdb.arm.m-profile">
    <reg name="r0" bitsize="32" type="uint32"/>
    <reg name="r1" bitsize="32" type="uint32"/>
    <reg name="r2" bitsize="32" type="uint32"/>
    <reg name="r3" bitsize="32" type="uint32"/>
    <reg name="r4" bitsize="32" type="uint32"/>
    <reg name="r5" bitsize="32" type="uint32"/>
    <reg name="r6" bitsize="32" type="uint32"/>
    <reg name="r7" bitsize="32" type="uint32"/>
    <reg name="r8" bitsize="32" type="uint32"/>
    <reg name="r9" bitsize="32" type="uint32"/>
    <reg name="r10" bitsize="32" type="uint32"/>
    <reg name="r11" bitsize="32" type="uint32"/>
    <reg name="r12" bitsize="32" type="uint32"/>
    <reg name="sp" bitsize="32" type="data_ptr"/>
    <reg name="lr" bitsize="32" type="uint32"/>
    <reg name="pc" bitsize="32" type="code_ptr"/>
    <reg name="xpsr" bitsize="32" type="uint32"/>
    <reg name="msp" bitsize="32" type="data_ptr"/>
    <reg name="psp" bitsize="32" type="data_ptr"/>
    <reg name="control" bitsize="32" type="uint32"/>
    <reg name="primask" bitsize="32" type="uint32"/>
    <reg name="basepri" bitsize="32" type="uint32"/>
    <reg name="faultmask" bitsize="32" type="uint32"/>
  </feature>
  <feature name="org.gnu.gdb.arm.vfp">
    <reg name="d0" bitsize="64" type="ieee_double"/>
    <reg name="d1" bitsize="64" type="ieee_double"/>
    <reg name="d2" bitsize="64" type="ieee_double"/>
    <reg name="d3" bitsize="64" type="ieee_double"/>
    <reg name="d4" bitsize="64" type="ieee_double"/>
    <reg name="d5" bitsize="64" type="ieee_double"/>
    <reg name="d6" bitsize="64" type="ieee_double"/>
    <reg name="d7" bitsize="64" type="ieee_double"/>
    <reg name="d8" bitsize="64" type="ieee_double"/>
    <reg name="d9" bitsize="64" type="ieee_double"/>
    <reg name="d10" bitsize="64" type="ieee_double"/>
    <reg name="d11" bitsize="64" type="ieee_double"/>
    <reg name="d12" bitsize="64" type="ieee_double"/>
    <reg name="d13" bitsize="64" type="ieee_double"/>
    <reg name="d14" bitsize="64" type="ieee_double"/>
    <reg name="d15" bitsize="64" type="ieee_double"/>
    <reg name="fpscr" bitsize="32" type="int"/>
  </feature>
</target>
)XML";

bool wideRegisterIndex(const std::uint32_t index) noexcept {
    return index >= double_register_base && index < double_register_base + double_register_count;
}

void refreshActiveSpMirror(cpu::CpuState& state) noexcept {
    state.r[sp_register_index] = state.activeSp();
}

} // namespace

std::string_view targetDescription() noexcept {
    return target_description;
}

std::optional<std::string> readFeaturePacket(const std::string_view packet) {
    constexpr std::string_view feature_prefix = "qXfer:features:read:";
    if (!packet.starts_with(feature_prefix)) return std::nullopt;
    const std::string_view spec = packet.substr(feature_prefix.size());
    const auto colon = spec.rfind(':');
    if (colon == std::string_view::npos) return std::nullopt;
    if (spec.substr(0U, colon) != "target.xml") return std::nullopt;
    const std::string_view range = spec.substr(colon + 1U);
    const auto comma = range.find(',');
    if (comma == std::string_view::npos) return std::string("E01");
    const auto offset = parseHex(range.substr(0U, comma));
    const auto length = parseHex(range.substr(comma + 1U));
    if (!offset || !length) return std::string("E01");
    if (*offset >= target_description.size()) return std::string("l");
    const std::size_t available = target_description.size() - static_cast<std::size_t>(*offset);
    const std::size_t count = std::min<std::size_t>(
        static_cast<std::size_t>(*length), available
    );
    std::string reply;
    reply.reserve(count + 1U);
    reply.push_back(count < available ? 'm' : 'l');
    reply.append(target_description.substr(static_cast<std::size_t>(*offset), count));
    return reply;
}

std::string readAllRegisters(const sim::Board& board) {
    const cpu::CpuState& state = board.cpu().state();
    std::string text;
    text.reserve(register_count * single_register_nibbles);
    for (std::size_t index = 0U; index < sp_register_index; ++index) {
        text += hexValue(state.r[index], 4U);
    }
    text += hexValue(state.readRegister(static_cast<std::uint8_t>(sp_register_index)), 4U);
    text += hexValue(state.r[lr_register_index], 4U);
    text += hexValue(state.r[pc_register_index] & ~std::uint32_t{1U}, 4U);
    text += hexValue(state.xpsr, 4U);
    text += hexValue(state.msp, 4U);
    text += hexValue(state.psp, 4U);
    text += hexValue(state.control, 4U);
    text += hexValue(state.primask, 4U);
    text += hexValue(state.basepri, 4U);
    text += hexValue(state.faultmask, 4U);
    for (std::size_t index = 0U; index < double_register_count; ++index) {
        const std::uint32_t low = std::bit_cast<std::uint32_t>(state.s[index * 2U]);
        const std::uint32_t high = std::bit_cast<std::uint32_t>(state.s[index * 2U + 1U]);
        text += hexValue(
            static_cast<std::uint64_t>(low) | (static_cast<std::uint64_t>(high) << 32U), 8U
        );
    }
    text += hexValue(state.fpscr, 4U);
    return text;
}

std::optional<std::string> readSingleRegister(
    const sim::Board& board,
    const std::uint32_t index
) {
    const cpu::CpuState& state = board.cpu().state();
    if (index < sp_register_index) return hexValue(state.r[index], 4U);
    switch (index) {
    case sp_register_index:
        return hexValue(state.readRegister(static_cast<std::uint8_t>(index)), 4U);
    case lr_register_index: return hexValue(state.r[lr_register_index], 4U);
    case pc_register_index:
        return hexValue(state.r[pc_register_index] & ~std::uint32_t{1U}, 4U);
    case xpsr_register_index: return hexValue(state.xpsr, 4U);
    case msp_register_index: return hexValue(state.msp, 4U);
    case psp_register_index: return hexValue(state.psp, 4U);
    case control_register_index: return hexValue(state.control, 4U);
    case primask_register_index: return hexValue(state.primask, 4U);
    case basepri_register_index: return hexValue(state.basepri, 4U);
    case faultmask_register_index: return hexValue(state.faultmask, 4U);
    case fpscr_register_index: return hexValue(state.fpscr, 4U);
    default: break;
    }
    if (wideRegisterIndex(index)) {
        const std::size_t pair = index - double_register_base;
        const std::uint32_t low = std::bit_cast<std::uint32_t>(state.s[pair * 2U]);
        const std::uint32_t high = std::bit_cast<std::uint32_t>(state.s[pair * 2U + 1U]);
        return hexValue(
            static_cast<std::uint64_t>(low) | (static_cast<std::uint64_t>(high) << 32U), 8U
        );
    }
    return std::nullopt;
}

bool writeSingleRegister(
    sim::Board& board,
    const std::uint32_t index,
    const std::uint64_t value
) {
    cpu::CpuState& state = board.cpu().state();
    const auto word = static_cast<std::uint32_t>(value);
    if (index < sp_register_index) {
        state.r[index] = word;
        return true;
    }
    switch (index) {
    case sp_register_index:
        state.writeRegister(static_cast<std::uint8_t>(index), word);
        return true;
    case lr_register_index:
        state.r[lr_register_index] = word;
        return true;
    case pc_register_index:
        state.r[pc_register_index] = word & ~std::uint32_t{1U};
        return true;
    case xpsr_register_index:
        state.xpsr = word;
        state.thumb = (word & cpu::xpsr_t) != 0U;
        state.it_state = static_cast<std::uint8_t>(
            ((word >> 25U) & 0x3U) | (((word >> 10U) & 0x3fU) << 2U)
        );
        refreshActiveSpMirror(state);
        return true;
    case msp_register_index:
        state.msp = word;
        refreshActiveSpMirror(state);
        return true;
    case psp_register_index:
        state.psp = word;
        refreshActiveSpMirror(state);
        return true;
    case control_register_index:
        state.control = word;
        refreshActiveSpMirror(state);
        return true;
    case primask_register_index:
        state.primask = word;
        return true;
    case basepri_register_index:
        state.basepri = word;
        return true;
    case faultmask_register_index:
        state.faultmask = word;
        return true;
    case fpscr_register_index:
        state.fpscr = word;
        return true;
    default: break;
    }
    if (wideRegisterIndex(index)) {
        const std::size_t pair = index - double_register_base;
        state.s[pair * 2U] = std::bit_cast<float>(word);
        state.s[pair * 2U + 1U] = std::bit_cast<float>(static_cast<std::uint32_t>(value >> 32U));
        return true;
    }
    return false;
}

bool writeAllRegisters(sim::Board& board, const std::string_view text) {
    std::size_t position = 0U;
    for (std::uint32_t index = 0U; index < register_count; ++index) {
        const std::size_t nibbles = wideRegisterIndex(index)
            ? double_register_nibbles : single_register_nibbles;
        if (position + nibbles > text.size()) return false;
        const auto value = parseLittleEndianHex(text.substr(position, nibbles));
        if (!value || !writeSingleRegister(board, index, *value)) return false;
        position += nibbles;
    }
    return position == text.size();
}

} // namespace fil::debug::detail
