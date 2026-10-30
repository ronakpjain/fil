#pragma once

#include "fil/cpu/micro_jit_ir.hpp"

namespace fil::cpu {

/** Mixed reversible IR. Memory remains checked at execution time: lowering
 * proves instruction shape, never mapping, permissions, or RAM stability.
 * The existing pure MicroBlock ABI and its admission policy are unchanged.
 */
enum class NativeOp : std::uint8_t {
    integer,
    load_word,
    store_word,
    load_byte,
    store_byte,
    load_half,
    store_half,
    load_byte_signed,
    load_half_signed,
};

struct NativeInst {
    NativeOp kind{NativeOp::integer};
    MicroInst integer{};
    std::uint32_t pc{0};
    std::uint32_t offset{0};
    std::uint8_t size{0};
    std::uint8_t rd{0};
    std::uint8_t rn{0};
    std::uint32_t absolute_address{0};
    bool absolute{false};
};

/** @brief True for any native memory operation. */
[[nodiscard]] constexpr bool nativeOpIsMemory(NativeOp op) noexcept {
    return op != NativeOp::integer;
}
/** @brief True for native load operations (any width/sign). */
[[nodiscard]] constexpr bool nativeOpIsLoad(NativeOp op) noexcept {
    return op == NativeOp::load_word || op == NativeOp::load_byte
        || op == NativeOp::load_half || op == NativeOp::load_byte_signed
        || op == NativeOp::load_half_signed;
}
/** @brief Accessed width in bytes for a native memory operation. */
[[nodiscard]] constexpr std::uint32_t nativeOpWidth(NativeOp op) noexcept {
    switch (op) {
    case NativeOp::load_byte:
    case NativeOp::store_byte:
    case NativeOp::load_byte_signed: return 1U;
    case NativeOp::load_half:
    case NativeOp::store_half:
    case NativeOp::load_half_signed: return 2U;
    default: return 4U;
    }
}

struct NativeBlock {
    static constexpr std::size_t max_ops = MicroBlock::max_ops;
    std::vector<NativeInst> ops;
    std::uint32_t entry_pc{0};
    bool ends_with_branch{false};
    bool has_memory{false};
};

/** Unconditional word accesses with r0-r12 data registers are admitted.
 * Immediate bases may be r0-r13 (SP is read-only); literal loads normalize
 * aligned PC-relative addresses. Other PC operands, writeback, predication and
 * multi-access instructions decline. On failure, the output is unchanged.
 */
[[nodiscard]] MicroLowerResult tryLowerNativeInst(const MicroSource& source, NativeInst& out) noexcept;
[[nodiscard]] MicroLowerResult buildNativeBlock(std::span<const MicroSource> sources, NativeBlock& out) noexcept;

} // namespace fil::cpu
