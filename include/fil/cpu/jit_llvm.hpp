#pragma once

/** @file jit_llvm.hpp
 *  @brief Optional LLVM ORC lowering for conservative pure-integer blocks.
 */

#include "fil/cpu/instruction.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace fil::cpu {

class LlvmJitEngine {
public:
    using BlockFunction = std::uint64_t (*)(std::uint32_t* registers, std::uint32_t* xpsr);

    struct CompiledBlock {
        BlockFunction function{nullptr};
        std::uint64_t instructions{0};
        std::string symbol;
    };

    LlvmJitEngine();
    ~LlvmJitEngine();
    LlvmJitEngine(const LlvmJitEngine&) = delete;
    LlvmJitEngine& operator=(const LlvmJitEngine&) = delete;

    /** Compiles a pure block accepted by planJitBlock; throws on unsupported lowering. */
    [[nodiscard]] CompiledBlock compile(std::span<const DecodedInstruction> instructions);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace fil::cpu
