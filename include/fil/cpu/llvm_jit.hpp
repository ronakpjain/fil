#pragma once

/** @file llvm_jit.hpp
 *  @brief LLVM ORC native-code backend for a conservative CPU instruction subset.
 */

#include "fil/cpu/instruction.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace fil::cpu {

struct CpuState;

/** An already-decoded instruction and its address/encoded width. */
struct NativeJitInstruction {
    DecodedInstruction decoded{};
    std::uint32_t pc{0};
    std::uint8_t size{0};
};

class LlvmJit;
class CortexM4;

/** Compiled native block; owns its ORC resources and keeps the runtime alive. */
class NativeJitKernel {
public:
    ~NativeJitKernel();
    NativeJitKernel(const NativeJitKernel&) = delete;
    NativeJitKernel& operator=(const NativeJitKernel&) = delete;

    /** Execute up to max_instructions, without throwing or consulting the interpreter. */
    void execute(CpuState& state, std::size_t max_instructions) const noexcept;

private:
    using Function = void (*)(CpuState*, std::size_t);
    // CortexM4 already checked architectural context and owns the kernel.
    [[nodiscard]] Function entryPoint() const noexcept;
    struct Impl;
    explicit NativeJitKernel(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
    friend class LlvmJit;
    friend class CortexM4;
};

/** LLVM ORC LLJIT compiler for straight-line Cortex-M integer kernels. */
class LlvmJit {
public:
    ~LlvmJit();
    LlvmJit(const LlvmJit&) = delete;
    LlvmJit& operator=(const LlvmJit&) = delete;

    [[nodiscard]] static std::unique_ptr<LlvmJit> create(std::string& error);
    [[nodiscard]] static bool supports(const NativeJitInstruction& instruction) noexcept;
    [[nodiscard]] std::shared_ptr<const NativeJitKernel> compile(
        std::span<const NativeJitInstruction> instructions,
        std::string& error
    );

private:
    struct Impl;
    explicit LlvmJit(std::shared_ptr<Impl> impl) noexcept;
    std::shared_ptr<Impl> impl_;
    friend class NativeJitKernel;
};

} // namespace fil::cpu
