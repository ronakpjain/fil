#pragma once

/** @file micro_jit.hpp
 *  @brief Bespoke FilIR stencil JIT: host machine code without LLVM.
 *
 *  MicroJit compiles a FilIR MicroBlock (straight-line unconditional
 *  integer ops, terminal B/BL) into a single straight-line AArch64
 *  function with one entry point and no internal branches:
 *
 *      void fn(CpuState* state);
 *
 *  The caller guarantees full-block execution (max >= block size),
 *  Thumb/IT/pending/halted checks, and generation validity; the kernel
 *  touches only r0-r12, LR, xPSR NZCV, r15, and instruction_address.
 *  Guest N/Z/C/V ride the host flags between ops; Q/GE/IT/ISR/T bits
 *  are preserved. Cache slots own no resources: code lives in one
 *  bump-allocated RX arena for the CPU lifetime, so eviction is a
 *  slot-pointer clear and executable-write invalidation keeps working
 *  through the existing generation checks.
 */

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace fil::cpu {

struct CpuState;
struct MicroBlock;

/** Full-block kernel: executes every op in the compiled block. */
using MicroFunction = void (*)(CpuState*);

class MicroJit {
public:
    ~MicroJit();
    MicroJit(const MicroJit&) = delete;
    MicroJit& operator=(const MicroJit&) = delete;

    /** True on hosts with a stencil emitter (AArch64); else all compiles decline. */
    [[nodiscard]] static bool hostSupported() noexcept;

    [[nodiscard]] static std::unique_ptr<MicroJit> create(std::string& error);

    /** Compiles a block; returns nullptr (with error set) on decline/failure. */
    [[nodiscard]] MicroFunction compile(const MicroBlock& block, std::string& error);

    [[nodiscard]] std::uint64_t compilations() const noexcept { return compilations_; }
    [[nodiscard]] std::uint64_t bytesEmitted() const noexcept { return bytes_emitted_; }

private:
    MicroJit() noexcept = default;
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::uint64_t compilations_{0};
    std::uint64_t bytes_emitted_{0};
};

} // namespace fil::cpu
