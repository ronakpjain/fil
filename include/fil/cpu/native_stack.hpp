#pragma once
#include "fil/cpu/native_execution.hpp"
#include <cstdint>
namespace fil::cpu {
enum class NativeStackResult : std::uint32_t {
    success = 0, memory_declined = 1, branch_mismatch = 2, dynamic_return = 3
};
/**
 * With allow_dynamic_returns, dynamic_return commits a valid alternate POP PC.
 * The caller must charge this op and exit immediately, retaining the actual PC.
 * memory_declined and branch_mismatch leave registers/SP/footprints unchanged.
 */
[[nodiscard]] NativeStackResult nativeStackTransfer(NativeExecutionContext*, std::uint32_t instruction_pc,
    std::uint32_t register_list, std::uint32_t push, std::uint32_t expected_pc) noexcept;
}
