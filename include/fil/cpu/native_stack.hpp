#pragma once
#include "fil/cpu/native_execution.hpp"
#include <cstdint>
namespace fil::cpu {
enum class NativeStackResult : std::uint32_t { success = 0, memory_declined = 1, branch_mismatch = 2 };
[[nodiscard]] NativeStackResult nativeStackTransfer(NativeExecutionContext*, std::uint32_t instruction_pc,
    std::uint32_t register_list, std::uint32_t push, std::uint32_t expected_pc) noexcept;
}
