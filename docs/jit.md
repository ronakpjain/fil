# Native block JIT experiment

This branch starts a conservative native-block tier without changing interpreter
semantics. `classifyJitBoundary()` separates pure integer instructions from exits
that require memory/MMIO, control-flow, exception/system, or floating-point
handling. `planJitBlock()` selects a bounded straight-line prefix (maximum 64
instructions) suitable for later LLVM ORC lowering.

The initial tier deliberately exits before all memory operations. That keeps
`MemoryBus` faults, MMIO synchronization, mutation journals, read footprints, and
self-modifying-code generations authoritative in the interpreter. Branches end a
block, so target-PC validation and deterministic world scheduling remain unchanged.
FP and system instructions also remain interpreted until dedicated equivalence
tests exist.

The optional `FIL_ENABLE_LLVM_JIT` backend uses LLVM ORC to lower accepted blocks
to native functions operating on the integer register file and xPSR pointer. The
first executable lowering supports MOVW, MOVT, flag-free immediate ADD/SUB, and
NOP; unsupported pure instructions fail closed before module installation. ORC
unit coverage compiles and executes a two-instruction block and verifies register
state plus exact instruction count.

Configure with `-DFIL_ENABLE_LLVM_JIT=ON` and an LLVM package path such as
`-DLLVM_DIR=/opt/homebrew/opt/llvm/lib/cmake/llvm`. Builds without LLVM remain the
default. The measured-losing single-instruction integration is independently
available through `FIL_ENABLE_SINGLE_INSTRUCTION_JIT`; it is off by default so an
LLVM-capable build does not enlarge the decoded cache or initialize ORC at run time. `CortexM4` now counts executions per decoded-cache PC, compiles only saturated
hot entries after 65,535 hits, and stores the resulting native function in the same
execution-generation-tagged cache entry. IT-block instructions and active-SP/PC
writes remain interpreted. Compilation failures mark only that cache entry as
rejected, preserving deterministic fallback.

The integrated six-board run reaches the exact one-second boundary with identical
96M instruction/cycle totals and terminal PCs. Raising the tier threshold reduced
the non-IPO runtime from about 1.25 s to 1.16 s; enabling the project's supported
IPO configuration reduces the LLVM build further to about 0.94 s. A 4,096-hit IPO
tier measured about 0.96 s, confirming that ORC startup and one native call per
target instruction still dominate. The
next performance stage must compile and dispatch multi-instruction plans up to the
world event horizon. Unit tests cover
pure ALU acceptance, PC-writing rejection, every major boundary class, maximal
prefix selection, and the compilation cap.
