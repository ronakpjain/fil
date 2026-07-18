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
default. The next integration step is a hot block cache keyed by guest PC,
executable-memory generation, and IT state, followed by event-horizon dispatch
from `CortexM4`.

This backend stage is correctness groundwork, not a claimed simulator speedup. Unit tests cover
pure ALU acceptance, PC-writing rejection, every major boundary class, maximal
prefix selection, and the compilation cap.
