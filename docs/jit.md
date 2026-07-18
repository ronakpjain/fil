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

LLVM ORC headers are available on the development host through Homebrew LLVM 22.
The next implementation step is an optional `FIL_ENABLE_LLVM_JIT` backend that
lowers a `JitBlockPlan` to a function operating on `CpuState`, returns exact
instruction/cycle counts, and exits before the world's event horizon. The block
cache must be keyed by guest PC, executable-memory generation, and IT state.

This commit is correctness groundwork, not a claimed speedup. Unit tests cover
pure ALU acceptance, PC-writing rejection, every major boundary class, maximal
prefix selection, and the compilation cap.
