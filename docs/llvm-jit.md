# Native LLVM JIT

`--jit` enables an experimental LLVM ORC backend when the executable was built with LLVM support. Unlike the cached-handler implementation, it emits LLVM IR, optimizes it with the O2 module pipeline, and materializes **host machine code** through ORC LLJIT. Generated kernels do not call the interpreter.

The interpreter remains the default and correctness reference. Unsupported, cold, or unsafe execution uses the existing interpreter/cached handlers. A native backend is not automatically a speedup: compilation and function-call costs remain, and the network scheduler still preserves each observable instruction boundary.

## Build

CMake attempts LLVM discovery by default, including `llvm-config --cmakedir`. LLVM 18 or newer and a 64-bit host are required; LLVM 23.1.2 on Apple Silicon is the locally validated toolchain. The target-triple API is version-gated for older LLVM releases, but the complete backend has not been built against every accepted LLVM version.

```bash
cmake -S . -B build-llvm -DCMAKE_BUILD_TYPE=Release \
  -DLLVM_DIR="$(llvm-config --cmakedir)"
cmake --build build-llvm --parallel 8
ctest --test-dir build-llvm --output-on-failure
./build-llvm/fil run-network configs/networks/per_vehicle.json --jit
```

The configure log says whether native LLVM support is enabled. Missing LLVM is not a dependency failure: the project builds without the native backend and `--jit` then uses cached handlers. To explicitly build without LLVM, use `-DFIL_ENABLE_LLVM_JIT=OFF`. `-DFIL_LLVM_COMPONENT_LINKING=ON` exercises component-library linking rather than a monolithic LLVM library.

`run` and `run-network` print per-board `native_jit` counters only when `--jit` is requested. `backend=llvm-orc` identifies the configured backend; **positive `executions` and `instructions` confirm actual native execution**, rather than merely an enabled flag. Zero counts can mean the firmware did not reach a supported hot path. `failures` and `native_jit_error` report compilation failures; execution falls back rather than changing guest behavior.

## Initial native subset

- NOP.
- Immediate MOV, MOVW, MOVT.
- Immediate ADD, SUB, CMP, CMN, including architectural NZCV/carry updates.
- Unconditional B and BL as the final instruction of a block, including link-register updates and branch-to-fallthrough cases.

The supported forms use ordinary integer registers r0–r12, except for explicit PC/LR effects of B/BL. SP and PC operand forms, conditional execution, memory/MMIO operations, system instructions, and floating-point operations are not lowered. They retain existing execution paths. This is a genuine but deliberately incomplete native JIT, not a full ARMv7-M translator.

## Safety and lifetime

Native kernels use the verified host `CpuState` layout and preserve unrelated fields, including FP bit patterns. Runtime prefix limits prevent a kernel from retiring more instructions than admitted by its caller; zero limits make no changes. IT, invalid Thumb, halted, and pending exception contexts do not execute native kernels.

The shared CPU front end retains fetch permissions, conditions, faults, pipeline timing, and diagnostics. Decoded-cache PC/execution-generation mismatches clear native entries, so executable writes cannot reuse stale code. Supported native blocks contain no memory writes, so they cannot invalidate themselves while running. Compilation does not advance simulated time or mutate guest registers.

General network dispatch remains one instruction. Larger kernels run only through the existing board horizon proof or synchronized pure fixed-cost lane proof. Native generation is **not** permission to skip MMIO, scheduled events, SysTick, stop addresses, instruction budgets, or trace boundaries.

ORC engines are initialized lazily; native single-instruction compilation starts after 128 eligible hits at a decoded entry. Block preparation uses the existing 50-probe hotness gate. A CPU retains at most 32 native kernels, bounding compilation work and stale-code retention. Kernels keep the runtime alive and remove their ORC resources on destruction. Executable-write invalidation does not immediately reclaim retained kernels; subsequent compilation can reach the budget and fall back safely.

## Validation and performance

Native-specific tests compare emitted kernels and CPU integration against repeated interpreter steps, including arithmetic flag edges, prefix limits, branches, preserved FP state, exceptional-context fallback, and code invalidation. Existing event, interrupt, scheduler, and MMIO differential tests also run with LLVM enabled. See `performance.md` for measurements and their limitations.

For comparisons, use the same LLVM-enabled executable with and without `--jit`, identical firmware/ADC settings, and alternating runs. Include compilation in end-to-end timings rather than reporting only an already-warmed kernel. Strip only `native_jit` diagnostic lines when comparing simulator summary output; compare traces without filtering. ADC decimation changes guest-visible behavior and must be held fixed within each comparison.
