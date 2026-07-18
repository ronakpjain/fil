# Thumb instruction coverage

This document describes the decoder and executor currently in this repository. It is a supported-subset statement, not a claim of complete ARMv7E-M or Cortex-M4F conformance. An instruction is supported only when its exact encoding matches a decoder row and the executor accepts its operand restrictions.

## Coverage labels

- **Unit covered** means representative encodings and semantics are exercised by the required CPU tests.
- **Acceptance exercised** means the subset is also exercised by the real `g4_testing` one-second run.
- **Implemented** means code exists, but every encoding and architectural corner case does not necessarily have a dedicated test.

## 16-bit Thumb families

| Family | Implemented forms | Evidence and limits |
|---|---|---|
| Shifts | `LSL`, `LSR`, `ASR` immediate and register; `ROR` register | Unit covered, including shift amounts 0, 32, and greater than 32. |
| Arithmetic | `ADD`, `ADC`, `SUB`, `SBC`, `RSB`, `CMP`, `CMN` | Unit covered for flags and representative register/immediate forms. Includes high-register ADD/CMP and SP/PC address generation. |
| Logical and move | `MOV`, `AND`, `ORR`, `EOR`, `BIC`, `MVN`, `TST` | Unit covered through startup and wide-ALU slices. |
| Multiply | `MUL` | Unit covered for low 32-bit product and flag behavior. |
| Extend | `SXTB`, `SXTH`, `UXTB`, `UXTH` | Representative byte forms unit covered. |
| Byte reversal | `REV`, `REV16`, `REVSH` | Implemented for low-register 16-bit forms; representative `REV16` semantics are unit covered. |
| Loads and stores | `LDR`, `STR`, `LDRB`, `STRB`, `LDRH`, `STRH`, `LDRSB`, `LDRSH` | Literal, SP-relative, immediate, and register-offset families are implemented. Byte/halfword semantics and PC loads are unit covered. |
| Multiple transfer | `PUSH`, `POP`, `LDM`, `STM` | Unit covered for register ordering, writeback, PC return, and EXC_RETURN markers. |
| Control flow | conditional and unconditional `B`, `BLX` register, `BX`, `CBZ`, `CBNZ` | Branch displacement, `BL`/`BX` calls, and Thumb-target validation are unit covered. `BL` itself is a 32-bit encoding. |
| Predication | `IT` | True and false slots plus IT-state advancement are unit covered. Implicit flag suppression inside an IT block is implemented; nested IT is rejected. |
| System and hints | `SVC`, `CPSIE`, `CPSID`, `NOP`, `WFI`, `WFE`, `SEV`, `BKPT` | SVC and mask changes are unit covered. Hints have simplified behavior described below; BKPT is a host stop boundary. |

## 32-bit Thumb-2 families

| Family | Implemented forms | Evidence and limits |
|---|---|---|
| Immediate construction | `MOVW`, `MOVT` | Decode and execution unit covered. |
| Data processing | `AND`, `BIC`, `ORR`, `ORN`, `EOR`, `ADD`, `ADC`, `SBC`, `SUB`, `RSB` | Modified-immediate and shifted-register forms are implemented, including `TST`, `CMP`, `CMN`, `MOV`, and `MVN` aliases. Plain-immediate `ADDW`/`SUBW` are also implemented. Real SystemInit `ORR.W`, shifted CMP/MOV, and `ADDW` encodings are unit covered. |
| Register shifts | `LSL`, `LSR`, `ASR`, `ROR` | Register-controlled wide forms are implemented; representative `LSL.W` is unit covered. `RRX` is available through shifted data-processing operand decoding. |
| Multiply/divide | `MUL`, `MLA`, `MLS`, `UMULL`, `SMULL`, `UDIV`, `SDIV` | Real firmware encodings and results are unit covered. Divide by zero currently produces zero. |
| Parallel/select | `UADD8`, `SEL` | Per-byte addition, APSR.GE carry lanes, and GE-selected bytes are unit covered with production-board encodings. |
| Bit and extend | `CLZ`, `UBFX`, `SXTB`, `SXTH`, `UXTB`, `UXTH` | Representative real encodings are unit covered; wide extend supports rotations by multiples of eight. |
| Loads and stores | wide `LDR`, `STR`, `LDRB`, `STRB`, `LDRH`, `STRH`, `LDRSB`, `LDRSH`, plus `LDRD`, `STRD` | Selected literal, 12-bit immediate, register-offset, pre-index, post-index, and writeback forms are implemented. Real doubleword stack forms are unit covered. |
| Multiple transfer | wide `PUSH`, `POP`, `LDM`, `STM` | Implemented for accepted register lists and increment-after/decrement-before forms. EXC_RETURN through POP/LDM is recognized. |
| Branch | conditional and unconditional `B.W`, `BL` | Decode and execution are unit covered for representative branches. Immediate `BLX` is not implemented. |
| Special registers | `MRS`, `MSR` | Supported registers are IPSR for reads; MSP, PSP, PRIMASK, BASEPRI, BASEPRI_MAX, FAULTMASK, and CONTROL as applicable. Representative FreeRTOS forms are unit covered. |
| Ordering | `DMB`, `DSB`, `ISB`, wide `NOP` | Decoded and unit covered as deterministic no-ops in this single-threaded model. |
| FP loads/stores | `VLDR`, `VSTR`, `VSTM`, `VLDM` | Positive/negative base and PC-relative scalar transfers are implemented for S registers and D-register bit pairs. S-register lists and D-register `VPUSH`/`VPOP` aliases use adjacent S-register storage. FreeRTOS `VSTMDB {s16-s31}` and `VLDMIA {s16-s31}` context forms are unit covered. |
| FP moves | `VMOV` core-to-S, S-to-core, S-to-S, and modified immediate | Core/S bit transfers and immediate expansion are unit covered in a real scalar pipeline. Core-register pairs and general FPSCR moves are not implemented. |
| FP arithmetic | `VADD.F32`, `VSUB.F32`, `VMUL.F32`, `VNMUL.F32`, `VDIV.F32`, `VFMA.F32`, `VFMS.F32`, `VFNMS.F32`, `VNEG.F32`, `VABS.F32`, `VSQRT.F32` | Implemented with host `float`/`std::fma` operations. Add, subtract, multiply, and divide are directly unit covered; the other listed operations are implemented without a dedicated result test for every encoding. |
| FP convert/compare | signed/unsigned `VCVT` between F32 and 32-bit integer bits, `VCMP`/`VCMPE` with S register or zero, `VMRS APSR_nzcv,FPSCR` | Representative signed and unsigned conversions, compare flags, and APSR flag transfer are unit covered. |

## CPU and exception semantics

The core implements architectural PC reads, Thumb-target validation, APSR N/Z/C/V updates, IT state, MSP/PSP selection, PRIMASK/BASEPRI/FAULTMASK masking, and deterministic fetch/data bus faults. FP state is stored as S0-S31; D0-D15 memory transfers alias adjacent S-register bits. Every instruction currently costs one cycle.

At board level, SVC enters exception 11, pending SysTick/PendSV/NVIC exceptions vector through VTOR, and `BX`, `POP`, or `LDM` with a recognized EXC_RETURN restores a frame. Basic integer frames and extended frames containing S0-S15 plus FPSCR are unit covered. FreeRTOS-style manual S16-S31 transfers are covered separately by VSTM/VLDM.

The one-second real-firmware acceptance command is:

```bash
./build/fil run configs/boards/g4_testing.json \
  --duration-ms 1000 \
  --max-instructions 50000000
```

With the configured external ELF it reaches FreeRTOS scheduling and continues to the
time boundary after 16,000,000 instructions with no undefined instruction and zero
top-level unknown MMIO addresses.

The opt-in `fil.per.audit_instructions` CTest runs
`tools/audit_per_instructions.py` over all seven configured PER ELFs. The tool uses
`arm-none-eabi-objdump -d` as its code/data boundary authority, groups contiguous
instructions, and checks that `fil disasm-window` agrees on address, raw halfwords,
instruction width, and supported status. With the current artifacts it audits
55,068 objdump-recognized instructions with zero unsupported results and zero
address/width mismatches. It writes the observed per-ELF counts to
`per-instruction-audit.json` in the CMake build directory rather than embedding
artifact-dependent counts in the test. The report also records the fil and objdump
versions plus each ELF's SHA-256 digest so results can be tied to exact inputs.

Run the same audit outside CTest with:

```bash
python3 tools/audit_per_instructions.py \
  --fil ./build/fil \
  --objdump arm-none-eabi-objdump \
  --report /tmp/per-instruction-audit.json \
  /absolute/path/to/PER/Projects/firmware/output/g4_testing/g4_testing.elf \
  /absolute/path/to/PER/Projects/firmware/output/dashboard/dashboard.elf \
  /absolute/path/to/PER/Projects/firmware/output/main_module/main_module.elf \
  /absolute/path/to/PER/Projects/firmware/output/torque_vector/torque_vector.elf \
  /absolute/path/to/PER/Projects/firmware/output/a_box/a_box.elf \
  /absolute/path/to/PER/Projects/firmware/output/front_driveline/front_driveline.elf \
  /absolute/path/to/PER/Projects/firmware/output/rear_driveline/rear_driveline.elf
```

The automated audit deliberately does not compare operand fields or normalize
objdump mnemonic aliases and condition suffixes. A separate targeted audit checked
mnemonic kinds and operand fields for 2,268 ADDW/scalar-VFP instances. Both scans
cover instructions present in those particular binaries; neither implies support
for encodings absent from them or replaces execution tests.

## Intentional simplifications

- `WFI`, `WFE`, and `SEV` do not currently suspend the CPU or maintain an event latch.
- `DMB`, `DSB`, and `ISB` are no-ops because execution and device callbacks are single-threaded and ordered.
- `BKPT` stops the emulator instead of entering DebugMonitor.
- Instructions take one cycle regardless of data, branch direction, memory target, or pipeline effects.
- CPACR is modeled as system-control state, but the interpreter does not currently gate FP instructions on FPU enable.
- Scalar FP arithmetic uses the host's IEEE-754 `float` operations. FPSCR rounding modes, exception enables/status, flush-to-zero, default-NaN mode, signaling-NaN distinctions, and NaN payload preservation are not modeled. Finite in-range `VCVT` results truncate toward zero; unsigned out-of-range values clamp, while signed invalid/out-of-range conversion produces `0x80000000`.
- Division-by-zero trapping, unaligned-access trapping through CCR, and privilege checks for special-register operations are not enforced.
- A decode or memory failure stops with a structured diagnostic; it is not generally converted into UsageFault, BusFault, or HardFault escalation.
- Extended floating-point exception frames are eager. Lazy stacking behavior is not modeled.

## Unsupported instruction areas

Unsupported encodings fail as `unimplemented-instruction`; they are not treated as NOPs. Notable unsupported areas include:

- double-precision FP arithmetic, core-register-pair FP moves, VMSR/general VMRS forms, fixed-point conversions, and FP encodings not listed above;
- exclusive and acquire/release operations such as `LDREX` and `STREX`;
- signed or accumulating long multiplies not listed above;
- DSP, SIMD, saturating, packing, and parallel add/subtract families other than `UADD8` and `SEL`;
- `RBIT`, signed bitfield extract, bitfield insert/clear, and table branches;
- immediate `BLX`, coprocessor encodings outside the listed VFP subset, preload hints, and cache-maintenance operations;
- MPU behavior, unprivileged memory permission enforcement, and debug instruction semantics.

Tests live in `tests/unit/decoder_test.cpp`, `tests/unit/alu_test.cpp`,
`tests/unit/cpu_step_test.cpp`, `tests/unit/cortexm_test.cpp`,
`tests/unit/exceptions_test.cpp`, and `tests/unit/startup_runtime_test.cpp`. The last
test runs the committed `startup_runtime.elf` reset handler through `.data` copying,
`.bss` zeroing, `main`, and its `BKPT` boundary.
