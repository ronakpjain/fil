# STM32G4 Emulator Implementation Plan

## Implementation status: initial useful emulator complete

The implementation described by this plan was completed through WP-6 and milestones
M4-M9 on 2026-07-17. The assumptions and task ordering below are retained as the
historical design record; the current supported boundary is maintained in
[Thumb instruction coverage](thumb_instruction_coverage.md) and
[STM32G4 peripheral coverage](stm32g4_peripheral_coverage.md).

The Definition of Done in section 22 is satisfied by the following checked evidence:

- committed, hermetic ELF fixtures exercise split LMA/VMA loading and run reset code
  through `.data` copying, `.bss` zeroing, `main`, and a deterministic `BKPT`;
- the normal and combined ASan/UBSan CTest suites pass, and regenerating the fixture
  ELFs with the ARM GNU toolchain leaves those tests passing;
- two identical fixture runs produce byte-identical normalized JSON-lines traces;
- the reproducible static gate scans 55,068 objdump-recognized instructions across
  all seven configured PER ELFs with zero unsupported or address/width mismatches;
  a targeted audit separately validates mnemonic kinds and operand fields for 2,268
  ADDW/scalar-VFP instances;
- every configured board completes a 10 ms strict-MMIO smoke run with zero top-level
  unknown MMIO addresses;
- `g4_testing` reaches FreeRTOS scheduling and completes one simulated second at the
  requested time boundary after 16,000,000 instructions, with zero unknown MMIO;
- the configured six-board CAN world runs deterministically, while unit and CLI tests
  cover FDCAN message-RAM TX/RX, filtering, interrupts, and scheduled external CAN
  injection; the one-second world runs in a measured 0.65--0.74 s on the development
  host with exact accelerated/unaccelerated logical results; and
- GPIO/USART traces, binary USART TX logs, deterministic ADC input, timers, DMA/SPI,
  watchdog reset requests, configuration-only board changes, diagnostics, and
  coverage documentation are implemented and tested.

This status denotes completion of the plan's **initial useful release**, not complete
ARMv7E-M or STM32G474 conformance. Deliberate timing, instruction, exception, and
peripheral simplifications remain explicitly documented in the two coverage pages.

## 0. Assumptions, Verified Baseline, and Scope

### Assumptions

- The emulator is a new standalone C++20 project; this repository currently contains only this plan.
- The first host target is macOS, but emulator code must avoid host-endian, host-pointer-width, and Apple-only assumptions so Linux can be added without redesign.
- `arm-none-eabi-gcc`, `arm-none-eabi-readelf`, and `arm-none-eabi-objdump` are available for fixture generation and diagnostics. They are test/build-time tools, not runtime dependencies of `fil`.
- The STM32G474 reference manual and ARMv7-M Architecture Reference Manual are the behavioral authorities. CMSIS headers may supply addresses and masks, but must not be copied into public emulator APIs.
- Determinism is more important than wall-clock speed. Given the same ELF, config, seed, and limits, observable trace records must have the same order and values.
- The initial target is the existing PER firmware build, not every valid ARMv7E-M binary or every STM32G4 part.

### Baseline verified from current PER artifacts

The following was checked against the existing artifacts on 2026-07-13:

- All seven target ELFs exist and are ELF32, little-endian ARM executables.
- `g4_testing.elf` reports ARM EABI v5, Thumb-2, ARMv7E-M, VFPv4-D16, and hard-float register arguments.
- The normal linker script defines:
  - flash at `0x08000000`, 512 KiB;
  - SRAM at `0x20000000`, 128 KiB;
  - CCM SRAM at `0x10000000`, 32 KiB.
- Current ELFs use separate load and run addresses for `.data`. For example, `g4_testing.elf` has a RAM `PT_LOAD` with `p_vaddr=0x20000000` and `p_paddr=0x0800356c`. The loader must preserve the initializer bytes in flash at the physical/load address so reset code can copy them into SRAM.
- Current application entry points and vector tables are in the `0x08000000` image. Bootloader-offset linker scripts also exist and therefore vector base and flash image base cannot be hardcoded permanently.
- PER FreeRTOS is configured for a 1 kHz tick, preemption, 4 NVIC priority bits, `BASEPRI` masking, static allocation, and DWT cycle-counter runtime statistics.
- PHAL G4 currently exercises RCC, GPIO, ADC, DMA/DMAMUX, FDCAN, SPI, and USART directly through CMSIS register structures.

Record future artifact observations in the coverage documents rather than silently expanding scope.

### Scope

**In scope for the first useful release**

- Load and inspect unstripped or stripped bare-metal ELF32 ARM images.
- Interpret the ARMv7E-M Thumb/Thumb-2 and FPv4-SP-D16 instructions actually emitted by the PER build.
- Boot a single STM32G474 image through reset, C runtime initialization, `main`, and FreeRTOS scheduling.
- Provide deterministic behavioral models for the peripherals needed by the target boards.
- Run multiple isolated MCU instances connected through explicit virtual devices and CAN buses.
- Produce diagnostics sufficient to turn every unsupported instruction, MMIO access, and spin into a focused regression test.

**Out of scope until after the initial useful release**

- Binary compatibility with non-ARM ELFs, ARM/A32 state, TrustZone, Cortex-M7 cache behavior, or non-G4 STM32 families.
- Cycle-, bus-, or pin-electrical accuracy; analog effects; oscillator drift; CAN arbitration timing fidelity.
- Self-programming flash, boot ROM behavior, and bootloader update flows unless a target firmware test requires them.
- GDB remote protocol, semihosting beyond a diagnostic stub, JIT compilation, and real-time host synchronization.
- Security isolation: firmware and config files are treated as trusted local inputs in the initial version.

## 1. Goals

### Primary goals

1. Load existing STM32G4 ARM `.elf` binaries, for example:
   - `/Users/ronak/coding/PER/Projects/firmware/output/g4_testing/g4_testing.elf`
   - `/Users/ronak/coding/PER/Projects/firmware/output/dashboard/dashboard.elf`
   - `/Users/ronak/coding/PER/Projects/firmware/output/main_module/main_module.elf`
   - `/Users/ronak/coding/PER/Projects/firmware/output/torque_vector/torque_vector.elf`
   - `/Users/ronak/coding/PER/Projects/firmware/output/a_box/a_box.elf`
   - `/Users/ronak/coding/PER/Projects/firmware/output/front_driveline/front_driveline.elf`
   - `/Users/ronak/coding/PER/Projects/firmware/output/rear_driveline/rear_driveline.elf`
2. Emulate enough ARMv7E-M / Cortex-M4F to execute firmware compiled with:
   - `-mcpu=cortex-m4`
   - `-mthumb`
   - `-mfpu=fpv4-sp-d16`
   - `-mfloat-abi=hard`
3. Model STM32G474 memory and peripheral MMIO well enough for firmware startup, FreeRTOS, PHAL, CAN, UART, timers, ADC, SPI, DMA, GPIO, and watchdog behavior.
4. Support board-level mocking by config file: GPIO inputs, ADC values, UART devices, SPI devices, CAN buses, and scripted faults.
5. Eventually run multiple board firmwares at once, connected by shared virtual CAN buses.

### Non-goals for initial versions

- Cycle-accurate Cortex-M timing.
- Bit-perfect STM32G4 peripheral implementation.
- Electrical/analog simulation accuracy.
- JTAG/SWD debug protocol.
- Full GDB server initially.
- Running arbitrary operating systems; target is bare-metal / FreeRTOS firmware.

---

## 2. High-Level Architecture

```text
fil
├── CLI
│   ├── inspect ELF / config
│   ├── run one board
│   ├── run network of boards
│   └── emit traces/logs
│
├── Emulator instance, one per board
│   ├── CPU core: custom Cortex-M4F Thumb/Thumb-2 interpreter
│   ├── Memory bus: flash, SRAM, aliases, MMIO dispatch
│   ├── Cortex-M system peripherals: SCB, NVIC, SysTick, DWT stubs
│   └── STM32G4 peripherals: RCC, GPIO, USART, FDCAN, TIM, ADC, SPI, DMA, etc.
│
├── Board model
│   ├── ELF path
│   ├── MCU definition
│   ├── pin/peripheral configuration
│   ├── attached virtual devices
│   └── scripted inputs/faults
│
└── Shared simulation world
    ├── deterministic simulated time
    ├── event queue
    ├── shared CAN buses
    ├── trace sinks
    └── multi-board scheduler
```

The emulator is split into reusable layers:

1. **CPU layer** knows only about ARM architectural state and memory reads/writes.
2. **Memory/bus layer** maps addresses to RAM/flash/MMIO devices.
3. **Cortex-M system layer** handles exceptions, SysTick, NVIC, SCB, and control registers.
4. **STM32G4 layer** implements microcontroller peripheral register blocks.
5. **Board/device layer** models external world behavior.

---

## 3. Repository Layout

Initial target layout:

```text
CMakeLists.txt
README.md
docs/
  STM32G4_EMULATOR_IMPLEMENTATION_PLAN.md
  architecture.md
  thumb_instruction_coverage.md
  stm32g4_peripheral_coverage.md
configs/
  mcus/
    stm32g474retx.json
  boards/
    g4_testing.json
    dashboard.json
    main_module.json
    torque_vector.json
    a_box.json
    front_driveline.json
    rear_driveline.json
  networks/
    per_vehicle.json
include/fil/
  common/types.hpp
  common/result.hpp
  common/log.hpp
  config/config.hpp
  elf/elf_loader.hpp
  mem/address.hpp
  mem/memory_bus.hpp
  cpu/cortex_m4.hpp
  cpu/instruction.hpp
  cpu/decoder.hpp
  cpu/fpu.hpp
  cortexm/exceptions.hpp
  cortexm/nvic.hpp
  cortexm/scb.hpp
  cortexm/systick.hpp
  stm32g4/stm32g4.hpp
  stm32g4/peripheral.hpp
  stm32g4/rcc.hpp
  stm32g4/gpio.hpp
  stm32g4/usart.hpp
  stm32g4/fdcan.hpp
  stm32g4/tim.hpp
  stm32g4/adc.hpp
  stm32g4/spi.hpp
  stm32g4/dma.hpp
  devices/can_bus.hpp
  devices/uart_device.hpp
  devices/spi_device.hpp
  sim/event_loop.hpp
  sim/trace.hpp
src/
  main.cpp
  cli/cli.cpp
  config/config.cpp
  elf/elf_loader.cpp
  mem/memory_bus.cpp
  mem/region.cpp
  cpu/cortex_m4.cpp
  cpu/decoder.cpp
  cpu/execute.cpp
  cpu/fpu.cpp
  cortexm/exceptions.cpp
  cortexm/nvic.cpp
  cortexm/scb.cpp
  cortexm/systick.cpp
  stm32g4/stm32g4.cpp
  stm32g4/rcc.cpp
  stm32g4/flash.cpp
  stm32g4/pwr.cpp
  stm32g4/gpio.cpp
  stm32g4/usart.cpp
  stm32g4/fdcan.cpp
  stm32g4/tim.cpp
  stm32g4/adc.cpp
  stm32g4/spi.cpp
  stm32g4/dma.cpp
  stm32g4/watchdog.cpp
  devices/can_bus.cpp
  devices/uart_device.cpp
  devices/spi_device.cpp
  sim/event_loop.cpp
  sim/trace.cpp
tests/
  unit/
  firmware_src/
  firmware_elf/
  integration/
tools/
  build_test_firmware.sh
  trace_decode.py
```

Use C++20 for the emulator. Use C where useful only for small helper test firmware, not for emulator internals unless a pure-C module is clearly simpler.

---

## 4. Build System

### CMake targets

```text
fil                 Main CLI executable
fil_core            Static library: CPU, memory, peripherals, config
fil_tests           Unit tests
firmware_tests        Optional target to build tiny ARM ELF test fixtures
```

### Dependency policy

Because this is a standalone emulator, keep dependencies minimal.

Allowed initially:

- C++ standard library.
- CMake.
- Optional small test framework such as Catch2 if vendored or fetched explicitly.

Avoid initially:

- CPU emulation engines.
- System simulation engines.
- Large framework dependencies.

For config parsing, prefer one of:

1. **JSON subset parser implemented in repo**, enough for our config files.
2. Later optional support for YAML via a small dependency if desired.

Start with `.json` config files to reduce parser complexity.

---

## 5. CLI Design

### Commands

```bash
fil inspect-elf <firmware.elf>
fil inspect-config <board.json>
fil run <board.json> [--duration-ms N] [--max-instructions N] [--trace trace.jsonl]
fil run-network <network.json> [--duration-ms N] [--trace trace.jsonl]
fil disasm-window <firmware.elf> --addr 0x08001234 --count 32
```

### Example

```bash
fil run configs/boards/g4_testing.json \
  --duration-ms 1000 \
  --max-instructions 50000000 \
  --trace traces/g4_testing.jsonl
```

### Runtime modes

- `--strict-mmio`: abort on unknown MMIO access.
- `--lenient-mmio`: log unknown MMIO and return default values. This should be default early on.
- `--strict-instr`: abort on unimplemented instruction. This should always be true initially.
- `--trace-mmio`: log every MMIO access.
- `--trace-instr`: log every instruction; expensive but useful.
- `--detect-spin`: report probable wait loops.

---

## 6. ELF Loader

Implement a small ELF32 loader in `src/elf/elf_loader.cpp`.

### Supported ELF format

- ELF32.
- Little-endian.
- ARM machine type.
- Program headers with `PT_LOAD`.
- Optional symbol table loading for diagnostics.
- Optional DWARF ignored initially.

### Loader flow

1. Read ELF header.
2. Validate:
   - magic `0x7F 'E' 'L' 'F'`
   - class = 32-bit
   - endian = little
   - machine = ARM
3. Iterate program headers.
4. For each `PT_LOAD`:
   - validate `p_filesz <= p_memsz`, file bounds, integer overflow, and target address range;
   - treat `p_paddr` as the load-memory address and `p_vaddr` as the runtime address for these bare-metal images;
   - copy file bytes into the load image at `p_paddr`;
   - if `p_paddr == p_vaddr`, expose those bytes directly at the runtime address;
   - if `p_paddr != p_vaddr`, leave the runtime RAM at reset value and let startup code perform the `.data` copy;
   - reserve/validate `p_memsz` at the runtime address, but do not eagerly zero `.bss`; reset/startup code must do that;
   - record R/W/X flags for diagnostics now, with permission enforcement enabled later by policy.
5. Reject overlapping file-backed load ranges unless their bytes are identical. Give the diagnostic both program-header indexes and address ranges.
6. Determine vector table address:
   - default `0x08000000`.
   - allow config override for bootloader-offset images.
7. Read:
   - initial MSP from `[vector_base + 0]`
   - reset handler from `[vector_base + 4]`
8. Validate vectors before execution:
   - MSP is 8-byte aligned and lies in configured writable memory;
   - reset handler has Thumb bit set and resolves to executable flash;
   - vector table is at least 128-byte aligned for the initial target.
9. Initialize CPU:
   - `MSP = initial_msp`
   - active SP = MSP
   - `PC = reset_handler & ~1u`
   - Thumb state = `reset_handler & 1`
   - `xPSR.T = 1`

### Output from `inspect-elf`

Print:

```text
ELF: dashboard.elf
entry: 0x08000abc
load segments:
  0x08000000..0x0801c123 R-X file=...
  0x20000000..0x20001234 RW- file=...
vector table: 0x08000000
initial MSP: 0x2001fff0
reset handler: 0x08000451
symbols loaded: yes/no
```

### Implementation details

Create structs matching ELF layout exactly using fixed-width types and manual little-endian reads. Do **not** rely on host struct packing.

```cpp
struct ElfLoadSegment {
  uint32_t vaddr;
  uint32_t mem_size;
  uint32_t file_size;
  uint32_t file_offset;
  bool readable;
  bool writable;
  bool executable;
};
```

---

## 7. Memory System

### Address regions for STM32G474

Base memory map for STM32G474RETx-class targets:

```text
0x00000000..0x0007ffff  boot alias, initially alias flash
0x08000000..0x0807ffff  flash, 512 KiB default
0x10000000..0x10007fff  CCM SRAM, 32 KiB default
0x1fff0000..0x1fffffff  system memory / option bytes stubs
0x20000000..0x2001ffff  SRAM, 128 KiB default, configurable
0x40000000..0x5fffffff  STM32 peripheral MMIO
0xe0000000..0xe00fffff  Cortex-M system control space
```

Actual sizes should come from MCU config:

```json
{
  "name": "stm32g474retx",
  "flash_base": "0x08000000",
  "flash_size": "512K",
  "sram_base": "0x20000000",
  "sram_size": "128K",
  "ccm_sram_base": "0x10000000",
  "ccm_sram_size": "32K"
}
```

### Memory API

```cpp
class MemoryBus {
public:
  uint8_t  read8(uint32_t addr);
  uint16_t read16(uint32_t addr);
  uint32_t read32(uint32_t addr);
  uint64_t read64(uint32_t addr);

  void write8(uint32_t addr, uint8_t value);
  void write16(uint32_t addr, uint16_t value);
  void write32(uint32_t addr, uint32_t value);
  void write64(uint32_t addr, uint64_t value);

  void mapRam(uint32_t base, uint32_t size, std::string name);
  void mapRom(uint32_t base, std::vector<uint8_t> data, std::string name);
  void mapAlias(uint32_t alias_base, uint32_t target_base, uint32_t size);
  void mapMmio(uint32_t base, uint32_t size, MmioDevice* device, std::string name);
};
```

### Unaligned accesses

Cortex-M supports some unaligned word/halfword accesses depending on configuration. Implement them in the memory bus by composing bytes, unless `SCB->CCR.UNALIGN_TRP` is set. If `UNALIGN_TRP` is set and unaligned access occurs, raise UsageFault once fault handling exists; before that, abort with diagnostic.

### Endianness

All target memory is little-endian.

### Access contract and faults

Every bus access carries an access type (`instruction_fetch`, `data_read`, `data_write`, or `debug`) and width. The bus must:

1. check address-range overflow before lookup;
2. reject an access crossing two regions unless it is an explicitly supported unaligned RAM/ROM access assembled byte-by-byte;
3. dispatch MMIO once at the original width rather than decomposing a 32-bit register write into bytes;
4. return a typed `BusFault` containing address, width, access type, PC, and region name;
5. avoid C++ exceptions in the hot path; use an explicit fault/result value that the CPU converts into an architectural fault or fatal bring-up diagnostic.

Reset values are explicit: SRAM/CCM SRAM start zero-filled for deterministic runs, flash starts `0xff` outside loaded bytes, and unknown lenient MMIO reads return a configurable value defaulting to zero. Trace each unknown MMIO address once plus a final count to avoid unbounded logs.

### Region lookup

Use a sorted vector of non-overlapping memory regions at first. Validate overlap and 32-bit wraparound at map time. If performance becomes an issue, add page-indexed fast lookup:

```cpp
static constexpr uint32_t PAGE_BITS = 12;
std::array<Region*, 1 << 20> page_table; // optional sparse structure later
```

---

## 8. CPU Core: Cortex-M4F Interpreter

Implement a custom ARMv7E-M Thumb/Thumb-2 interpreter.

### Architectural state

```cpp
struct CpuState {
  uint32_t r[16];       // r0-r15, where r13=visible SP, r14=LR, r15=PC
  uint32_t xpsr;        // APSR + IPSR + EPSR fields

  uint32_t msp;
  uint32_t psp;
  uint32_t primask;
  uint32_t basepri;
  uint32_t faultmask;
  uint32_t control;

  bool thumb;
  bool halted;

  // FPU: FPv4-SP-D16. D16 means s0-s31 / d0-d15.
  float s[32];
  uint32_t fpscr;

  // IT block execution state, mirrored with xPSR.IT bits.
  uint8_t it_state;
};
```

Keep `msp` and `psp` separate from `r[13]`; helper methods expose the active SP based on `CONTROL.SPSEL` and handler/thread mode.

### PC semantics

Thumb instructions observe PC as current instruction address + 4 for many operations. Implement helpers:

```cpp
uint32_t currentInstrAddr() const;
uint32_t architecturalPcForRead() const; // usually instr_addr + 4, aligned as needed
```

Branch writes must clear bit 0 internally but validate Thumb bit rules:

```cpp
void branchWritePc(uint32_t target) {
  if ((target & 1u) == 0) trapInvalidState();
  r[15] = target & ~1u;
  thumb = true;
}
```

### Execution loop

```cpp
while (!state.halted && budget.notExpired()) {
  uint32_t pc = state.r[15];
  uint16_t hw1 = mem.read16(pc);

  DecodedInstruction insn;
  if (is32BitThumbPrefix(hw1)) {
    uint16_t hw2 = mem.read16(pc + 2);
    insn = decode32(hw1, hw2);
    state.r[15] += 4;
  } else {
    insn = decode16(hw1);
    state.r[15] += 2;
  }

  if (conditionPasses(insn.condition, state.xpsr, state.it_state)) {
    execute(insn);
  }

  advanceItStateIfNeeded();
  cycles += estimateCycles(insn);
  system.tick(cycles);
  exceptionController.checkAndEnterPendingException(state);
}
```

Increment PC before execute, then branch instructions overwrite it. This simplifies exception diagnostics and follows common interpreter structure.

### Flags

Implement helpers for all flag-affecting operations:

```cpp
struct AddResult { uint32_t value; bool n,z,c,v; };
AddResult addWithCarry(uint32_t x, uint32_t y, bool carry_in);
ShiftResult shiftC(uint32_t value, ShiftType type, uint32_t amount, bool old_carry);
```

Update APSR bits:

- N bit 31
- Z bit 30
- C bit 29
- V bit 28
- Q bit 27 for saturation later

### Instruction decoder design

Use a table-driven decoder with explicit masks, not a giant opaque switch. Each entry:

```cpp
struct DecodePattern16 {
  uint16_t mask;
  uint16_t value;
  InstrKind kind;
  DecodeFn16 decode;
};

struct DecodePattern32 {
  uint32_t mask;
  uint32_t value;
  InstrKind kind;
  DecodeFn32 decode;
};
```

The decoded instruction should be semantic, not just raw bits:

```cpp
struct DecodedInstruction {
  InstrKind kind;
  Condition cond;
  uint8_t rd, rn, rm, ra;
  uint32_t imm;
  ShiftType shift_type;
  uint8_t shift_amount;
  bool set_flags;
  bool index;
  bool add;
  bool writeback;
  bool is_32bit;
  uint32_t raw;
};
```

### Initial instruction coverage

Implement in the order firmware is likely to hit them.

#### Milestone A: startup and C runtime

- `MOV`, `MOVS`, `MOVW`, `MOVT`
- `ADD`, `ADDS`, `ADC`, `SUB`, `SUBS`, `SBC`, `RSB`
- `CMP`, `CMN`, `TST`
- `AND`, `ORR`, `EOR`, `BIC`, `MVN`, `ORN`
- `LSL`, `LSR`, `ASR`, `ROR`, `RRX`
- `LDR`, `LDR.W`, literal `LDR`
- `STR`, `STR.W`
- `LDRB`, `LDRH`, `LDRSB`, `LDRSH`
- `STRB`, `STRH`
- `LDM`, `STM`
- `PUSH`, `POP`
- `B`, conditional `B`, `BL`, `BLX`, `BX`
- `CBZ`, `CBNZ`
- `IT`
- `NOP`, `WFI`, `WFE`, `SEV`
- `DMB`, `DSB`, `ISB`
- `MRS`, `MSR`
- `SVC`, `BKPT`

#### Milestone B: optimized GCC output

- `MLA`, `MLS`, `MUL`
- `UMULL`, `SMULL`, `UMLAL`, `SMLAL`
- `UDIV`, `SDIV`
- `REV`, `REV16`, `REVSH`, `RBIT`
- `CLZ`
- `UXTB`, `UXTH`, `SXTB`, `SXTH`
- `UBFX`, `SBFX`, `BFI`, `BFC`
- `SSAT`, `USAT`
- `TBB`, `TBH`
- `LDRD`, `STRD`
- exclusive accesses: `LDREX`, `STREX`, `CLREX`

#### Milestone C: FPU / hard-float support

PER firmware is built with hard-float (`-mfpu=fpv4-sp-d16 -mfloat-abi=hard`), so the emulator must support common VFP instructions early.

Implement:

- FPU register file: `s0-s31`, `d0-d15` aliases.
- `VMOV` core<->single, immediate, register-register.
- `VLDR`, `VSTR` single.
- `VPUSH`, `VPOP`.
- `VLDM`, `VSTM` single/double register lists as emitted by context switches.
- `VADD.F32`, `VSUB.F32`, `VMUL.F32`, `VDIV.F32`.
- `VMLA.F32`, `VMLS.F32`, `VNMLA`, `VNMLS` if encountered.
- `VCMP.F32`, `VCMPE.F32`.
- `VMRS APSR_nzcv, FPSCR` and `VMSR FPSCR`.
- `VCVT` between `s32/u32` and `f32`.
- `VSQRT.F32` if encountered.
- `VABS.F32`, `VNEG.F32`.

Floating-point can initially use host `float` semantics. That is not perfectly IEEE-identical for all FPSCR modes, but is enough for board-level firmware tests.

### Undefined/unimplemented instruction behavior

On unimplemented instruction:

1. Print PC, raw halfwords, current function symbol if available.
2. Print nearby disassembly-like hex window.
3. Abort.
4. Add a unit test for that instruction before implementing it.

Example diagnostic:

```text
unimplemented instruction
  pc=0x0800349a
  raw16=0xed9f raw32=0xed9f0a12
  symbol=control_loop+0x42
  r0=... r1=... sp=...
```

---

## 9. Cortex-M Exception and Interrupt Model

This is required for FreeRTOS. Implement after startup instructions but before serious firmware execution.

### Exceptions to support

Core exceptions:

```text
1  Reset
2  NMI              stub
3  HardFault        diagnostic handler
4  MemManage        optional later
5  BusFault         optional later
6  UsageFault       optional later
11 SVCall           required by FreeRTOS
14 PendSV           required by FreeRTOS
15 SysTick          required by FreeRTOS
```

External IRQs begin at exception number 16. STM32 IRQ number `n` maps to exception `16 + n`.

### Vector table

- Default vector base from ELF/config, usually `0x08000000`.
- Support `SCB->VTOR` writes so firmware can relocate vector table.
- On exception entry, handler address = `mem.read32(VTOR + exception_number * 4)`.
- Handler address must have Thumb bit set.

### Exception entry stack frame

Basic Cortex-M frame, pushed to active stack:

```text
r0
r1
r2
r3
r12
lr
pc
xpsr
```

The frame occupies ascending addresses from the new SP: `r0`, `r1`, `r2`, `r3`, `r12`, `lr`, return `pc`, `xpsr`. Do not implement this as repeated ambiguous pushes. Compute the final SP once, then write fixed offsets:

```cpp
void enterException(ExceptionNumber exc) {
  const uint32_t old_sp = activeSp();
  const bool add_padding = ccr_stkalign && ((old_sp & 0x7u) != 0);
  const uint32_t frame_bytes = 8u * 4u + (add_padding ? 4u : 0u);
  const uint32_t new_sp = old_sp - frame_bytes;

  mem.write32(new_sp + 0x00, r0);
  mem.write32(new_sp + 0x04, r1);
  mem.write32(new_sp + 0x08, r2);
  mem.write32(new_sp + 0x0c, r3);
  mem.write32(new_sp + 0x10, r12);
  mem.write32(new_sp + 0x14, lr);
  mem.write32(new_sp + 0x18, return_pc | 1u);
  mem.write32(new_sp + 0x1c, stackedXpsr(add_padding));
  setActiveSp(new_sp);

  lr = makeExcReturn(...);
  setIpsr(exc);
  pc = vectorHandler & ~1u;
  mode = Handler;
}
```

Before mutating architectural state, validate the entire stack range and handler vector so a failed entry does not leave a half-written frame. Track active exceptions as a stack; NVIC active bits and `ICSR.VECTACTIVE` derive from it.

### Exception return

Branching to `0xfffffff1`, `0xfffffff9`, `0xfffffffd`, or FPU variants performs exception return, not a normal branch.

Implement `BX LR` / `POP {..., PC}` detection for EXC_RETURN values:

- Restore basic frame from MSP or PSP based on EXC_RETURN bits.
- Restore thread/handler mode.
- Restore PC/xPSR.
- Restore FPU extended frame if EXC_RETURN indicates one is present.

### FreeRTOS-specific requirements

FreeRTOS Cortex-M ports rely on:

- `SVC` to start first task.
- `PendSV` for context switching.
- `SysTick` for ticks.
- `MRS/MSR PSP/MSP/BASEPRI/PRIMASK/CONTROL`.
- FPU context save/restore instructions if hard-float.
- `DSB`, `ISB` barriers as no-op ordering points.

Implement these before expecting firmware tasks to run correctly.

### Priority/masking model

Initial model:

- Track enabled and pending bits.
- Track configurable priority bytes.
- Respect `PRIMASK`: masks all configurable external interrupts.
- Respect `BASEPRI`: masks interrupts with numerically equal/lower urgency.
- System handlers `SVC`, `PendSV`, `SysTick` use SHPR priorities.

Tail-chaining and late-arrival can be ignored initially.

---

## 10. Cortex-M System Control Space

Map `0xE0000000..0xE00FFFFF` to custom system devices.

### SysTick: `0xE000E010`

Registers:

```text
0x00 CTRL
0x04 LOAD
0x08 VAL
0x0c CALIB
```

Behavior:

- `CTRL.ENABLE`: enable counter.
- `CTRL.TICKINT`: pend SysTick exception on wrap.
- `CTRL.CLKSOURCE`: can be accepted; no cycle-accurate distinction initially.
- `CTRL.COUNTFLAG`: set on wrap and clear-on-read.
- `LOAD`: reload value.
- `VAL`: current value, write clears current value.

Implementation approach:

- Maintain simulated cycle counter.
- On each CPU step batch, call `systick.advance(cycles)`.
- When counter reaches zero, reload and pend SysTick if enabled.

### NVIC: `0xE000E100` region

Implement:

```text
ISER  enable set
ICER  enable clear
ISPR  pending set
ICPR  pending clear
IABR  active bits, optional
IPR   priorities
STIR  software trigger interrupt, optional
```

### SCB: `0xE000ED00`

Implement at least:

```text
CPUID    read plausible Cortex-M4 ID
ICSR     pending bits for PendSV/SysTick, VECTACTIVE
VTOR     vector table offset
AIRCR    priority grouping / system reset request
SCR      sleep control
CCR      config flags, including UNALIGN_TRP
SHPR1-3  system handler priorities
SHCSR    fault enables/status, stub initially
CFSR/HFSR/MMFAR/BFAR diagnostic stubs
CPACR    FPU access enable
```

Important behavior:

- `ICSR.PENDSVSET` pends PendSV.
- `ICSR.PENDSVCLR` clears PendSV.
- `ICSR.PENDSTSET` pends SysTick.
- `AIRCR.SYSRESETREQ` requests simulated reset.
- `CPACR` enables FPU access. If firmware executes FPU before CPACR enables it, raise UsageFault or log diagnostic.

### DWT/ITM stubs

Firmware may touch debug registers. Stub:

- DWT cycle counter.
- DEMCR.
- ITM stimulus ports if debug printing uses SWO.

---

## 11. Simulated Time and Event Loop

### Principles

- Deterministic by default.
- Not real-time unless requested later.
- CPU instruction count drives approximate cycles.
- Peripherals schedule events in simulated time.

### Core types

```cpp
using SimTimeNs = uint64_t;

struct Event {
  SimTimeNs at;
  uint64_t sequence;
  std::function<void()> callback;
};

class EventLoop {
public:
  EventId scheduleAfter(SimTimeNs delta, Callback cb);
  void cancel(EventId id);
  void runDueEvents(SimTimeNs now);
};
```

Events are ordered by `(at, sequence)`, where `sequence` is monotonically assigned. Callbacks may schedule new events at the current timestamp; those run after already-queued events at that timestamp. Callbacks must not retain raw pointers to boards or peripherals that can be destroyed; board teardown cancels owned event IDs. Add a maximum same-time event count to diagnose zero-delay livelocks.

### CPU/time integration

Start simple:

- Each 16-bit instruction = 1 cycle.
- Each 32-bit instruction = 1-2 cycles.
- Loads/stores = 2 cycles.
- Branch = 2 cycles.
- FPU = configurable approximate cycles.

Convert cycles to time using current system clock estimate. RCC mock maintains `sysclk_hz`, initially default 16 MHz and updated approximately when PLL config succeeds.

---

## 12. STM32G4 Peripheral MMIO Model

Each peripheral derives from:

```cpp
class MmioDevice {
public:
  virtual uint32_t read(uint32_t offset, AccessSize size) = 0;
  virtual void write(uint32_t offset, AccessSize size, uint32_t value) = 0;
  virtual const char* name() const = 0;
};
```

Keep each peripheral register block as simple `uint32_t regs[]` plus custom hooks for side effects.

### Peripheral base addresses to support early

From STM32G4 CMSIS-style layout:

```text
PERIPH_BASE   0x40000000
APB1          0x40000000
APB2          0x40010000
AHB1          0x40020000
AHB2          0x48000000

FDCAN1        0x40006400
PWR           0x40007000
TIM1          0x40012c00
SPI1          0x40013000
USART1        0x40013800
RCC           0x40021000
FLASH_R       0x40022000
GPIOA         0x48000000
GPIOB         0x48000400
GPIOC         0x48000800
GPIOD         0x48000c00
GPIOE         0x48001000
GPIOF         0x48001400
GPIOG         0x48001800
ADC1          0x50000000
```

Add exact additional bases as traces show them.

---

## 13. Required Peripherals

### 13.1 RCC

Purpose: prevent firmware from hanging during clock and peripheral enable setup.

Implement register set enough for PHAL G4 clock code:

- `CR`
- `ICSCR`
- `CFGR`
- `PLLCFGR`
- `CIER/CIFR/CICR`
- `AHB1ENR`, `AHB2ENR`, `AHB3ENR`
- `APB1ENR1`, `APB1ENR2`, `APB2ENR`
- reset registers
- `CCIPR`, `BDCR`, `CSR`, `CRRCR`, `CCIPR2` as stubs

Behavior:

- If firmware sets `HSION`, set `HSIRDY` immediately.
- If firmware sets `HSEON`, set `HSERDY` immediately unless config says no HSE.
- If firmware sets `PLLON`, set `PLLRDY` immediately.
- Track `SW` clock source writes and reflect selected source in `SWS`.
- Peripheral clock enable registers should store written bits.
- Unknown reserved bits can be masked later; initially preserve writes.

Validation:

- Clock setup exits without spin.
- Trace shows PLL/HSE ready bits returned.

### 13.2 FLASH register interface

Register block at `FLASH_R_BASE`.

Implement:

- `ACR`: latency, prefetch, caches writable.
- `SR`: ready/not busy default.
- `CR`: lock/program/erase stubs.
- Option byte registers as read/write stubs.

Behavior:

- Always not busy unless simulating flash operation.
- Writes to flash memory itself should fault/log unless firmware intentionally programs flash; later add flash programming model for bootloader tests.

### 13.3 PWR

Implement enough power-control registers to be writable/readable. Ready/status bits should be permissive.

### 13.4 GPIO

Register offsets:

```text
0x00 MODER
0x04 OTYPER
0x08 OSPEEDR
0x0c PUPDR
0x10 IDR
0x14 ODR
0x18 BSRR
0x1c LCKR
0x20 AFRL
0x24 AFRH
0x28 BRR
```

Behavior:

- `ODR` stores output state.
- `BSRR` lower 16 bits set pins; upper 16 bits reset pins.
- `BRR` resets pins.
- `IDR` reads external pin input if configured, otherwise can mirror ODR for output pins.
- Writes log pin transitions when tracing enabled.

Config example:

```json
{
  "gpio": {
    "PA0": { "mode": "input", "value": 1 },
    "PC13": { "mode": "output", "trace": true }
  }
}
```

### 13.5 USART / UART

Implement USART1/2/3 and LPUART as encountered.

Registers:

- `CR1`, `CR2`, `CR3`
- `BRR`
- `GTPR`, `RTOR`, `RQR`
- `ISR`, `ICR`
- `RDR`, `TDR`

Behavior:

- `ISR.TXE_TXFNF` and `ISR.TC` should be set when TX is ready.
- Write to `TDR` appends byte to TX log/device.
- RX queue configured from board/device.
- If RX queue non-empty, set `RXNE_RXFNE`.
- Reading `RDR` pops one byte and updates flags.
- If `CR1.RXNEIE` enabled and RX data exists, pend USART IRQ.
- If `CR1.IDLEIE` enabled, schedule IDLE event after configured gap.
- `ICR` clears error/IDLE flags.

Device abstraction:

```cpp
class UartEndpoint {
public:
  virtual void onTxByte(uint8_t byte, SimTimeNs now) = 0;
  virtual std::optional<uint8_t> nextRxByte(SimTimeNs now) = 0;
};
```

Initial endpoints:

- log-only endpoint
- scripted RX/TX endpoint
- Nextion display mock later
- u-blox GPS mock later

### 13.6 TIM timers

Implement general-purpose/basic timer behavior for timers that firmware touches.

Registers:

- `CR1`, `CR2`, `SMCR`, `DIER`, `SR`, `EGR`
- `CNT`, `PSC`, `ARR`
- `CCR1..CCR4`
- `CCMR1/2`, `CCER`, `BDTR` for PWM stubbing

Behavior:

- If `CR1.CEN`, counter advances with simulated time.
- `PSC` and `ARR` determine update period.
- On overflow:
  - set `SR.UIF`
  - if `DIER.UIE`, pend timer IRQ.
- `EGR.UG` forces update event.
- PWM compare registers are stored and traceable; exact waveform output can come later.

### 13.7 ADC

Implement ADC1/2/3 as needed.

Behavior required by PHAL:

- Calibration completes immediately.
- Enabling ADC sets ready flag.
- Starting conversion sets EOC/EOS after tiny simulated delay or immediately.
- `DR` returns configured channel value.
- Sequence registers select channels.
- DMA mode can push readings into DMA buffer.

Config example:

```json
{
  "adc": {
    "ADC1": {
      "channels": {
        "1": 2048,
        "2": { "type": "constant", "value": 3100 },
        "3": { "type": "sine", "min": 1000, "max": 3000, "period_ms": 1000 }
      }
    }
  }
}
```

### 13.8 SPI

Implement SPI1/2/3.

Registers:

- `CR1`, `CR2`
- `SR`
- `DR`
- `CRCPR`, `RXCRCR`, `TXCRCR`

Behavior:

- TXE ready by default.
- BSY set during transfer, then cleared.
- Write to `DR` sends byte/word to attached SPI device.
- Read from `DR` returns device response.
- DMA mode delegates to DMA controller for block transfers.

Device abstraction:

```cpp
class SpiDevice {
public:
  virtual std::vector<uint8_t> transfer(std::span<const uint8_t> tx) = 0;
};
```

Initial devices:

- generic zero-fill device
- echo device
- BMI088 stub
- ADBMS6380/ADBMS BMS stub for `a_box`

### 13.9 DMA and DMAMUX

DMA is needed because PHAL USART/SPI/ADC code may use DMA.

Model:

- DMA1/DMA2 channels.
- DMAMUX channel request mapping.
- Each channel has `CCR`, `CNDTR`, `CPAR`, `CMAR`.
- Interrupt/status registers: `ISR`, `IFCR`.

Behavior:

- On channel enable, inspect direction and peripheral address.
- For memory-to-peripheral:
  - copy bytes from memory to attached peripheral TX path.
- For peripheral-to-memory:
  - ask attached peripheral for bytes/samples and write memory.
- Set transfer-complete flag.
- Pend DMA IRQ if TC interrupt enabled.
- Circular mode repeats or refills when peripheral triggers.

Start with immediate completion; later schedule transfer duration.

### 13.10 FDCAN and virtual CAN

This is one of the most important board peripherals.

Implement enough for PER PHAL and generated CAN node code, not full Bosch M_CAN initially.

Registers/areas:

- FDCAN core registers around `FDCANx_BASE`.
- FDCAN message RAM region, if firmware writes it directly.
- CCCR/init/config mode.
- NBTP timing config accepted.
- TX FIFO/queue status.
- RX FIFO status.
- Interrupt flags/enable.
- Standard/extended filter config can be permissive first.

Internal abstraction:

```cpp
struct CanFrame {
  uint32_t id;
  bool extended;
  bool fd;
  bool brs;
  uint8_t dlc;
  std::array<uint8_t, 64> data;
};

class VirtualCanBus {
public:
  void attach(FdcanPeripheral* node);
  void send(FdcanPeripheral* sender, const CanFrame& frame);
  void inject(const CanFrame& frame);
};
```

Behavior:

- When firmware writes a TX element and requests transmission, convert it to `CanFrame` and send to virtual bus.
- Bus delivers to all attached nodes except sender unless loopback is enabled.
- Receiving node pushes frame to RX FIFO, sets interrupt flag, and pends FDCAN IRQ if enabled.
- Filtering initially accepts all; add real filters later when required.

Trace format:

```json
{"time_ns":12345,"board":"dashboard","bus":"vehicle","type":"can_tx","id":"0x123","dlc":8,"data":"0102030405060708"}
```

### 13.11 Watchdogs

Implement IWDG/WWDG as stubs initially:

- Accept key reload writes.
- If enabled and not kicked before timeout, request simulated reset.
- Default config can disable reset behavior for easier early bring-up.

### 13.12 Other peripherals as encountered

Add stubs for:

- I2C
- RTC
- EXTI/SYSCFG
- COMP/OPAMP
- DAC
- CRC
- RNG if touched

Use MMIO trace to drive exact implementation order.

---

## 14. Board Configuration

Start with JSON configs.

### Single-board config example

```json
{
  "name": "g4_testing",
  "mcu": "configs/mcus/stm32g474retx.json",
  "elf": "/Users/ronak/coding/PER/Projects/firmware/output/g4_testing/g4_testing.elf",
  "vector_base": "0x08000000",
  "run": {
    "default_duration_ms": 1000,
    "max_instructions": 50000000
  },
  "gpio": {
    "PC13": { "mode": "output", "trace": true }
  },
  "can": {
    "FDCAN1": { "bus": "vehicle", "loopback": false }
  },
  "usart": {
    "USART1": { "tx_log": "logs/g4_testing_usart1.log" },
    "USART2": { "tx_log": "logs/g4_testing_usart2.log" },
    "USART3": { "tx_log": "logs/g4_testing_usart3.log" }
  },
  "adc": {},
  "spi": {}
}
```

### Network config example

```json
{
  "name": "per_vehicle",
  "buses": {
    "vehicle": { "type": "can", "bitrate": 500000 }
  },
  "boards": [
    "configs/boards/dashboard.json",
    "configs/boards/main_module.json",
    "configs/boards/torque_vector.json",
    "configs/boards/a_box.json",
    "configs/boards/front_driveline.json",
    "configs/boards/rear_driveline.json"
  ]
}
```

### Fault injection config

Later add:

```json
{
  "faults": [
    { "at_ms": 1000, "type": "gpio_set", "pin": "PA0", "value": 0 },
    { "at_ms": 2000, "type": "can_drop", "bus": "vehicle", "id": "0x123" },
    { "at_ms": 3000, "type": "adc_set", "adc": "ADC1", "channel": 3, "value": 4095 }
  ]
}
```

---

## 15. Tracing and Diagnostics

Diagnostics are critical because the first real-firmware runs will expose missing instructions and missing peripheral bits.

### Trace streams

Support JSONL trace categories:

- `instr`: instruction execution, optional.
- `mmio_read` / `mmio_write`.
- `exception_enter` / `exception_return`.
- `irq_pend` / `irq_clear`.
- `gpio_write`.
- `uart_tx` / `uart_rx`.
- `can_tx` / `can_rx`.
- `dma_start` / `dma_complete`.
- `spin_detected`.
- `unimplemented_instruction`.

### Symbolization

ELF loader should optionally parse symbol table. Diagnostics should report:

```text
pc=0x08001234 symbol=PHAL_configureClockRates+0x88
```

### Spin detector

Detect common firmware wait loops:

- Same PC range executed many times.
- Same MMIO address repeatedly read.
- Same register value returned repeatedly.

Diagnostic:

```text
possible spin detected
  pc=0x08004562 PHAL_configurePLLSystemClock+0x34
  repeated mmio read: RCC.CR offset=0x00 value=0x03000001
  likely waiting for bit: PLLRDY
```

Implementation:

```cpp
struct SpinWindow {
  uint32_t last_pc;
  uint32_t last_mmio_addr;
  uint32_t last_mmio_value;
  uint64_t repeat_count;
};
```

---

## 16. Implementation Milestones

### Milestone 0: Skeleton

Deliverables:

- CMake builds `fil`.
- CLI parses basic commands.
- Unit test harness exists.

Validation:

```bash
cmake -S . -B build
cmake --build build
./build/fil --help
ctest --test-dir build
```

### Milestone 1: ELF load and memory map

Deliverables:

- ELF32 loader.
- Flash/SRAM memory regions.
- Vector table extraction.
- `inspect-elf` command.

Validation:

```bash
fil inspect-elf /Users/ronak/coding/PER/Projects/firmware/output/g4_testing/g4_testing.elf
```

### Milestone 2: Startup instruction subset

Deliverables:

- CPU state.
- Thumb decoder/executor for startup/C runtime instructions.
- Basic branch/load/store/arithmetic/stack instructions.

Validation:

- Run tiny hand-authored startup ELF.
- Execute reset handler through `.data` copy and `.bss` zero loops.

### Milestone 3: Basic STM32 startup peripherals

Deliverables:

- RCC mock.
- FLASH register mock.
- PWR mock.
- GPIO mock.
- Lenient unknown MMIO.
- Spin detector.

Validation:

- Real `g4_testing.elf` reaches `main` or first major peripheral init.

### Milestone 4: Cortex-M exceptions and SysTick

Deliverables:

- SCB, NVIC, SysTick.
- Exception entry/return.
- SVC, PendSV, SysTick support.
- `MRS/MSR`, `SVC`, `WFI`, barriers.

Validation:

- Tiny FreeRTOS or scheduler test ELF starts first task.
- SysTick increments over simulated time.

### Milestone 5: FPU support

Deliverables:

- FP register file.
- Common VFP data movement/arithmetic/conversion instructions.
- FPU context save/restore instructions.
- CPACR handling.

Validation:

- Hard-float compiled test firmware computes expected float results.
- PER firmware no longer aborts on VFP instructions.

### Milestone 6: USART, timers, ADC

Deliverables:

- USART TX/RX logs and scripted input.
- TIM update events/IRQs.
- ADC configured values and conversion flags.

Validation:

- Firmware UART output captured.
- Timer-driven code progresses.
- ADC reads return config values.

### Milestone 7: DMA and SPI

Deliverables:

- DMA channel model.
- DMAMUX stubs.
- SPI transfers and attached devices.

Validation:

- SPI transfers complete.
- DMA interrupt/completion paths work.

### Milestone 8: FDCAN and virtual bus

Deliverables:

- FDCAN TX/RX model.
- Virtual CAN bus.
- CAN injection CLI.
- CAN trace output.

Validation:

- One firmware node emits CAN frames.
- Injected CAN frame triggers firmware RX callback path.

### Milestone 9: Multi-board network

Deliverables:

- Multiple emulator instances in one process.
- Shared event loop.
- Shared CAN buses.
- Network config.

Validation:

- Dashboard/main_module/etc. run together.
- Heartbeats and control frames propagate between boards.

### Milestone dependency and exit gates

```text
M0 skeleton
 └─ M1 ELF + memory
     └─ M2 integer CPU startup
         ├─ M3 startup MMIO
         └─ M4 exceptions + scheduler
             ├─ M5 FPU
             └─ M6 USART/TIM/ADC
                 └─ M7 DMA/SPI
                     └─ M8 FDCAN
                         └─ M9 multi-board
```

A milestone is complete only when all of the following are true:

1. Its unit tests and earlier milestone tests pass under `ctest`.
2. Its documented smoke command exits with the expected reason, not merely without crashing.
3. No sanitizer finding occurs in host tests (`-fsanitize=address,undefined` where supported).
4. New instruction/register coverage is recorded in the corresponding coverage document.
5. User-visible behavior and known limitations are documented.
6. The next milestone can be disabled without breaking completed lower layers.

Use stable stop reasons in test assertions:

```cpp
enum class StopReason {
  Halted,
  Breakpoint,
  InstructionBudget,
  TimeBudget,
  UnimplementedInstruction,
  ArchitecturalFault,
  ConfigError,
  HostError,
};
```

The CLI maps normal budget exhaustion and explicit test breakpoints to success only when requested by the command/config. Unsupported instructions, malformed config, and unhandled architectural faults are failures.

---

## 17. Testing Strategy

### Test layers and hermeticity

- **Host unit tests** require no ARM toolchain and directly test C++ APIs.
- **Instruction fixture tests** use committed tiny ELF/binary fixtures. Regeneration requires the ARM toolchain, but ordinary `ctest` must not.
- **Integration tests** run `fil` against committed synthetic firmware and normalized configs.
- **PER smoke tests** are opt-in because their ELFs live outside this repository. Discover them through `FIL_PER_FIRMWARE_DIR`; never embed one developer's absolute path in required tests.
- Tests must not depend on host time, thread scheduling, locale, unordered-container iteration order, or writable source directories.

### Unit tests

Test every core piece independently:

- ELF parser.
- Memory bus alignment and permissions.
- Instruction decoding.
- ALU flags.
- Branch/link behavior.
- Load/store addressing modes.
- Stack push/pop.
- Exception frame layout.
- SysTick counting.
- NVIC pending/enable behavior.
- GPIO BSRR/ODR/IDR behavior.
- USART flags.
- CAN bus delivery.

### Instruction tests

Create small assembly snippets under `tests/firmware_src/asm/`, compile with `arm-none-eabi-gcc`, and run in emulator.

Pattern:

1. Set inputs in registers/memory.
2. Execute instruction sequence.
3. Store outputs to known RAM address.
4. Halt via `BKPT` or infinite loop.
5. Test harness checks memory/registers.

Example:

```asm
.syntax unified
.thumb
.global Reset_Handler
Reset_Handler:
  movs r0, #1
  movs r1, #2
  adds r2, r0, r1
  ldr r3, =0x20000000
  str r2, [r3]
  bkpt #0
```

### Firmware smoke tests

Run real PER firmware with increasing strictness:

```bash
fil run configs/boards/g4_testing.json --duration-ms 10 --trace traces/g4_testing.jsonl
fil run configs/boards/dashboard.json --duration-ms 100 --trace traces/dashboard.jsonl
fil run configs/boards/main_module.json --duration-ms 100 --trace traces/main_module.jsonl
```

Pass criteria evolves by milestone:

- Milestone 1: loads ELF and vectors.
- Milestone 3: reaches `main`.
- Milestone 4: FreeRTOS scheduler starts.
- Milestone 8: CAN frames flow.
- Milestone 9: multiple boards interact.

### Required validation matrix

| Area | Positive case | Boundary/error case | Differential/reference check |
|---|---|---|---|
| ELF | valid split LMA/VMA image | truncated header, overflow, overlap, invalid vector | compare segment report with `arm-none-eabi-readelf -l` |
| Memory | aligned/unaligned RAM and aliases | wraparound, ROM write, cross-region access | byte-composed little-endian oracle |
| Decoder | each supported encoding family | undefined/reserved encodings | compare fixture disassembly with `arm-none-eabi-objdump` |
| ALU | representative arithmetic/shift | zero shift, carry edges, signed overflow | table-generated mathematical oracle |
| Exceptions | MSP and PSP frames | bad vector, stacking fault, nested priority | expected frame bytes from ARM pseudocode |
| SysTick/NVIC | wrap and preemption | masks, equal priorities, disabled IRQ | deterministic hand-authored scenario |
| Peripheral | documented reset/read/write behavior | reserved widths/bits and disabled clock | PHAL call-path integration fixture |
| Scheduler | two tasks alternate at 1 kHz | `BASEPRI`, pending while masked | expected task/exception trace |
| CAN/network | ordered delivery | drop, duplicate, simultaneous send | golden normalized JSONL trace |

### Regression tests from traces

When a bug is found in real firmware:

1. Save minimal trace/repro config.
2. Add unit/integration test.
3. Fix emulator.
4. Keep test permanently.

---

## 18. Development Workflow

For each missing instruction/peripheral behavior:

1. Run firmware until failure/spin.
2. Inspect diagnostic:
   - PC/symbol.
   - instruction raw bits or MMIO address.
   - register state.
3. Add focused unit test.
4. Implement minimal correct behavior.
5. Re-run unit test.
6. Re-run firmware smoke.
7. Update coverage docs.

Do not implement huge peripheral blocks speculatively. Implement enough behavior for real firmware, with clear TODOs for unsupported features.

---

## 19. Rollback and Safety Plan

- Keep CPU, memory, and each peripheral in separate files/classes.
- Each milestone should compile and pass tests independently.
- Unknown MMIO is non-fatal by default during bring-up.
- Unimplemented instructions are fatal with excellent diagnostics.
- Every peripheral can be replaced by a stub in config if needed.
- Use feature flags/config to disable watchdog resets, strict faults, and strict MMIO during early bring-up.

Rollback examples:

- Bad FDCAN implementation: switch FDCAN device to log-only stub while CPU work continues.
- Bad FPU implementation: add targeted instruction trap and tests; keep non-FPU tests passing.
- Bad exception behavior: run non-FreeRTOS tiny firmware tests while fixing stack frame logic.

---

## 20. Major Risks and Mitigations

### Risk: Thumb-2/FPU implementation scope is large

Mitigation:

- Implement trace-guided instruction coverage.
- Use small assembly tests per instruction class.
- Prioritize GCC-generated instruction patterns from PER firmware.

### Risk: FreeRTOS depends on precise exception behavior

Mitigation:

- Implement SVC/PendSV/SysTick carefully and test with tiny FreeRTOS fixture.
- Pay attention to PSP/MSP, EXC_RETURN, BASEPRI, and FPU context instructions.

### Risk: Firmware spins on missing hardware-ready bits

Mitigation:

- Spin detector.
- RCC/FLASH/PWR ready bits implemented early.
- Lenient MMIO mode with clear repeated-read diagnostics.

### Risk: FDCAN complexity

Mitigation:

- Model firmware-observed behavior first.
- Keep message RAM abstraction simple.
- Accept all filters initially.
- Add exact filter behavior only if firmware depends on it.

### Risk: DMA complexity

Mitigation:

- Immediate-completion DMA first.
- Implement per-peripheral DMA hooks.
- Add circular mode only when a target needs it.

### Risk: Multi-board performance

Mitigation:

- Start with one board.
- Add instruction dispatch profiling.
- Optimize decoder and memory region lookup later.
- Optional block cache/JIT is out of scope until interpreter works correctly.

---

## 21. First Concrete Implementation Task List

Execute these as small, reviewable work packages. Do not start the next package until its verification passes.

### WP-0: Project and test skeleton

**Touch points**

- `CMakeLists.txt`
- `src/main.cpp`
- `src/cli/cli.cpp`, `include/fil/cli/cli.hpp`
- `tests/CMakeLists.txt`, `tests/unit/smoke_test.cpp`
- `.gitignore`, `README.md`

**Tasks**

1. Define `fil_core`, `fil`, and `fil_tests`; enable C++20 without compiler extensions.
2. Add warning flags per compiler and opt-in ASan/UBSan CMake options.
3. Implement `fil --help` and stable process exit-code constants.
4. Register one host smoke test and verify out-of-tree builds.
5. Document prerequisites and exact build/test commands.

**Exit check**

```bash
cmake -S . -B build -G Ninja -DFIL_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
./build/fil --help
```

### WP-1: Common result, diagnostics, and config primitives

**Touch points**

- `include/fil/common/result.hpp`, `include/fil/common/error.hpp`
- `include/fil/common/log.hpp`, `src/common/log.cpp`
- `include/fil/config/config.hpp`, `src/config/config.cpp`
- `tests/unit/config_test.cpp`

**Tasks**

1. Define typed error categories with source context; avoid parsing diagnostics from strings in tests.
2. Implement integer parsing for decimal/hex addresses and size suffixes (`K`, `M`) with overflow checks.
3. Parse the minimal MCU and board schemas; reject unknown keys by default, with an explicit compatibility flag if leniency is needed later.
4. Resolve relative paths against the containing config file, not process CWD.
5. Add config schema version `1` now to permit future migrations.

**Exit check:** valid sample configs round-trip to normalized debug output; malformed addresses, missing keys, duplicate devices, and bad paths produce deterministic diagnostics.

### WP-2: ELF inspection without execution

**Touch points**

- `include/fil/elf/elf_loader.hpp`, `src/elf/elf_loader.cpp`
- `src/cli/inspect_elf.cpp`
- `tests/unit/elf_loader_test.cpp`
- `tests/fixtures/elf/`

**Tasks**

1. Implement bounds-checked little-endian readers; never cast file bytes to ELF structs.
2. Parse ELF/program/section headers and ARM attributes needed for compatibility reporting.
3. Build separate load-image and runtime-range records for split LMA/VMA segments.
4. Parse `SHT_SYMTAB`/`SHT_DYNSYM` when present; sort function symbols for nearest-symbol lookup.
5. Implement `inspect-elf`, including machine/ABI/float attributes, vectors, and segment mapping.
6. Add malformed-file tests plus a split `.data` fixture matching the current PER linker pattern.

**Exit check:** normalized `inspect-elf` segment output agrees with `arm-none-eabi-readelf -h -l -A` for every available PER ELF.

### WP-3: Memory map and reset image

**Touch points**

- `include/fil/mem/region.hpp`, `src/mem/region.cpp`
- `include/fil/mem/memory_bus.hpp`, `src/mem/memory_bus.cpp`
- `tests/unit/memory_bus_test.cpp`

**Tasks**

1. Add RAM, ROM, alias, and MMIO region types with overlap validation.
2. Map flash, CCM SRAM, SRAM, boot alias, and system-control ranges from MCU config.
3. Materialize ELF file bytes at load addresses while reserving runtime ranges.
4. Implement little-endian 8/16/32/64-bit accesses, access metadata, and typed bus faults.
5. Prove with a fixture that reset code can read `.data` initializers from flash while destination SRAM begins at reset value.

**Exit check:** all memory boundary, alias, permission, split-LMA/VMA, and overflow tests pass under sanitizers.

### WP-4: CPU fetch/decode foundation

**Touch points**

- `include/fil/cpu/{cortex_m4,instruction,decoder}.hpp`
- `src/cpu/{cortex_m4,decoder,execute}.cpp`
- `tests/unit/{decoder,alu,cpu_step}_test.cpp`

**Tasks**

1. Define architectural state, explicit active-SP helpers, APSR/IPSR/EPSR helpers, and reset semantics.
2. Fetch 16/32-bit Thumb instructions with execute access type and precise instruction address.
3. Add mask/value decoder tables with startup-time overlap assertions.
4. Implement `addWithCarry`, shifts, condition evaluation, and IT-state advancement independently of instruction handlers.
5. Add one-step execution and bounded-run APIs returning `StopReason` and a structured diagnostic snapshot.
6. Implement unimplemented-instruction reporting before implementing instruction semantics.

**Exit check:** reset vectors initialize MSP/PC/xPSR correctly; 16/32-bit fetch and decoder-negative tests pass.

### WP-5: Startup instruction slice

Implement and test encoding families, not only mnemonic names, in this order:

1. literal/immediate/register `LDR` and immediate/register `STR`;
2. `MOV`/`MOVS`, `MOVW`, `MOVT`;
3. `ADD`/`ADDS`, `SUB`/`SUBS`, `CMP` and flag edge cases;
4. shifts and logical operations used by startup loops;
5. `B`, conditional `B`, `BL`, `BX`, `BLX`;
6. `PUSH`, `POP`, `LDM`, `STM` including PC/LR cases;
7. byte/halfword loads and stores;
8. `CBZ`/`CBNZ`, `IT`, `NOP`, `BKPT`.

For each family: commit a positive fixture, boundary cases, reserved-encoding negative test, implementation, and coverage row.

**Exit check:** a synthetic startup ELF copies `.data`, zeros `.bss`, calls `main`, writes a sentinel, and stops at `BKPT` with exact expected register/memory state.

### WP-6: First real-firmware bring-up loop

1. Add the minimal `run` command with instruction budget, trace path, and lenient MMIO policy.
2. Run `g4_testing.elf` to the first unsupported instruction; preserve a compact reproducer.
3. Continue instruction-by-instruction until the first repeated MMIO wait.
4. Implement only RCC/FLASH/PWR behavior required to exit that wait, with register-level tests.
5. Add GPIO and a `main`-reached symbol/PC breakpoint assertion.
6. Save a normalized smoke expectation that is independent of absolute host paths.

**Exit check:** `g4_testing.elf` reaches `main` deterministically from reset, or the remaining blocker is explicitly listed with a reproducer and assigned to M4/M5.

### Subsequent work packages

After WP-6, follow milestone order M4-M9. Split each peripheral into: register storage/reset tests, side-effect tests, IRQ/DMA integration tests, then real-firmware smoke. Never combine a CPU semantic change and unrelated peripheral behavior in one work package.

---

## 22. Definition of Done for Initial Useful Emulator

The first useful version is complete when:

1. `fil inspect-elf` works on all G4 PER `.elf` files.
2. `fil run configs/boards/g4_testing.json --duration-ms 1000` executes without unimplemented instructions.
3. Reset handler reaches `main`.
4. FreeRTOS scheduler starts and SysTick advances.
5. GPIO writes are logged.
6. USART TX is logged.
7. At least one FDCAN frame can be transmitted or injected.
8. Unknown MMIO accesses are either implemented or documented in peripheral coverage.
9. The emulator can be configured without modifying code for a different G4 PER board.
10. Repeating the same run produces a byte-identical normalized trace.
11. Required tests run without the external PER checkout; PER artifact smoke tests are clearly marked opt-in.
12. ASan/UBSan host test runs are clean on a supported compiler.

---

## 23. Rollback Strategy by Layer

The general rollback principles in section 19 apply at feature level. At implementation level:

- Keep every milestone behind construction/config boundaries; a new peripheral replaces an `UnknownMmioDevice` mapping rather than modifying CPU code.
- Preserve the last passing synthetic fixture and smoke expectation before changing CPU, exception, or time semantics.
- If a regression appears, first disable the newest peripheral/event source, then bisect by instruction family or register hook.
- Config schema changes require either backward-compatible defaults or a versioned migration; do not silently reinterpret existing fields.
- Trace schema changes add fields compatibly during the initial version. A breaking change increments `trace_schema_version` and keeps the prior parser fixture.
- Do not roll back by weakening strict instruction validation. A temporary peripheral fallback is acceptable; silently treating an unknown instruction as `NOP` is not.

A work package is safe to revert when its tests, config additions, and coverage rows are contained in the same change and lower-layer tests remain independent.

---

## 24. Open Decisions to Resolve Before Their Milestones

These do not block WP-0, but must be decided before the named work package:

1. **Test framework (WP-0):** lightweight in-repo harness versus pinned Catch2. Choose based on offline/reproducible build requirements.
2. **JSON parser (WP-1):** small strict in-repo parser versus pinned small dependency. Do not implement a permissive partial parser without source-location errors.
3. **ELF permission semantics (WP-3):** diagnostic-only versus enforced R/W/X before MPU support. Default recommendation: enforce ROM writes, report execute permissions, and leave SRAM execution configurable.
4. **FPU edge semantics (M5):** identify whether PER relies on NaN payloads, denormal handling, rounding modes, or FP exceptions before treating host `float` as sufficient.
5. **Peripheral clock gating (M3/M6):** initially log access while disabled, then enforce only where PHAL behavior/tests require it.
6. **FDCAN message RAM layout (M8):** model exact configured element layout from register values rather than inventing a parallel hidden queue format.
7. **Multi-board scheduling quantum (M9):** choose a fixed instruction/cycle quantum and prove runs are invariant under host performance. Event deadlines must cap a board's quantum.
8. **Firmware redistribution:** determine whether any PER ELF may be committed. Until then, commit only purpose-built fixtures and keep PER smoke tests external.
