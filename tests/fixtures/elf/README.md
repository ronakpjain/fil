# ELF fixtures

`split_image.elf` is a synthetic Cortex-M4F image used by hermetic parser tests. It contains separate flash and SRAM load/runtime addresses and a symbol table.

`startup_runtime.elf` is a synthetic Cortex-M4 startup image used by CPU integration tests. Its reset handler copies three `.data` words from flash, zeros `.bss`, calls `main`, writes a sentinel, and stops at `BKPT`.

`scheduler_tick.elf` is a synthetic Cortex-M4 scheduler image used by board-level
exception integration tests. SVC pends PendSV, PendSV restores a synthetic task
frame onto PSP, and that task receives and returns from one SysTick interrupt
before stopping at `BKPT`.

`hard_float.elf` is compiled from C for the Cortex-M4F hard-float ABI. It passes
floating-point arguments and results through VFP registers, executes scalar
addition, multiplication, division, and signed/unsigned conversions, stores
exact verification words in RAM, and stops at `BKPT`.

Regenerate it with:

```bash
./tools/build_test_firmware.sh
```

The ARM GNU toolchain is needed only for regeneration, not ordinary tests.
