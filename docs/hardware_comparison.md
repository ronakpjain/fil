# Hardware comparison

`fil compare-stlink` runs the same ELF in the emulator and on an attached
STM32G47x/G48x through OpenOCD, then compares architectural state at a common
halt boundary. The command is intended for deterministic model validation, not
cycle or wall-clock benchmarking.

## Requirements and safety

The host needs a POSIX environment and OpenOCD with the `stlink` interface and
`stm32g4x` target files. The command always connects to, resets, runs, and
finally leaves the target halted at reset. It does not modify flash unless
`--flash` is present.

Before any optional erase/program operation, the generated OpenOCD script reads
`DBGMCU_IDCODE` and requires STM32G47x/G48x device ID `0x469`. `--flash` erases,
programs, and verifies the ELF, so preserve any firmware that must be restored.
Use `--serial` when more than one ST-Link is connected.

## Deterministic probe

The committed probe initializes every compared core register, fixes the APSR
flags immediately before `BKPT`, and writes a 16-byte SRAM result block. This
avoids false mismatches from architecturally unspecified general-register reset
values.

Regenerate it only when its source changes:

```bash
cmake --build build --target firmware_tests
```

Compare the committed probe and retain all JSON artifacts:

```bash
./build/fil compare-stlink \
  tests/fixtures/config/hardware_compare_board.json \
  --flash \
  --memory 0x20000000:16 \
  --artifacts /tmp/fil-hardware-comparison
```

A subsequent run can omit `--flash` while the same image remains programmed.
The command returns success only when all selected values match. Standard output
is a stable JSON comparison object; `--artifacts` additionally writes
`emulator.json`, `hardware.json`, and `comparison.json`.

## Stop boundaries

With no stop option, both executions run until firmware executes `BKPT`. OpenOCD
reports PC at the breakpoint instruction; the emulator snapshot normalizes its
PC to the same halted-boundary convention.

For firmware without a suitable `BKPT`, use one hardware breakpoint and the same
pre-instruction emulator boundary:

```bash
./build/fil compare-stlink board.json \
  --stop-at-symbol comparison_ready \
  --max-instructions 100000 \
  --memory 0x20000000:64
```

`--stop-address` is the numeric equivalent. The timeout bounds OpenOCD and target
execution; the emulator is bounded independently by `--max-instructions`.

## Comparison scope

By default the command compares `r0` through `r15`, `xPSR`, `MSP`, `PSP`,
`PRIMASK`, `BASEPRI`, `FAULTMASK`, and `CONTROL`. Repeat `--register` to compare
only named registers, or repeat `--ignore-register` to remove fields from the
default set. Firmware must initialize any general-purpose registers selected for
comparison.

Repeat `--memory ADDRESS:LENGTH` for byte-exact RAM or flash checks. Emulator
snapshot capture intentionally rejects MMIO and alias ranges because reading
peripheral registers can itself change device state. Floating-point registers,
peripheral state, and execution timing are outside the initial comparison
contract.

Useful connection options are:

```text
--openocd PATH       OpenOCD executable (default: openocd)
--serial ID          Select one ST-Link probe
--timeout-ms N       Hardware halt/process timeout (default: 10000)
--strict-mmio        Make the emulator reject unknown MMIO
```

The normal GoogleTest suite uses a fake OpenOCD executable and never touches
hardware. Physical-board comparison remains an explicit manual operation.
