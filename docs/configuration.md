# Configuration

Configuration files are strict JSON with `schema_version: 1`. Duplicate keys, unknown keys, invalid types, and out-of-range values are rejected. A referenced path is resolved relative to the configuration file containing it, not the process working directory.

Unsigned values may be JSON integers or strings. String values accept decimal, hexadecimal with `0x`, and binary size suffixes `K` and `M`.

## MCU configuration

An MCU file defines memory geometry:

```json
{
  "schema_version": 1,
  "name": "stm32g474retx",
  "flash_base": "0x08000000",
  "flash_size": "512K",
  "sram_base": "0x20000000",
  "sram_size": "128K",
  "ccm_sram_base": "0x10000000",
  "ccm_sram_size": "32K",
  "hse_hz": 16000000
}
```

All memory fields shown above are required. `hse_hz` is optional and defaults to
8 MHz; set it to the board's external high-speed oscillator frequency so HSE and
HSE-derived PLL timing match the firmware's clock assumptions.

## Board configuration

A board binds one MCU, one firmware ELF, run defaults, and deterministic external devices:

```json
{
  "schema_version": 1,
  "name": "example_board",
  "mcu": "../mcus/stm32g474retx.json",
  "elf": "../../firmware/example.elf",
  "vector_base": "0x08000000",
  "run": {
    "default_duration_ms": 1000,
    "max_instructions": 50000000
  },
  "gpio": {
    "PA0": {"mode": "input", "value": true, "trace": false}
  },
  "can": {
    "FDCAN1": {"bus": "vehicle", "loopback": false}
  },
  "usart": {
    "USART1": {"rx": [72, 105], "tx_log": "logs/usart1.bin"}
  },
  "adc": {
    "ADC1": {
      "channels": {
        "2": 2048,
        "3": {"type": "constant", "value": 1024},
        "4": {"type": "sine", "min": 100, "max": 3900, "period_ms": 1000}
      }
    }
  },
  "spi": {
    "SPI1": {"device": "echo"}
  }
}
```

`vector_base` and every peripheral section are optional. Run defaults are 1000 simulated milliseconds and 50,000,000 instructions when `run` is omitted.

Supported board names and values are:

- GPIO pins `PA0` through `PG15`; `mode` is `input`, `output`, `alternate`, or `analog`.
- `FDCAN1`, `FDCAN2`, and `FDCAN3`; the named bus must exist in a network config.
- `USART1`, `USART2`, and `USART3`; `rx` is a deterministic byte array.
- `ADC1` through `ADC4`; scalar and `constant` values are 0 through 4095, and sine periods must be nonzero.
- `SPI1`, `SPI2`, and `SPI3`; built-in devices are `zero` and `echo`.

Current binding details matter when authoring tests:

- GPIO `value` installs an external input override. `mode` and `trace` are validated metadata; firmware MMIO still controls MODER, and modeled output transitions are available in the normal trace.
- `tx_log` is resolved relative to the board file, opened in binary truncate mode at
  board initialization, and receives transmitted bytes immediately. `--trace` also
  captures timestamped USART transmit records. The log's parent directory must
  already exist.
- CAN attachments are connected by `run-network`. A standalone `run` validates the FDCAN instance but has no virtual bus attachment.
- The ADC register model currently converts external channels 0 through 19. A
  conversion completes after a deterministic 5 us default delay and samples the
  configured source at that simulated timestamp. Constants return their configured
  12-bit value; sine sources repeat over `period_ms` between `min` and `max`, rounded
  to the nearest integer. Unconfigured channels return zero. The schema accepts
  channel names through 31 for forward compatibility.
- SPI `zero` returns zero-filled response bytes; `echo` returns transmitted bytes immediately.

## Network configuration

A network declares named CAN buses and a stable board order:

```json
{
  "schema_version": 1,
  "name": "per_vehicle",
  "buses": {
    "vehicle": {"type": "can", "bitrate": 500000}
  },
  "boards": [
    "../boards/dashboard.json",
    "../boards/main_module.json"
  ]
}
```

At least one board is required. Only `type: "can"` is supported, bitrate must be nonzero, board names must be unique, and every board CAN attachment must name a declared bus. Board array order is the deterministic round-robin scheduling order. Bitrate is currently topology metadata; virtual CAN delivery is synchronous and does not model wire time or arbitration.

The repository provides `configs/networks/per_vehicle.json` and board files for `g4_testing`, `dashboard`, `main_module`, `torque_vector`, `a_box`, `front_driveline`, and `rear_driveline`. Their ELF paths target a sibling PER checkout and must exist to run them.

## Inspection and execution

Validate and print a normalized summary without starting emulation:

```bash
./build/fil inspect-config configs/mcus/stm32g474retx.json
./build/fil inspect-config configs/boards/g4_testing.json
./build/fil inspect-config configs/networks/per_vehicle.json
./build/fil inspect-elf path/to/firmware.elf
./build/fil disasm-window path/to/firmware.elf --addr 0x08001234 --count 32
```

CLI options override run defaults without modifying JSON:

```bash
./build/fil run configs/boards/g4_testing.json \
  --duration-ms 1000 \
  --max-instructions 50000000 \
  --strict-mmio

./build/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 100 \
  --max-instructions 5000000 \
  --quantum 1024 \
  --inject-can vehicle@5:0x123:0102 \
  --inject-can vehicle@20:0x456:aabb
```

For `run-network`, `--duration-ms` is a shared simulated-time deadline and
`--max-instructions` is an independent budget for each board. The repeatable
`--inject-can BUS[@TIME_MS]:ID:HEXDATA` option schedules validated classic CAN or
CAN-FD frames on a declared bus; omitting `@TIME_MS` means time zero.
