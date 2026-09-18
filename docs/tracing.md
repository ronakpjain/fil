# JSONL trace contract

`fil run` and `fil run-network` write deterministic JSON Lines when passed
`--trace FILE`. Each line is one compact JSON object and one simulation event.
Without `--trace`, the CLI disables trace collection to avoid retaining diagnostic
history. Add `--trace-instr` when instruction records are also required.

`fil watch-network` instead prints selected events as human-readable lines while a
network runs and accepts CAN frames on standard input. Its live output is not JSONL;
see [Live network monitoring](watch_network.md) for its filters, format, and control
syntax.

## Common fields and ordering

Every record begins with these fields in this serialized order:

| Field | JSON type | Meaning |
|---|---:|---|
| `time_ns` | number | Timestamp on the shared monotonic simulated-time clock. |
| `sequence` | number | Recorder-wide insertion sequence used to order records at the same timestamp. |
| `source` | string | Board, peripheral, network, or CAN endpoint that emitted the event. |
| `type` | string | Stable event-kind name. |

Event-specific fields follow the common fields. They are currently serialized as
JSON strings, including numeric and Boolean-looking values. Consumers should parse
them according to `type`, tolerate additional fields, and use `sequence` as the
total-order tie breaker. The recorder emits records in insertion order; `clear()`
empties the trace and restarts sequence numbering at zero. Identical inputs and run
options produce byte-identical normalized output.

For example:

```json
{"time_ns":250,"sequence":7,"source":"dashboard.GPIOA","type":"gpio_output","pin":"7","value":"1"}
```

## Source names

| Source shape | Used for | Example |
|---|---|---|
| `board` | CPU, exception, and board-run events | `dashboard` |
| `board.device` | STM32 peripheral events | `dashboard.FDCAN1` |
| `network` | World lifecycle and world event-loop events | `per_vehicle` |
| `bus/node` | Virtual CAN fabric transmit and delivery events | `vehicle/dashboard.FDCAN1` |
| `bus/external` | CAN frames injected by the CLI or host | `vehicle/external` |

Board qualification makes same-named peripherals on different boards unambiguous.
The virtual CAN fabric and an FDCAN controller can both emit `can_tx` or `can_rx`;
the source shape distinguishes the bus-facing record from the device-facing one.

## CAN records

Bus-facing `can_tx` and `can_rx` records contain:

| Field | Meaning |
|---|---|
| `id` | Lowercase hexadecimal identifier with a `0x` prefix. |
| `extended` | `"true"` for a 29-bit identifier, otherwise `"false"`. |
| `fd` | CAN-FD format flag. |
| `brs` | CAN-FD bit-rate-switch flag. |
| `dlc` | Encoded CAN data-length code in the range 0 through 15. |
| `length` | Decoded payload byte count. |
| `data` | Exactly `length` bytes as lowercase hexadecimal without separators. |

`dlc` is not a byte count for CAN-FD lengths above eight. For example, DLC 9
means a 12-byte payload:

```json
{"time_ns":250,"sequence":8,"source":"vehicle/external","type":"can_tx","id":"0x5a3","extended":"false","fd":"true","brs":"true","dlc":"9","length":"12","data":"000102030405060708090a0b"}
```

Device-facing FDCAN records currently contain decimal-string `id`, encoded `dlc`,
and decoded `length`. When the controller is attached to a virtual bus, payload bytes
and format flags are present on the bus-facing record.

## Current event types

The current semantic event set is:

| Types | Event-specific fields |
|---|---|
| `world_start` | `boards`, `quantum` |
| `world_stop` | `reason`, `instructions`, `cycles` |
| `instr` | hexadecimal `pc`, hexadecimal raw instruction `raw` |
| `exception_enter` | decimal exception number `exception` |
| `exception_return` | hexadecimal `exc_return` |
| `spin_detected` | `pc`, `period_instructions`, `period_cycles` |
| `event_livelock` | none |
| `gpio_input`, `gpio_output` | `pin`, `value` |
| `uart_tx`, `uart_rx` | `byte` |
| `uart_idle` | none |
| `clock_change` | `frequency_hz` |
| `can_tx`, `can_rx` | CAN fields described above; the exact set depends on source shape |
| `spi_transfer` | hexadecimal `tx`, hexadecimal `rx` |
| `timer_update` | `forced` |
| `adc_calibrated` | none |
| `adc_sample` | `channel`, `value` |
| `dma_transfer` | `channel`, `items`, `direction`, `success` |
| `dmamux_request` | `channel`, `request` |
| `watchdog_reload`, `watchdog_timeout` | none |
| `unknown_mmio_read`, `unknown_mmio_write` | hexadecimal `offset`, byte `size`, hexadecimal `value`, hexadecimal `pc` |

## Instruction-trace cost

`--trace-instr` emits one `instr` record per executed instruction. It substantially
increases memory use and JSON output, and it disables proven-loop batching so that
no logical instruction is omitted. Use an instruction or time budget and combine it
with `--trace FILE`; specifying `--trace-instr` alone does not enable the CLI trace
recorder or create an output file.

## MMIO scope

The trace is semantic, not an every-access bus log. There are no generic
`mmio_read` or `mmio_write` events and no `--trace-mmio` CLI option. Modeled
peripherals emit the side-effect events listed above. Explicit placeholder device
blocks can emit `unknown_mmio_read` and `unknown_mmio_write`, while addresses handled
by the top-level lenient MMIO router are aggregated in the CLI's
`unknown_mmio_addresses` summary rather than written as individual trace records.
