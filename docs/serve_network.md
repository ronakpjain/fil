# Binary network service

`fil serve-network NETWORK --transport stdio` serves a live network to another
process over standard input and standard output. Version 1 supports only the
`stdio` transport. It is the binary counterpart to the human-facing
[`watch-network`](watch_network.md): both run the same paced, sliced network
monitor and support CAN, ADC, GPIO, and stop controls, but `serve-network` emits
only protocol frames on stdout. Diagnostics and rejected-request explanations go
to stderr. `run-network` remains the deterministic batch command with its
human-readable summary and optional JSONL trace file.

```bash
./build/fil serve-network configs/networks/per_vehicle.json \
  --transport stdio --refresh-ms 1 --live-filter can_tx
```

The service sends a `HELLO` frame after loading the network, then streams selected
trace records and replies to client requests. Trace filtering follows
`watch-network`: the default is `can_tx`; use repeatable `--live-filter TYPE` (or
`--trace-type TYPE`) to select other event types. `--trace-instr` enables
instruction records. `--duration-ms`, `--refresh-ms`, `--max-instructions`,
`--quantum`, `--adc-decimation`, `--strict-mmio`, `--lenient-mmio`,
`--no-wall-pacing`, `--no-loop-batching`, `--jit`, and `--detect-spin` retain the
corresponding watch behavior. `--transport stdio` is required; any other
transport is rejected.

## Wire framing

The protocol is a byte stream. Each frame begins with this 16-byte header;
integers are explicitly encoded little-endian, with no native-struct padding:

| Offset | Size | Field | Version 1 value / meaning |
|---:|---:|---|---|
| 0 | 4 | Magic | ASCII `FILN` |
| 4 | 1 | Version | `1` |
| 5 | 1 | Kind | Message kind from the tables below |
| 6 | 2 | Flags | Must be zero |
| 8 | 4 | Payload length | Bytes following the header, at most 65,536 |
| 12 | 4 | Request ID | Client request correlation ID; zero for unsolicited server frames |

A receiver must handle partial headers/payloads and multiple concatenated frames.
Strings are encoded as `u16 byte_length` followed by that many UTF-8 bytes, with
no terminator. Numeric payload fields use the same little-endian convention.

## Client requests

Request IDs must be nonzero and should be unique among outstanding requests.
Requests are applied in wire order at network slice boundaries. A successful
reply's timestamp is the simulated time at which the operation took effect.
Trace events synchronously caused by a request can be sent before that request's
reply.

| Kind | Name | Payload |
|---:|---|---|
| `0x01` | `CAN_INJECT` | bus string; `u32 id`; `u8 flags`; `u8 data_length`; `data_length` raw bytes |
| `0x02` | `ADC_SET` | board string; ADC instance string; `u8 channel`; `u16 value` |
| `0x03` | `GPIO_SET` | board string; GPIO port string; `u8 pin`; `u8 action` |
| `0x04` | `STOP` | Empty |

`CAN_INJECT` flag bit 0 selects an extended identifier, bit 1 selects CAN-FD,
and bit 2 enables bit-rate switching; all other bits must be zero. Standard IDs
must fit 11 bits and extended IDs 29 bits. Bit-rate switching requires CAN-FD.
Payload lengths are `0..8`, `12`, `16`, `20`, `24`, `32`, `48`, or `64` bytes;
classic CAN is limited to `0..8` bytes. The server normalizes the payload length
to its CAN DLC.

For example, a `CAN_INJECT` request with ID `0x123`, bus `vehicle`, request ID
`1`, and two data bytes `de ad` is the following 33-byte frame (`u16`, `u32`,
and other multibyte values shown in wire order):

```text
46 49 4c 4e 01 01 00 00 11 00 00 00 01 00 00 00
07 00 76 65 68 69 63 6c 65 23 01 00 00 00 02 de ad
```

`ADC_SET` accepts channels `0..19` and values `0..4095`. `GPIO_SET` accepts pins
`0..15`; action `0` drives low, `1` drives high, and `2` releases the external
drive. `STOP` acknowledges and then ends the session, equivalent to `quit` in
`watch-network`.

## Server frames

| Kind | Name | Request ID | Payload |
|---:|---|---:|---|
| `0x80` | `HELLO` | `0` | network-name string; `u32 board_count`; `u32 can_bus_count` |
| `0x81` | `REPLY` | Matching request ID | `u16 status`; `u64 applied_time_ns`; diagnostic string |
| `0x82` | `TRACE` | `0` | `u64 time_ns`; `u64 sequence`; source string; type string; `u16 field_count`; ordered key/value string pairs |
| `0x83` | `END` | `0` | `u64 final_time_ns`; `u8 reason`; `u8 process_exit_code`; diagnostic string |

Reply status values are `0` success, `1` invalid request/payload, `2` unknown bus,
board, or peripheral, and `3` unsupported message kind. A failed semantic request
gets a `REPLY`, is reported on stderr, and does not stop the service. Trace fields
remain ordered strings, as in the JSONL trace contract; `sequence` gaps are
possible when trace types are filtered.

End reason values are `0` client `STOP`, `1` clean input EOF, `2` duration limit,
`3` all boards reached terminal CPU boundaries, and `4` runtime failure. The
process exit code is the CLI exit code (`0`, `2`, `3`, `4`, or `70`). Startup
argument/configuration errors occur before `HELLO`, are reported on stderr, and
produce no stdout text or frames.

## Errors, EOF, and backpressure

Invalid magic, version, header flags, oversized frames, or truncated input are
fatal protocol errors. The service reports the error on stderr, closes the stream,
and does not attempt to resynchronize. Unknown message kinds and malformed
request payloads receive a failed `REPLY` when the frame boundary is valid.

Clean EOF at a frame boundary stops an unlimited service and emits `END`. If a
finite `--duration-ms` is set, clean EOF does not stop the network; it runs until
the duration limit or until all boards stop. A duration of `0` means unlimited,
matching `watch-network`. Partial frames at EOF are errors, not clean EOF.

Frames are flushed as they are written. A slow stdout consumer applies normal
pipe backpressure and may slow simulation wall-clock progress; the service does
not queue or drop trace frames. If stdout is closed, the service exits with a
runtime error; a final `END` cannot be guaranteed on a broken output stream.

For event names and the meaning of trace fields, see [Tracing](tracing.md). For
the interactive text syntax and live-monitor behavior, see
[Live network monitoring](watch_network.md).
