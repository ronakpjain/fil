# Live network monitoring

`fil watch-network` runs a configured multi-board CAN network in short simulated-time
slices, prints selected trace events as they occur, and accepts CAN frames on
standard input. It is intended for interactive diagnostics and host-driven testing;
use [`run-network`](../README.md#run-firmware) with `--trace` when a persistent,
byte-reproducible JSONL artifact is required.

## Start a monitor

```bash
./build/fil watch-network configs/networks/per_vehicle.json \
  --refresh-ms 10 \
  --live-filter can_tx \
  --live-filter can_rx \
  --control-stdin
```

The command first prints the network name, then prints each selected event in a
human-readable form:

```text
watching network per_vehicle
[5.000 ms] vehicle/external can_tx id=0x123 extended=false fd=false brs=false dlc=4 length=4 data=01020304
```

Live output is not JSONL. Fields depend on the event type; see the
[JSONL trace contract](tracing.md) for event names, source shapes, and field
meanings.

By default, only `can_tx` events are printed. The first `--live-filter TYPE` replaces
that default, and the option may be repeated to select multiple exact event types.
For example, `--live-filter can_tx --live-filter can_rx` displays both directions.

## Inject CAN frames

While the monitor is running, enter one command per line:

```text
vehicle:0x123:01020304
vehicle:0x456:aabb
```

The syntax is `BUS:ID:HEXDATA`. The bus must be declared in the network
configuration. Standard 11-bit and extended 29-bit identifiers are accepted;
identifiers above `0x7ff` are treated as extended. Payloads must contain an even
number of hexadecimal digits and use a valid classic CAN or CAN-FD length. Frames
are injected at the network's current simulated time.

Enter `quit` or `exit` to stop cleanly. Invalid commands are reported to standard
error and ignored without stopping the simulation.

## Options and stopping behavior

| Option | Meaning |
|---|---|
| `--duration-ms N` | Stop after `N` milliseconds of simulated time. Without it, the monitor runs until `quit`, `exit`, or standard-input EOF. |
| `--refresh-ms N` | Run and poll input in slices of `N` simulated milliseconds; defaults to `1` and must be nonzero. |
| `--live-filter TYPE` | Print one exact trace event type. Repeat to select more than one type. |
| `--max-instructions N` | Set the per-board instruction budget used for each run slice. |
| `--quantum N` | Set the deterministic network scheduling quantum; defaults to `1024`. |
| `--strict-mmio` | Treat accesses outside modeled MMIO blocks as faults. |
| `--lenient-mmio` | Use lenient MMIO handling; this is the default. |
| `--no-loop-batching` | Disable proven-loop batching. |
| `--control-stdin` | Explicitly marks standard input as the control channel; standard input is monitored by default. |

When `--duration-ms` is present, standard-input EOF does not end the run; the
network continues to the requested simulated-time limit. Without a duration, EOF
stops the monitor.
