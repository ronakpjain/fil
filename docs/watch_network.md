# Live network monitoring

`fil watch-network` runs a configured multi-board CAN network in short simulated-time
slices, prints selected trace events as they occur, and accepts CAN frames on
standard input. It is intended for interactive diagnostics and host-driven testing;
use [`run-network`](options.md#network-only) with `--trace` when a persistent,
byte-reproducible JSONL artifact is required.

## Start a monitor

`watch-network` paces simulation time to wall-clock time: after each slice it
waits until wall time catches up before running the next one, so live
consumers observe real message rates. When the model runs slower than real
time there is nothing to wait for and slices run back-to-back; trace order is
unchanged either way. Pass `--no-wall-pacing` to always run back-to-back.

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

## Inject ADC and GPIO values

ADC and GPIO commands take effect immediately at the current simulated time:

```text
adc dashboard ADC1 5 2048
gpio dashboard GPIOA 5 1
gpio dashboard GPIOA 5 release
```

The `adc` command sets the constant input value for one ADC channel; the board
and instance must name a configured board and its ADC peripheral, the channel
must be 0..19, and the value must be 0..4095. The `gpio` command drives one
externally controlled input pin; the port must name a GPIO peripheral such as
`GPIOA`, the pin must be 0..15, and the value is `0`, `1`, or `release` to stop
driving the pin and let firmware outputs own it.

Enter `quit` or `exit` to stop cleanly. Invalid commands are reported to standard
error and ignored without stopping the simulation.

## Options and stopping behavior

| Option | Meaning |
|---|---|
| `--duration-ms N` | Stop after at most `N` milliseconds of simulated time; `0` runs without a simulated-time limit. The monitor also stops if all boards reach terminal CPU boundaries. Without a duration, it runs until `quit`, `exit`, standard-input EOF, or all boards stop. |
| `--refresh-ms N` | Run and poll input in slices of `N` simulated milliseconds; defaults to `1` and must be nonzero. |
| `--no-wall-pacing` | Run slices back-to-back instead of pacing simulation time to wall-clock time. Pacing is on by default. |
| `--live-filter TYPE` / `--trace-type TYPE` | Print one exact trace event type. Repeat to select more than one type; default `can_tx`. |
| `--max-instructions N` | Set the per-board instruction budget used for each run slice. |
| `--quantum N` | Set the deterministic network scheduling quantum; defaults to `1024`. |
| `--adc-decimation N` | Keep one of every `N` continuous ADC scans; factor 1–1024, default 1. |
| `--trace FILE` | Also write a persistent JSONL trace; unset by default. |
| `--jit` | Opt into experimental cached/native execution; off by default, not a speed guarantee. |
| `--trace-instr` | Emit instruction trace events; add `--live-filter instr` to print them. |
| `--detect-spin` | Enable spin-loop detection; off by default. |
| `--strict-mmio` | Treat accesses outside modeled MMIO blocks as faults. |
| `--lenient-mmio` | Use lenient MMIO handling; this is the default. |
| `--no-loop-batching` | Disable proven-loop batching; batching is on by default. |
| `--control-stdin` | Accepted compatibility marker; standard input is always monitored. |

See [Options and recommended defaults](options.md) for the complete build/CLI
reference. Experimental network scheduler flags are not accepted by this command.

When `--duration-ms` is present, standard-input EOF does not end the run; the
network continues toward the requested simulated-time limit unless all boards
reach terminal CPU boundaries first. Without a duration, EOF stops the monitor.
