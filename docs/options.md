# CLI options

Run `fil --help` for a synopsis. For JSON inputs, see
[Configuration](configuration.md); for building, see the [README](../README.md)
and `make help`.

## Defaults

Fidelity-preserving speedups are automatic: JIT, proven single-board loop
batching, and eligible guarded network RAM capsules. Diagnostics retain safe
fallbacks; no optimization switches are needed. Network loop batching stays off.

**ADC decimation defaults to 1 (full fidelity).** `--adc-decimation N` accepts
1–1024; values above 1 trade fidelity for speed by skipping continuous ADC scans
and their sample, DMA, and interrupt effects.

## Run a board or network

```bash
fil run board.json --duration-ms 1000
fil run-network network.json --duration-ms 1000 --strict-mmio
```

| Option | Purpose / default |
| --- | --- |
| `--duration-ms N` | Board: config value (normally 1000); network: 1000. |
| `--max-instructions N` | Board: config value (normally 50,000,000); network: 50,000,000 per board. |
| `--strict-mmio` / `--lenient-mmio` | Fault on unmodeled MMIO or allow compatibility behavior; default lenient. |
| `--trace FILE` | Write JSONL events; off by default. |
| `--trace-instr` | Include instruction events; off by default. |
| `--detect-spin` | Stop on proven repeated CPU state; off by default. Can stop legitimate idle loops. `--no-detect-spin` restores the default. |
| `--allow-breakpoint` | Accept a firmware breakpoint as a successful stop; off by default. |
| `--adc-decimation N` | Explicit fidelity-for-speed tradeoff; default 1. |

Board-only stop controls: `--stop-address ADDR` or `--stop-at-symbol NAME`.
If both are supplied, the symbol takes precedence.

Network-only controls:

- `--quantum N`: scheduling work cap; default 1024, must be nonzero.
- `--inject-can BUS[@TIME_MS]:ID:HEXDATA`: repeatable CAN injection; omitted
  time means time zero.

See [Tracing](tracing.md) and [Performance](performance.md) for details.

## Live monitoring and binary service

```bash
fil watch-network network.json
fil serve-network network.json --transport stdio
```

Both accept `--duration-ms`, `--max-instructions`, `--quantum`, MMIO controls,
`--trace-instr`, `--detect-spin`, and `--adc-decimation` from above.
Duration defaults to unlimited; `0` also means unlimited. The instruction limit
is 50,000,000 per board **per slice**. Neither accepts `--trace FILE`.

| Option | Purpose / default |
| --- | --- |
| `--refresh-ms N` | Simulated milliseconds per slice; default 1, must be nonzero. |
| `--live-filter TYPE` / `--trace-type TYPE` | Repeatable event filter; default `can_tx`. The first explicit filter replaces the default. |
| `--no-wall-pacing` | Run slices back-to-back instead of pacing to real time; live I/O stays active. |
| `--transport stdio` | Required for `serve-network`; stdout is reserved for the binary protocol. |

Watch stdin accepts CAN frames, ADC/GPIO changes, and `quit`/`exit`.
Control is always active; `--control-stdin` is an optional watch-only
compatibility marker. Unlimited runs stop on clean EOF; finite runs do not.
See [Live monitoring](watch_network.md) for text commands and
[Binary network service](serve_network.md) for the protocol.

## Inspect inputs

- `inspect-config PATH`: validate and normalize MCU, board, network, or stimulus JSON.
- `inspect-elf ELF`: inspect an ELF32 ARM image.
- `disasm-window ELF [--addr ADDR] [--count N]`: decode Thumb instructions;
  defaults to the entry point with the Thumb bit cleared and 32 instructions.
- `--help` / `-h` and `--version`: show usage or version.

## Compare with hardware

`fil compare-stlink board.json` **controls and resets a physical target**, even
without flashing. Read [Hardware comparison](hardware_comparison.md) first.

| Option | Purpose / default |
| --- | --- |
| `--flash` | Explicit permission to flash; off by default. |
| `--memory ADDR:LENGTH` | Repeatable memory range; none by default. |
| `--register NAME` / `--ignore-register NAME` | Repeatable register selection/exclusion; all known registers selected by default. |
| `--stop-address ADDR` / `--stop-at-symbol NAME` | Optional, mutually exclusive stop controls. |
| `--max-instructions N` | Config value; normally 50,000,000. |
| `--strict-mmio` / `--lenient-mmio` | Default lenient. |
| `--serial ID` | Select a probe; no explicit selection by default. |
| `--openocd PATH` | Default `openocd`. |
| `--timeout-ms N` | Default 10,000. |
| `--artifacts DIRECTORY` | Save comparison artifacts; unset by default. |
