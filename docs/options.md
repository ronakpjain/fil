# CLI options

Run `fil --help` for a synopsis. For JSON inputs, see
[Configuration](configuration.md); for building, see the [README](../README.md)
and `make help`.

## Defaults

JIT is automatic. Run, watch, and serve
have unlimited simulated time and instruction budgets by default. A board config's
nonzero `run.default_duration_ms` and `run.max_instructions` override those
unlimited defaults; network configs use unlimited defaults. Network commands use
independent board slices with a **1000 ns cross-board synchronization window**.
This is a bounded-skew communication model, not exact cross-board instruction
ordering. ADC, timers, DMA and interrupts still execute locally at full fidelity.
Instruction tracing, spin detection and the alternative loop scheduler retain
exact scheduling fallbacks.

**ADC decimation defaults to 1 (full fidelity).** `--adc-decimation N` accepts
1–1024; values above 1 trade fidelity for speed by skipping continuous ADC scans
and their sample, DMA, and interrupt effects.

## Run a board or network

| Command | Input / behavior |
| --- | --- |
| `fil run <config.json>` | Detects a board or network config and runs it to completion by default. |
| `fil watch <config.json>` | Live monitor for a network config; a board config is accepted as a one-board network. |
| `fil serve <config.json>` | Binary stdio service for a network config; a board config is accepted as a one-board network. |

`serve` supports stdio only; it does not take a transport option.

Run prints an indented human-readable report: the first line names the board or
network, summary fields are aligned underneath it, and each network board appears
in its own `board:` section. Simulated time is shown with a unit and the exact
nanosecond count. Stimulus expectations print as a dedicated block after the
summary, with failures first; `--jit-stats` adds opt-in internal JIT counters.
The report uses colors only when stdout is an interactive terminal; a non-empty
`NO_COLOR` environment variable disables them.

```bash
fil run board.json --duration 1s
fil run network.json --duration 1s --strict-mmio
```

| Option | Purpose / default |
| --- | --- |
| `--duration D` | Unlimited unless a board config sets nonzero `run.default_duration_ms`; network default is unlimited. Bare values are seconds; `us`, `ms`, `s`, and `m` suffixes are supported. `0` means unlimited. |
| `--instruction-budget N` | Unlimited unless a board config sets nonzero `run.max_instructions`; network default is unlimited, per board. `0` means unlimited. |
| `--strict-mmio` / `--lenient-mmio` | Fault on unmodeled MMIO or allow compatibility behavior; default lenient. |
| `--trace FILE` | Write JSONL events; off by default. |
| `--trace-instr` | Include instruction events; off by default. |
| `--detect-spin` | Stop on proven repeated CPU state; off by default. Can stop legitimate idle loops. `--no-detect-spin` restores the default. |
| `--allow-breakpoint` | Accept a firmware breakpoint as a successful stop; off by default. |
| `--adc-decimation N` | Explicit fidelity-for-speed tradeoff; default 1. |
| `--jit-stats` | Print internal JIT counter groups (debug only) in the run report; off by default. |

Board-only stop controls: `--stop-address ADDR` or `--stop-at-symbol NAME`.
If both are supplied, the symbol takes precedence.

Network-only controls:

- `--quantum N`: exact-scheduler work cap; default 1024, must be nonzero.
- `--parallel-lanes` / `--no-parallel-lanes`: use eligible parallel owner lanes
  by default; disable them for serial validation. Tracing and observation barriers
  can require serial fallback.
- `--cross-board-window-ns N`: independent execution window, default 100000
  (100 µs).
  Shared callbacks run at rendezvous with their logical timestamps preserved;
  a receiving board observes the effect at its next instruction boundary.
  Communication delay is bounded by one window plus ordinary in-flight
  instruction latency. Larger values increase allowed cross-board skew.
  `0` selects the exact reference scheduler with eligible guarded RAM capsules.
- `--inject-can BUS[@TIME_MS]:ID:HEXDATA`: repeatable CAN injection; omitted
  time means time zero.

See [Tracing](tracing.md) and [Performance](performance.md) for details.

## Live monitoring and binary service

```bash
fil watch network.json
fil serve network.json
```

Both accept `--duration`, `--instruction-budget`, `--quantum`, MMIO controls,
`--trace-instr`, `--detect-spin`, `--cross-board-window-ns`,
`--parallel-lanes`, `--no-parallel-lanes`, and `--adc-decimation` from above.
Duration and instruction budget default to unlimited; `0` also means unlimited.
For watch and serve, the instruction budget applies per board per run slice.
Neither accepts `--trace FILE`. Stdio is the only serve transport; `--transport`
is rejected.

| Option | Purpose / default |
| --- | --- |
| `--refresh-ms N` / `--refresh-us N` | Simulated time per slice in milliseconds or microseconds; default 100 µs, must be nonzero. |
| `--live-filter TYPE` / `--trace-type TYPE` | Repeatable event filter; default `can_tx`. The first explicit filter replaces the default. |
| `--no-wall-pacing` | Run slices back-to-back instead of pacing to real time; live I/O stays active. |

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
| `--max-instructions N` | Unlimited unless the board config sets nonzero `run.max_instructions`; `0` means unlimited. |
| `--strict-mmio` / `--lenient-mmio` | Default lenient. |
| `--serial ID` | Select a probe; no explicit selection by default. |
| `--openocd PATH` | Default `openocd`. |
| `--timeout-ms N` | Default 10,000. |
| `--artifacts DIRECTORY` | Save comparison artifacts; unset by default. |
