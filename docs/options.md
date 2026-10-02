# Options and recommended defaults

This is the CLI/build reference for the current source. Run `fil --help` for a
short synopsis. Flags belong to the command listed below, not to every command.
For board/network JSON fields see [Configuration](configuration.md).

## Opinionated policy

Prefer **Release + supported IPO, the interpreter, proven-loop batching, and ADC
decimation 1**. This keeps the modeled peripheral behavior intact without paying
experimental compilation/rollback overhead. Keep spin detection and instruction
tracing off unless diagnosing a failure. Use `--strict-mmio` for validation so
missing peripheral coverage is visible; the CLI's compatibility default remains
lenient. Do not promote JIT or experimental schedulers based on synthetic tests.
Measure the actual firmware first, including startup.

For interactive use retain wall pacing and the default `can_tx` filter. Add
`can_rx` only when needed. For throughput use `run-network`, not a paced monitor.
PGO is an explicit workload-specific build, not a universal default. Never change
ADC decimation to obtain a purported fidelity-preserving speedup.

```bash
make BUILD_DIR=build-release BUILD_TYPE=Release IPO=ON test
./build-release/fil run-network configs/networks/per_vehicle.json \
  --duration-ms 1000 --max-instructions 50000000 --quantum 1024 \
  --strict-mmio --adc-decimation 1
./build-release/fil watch-network configs/networks/per_vehicle.json \
  --strict-mmio --refresh-ms 1 --live-filter can_tx
```

These are recommendations, not hidden overrides. The actual defaults follow.

## Global and inspection commands

- `fil --help` / `fil -h`: help. `fil --version`: version.
- `inspect-config PATH`: validate/normalize one MCU or board JSON; no flags.
- `inspect-elf PATH`: inspect one ELF32 ARM image; no flags.
- `disasm-window ELF [--addr ADDR] [--count N]`: address defaults to the ELF entry
  point with Thumb bit cleared; count defaults to **32**.

## Execution flags

`run BOARD`, `run-network NETWORK`, `watch-network NETWORK`, and
`serve-network NETWORK --transport stdio` share these execution options:

| Flag | Actual default / contract |
| --- | --- |
| `--duration-ms N` | `run`: board config `default_duration_ms` (schema default 1000); network: 1000; watch: unlimited. Watch `0` explicitly means unlimited. |
| `--max-instructions N` | `run`: board config `max_instructions` (schema default 50,000,000); network: 50,000,000 per board; watch: 50,000,000 per board **per slice**. |
| `--trace FILE` | `run` and `run-network` write JSONL only when requested. `watch-network` and `serve-network` do not accept this option. See [Tracing](tracing.md). |
| `--trace-instr` | Off; enables instruction events and prevents batched execution. |
| `--strict-mmio` / `--lenient-mmio` | Lenient; strict faults on unmodeled MMIO. |
| `--detect-spin` | Off; diagnostic stopping on proven repeated CPU state. Legitimate idle loops can trigger it. |
| `--no-loop-batching` | Batching is on; disables proven-loop skipping. |
| `--jit` | Off; enables cached handlers and native LLVM when built/eligible. Not a speed guarantee. |
| `--adc-decimation N` | 1; range 1–1024. Values above 1 change continuous ADC sample/DMA/interrupt behavior. |

`run` and `run-network` additionally accept `--no-detect-spin` and
`--loop-batching` to explicitly restore those settings, and `--allow-breakpoint`
(default off) to permit a breakpoint stop without treating it as failure.

### Single-board only

`--stop-address ADDR` and `--stop-at-symbol NAME` are unset by default. If both
are passed to `run`, symbol resolution takes precedence over the address.

### Network only

| Flag | Actual default / contract |
| --- | --- |
| `--quantum N` | 1024; must be nonzero. Scheduling work cap, not permission to cross observable events. Also accepted by watch and serve. |
| `--inject-can BUS[@TIME_MS]:ID:HEXDATA` | None; repeatable scheduled CAN injection. Omitted time means time zero. |
| `--transactional-slices` / `--no-transactional-slices` | Off; experimental reversible parallel lane epochs. Worker execution requires multiple boards, no tracing, and no spin detection. |
| `--deferred-prefixes` | Off; experimental pure deferred prefixes. Requires `--jit`; incompatible with instruction tracing, spin detection, transactional slices, and RAM capsules. |
| `--ram-capsules` | Off; experimental reversible private-RAM prefixes, including guarded spans across other boards' audited ADC events. Every conversion is retained; custom callbacks and unsupported DMA paths remain global observation barriers. Requires `--jit --no-loop-batching`; incompatible with deferred prefixes, instruction tracing, spin detection, and transactional slices. |

Experimental scheduler flags above are **run-network only**. See
[Performance](performance.md) for observation barriers and ownership restrictions.
There is no current `--execution-windows` CLI flag.

### Watch and serve

`watch-network` uses human-readable text on stdio. `serve-network` requires
`--transport stdio` and uses the binary protocol documented in
[Binary network service](serve_network.md); it shares watch pacing, filtering,
and simulation flags but never writes text to stdout.

| Flag | Actual default / contract |
| --- | --- |
| `--refresh-ms N` | 1 simulated ms per slice; must be nonzero. |
| `--live-filter TYPE` / `--trace-type TYPE` | `can_tx`; first explicit filter replaces the default. Repeat for multiple exact types. |
| `--no-wall-pacing` | Pacing is on; disable to run slices back-to-back while retaining live I/O. |
| `--control-stdin` | Watch-only accepted compatibility marker; stdio control is always active. Serve rejects it. |

`watch-network` stdin accepts `BUS:ID:HEXDATA`, `adc BOARD INSTANCE CHANNEL VALUE`,
`gpio BOARD PORT PIN 0|1|release`, `quit`, and `exit`. The binary `serve-network`
request/trace format is specified in [Binary network service](serve_network.md).
Both commands stop on clean EOF when unlimited; with a finite duration, EOF does
not stop the simulation. See [Live monitoring](watch_network.md) for the text
syntax and pacing behavior.

## Hardware comparison

`compare-stlink BOARD` controls/resets a **physical target** even without flashing.

| Flag | Actual default / contract |
| --- | --- |
| `--flash` | Off; explicit permission to flash the configured ELF. |
| `--memory ADDR:LENGTH` | No memory ranges; repeatable. |
| `--register NAME` | All known registers when no selection is given; repeatable. |
| `--ignore-register NAME` | None; repeatable. |
| `--stop-address ADDR` / `--stop-at-symbol NAME` | Unset; mutually exclusive for this command. |
| `--max-instructions N` | Board config value, schema default 50,000,000. |
| `--serial ID` | No explicit probe selection. |
| `--openocd PATH` | `openocd`. |
| `--timeout-ms N` | 10,000. |
| `--artifacts DIRECTORY` | No explicit artifact directory. |
| `--strict-mmio` / `--lenient-mmio` | Lenient. |

See [Hardware comparison](hardware_comparison.md) before connecting a target.

## CMake build options

| Cache variable | Default / purpose |
| --- | --- |
| `CMAKE_BUILD_TYPE` | Release when unset for a single-config generator; choose explicitly for multi-config builds. |
| `FIL_BUILD_TESTS` | ON; GoogleTest suite. |
| `FIL_BUILD_DOCS` | ON; adds docs target if Doxygen is found. |
| `FIL_ENABLE_IPO` | ON; supported optimized configurations only, disabled with sanitizers. |
| `FIL_ENABLE_ASAN`, `FIL_ENABLE_UBSAN` | OFF; diagnostic builds, not throughput baselines. |
| `FIL_ENABLE_LLVM_JIT` | ON; opportunistic LLVM >=18 discovery on a 64-bit host. Runtime JIT still defaults off. |
| `FIL_LLVM_COMPONENT_LINKING` | OFF; prefer monolithic LLVM when available. |
| `FIL_PGO_GENERATE` | OFF; Clang profile instrumentation. |
| `FIL_PGO_PROFILE` | Empty; path to an existing Clang `.profdata`. Mutually exclusive with generation; neither PGO mode supports configured sanitizers. |
| `FIL_PER_FIRMWARE_DIR` | Empty; opt-in external PER compatibility tests. |
| `LLVM_DIR` | Optional LLVM CMake installation override. |

## Make shortcuts and overrides

`make help` lists targets. `make` builds; `test`/`check`, `test-list`, `test-per`,
`release`, `debug`, `test-debug`, `sanitize`, `test-sanitize`, `docs`,
`firmware-tests`/`fixtures`, and `benchmark` provide common workflows.
`clean`/`clean-all` remove only configured root-level build directories.

- Build: `BUILD_DIR=build`, `BUILD_TYPE=Release`, `JOBS`=host CPU count,
  `TESTS=ON`, `DOCS=ON`, `IPO=ON`, `ASAN=OFF`, `UBSAN=OFF`,
  `PGO_GENERATE=OFF`, empty `FIL_PGO_PROFILE` and `PER_FIRMWARE_DIR`.
  `GENERATOR`, `CMAKE_ARGS`, `BUILD_ARGS`, `CTEST_ARGS` pass extra settings.
  Tools are overridable with `CMAKE`, `CTEST`, `PYTHON`.
- Directory presets: `DEBUG_BUILD_DIR=build-debug`,
  `RELEASE_BUILD_DIR=build-release`, `SANITIZE_BUILD_DIR=build-sanitize`,
  `PGO_GENERATE_BUILD_DIR=build-pgo-generate`, `PGO_BUILD_DIR=build-pgo`.
- CLI wrappers: `cli CLI_ARGS='...'` (default `--help`), `app-help`, `version`,
  `run RUN_CONFIG=... RUN_ARGS='...'`,
  `run-network NETWORK_CONFIG=... NETWORK_ARGS='...'`,
  `watch-network NETWORK_CONFIG=... WATCH_ARGS='...'`.
  Network wrapper defaults to 1000 ms/quantum 1024; watch wrapper uses refresh
  10 ms and both CAN directions, unlike the CLI's 1 ms/`can_tx` defaults.
  The single-board wrapper uses the hardware-comparison fixture and
  `--allow-breakpoint`.
- Inspection: `inspect-config CONFIG=...`, `inspect-elf ELF=...`,
  `disasm-window ELF=... DISASM_ADDR=... DISASM_COUNT=...` (wrapper count 8,
  address `0x08000000`, unlike CLI defaults).
- Hardware: `compare-stlink CONFIRM_STLINK=YES COMPARE_CONFIG=... COMPARE_ARGS='...'`;
  confirmation is required and flashing is never implicit.
- Synthetic benchmark: `BENCH_REPS=3`, `BENCH_ARGS='...'`; not proof of real
  firmware throughput.
- PGO: `pgo` cleans profiles, generates, trains, merges, rebuilds, and tests.
  Individual targets: `pgo-generate`, `pgo-train`, `pgo-merge`, `pgo-use`,
  `pgo-test`, `clean-pgo`. `PGO_CXX=clang++`; `LLVM_PROFDATA` is auto-selected
  (PATH, then `xcrun` on macOS). Profile defaults:
  `PGO_PROFILE=build-pgo-generate/fil.profdata`,
  `PGO_PROFILE_PATTERN=build-pgo-generate/fil-%p.profraw`.
  `PGO_TRAIN_ARGS` defaults to the six-board PER network, 1000 ms,
  50,000,000 instructions, quantum 1024, strict MMIO, JIT, ADC decimation 1.
  **Override it to match the interpreter/scheduler you deploy**; the default
  training command is not the runtime default.

For reproducible real-network measurements see [Performance](performance.md).
