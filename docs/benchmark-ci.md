# Benchmark CI

Every commit is built and benchmarked on **x86_64** (`ubuntu-24.04`) and
**aarch64** (`ubuntu-24.04-arm`). Releases ship portable binaries for
Linux x86_64/aarch64 and macOS arm64.

## Workflows

| Workflow | Trigger | Purpose |
|---|---|---|
| `ci.yml` | push to `main`, PRs | Build + `ctest` on both archs (Release, portable) |
| `benchmark.yml` | push to `main`, PRs, manual | Full real-firmware network benchmark, one JSONL row per (commit, arch) |
| `release.yml` | tag `v*` | Portable `fil` binaries + `SHA256SUMS` on a GitHub Release |

`benchmark.yml` drives the same script with `--commits <sha>` for a single
commit, so the per-commit path and range sweeps share one code path.

## Benchmark workload

`tools/bench_full_network.sh` runs the six-board `per_vehicle` network for
5000 ms simulated time (`--strict-mmio --adc-decimation 1
--cross-board-window-ns 100000`), 3 reps, and reports the median wall time,
`instructions`, `cycles`, per-board clocks, and throughput. The hard gate is
realtime ratio above 1.0; trends come from the recorded history.

Builds are **Release + IPO, `NATIVE=OFF`** (portable). Host tuning is
deliberately off so numbers stay comparable across runners and years; the
exact compiler and flags are recorded in every row.

## Firmware provenance

Board ELFs come from the `PurdueElectricRacing/monorepo` **Build Firmware**
workflow artifacts (`firmware-master-*`), resolved by
`tools/fetch_per_firmware.sh`:

- `latest` — the most recent non-expired master artifact.
- `contemporary <date>` — the newest non-expired artifact with
  `created_at <= date`; if every older artifact has expired (GitHub keeps
  master artifacts 90 days), the oldest available artifact is used and the
  row records `"firmware_provenance": "fallback-oldest"`.

`benchmark.yml` uses contemporary-to-commit-date (which equals latest for new
commits). The driver pre-fetches one artifact catalog and resolves each commit
locally, downloading each distinct firmware version once.

`tools/make_ci_network.py` generates a network config from **that commit's
own** `configs/` templates with only the `elf`/`mcu` paths repointed at
absolute locations, so each era is benchmarked under its own schema, bus
names, and scheduling defaults. Boards missing from the artifact are dropped
(and recorded); a commit with zero runnable boards is recorded as failed,
not retried forever.

## Era compatibility

`bench_full_network.sh` probes each binary and adapts:

- `run-network` vs `run` command name,
- `--duration-ms` vs `--duration`, `--max-instructions` vs `--instruction-budget`,
- optional `--quantum`, `--strict-mmio`, `--adc-decimation 1`,
  `--cross-board-window-ns` (omitted when the binary predates them).

Validation requires the simulated counters and a time-budget stop. Era
mismatches (board count, missing clock reports, clock rate changes, stop
reasons) are fatal in strict mode (used for current commits) and warnings in
`--lenient` mode (used for history). The firmware clock check is consistency,
not a fixed value: all boards must agree (`--expect-hz` opts into an absolute
value). Current PER firmware runs at 16 MHz HSE, which is recorded in
`clocks_hz` rather than rejected. The probe results, warnings, and clocks
are all part of the machine-readable output (`--json-output`).

The driver tries `--lenient` with strict MMIO first, then retries with lenient
MMIO; both attempts are recorded. Unbuildable commits, unparseable outputs,
and genuine emulation gaps (e.g. an instruction the old fil never
implemented) become `build_failed`/`bench_failed` rows with reasons — one bad
era never stops the sweep, and reruns skip commits already recorded.

## History (`benchmark-data` branch)

`results/x86_64.jsonl` and `results/aarch64.jsonl`, one object per line:

```json
{"fil_sha": "...", "fil_date": "...", "arch": "x86_64", "os": "linux",
 "per_sha": "...", "per_artifact": "firmware-master-abc1234",
 "firmware_provenance": "exact",
 "status": "success", "attempts": [...],
 "bench": {"median_wall_s": "0.512", "throughput_x": "9.766",
           "instructions": ["194032368", ...], "clocks_hz": {...},
           "included_flags": [...], "warnings": [...], ...}}
```

Query it locally, e.g. median wall over time:

```bash
git clone --branch benchmark-data <repo> bench-data
python3 -c "
import json
for line in open('bench-data/results/x86_64.jsonl'):
    r = json.loads(line)
    if r.get('status') == 'success':
        print(r['fil_date'][:10], r['fil_sha'][:7],
              r['bench']['median_wall_s'], r['per_artifact'][-7:])
"
```

## Releases

```bash
git tag v0.2.0 && git push origin v0.2.0
```

`release.yml` builds portable Release binaries on Linux x86_64, Linux
aarch64, and macOS arm64 (no tests/docs in the binaries, same code as CI),
smoke-tests `--version`/`--help`, and attaches
`fil-linux-x86_64`, `fil-linux-aarch64`, `fil-macos-arm64`, and `SHA256SUMS`
to the release with generated notes. There is no macOS Intel runner; Intel
Macs can run the arm64 binary under Rosetta or build from source.
