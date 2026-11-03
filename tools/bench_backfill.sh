#!/usr/bin/env bash
# Benchmark a range of fil commits against era-appropriate PER firmware.
#
# For each selected commit: check out the commit in an isolated worktree,
# resolve the newest PER firmware artifact not newer than the commit date
# (fallback: oldest available, recorded as fallback-oldest), build fil
# (Release, portable), generate a network config from that commit's own
# configs repointed at the artifact ELFs, and run tools/bench_full_network.sh
# with lenient era validation (strict MMIO first, lenient MMIO retry).
#
# Every commit produces exactly one JSONL row (success or recorded failure),
# so one broken era never stops the sweep. Safe to re-run: commits already
# present in --skip-from are skipped.
#
# The tools used (this script, bench_full_network.sh, make_ci_network.py,
# fetch_per_firmware.sh) always come from the CURRENT checkout; only the
# built sources and config templates come from each historical commit.
#
# Usage:
#   tools/bench_backfill.sh --to HEAD [--from SHA] [--include-from]
#     [--commits "sha..."] [--shard-index N --shard-count M]
#     --catalog FILE --results FILE
#     [--duration-ms 5000] [--reps 3] [--work-root DIR] [--firmware-cache DIR]
#     [--skip-from FILE] [--arch ARCH] [--os OS] [--compiler STR]

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd -P)

FROM=""
TO="HEAD"
INCLUDE_FROM=0
COMMITS=""
SHARD_INDEX=0
SHARD_COUNT=1
CATALOG=""
RESULTS=""
DURATION_MS=5000
REPS=3
WORK_ROOT="${TMPDIR:-/tmp}/fil-backfill"
FIRMWARE_CACHE=""
SKIP_FROM=""
ARCH="$(uname -m)"
OS="$(uname -s)"
COMPILER=""

usage() {
    sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
}

fail() {
    printf 'bench_backfill.sh: %s\n' "$*" >&2
    exit 2
}

while (($#)); do
    case $1 in
        --from) FROM=$2; shift 2 ;;
        --to) TO=$2; shift 2 ;;
        --include-from) INCLUDE_FROM=1; shift ;;
        --commits) COMMITS=$2; shift 2 ;;
        --shard-index) SHARD_INDEX=$2; shift 2 ;;
        --shard-count) SHARD_COUNT=$2; shift 2 ;;
        --catalog) CATALOG=$2; shift 2 ;;
        --results) RESULTS=$2; shift 2 ;;
        --duration-ms) DURATION_MS=$2; shift 2 ;;
        --reps) REPS=$2; shift 2 ;;
        --work-root) WORK_ROOT=$2; shift 2 ;;
        --firmware-cache) FIRMWARE_CACHE=$2; shift 2 ;;
        --skip-from) SKIP_FROM=$2; shift 2 ;;
        --arch) ARCH=$2; shift 2 ;;
        --os) OS=$2; shift 2 ;;
        --compiler) COMPILER=$2; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) fail "unrecognized argument: $1" ;;
    esac
done

[[ -n $CATALOG ]] || fail '--catalog is required'
[[ -f $CATALOG ]] || fail "catalog not found: $CATALOG"
[[ -n $RESULTS ]] || fail '--results is required'
((SHARD_COUNT >= 1)) || fail '--shard-count must be >= 1'
((SHARD_INDEX >= 0 && SHARD_INDEX < SHARD_COUNT)) || fail '--shard-index out of range'
command -v git >/dev/null || fail 'git is required'
command -v cmake >/dev/null || fail 'cmake is required'
command -v python3 >/dev/null || fail 'python3 is required'
command -v gh >/dev/null || fail 'gh CLI is required (set GH_TOKEN for auth)'
command -v unzip >/dev/null || fail 'unzip is required'

FIRMWARE_CACHE=${FIRMWARE_CACHE:-$WORK_ROOT/firmware}
mkdir -p "$WORK_ROOT" "$FIRMWARE_CACHE"
: >> "$RESULTS"

if [[ -z $COMPILER ]]; then
    CXX_USED=${CXX:-c++}
    COMPILER=$($CXX_USED --version 2>/dev/null | head -1 || echo unknown)
fi

# Commit list, oldest first.
if [[ -n $COMMITS ]]; then
    # shellcheck disable=SC2206
    ALL_COMMITS=($COMMITS)
else
    RANGE="$TO"
    if [[ -n $FROM ]]; then
        if ((INCLUDE_FROM)); then
            if git -C "$REPO_ROOT" rev-parse --verify --quiet "$FROM^" >/dev/null; then
                RANGE="$FROM^..$TO"
            else
                RANGE="$TO"  # FROM is the root: take everything up to TO.
            fi
        else
            RANGE="$FROM..$TO"
        fi
    fi
    ALL_COMMITS=()
    while IFS= read -r line; do
        [[ -n $line ]] && ALL_COMMITS+=("$line")
    done < <(git -C "$REPO_ROOT" rev-list --reverse --topo-order "$RANGE")
fi
TOTAL=${#ALL_COMMITS[@]}
((TOTAL > 0)) || { printf 'bench_backfill.sh: no commits selected\n' >&2; exit 0; }
printf 'bench_backfill.sh: %d commits, shard %d/%d\n' "$TOTAL" "$SHARD_INDEX" "$SHARD_COUNT" >&2

WT="$WORK_ROOT/worktree-$SHARD_INDEX"
BUILD_DIR="$WORK_ROOT/build-$SHARD_INDEX"
NET_DIR="$WORK_ROOT/net-$SHARD_INDEX"
mkdir -p "$BUILD_DIR" "$NET_DIR"

cleanup_worktree() {
    git -C "$REPO_ROOT" worktree remove --force "$WT" 2>/dev/null || true
}
trap cleanup_worktree EXIT

# Resolve newest catalog artifact with created_at <= ISO date (else oldest).
resolve_firmware() {
    python3 -c '
import sys
want = sys.argv[1]
rows = [l.rstrip("\n").split("\t") for l in open(sys.argv[2]) if l.strip()]
rows.sort(key=lambda r: r[0], reverse=True)
older = [r for r in rows if r[0] <= want]
if older:
    pick, prov = older[0], "exact"
else:
    pick, prov = rows[-1], "fallback-oldest"
print("\t".join(pick) + "\t" + prov)
' "$1" "$CATALOG"
}

# Append one JSONL row. Args: k=v pairs via env ROW_* + bench json path (or empty).
append_row() {
    python3 -c '
import json, os, sys
row = {}
for key, value in os.environ.items():
    if key.startswith("ROW_"):
        name = key[4:]
        if name in ("BOARDS", "DROPPED", "ATTEMPTS"):
            row[name.lower()] = json.loads(value) if value else []
        elif name in ("EXPECT_BOARDS", "DURATION_MS", "REPS", "WINDOW_NS"):
            row[name.lower()] = int(value)
        else:
            row[name.lower()] = value
bench_path = sys.argv[1]
if bench_path:
    with open(bench_path) as handle:
        row["bench"] = json.load(handle)
else:
    row["bench"] = None
with open(os.environ["RESULTS"], "a") as handle:
    handle.write(json.dumps(row, sort_keys=True) + "\n")
' "$1"
}

export RESULTS

already_done() {
    [[ -n $SKIP_FROM && -f $SKIP_FROM ]] && grep -Fq "\"fil_sha\": \"$1\"" "$SKIP_FROM"
}

INDEX=0
for SHA in "${ALL_COMMITS[@]}"; do
    COMMIT_INDEX=$INDEX
    INDEX=$((INDEX + 1))
    ((COMMIT_INDEX % SHARD_COUNT == SHARD_INDEX)) || continue

    if already_done "$SHA"; then
        printf 'bench_backfill.sh: [%s] already recorded, skipping\n' "$SHA" >&2
        continue
    fi

    FIL_DATE=$(git -C "$REPO_ROOT" show -s --format=%cI "$SHA")
    FIL_SUBJECT=$(git -C "$REPO_ROOT" show -s --format=%s "$SHA")

    # Fresh row environment per commit.
    for var in $(compgen -e ROW_); do unset "$var"; done
    export ROW_FIL_SHA="$SHA" ROW_FIL_DATE="$FIL_DATE" ROW_FIL_SUBJECT="$FIL_SUBJECT"
    export ROW_ARCH="$ARCH" ROW_OS="$OS" ROW_COMPILER="$COMPILER"
    export ROW_DURATION_MS="$DURATION_MS" ROW_REPS="$REPS" ROW_WINDOW_NS=100000
    export ROW_BOARDS="[]" ROW_DROPPED="[]" ROW_ATTEMPTS="[]"
    export ROW_STATUS="" ROW_REASON="" ROW_EXPECT_BOARDS=0
    export ROW_PER_SHA="" ROW_PER_RUN_ID="" ROW_PER_ARTIFACT=""
    export ROW_PER_CREATED_AT="" ROW_FIRMWARE_PROVENANCE=""

    printf 'bench_backfill.sh: [%s] %s %s\n' "$SHA" "$FIL_DATE" "$FIL_SUBJECT" >&2

    # Firmware for this era.
    FW_SEL=$(resolve_firmware "$FIL_DATE") \
        || { export ROW_STATUS=no_firmware ROW_REASON="firmware resolution failed"; append_row ""; continue; }
    IFS=$'\t' read -r FW_CREATED FW_ID FW_NAME FW_RUN FW_SHA FW_PROV <<< "$FW_SEL"
    [[ -n ${FW_ID:-} ]] \
        || { export ROW_STATUS=no_firmware ROW_REASON="firmware resolution failed"; append_row ""; continue; }
    export ROW_PER_SHA="$FW_SHA" ROW_PER_RUN_ID="$FW_RUN" ROW_PER_ARTIFACT="$FW_NAME"
    export ROW_PER_CREATED_AT="$FW_CREATED" ROW_FIRMWARE_PROVENANCE="$FW_PROV"

    # Atomic download: a failed fetch must not poison the cache.
    FW_ZIP="$FIRMWARE_CACHE/$FW_NAME.zip"
    if [[ ! -f $FW_ZIP ]]; then
        printf 'bench_backfill.sh: downloading %s\n' "$FW_NAME" >&2
        if ! gh api "repos/PurdueElectricRacing/monorepo/actions/artifacts/$FW_ID/zip" > "$FW_ZIP.tmp"; then
            rm -f "$FW_ZIP.tmp"
            export ROW_STATUS=no_firmware ROW_REASON="artifact download failed: $FW_NAME"
            append_row ""
            continue
        fi
        mv "$FW_ZIP.tmp" "$FW_ZIP"
    fi
    FW_UNPACK="$FIRMWARE_CACHE/unpacked-$FW_NAME"
    if [[ ! -d $FW_UNPACK ]]; then
        rm -rf "$FW_UNPACK.tmp" "$FW_UNPACK"
        mkdir -p "$FW_UNPACK.tmp"
        if ! unzip -o -q "$FW_ZIP" -d "$FW_UNPACK.tmp"; then
            rm -rf "$FW_UNPACK.tmp"
            export ROW_STATUS=no_firmware ROW_REASON="artifact unpack failed: $FW_NAME"
            append_row ""
            continue
        fi
        mv "$FW_UNPACK.tmp" "$FW_UNPACK"
    fi
    if [[ -f "$FW_UNPACK/firmware/output/dashboard/dashboard.elf" ]]; then
        FW_DIR="$FW_UNPACK/firmware/output"
    elif [[ -f "$FW_UNPACK/output/dashboard/dashboard.elf" ]]; then
        FW_DIR="$FW_UNPACK/output"
    else
        FOUND=$(find "$FW_UNPACK" -path '*dashboard/dashboard.elf' -print -quit 2>/dev/null || true)
        if [[ -n $FOUND ]]; then FW_DIR=$(dirname "$(dirname "$FOUND")"); else FW_DIR=""; fi
    fi
    if [[ -z ${FW_DIR:-} ]]; then
        export ROW_STATUS=no_firmware ROW_REASON="no dashboard firmware in $FW_NAME"
        append_row ""
        continue
    fi

    # Check out the commit in this shard's worktree (recovering from any
    # stale directory left by an interrupted run).
    if ! git -C "$WT" rev-parse --git-dir >/dev/null 2>&1; then
        git -C "$REPO_ROOT" worktree remove --force "$WT" 2>/dev/null || true
        rm -rf "$WT"
        git -C "$REPO_ROOT" worktree add --detach "$WT" "$SHA" >&2 \
            || { export ROW_STATUS=build_failed ROW_REASON="worktree checkout failed"; append_row ""; continue; }
    else
        git -C "$WT" checkout -q "$SHA" >&2 \
            || { export ROW_STATUS=build_failed ROW_REASON="worktree checkout failed"; append_row ""; continue; }
    fi

    # Build (portable Release; tests/docs off for speed).
    BUILD_LOG="$WORK_ROOT/build-$SHARD_INDEX-$SHA.log"
    CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release
        -DFIL_BUILD_TESTS=OFF -DFIL_BUILD_DOCS=OFF
        -DFIL_ENABLE_IPO=ON -DFIL_ENABLE_NATIVE=OFF)
    if command -v ccache >/dev/null 2>&1; then
        CMAKE_ARGS+=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
    fi
    if ! cmake -S "$WT" -B "$BUILD_DIR" "${CMAKE_ARGS[@]}" >"$BUILD_LOG" 2>&1; then
        export ROW_STATUS=build_failed ROW_REASON="cmake configure failed"
        append_row ""
        continue
    fi
    if ! cmake --build "$BUILD_DIR" --parallel "${JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)}" \
            >>"$BUILD_LOG" 2>&1; then
        export ROW_STATUS=build_failed ROW_REASON="build failed"
        append_row ""
        continue
    fi
    FIL_BIN="$BUILD_DIR/fil"
    [[ -x $FIL_BIN ]] || { export ROW_STATUS=build_failed ROW_REASON="fil binary missing after build"; append_row ""; continue; }

    # Era network config from this commit's own templates.
    NET_OUT="$NET_DIR/$SHA"
    rm -rf "$NET_OUT"; mkdir -p "$NET_OUT"
    GEN_LOG="$WORK_ROOT/net-$SHARD_INDEX-$SHA.log"
    if ! python3 "$SCRIPT_DIR/make_ci_network.py" --repo "$WT" \
            --firmware-dir "$FW_DIR" --out-dir "$NET_OUT" >"$GEN_LOG" 2>&1; then
        export ROW_STATUS=bench_failed ROW_REASON="network generation failed"
        append_row ""
        continue
    fi
    GEN_VARS="$WORK_ROOT/net-$SHARD_INDEX-$SHA.vars.json"
    python3 - "$GEN_LOG" "$GEN_VARS" <<'EOF'
import json, sys
vals = {}
with open(sys.argv[1]) as fh:
    for line in fh:
        line = line.rstrip("\n")
        if "=" in line and not line.startswith((" ", "\t", "warning")):
            key, value = line.split("=", 1)
            if key in ("NETWORK", "BOARDS", "DROPPED", "EXPECT_BOARDS"):
                vals[key] = value
with open(sys.argv[2], "w") as fh:
    json.dump(vals, fh)
EOF
    NETWORK=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("NETWORK",""))' "$GEN_VARS")
    GEN_EXPECT=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("EXPECT_BOARDS","0"))' "$GEN_VARS")
    GEN_BOARDS=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("BOARDS",""))' "$GEN_VARS")
    GEN_DROPPED=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("DROPPED",""))' "$GEN_VARS")
    export ROW_EXPECT_BOARDS="$GEN_EXPECT"
    BOARDS_JSON=$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1].split() if len(sys.argv)>1 and sys.argv[1] else []))' "$GEN_BOARDS")
    DROPPED_JSON=$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1].split() if len(sys.argv)>1 and sys.argv[1] else []))' "$GEN_DROPPED")
    export ROW_BOARDS="$BOARDS_JSON" ROW_DROPPED="$DROPPED_JSON"
    if [[ -z $NETWORK || $GEN_EXPECT -eq 0 ]]; then
        export ROW_STATUS=bench_failed ROW_REASON="no boards with firmware available"
        append_row ""
        continue
    fi

    # Benchmark: strict MMIO first, lenient MMIO retry. Era validation lenient.
    BENCH_JSON="$WORK_ROOT/bench-$SHARD_INDEX-$SHA.json"
    ATTEMPTS_JSON="[]"
    BENCH_OK=0
    for MMIO in strict lenient; do
        rm -f "$BENCH_JSON"
        BENCH_LOG="$WORK_ROOT/bench-$SHARD_INDEX-$SHA-$MMIO.log"
        if "$SCRIPT_DIR/bench_full_network.sh" \
                --binary "$FIL_BIN" --network "$NETWORK" \
                --duration-ms "$DURATION_MS" --reps "$REPS" \
                --expect-boards "$GEN_EXPECT" --mmio "$MMIO" --lenient \
                --json-output "$BENCH_JSON" >"$BENCH_LOG" 2>&1; then
            BENCH_OK=1
            ATTEMPTS_JSON=$(python3 -c 'import json,sys; print(json.dumps([{"mmio":sys.argv[1],"validation":"lenient","status":"success"}]))' "$MMIO")
            if [[ $MMIO == lenient ]]; then
                ATTEMPTS_JSON=$(python3 -c 'import json; print(json.dumps([{"mmio":"strict","validation":"lenient","status":"failed"},{"mmio":"lenient","validation":"lenient","status":"success"}]))')
            fi
            break
        fi
    done
    export ROW_ATTEMPTS="$ATTEMPTS_JSON"
    if ((BENCH_OK)); then
        export ROW_STATUS=success ROW_REASON=""
        append_row "$BENCH_JSON"
        printf 'bench_backfill.sh: [%s] success\n' "$SHA" >&2
    else
        export ROW_ATTEMPTS='[{"mmio":"strict","validation":"lenient","status":"failed"},{"mmio":"lenient","validation":"lenient","status":"failed"}]'
        TAIL=$(tail -n 5 "$WORK_ROOT/bench-$SHARD_INDEX-$SHA-lenient.log" 2>/dev/null | tr '\n' ';' | cut -c1-500)
        export ROW_STATUS=bench_failed ROW_REASON="benchmark failed (strict+lenient mmio): $TAIL"
        append_row ""
        printf 'bench_backfill.sh: [%s] benchmark failed\n' "$SHA" >&2
    fi
done

printf 'bench_backfill.sh: shard %d done\n' "$SHARD_INDEX" >&2
