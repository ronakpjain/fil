#!/usr/bin/env bash
# Fetch PER firmware ELFs from the PurdueElectricRacing/monorepo "Build Firmware"
# workflow artifacts.
#
# Artifact layouts changed over time (output/... vs firmware/output/...); the
# unpacked root is auto-detected and reported as FIRMWARE_DIR, which always
# contains <board>/<board>.elf entries.
#
# Modes:
#   catalog --catalog FILE [--repo R]
#       Dump every non-expired firmware-master-* artifact (newest first) as TSV:
#       created_at, artifact_id, name, run_id, head_sha. Cheap to reuse.
#   latest --dest DIR [--repo R]
#       Most recent non-expired firmware-master-* artifact.
#   contemporary --date ISO8601 --dest DIR [--catalog FILE] [--repo R]
#       Newest non-expired artifact with created_at <= date. When --catalog is
#       given, resolution is local (no API calls); the artifact is then
#       downloaded by id. If nothing qualifies, the oldest available artifact
#       is used and reported as PROVENANCE=fallback-oldest.
#
# latest/contemporary print KEY=VALUE lines on stdout for eval by the caller:
#   PER_SHA, PER_RUN_ID, PER_ARTIFACT, PER_CREATED_AT, PROVENANCE, FIRMWARE_DIR
# Human progress goes to stderr. Requires `gh` and python3.
#
# Example:
#   tools/fetch_per_firmware.sh --mode catalog --catalog /tmp/per-catalog.tsv
#   eval "$(tools/fetch_per_firmware.sh --mode latest --dest /tmp/per-fw)"
#   eval "$(tools/fetch_per_firmware.sh --mode contemporary --date 2026-08-01 \
#     --catalog /tmp/per-catalog.tsv --dest /tmp/per-fw)"

set -euo pipefail

REPO="PurdueElectricRacing/monorepo"
MODE="latest"
DATE=""
DEST=""
CATALOG=""
PER_PAGE=100
MAX_PAGES=100

usage() {
    cat <<'EOF'
Usage: fetch_per_firmware.sh --mode catalog|latest|contemporary [OPTIONS]
  --dest DIR       unpack firmware here (latest/contemporary)
  --catalog FILE   write (catalog) or reuse (contemporary) the artifact catalog
  --date ISO8601   upper bound for contemporary mode
  --repo OWNER/REPO (default: PurdueElectricRacing/monorepo)
  --per-page N     (default: 100)
  --max-pages N    (default: 100)
EOF
}

fail() {
    printf 'fetch_per_firmware.sh: %s\n' "$*" >&2
    exit 2
}

while (($#)); do
    case $1 in
        --mode) MODE=$2; shift 2 ;;
        --date) DATE=$2; shift 2 ;;
        --dest) DEST=$2; shift 2 ;;
        --catalog) CATALOG=$2; shift 2 ;;
        --repo) REPO=$2; shift 2 ;;
        --per-page) PER_PAGE=$2; shift 2 ;;
        --max-pages) MAX_PAGES=$2; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) fail "unrecognized argument: $1" ;;
    esac
done

case $MODE in
    catalog|latest|contemporary) ;;
    *) fail '--mode must be catalog, latest, or contemporary' ;;
esac
if [[ $MODE == contemporary && -z $DATE ]]; then
    fail '--mode contemporary requires --date'
fi
if [[ ($MODE == latest || $MODE == contemporary) && -z $DEST ]]; then
    fail "--mode $MODE requires --dest"
fi
if [[ $MODE == catalog && -z $CATALOG ]]; then
    fail '--mode catalog requires --catalog'
fi
command -v gh >/dev/null 2>&1 || fail 'gh CLI is required'
command -v python3 >/dev/null 2>&1 || fail 'python3 is required'

log() { printf 'fetch_per_firmware.sh: %s\n' "$*" >&2; }

# Build the catalog: all non-expired firmware-master-* artifacts, newest first.
build_catalog() {
    local out=$1 page=1 total=-1
    : > "$out"
    while ((page <= MAX_PAGES)); do
        log "listing artifacts (page $page)..."
        local body
        body=$(gh api "repos/$REPO/actions/artifacts?per_page=$PER_PAGE&page=$page") \
            || fail "failed to list artifacts for $REPO"
        python3 -c '
import sys, json
page = json.load(sys.stdin)
for a in page.get("artifacts", []):
    if a.get("expired"):
        continue
    name = a.get("name", "")
    if not name.startswith("firmware-master-"):
        continue
    wr = a.get("workflow_run") or {}
    print("\t".join([a.get("created_at", ""), str(a.get("id")),
                      name, str(wr.get("id")), wr.get("head_sha", "")]))
' <<< "$body" >> "$out"
        total=$(python3 -c 'import sys,json; print(json.load(sys.stdin).get("total_count", 0))' <<< "$body")
        if ((page * PER_PAGE >= total)); then
            break
        fi
        page=$((page + 1))
    done
    local count
    count=$(wc -l < "$out" | tr -d ' ')
    log "catalog holds $count firmware artifacts"
    ((count > 0)) || fail "no non-expired firmware-master-* artifact found in $REPO"
}

if [[ $MODE == catalog ]]; then
    build_catalog "$CATALOG"
    exit 0
fi

CATALOG_FILE=$CATALOG
if [[ -z $CATALOG_FILE ]]; then
    CATALOG_FILE=$(mktemp "${TMPDIR:-/tmp}/per-catalog.XXXXXXXX.tsv")
    build_catalog "$CATALOG_FILE"
else
    [[ -f $CATALOG_FILE ]] || fail "catalog file not found: $CATALOG_FILE"
fi

export WANT_MODE="$MODE" WANT_DATE="$DATE"
SELECTION=$(python3 -c '
import os, sys
mode = os.environ["WANT_MODE"]
want = os.environ.get("WANT_DATE", "")
rows = [l.rstrip("\n").split("\t") for l in open(sys.argv[1]) if l.strip()]
# rows already newest-first (API order); sort defensively by created_at desc.
rows.sort(key=lambda r: r[0], reverse=True)
if not rows:
    sys.exit(3)
if mode == "latest":
    pick, provenance = rows[0], "exact"
else:
    older = [r for r in rows if r[0] <= want]
    if older:
        pick, provenance = older[0], "exact"
    else:
        pick, provenance = rows[-1], "fallback-oldest"
print("\t".join(pick) + "\t" + provenance)
' "$CATALOG_FILE") || fail "no firmware artifact qualifies"

IFS=$'\t' read -r CREATED_AT ARTIFACT_ID ARTIFACT_NAME RUN_ID HEAD_SHA PROVENANCE <<< "$SELECTION"
[[ -n ${ARTIFACT_ID:-} ]] || fail "failed to select a firmware artifact"
log "selected $ARTIFACT_NAME (run $RUN_ID, sha ${HEAD_SHA:0:7}, created $CREATED_AT, provenance $PROVENANCE)"

mkdir -p "$DEST"
ZIP="$DEST/$ARTIFACT_NAME.zip"
if [[ ! -f $ZIP ]]; then
    log "downloading artifact $ARTIFACT_ID..."
    gh api "repos/$REPO/actions/artifacts/$ARTIFACT_ID/zip" > "$ZIP.tmp" \
        || fail "artifact download failed"
    mv "$ZIP.tmp" "$ZIP"
else
    log "reusing cached $ZIP"
fi
UNPACK="$DEST/unpacked-${ARTIFACT_NAME}"
if [[ ! -d $UNPACK ]]; then
    rm -rf "$UNPACK.tmp" "$UNPACK"
    mkdir -p "$UNPACK.tmp"
    unzip -o -q "$ZIP" -d "$UNPACK.tmp" || fail "failed to unpack firmware artifact"
    mv "$UNPACK.tmp" "$UNPACK"
fi
# Artifact layout changed over time; detect the root holding <board>/<board>.elf.
FIRMWARE_DIR=""
if [[ -f "$UNPACK/firmware/output/dashboard/dashboard.elf" ]]; then
    FIRMWARE_DIR="$UNPACK/firmware/output"
elif [[ -f "$UNPACK/output/dashboard/dashboard.elf" ]]; then
    FIRMWARE_DIR="$UNPACK/output"
else
    FOUND=$(find "$UNPACK" -path '*dashboard/dashboard.elf' -print -quit 2>/dev/null || true)
    if [[ -n $FOUND ]]; then
        FIRMWARE_DIR=$(dirname "$(dirname "$FOUND")")
    fi
fi
[[ -n $FIRMWARE_DIR ]] || fail "no dashboard/dashboard.elf found in $ARTIFACT_NAME"
log "firmware root: $FIRMWARE_DIR"

printf 'PER_SHA=%s\n' "$HEAD_SHA"
printf 'PER_RUN_ID=%s\n' "$RUN_ID"
printf 'PER_ARTIFACT=%s\n' "$ARTIFACT_NAME"
printf 'PER_CREATED_AT=%s\n' "$CREATED_AT"
printf 'PROVENANCE=%s\n' "$PROVENANCE"
printf 'FIRMWARE_DIR=%s\n' "$FIRMWARE_DIR"
