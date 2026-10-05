#!/usr/bin/env bash
# Check reference integrity, not whether each assertion proves its cited claim.
set -euo pipefail
export LC_ALL=C
root="$(cd "$(dirname "$0")/.." && pwd)"
document="$root/docs/stm32g4_peripheral_coverage.md"
manual_dir=""
while (($#)); do
    case "$1" in
        --document|--manual-dir)
            if (($# < 2)); then printf 'Missing value for %s\n' "$1" >&2; exit 2; fi
            if [[ "$1" == --document ]]; then document="$2"; else manual_dir="$2"; fi
            shift 2 ;;
        *) printf 'Usage: %s [--document FILE] [--manual-dir DIRECTORY]\n' "$0" >&2; exit 2 ;;
    esac
done
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

# Flatten each C++ file to handle multiline TEST and TEST_F declarations.
find "$root/tests" -name '*.cpp' -type f -exec awk '
    { text = text " " $0 }
    END {
        while (match(text, /TEST(_F)?[[:space:]]*\([[:space:]]*[[:alnum:]_]+[[:space:]]*,[[:space:]]*[[:alnum:]_]+[[:space:]]*\)/)) {
            declaration = substr(text, RSTART, RLENGTH)
            text = substr(text, RSTART + RLENGTH)
            sub(/^TEST(_F)?[[:space:]]*\(/, "", declaration)
            gsub(/[[:space:]\)]/, "", declaration)
            sub(/,/, ".", declaration)
            print declaration
        }
    }
' {} \; | sort -u > "$work/tests"
awk '
    {
        text = $0
        while (match(text, /`[[:alnum:]_]*Test\.[[:alnum:]_]+`/)) {
            print substr(text, RSTART + 1, RLENGTH - 2)
            text = substr(text, RSTART + RLENGTH)
        }
    }
' "$document" | sort -u > "$work/references"
awk '
    {
        text = $0
        while (match(text, /RM0440 §[[:space:]]*[0-9]+(\.[0-9]+)*/)) {
            section = substr(text, RSTART, RLENGTH)
            text = substr(text, RSTART + RLENGTH)
            sub(/^RM0440 §[[:space:]]*/, "", section)
            print section
        }
    }
' "$document" | sort -u > "$work/sections"
if [[ ! -s "$work/references" || ! -s "$work/sections" ]]; then
    printf 'Coverage document must cite named tests and RM0440 sections.\n' >&2
    exit 1
fi
comm -23 "$work/references" "$work/tests" > "$work/missing-tests"
failed=0
while IFS= read -r name; do printf 'Unknown test: %s\n' "$name" >&2; failed=1; done < "$work/missing-tests"
if [[ -n "$manual_dir" ]]; then
    if [[ ! -d "$manual_dir" ]]; then printf 'Manual directory does not exist: %s\n' "$manual_dir" >&2; exit 2; fi
    find "$manual_dir" -name 'chapter-*.md' -type f -exec awk '
        /^#+ [0-9]/ { section = $2; sub(/[^0-9.].*$/, "", section); sub(/\.$/, "", section); print section }
    ' {} \; | sort -u > "$work/headings"
    comm -23 "$work/sections" "$work/headings" > "$work/missing-sections"
    while IFS= read -r section; do printf 'Unknown RM0440 section: %s\n' "$section" >&2; failed=1; done < "$work/missing-sections"
fi
if ((failed)); then exit 1; fi
printf 'Verified %s named tests and %s RM0440 section references.\n' \
    "$(wc -l < "$work/references" | tr -d ' ')" "$(wc -l < "$work/sections" | tr -d ' ')"
if [[ -z "$manual_dir" ]]; then printf 'Manual headings were not checked; supply --manual-dir to check them.\n'; fi
