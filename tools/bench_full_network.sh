#!/usr/bin/env bash
# Measure the full PER network with JIT and ADC decimation 1.
# Timings include startup and JIT compilation. Optional trace comparison checks
# interpreter/JIT equivalence before collecting measurements.

set -euo pipefail

ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
NETWORK="$ROOT_DIR/configs/networks/per_vehicle.json"

usage() {
    cat <<'EOF'
Usage: bench_full_network.sh [OPTIONS]

Options:
  --binary PATH             fil executable (default: build-pgo/fil)
  --reference-binary PATH   matched baseline executable; runs alternate order
  --duration-ms N           simulated duration per measurement (default: 5000)
  --reps N                  repetitions per executable (default: 3)
  --check-trace             compare 1,000 ms interpreter/JIT traces first
  --no-loop-batching        disable loop batching
  --deferred-prefixes       enable experimental interruptible JIT prefixes
  --no-jit                  disable JIT
  --no-ram-capsules         disable private-RAM JIT capsules
  -h, --help                show this help
EOF
}

fail() {
    printf 'bench_full_network.sh: %s\n' "$*" >&2
    exit 2
}

positive_integer() {
    [[ $1 =~ ^[0-9]+$ ]] && ((10#$1 > 0))
}

binary="$ROOT_DIR/build-pgo/fil"
reference_binary=
duration_ms=5000
reps=3
check_trace=0
no_loop_batching=0
deferred_prefixes=0
no_jit=0
no_ram_capsules=0

while (($#)); do
    case $1 in
        --binary|--reference-binary|--duration-ms|--reps)
            (($# >= 2)) || fail "$1 requires a value"
            option=$1
            value=$2
            shift 2
            case $option in
                --binary) binary=$value ;;
                --reference-binary) reference_binary=$value ;;
                --duration-ms)
                    positive_integer "$value" || fail '--duration-ms must be a positive integer'
                    duration_ms=$((10#$value))
                    ;;
                --reps)
                    positive_integer "$value" || fail '--reps must be a positive integer'
                    reps=$((10#$value))
                    ;;
            esac
            ;;
        --binary=*) binary=${1#*=}; shift ;;
        --reference-binary=*) reference_binary=${1#*=}; shift ;;
        --duration-ms=*)
            value=${1#*=}
            positive_integer "$value" || fail '--duration-ms must be a positive integer'
            duration_ms=$((10#$value))
            shift
            ;;
        --reps=*)
            value=${1#*=}
            positive_integer "$value" || fail '--reps must be a positive integer'
            reps=$((10#$value))
            shift
            ;;
        --check-trace) check_trace=1; shift ;;
        --no-loop-batching) no_loop_batching=1; shift ;;
        --deferred-prefixes) deferred_prefixes=1; shift ;;
        --no-jit) no_jit=1; shift ;;
        --no-ram-capsules) no_ram_capsules=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) fail "unrecognized argument: $1" ;;
    esac
done

absolute_path() {
    case $1 in
        /*) printf '%s\n' "$1" ;;
        *) printf '%s/%s\n' "$PWD" "$1" ;;
    esac
}
binary=$(absolute_path "$binary")
[[ -x $binary ]] || fail "fil executable not found or not executable: $binary"
if [[ -n $reference_binary ]]; then
    reference_binary=$(absolute_path "$reference_binary")
    [[ -x $reference_binary ]] || fail "reference executable not found or not executable: $reference_binary"
fi

work_dir=$(mktemp -d "${TMPDIR:-/tmp}/fil-network-bench.XXXXXXXX")
trap 'rm -rf "$work_dir"' EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM
run_number=0

# On success, RUN_NORMALIZED, RUN_TIME_NS, RUN_INSTRUCTIONS, and RUN_WALL are set.
run_network() {
    local executable=$1 duration=$2 jit=$3 trace=$4 no_batch=$5 deferred=$6 capsules=$7
    local stdout_file stderr_file timing_file raw_file normalized_file validation_file
    local wall status
    local -a command
    run_number=$((run_number + 1))
    stdout_file="$work_dir/run-$run_number.stdout"
    stderr_file="$work_dir/run-$run_number.stderr"
    timing_file="$work_dir/run-$run_number.time"
    raw_file="$work_dir/run-$run_number.parsed"
    normalized_file="$work_dir/run-$run_number.normalized"
    validation_file="$work_dir/run-$run_number.validation"
    command=("$executable" run-network "$NETWORK"
        --duration-ms "$duration"
        --max-instructions 1000000000 --quantum 1024
        --strict-mmio --adc-decimation 1)
    ((jit)) || command+=(--no-jit)
    ((no_batch)) && command+=(--no-loop-batching)
    ((deferred)) && command+=(--deferred-prefixes)
    ((capsules)) || command+=(--no-ram-capsules)
    if [[ -n $trace ]]; then
        command+=(--trace "$trace")
    fi

    TIMEFORMAT='%6R'
    if (cd "$ROOT_DIR" && { time "${command[@]}" </dev/null > "$stdout_file" 2> "$stderr_file"; }) 2> "$timing_file"; then
        status=0
    else
        status=$?
    fi
    if ((status != 0)); then
        printf 'COMMAND FAILED (exit %d):' "$status" >&2
        printf ' %q' "${command[@]}" >&2
        printf '\nstdout:\n' >&2
        cat "$stdout_file" >&2
        printf '\nstderr:\n' >&2
        cat "$stderr_file" >&2
        return "$status"
    fi
    wall=$(awk '/^[0-9]+([.][0-9]+)?$/ { value = $0 } END { if (value != "") print value }' "$timing_file")
    [[ $wall =~ ^[0-9]+([.][0-9]+)?$ ]] || fail 'could not read benchmark wall time'
    if [[ $wall =~ ^0+([.]0+)?$ ]]; then
        fraction=${wall#*.}
        wall=0.
        for ((precision = 0; precision < ${#fraction}; precision++)); do wall="${wall}0"; done
        wall="${wall}5"
    fi

    awk '
        {
            separator = index($0, ": ")
            if (separator > 0) {
                key = substr($0, 1, separator - 1)
                value = substr($0, separator + 2)
                if (key == "stop" || key == "boards" || key == "time_ns" ||
                    key == "instructions" || key == "cycles" || key == "event_callbacks")
                    summary[key] = value
            }
            if ($0 ~ /^board .+: stop=[a-z-]+ instructions=[0-9]+ pc=0x[0-9a-fA-F]+$/) {
                line = $0
                sub(/^board /, "", line)
                name = line
                sub(/: stop=.*/, "", name)
                fields = $0
                sub(/^.*: stop=/, "", fields)
                count = split(fields, pieces, " ")
                reason = pieces[1]
                instruction_count = pieces[2]
                pc = pieces[3]
                sub(/^instructions=/, "", instruction_count)
                sub(/^pc=0x/, "", pc)
                printf "B|%s|%s|%s|%s\n", name, reason, instruction_count, pc
            }
        }
        END {
            for (key in summary)
                printf "S|%s|%s\n", key, summary[key]
        }
    ' "$stdout_file" > "$raw_file"

    if ! awk -F'|' -v duration="$duration" '
        $1 == "S" { summary[$2] = $3 }
        $1 == "B" {
            name = $2
            if (name in board_stop) {
                print "duplicate board result: " name
                invalid = 1
            } else {
                board_stop[name] = $3
                board_instructions[name] = $4 + 0
                pc = tolower($5)
                sub(/^0+/, "", pc)
                if (pc == "") pc = "0"
                board_pc[name] = pc
                board_count++
                instruction_total += $4 + 0
            }
        }
        END {
            if (summary["stop"] != "time-budget") {
                print "stop reason is not time-budget: " summary["stop"]
                invalid = 1
            }
            if (summary["boards"] != "6") {
                print "reported board count is not six: " summary["boards"]
                invalid = 1
            }
            if (summary["time_ns"] !~ /^[0-9]+$/ || summary["time_ns"] < duration * 1000000) {
                print "run stopped before the requested duration: " summary["time_ns"]
                invalid = 1
            }
            if (summary["instructions"] !~ /^[0-9]+$/) {
                print "missing or invalid network instruction count: " summary["instructions"]
                invalid = 1
            }
            if (board_count != 6) {
                print "expected six board results, got " board_count
                invalid = 1
            }
            for (name in board_stop) {
                if (board_stop[name] != "time-budget") {
                    print "board " name " did not reach the time budget: " board_stop[name]
                    invalid = 1
                }
            }
            if (summary["instructions"] ~ /^[0-9]+$/ && instruction_total != summary["instructions"] + 0) {
                print "board instruction counts do not match the network total"
                invalid = 1
            }
            if (invalid) exit 1
            for (key in summary)
                printf "S|%s|%s\n", key, summary[key]
            for (name in board_stop)
                printf "B|%s|%s|%.0f|%s\n", name, board_stop[name], board_instructions[name], board_pc[name]
        }
    ' "$raw_file" > "$normalized_file" 2> "$validation_file"; then
        printf 'Incomplete full-network run (duration %s ms). Parsed output:\n' "$duration" >&2
        cat "$raw_file" >&2
        cat "$validation_file" >&2
        printf 'stderr:\n' >&2
        cat "$stderr_file" >&2
        return 1
    fi
    LC_ALL=C sort -o "$normalized_file" "$normalized_file"
    RUN_NORMALIZED=$normalized_file
    RUN_TIME_NS=$(awk -F'|' '$1 == "S" && $2 == "time_ns" { print $3 }' "$normalized_file")
    RUN_INSTRUCTIONS=$(awk -F'|' '$1 == "S" && $2 == "instructions" { print $3 }' "$normalized_file")
    RUN_WALL=$wall
}

print_measurement() {
    local number=$1 label=$2
    local simulated wall_display
    simulated=$(awk -v ns="$RUN_TIME_NS" 'BEGIN { printf "%.9f", ns / 1000000000 }')
    wall_display=$(awk -v wall="$RUN_WALL" 'BEGIN { printf "%.3f", wall }')
    printf 'Run %s (%s): wall=%ss simulated=%ss instructions=%s\n' \
        "$number" "$label" "$wall_display" "$simulated" "$RUN_INSTRUCTIONS"
}

if ((check_trace)); then
    reference_trace="$work_dir/interpreter.jsonl"
    candidate_trace="$work_dir/jit.jsonl"
    run_network "$binary" 1000 0 "$reference_trace" 1 0 0
    interpreter_summary=$RUN_NORMALIZED
    run_network "$binary" 1000 "$((1 - no_jit))" "$candidate_trace" \
        "$no_loop_batching" "$deferred_prefixes" "$((1 - no_ram_capsules))"
    if ! cmp -s "$interpreter_summary" "$RUN_NORMALIZED"; then
        fail 'Full-network interpreter/JIT counters or final board PCs differ'
    fi
    if ! cmp -s "$reference_trace" "$candidate_trace"; then
        fail 'Full-network interpreter/JIT traces differ'
    fi
    printf '1,000 ms full-network traces, counters and final board PCs: identical\n'
fi

candidate_times=()
reference_times=()
for ((repetition = 0; repetition < reps; repetition++)); do
    order=(candidate)
    if [[ -n $reference_binary ]]; then
        order+=(reference)
        if ((repetition % 2 == 0)); then
            order=(reference candidate)
        fi
    fi
    summaries=()
    for label in "${order[@]}"; do
        if [[ $label == candidate ]]; then
            executable=$binary
            measurements_name=candidate_times
        else
            executable=$reference_binary
            measurements_name=reference_times
        fi
        run_network "$executable" "$duration_ms" "$((1 - no_jit))" '' \
            "$no_loop_batching" "$deferred_prefixes" "$((1 - no_ram_capsules))"
        print_measurement "$((repetition + 1))" "$label"
        if [[ $measurements_name == candidate_times ]]; then
            candidate_times+=("$RUN_WALL")
        else
            reference_times+=("$RUN_WALL")
        fi
        summaries+=("$RUN_NORMALIZED")
    done
    if ((${#summaries[@]} == 2)) && ! cmp -s "${summaries[0]}" "${summaries[1]}"; then
        fail 'Reference/candidate counters or final board PCs differ'
    fi
done

median() {
    printf '%s\n' "$@" | LC_ALL=C sort -n | awk '
        { values[NR] = $1 }
        END {
            if (NR % 2) printf "%.9f", values[(NR + 1) / 2]
            else printf "%.9f", (values[NR / 2] + values[NR / 2 + 1]) / 2
        }
    '
}
candidate_median=$(median "${candidate_times[@]}")
throughput=$(awk -v d="$duration_ms" -v m="$candidate_median" \
    'BEGIN { printf "%.3f", (d / 1000) / m }')
if [[ -n $reference_binary ]]; then
    reference_median=$(median "${reference_times[@]}")
    speedup=$(awk -v r="$reference_median" -v c="$candidate_median" \
        'BEGIN { printf "%.3f", r / c }')
    printf 'Reference median: %.3fs; candidate speedup: %sx\n' "$reference_median" "$speedup"
fi
printf 'ADC decimation: 1; all six boards reached the time budget\n'
printf 'Median: %.3fs; throughput: %sx realtime\n' "$candidate_median" "$throughput"
awk -v median="$candidate_median" -v duration="$duration_ms" \
    'BEGIN { exit (median < duration / 1000) ? 0 : 1 }'
