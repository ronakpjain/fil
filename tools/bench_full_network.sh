#!/usr/bin/env bash
# Measure the full PER network using the automatic execution path and ADC decimation 1.
# Timings include startup and JIT compilation.

set -euo pipefail

ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
NETWORK="$ROOT_DIR/configs/networks/per_vehicle.json"

usage() {
    cat <<'EOF'
Usage: bench_full_network.sh [OPTIONS]

Options:
  --binary PATH             fil executable (default: build-pgo/fil)
  --reference-binary PATH   matched baseline executable; runs alternate order
  --network PATH            network config (default: configs/networks/per_vehicle.json)
  --duration-ms N           simulated duration per measurement (default: 5000)
  --reps N                  repetitions per executable (default: 3)
  --cross-board-window-ns N  cross-board skew window (default: 100000; 0 = exact)
  --min-throughput X        required realtime ratio (default: above 1.0)
  --min-speedup X           required speedup over --reference-binary
  --mmio MODE               strict (default) or lenient unknown-MMIO handling
  --lenient                 warn instead of failing on era validation mismatches
                          (old firmware/fil: board count, clocks, stop reasons)
  --expect-boards N         expected board count (default: 6)
  --expect-hz HZ            require every board clock to equal HZ (default: unset,
                          only require all boards to agree)
  --json-output FILE        write machine-readable results (walls, counters, median) as JSON
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

validate_duration() {
    local value=$1
    [[ $value =~ ^[0-9]+$ ]] || fail '--duration-ms must be a positive integer'
    # Bound the scaled budget before evaluating potentially overflowing arithmetic.
    value=${value#"${value%%[!0]*}"}
    [[ -n $value ]] || fail '--duration-ms must be a positive integer'
    if ((${#value} > 13)) || { ((${#value} == 13)) && [[ $value > 9223372036854 ]]; }; then
        fail '--duration-ms exceeds the supported instruction-budget range'
    fi
}

binary="$ROOT_DIR/build-pgo/fil"
reference_binary=
network="$ROOT_DIR/configs/networks/per_vehicle.json"
json_output=
duration_ms=5000
reps=3
cross_board_window_ns=100000
min_throughput=
min_speedup=
speedup_requested=0
threshold_requested=0
mmio_mode=strict
lenient=0
expect_boards=6
expect_hz=

while (($#)); do
    case $1 in
        --binary|--reference-binary|--network|--json-output|--duration-ms|--reps|--min-throughput|--min-speedup|--cross-board-window-ns|--mmio|--expect-boards|--expect-hz)
            (($# >= 2)) || fail "$1 requires a value"
            option=$1
            value=$2
            shift 2
            case $option in
                --binary) binary=$value ;;
                --reference-binary) reference_binary=$value ;;
                --network) network=$value ;;
                --json-output) json_output=$value ;;
                --duration-ms)
                    validate_duration "$value"
                    duration_ms=$((10#$value))
                    ;;
                --min-throughput) min_throughput=$value; threshold_requested=1 ;;
                --min-speedup) min_speedup=$value; speedup_requested=1 ;;
                --mmio)
                    [[ $value == strict || $value == lenient ]] || fail '--mmio must be strict or lenient'
                    mmio_mode=$value ;;
                --expect-boards)
                    positive_integer "$value" || fail '--expect-boards must be a positive integer'
                    expect_boards=$((10#$value)) ;;
                --expect-hz)
                    positive_integer "$value" || fail '--expect-hz must be a positive integer'
                    expect_hz=$((10#$value)) ;;
                --cross-board-window-ns)
                    [[ $value =~ ^[0-9]+$ ]] || fail '--cross-board-window-ns must be a nonnegative integer'
                    cross_board_window_ns=$value ;;
                --reps)
                    positive_integer "$value" || fail '--reps must be a positive integer'
                    reps=$((10#$value))
                    ;;
            esac
            ;;
        --binary=*) binary=${1#*=}; shift ;;
        --reference-binary=*) reference_binary=${1#*=}; shift ;;
        --network=*) network=${1#*=}; shift ;;
        --json-output=*) json_output=${1#*=}; shift ;;
        --duration-ms=*)
            value=${1#*=}
            validate_duration "$value"
            duration_ms=$((10#$value))
            shift
            ;;
        --reps=*)
            value=${1#*=}
            positive_integer "$value" || fail '--reps must be a positive integer'
            reps=$((10#$value))
            shift
            ;;
        --min-throughput=*) min_throughput=${1#*=}; threshold_requested=1; shift ;;
        --min-speedup=*) min_speedup=${1#*=}; speedup_requested=1; shift ;;
        --mmio=*)
            value=${1#*=}
            [[ $value == strict || $value == lenient ]] || fail '--mmio must be strict or lenient'
            mmio_mode=$value; shift ;;
        --expect-boards=*)
            value=${1#*=}
            positive_integer "$value" || fail '--expect-boards must be a positive integer'
            expect_boards=$((10#$value)); shift ;;
        --expect-hz=*)
            value=${1#*=}
            positive_integer "$value" || fail '--expect-hz must be a positive integer'
            expect_hz=$((10#$value)); shift ;;
        --lenient) lenient=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) fail "unrecognized argument: $1" ;;
    esac
done

if ((threshold_requested)); then
    [[ $min_throughput =~ ^[0-9]+([.][0-9]+)?$ ]] || fail '--min-throughput must be a positive number'
    awk -v value="$min_throughput" 'BEGIN { exit value > 0 ? 0 : 1 }' \
        || fail '--min-throughput must be a positive number'
fi
if ((speedup_requested)); then
    [[ -n $reference_binary ]] || fail '--min-speedup requires --reference-binary'
    [[ $min_speedup =~ ^[0-9]+([.][0-9]+)?$ ]] || fail '--min-speedup must be a positive number'
    awk -v value="$min_speedup" 'BEGIN { exit value > 0 ? 0 : 1 }' \
        || fail '--min-speedup must be a positive number'
fi

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

# Resolve per-binary CLI compatibility. fil's run flags changed across history
# (run-network vs run, --duration-ms vs --duration, --max-instructions vs
# --instruction-budget, late additions like --adc-decimation and
# --cross-board-window-ns), so probe the binary and use only what it accepts.
# Probe failures for non-option reasons keep the flag (the real run surfaces
# the underlying error). Results are stored in ${PREFIX}RUN_CMD,
# ${PREFIX}DUR_NAME, ${PREFIX}DUR_SUFFIXED (1 when the value needs an ms
# suffix), ${PREFIX}BUDGET_NAME, ${PREFIX}HAVE_QUANTUM/HAVE_STRICT/HAVE_ADC/
# HAVE_WINDOW (0/1), and ${PREFIX}INCLUDED/${PREFIX}OMITTED (| separated).
probe_out=$work_dir/probe.out
probe_err=$work_dir/probe.err
probe_binary() {
    local exe=$1 prefix=$2
    local run_cmd=run dur_name=--duration dur_suffixed=1 budget_name=--instruction-budget
    local have_quantum=1 have_strict=1 have_adc=1 have_window=1
    local included= omitted=
    local dur_probe=1ms
    __try() {
        "$exe" "$run_cmd" "$network" "$dur_name" "$dur_probe" \
            "$budget_name" 10000 "$@" </dev/null >"$probe_out" 2>"$probe_err"
    }
    __unknown_option() {
        grep -qi 'unknown.*option' "$probe_err" && grep -qF -- "$1" "$probe_err"
    }
    # Command name: old binaries spell it run-network; unknown *command*
    # (not option) errors mean the name itself is wrong.
    run_cmd=run-network
    if ! __try; then
        if grep -qi 'unknown.*command' "$probe_err"; then
            run_cmd=run
            __try || true
        fi
    fi
    # Duration style.
    if ! __try; then
        if __unknown_option --duration; then
            dur_name=--duration-ms; dur_suffixed=0; dur_probe=1
        fi
    fi
    # Budget style.
    if ! __try; then
        if __unknown_option --instruction-budget; then
            budget_name=--max-instructions
        fi
    fi
    # Optional flags, probed one at a time.
    if ! __try --quantum 1024; then
        if __unknown_option --quantum; then have_quantum=0; fi
    fi
    if [[ $mmio_mode == strict ]]; then
        if ! __try --strict-mmio; then
            if __unknown_option --strict-mmio; then have_strict=0; fi
        fi
    else
        have_strict=0
    fi
    if ! __try --adc-decimation 1; then
        if __unknown_option --adc-decimation; then have_adc=0; fi
    fi
    if ! __try --cross-board-window-ns "$cross_board_window_ns"; then
        if __unknown_option --cross-board-window-ns; then have_window=0; fi
    fi
    if ((dur_suffixed)); then included="${included}|${dur_name} <duration>ms"; else included="${included}|${dur_name} <duration>"; fi
    included="${included}|${budget_name} <budget>"
    ((have_quantum)) && included="${included}|--quantum 1024" || omitted="${omitted}|--quantum 1024"
    if [[ $mmio_mode == strict ]]; then
        ((have_strict)) && included="${included}|--strict-mmio" || omitted="${omitted}|--strict-mmio"
    else
        omitted="${omitted}|--strict-mmio (mmio=lenient)"
    fi
    ((have_adc)) && included="${included}|--adc-decimation 1" || omitted="${omitted}|--adc-decimation 1"
    if ((have_window)); then
        included="${included}|--cross-board-window-ns $cross_board_window_ns"
    else
        omitted="${omitted}|--cross-board-window-ns $cross_board_window_ns"
    fi
    printf -v "${prefix}RUN_CMD" '%s' "$run_cmd"
    printf -v "${prefix}DUR_NAME" '%s' "$dur_name"
    printf -v "${prefix}DUR_SUFFIXED" '%s' "$dur_suffixed"
    printf -v "${prefix}BUDGET_NAME" '%s' "$budget_name"
    printf -v "${prefix}HAVE_QUANTUM" '%s' "$have_quantum"
    printf -v "${prefix}HAVE_STRICT" '%s' "$have_strict"
    printf -v "${prefix}HAVE_ADC" '%s' "$have_adc"
    printf -v "${prefix}HAVE_WINDOW" '%s' "$have_window"
    printf -v "${prefix}INCLUDED" '%s' "${included#|}"
    printf -v "${prefix}OMITTED" '%s' "${omitted#|}"
    printf 'Probed %s: %s %s %s\n' "$exe" "$run_cmd" "$dur_name" "$budget_name" >&2
}
probe_binary "$binary" CAND_
if [[ -n $reference_binary && $reference_binary != "$binary" ]]; then
    probe_binary "$reference_binary" REF_
else
    REF_RUN_CMD=$CAND_RUN_CMD
    REF_DUR_NAME=$CAND_DUR_NAME
    REF_DUR_SUFFIXED=$CAND_DUR_SUFFIXED
    REF_BUDGET_NAME=$CAND_BUDGET_NAME
    REF_HAVE_QUANTUM=$CAND_HAVE_QUANTUM
    REF_HAVE_STRICT=$CAND_HAVE_STRICT
    REF_HAVE_ADC=$CAND_HAVE_ADC
    REF_HAVE_WINDOW=$CAND_HAVE_WINDOW
    REF_INCLUDED=$CAND_INCLUDED
    REF_OMITTED=$CAND_OMITTED
fi

# On success, RUN_NORMALIZED, RUN_TIME_NS, RUN_INSTRUCTIONS, and RUN_WALL are set.
run_network() {
    local executable=$1 duration=$2
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
    local prefix=CAND_
    if [[ -n $reference_binary && $executable == "$reference_binary" ]]; then prefix=REF_; fi
    local _run_cmd="${prefix}RUN_CMD" _dur_name="${prefix}DUR_NAME"
    local _dur_suffixed="${prefix}DUR_SUFFIXED" _budget_name="${prefix}BUDGET_NAME"
    local _have_q="${prefix}HAVE_QUANTUM" _have_s="${prefix}HAVE_STRICT"
    local _have_a="${prefix}HAVE_ADC" _have_w="${prefix}HAVE_WINDOW"
    local dur_value="${duration}ms" budget_value="$((duration * 1000000))"
    if [[ ${!_dur_suffixed} == 0 ]]; then dur_value="$duration"; fi
    command=("$executable" "${!_run_cmd}" "$network"
        "${!_dur_name}" "$dur_value" "${!_budget_name}" "$budget_value")
    [[ ${!_have_q} == 1 ]] && command+=(--quantum 1024)
    [[ ${!_have_s} == 1 ]] && command+=(--strict-mmio)
    [[ ${!_have_a} == 1 ]] && command+=(--adc-decimation 1)
    [[ ${!_have_w} == 1 ]] && command+=(--cross-board-window-ns "$cross_board_window_ns")

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
            separator = index($0, ":")
            if (separator > 0) {
                leading = match($0, /[^ ]/) - 1
                key = substr($0, 1, separator - 1)
                value = substr($0, separator + 1)
                sub(/^[[:space:]]+/, "", key)
                sub(/[[:space:]]+$/, "", key)
                sub(/^[[:space:]]+/, "", value)
                sub(/[[:space:]]+$/, "", value)
                # Original reports used unindented summary fields and
                # "board NAME: stop=... instructions=... pc=..." rows.
                if (leading == 0 && key ~ /^board /) {
                    name = substr(key, 7)
                    count = split(value, fields, /[[:space:]]+/)
                    for (i = 1; i <= count; i++) {
                        split(fields[i], pair, "=")
                        if (pair[1] == "stop") board_stop[name] = pair[2]
                        if (pair[1] == "instructions") board_instructions[name] = pair[2]
                        if (pair[1] == "pc") board_pc[name] = pair[2]
                    }
                    next
                }
                if (leading == 0 && key == "clock") {
                    count = split(value, fields, /[[:space:]]+/)
                    name = ""; hz = ""
                    for (i = 1; i <= count; i++) {
                        split(fields[i], pair, "=")
                        if (pair[1] == "board") name = pair[2]
                        if (pair[1] == "hz") hz = pair[2]
                    }
                    if (name != "" && hz != "") board_clock[name] = hz
                    next
                }
                # Intermediate reports put board fields on indented lines.
                if (leading == 4 && active_board != "") {
                    if (key == "stop") board_stop[active_board] = value
                    if (key == "instructions") board_instructions[active_board] = value
                    if (key == "pc") board_pc[active_board] = value
                    if (key == "clock") {
                        sub(/ Hz$/, "", value)
                        board_clock[active_board] = value
                    }
                    next
                }
                if (leading != 0 && leading != 2) next
                if (key == "board") {
                    active_board = value
                    rest = value
                    name = rest
                    sub(/[ ].*$/, "", name)
                    active_board = name
                    if (match(rest, /stop: [a-z-]+/)) {
                        board_stop[name] = substr(rest, RSTART + 6, RLENGTH - 6)
                    }
                    if (match(rest, /instructions: [0-9]+/)) {
                        board_instructions[name] = substr(rest, RSTART + 14, RLENGTH - 14)
                    }
                    if (match(rest, /pc: 0x[0-9a-f]+/)) {
                        board_pc[name] = substr(rest, RSTART + 4, RLENGTH - 4)
                    }
                    if (match(rest, /clock: [0-9]+ Hz/)) {
                        board_clock[name] = substr(rest, RSTART + 7, RLENGTH - 10)
                    }
                } else if (key == "time" && match(value, /\([0-9]+ ns\)/)) {
                    time_ns = substr(value, RSTART, RLENGTH)
                    gsub(/[^0-9]/, "", time_ns)
                    summary["time_ns"] = time_ns
                } else {
                    summary[key] = value
                }
            }
        }
        END {
            for (key in summary)
                printf "S|%s|%s\n", key, summary[key]
            for (name in board_stop)
                printf "B|%s|%s|%s|%s\n", name, board_stop[name], board_instructions[name], board_pc[name]
            for (name in board_clock)
                printf "C|%s|%s\n", name, board_clock[name]
        }
    ' "$stdout_file" > "$raw_file"

    if ! awk -F'|' -v duration="$duration" -v strict="$((1 - lenient))" \
        -v expect_boards="$expect_boards" -v expect_hz="$expect_hz" \
        -v warnfile="$validation_file" '
        function fail(msg) { print msg > warnfile; fatal = 1 }
        function warn(msg) { print msg > warnfile; if (strict) invalid = 1 }
        $1 == "S" { summary[$2] = $3 }
        $1 == "C" { clocks[$2] = $3; clock_count++ }
        $1 == "B" {
            name = $2
            if (name in board_stop) {
                fail("duplicate board result: " name)
            } else {
                board_stop[name] = $3
                board_instructions[name] = $4 + 0
                pc = tolower($5)
                sub(/^0x/, "", pc)
                sub(/^0+/, "", pc)
                if (pc == "") pc = "0"
                board_pc[name] = pc
                board_count++
                instruction_total += $4 + 0
            }
        }
        END {
            if (summary["stop"] != "time-budget") {
                warn("stop reason is not time-budget: " summary["stop"])
            }
            if (summary["boards"] + 0 != expect_boards + 0) {
                warn("reported board count is not " expect_boards ": " summary["boards"])
            }
            if (summary["time_ns"] !~ /^[0-9]+$/ || summary["time_ns"] + 0 < duration * 1000000) {
                fail("run stopped before the requested duration: " summary["time_ns"])
            }
            if (summary["instructions"] !~ /^[0-9]+$/) {
                fail("missing or invalid network instruction count: " summary["instructions"])
            }
            if (board_count == 0) {
                fail("no board results parsed")
            } else if (board_count != expect_boards + 0) {
                warn("expected " expect_boards " board results, got " board_count)
            }
            if (clock_count != board_count) {
                warn("expected " board_count " board clock reports, got " clock_count)
            }
            for (name in board_stop) {
                if ((name in clocks)) {
                    if (reference_hz == "") reference_hz = clocks[name]
                    if (clocks[name] != reference_hz) {
                        warn("board " name " clock " clocks[name] " Hz differs from " reference_hz " Hz")
                    }
                    if (expect_hz != "" && clocks[name] != expect_hz) {
                        warn("board " name " is not running at " expect_hz " Hz: " clocks[name])
                    }
                }
                if (board_stop[name] != "time-budget") {
                    warn("board " name " did not reach the time budget: " board_stop[name])
                }
            }
            if (summary["instructions"] ~ /^[0-9]+$/ && instruction_total != summary["instructions"] + 0) {
                warn("board instruction counts do not match the network total")
            }
            if (fatal || invalid) exit 1
            for (key in summary)
                printf "S|%s|%s\n", key, summary[key]
            for (name in clocks)
                printf "C|%s|%s\n", name, clocks[name]
            for (name in board_stop)
                printf "B|%s|%s|%.0f|%s\n", name, board_stop[name], board_instructions[name], board_pc[name]
        }
    ' "$raw_file" > "$normalized_file" 2> "$validation_file"; then
        if ((lenient)); then
            printf 'Full-network run failed fatal validation (duration %s ms, lenient mode):\n' "$duration" >&2
        else
            printf 'Incomplete full-network run (duration %s ms). Parsed output:\n' "$duration" >&2
        fi
        cat "$raw_file" >&2
        cat "$validation_file" >&2
        if ((lenient)); then
            printf 'stdout (tail):\n' >&2
            tail -n 30 "$stdout_file" >&2
            printf 'stderr (tail):\n' >&2
            tail -n 20 "$stderr_file" >&2
        else
            printf 'stderr:\n' >&2
            cat "$stderr_file" >&2
        fi
        return 1
    fi
    if [[ -s $validation_file ]]; then
        cat "$validation_file" >> "$work_dir/all-warnings.txt"
        if ((lenient)); then
            printf 'Run %s warnings:\n' "$run_number" >&2
            cat "$validation_file" >&2
        fi
    fi
    LC_ALL=C sort -o "$normalized_file" "$normalized_file"
    RUN_NORMALIZED=$normalized_file
    RUN_TIME_NS=$(awk -F'|' '$1 == "S" && $2 == "time_ns" { print $3 }' "$normalized_file")
    RUN_INSTRUCTIONS=$(awk -F'|' '$1 == "S" && $2 == "instructions" { print $3 }' "$normalized_file")
    RUN_CYCLES=$(awk -F'|' '$1 == "S" && $2 == "cycles" { print $3 }' "$normalized_file")
    RUN_WALL=$wall
}

print_measurement() {
    local number=$1 label=$2
    local simulated wall_display
    simulated=$(awk -v ns="$RUN_TIME_NS" 'BEGIN { printf "%.9f", ns / 1000000000 }')
    wall_display=$(awk -v wall="$RUN_WALL" 'BEGIN { printf "%.3f", wall }')
    printf 'Run %s (%s): wall=%ss simulated=%ss instructions=%s cycles=%s\n' \
        "$number" "$label" "$wall_display" "$simulated" "$RUN_INSTRUCTIONS" "$RUN_CYCLES"
}

candidate_times=()
candidate_instructions=()
candidate_cycles=()
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
        run_network "$executable" "$duration_ms"
        print_measurement "$((repetition + 1))" "$label"
        if [[ $measurements_name == candidate_times ]]; then
            candidate_times+=("$RUN_WALL")
            candidate_instructions+=("$RUN_INSTRUCTIONS")
            candidate_cycles+=("$RUN_CYCLES")
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
    # Compare unrounded measurements; display rounding must not pass the gate.
    if [[ -n $min_speedup ]]; then
        awk -v r="$reference_median" -v c="$candidate_median" -v minimum="$min_speedup" \
            'BEGIN { exit r / c >= minimum ? 0 : 1 }' \
            || fail "Candidate did not reach the required ${min_speedup}x speedup"
    fi
fi
printf 'ADC decimation: 1; all six boards reached the time budget\n'
printf 'Median: %.3fs; throughput: %sx realtime\n' "$candidate_median" "$throughput"
if [[ -n $json_output ]]; then
    json_escape() { printf '%s' "$1" | sed -e 's/\\/\\\\/g' -e 's/"/\\"/g'; }
    join_json() { local sep=; for v in "$@"; do printf '%s"%s"' "$sep" "$v"; sep=,; done; }
    pipe_json() {
        local IFS='|'
        local sep= part
        for part in $1; do
            [[ -n $part ]] || continue
            printf '%s"%s"' "$sep" "$(json_escape "$part")"
            sep=', '
        done
    }
    warnings_json=
    if [[ -f $work_dir/all-warnings.txt ]]; then
        warnings_json=$(awk '{ gsub(/\\/, "\\\\"); gsub(/"/, "\\\""); printf "%s\"%s\"", (NR>1?", ":""), $0 }' "$work_dir/all-warnings.txt")
    fi
    clocks_json=$(awk -F'|' '$1 == "C" { printf "%s\"%s\": \"%s\"", (n++ ? ", " : ""), $2, $3 }' "$RUN_NORMALIZED")
    {
        printf '{\n  "network": "%s",\n' "$(json_escape "$network")"
        printf '  "duration_ms": %s,\n  "reps": %s,\n  "cross_board_window_ns": "%s",\n' \
            "$duration_ms" "$reps" "$cross_board_window_ns"
        printf '  "run_cmd": "%s",\n' "$CAND_RUN_CMD"
        printf '  "duration_flag": "%s",\n  "budget_flag": "%s",\n' "$CAND_DUR_NAME" "$CAND_BUDGET_NAME"
        printf '  "expect_boards": %s,\n' "$expect_boards"
        if ((lenient)); then printf '  "validation": "lenient",\n'; else printf '  "validation": "strict",\n'; fi
        printf '  "mmio": "%s",\n' "$mmio_mode"
        printf '  "included_flags": [%s],\n' "$(pipe_json "$CAND_INCLUDED")"
        printf '  "omitted_unsupported": [%s],\n' "$(pipe_json "$CAND_OMITTED")"
        printf '  "warnings": [%s],\n' "$warnings_json"
        printf '  "clocks_hz": {%s},\n' "$clocks_json"
        printf '  "walls": [%s],\n' "$(join_json ${candidate_times[@]})"
        printf '  "instructions": [%s],\n' "$(join_json ${candidate_instructions[@]})"
        printf '  "cycles": [%s],\n' "$(join_json ${candidate_cycles[@]})"
        printf '  "median_wall_s": "%s",\n  "throughput_x": "%s"' "$candidate_median" "$throughput"
        if [[ -n $reference_binary ]]; then
            printf ',\n  "reference_median_wall_s": "%s",\n  "speedup": "%s"' "$reference_median" "$speedup"
        fi
        printf '\n}\n'
    } > "$json_output"
    printf 'Wrote JSON results to %s\n' "$json_output"
fi
# Test the unrounded ratio: the displayed three decimals are not acceptance evidence.
awk -v median="$candidate_median" -v duration="$duration_ms" -v minimum="$min_throughput" \
    'BEGIN {
        ratio = (duration / 1000) / median
        exit (minimum == "" ? ratio > 1 : ratio >= minimum) ? 0 : 1
    }'
