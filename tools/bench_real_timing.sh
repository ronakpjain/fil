#!/usr/bin/env bash
# Real-timing benchmark for fil (no PER checkout required).
#
# Generates hand-assembled Thumb ELF fixtures, runs fixed-simulation-time
# workloads, and reports instructions/cycles/CPI/wall time/throughput.
# Workloads run on one board and a synthetic six-board network, with loop
# batching enabled and disabled.

set -euo pipefail

FLASH_BASE=$((0x08000000))
SRAM_TOP=$((0x20020000))
NOP=$((0xBF00))

usage() {
    cat <<'EOF'
Usage: bench_real_timing.sh [FIL_BINARY] [--reps N]

Run the real-timing benchmark with the given fil binary (default:
./build-release/fil). Each case runs for one simulated second.
EOF
}

fail() {
    printf 'bench_real_timing.sh: %s\n' "$*" >&2
    exit 2
}

pack_le() {
    local value=$1 bytes=$2 i byte octal
    for ((i = 0; i < bytes; i++)); do
        byte=$(((value >> (i * 8)) & 255))
        printf -v octal '%03o' "$byte"
        printf '%b' "\\${octal}"
    done
}

ldr_literal() {
    local rt=$1 target_addr=$2 instr_addr=$3 pc imm8
    pc=$(((instr_addr + 4) & ~3))
    (( (target_addr - pc) % 4 == 0 && target_addr >= pc )) || return 1
    imm8=$(((target_addr - pc) / 4))
    ((imm8 < 256)) || return 1
    ENCODED=$((0x4800 | (rt << 8) | imm8))
}

str_imm() {
    local rt=$1 rn=$2
    ENCODED=$((0x6000 | (rn << 3) | rt))
}

movs() {
    local rd=$1 imm8=$2
    ENCODED=$((0x2000 | (rd << 8) | imm8))
}

b_to() {
    local target=$1 instr_addr=$2 off
    off=$((target - (instr_addr + 4)))
    ((off % 2 == 0 && off >= -2048 && off <= 2046)) || return 1
    ENCODED=$((0xE000 | ((off >> 1) & 0x7FF)))
}

build_idle_code() {
    CODE=()
    CODE+=("$NOP" "$NOP" "$NOP" "$NOP")
    b_to "$((FLASH_BASE + 8))" "$((FLASH_BASE + 8 + ${#CODE[@]} * 2))" \
        || fail 'could not encode idle loop branch'
    CODE+=("$ENCODED")
}

build_pll_code() {
    local setup_count=10 loop_count=6 code_base lit_base
    local l_acr l_acrv l_pll l_pllv l_cfgr a loop_start value
    CODE=()
    code_base=$((FLASH_BASE + 8))
    lit_base=$((code_base + (setup_count + loop_count) * 2))
    l_acr=$((lit_base + 4 * 0))
    l_acrv=$((lit_base + 4 * 1))
    l_pll=$((lit_base + 4 * 2))
    l_pllv=$((lit_base + 4 * 3))
    l_cfgr=$((lit_base + 4 * 4))
    a=$code_base

    ldr_literal 0 "$l_acr" "$a" || fail 'could not encode FLASH_ACR literal load'
    CODE+=("$ENCODED"); a=$((a + 2))
    ldr_literal 1 "$l_acrv" "$a" || fail 'could not encode FLASH_ACR value load'
    CODE+=("$ENCODED"); a=$((a + 2))
    str_imm 1 0; CODE+=("$ENCODED"); a=$((a + 2))
    ldr_literal 0 "$l_pll" "$a" || fail 'could not encode PLL literal load'
    CODE+=("$ENCODED"); a=$((a + 2))
    ldr_literal 1 "$l_pllv" "$a" || fail 'could not encode PLL value load'
    CODE+=("$ENCODED"); a=$((a + 2))
    str_imm 1 0; CODE+=("$ENCODED"); a=$((a + 2))
    ldr_literal 0 "$l_cfgr" "$a" || fail 'could not encode clock config literal load'
    CODE+=("$ENCODED"); a=$((a + 2))
    movs 1 3; CODE+=("$ENCODED"); a=$((a + 2))
    str_imm 1 0; CODE+=("$ENCODED"); a=$((a + 2))
    CODE+=("$NOP"); a=$((a + 2))
    ((a == code_base + setup_count * 2)) || fail 'PLL setup instruction count drifted'

    loop_start=$a
    for ((i = 0; i < 4; i++)); do
        CODE+=("$NOP"); a=$((a + 2))
    done
    b_to "$loop_start" "$a" || fail 'could not encode PLL loop branch'
    CODE+=("$ENCODED"); a=$((a + 2))
    CODE+=("$NOP"); a=$((a + 2))
    ((a == lit_base)) || fail 'PLL literal pool alignment drifted'

    for value in $((0x40022000)) $((0x304)) $((0x4002100C)) $((0x5533)) $((0x40021008)); do
        CODE+=("$((value & 0xFFFF))" "$(((value >> 16) & 0xFFFF))")
    done
}

build_image() {
    local output=$1 code_size blob_size word halfword
    shift
    CODE=("$@")
    code_size=$((${#CODE[@]} * 2))
    blob_size=$((8 + code_size))

    {
        # ELF32 little-endian ARM executable header.
        printf '\177ELF\001\001\001\000\000\000\000\000\000\000\000\000'
        pack_le 2 2                 # e_type: ET_EXEC
        pack_le 40 2                # e_machine: ARM
        pack_le 1 4                 # e_version
        pack_le 0 4                 # e_entry
        pack_le 52 4                # e_phoff
        pack_le 0 4                 # e_shoff
        pack_le 0 4                 # e_flags
        pack_le 52 2                # e_ehsize
        pack_le 32 2                # e_phentsize
        pack_le 1 2                 # e_phnum
        pack_le 0 2                 # e_shentsize
        pack_le 0 2                 # e_shnum
        pack_le 0 2                 # e_shstrndx
        pack_le 1 4                 # p_type: PT_LOAD
        pack_le 84 4                # p_offset
        pack_le "$FLASH_BASE" 4    # p_vaddr
        pack_le "$FLASH_BASE" 4    # p_paddr
        pack_le "$blob_size" 4     # p_filesz
        pack_le "$blob_size" 4     # p_memsz
        pack_le 5 4                 # p_flags: PF_R | PF_X
        pack_le 4 4                 # p_align
        pack_le "$SRAM_TOP" 4
        pack_le "$((FLASH_BASE + 9))" 4
        for halfword in "${CODE[@]}"; do
            pack_le "$halfword" 2
        done
    } > "$output"
}

write_board_config() {
    local root=$1 name=$2 elf=$3
    printf '{"schema_version":1,"name":"%s","mcu":"mcu.json","elf":"%s.elf","vector_base":"0x08000000"}\n' \
        "$name" "$name" > "$root/$name.json"
    cp "$elf" "$root/$name.elf"
}

write_configs() {
    local root=$1 idle_elf=$2 pll_elf=$3 i
    printf '{"schema_version":1,"name":"stm32g474retx","flash_base":"0x08000000","flash_size":"512K","sram_base":"0x20000000","sram_size":"128K","ccm_sram_base":"0x10000000","ccm_sram_size":"32K","hse_hz":16000000}\n' \
        > "$root/mcu.json"
    write_board_config "$root" idle "$idle_elf"
    write_board_config "$root" pll "$pll_elf"

    for ((i = 0; i < 6; i++)); do
        write_board_config "$root" "node$i" "$idle_elf"
    done
    {
        printf '{"schema_version":1,"name":"bench-six","buses":{},"boards":['
        for ((i = 0; i < 6; i++)); do
            ((i == 0)) || printf ','
            printf '"node%d.json"' "$i"
        done
        printf ']}\n'
    } > "$root/six.json"
}

parse_output() {
    local text_file=$1 output_file=$2
    awk '
        {
            colon = index($0, ":")
            if (colon > 0) {
                key = substr($0, 1, colon - 1)
                value = substr($0, colon + 1)
                sub(/^[[:space:]]+/, "", key)
                sub(/^[[:space:]]+/, "", value)
                if (key == "instructions" || key == "cycles" || key == "time_ns")
                    values[key] = value
            }
        }
        END {
            for (key in values)
                print key "=" values[key]
        }
    ' "$text_file" > "$output_file"
}

run_case() {
    local binary=$1 case_id=$2 reps=$3 run_dir=$4
    local repetition wall status out err timing vals precision fraction
    local -a case_args=() times=()
    local last_vals=
    case $case_id in
        0) case_args=("${case_args_0[@]}") ;;
        1) case_args=("${case_args_1[@]}") ;;
        2) case_args=("${case_args_2[@]}") ;;
        3) case_args=("${case_args_3[@]}") ;;
        4) case_args=("${case_args_4[@]}") ;;
        5) case_args=("${case_args_5[@]}") ;;
        *) fail "unknown benchmark case: $case_id" ;;
    esac
    TIMEFORMAT='%6R'
    for ((repetition = 0; repetition < reps; repetition++)); do
        out="$run_dir/case-$RANDOM-$repetition.out"
        err="$run_dir/case-$RANDOM-$repetition.err"
        timing="$run_dir/case-$RANDOM-$repetition.time"
        vals="$run_dir/case-$RANDOM-$repetition.vals"
        if { time "$binary" "${case_args[@]}" > "$out" 2> "$err"; } 2> "$timing"; then
            status=0
        else
            status=$?
        fi
        wall=$(awk '/^[0-9]+([.][0-9]+)?$/ { value = $0 } END { if (value != "") print value }' "$timing")
        [[ $wall =~ ^[0-9]+([.][0-9]+)?$ ]] || fail 'could not read benchmark wall time'
        if [[ $wall =~ ^0+([.]0+)?$ ]]; then
            fraction=${wall#*.}
            wall=0.
            for ((precision = 0; precision < ${#fraction}; precision++)); do wall="${wall}0"; done
            wall="${wall}5"
        fi
        times+=("$wall")
        if ((status != 0)); then
            printf 'COMMAND FAILED (exit %d):' "$status" >&2
            printf ' %q' "$binary" "${case_args[@]}" >&2
            printf '\n%s\n%s\n' "$(tail -c 2000 "$out")" "$(tail -c 2000 "$err")" >&2
            return "$status"
        fi
        parse_output "$out" "$vals"
        last_vals=$vals
    done
    CASE_WALL=$(printf '%s\n' "${times[@]}" | LC_ALL=C sort -n | awk '
        { values[NR] = $1 }
        END {
            if (NR % 2) printf "%.9f", values[(NR + 1) / 2]
            else printf "%.9f", (values[NR / 2] + values[NR / 2 + 1]) / 2
        }
    ')
    CASE_VALS=$last_vals
}

fil=./build-release/fil
if (($# > 0)) && [[ $1 != -* ]]; then
    fil=$1
    shift
fi
reps=3
while (($#)); do
    case $1 in
        --reps)
            (($# >= 2)) || fail '--reps requires a value'
            [[ $2 =~ ^[0-9]+$ ]] || fail '--reps must be a positive integer'
            reps=$((10#$2))
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            fail "unknown option: $1"
            ;;
    esac
done
((reps > 0)) || fail '--reps must be a positive integer'

if [[ $fil != /* ]]; then
    fil="$PWD/$fil"
fi
[[ -x $fil ]] || fail "fil executable not found or not executable: $fil"

run_root=$(mktemp -d "${TMPDIR:-/tmp}/fil-bench.XXXXXXXX")
trap 'rm -rf "$run_root"' EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM
idle_elf="$run_root/idle_gen.elf"
pll_elf="$run_root/pll_gen.elf"
build_idle_code
build_image "$idle_elf" "${CODE[@]}"
build_pll_code
build_image "$pll_elf" "${CODE[@]}"
write_configs "$run_root" "$idle_elf" "$pll_elf"

case_names=(
    'idle16 x1'
    'idle16 x1 no-batch'
    'idle16 x6'
    'idle16 x6 no-batch'
    'pll170 x1'
    'pll170 x1 no-batch'
)
declare -a case_args_0 case_args_1 case_args_2 case_args_3 case_args_4 case_args_5
case_args_0=(run "$run_root/idle.json" --duration-ms 1000 --max-instructions 200000000 --strict-mmio)
case_args_1=(run "$run_root/idle.json" --duration-ms 1000 --max-instructions 200000000 --strict-mmio --no-loop-batching --no-jit)
case_args_2=(run-network "$run_root/six.json" --duration-ms 1000 --max-instructions 200000000 --quantum 1024 --strict-mmio)
case_args_3=(run-network "$run_root/six.json" --duration-ms 1000 --max-instructions 200000000 --quantum 1024 --strict-mmio --no-loop-batching --no-jit --no-ram-capsules)
case_args_4=(run "$run_root/pll.json" --duration-ms 1000 --max-instructions 400000000 --strict-mmio)
case_args_5=(run "$run_root/pll.json" --duration-ms 1000 --max-instructions 400000000 --strict-mmio --no-loop-batching --no-jit)

printf '%-20s %12s %12s %6s %12s %10s %9s\n' \
    case instructions cycles cpi time_ns wall_s x_realtime
printf '%s\n' 'model: variable CPI + FLASH_ACR wait states + ART; deadline overshoot < 1 ms is atomic batch completion'
ok=1
for i in 0 1 2 3 4 5; do
    args_name="case_args_$i"
    run_case "$fil" "$i" "$reps" "$run_root"
    vals=$(<"$CASE_VALS")
    instructions=$(awk -F= '$1 == "instructions" {print $2}' <<< "$vals")
    cycles=$(awk -F= '$1 == "cycles" {print $2}' <<< "$vals")
    time_ns=$(awk -F= '$1 == "time_ns" {print $2}' <<< "$vals")
    [[ $instructions =~ ^[0-9]+$ && $cycles =~ ^[0-9]+$ && $time_ns =~ ^[0-9]+$ ]] \
        || fail "missing or invalid simulator counters for ${case_names[$i]}"
    cpi=$(awk -v c="$cycles" -v n="$instructions" 'BEGIN { printf "%.2f", n ? c/n : 0 }')
    throughput=$(awk -v t="$time_ns" -v w="$CASE_WALL" 'BEGIN { printf "%.2f", (w > 0 ? (t/1000000000)/w : 0) }')
    printf '%-20s %12d %12d %6s %12d %10.3f %9.2f\n' \
        "${case_names[$i]}" "$instructions" "$cycles" "$cpi" "$time_ns" "$CASE_WALL" "$throughput"
    if ((cycles < instructions)); then
        printf '  FAIL: cycles < instructions\n'
        ok=0
    fi
    if ((time_ns < 1000000000 || time_ns > 1001000000)); then
        printf '  FAIL: run missed the 1 s deadline window: %d\n' "$time_ns"
        ok=0
    fi
done
exit $((1 - ok))
