#!/usr/bin/env bash
# Audit every instruction recognized by objdump against fil's Thumb decoder.
set -uo pipefail
shopt -s nocasematch

prog=${0##*/}
fil=''
objdump=arm-none-eabi-objdump
report=''
maximum_chunk=4096
elfs=()
usage() {
  cat <<EOF
Usage: $prog --fil PATH [--objdump PATH] [--report PATH] [--maximum-chunk N] ELF...

Audit every objdump instruction by asking fil to decode the same addresses.

Options:
  --fil PATH             Path to the fil executable (required)
  --objdump PATH         ARM objdump executable (default: arm-none-eabi-objdump)
  --report PATH          Optional JSON report output
  --maximum-chunk N      Maximum contiguous instructions per request (default: 4096)
  -h, --help             Show this help
EOF
}
fail() { printf '%s: %s\n' "$prog" "$1" >&2; exit 1; }
while (($#)); do
  case $1 in
    --fil|--objdump|--report|--maximum-chunk)
      option=$1; shift
      (($#)) || fail "$option requires a value"
      case $option in
        --fil) fil=$1 ;;
        --objdump) objdump=$1 ;;
        --report) report=$1 ;;
        --maximum-chunk) maximum_chunk=$1 ;;
      esac
      shift ;;
    --fil=*) fil=${1#*=}; shift ;;
    --objdump=*) objdump=${1#*=}; shift ;;
    --report=*) report=${1#*=}; shift ;;
    --maximum-chunk=*) maximum_chunk=${1#*=}; shift ;;
    -h|--help) usage; exit 0 ;;
    --) shift; while (($#)); do elfs+=("$1"); shift; done ;;
    -*) fail "unrecognized option: $1" ;;
    *) elfs+=("$1"); shift ;;
  esac
done
[[ -n $fil ]] || fail "the following arguments are required: --fil"
((${#elfs[@]})) || fail "the following arguments are required: ELF"
[[ $maximum_chunk =~ ^[0-9]+$ ]] || fail "invalid --maximum-chunk: $maximum_chunk"
# Bound arithmetic to Bash's signed integer range and reject zero.
((10#$maximum_chunk > 0 && 10#$maximum_chunk <= 2147483647)) || fail "--maximum-chunk must be positive"
maximum_chunk=$((10#$maximum_chunk))
if [[ -n $report ]] && ! command -v jq >/dev/null 2>&1; then fail "jq is required to write a JSON report"; fi

tmpdir=$(mktemp -d) || fail "could not create temporary directory"
trap 'rm -rf "$tmpdir"' EXIT
run_checked() {
  local output status rendered arg
  : >"$tmpdir/stderr"
  output=$("$@" 2>"$tmpdir/stderr"); status=$?
  if ((status != 0)); then
    rendered=''; for arg in "$@"; do rendered+="${rendered:+ }$arg"; done
    printf 'command failed with exit %d: %s\nstdout:\n%s\nstderr:\n%s\n' "$status" "$rendered" "$output" "$(cat "$tmpdir/stderr")" >&2
    return 1
  fi
  RUN_OUTPUT=$output
}

# Converts a hexadecimal address string to a canonical eight-digit address.
canon_addr() { printf '%08x' "$((16#$1))"; }
resolve_path() {
  local path=$1 current_dir target base links=0
  [[ $path == /* ]] || path="$PWD/$path"
  while [[ -L $path ]]; do
    links=$((links + 1))
    ((links <= 40)) || return 1
    current_dir=$(cd -P "$(dirname "$path")" && pwd) || return 1
    target=$(readlink "$path") || return 1
    if [[ $target == /* ]]; then path=$target; else path="$current_dir/$target"; fi
  done
  current_dir=$(cd -P "$(dirname "$path")" && pwd) || return 1
  base=${path##*/}
  printf '%s/%s\n' "$current_dir" "$base"
}

reports_json='[]'
total_instructions=0
total_decoded=0
for elf in "${elfs[@]}"; do
  [[ -f $elf ]] || fail "ELF does not exist: $elf"
  if ! run_checked "$objdump" -d "$elf"; then exit 1; fi
  dump=$RUN_OUTPUT
  addresses=(); encodings=(); mnemonics=()
  while IFS= read -r line || [[ -n $line ]]; do
    # Objdump uses tab-separated address, encoding, and assembly columns.
    [[ $line == *$'\t'*$'\t'* ]] || continue
    address_text=${line%%$'\t'*}; address_text=${address_text//[[:space:]]/}
    [[ $address_text =~ ^[0-9a-fA-F]+:$ ]] || continue
    rest=${line#*$'\t'}; raw=${rest%%$'\t'*}; assembly=${rest#*$'\t'}
    read -r -a words <<<"$raw"
    ((${#words[@]} == 1 || ${#words[@]} == 2)) || continue
    valid=1
    for word in "${words[@]}"; do [[ $word =~ ^[0-9a-fA-F]{4}$ ]] || valid=0; done
    ((valid)) || continue
    assembly=${assembly#"${assembly%%[!$' \t']*}"}
    [[ -n $assembly ]] || continue
    read -r mnemonic _ <<<"$assembly"
    [[ $mnemonic != .* ]] || continue
    hex=${address_text%:}; addr=$(canon_addr "$hex") || fail "invalid address in objdump output: $address_text"
    encoding=''; for word in "${words[@]}"; do encoding+="${encoding:+ }$word"; done
    addresses+=("$addr"); encodings+=("$encoding"); mnemonics+=("$mnemonic")
  done <<<"$dump"
  ((${#addresses[@]})) || fail "objdump found no instructions in $elf"
  duplicate=$(printf '%s\n' "${addresses[@]}" | LC_ALL=C sort | uniq -d | head -n 1)
  [[ -z $duplicate ]] || fail "objdump emitted duplicate instruction address 0x$duplicate for $elf"

  chunks=0; decoded=0; failures=()
  start=0; count=${#addresses[@]}
  while ((start < count)); do
    end=$((start + 1))
    while ((end < count && end - start < maximum_chunk)); do
      prev=$((16#${addresses[end-1]})); current=$((16#${addresses[end]}))
      width=$(($(wc -w <<<"${encodings[end-1]}" | tr -d ' ') * 2))
      ((current == prev + width)) || break
      ((end+=1))
    done
    n=$((end-start)); ((chunks+=1))
    if ! run_checked "$fil" disasm-window "$elf" --addr "0x${addresses[start]}" --count "$n"; then exit 1; fi
    output=$RUN_OUTPUT
    actual=(); while IFS= read -r line || [[ -n $line ]]; do [[ $line =~ [^[:space:]] ]] && actual+=("$line"); done <<<"$output"
    if ((${#actual[@]} != n)); then
      failures+=("0x${addresses[start]}: expected $n decoded lines, got ${#actual[@]}")
      start=$end; continue
    fi
    for ((j=0; j<n; j++)); do
      line=${actual[j]}; read -r -a fields <<<"$line"
      if ((${#fields[@]} < 3)) || [[ ${fields[0]} != 0x*:* ]]; then fail "unexpected fil disassembly line: '$line'"; fi
      hex=${fields[0]#0x}; hex=${hex%:}; [[ $hex =~ ^[0-9a-fA-F]+$ ]] || fail "unexpected fil disassembly line: '$line'"
      got=$(canon_addr "$hex"); idx=$((start+j)); expected=${addresses[idx]}
      k=1; actual_encoding=''
      while ((k < ${#fields[@]} && k <= 2)) && [[ ${fields[k]} =~ ^[0-9a-fA-F]{4}$ ]]; do
        actual_encoding+="${actual_encoding:+ }${fields[k]}"; ((k+=1))
      done
      [[ -n $actual_encoding && $k -lt ${#fields[@]} ]] || fail "missing raw encoding or mnemonic in fil output: '$line'"
      actual_mnemonic=${fields[k]}
      if [[ $got != "$expected" ]]; then failures+=("0x$expected: fil advanced to 0x$got"); continue; fi
      if [[ $actual_encoding != "${encodings[idx]}" ]]; then failures+=("0x$got: raw encoding differs: objdump ${encodings[idx]}, fil $actual_encoding"); continue; fi
      if [[ $actual_mnemonic == unsupported ]]; then failures+=("0x$got: unsupported objdump mnemonic ${mnemonics[idx]} (${encodings[idx]})"); continue; fi
      ((decoded+=1))
    done
    start=$end
  done
  if ((${#failures[@]})); then
    details=''; shown=${#failures[@]}; ((shown > 50)) && shown=50
    for ((i=0; i<shown; i++)); do details+="${details:+$'\n'}${failures[i]}"; done
    (( ${#failures[@]} <= 50 )) || details+=$'\n...'" $((${#failures[@]}-50)) additional failures"
    fail "instruction audit failed for $elf:"$'\n'"$details"
  fi
  if command -v sha256sum >/dev/null 2>&1; then
    sha=$(sha256sum -- "$elf") || fail "could not hash ELF: $elf"
  else
    sha=$(shasum -a 256 "$elf") || fail "could not hash ELF: $elf"
  fi
  sha=${sha%% *}
  base=${elf##*/}; name=${base%.*}
  printf '%s: %d objdump instructions decoded, 0 unsupported, 0 address/width mismatches\n' "$name" "$decoded"
  if [[ -n $report ]]; then
    resolved=$(resolve_path "$elf") || fail "could not resolve ELF path: $elf"
    item=$(jq -n --arg name "$name" --arg path "$resolved" --arg sha "$sha" --argjson objdump "$count" --argjson decoded "$decoded" --argjson chunks "$chunks" '{name:$name,path:$path,sha256:$sha,objdump_instructions:$objdump,decoded_instructions:$decoded,contiguous_chunks:$chunks,unsupported:0,address_or_width_mismatches:0}') || fail "could not construct report"
    reports_json=$(jq --argjson item "$item" '. + [$item]' <<<"$reports_json") || fail "could not construct report"
  fi
  total_instructions=$((total_instructions + count)); total_decoded=$((total_decoded + decoded))
done
if ! run_checked "$fil" --version; then exit 1; fi
fil_version=$(printf '%s' "$RUN_OUTPUT" | sed -e 's/[[:space:]]*$//')
if ! run_checked "$objdump" --version; then exit 1; fi
objdump_version=${RUN_OUTPUT%%$'\n'*}
if [[ -n $report ]]; then
  parent=${report%/*}; [[ $parent != "$report" ]] || parent=.
  mkdir -p -- "$parent" || fail "could not create report directory: $parent"
  jq -n --arg fv "$fil_version" --arg ov "$objdump_version" --argjson count "${#elfs[@]}" --argjson total "$total_instructions" --argjson decoded "$total_decoded" --argjson elfs "$reports_json" '{schema_version:1,fil_version:$fv,objdump_version:$ov,elf_count:$count,total_objdump_instructions:$total,total_decoded_instructions:$decoded,unsupported:0,address_or_width_mismatches:0,elfs:$elfs}' >"$report" || fail "could not write JSON report: $report"
fi
printf 'total: %d instructions across %d ELFs\n' "$total_decoded" "${#elfs[@]}"
