#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="$root/tests/firmware_src/elf_fixture"
startup_source_dir="$root/tests/firmware_src/startup_fixture"
scheduler_source_dir="$root/tests/firmware_src/scheduler_tick_fixture"
hard_float_source_dir="$root/tests/firmware_src/hard_float_fixture"
hardware_compare_source_dir="$root/tests/firmware_src/hardware_compare_fixture"
output_dir="$root/tests/fixtures/elf"
compiler="${ARM_NONE_EABI_GCC:-arm-none-eabi-gcc}"

mkdir -p "$output_dir"
"$compiler" \
    -mcpu=cortex-m4 \
    -mthumb \
    -mfpu=fpv4-sp-d16 \
    -mfloat-abi=hard \
    -nostdlib \
    -Wl,--build-id=none \
    -Wl,-T,"$source_dir/linker.ld" \
    "$source_dir/startup.S" \
    -o "$output_dir/split_image.elf"

"$compiler" \
    -mcpu=cortex-m4 \
    -mthumb \
    -nostdlib \
    -Wl,--build-id=none \
    -Wl,-T,"$startup_source_dir/linker.ld" \
    "$startup_source_dir/startup.S" \
    -o "$output_dir/startup_runtime.elf"

"$compiler" \
    -mcpu=cortex-m4 \
    -mthumb \
    -nostdlib \
    -Wl,--build-id=none \
    -Wl,-T,"$scheduler_source_dir/linker.ld" \
    "$scheduler_source_dir/startup.S" \
    -o "$output_dir/scheduler_tick.elf"

"$compiler" \
    -mcpu=cortex-m4 \
    -mthumb \
    -nostdlib \
    -Wl,--build-id=none \
    -Wl,-T,"$hardware_compare_source_dir/linker.ld" \
    "$hardware_compare_source_dir/startup.S" \
    -o "$output_dir/hardware_compare.elf"

"$compiler" \
    -mcpu=cortex-m4 \
    -mthumb \
    -mfpu=fpv4-sp-d16 \
    -mfloat-abi=hard \
    -std=c11 \
    -O1 \
    -ffreestanding \
    -fno-builtin \
    -fno-inline \
    -fno-ipa-cp \
    -ffp-contract=off \
    -fno-stack-protector \
    -fno-unwind-tables \
    -fno-asynchronous-unwind-tables \
    -nostdlib \
    -Wl,--build-id=none \
    -Wl,-T,"$hard_float_source_dir/linker.ld" \
    "$hard_float_source_dir/startup.S" \
    "$hard_float_source_dir/hard_float.c" \
    -o "$output_dir/hard_float.elf"

echo "wrote $output_dir/split_image.elf, $output_dir/startup_runtime.elf, $output_dir/scheduler_tick.elf, $output_dir/hardware_compare.elf, and $output_dir/hard_float.elf"
