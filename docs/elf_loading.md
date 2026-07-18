# ELF Loading

The ELF loader accepts executable ELF32, little-endian ARM images. It uses program headers for loading and section headers only for optional symbols and ARM build attributes, so stripped firmware remains loadable.

## Load address versus runtime address

Bare-metal linker scripts commonly give initialized data two addresses:

- `p_paddr` is the physical/load-memory address containing initial bytes, usually in flash.
- `p_vaddr` is the runtime address where firmware expects the object, usually in SRAM.

`fil` preserves both addresses. It does not prepopulate runtime SRAM from a split segment; reset code must copy `.data`, matching hardware startup behavior. Zero-sized file data with nonzero memory size represents storage such as `.bss` or reserved stack/heap space.

## Validation

Loading rejects:

- invalid ELF magic, class, endianness, machine, type, or version;
- headers, sections, strings, and segment bytes outside the file;
- `p_filesz` larger than `p_memsz`;
- target address ranges that wrap 32-bit address space;
- overlapping physical load ranges containing different bytes;
- vector-table addresses outside file-backed load memory.

All integer fields are read manually as little-endian values. The parser never casts file bytes to host structs, avoiding alignment, padding, and host-endian assumptions.

## Optional metadata

When present, symbol tables are retained in address order for `symbol+offset` diagnostics. Recognized `.ARM.attributes` include CPU, Thumb ISA, floating-point architecture, hard-float use, VFP argument convention, and unaligned-access tags.

Inspect an image with:

```bash
fil inspect-elf path/to/firmware.elf
```
