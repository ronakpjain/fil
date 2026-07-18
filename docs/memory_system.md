# Memory System

The memory bus maps non-overlapping regions into the Cortex-M 32-bit address space. It is independent of any firmware framework or application layout.

## Region types

- **RAM** is mutable, zero-initialized storage. Execute permission is configurable and disabled by default.
- **ROM** is read-only to CPU accesses, initialized to erased `0xff` bytes, and executable by default. The loader may populate it through a privileged setup path.
- **Alias** regions translate an address range into another mapped region. The STM32 boot alias at `0x00000000` can therefore mirror flash without duplicating bytes.
- **MMIO** regions dispatch the original access width and offset exactly once to a caller-owned peripheral model.

Regions are validated for empty ranges, 32-bit wraparound, and overlap when mapped.

## Access behavior

RAM and ROM use target little-endian byte order. Unaligned 16-, 32-, and 64-bit operations are supported when every byte remains in one region. An operation crossing a region boundary faults instead of being split. MMIO operations are never decomposed into byte accesses.

Each access includes its purpose and originating PC. Instruction fetches require execute permission. Failures return a structured `BusFault` describing the address, width, access type, region, PC, and machine-readable reason.

## ELF materialization

`MemoryBus::materialize()` validates every segment's runtime allocation, then copies only file-backed bytes to physical load addresses. For split `.data` segments, initializer bytes enter flash while runtime SRAM remains zero until firmware reset code copies them. File-unbacked `.bss` storage remains at its deterministic RAM reset value.
