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

Register-backed peripherals merge byte lanes one word at a time. Each touched
word is loaded or stored once, in ascending address order, preserving
read-to-clear effects and changes made by earlier write callbacks. Bounds are
validated before any register effects. Register banks have inline capacity for
64 words and contiguous overflow storage for larger blocks; reset copies values
into the existing bank. Interrupt and override flags use bitsets.

## ELF materialization

`MemoryBus::materialize()` validates every segment's runtime allocation, then copies only file-backed bytes to physical load addresses. For split `.data` segments, initializer bytes enter flash while runtime SRAM remains zero until firmware reset code copies them. File-unbacked `.bss` storage remains at its deterministic RAM reset value.

## Native execution bindings

`directReadWindow()` borrows readable RAM/ROM/flash backing when read-footprint
tracking is off. `directWriteWindow()` borrows writable, non-executable RAM
when read/store-footprint tracking and write journaling are off. Both decline
without changing their outputs for ineligible mappings. Native instructions
check the complete access width against each binding, including unaligned
accesses and tiny regions. MMIO and alias mappings require checked execution.

Bindings are valid while bus identity, mappings and tracking modes remain
fixed; callers release them before changing these or resetting the board.
Native stores preserve exact changed-byte mutation and side-effect counts,
including unchanged stores. A committed region can accumulate these counts
and publish them before returning to checked execution or invoking observers.
Callers must bound a region before event/interrupt boundaries and must not
invoke observers within it, so accounting is published before callbacks can
observe native effects.
