# Timing model

`fil` uses an approximate Cortex-M4 pipeline and STM32G4 flash timing model to pace simulated time. It is not cycle-accurate hardware. For throughput comparisons and optimizations see [Performance](performance.md).

## Pipeline and flash

`CortexM4::stepFast()` charges decoded instruction-class costs rather than one cycle for every instruction. ALU instructions commonly cost one cycle; loads and branches cost more, multi-register transfers scale with register count, and divide/FP operations can cost multiple cycles. Taken branches incur refill; failed conditions have no memory or branch effect. The decode cache stores static costs, while data-dependent divides are computed at execution.

The board adds flash-fetch stalls according to the configured `FLASH_ACR` latency and simplified prefetch/instruction-cache behavior. Sequential fetches can hit a modeled fetch row; discontinuities can pay the configured wait states. Non-flash execution has no flash-fetch stall. The board caches clock/flash state and invalidates it when the underlying configuration changes. These approximations are informed by ARM DDI0439C and STM32G4 RM0440/DS12288; they do not model every silicon contention or cache effect.

## Simulated time and performance

`Board::accountCycles()` converts charged cycles at the current modeled system clock to nanoseconds while retaining the fractional remainder. Different clock settings and instruction mixes therefore change how many instructions execute in a fixed simulated duration. Loop batching carries the proven iteration's cycle cost forward without dispatching each instruction, provided clock and flash state remain stable for the proof.

A valid timing check uses `cycles >= instructions` and a simulated deadline close to the requested duration, not `cycles == instructions`. Benchmark host wall time separately: a higher modeled CPI can reduce the number of instructions needed to reach the same simulated deadline without making interpreter dispatch faster. The synthetic `make benchmark` fixtures check timing invariants but do not stand in for real ADC/DMA firmware.
