# Peripheral reference

These pages describe FIL's modeled peripherals: how firmware interacts with them,
which behavior is functional, and where simulation differs from hardware. They
are usage and fidelity references, not complete register manuals. Stored register
bits do not necessarily imply modeled hardware behavior.

| Model | Reference |
|---|---|
| RCC clock/reset control | [RCC](rcc.md) |
| Flash interface and erase backing | [FLASH](flash.md) |
| Power control | [PWR](pwr.md) |
| GPIO pins and external drive | [GPIO](gpio.md) |
| SYSCFG routing and external interrupts | [SYSCFG and EXTI](exti.md) |
| CRC calculation unit | [CRC](crc.md) |
| ADC regular conversions and common clocks | [ADC](adc.md) |
| General/advanced timer subset | [TIM](tim.md) |
| Independent and window watchdogs | [IWDG and WWDG](watchdogs.md) |
| Serial interfaces | [USART](usart.md), [SPI](spi.md) |
| DMA transfers and request routing | [DMA and DMAMUX](dma.md) |
| FDCAN controllers and virtual CAN buses | [CAN and FDCAN](can.md) |
| SysTick, NVIC, SCB, CoreDebug/DWT and FP state | [Cortex-M system peripherals](cortex_m.md) |

For mapped instances/addresses and exact reference-manual/register-to-test
traceability, see [STM32G4 peripheral coverage](../stm32g4_peripheral_coverage.md).
That matrix distinguishes tested hardware subsets, simulator policies,
storage-only registers, and unsupported features. Reference citations and named
tests on these pages are evidence for the stated subset, not blanket conformance
claims.

See [configuration](../configuration.md) for board and network setup,
[architecture](../architecture.md) for scheduling and MMIO contracts, and
[tracing](../tracing.md) for observing peripheral activity. External live inputs
are documented in [watch-network](../watch_network.md) and
[serve-network](../serve_network.md).
