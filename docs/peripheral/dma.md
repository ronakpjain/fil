# DMA and DMAMUX

DMA1/2 provide channel register layout and request-driven transfers through the attached MemoryBus. Supported items are 1, 2, or 4 bytes, with direction and peripheral/memory increment selection. CNDTR decrements, circular mode reloads, and completion/error set flags and may assert interrupts. Peripheral requests trigger work; serial peripherals can drain bursts in the model. DMAMUX channel configuration selects request IDs and routes modeled ADC1–5, USART1–3, and SPI1–3 requests to DMA channels.

**Use:** program channel source/destination, count, widths, direction, increments and circular/interrupt controls; enable the channel and select the matching request in DMAMUX. Confirm destination regions are accessible to MemoryBus. Completion/error bits are acknowledged through DMA flag-clear registers.

**Limits:** no arbitration, bandwidth/transfer timing, alignment enforcement, half-transfer interrupt, general burst engine, DMA request generators, or DMAMUX synchronization. This is request/callback-driven. DMAMUX overrun flags are not generated. Not every request route has end-to-end tests.

**RM0440:** DMA §12.4.3, §12.4.5–12.4.7; §12.6.1–12.6.4 (`ISR/IFCR/CCRx/CNDTRx`). DMAMUX §13.6.1 `CxCR`, §13.6.2 `CSR`; routing §14.3 Table 100. USART/SPI request controls: §40.8.4 `CR3`, §42.9.2 `CR2`; ADC data management §21.4.26.

**Tests:** `PeripheralTest.DmaByteHalfwordWordAndBothDirectionsHonorIncrementBits`; `PeripheralTest.DmaWidthsIncrementsCircularAndFailureFlags`; `PeripheralTest.DmamuxRequestSelectorAndReadOnlyClearStatus`; `Stm32G4Test.IntegratedUsartAndSpiTransfersReachDmaThroughDmamux`; `Stm32G4Test.Adc5RoutesItsDmaRequestAndDedicatedInterrupt`.

See the [coverage matrix](../stm32g4_peripheral_coverage.md#dma-and-dmamux) for the tested request routes and omissions.
