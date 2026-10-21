# SPI

SPI1–3 implement immediate byte or halfword transfers on DR writes. The response can be zero/echo or supplied by a callback; reads consume queued response data. RXNE/TXE/brief BSY status, selected CR2 interrupt gates, transfer tracing, and RX/TX DMA request callbacks are modeled.

**Use:** perform DR accesses at supported widths and configure CR2 interrupt/DMA enables. DMA requires the selected DMAMUX request and active channel. A custom response callback can model a peripheral response without serial-line simulation.

**Limits:** no serial clock, polarity/phase, chip-select/NSS sequencing, frame formats beyond access width, FIFO depth, CRC, underrun/overrun, or I2S. Configured DS does not change supported transfer widths.

**RM0440:** §42.5.9–42.5.10; §42.9.2 `CR2`; §42.9.3 `SR`; §42.9.4 `DR`.

**Tests:** `PeripheralTest.SpiConsumesFramesAndSignalsReceiveDma`; `PeripheralTest.ModelsUsartAndSpiDataPaths`; `Stm32G4Test.IntegratedUsartAndSpiTransfersReachDmaThroughDmamux`.

See the [coverage matrix](../stm32g4_peripheral_coverage.md#spi) for DMA and fidelity details.
