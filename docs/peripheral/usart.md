# USART

USART1–3 expose a scripted/provider receive queue and immediate transmit path. Reading RDR consumes queued data; TDR writes append bytes to a log/callback. RXNE, IDLE, TC, TXE and selected acknowledge flags/interrupt gates are modeled, as are CR3 DMAT/DMAR requests routed through DMAMUX to DMA. Board configuration can provide binary `tx_log` output.

**Use:** enqueue/provide RX bytes, enable UE/RE or UE/TE as appropriate, consume RDR or write TDR, and configure CR1/CR3 interrupt or DMA request enables. A DMA transfer requires a compatible DMAMUX request selection and enabled DMA channel.

**Limits:** TX and transfer requests are immediate; there is no baud/frame timing, parity, oversampling, FIFO depth, framing errors, CTS/RTS, or synchronous mode. IDLE uses an emulator gap; provider refill is observation-driven, not a hardware event.

**RM0440:** §40.5.5, §40.5.6, §40.5.19; §40.8.1 `CR1`; §40.8.4 `CR3`; §40.8.8 `RQR`; §40.8.9 `ISR`; §40.8.11 `ICR`; §40.8.12 `RDR`; §40.8.13 `TDR`.

**Tests:** `PeripheralTest.ModelsUsartAndSpiDataPaths`; `PeripheralTest.UsartAcknowledgementsIdleRefillAndInterruptGates`; `PeripheralTest.SignalsUsartDmaRequestsForTxAndRx`; `Stm32G4Test.IntegratedUsartAndSpiTransfersReachDmaThroughDmamux`.

See the [coverage matrix](../stm32g4_peripheral_coverage.md#usart) for flags, route details, and unsupported behavior.
