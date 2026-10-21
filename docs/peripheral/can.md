# CAN and FDCAN

FIL connects STM32G4 FDCAN controllers through named in-process virtual CAN
buses. This is a frame-level simulation, not an electrical CAN implementation.
See the [peripheral index](index.md) for other device models.

## Configuration and input

Network CAN buses use bitrate-aware `timed` delivery by default:

```json
"buses": {
  "vehicle": {
    "type": "can",
    "bitrate": 500000,
    "timing": "timed",
    "queue_capacity": 256
  }
}
```

Attach board controllers to the declared bus using the
[network configuration](../configuration.md) schema. Queue capacity bounds the
combined pending and in-flight requests per bus. Bitrate must be nonzero.
Normalized configuration inspection includes the timing mode and capacity.
`timed_fifo` is a deprecated alias for `timed`, not a separate FIFO-only mode.

Explicit `timing: "immediate"` broadcasts synchronously without wire occupancy;
use it for idealized fixtures, not throughput validation. The standalone
`VirtualCanBus` C++ constructor also retains an immediate-mode compatibility
default; network construction selects timed mode explicitly.

External frames can enter through batch stimuli, interactive
[watch-network](../watch_network.md), or binary
[serve-network](../serve_network.md). All these inputs and attached firmware
transmissions use the same bus scheduler in timed mode.

## Wire timing and arbitration

Timed delivery supports standard and extended classic-CAN data frames, with zero
to eight payload bytes. The duration encoder builds protocol fields, calculates
CRC-15 using polynomial `0x4599`, and counts bit stuffing from SOF through the CRC
sequence. CRC delimiter, ACK fields, EOF, and the three-bit intermission are
unstuffed. A standard frame occupies `44 + 8*N` bits through EOF before stuffing;
extended frames have their own field layout.

The shared event loop schedules arbitration and EOF delivery. Transmit traces
mark SOF; receivers see the frame at EOF. Another frame cannot start before the
three-bit intermission ends. Integer nanosecond division carries a fractional
numerator across consecutive transmissions to avoid cumulative rounding drift.
The standalone duration helper rounds a single frame upward to nanoseconds.
Separate buses progress independently.

At each arbitration boundary, only each sender's oldest eligible request
contends. Actual arbitration bits are compared dominant-zero first, including
standard/extended SRR and IDE distinctions. An active frame is not preempted.
External injections form one ordered sender stream, so a later low-ID upload
word cannot overtake an earlier word. Firmware sender order follows controller
submission order; bus arbitration does not implement missing controller TX
priority-selection semantics.

Equal arbitration keys use admission order as a deterministic tie-break. Real
same-ID transmissions with different data can collide after arbitration; this
model instead sends them separately. A request admitted after an arbitration
callback runs participates at a subsequent boundary, even at the same timestamp.

## FDCAN controller behavior

The controller models a fixed shared message-RAM layout, three TX buffers, and
RX FIFO 0 with three elements. Supported filtering includes standard/extended
range, dual-ID, and classic-mask forms, plus nonmatching-frame routing/rejection.
FIFO acknowledgement frees entries; full/lost and overwrite behavior follow the
modeled register subset. Enabled peripheral interrupt sources drive the selected
NVIC interrupt lines through `IR`, `IE`, `ILS`, and `ILE`.

TX requests snapshot message-RAM contents on submission. In timed mode, `TXBRP`
remains pending until EOF; completion then updates `TXBTO`, FIFO indices, and
completion interrupts. Later message-RAM writes do not alter the queued payload.
Admission or scheduling failures clear pending ownership and report a controller
protocol error rather than claiming successful transmission.

Reset or sender detach cancels its pending work and immediately aborts its
in-flight frame without delivery. The bus becomes available again and restores
the pre-frame fractional timing remainder. This is a simulator lifecycle policy,
not a model of physical mid-frame reset/error recovery.

## Admission, traces, and receiver drops

A successful timed injection means **queue admission**, not wire completion,
ACK observation, or receiver FIFO acceptance. Binary service protocol v2 returns
status `4` when the queue is full; that request was not admitted. Retry after
simulation advances, without resending successfully admitted frames. See the
[service contract](../serve_network.md) for framing, STOP/EOF, and retry details.

Relevant trace types are:

- `can_tx`: virtual-bus SOF (immediate-mode traces retain synchronous behavior).
- `can_rx`: EOF delivery attempt, emitted before the peripheral callback.
- `can_tx_complete`: attached timed FDCAN completion at EOF.
- `can_tx_rejected`: bus admission/timing rejection.
- `can_rx_drop`: FDCAN full-FIFO rejection.

A bus receive trace does not prove receiver acceptance. Firmware executes between
scheduled wire events, but a receiver that does not service its bounded FIFO can
still lose frames. No FIFO enlargement or wall-clock sleep is used to enforce
wire timing. Live input admission times can depend on host arrival; replay must
preserve simulated admission timestamps and order.

## Fidelity and evidence

Timed CAN-FD is explicitly unsupported: configuration does not supply a separate
data bitrate, and classic-CAN CRC/stuffing rules are not valid for FD timing.
Immediate mode can transport normalized FD payloads within the controller's
modeled subset. ACK failures, error frames, retries, bus-off, propagation delay,
and electrical faults are not modeled. Controller bit-timing registers do not
configure the virtual bus's bitrate. RX FIFO 1, TX event FIFO, XIDAM filtering,
and full M_CAN configuration-rule conformance are unsupported or incomplete.

Register references and controller conformance evidence are maintained in the
[FDCAN coverage matrix](../stm32g4_peripheral_coverage.md#fdcan): RM0440
§44.3.6–§44.3.12 cover message RAM/filter elements; §44.4.6 covers `CCCR`;
§44.4.15–§44.4.18 cover interrupt gating; §44.4.19 and §44.4.22–§44.4.23 cover
RX routing/status/acknowledgement. Bus scheduling policies above are simulator
contracts, not claims of complete register or physical-CAN conformance.

Representative tests include:

- `CanBusTest.CountsStuffedClassicFramesAndIncludesIntermission`
- `CanBusTest.TimedArbitrationUsesSenderHeadsAndStandardFormatPriority`
- `CanBusTest.TimedEqualIdTieUsesStableAdmissionOrder`
- `CanBusTest.TimedArbitrationKeepsExternalInjectionOrder`
- `CanBusTest.NewlyEligibleSenderAdvancesFutureArbitrationFrontier`
- `CanBusTest.TimedQueueCarriesFractionalBitTimeRemainder`
- `FdcanTest.TimedTransmissionDefersCompletionAndResetCancels`
- `FdcanTest.TimedAdmissionFailureClearsTransmitOwnership`
- `FdcanTest.TracesBoundedReceiveFifoDrops`

Coverage is synthetic, not exhaustive across all frame encodings or execution
backends. No six-board PER bootloader acceptance result or physical hardware
validation is claimed.

Implementation: `src/devices/can_bus.cpp`, `src/stm32g4/fdcan.cpp`, and
`src/sim/world.cpp`. Tests: `tests/unit/can_bus_test.cpp`,
`tests/unit/fdcan_test.cpp`, and `tests/unit/world_test.cpp`.
