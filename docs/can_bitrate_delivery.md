# Proposal: bitrate-aware CAN delivery

Status: default partial implementation. This document preserves the original
design proposal. Per-bus `timing: "timed"` arbitrates classic-CAN transmissions
through the world's shared event loop with bounded sender queues and is the
default; `timing: "immediate"` remains available as an explicit compatibility mode.
The former `timed_fifo` spelling remains a deprecated configuration/API alias. Firmware TX ownership
and completion flags remain pending until the timed transmission completion,
and reset/detach cancel owned work. Timed `CAN_INJECT` reports queue admission;
completion/delivery is later. Transmit trace timestamps mark SOF, receiver trace
and callbacks occur at EOF, and the next queued frame cannot start until after
intermission. The timed model uses deterministic frame-level arbitration among per-sender queue heads.
It does not include CAN-FD timing, ACK failures, retransmissions, error frames,
bus-off, propagation, or electrical faults.

The duration helper supports standard and extended classic-CAN data frames,
CRC-15 (CAN polynomial 0x4599), stuffing from SOF through the CRC sequence, and
unstuffed CRC delimiter/ACK/EOF fields. Timed occupancy includes the three-bit
intermission; its per-bus integer numerator remainder is carried across frames
to avoid cumulative drift. The public standalone helper rounds an individual
frame upward to nanoseconds. Timed mode rejects CAN-FD frames and zero bitrate.

## Motivation

In explicit `immediate` mode, `VirtualCanBus` broadcasts `send()` and
`inject()` synchronously at the supplied timestamp. In the default `timed` mode, requests are admitted to bounded sender queues and
arbitrated at deferred shared-event-loop boundaries
in the world event loop. The timing mode and queue capacity appear in normalized network
configuration.

The `serve-network` stdio handler reads up to 4096 bytes and dispatches all complete
requests in that read before returning to world execution. An unpaced client can
therefore inject many CAN frames at one simulated instant. The emulated FDCAN RX
FIFO holds three frames; firmware cannot drain it between those immediate
injections. A successful injection reply does not imply receiver FIFO acceptance.

The proposal's motivating six-board PER experiment report is retained here as
historical context, but no raw run logs, firmware package, or reproducible test
artifacts are included in this repository. It is not treated as a verified result
of the current implementation and no PER acceptance claim is made.

## Implemented timed arbitration and remaining fidelity limits

A queue slot counts pending and in-flight requests. Queue-full admission returns
an explicit failure; the stdio protocol reports status 4 (`queue_full`) and the
client may retry after world execution advances. No blocking wait is performed.
At each deferred arbitration event, only each sender's oldest request is eligible;
external injections form one ordered sender stream. Classic CAN arbitration bits
are compared dominant-zero first, including standard/extended SRR and IDE bits.
Equal arbitration keys use admission sequence as a deterministic simulator tie-break.
That tie-break is not physical CAN behavior: same-ID contenders with differing
post-arbitration data are not modeled as bit errors/retransmissions. Requests
arriving after an arbitration callback are considered at the next arbitration
point. Reset or detach cancels that sender's pending work and aborts its in-flight
frame immediately, releasing the bus; the fractional timing remainder is restored
to the last completed frame boundary.

The host may submit an upload without sleeps. FIL queues transmission requests,
arbitrates them on the simulated bus, and lets firmware execute between frame
completions. Configured bitrate determines simulated wire occupancy; wall-clock
sleep does not.

For an otherwise idle bus:

```text
host submits A, B, C at simulated time t0
    A transmits [t0, t1); receiver sees A at t1
    intermission
    B transmits [t2, t3); receiver sees B at t3
    intermission
    C transmits [t4, t5); receiver sees C at t5
```

Firmware and peripherals progress throughout these intervals. A receiver that
cannot drain its FIFO fast enough still loses frames. Do not enlarge the FIFO or
silently slow the bus to make uploads succeed.

## Frame duration

Start with classic CAN data frames. For a standard-ID frame with `N` payload bytes,
the unstuffed frame from SOF through EOF occupies `44 + 8*N` bits. Add the actual
stuff bits in the SOF-through-CRC-sequence region, and account separately for the
three-bit intermission before the next frame can start. Extended-ID frames have a
different arbitration/control layout and need their own encoder/counting path.

For seven data bytes, the standard frame is 100 bits before stuffing, or 103 bits
including intermission. At 500 kbit/s that is at least 206 microseconds between
successive frame starts; stuffing increases it. This is an illustration, not a
fixed per-frame delay or a worst-case bound.

The duration helper should construct the protocol fields, calculate the CAN CRC,
and count bit stuffing over the correct region. ACK, CRC delimiter, ACK delimiter,
EOF, and intermission must not accidentally receive ordinary frame bit stuffing.
Use integer simulated-time arithmetic, checked for overflow, and retain fractional
bit-time remainder so rounding does not accumulate avoidable long-run drift.
Define how arbitrary configured bitrates map onto the event loop's nanosecond
resolution.

A fixed worst-case frame size is a possible preliminary conservative mode, but it
must be labelled approximate. Extra spacing can hide RX-service problems, so it
cannot establish that an upload sustains the real bus's maximum throughput.

CAN-FD requires a separate duration model: nominal and data bitrates, BRS phase
boundaries, and CAN-FD CRC/stuffing rules. Do not apply the classic formula to FD
frames or silently assume a data bitrate that configuration does not supply.
Unsupported timing combinations should fail explicitly in timed mode.

## Shared bus scheduler

Each bus needs pending transmission requests, an in-flight frame, and its next
eligible start time. A request records sender identity, frame contents, requested
simulated time, and a stable tie-break sequence.

1. Validate the request and reserve bounded queue capacity.
2. If idle and eligible, select the next transmission.
3. Schedule an EOF delivery event on the world's existing shared event loop.
4. At EOF, deliver the frame to attached receivers and report transmitter
   completion. Apply normal receiver filters and FIFO limits.
5. After the three-bit intermission, defer the next arbitration among eligible sender heads.

An active frame is not preempted. At arbitration, only the head request for each
sender participates; external injections share one sender queue and retain
admission order. The arbitration key compares standard/extended identifier fields
as transmitted on CAN. Equal keys select the earliest admitted frame; this stable
tie-break is a simulator policy, not a same-ID collision/error model. Bitrate zero
and timed CAN-FD requests are explicitly rejected before queue admission.

Within a sender, requests remain FIFO, so arbitration cannot reorder sequential
firmware words or an external upload backlog. FDCAN buffer submission order remains
the controller's current behavior; this scheduler does not add missing TX-buffer
priority semantics.

Both firmware `send()` and external `inject()` must use this scheduler. Otherwise
upload frames occupy the bus while replies or telemetry unrealistically bypass it.
Separate buses progress independently.

## Controller and world integration

Transmission submission must no longer mean immediate hardware completion.
Integrate scheduling with FDCAN TX buffer ownership, completion flags/interrupts,
queue advancement, and cancellation. Firmware must not observe a completed send
before its modeled transmission completes. Snapshot the payload at the appropriate
controller ownership boundary so later buffer writes do not alter an in-flight
frame.

Scheduled CAN events must participate in world scheduling frontiers. Every
receiver must be advanced consistently to the delivery time before callbacks or
interrupts make the frame observable. Deferred execution, JIT, and RAM-capsule
prefixes must not skip across delivery events. Reuse the event-loop and
materialization mechanisms; do not recursively run the world from an injection
callback or move its clock forward without executing firmware.

Handle sender detach, reset, bus teardown, pending callbacks, and in-flight
cancellation without dangling receiver/controller references. Define equal-time
event ordering and how reset affects bus activity explicitly.

## Stdio semantics and backpressure

A `CAN_INJECT` reply should mean the frame was accepted into the simulated
transmitter queue at `applied_time_ns`, not that it reached the wire, was ACKed, or
entered a receiver FIFO. Document that distinction. Wire transmission and delivery
have separate `can_tx` SOF and `can_rx` EOF timestamps. The bus receive trace is
emitted before the peripheral callback, so it alone does not prove FIFO
acceptance; FDCAN reports full-FIFO drops as `can_rx_drop`.

Queues must be bounded. An unlimited host burst must not allocate unlimited
memory. Specify queue-full behavior: an explicit rejection with a documented retry
contract, or transport backpressure that still permits simulation and replies to
progress. Avoid blocking the simulation thread waiting for capacity that only
simulation can free. If a new reply status is introduced, update/version the wire
contract and clients rather than reusing an unrelated existing status.

Queue admission and wire scheduling are separate concerns. Do not make wire timing
depend on stdio read boundaries, host sleeps, or the size of an OS pipe buffer.
A continuously busy input stream must not starve world execution. For live input,
wall-clock arrival can still affect which simulated request-admission boundary is
reached; deterministic replay should record admission timestamps and ordering.

## Fidelity boundary

Serialization prevents zero-time bursts, but does not by itself model ACK failures,
error frames, retransmission, bus-off, propagation delay, or electrical faults.
Initially assuming a healthy bus with successful ACKs is reasonable if explicit.
A timed model must not report successful wire completion while accidentally
claiming these unimplemented failure modes were tested.

An idealized immediate mode may remain useful for existing fixtures. Make timing
mode explicit in configuration and normalized inspection output, with documented
defaults and migration guidance. Update tests and docs that rely on instantaneous
transmission. Test all relevant delivery entry points: batch stimuli, stdio,
firmware-originated frames, and any interactive injection path.

## Validation status

Synthetic coverage checks representative standard/extended classic frame
occupancy, intermission and fractional timing, FIFO spacing and backpressure,
shared-event delivery, FDCAN deferred TX completion/cancellation, and protocol
queue-full behavior. It is not exhaustive coverage of every identifier and
payload length or every execution backend.

No six-board PER experiment artifacts are present. No PER result is claimed.

### Remaining synthetic tests

- Known classic-CAN frame encodings, CRCs, stuffing counts, and durations for
  standard/extended IDs and payload lengths zero through eight.
- Back-to-back frames never overlap and have the required intermission.
- Firmware executes and services interrupts between deliveries; receive/drop
  timing follows the bounded FIFO rather than the host input chunk size.
- Concurrent senders obey the declared arbitration/ordering model, without
  preemption; separate buses remain independent.
- TX completion occurs at the modeled boundary, not at queue submission.
- Queue saturation, invalid bitrate, unsupported FD timing, cancellation, reset,
  detach, and time arithmetic boundaries fail or recover as specified.
- Identical timestamped requests produce identical wire traces under different
  stdio chunking, and reference/JIT/capsule execution agree on delivery boundaries.
- STOP/EOF and backpressure do not deadlock or lose final protocol diagnostics.

### PER acceptance experiment

Run all six resident bootloaders together using the PR #553 application package.
Use `serve-network` and submit each board's data stream without client sleeps,
respecting only explicit queue admission/backpressure. Update boards sequentially
first, then optionally test interleaved uploads and competing vehicle traffic.

Require image-size ACK and exact expected CRC ACK for every board, no sequence or
flash errors, and application-specific CAN traffic after each JUMP. Check reset
and metadata-based auto-boot separately; initial handoff does not prove reboot
persistence. Compare against the paced control and an intentionally overloaded
receiver fixture so passing results cannot come from an unlimited RX buffer.

Report wire utilization, queue depth, FIFO drops, admission timestamps, upload
completion time, simulator mode, and firmware identity. A pass supports this modeled
bus and tested workload; physical hardware validation remains necessary.

## Relevant implementation locations

- `include/fil/devices/can_bus.hpp` and `src/devices/can_bus.cpp`: immediate and
  opt-in FIFO bus API, classic-CAN duration calculation, and bounded scheduling.
- `src/cli/cli.cpp`: stdio request decoding and injection at slice boundaries.
- `src/stm32g4/fdcan.cpp`: receive FIFO, TX behavior, and interrupt integration.
- `src/sim/world.cpp`: shared-world scheduling and observation barriers.
- [Binary network service](serve_network.md): request/reply and trace contracts.
- [Architecture](architecture.md): event ordering and fidelity boundaries.
