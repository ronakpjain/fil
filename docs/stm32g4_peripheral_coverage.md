# STM32G4 peripheral coverage

The STM32G4 layer is a deterministic register-level model for startup, RTOS scheduling, and device-facing firmware tests. It preserves little-endian byte/halfword/word register accesses and implements selected side effects. It is not a complete STM32G474 electrical, timing, or register-accuracy model.

## Routed map

| Device | Address or instances | Current level |
|---|---|---|
| RCC | `0x40021000` | Startup clock side effects |
| FLASH | `0x40022000` | Startup/control side effects |
| PWR | `0x40007000` | Permissive register storage |
| GPIO | GPIOA-G at `0x48000000 + port * 0x400` | Digital input/output behavior |
| USART | USART1 `0x40013800`, USART2 `0x40004400`, USART3 `0x40004800` | Scripted byte data path |
| TIM | TIM1-8, TIM15-17, TIM20 at STM32G474 bases | Basic up-counter/update behavior |
| ADC | ADC1/2 at `0x50000000/0x50000100`, ADC3/4 at `0x50000400/0x50000500` | Clock-timed regular sequences |
| ADC common | ADC12 at `0x50000300`, ADC345 at `0x50000700` | Lenient sparse register stubs |
| SPI | SPI1 `0x40013000`, SPI2 `0x40003800`, SPI3 `0x40003c00` | Immediate byte/halfword transfers |
| DMA | DMA1 `0x40020000`, DMA2 `0x40020400` | Request-driven channel transfers |
| DMAMUX | `0x40020800` | Request selection and ADC routing |
| IWDG / WWDG | `0x40003000` / `0x40002c00` | Deterministic timeout scheduling |
| FDCAN | FDCAN1-3 at `0x40006400`, `0x40006800`, `0x40006c00` | Message-RAM TX/RX and interrupts |
| FDCAN message RAM | `0x4000a400`, three `0x350`-byte slices | CPU-visible shared storage |
| SYSCFG / EXTI | `0x40010000` / `0x40010400` | Lenient sparse register stubs |

Within a modeled register block, registers without a special hook generally retain written values. An address outside all routed blocks is handled by the top-level MMIO policy: lenient mode returns zero and counts the address, while `--strict-mmio` faults.

## Implemented device behavior

| Device | Implemented and tested semantics | Intentional simplifications or gaps |
|---|---|---|
| RCC | HSI reset state; immediate MSI, HSI, HSE, PLL, LSE, LSI, and HSI48 ready reflection; SW-to-SWS reflection; PLL-derived system-clock estimate; clock-change callback to timers; CICR clearing. Representative HSE/PLL startup is unit covered. | Oscillator startup delays, failures beyond absent HSE, prescaler trees, peripheral clock gates/resets, clock security, and exact PLL constraints are not modeled. MSI is a fixed 4 MHz estimate. |
| FLASH | Main and option lock reset state, documented key sequences, locked-CR protection, W1C status behavior, and permanently clear BUSY. Key unlock is unit covered. | No erase/program operation, latency effect, option-byte reload, ECC, bank behavior, or flash timing. ACR is retained register state. |
| PWR | Register storage and always-complete voltage-scaling status. | No voltage domains, stop/standby transitions, wakeup pins, backup-domain rules, or supply effects. |
| GPIO | MODER/ODR/IDR storage, BSRR/BRR atomic updates, external input overrides, output mirroring, timestamped transitions and callbacks. Unit and integrated routing covered. | No pull resistors, open-drain/electrical levels, slew, contention, alternate-function routing, EXTI edge generation, or clock gating. |
| USART | Scripted/provider RX queue, RDR pop, TDR TX log/callback, binary board `tx_log` output, RXNE/IDLE/TC/TXE and TEACK/REACK flags, selected CR1 interrupts, ICR/RQR clearing. Unit covered. | TX is instantaneous; no baud timing, framing, parity, oversampling, FIFO depth, errors, CTS/RTS, synchronous modes, or DMA wiring. |
| TIM | CR1 enable/one-pulse, PSC, ARR, CNT, EGR update, UIF/UIE, live counter from simulated time, update events/IRQs, RCC clock-change input. Exact-period update is unit covered. | One generic up-counter model is reused for all listed timers. Capture/compare, PWM, repetition/dead time, encoder/slave modes, complementary outputs, timer chaining, and most status bits are absent. |
| ADC | Immediate calibration/enable readiness; regular sequences of up to 16 SQR ranks; per-channel SMPR timing derived from the active system clock and selected resolution; DR/EOC/EOS; single and continuous modes; interrupts; and constant or simulated-time sine providers sampled at conversion completion. ADC1-4 conversion requests route through DMAMUX to DMA. ADC12 and ADC345 common windows remain lenient sparse storage. Sequence order and clock-derived timing are unit covered. | Channels 0-19 only; no injected sequences, external trigger routing, alignment/oversampling effects, calibration data, analog watchdog behavior, or functional common-block clock semantics. The current ADC clock follows the system clock, matching the PER firmware's synchronous-clock setup. |
| SPI | TDR/DR-style immediate 1- or 2-byte transfer, RXNE/TXE/BSY, interrupt and DMA-request callbacks, zero/echo or custom response, stable trace. Echo path is unit covered. | No serial clock or chip-select timing, frame formats beyond access width, FIFO depth, CRC, underrun/overrun, I2S, or integrated SPI-to-DMA request routing. |
| DMA | DMA1/2 channel register layout, one item per peripheral request, 1/2/4-byte widths, directions, address increment, circular count reload, TC/TE flags and completion interrupts, and MemoryBus copies. Unit covered. | No arbitration or transfer bandwidth, half-transfer interrupts, bursts, DMA request generators, or USART/SPI handshake routing. Memory-to-memory mode completes synchronously. |
| DMAMUX | Per-channel 7-bit request selection, overrun-flag clearing, and ADC1-4 request routing to DMA1/2 channels. Unit covered through ADC/DMA acceptance behavior. | Synchronization and request generators are absent; USART and SPI requests are not yet integrated. |
| IWDG | Key unlock/start/reload, prescaler/reload storage, 32 kHz-derived timeout and reset request callback. Timeout timing is unit covered. | Status/window details are minimal. An integrated timeout stops the board with `reset-requested`; it does not automatically reset and restart the emulated MCU. |
| WWDG | Enable/reload, a PCLK-derived timeout, and integrated reset request callback are implemented. | No dedicated unit coverage for the integrated WWDG path, incomplete refresh-window/early-wakeup rules, and no automatic reset/restart after the request. |
| FDCAN | INIT/CCE/clock-stop transitions; fixed PER message-RAM layout; standard/extended range, dual-ID, and mask filters; nonmatching policy; three-element RX FIFO 0 with acknowledge/overwrite/lost state; three TX elements; classic CAN/CAN-FD payloads through 64 bytes; IR W1C, IE/ILS/ILE line gating, RX and TX-complete interrupts. Dedicated unit tests cover control, filters, TX/RX RAM, FIFO state, and both interrupt lines. | Only RX FIFO 0 storage is functional. No arbitration or wire time, ACK/retry, bus-off/error counters, remote frames, timestamp clock, TX event FIFO, RX FIFO 1, high-priority messages, protocol timing, or full M_CAN configuration rules. Detached TX completes locally. |
| SYSCFG / EXTI | Reads default to zero and sparse writes are preserved in dedicated lenient stubs. | No SYSCFG remap/compensation behavior, EXTI routing, edge detection, pending IRQ generation, or strict per-register validation. |

## Cortex-M system window

The separate `0xe0000000..0xe00fffff` system device implements:

- SysTick CTRL/LOAD/VAL/COUNTFLAG and deterministic interrupt pending;
- NVIC enable, pending, active, software-trigger, and 240 priority bytes with four implemented priority bits;
- SCB CPUID, ICSR, VTOR, keyed AIRCR reset request, SCR, CCR, SHPR, SHCSR, MMFAR/BFAR storage, and CFSR/HFSR write-one-to-clear views;
- CPACR FPU-enable visibility, DEMCR, FPCCR, DWT CTRL, and gated DWT CYCCNT;
- priority selection under PRIMASK, BASEPRI, FAULTMASK, and current active exception.

BASEPRI reads, writes, and arbitration use the same four implemented priority bits
as NVIC/SHPR. Peripheral IRQ inputs are level-sensitive: an uncleared enabled source
re-pends on exception return, and shared sources remain asserted until all are
acknowledged or disabled. Software-pended interrupts remain independently latched.

Unit tests cover SysTick wrap, COUNTFLAG, NVIC enable/masking/priority, BASEPRI_MAX,
PendSV, CPACR, AIRCR, basic/extended exception entry and return, preservation of
re-pended nested interrupts, peripheral IRQ acknowledgment, shared IRQs, and reset/teardown. MPU, ITM/SWO, breakpoint/watchpoint comparators, debug transport, lazy FP stacking, and automatic fault escalation are not modeled.

## Virtual CAN and multi-board world

`run-network` creates named in-process buses, attaches configured FDCAN instances, and dispatches equal-time boards in configuration order; the instruction quantum caps only same-timestamp fairness bursts. CAN delivery is synchronous in node attachment order; loopback is optional per node. Frame validation covers 11- and 29-bit identifiers, classic CAN limits, CAN-FD DLC mapping, and BRS validity. Trace records include normalized transmit and receive frames. Their `dlc` field is the encoded CAN DLC and `length` is the decoded payload byte count; device-facing sources use `board.device`, while bus-facing sources use `bus/node`.

The CLI option `--inject-can BUS[@TIME_MS]:ID:HEXDATA` schedules an external frame
on the shared event loop and may be repeated. A missing `@TIME_MS` means time zero.

The configured bitrate is informational. The bus does not model arbitration priority, simultaneous transmit, serialization delay, physical errors, termination, or load.

## Interpreting unknown MMIO

The `unknown_mmio_addresses` counter records only addresses that miss every
top-level routed block. It does not count generic storage inside a modeled block or
accesses absorbed by the ADC-common, SYSCFG, and EXTI sparse stubs. The tables above,
not a zero unknown-address count, define peripheral fidelity. Strict MMIO remains
useful for finding entirely unrouted ranges.

Representative tests are in `tests/unit/peripheral_test.cpp`,
`tests/unit/fdcan_test.cpp`, `tests/unit/stm32g4_test.cpp`,
`tests/unit/cortexm_test.cpp`, `tests/unit/exceptions_test.cpp`,
`tests/unit/can_bus_test.cpp`, and `tests/unit/world_test.cpp`. External firmware
validation is described in [Testing](testing.md).
