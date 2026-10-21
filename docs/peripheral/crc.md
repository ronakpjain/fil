# CRC

The CRC block implements an MSB-first 32-bit accumulator with configurable polynomial and initial value. Writing INIT reloads the accumulator; CR reset reloads it as well. DR accepts right-aligned byte, halfword, and word inputs, while IDR is independent eight-bit storage.

**Use:** set POL and INIT, then write input data through DR in the intended widths and read the resulting accumulator. This models the non-reflected 32-bit algorithm used by tested firmware paths.

**Limits:** REV_IN/REV_OUT and polynomial sizes other than 32 bits do not implement alternate algorithms. Register access/reserved-bit constraints are partial.

**RM0440:** §16.3.3; §16.4.1 `DR`; §16.4.2 `IDR`; §16.4.3 `CR`; §16.4.4 `INIT`; §16.4.5 `POL`.

**Tests:** `CrcPeripheralTest.MatchesCrc32Mpeg2Reference`; `CrcPeripheralTest.HonorsPolynomialRegister`; `CrcPeripheralTest.SubwordInputsInitReloadAndIdrSemantics`; `CrcIntegrationTest.RoutesThroughStm32G4`.

See the [coverage matrix](../stm32g4_peripheral_coverage.md#crc) for unsupported algorithm fields.
