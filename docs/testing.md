# Testing

The required suite is hermetic: it uses committed synthetic Cortex-M4F fixtures and
does not require the external PER repository or an ARM toolchain. It uses the
installed GoogleTest CMake package; CTest discovers each test case independently.

## Required suite

From the repository root, the hand-maintained Makefile delegates to CMake:

```bash
make
make test
```

The defaults build optimized Release with tests enabled in `build/`. Choose a
specific generator or build directory with `make GENERATOR=Ninja BUILD_DIR=build-ninja`;
`make test` always builds before running CTest. The equivalent direct CMake workflow is:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DFIL_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

The tests cover ELF/config parsing, memory and MMIO faults, Thumb decode/execution,
Cortex-M exceptions, peripheral register behavior, deterministic events and traces,
CAN delivery and stimulus output-window checks, single- and multi-board scheduling, worker rollback, and exact-state
loop batching. Firmware fixtures exercise reset, `.data` copying, `.bss` clearing,
hard-float startup, SysTick scheduling, and deterministic breakpoint boundaries.

## Sanitizers

Use the isolated Debug ASan+UBSan build and test targets:

```bash
make test-sanitize
```

The output is in `build-sanitize/`. To use only one sanitizer or change the
configuration, pass `ASAN=ON UBSAN=OFF` (or the inverse) to the regular targets,
for example `make BUILD_DIR=build-asan BUILD_TYPE=Debug IPO=OFF ASAN=ON test`.
IPO and PGO are disabled for sanitizer builds.

## Source coverage

Clang can produce a local line/branch report without changing project files:

```bash
cmake -S . -B build-coverage -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DFIL_BUILD_TESTS=ON \
  -DFIL_BUILD_DOCS=OFF \
  -DFIL_ENABLE_IPO=OFF \
  -DCMAKE_CXX_FLAGS='-fprofile-instr-generate -fcoverage-mapping' \
  -DCMAKE_EXE_LINKER_FLAGS='-fprofile-instr-generate'
cmake --build build-coverage

rm -f /tmp/fil-*.profraw /tmp/fil.profdata
LLVM_PROFILE_FILE='/tmp/fil-%p.profraw' \
  ctest --test-dir build-coverage --output-on-failure
llvm-profdata merge -sparse /tmp/fil-*.profraw -o /tmp/fil.profdata
llvm-cov report \
  ./build-coverage/tests/fil_tests \
  ./build-coverage/fil \
  -instr-profile=/tmp/fil.profdata \
  -ignore-filename-regex='(/tests/|/usr/|/Library/|/opt/homebrew/)'
```

On macOS, prefix the LLVM tools with `xcrun`. Coverage percentages are diagnostic,
not a release gate; prioritize deterministic behavior and boundary cases over tests
that only execute lines.

## Physical hardware comparison

The hermetic suite exercises the ST-Link orchestration through a fake OpenOCD
process. A real STM32G47x/G48x comparison is always manual because it resets and
controls the attached target and can optionally rewrite flash. The explicit
`make compare-stlink CONFIRM_STLINK=YES` target requires the confirmation variable;
`--flash` is never passed unless explicitly included in `COMPARE_ARGS`. See
[Hardware comparison](hardware_comparison.md) for the probe fixture and command.

## External PER acceptance

External firmware checks are opt-in. Point the Makefile at the directory containing
the seven board output directories (and use a separate build directory):

```bash
make test-per BUILD_DIR=build-per \
  PER_FIRMWARE_DIR=/absolute/path/to/PER/Projects/firmware/output
```

This configures/builds first, then runs CTest's `per` label. CMake registers tests
only for firmware files that exist; the full six-board network test requires all
six non-test board images. `CTEST_ARGS='-LE long'` skips the longer integration case.

CMake adds ELF inspection, strict-MMIO board smoke runs, a one-second `g4_testing`
run, and the six-board CAN network test when the required files exist. Stimulus `expect` entries can assert firmware-originated CAN output within inclusive simulated-time windows. The network
test verifies firmware-originated transmit and receive records in a generated JSONL
trace. Use `-L per -LE long` for the short subset.

When Python 3.9+ and `arm-none-eabi-objdump` are available,
`fil.per.audit_instructions` also compares disassembly boundaries, raw encodings,
widths, and decoder support for every recognized instruction. Its JSON report records
input hashes and tool versions so results remain tied to exact artifacts.

## Regenerating firmware fixtures

The ARM GNU toolchain is needed only when changing fixture sources. The explicit
`make firmware-tests` (alias `make fixtures`) target rewrites committed ELF files;
review and commit those binaries only when their source changes are intentional:

```bash
make firmware-tests
make test
```

Normal test runs use the committed fixtures and remain independent of the
cross-compiler.
