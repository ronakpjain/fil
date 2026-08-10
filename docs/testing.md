# Testing

The required suite is hermetic: it uses committed synthetic Cortex-M4F fixtures and
does not require the external PER repository or an ARM toolchain. It uses the
installed GoogleTest CMake package; CTest discovers each test case independently.

## Required suite

```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DFIL_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

The tests cover ELF/config parsing, memory and MMIO faults, Thumb decode/execution,
Cortex-M exceptions, peripheral register behavior, deterministic events and traces,
CAN delivery, single- and multi-board scheduling, worker rollback, and exact-state
loop batching. Firmware fixtures exercise reset, `.data` copying, `.bss` clearing,
hard-float startup, SysTick scheduling, and deterministic breakpoint boundaries.

## Sanitizers

Request Debug explicitly for sanitizer builds:

```bash
cmake -S . -B build-sanitize -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DFIL_BUILD_TESTS=ON \
  -DFIL_ENABLE_ASAN=ON \
  -DFIL_ENABLE_UBSAN=ON
cmake --build build-sanitize
ctest --test-dir build-sanitize --output-on-failure
```

ASan and UBSan may be enabled separately when the host toolchain cannot combine
them. IPO and PGO are disabled for sanitizer builds.

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

## External PER acceptance

External firmware checks are opt-in. Point CMake at the directory containing the
seven board output directories:

```bash
cmake -S . -B build-per -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DFIL_BUILD_TESTS=ON \
  -DFIL_PER_FIRMWARE_DIR=/absolute/path/to/PER/Projects/firmware/output
cmake --build build-per
ctest --test-dir build-per -L per --output-on-failure
```

CMake adds ELF inspection, strict-MMIO board smoke runs, a one-second `g4_testing`
run, and the six-board CAN network test when the required files exist. The network
test verifies firmware-originated transmit and receive records in a generated JSONL
trace. Use `-L per -LE long` for the short subset.

When Python 3.9+ and `arm-none-eabi-objdump` are available,
`fil.per.audit_instructions` also compares disassembly boundaries, raw encodings,
widths, and decoder support for every recognized instruction. Its JSON report records
input hashes and tool versions so results remain tied to exact artifacts.

## Regenerating firmware fixtures

The ARM GNU toolchain is needed only when changing fixture sources:

```bash
cmake --build build --target firmware_tests
cmake --build build
ctest --test-dir build --output-on-failure
```

Review binary fixture changes alongside their source and keep normal test runs
independent of the cross-compiler.
