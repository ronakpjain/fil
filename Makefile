# Hand-maintained developer shortcuts; CMake remains the canonical build system.
ROOT_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

# Relative build paths are rooted at this Makefile so targets also work from elsewhere.
root_path = $(if $(filter /%,$(1)),$(abspath $(1)),$(abspath $(ROOT_DIR)/$(1)))

CMAKE ?= cmake
CTEST ?= ctest
PYTHON ?= python3
BUILD_DIR ?= build
BUILD_PATH = $(call root_path,$(BUILD_DIR))
BUILD_TYPE ?= Release
GENERATOR ?=
JOBS ?= $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

TESTS ?= ON
DOCS ?= ON
IPO ?= ON
ASAN ?= OFF
UBSAN ?= OFF
PGO_GENERATE ?= OFF
FIL_PGO_PROFILE ?=
PER_FIRMWARE_DIR ?=
CMAKE_ARGS ?=
BUILD_ARGS ?=
CTEST_ARGS ?=

ifeq ($(strip $(GENERATOR)),)
GENERATOR_ARGS :=
else
GENERATOR_ARGS := -G "$(GENERATOR)"
endif

ifneq ($(strip $(PER_FIRMWARE_DIR)),)
PER_FIRMWARE_ARG := -DFIL_PER_FIRMWARE_DIR:PATH="$(PER_FIRMWARE_DIR)"
else
PER_FIRMWARE_ARG :=
endif

DEBUG_BUILD_DIR ?= build-debug
RELEASE_BUILD_DIR ?= build-release
SANITIZE_BUILD_DIR ?= build-sanitize
PGO_GENERATE_BUILD_DIR ?= build-pgo-generate
PGO_BUILD_DIR ?= build-pgo
PGO_GENERATE_PATH = $(call root_path,$(PGO_GENERATE_BUILD_DIR))
PGO_PATH = $(call root_path,$(PGO_BUILD_DIR))
PGO_PROFILE ?= $(PGO_GENERATE_BUILD_DIR)/fil.profdata
PGO_PROFILE_PATH = $(call root_path,$(PGO_PROFILE))
PGO_PROFILE_PATTERN ?= $(PGO_GENERATE_BUILD_DIR)/fil-%p.profraw
# Representative default; override for a local firmware checkout or workload.
PGO_TRAIN_ARGS ?= run-network configs/networks/per_vehicle.json --duration-ms 1000 --max-instructions 50000000 --quantum 1024 --strict-mmio
PGO_CXX ?= clang++

ifeq ($(strip $(LLVM_PROFDATA)),)
ifneq ($(shell command -v llvm-profdata 2>/dev/null),)
LLVM_PROFDATA := llvm-profdata
else ifeq ($(shell uname -s 2>/dev/null),Darwin)
LLVM_PROFDATA := xcrun llvm-profdata
else
LLVM_PROFDATA := llvm-profdata
endif
endif

FIL_PATH = $(BUILD_PATH)/fil
BENCH_REPS ?= 3
BENCH_ARGS ?=

# CLI convenience defaults. Override these for a local firmware checkout/workload.
CLI_ARGS ?= --help
CONFIG ?= configs/mcus/stm32g474retx.json
ELF ?= tests/fixtures/elf/split_image.elf
RUN_CONFIG ?= tests/fixtures/config/hardware_compare_board.json
RUN_ARGS ?= --allow-breakpoint
NETWORK_CONFIG ?= configs/networks/per_vehicle.json
NETWORK_ARGS ?= --duration-ms 1000 --quantum 1024
WATCH_ARGS ?= --refresh-ms 10 --live-filter can_tx --live-filter can_rx --control-stdin
DISASM_ADDR ?= 0x08000000
DISASM_COUNT ?= 8
COMPARE_CONFIG ?= tests/fixtures/config/hardware_compare_board.json
CONFIRM_STLINK ?= NO

.DEFAULT_GOAL := all
.PHONY: all configure build clean clean-all help fil cli app-help version \
        run run-network watch-network inspect-config inspect-elf disasm-window \
        compare-stlink test check test-list test-per test-debug test-sanitize \
        release debug sanitize benchmark docs firmware-tests fixtures \
        pgo-generate pgo-train pgo-merge pgo-use pgo-test pgo clean-pgo

all: build

configure:
	$(CMAKE) -S "$(ROOT_DIR)" -B "$(BUILD_PATH)" $(GENERATOR_ARGS) \
		-DCMAKE_BUILD_TYPE="$(BUILD_TYPE)" \
		-DFIL_BUILD_TESTS="$(TESTS)" \
		-DFIL_BUILD_DOCS="$(DOCS)" \
		-DFIL_ENABLE_IPO="$(IPO)" \
		-DFIL_ENABLE_ASAN="$(ASAN)" \
		-DFIL_ENABLE_UBSAN="$(UBSAN)" \
		-DFIL_PGO_GENERATE="$(PGO_GENERATE)" \
		-DFIL_PGO_PROFILE:FILEPATH="$(FIL_PGO_PROFILE)" \
		$(PER_FIRMWARE_ARG) \
		$(CMAKE_ARGS)

build: configure
	$(CMAKE) --build "$(BUILD_PATH)" --parallel "$(JOBS)" --config "$(BUILD_TYPE)" $(BUILD_ARGS)

# Remove only a root-level build* directory configured by this project; refuse source/user data.
clean:
	@set -eu; \
	build_dir="$(BUILD_PATH)"; repo_dir="$(ROOT_DIR)"; \
	if [ -L "$$build_dir" ]; then \
		echo "Refusing to remove symlinked build directory: $$build_dir" >&2; exit 2; \
	fi; \
	if [ ! -e "$$build_dir" ]; then exit 0; fi; \
	parent="$$(dirname "$$build_dir")"; \
	parent_real="$$(CDPATH= cd "$$parent" && pwd -P)"; \
	build_name="$$(basename "$$build_dir")"; \
	build_real="$$parent_real/$$build_name"; \
	repo_real="$$(CDPATH= cd "$$repo_dir" && pwd -P)"; \
	if [ "$$parent_real" != "$$repo_real" ]; then \
		echo "Refusing to clean outside a root-level build directory: $$build_real" >&2; exit 2; \
	fi; \
	case "$$build_name" in build|build-*) ;; \
		*) echo "Refusing to clean a non-build directory: $$build_real" >&2; exit 2 ;; \
	esac; \
	if [ ! -f "$$build_real/CMakeCache.txt" ] || \
		! grep -Fqx "CMAKE_HOME_DIRECTORY:INTERNAL=$$repo_real" "$$build_real/CMakeCache.txt"; then \
		echo "Refusing to clean a directory not configured by this CMake project: $$build_real" >&2; exit 2; \
	fi; \
	$(CMAKE) -E rm -rf "$$build_real"

clean-all:
	$(MAKE) clean BUILD_DIR="$(BUILD_DIR)"
	$(MAKE) clean BUILD_DIR="$(RELEASE_BUILD_DIR)"
	$(MAKE) clean BUILD_DIR="$(DEBUG_BUILD_DIR)"
	$(MAKE) clean BUILD_DIR="$(SANITIZE_BUILD_DIR)"
	$(MAKE) clean BUILD_DIR="$(PGO_GENERATE_BUILD_DIR)"
	$(MAKE) clean BUILD_DIR="$(PGO_BUILD_DIR)"

# `make fil` builds the executable; `make cli CLI_ARGS='...'` runs arbitrary fil arguments.
fil: build

cli: build
	"$(FIL_PATH)" $(CLI_ARGS)

app-help: build
	"$(FIL_PATH)" --help

version: build
	"$(FIL_PATH)" --version

run: build
	"$(FIL_PATH)" run "$(RUN_CONFIG)" $(RUN_ARGS)

run-network: build
	"$(FIL_PATH)" run-network "$(NETWORK_CONFIG)" $(NETWORK_ARGS)

watch-network: build
	"$(FIL_PATH)" watch-network "$(NETWORK_CONFIG)" $(WATCH_ARGS)

inspect-config: build
	"$(FIL_PATH)" inspect-config "$(CONFIG)"

inspect-elf: build
	"$(FIL_PATH)" inspect-elf "$(ELF)"

disasm-window: build
	"$(FIL_PATH)" disasm-window "$(ELF)" --addr "$(DISASM_ADDR)" --count "$(DISASM_COUNT)"

# This target controls/resets a connected STM32; flashing is never enabled implicitly.
compare-stlink:
	@test "$(CONFIRM_STLINK)" = YES || { \
		echo "This controls/resets a physical STM32. Rerun with CONFIRM_STLINK=YES after checking the target." >&2; exit 2; }
	$(MAKE) build
	"$(FIL_PATH)" compare-stlink "$(COMPARE_CONFIG)" $(COMPARE_ARGS)

test: build
	$(CTEST) --test-dir "$(BUILD_PATH)" --output-on-failure -C "$(BUILD_TYPE)" $(CTEST_ARGS)

check: test

test-list: configure
	$(CTEST) --test-dir "$(BUILD_PATH)" -C "$(BUILD_TYPE)" -N

test-per:
	@test -n "$(strip $(PER_FIRMWARE_DIR))" || { \
		echo "Set PER_FIRMWARE_DIR to the external PER firmware output directory." >&2; exit 2; }
	$(MAKE) build
	$(CTEST) --test-dir "$(BUILD_PATH)" -L per --output-on-failure -C "$(BUILD_TYPE)" $(CTEST_ARGS)

release:
	$(MAKE) BUILD_DIR="$(RELEASE_BUILD_DIR)" BUILD_TYPE=Release build

debug:
	$(MAKE) BUILD_DIR="$(DEBUG_BUILD_DIR)" BUILD_TYPE=Debug IPO=OFF ASAN=OFF UBSAN=OFF build

test-debug:
	$(MAKE) BUILD_DIR="$(DEBUG_BUILD_DIR)" BUILD_TYPE=Debug IPO=OFF ASAN=OFF UBSAN=OFF test

sanitize:
	$(MAKE) BUILD_DIR="$(SANITIZE_BUILD_DIR)" BUILD_TYPE=Debug IPO=OFF ASAN=ON UBSAN=ON build

test-sanitize:
	$(MAKE) BUILD_DIR="$(SANITIZE_BUILD_DIR)" BUILD_TYPE=Debug IPO=OFF ASAN=ON UBSAN=ON test

benchmark: build
	$(PYTHON) "$(ROOT_DIR)/tools/bench_real_timing.py" "$(FIL_PATH)" --reps "$(BENCH_REPS)" $(BENCH_ARGS)

docs: configure
	$(CMAKE) --build "$(BUILD_PATH)" --target docs --config "$(BUILD_TYPE)"

firmware-tests: configure
	@command -v arm-none-eabi-gcc >/dev/null 2>&1 || { \
		echo "arm-none-eabi-gcc is required to regenerate committed firmware fixtures." >&2; exit 2; }
	$(CMAKE) --build "$(BUILD_PATH)" --target firmware_tests --config "$(BUILD_TYPE)"

fixtures: firmware-tests

# PGO is Clang-only. Start fresh with `make clean-pgo`, then train a representative workload.
pgo-generate:
	@CXX="$(PGO_CXX)" $(MAKE) BUILD_DIR="$(PGO_GENERATE_BUILD_DIR)" BUILD_TYPE=Release \
		IPO=OFF TESTS=OFF DOCS=OFF ASAN=OFF UBSAN=OFF PGO_GENERATE=ON FIL_PGO_PROFILE= build

pgo-train:
	@test -n "$(strip $(PGO_TRAIN_ARGS))" || { \
		echo "Set PGO_TRAIN_ARGS to representative fil arguments (for example: run-network ...)." >&2; exit 2; }
	$(MAKE) pgo-generate
	@mkdir -p "$(PGO_GENERATE_PATH)"
	LLVM_PROFILE_FILE="$(call root_path,$(PGO_PROFILE_PATTERN))" \
		"$(PGO_GENERATE_PATH)/fil" $(PGO_TRAIN_ARGS)

pgo-merge:
	@set -eu; \
	compiler_major="$$( $(PGO_CXX) --version 2>/dev/null | grep -Eo '[0-9]+[.][0-9]+' | head -1 | cut -d . -f 1 )"; \
	profdata_major="$$( $(LLVM_PROFDATA) --version 2>/dev/null | grep -Eo '[0-9]+[.][0-9]+' | head -1 | cut -d . -f 1 )"; \
	if [ -n "$$compiler_major" ] && [ -n "$$profdata_major" ] && [ "$$compiler_major" != "$$profdata_major" ]; then \
		echo "PGO compiler LLVM $$compiler_major does not match llvm-profdata $$profdata_major; set PGO_CXX and LLVM_PROFDATA from the same LLVM release." >&2; exit 2; \
	fi; \
	set -- "$(PGO_GENERATE_PATH)"/fil-*.profraw; \
	if [ ! -f "$$1" ]; then \
		echo "No PGO .profraw files found in $(PGO_GENERATE_PATH); run make pgo-train first." >&2; exit 2; \
	fi; \
	mkdir -p "$(dir $(PGO_PROFILE_PATH))"; \
	$(LLVM_PROFDATA) merge -sparse "$$@" -o "$(PGO_PROFILE_PATH)"

pgo-use: pgo-merge
	$(MAKE) clean BUILD_DIR="$(PGO_BUILD_DIR)"
	@CXX="$(PGO_CXX)" $(MAKE) BUILD_DIR="$(PGO_BUILD_DIR)" BUILD_TYPE=Release \
		IPO=ON TESTS=ON PGO_GENERATE=OFF FIL_PGO_PROFILE="$(PGO_PROFILE_PATH)" build

pgo-test: pgo-use
	$(CTEST) --test-dir "$(PGO_PATH)" --output-on-failure -C Release $(CTEST_ARGS)

# Run a complete PGO cycle; validate the workload before removing prior PGO outputs.
pgo:
	@test -n "$(strip $(PGO_TRAIN_ARGS))" || { \
		echo "Set PGO_TRAIN_ARGS to representative fil arguments (for example: run-network ...)." >&2; exit 2; }
	$(MAKE) clean-pgo
	$(MAKE) pgo-train
	$(MAKE) pgo-test

clean-pgo:
	$(MAKE) clean BUILD_DIR="$(PGO_GENERATE_BUILD_DIR)"
	$(MAKE) clean BUILD_DIR="$(PGO_BUILD_DIR)"

help:
	@printf '%s\n' \
	  'Build:          make [all] | configure | build | release | debug | sanitize' \
	  'Tests:          make test | test-list | test-per PER_FIRMWARE_DIR=/path/to/output' \
	  '                make test-debug | test-sanitize' \
	  'App:            make cli CLI_ARGS="--help" | app-help | version' \
	  'Commands:       make run [RUN_CONFIG=...] [RUN_ARGS="..."]' \
	  '                make run-network [NETWORK_CONFIG=...] [NETWORK_ARGS="..."]' \
	  '                make watch-network [NETWORK_CONFIG=...] [WATCH_ARGS="..."]' \
	  '                make inspect-config [CONFIG=...] | inspect-elf [ELF=...]' \
	  '                make disasm-window [ELF=...] [DISASM_ADDR=...] [DISASM_COUNT=...]' \
	  '                make compare-stlink CONFIRM_STLINK=YES [COMPARE_CONFIG=...] [COMPARE_ARGS="..."]' \
	  '                make docs | firmware-tests (rewrites fixture ELFs) | benchmark [BENCH_REPS=3]' \
	  'PGO:            make pgo (full cycle; default: six-board PER workload)' \
	  '                Override workload with PGO_TRAIN_ARGS="run-network ..."' \
	  '                Stages: pgo-generate | pgo-train | pgo-merge | pgo-use | pgo-test' \
	  '                Set PGO_CXX/LLVM_PROFDATA to matching LLVM versions' \
	  'Cleanup:        make clean [BUILD_DIR=...] | clean-pgo | clean-all' \
	  'Options:        BUILD_DIR=build GENERATOR=Ninja JOBS=8 BUILD_TYPE=Release'
