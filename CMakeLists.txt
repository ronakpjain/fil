cmake_minimum_required(VERSION 3.20)

project(fil VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The emulator's normal mode is a performance-sensitive interpreter. Keep
# explicit Debug/sanitizer builds available, but do not silently produce an
# unoptimized executable when a single-config generator is used without a
# build type.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

option(FIL_BUILD_TESTS "Build fil tests" ON)
option(FIL_BUILD_DOCS "Add the Doxygen HTML documentation target" ON)
option(FIL_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(FIL_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(FIL_ENABLE_IPO "Enable supported interprocedural optimization in non-sanitized builds" ON)
option(FIL_ENABLE_LLVM_JIT "Build the optional LLVM ORC native block backend" OFF)
option(FIL_PGO_GENERATE "Instrument a Clang build to generate a PGO profile" OFF)
set(FIL_PGO_PROFILE "" CACHE FILEPATH "Clang PGO profile (.profdata) to use when optimizing fil")
set(FIL_PER_FIRMWARE_DIR "" CACHE PATH "Optional PER firmware output directory for compatibility tests")

if(FIL_PGO_GENERATE AND FIL_PGO_PROFILE)
    message(FATAL_ERROR "FIL_PGO_GENERATE and FIL_PGO_PROFILE are mutually exclusive")
endif()
if((FIL_PGO_GENERATE OR FIL_PGO_PROFILE) AND (FIL_ENABLE_ASAN OR FIL_ENABLE_UBSAN))
    message(FATAL_ERROR "PGO cannot be combined with the configured sanitizers")
endif()
if((FIL_PGO_GENERATE OR FIL_PGO_PROFILE)
    AND NOT CMAKE_CXX_COMPILER_ID MATCHES "^(AppleClang|Clang)$")
    message(FATAL_ERROR "FIL PGO currently requires Clang or AppleClang")
endif()
if(FIL_PGO_PROFILE AND NOT EXISTS "${FIL_PGO_PROFILE}")
    message(FATAL_ERROR "FIL_PGO_PROFILE does not exist: ${FIL_PGO_PROFILE}")
endif()

set(FIL_IPO_SUPPORTED OFF)
if(FIL_ENABLE_IPO AND NOT FIL_ENABLE_ASAN AND NOT FIL_ENABLE_UBSAN)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT FIL_IPO_SUPPORTED OUTPUT FIL_IPO_ERROR LANGUAGES CXX)
    if(NOT FIL_IPO_SUPPORTED)
        message(WARNING "Interprocedural optimization is unavailable: ${FIL_IPO_ERROR}")
    endif()
endif()

function(fil_set_project_options target)
    target_compile_features(${target} PUBLIC cxx_std_20)
    set_target_properties(${target} PROPERTIES
        CXX_EXTENSIONS OFF
    )
    if(FIL_IPO_SUPPORTED)
        set_property(TARGET ${target} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
        set_property(TARGET ${target} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO TRUE)
        set_property(TARGET ${target} PROPERTY INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL TRUE)
    endif()

    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${target} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wsign-conversion
        )
    endif()

    if(FIL_ENABLE_ASAN OR FIL_ENABLE_UBSAN)
        if(MSVC)
            message(FATAL_ERROR "FIL_ENABLE_ASAN and FIL_ENABLE_UBSAN are not configured for MSVC")
        endif()

        set(sanitizers "")
        if(FIL_ENABLE_ASAN)
            list(APPEND sanitizers "address")
        endif()
        if(FIL_ENABLE_UBSAN)
            list(APPEND sanitizers "undefined")
        endif()
        list(JOIN sanitizers "," sanitizer_list)

        target_compile_options(${target} PRIVATE
            -fno-omit-frame-pointer
            "-fsanitize=${sanitizer_list}"
        )
        target_link_options(${target} PRIVATE
            "-fsanitize=${sanitizer_list}"
        )
    endif()
endfunction()

add_library(fil_core STATIC
    src/cli/cli.cpp
    src/common/error.cpp
    src/common/log.cpp
    src/config/config.cpp
    src/cpu/cortex_m4.cpp
    src/cpu/decoder.cpp
    src/cpu/execute.cpp
    src/cpu/jit.cpp
    src/cortexm/exceptions.cpp
    src/cortexm/system_control.cpp
    src/devices/can_bus.cpp
    src/elf/elf_loader.cpp
    src/mem/address.cpp
    src/mem/mcu_map.cpp
    src/mem/memory_bus.cpp
    src/mem/mmio_router.cpp
    src/sim/event_loop.cpp
    src/sim/board.cpp
    src/sim/trace.cpp
    src/sim/worker_pool.cpp
    src/sim/world.cpp
    src/stm32g4/adc.cpp
    src/stm32g4/dma.cpp
    src/stm32g4/flash.cpp
    src/stm32g4/fdcan.cpp
    src/stm32g4/gpio.cpp
    src/stm32g4/peripheral.cpp
    src/stm32g4/pwr.cpp
    src/stm32g4/rcc.cpp
    src/stm32g4/spi.cpp
    src/stm32g4/stm32g4.cpp
    src/stm32g4/tim.cpp
    src/stm32g4/usart.cpp
    src/stm32g4/watchdog.cpp
)
target_include_directories(fil_core PUBLIC include)
fil_set_project_options(fil_core)

if(FIL_ENABLE_LLVM_JIT)
    find_package(LLVM CONFIG REQUIRED)
    target_sources(fil_core PRIVATE src/cpu/jit_llvm.cpp)
    target_include_directories(fil_core SYSTEM PRIVATE ${LLVM_INCLUDE_DIRS})
    target_compile_definitions(fil_core PUBLIC FIL_HAS_LLVM_JIT=1)
    llvm_map_components_to_libnames(FIL_LLVM_LIBS
        Core
        OrcJIT
        ExecutionEngine
        Support
        nativecodegen
    )
    target_link_libraries(fil_core PRIVATE ${FIL_LLVM_LIBS})
endif()

add_executable(fil src/main.cpp)
target_link_libraries(fil PRIVATE fil_core)
fil_set_project_options(fil)

if(FIL_PGO_GENERATE)
    target_compile_options(fil_core PRIVATE -fprofile-instr-generate)
    target_compile_options(fil PRIVATE -fprofile-instr-generate)
    target_link_options(fil PRIVATE -fprofile-instr-generate)
elseif(FIL_PGO_PROFILE)
    target_compile_options(fil_core PRIVATE "-fprofile-instr-use=${FIL_PGO_PROFILE}")
    target_compile_options(fil PRIVATE "-fprofile-instr-use=${FIL_PGO_PROFILE}")
    target_link_options(fil PRIVATE "-fprofile-instr-use=${FIL_PGO_PROFILE}")
endif()

find_program(FIL_ARM_NONE_EABI_GCC arm-none-eabi-gcc)
if(FIL_ARM_NONE_EABI_GCC)
    add_custom_target(firmware_tests
        COMMAND ${CMAKE_COMMAND} -E env
            ARM_NONE_EABI_GCC=${FIL_ARM_NONE_EABI_GCC}
            ${PROJECT_SOURCE_DIR}/tools/build_test_firmware.sh
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMENT "Regenerating committed Cortex-M firmware fixtures"
        VERBATIM
    )
endif()

if(FIL_BUILD_DOCS)
    find_package(Doxygen QUIET)
    if(Doxygen_FOUND)
        add_custom_target(docs
            COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_SOURCE_DIR}/Doxyfile
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            COMMENT "Generating light-themed HTML documentation"
            VERBATIM
        )
    else()
        message(STATUS "Doxygen not found; the 'docs' target is unavailable")
    endif()
endif()

if(FIL_BUILD_TESTS)
    include(CTest)
    enable_testing()
    add_subdirectory(tests)
    if(FIL_PER_FIRMWARE_DIR)
        set(FIL_PER_GENERATED_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/per-smoke-configs")
        file(MAKE_DIRECTORY "${FIL_PER_GENERATED_CONFIG_DIR}")
        set(FIL_PER_MCU_CONFIG "${PROJECT_SOURCE_DIR}/configs/mcus/stm32g474retx.json")
        set(FIL_PER_NETWORK_BOARDS "")
        set(FIL_PER_NETWORK_COMPLETE TRUE)
        set(FIL_PER_AUDIT_ELFS "")

        foreach(per_board IN ITEMS
            g4_testing dashboard main_module torque_vector a_box front_driveline rear_driveline
        )
            set(per_elf "${FIL_PER_FIRMWARE_DIR}/${per_board}/${per_board}.elf")
            if(EXISTS "${per_elf}")
                list(APPEND FIL_PER_AUDIT_ELFS "${per_elf}")
                add_test(NAME "fil.per.inspect_${per_board}" COMMAND fil inspect-elf "${per_elf}")

                file(READ "${PROJECT_SOURCE_DIR}/configs/boards/${per_board}.json" per_config_json)
                string(REGEX REPLACE
                    "\"mcu\"[ \t]*:[ \t]*\"[^\"]*\""
                    "\"mcu\": \"${FIL_PER_MCU_CONFIG}\""
                    per_config_json "${per_config_json}"
                )
                string(REGEX REPLACE
                    "\"elf\"[ \t]*:[ \t]*\"[^\"]*\""
                    "\"elf\": \"${per_elf}\""
                    per_config_json "${per_config_json}"
                )
                set(per_generated_config "${FIL_PER_GENERATED_CONFIG_DIR}/${per_board}.json")
                file(WRITE "${per_generated_config}" "${per_config_json}")

                add_test(
                    NAME "fil.per.run_${per_board}_10ms"
                    COMMAND fil run "${per_generated_config}"
                        --duration-ms 10
                        --max-instructions 1000000
                        --strict-mmio
                        --no-detect-spin
                )
                set_tests_properties("fil.per.run_${per_board}_10ms" PROPERTIES
                    PASS_REGULAR_EXPRESSION "unknown_mmio_addresses: 0"
                    TIMEOUT 30
                    LABELS "per;integration"
                )

                if(per_board STREQUAL "g4_testing")
                    add_test(
                        NAME fil.per.run_g4_testing_1s
                        COMMAND ${CMAKE_COMMAND}
                            "-DFIL_EXECUTABLE=$<TARGET_FILE:fil>"
                            "-DBOARD_CONFIG=${per_generated_config}"
                            -P "${PROJECT_SOURCE_DIR}/tests/cmake/run_per_g4_smoke.cmake"
                    )
                    set_tests_properties(fil.per.run_g4_testing_1s PROPERTIES
                        TIMEOUT 120
                        LABELS "per;integration;long"
                    )
                endif()

                if(NOT per_board STREQUAL "g4_testing")
                    list(APPEND FIL_PER_NETWORK_BOARDS "${per_generated_config}")
                endif()
            elseif(NOT per_board STREQUAL "g4_testing")
                set(FIL_PER_NETWORK_COMPLETE FALSE)
            endif()
        endforeach()

        list(LENGTH FIL_PER_AUDIT_ELFS FIL_PER_AUDIT_ELF_COUNT)
        if(FIL_PER_AUDIT_ELF_COUNT EQUAL 7)
            find_package(Python3 3.9 QUIET COMPONENTS Interpreter)
            find_program(FIL_ARM_NONE_EABI_OBJDUMP NAMES arm-none-eabi-objdump)
            if(Python3_Interpreter_FOUND AND FIL_ARM_NONE_EABI_OBJDUMP)
                add_test(
                    NAME fil.per.audit_instructions
                    COMMAND "${Python3_EXECUTABLE}"
                        "${PROJECT_SOURCE_DIR}/tools/audit_per_instructions.py"
                        --fil "$<TARGET_FILE:fil>"
                        --objdump "${FIL_ARM_NONE_EABI_OBJDUMP}"
                        --report "${CMAKE_CURRENT_BINARY_DIR}/per-instruction-audit.json"
                        ${FIL_PER_AUDIT_ELFS}
                )
                set_tests_properties(fil.per.audit_instructions PROPERTIES
                    TIMEOUT 120
                    LABELS "per;integration;static"
                )
            else()
                message(STATUS
                    "Python 3.9+ or arm-none-eabi-objdump unavailable; "
                    "the opt-in PER instruction audit test will not be added"
                )
            endif()
        endif()

        if(FIL_PER_NETWORK_COMPLETE)
            set(per_network_json "{\n  \"schema_version\": 1,\n  \"name\": \"per_vehicle_smoke\",\n")
            string(APPEND per_network_json
                "  \"buses\": {\"vehicle\": {\"type\": \"can\", \"bitrate\": 500000}},\n"
            )
            string(APPEND per_network_json "  \"boards\": [")
            set(per_network_board_index 0)
            foreach(per_board_config IN LISTS FIL_PER_NETWORK_BOARDS)
                if(per_network_board_index GREATER 0)
                    string(APPEND per_network_json ", ")
                endif()
                string(APPEND per_network_json "\"${per_board_config}\"")
                math(EXPR per_network_board_index "${per_network_board_index} + 1")
            endforeach()
            string(APPEND per_network_json "]\n}\n")
            set(per_network_config "${FIL_PER_GENERATED_CONFIG_DIR}/per_vehicle.json")
            file(WRITE "${per_network_config}" "${per_network_json}")
            add_test(
                NAME fil.per.run_network_10ms
                COMMAND ${CMAKE_COMMAND}
                    "-DFIL_EXECUTABLE=$<TARGET_FILE:fil>"
                    "-DNETWORK_CONFIG=${per_network_config}"
                    "-DTRACE_FILE=${CMAKE_CURRENT_BINARY_DIR}/per-network-smoke.jsonl"
                    -P "${PROJECT_SOURCE_DIR}/tests/cmake/run_per_network_smoke.cmake"
            )
            set_tests_properties(fil.per.run_network_10ms PROPERTIES
                TIMEOUT 60
                LABELS "per;integration"
            )
        endif()
    endif()
endif()
