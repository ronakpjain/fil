find_package(GTest CONFIG REQUIRED)

add_executable(fil_tests
    unit/alu_test.cpp
    unit/inline_buffer_test.cpp
    unit/adc_clock_test.cpp
    unit/board_test.cpp
    unit/can_bus_test.cpp
    unit/can_end_to_end_test.cpp
    unit/config_test.cpp
    unit/cortexm_test.cpp
    unit/cpu_step_test.cpp
    unit/decoder_test.cpp
    unit/deferred_prefix_test.cpp
    unit/elf_loader_test.cpp
    unit/event_loop_test.cpp
    unit/effect_summary_test.cpp
    unit/forward_execution_test.cpp
    unit/exceptions_test.cpp
    unit/fdcan_test.cpp
    unit/crc_test.cpp
    unit/flash_test.cpp
    unit/gdb_server_test.cpp
    unit/hard_float_firmware_test.cpp
    unit/hardware_comparison_test.cpp
    unit/live_gdb_test.cpp
    unit/jit_test.cpp
    unit/micro_jit_test.cpp
    unit/micro_jit_ir_test.cpp
    unit/native_execution_test.cpp
    unit/native_jit_test.cpp
    unit/native_region_test.cpp
    unit/memory_bus_test.cpp
    unit/monitor_test.cpp
    unit/network_protocol_test.cpp
    unit/peripheral_test.cpp
    unit/scheduler_tick_test.cpp
    unit/smoke_test.cpp
    unit/startup_runtime_test.cpp
    unit/stimulus_test.cpp
    unit/stm32g4_test.cpp
    unit/timing_test.cpp
    unit/trace_test.cpp
    unit/world_test.cpp
    unit/world_time_test.cpp
)
target_include_directories(fil_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(fil_tests PRIVATE fil_core GTest::gtest_main)
target_compile_definitions(fil_tests PRIVATE FIL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
fil_set_project_options(fil_tests)

# Tests validate fil_core behavior; cross-translation-unit inlining only makes
# their link the slowest step of an incremental build. Keep tests non-LTO.
set_property(TARGET fil_tests PROPERTY
    INTERPROCEDURAL_OPTIMIZATION_RELEASE FALSE
    INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO FALSE
    INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL FALSE
)

if(FIL_USE_PCH)
    target_compile_options(fil_tests PRIVATE ${FIL_PCH_COMPILE_OPTIONS})
    target_precompile_headers(fil_tests PRIVATE <gtest/gtest.h> ${FIL_PCH_HEADERS})
endif()

if(FIL_PGO_GENERATE)
    # Training runs the instrumented test suite so every fil_core translation
    # unit contributes counters, including cold paths a single CLI workload
    # never reaches. The profile runtime must be linked for the instrumented
    # fil_core objects; the test sources themselves stay uninstrumented.
    target_link_options(fil_tests PRIVATE -fprofile-instr-generate)
endif()

include(GoogleTest)
gtest_discover_tests(fil_tests TEST_PREFIX "fil.")

add_test(NAME fil.cli_help COMMAND fil --help)
set_tests_properties(fil.cli_help PROPERTIES
    PASS_REGULAR_EXPRESSION "deterministic STM32G4 firmware emulator"
)
