find_package(GTest CONFIG REQUIRED)

add_executable(fil_tests
    unit/alu_test.cpp
    unit/board_test.cpp
    unit/can_bus_test.cpp
    unit/can_end_to_end_test.cpp
    unit/config_test.cpp
    unit/cortexm_test.cpp
    unit/cpu_step_test.cpp
    unit/decoder_test.cpp
    unit/elf_loader_test.cpp
    unit/event_loop_test.cpp
    unit/exceptions_test.cpp
    unit/fdcan_test.cpp
    unit/crc_test.cpp
    unit/flash_test.cpp
    unit/hard_float_firmware_test.cpp
    unit/hardware_comparison_test.cpp
    unit/memory_bus_test.cpp
    unit/peripheral_test.cpp
    unit/scheduler_tick_test.cpp
    unit/smoke_test.cpp
    unit/startup_runtime_test.cpp
    unit/stm32g4_test.cpp
    unit/world_test.cpp
    unit/world_time_test.cpp
)
target_include_directories(fil_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(fil_tests PRIVATE fil_core GTest::gtest_main)
target_compile_definitions(fil_tests PRIVATE FIL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
fil_set_project_options(fil_tests)

include(GoogleTest)
gtest_discover_tests(fil_tests TEST_PREFIX "fil.")

add_test(NAME fil.cli_help COMMAND fil --help)
set_tests_properties(fil.cli_help PROPERTIES
    PASS_REGULAR_EXPRESSION "deterministic STM32G4 firmware emulator"
)
