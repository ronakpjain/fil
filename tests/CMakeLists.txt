add_executable(fil_tests
    unit/board_test.cpp
    unit/can_bus_test.cpp
    unit/config_test.cpp
    unit/cortexm_test.cpp
    unit/exceptions_test.cpp
    unit/elf_loader_test.cpp
    unit/event_loop_test.cpp
    unit/fdcan_test.cpp
    unit/hard_float_firmware_test.cpp
    unit/memory_bus_test.cpp
    unit/peripheral_test.cpp
    unit/scheduler_tick_test.cpp
    unit/smoke_test.cpp
    unit/stm32g4_test.cpp
    unit/startup_runtime_test.cpp
    unit/world_test.cpp
    unit/world_time_test.cpp
)
target_include_directories(fil_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(fil_tests PRIVATE fil_core)
target_compile_definitions(fil_tests PRIVATE FIL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
fil_set_project_options(fil_tests)

add_test(NAME fil.unit COMMAND fil_tests)

add_executable(fil_cpu_tests
    unit/alu_test.cpp
    unit/cpu_step_test.cpp
    unit/decoder_test.cpp
)
target_include_directories(fil_cpu_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(fil_cpu_tests PRIVATE fil_core)
target_compile_definitions(fil_cpu_tests PRIVATE FIL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
fil_set_project_options(fil_cpu_tests)
add_test(NAME fil.cpu COMMAND fil_cpu_tests)

add_executable(fil_can_end_to_end_tests
    unit/can_end_to_end_test.cpp
)
target_include_directories(fil_can_end_to_end_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(fil_can_end_to_end_tests PRIVATE fil_core)
target_compile_definitions(fil_can_end_to_end_tests PRIVATE FIL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
fil_set_project_options(fil_can_end_to_end_tests)
add_test(NAME fil.can_end_to_end COMMAND fil_can_end_to_end_tests)

add_test(NAME fil.cli_help COMMAND fil --help)
set_tests_properties(fil.cli_help PROPERTIES
    PASS_REGULAR_EXPRESSION "deterministic STM32G4 firmware emulator"
)
