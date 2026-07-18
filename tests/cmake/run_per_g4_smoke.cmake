cmake_minimum_required(VERSION 3.20)

foreach(required_variable IN ITEMS FIL_EXECUTABLE BOARD_CONFIG)
    if(NOT DEFINED ${required_variable} OR "${${required_variable}}" STREQUAL "")
        message(FATAL_ERROR "${required_variable} is required")
    endif()
endforeach()

execute_process(
    COMMAND "${FIL_EXECUTABLE}" run "${BOARD_CONFIG}"
        --duration-ms 1000
        --max-instructions 50000000
        --strict-mmio
        --no-detect-spin
    RESULT_VARIABLE run_result
    OUTPUT_VARIABLE run_stdout
    ERROR_VARIABLE run_stderr
    TIMEOUT 120
)

if(NOT "${run_result}" STREQUAL "0")
    message(FATAL_ERROR
        "g4_testing one-second smoke failed with exit ${run_result}\n"
        "stdout:\n${run_stdout}\n"
        "stderr:\n${run_stderr}"
    )
endif()

string(REPLACE "\r\n" "\n" normalized_stdout "${run_stdout}")
set(padded_stdout "\n${normalized_stdout}\n")
foreach(required_line IN ITEMS
    "board: g4_testing"
    "stop: time-budget"
    "time_ns: 1000000000"
    "unknown_mmio_addresses: 0"
)
    string(FIND "${padded_stdout}" "\n${required_line}\n" line_position)
    if(line_position EQUAL -1)
        message(FATAL_ERROR
            "g4_testing smoke output is missing exact line '${required_line}'\n"
            "stdout:\n${run_stdout}"
        )
    endif()
endforeach()

string(FIND "${run_stdout}" "unknown_mmio:" unknown_detail_position)
if(NOT unknown_detail_position EQUAL -1)
    message(FATAL_ERROR "g4_testing smoke reported an unknown MMIO detail:\n${run_stdout}")
endif()

message(STATUS "g4_testing reached exactly 1000000000 ns with zero unknown MMIO")
