cmake_minimum_required(VERSION 3.20)

foreach(required_variable IN ITEMS FIL_EXECUTABLE BOARD_CONFIG)
    if(NOT DEFINED ${required_variable} OR "${${required_variable}}" STREQUAL "")
        message(FATAL_ERROR "${required_variable} is required")
    endif()
endforeach()

execute_process(
    COMMAND ${FIL_EMULATOR} "${FIL_EXECUTABLE}" run "${BOARD_CONFIG}"
        --duration 1s
        --instruction-budget 50000000
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
)
    string(FIND "${padded_stdout}" "\n${required_line}\n" line_position)
    if(line_position EQUAL -1)
        message(FATAL_ERROR
            "g4_testing smoke output is missing exact line '${required_line}'\n"
            "stdout:\n${run_stdout}"
        )
    endif()
endforeach()

string(REGEX MATCH "\n  stop:[ ]+time-budget\n" stop_match "${padded_stdout}")
if(stop_match STREQUAL "")
    message(FATAL_ERROR "g4_testing smoke output is missing the time-budget stop\nstdout:\n${run_stdout}")
endif()

string(REGEX MATCH "\n  unknown_mmio:[ ]+0\n" mmio_match "${padded_stdout}")
if(mmio_match STREQUAL "")
    message(FATAL_ERROR "g4_testing smoke output is missing a zero unknown_mmio line\nstdout:\n${run_stdout}")
endif()

string(REGEX MATCH "\\(([0-9]+) ns\\)" time_match "${normalized_stdout}")
if(time_match STREQUAL "")
    message(FATAL_ERROR "g4_testing smoke output is missing a numeric time line\nstdout:\n${run_stdout}")
endif()
set(stop_time_ns "${CMAKE_MATCH_1}")
if(stop_time_ns LESS 1000000000 OR stop_time_ns GREATER_EQUAL 1001000000)
    message(FATAL_ERROR
        "g4_testing smoke stop time ${stop_time_ns} ns is outside [1000000000, 1001000000)\n"
        "stdout:\n${run_stdout}"
    )
endif()

string(REGEX MATCHALL "\n    0x[0-9a-f]+  reads=[0-9]+ writes=[0-9]+" mmio_details
    "${normalized_stdout}")
list(LENGTH mmio_details mmio_detail_count)
if(NOT mmio_detail_count EQUAL 0)
    message(FATAL_ERROR
        "g4_testing smoke reported ${mmio_detail_count} unknown MMIO detail lines:\n${run_stdout}"
    )
endif()

message(STATUS "g4_testing reached the 1 s deadline within documented overshoot with zero unknown MMIO")
