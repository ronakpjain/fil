cmake_minimum_required(VERSION 3.20)

foreach(required_variable IN ITEMS FIL_EXECUTABLE NETWORK_CONFIG TRACE_FILE)
    if(NOT DEFINED ${required_variable} OR "${${required_variable}}" STREQUAL "")
        message(FATAL_ERROR "${required_variable} is required")
    endif()
endforeach()

# A failed run must not accidentally validate a trace left by an earlier pass.
file(REMOVE "${TRACE_FILE}")
execute_process(
    COMMAND ${FIL_EMULATOR} "${FIL_EXECUTABLE}" run "${NETWORK_CONFIG}"
        --duration 20ms
        --instruction-budget 1000000
        --quantum 512
        --strict-mmio
        --no-detect-spin
        --trace "${TRACE_FILE}"
    RESULT_VARIABLE run_result
    OUTPUT_VARIABLE run_stdout
    ERROR_VARIABLE run_stderr
    TIMEOUT 60
)

if(NOT "${run_result}" STREQUAL "0")
    message(FATAL_ERROR
        "six-board PER network smoke failed with exit ${run_result}\n"
        "stdout:\n${run_stdout}\n"
        "stderr:\n${run_stderr}"
    )
endif()

string(REPLACE "\r\n" "\n" normalized_stdout "${run_stdout}")
set(padded_stdout "\n${normalized_stdout}\n")
foreach(required_line IN ITEMS
    "network: per_vehicle_smoke"
)
    string(FIND "${padded_stdout}" "\n${required_line}\n" line_position)
    if(line_position EQUAL -1)
        message(FATAL_ERROR
            "network smoke output is missing exact line '${required_line}'\n"
            "stdout:\n${run_stdout}"
        )
    endif()
endforeach()

string(REGEX MATCH "\n  stop:[ ]+time-budget\n" stop_match "${padded_stdout}")
if(stop_match STREQUAL "")
    message(FATAL_ERROR "network smoke output is missing the time-budget stop\nstdout:\n${run_stdout}")
endif()

string(REGEX MATCH "\n  boards:[ ]+6\n" boards_match "${padded_stdout}")
if(boards_match STREQUAL "")
    message(FATAL_ERROR "network smoke output is missing a six-board count\nstdout:\n${run_stdout}")
endif()

string(REGEX MATCH "\\(([0-9]+) ns\\)" time_match "${normalized_stdout}")
if(time_match STREQUAL "")
    message(FATAL_ERROR "network smoke output is missing a numeric time line\nstdout:\n${run_stdout}")
endif()
set(stop_time_ns "${CMAKE_MATCH_1}")
if(stop_time_ns LESS 20000000 OR stop_time_ns GREATER_EQUAL 21000000)
    message(FATAL_ERROR
        "network smoke stop time ${stop_time_ns} ns is outside [20000000, 21000000)\n"
        "stdout:\n${run_stdout}"
    )
endif()

string(REGEX MATCHALL "\n    stop:[ ]+time-budget\n" board_stop_lines
    "${normalized_stdout}")
list(LENGTH board_stop_lines board_stop_count)
if(NOT board_stop_count EQUAL 6)
    message(FATAL_ERROR
        "expected six boards to stop at the time budget, found ${board_stop_count}\n"
        "stdout:\n${run_stdout}"
    )
endif()

string(REGEX MATCHALL "unknown_mmio:" mmio_lines "${normalized_stdout}")
list(LENGTH mmio_lines mmio_line_count)
if(NOT mmio_line_count EQUAL 0)
    message(FATAL_ERROR
        "network smoke reported ${mmio_line_count} unknown MMIO lines:\n${run_stdout}"
    )
endif()

if(NOT EXISTS "${TRACE_FILE}")
    message(FATAL_ERROR "network smoke did not create trace '${TRACE_FILE}'")
endif()
file(READ "${TRACE_FILE}" trace_jsonl)

# No CAN frame is injected by this test. A shared-bus source therefore proves
# that real firmware transmitted the frame and at least one peer received it.
string(REGEX MATCH "\"source\":\"[^\"/]+/[^\"]+\",\"type\":\"can_tx\"" firmware_can_tx
    "${trace_jsonl}")
string(REGEX MATCH "\"source\":\"[^\"/]+/[^\"]+\",\"type\":\"can_rx\"" firmware_can_rx
    "${trace_jsonl}")
if(firmware_can_tx STREQUAL "")
    message(FATAL_ERROR "network trace contains no firmware-originated shared-bus can_tx record")
endif()
if(firmware_can_rx STREQUAL "")
    message(FATAL_ERROR "network trace contains no shared-bus can_rx record")
endif()

message(STATUS "six-board PER network ran 20 ms with real CAN TX/RX")
