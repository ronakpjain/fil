cmake_minimum_required(VERSION 3.20)

foreach(required_variable IN ITEMS FIL_EXECUTABLE NETWORK_CONFIG TRACE_FILE)
    if(NOT DEFINED ${required_variable} OR "${${required_variable}}" STREQUAL "")
        message(FATAL_ERROR "${required_variable} is required")
    endif()
endforeach()

# A failed run must not accidentally validate a trace left by an earlier pass.
file(REMOVE "${TRACE_FILE}")
execute_process(
    COMMAND "${FIL_EXECUTABLE}" run-network "${NETWORK_CONFIG}"
        --duration-ms 10
        --max-instructions 1000000
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
    "stop: time-budget"
    "boards: 6"
    "time_ns: 10000000"
)
    string(FIND "${padded_stdout}" "\n${required_line}\n" line_position)
    if(line_position EQUAL -1)
        message(FATAL_ERROR
            "network smoke output is missing exact line '${required_line}'\n"
            "stdout:\n${run_stdout}"
        )
    endif()
endforeach()

string(REGEX MATCHALL "\nboard [^\n]*: stop=time-budget [^\n]*" board_stop_lines
    "\n${normalized_stdout}")
list(LENGTH board_stop_lines board_stop_count)
if(NOT board_stop_count EQUAL 6)
    message(FATAL_ERROR
        "expected six boards to stop at the time budget, found ${board_stop_count}\n"
        "stdout:\n${run_stdout}"
    )
endif()

string(FIND "${run_stdout}" "unknown_mmio:" unknown_detail_position)
if(NOT unknown_detail_position EQUAL -1)
    message(FATAL_ERROR "network smoke reported an unknown MMIO detail:\n${run_stdout}")
endif()

if(NOT EXISTS "${TRACE_FILE}")
    message(FATAL_ERROR "network smoke did not create trace '${TRACE_FILE}'")
endif()
file(READ "${TRACE_FILE}" trace_jsonl)

# No CAN frame is injected by this test. A shared-bus source therefore proves
# that real firmware transmitted the frame and at least one peer received it.
string(REGEX MATCH "\"source\":\"vehicle/[^\"]+\",\"type\":\"can_tx\"" firmware_can_tx
    "${trace_jsonl}")
string(REGEX MATCH "\"source\":\"vehicle/[^\"]+\",\"type\":\"can_rx\"" firmware_can_rx
    "${trace_jsonl}")
if(firmware_can_tx STREQUAL "")
    message(FATAL_ERROR "network trace contains no firmware-originated shared-bus can_tx record")
endif()
if(firmware_can_rx STREQUAL "")
    message(FATAL_ERROR "network trace contains no shared-bus can_rx record")
endif()

message(STATUS "six-board PER network reached exactly 10000000 ns with real CAN TX/RX")
