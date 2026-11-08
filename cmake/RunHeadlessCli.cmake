# Script-mode test:
#   cmake -DCLI=<path to warehousesim_cli> -DWORK_DIR=<scratch directory> -P RunHeadlessCli.cmake
# Runs the simulation with no window or display on every predefined layout, checks it
# completes, and checks that equal seeds give byte-identical output. Then runs a map file the
# same way, and checks that maps that are invalid or cannot host a simulation are refused.
if(NOT CLI)
  message(FATAL_ERROR "pass -DCLI=<path to warehousesim_cli>")
endif()
if(NOT WORK_DIR)
  message(FATAL_ERROR "pass -DWORK_DIR=<a directory the test may write to>")
endif()

foreach(layout demo compact fulfilment)
  foreach(run first second)
    execute_process(
      COMMAND "${CLI}" --layout ${layout} --ticks 150 --seed 3
      RESULT_VARIABLE status
      OUTPUT_VARIABLE output_${run}
      ERROR_VARIABLE errors)
    if(NOT status EQUAL 0)
      message(FATAL_ERROR "${layout}: CLI exited with '${status}': ${errors}")
    endif()
  endforeach()
  if(NOT output_first STREQUAL output_second)
    message(FATAL_ERROR "${layout}: two runs with the same seed produced different output")
  endif()
  string(FIND "${output_first}" "\"tick\": 150" tick_position)
  if(tick_position EQUAL -1)
    message(FATAL_ERROR "${layout}: output does not report tick 150:\n${output_first}")
  endif()
endforeach()

execute_process(
  COMMAND "${CLI}" --layout no-such-layout
  RESULT_VARIABLE bad_status
  OUTPUT_QUIET ERROR_QUIET)
if(NOT bad_status EQUAL 2)
  message(FATAL_ERROR "unknown layout should exit with status 2, got '${bad_status}'")
endif()

# ---- a map file ------------------------------------------------------------------------------
file(MAKE_DIRECTORY "${WORK_DIR}")
set(map_file "${WORK_DIR}/headless_cli_map.json")
file(WRITE "${map_file}" [=[
{
  "format": "warehousesim-map",
  "version": 1,
  "name": "cmake test",
  "width": 9,
  "height": 4,
  "shelves": [[2, 1], [3, 1], [4, 1], [5, 1], [6, 1]],
  "pickups": [[0, 0]],
  "dropoffs": [[8, 3]]
}
]=])

foreach(run first second)
  execute_process(
    COMMAND "${CLI}" --map "${map_file}" --ticks 150 --seed 3
    RESULT_VARIABLE status
    OUTPUT_VARIABLE map_output_${run}
    ERROR_VARIABLE errors)
  if(NOT status EQUAL 0)
    message(FATAL_ERROR "map file: CLI exited with '${status}': ${errors}")
  endif()
endforeach()
if(NOT map_output_first STREQUAL map_output_second)
  message(FATAL_ERROR "map file: two runs with the same seed produced different output")
endif()
string(FIND "${map_output_first}" "\"tick\": 150" map_tick_position)
if(map_tick_position EQUAL -1)
  message(FATAL_ERROR "map file: output does not report tick 150:\n${map_output_first}")
endif()

# Each bad map: file content, the exit status expected, and a piece of the message.
function(expect_refused name content expected_status expected_message)
  set(bad_file "${WORK_DIR}/headless_cli_bad_map.json")
  file(WRITE "${bad_file}" "${content}")
  execute_process(
    COMMAND "${CLI}" --map "${bad_file}" --ticks 5
    RESULT_VARIABLE status
    OUTPUT_QUIET
    ERROR_VARIABLE errors)
  file(REMOVE "${bad_file}")
  if(NOT status EQUAL ${expected_status})
    message(FATAL_ERROR "${name}: expected exit status ${expected_status}, got '${status}': ${errors}")
  endif()
  string(FIND "${errors}" "${expected_message}" message_position)
  if(message_position EQUAL -1)
    message(FATAL_ERROR "${name}: the message should mention '${expected_message}':\n${errors}")
  endif()
endfunction()

set(header "\"format\": \"warehousesim-map\", \"version\": 1")
expect_refused("not JSON" "{ this is not json" 1 "not valid JSON")
expect_refused("newer version" "{\"format\": \"warehousesim-map\", \"version\": 2}" 1 "unsupported version 2")
expect_refused("bad dimensions" "{${header}, \"width\": 0, \"height\": 3}" 1 "/width: must be from 1 to 512")
expect_refused("object outside the grid"
  "{${header}, \"width\": 3, \"height\": 3, \"walls\": [[3, 0]]}" 1 "outside the 3 x 3 grid")
expect_refused("no dropoff"
  "{${header}, \"width\": 3, \"height\": 3, \"pickups\": [[0, 0]]}" 1 "no dropoff")

execute_process(
  COMMAND "${CLI}" --map "${WORK_DIR}/no-such-map.json"
  RESULT_VARIABLE missing_status
  OUTPUT_QUIET ERROR_QUIET)
if(NOT missing_status EQUAL 1)
  message(FATAL_ERROR "a missing map file should exit with status 1, got '${missing_status}'")
endif()

execute_process(
  COMMAND "${CLI}" --map "${map_file}" --layout demo
  RESULT_VARIABLE both_status
  OUTPUT_QUIET ERROR_QUIET)
if(NOT both_status EQUAL 2)
  message(FATAL_ERROR "--map together with --layout should exit with status 2, got '${both_status}'")
endif()

file(REMOVE "${map_file}")
