# Script-mode test:
#   cmake -DCLI=<path to warehousesim_cli> -DWORK_DIR=<scratch directory> -P RunHeadlessCli.cmake
# Runs the simulation with no window or display on every predefined layout, checks it
# completes, and checks that equal seeds give byte-identical output. Then runs a map file the
# same way, and checks that maps that are invalid or cannot host a simulation are refused. Last,
# records runs, replays them and checks that a replay matches and that a tampered recording is found.
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

# ---- recording and replay --------------------------------------------------------------------
# A recorded run replays to the same output, and the replay checks itself against the recording.
set(recording "${WORK_DIR}/headless_cli_recording.json")
foreach(layout demo compact fulfilment)
  execute_process(
    COMMAND "${CLI}" --layout ${layout} --ticks 250 --seed 5 --record "${recording}"
            --checkpoint-interval 50
    RESULT_VARIABLE status
    OUTPUT_VARIABLE recorded_output
    ERROR_VARIABLE errors)
  if(NOT status EQUAL 0)
    message(FATAL_ERROR "${layout}: recording exited with '${status}': ${errors}")
  endif()
  execute_process(
    COMMAND "${CLI}" --layout ${layout} --ticks 250 --seed 5
    OUTPUT_VARIABLE plain_output
    RESULT_VARIABLE plain_status)
  if(NOT recorded_output STREQUAL plain_output)
    message(FATAL_ERROR "${layout}: recording a run changed its output")
  endif()
  foreach(run first second)
    execute_process(
      COMMAND "${CLI}" --replay "${recording}"
      RESULT_VARIABLE status
      OUTPUT_VARIABLE replayed_${run}
      ERROR_VARIABLE errors)
    if(NOT status EQUAL 0)
      message(FATAL_ERROR "${layout}: replay exited with '${status}': ${errors}")
    endif()
    string(FIND "${errors}" "match the recording" matched)
    if(matched EQUAL -1)
      message(FATAL_ERROR "${layout}: the replay should say that it matched:\n${errors}")
    endif()
  endforeach()
  if(NOT replayed_first STREQUAL recorded_output OR NOT replayed_second STREQUAL recorded_output)
    message(FATAL_ERROR "${layout}: the replay printed another final state than the run")
  endif()
endforeach()

# A recording that has been tampered with is found out: the replay exits with 1 and says where.
file(READ "${recording}" recording_text)
string(REGEX MATCH "\"checksum\":\"[0-9a-f]+\",\"tick\":100" checkpoint "${recording_text}")
if(checkpoint STREQUAL "")
  message(FATAL_ERROR "the recording has no checkpoint at tick 100:\n${recording_text}")
endif()
string(REPLACE "${checkpoint}" "\"checksum\":\"0123456789abcdef\",\"tick\":100" tampered_text "${recording_text}")
set(tampered "${WORK_DIR}/headless_cli_tampered.json")
file(WRITE "${tampered}" "${tampered_text}")
execute_process(
  COMMAND "${CLI}" --replay "${tampered}"
  RESULT_VARIABLE tampered_status
  OUTPUT_VARIABLE tampered_output
  ERROR_VARIABLE tampered_errors)
if(NOT tampered_status EQUAL 1)
  message(FATAL_ERROR "a tampered recording should exit with status 1, got '${tampered_status}'")
endif()
string(FIND "${tampered_errors}" "differs from the recording at tick 100" tampered_position)
if(tampered_position EQUAL -1)
  message(FATAL_ERROR "the divergence should name tick 100:\n${tampered_errors}")
endif()
if(NOT tampered_output STREQUAL "")
  message(FATAL_ERROR "a replay that diverged must not print a final state as if it had not")
endif()
execute_process(
  COMMAND "${CLI}" --replay "${tampered}" --no-verify
  RESULT_VARIABLE unverified_status
  OUTPUT_QUIET ERROR_QUIET)
if(NOT unverified_status EQUAL 0)
  message(FATAL_ERROR "--no-verify should play a tampered recording, got '${unverified_status}'")
endif()
file(REMOVE "${tampered}")

# What is not a usable recording, or not a usable command line, is refused.
file(WRITE "${recording}" "{ this is not json")
execute_process(COMMAND "${CLI}" --replay "${recording}"
                RESULT_VARIABLE broken_status OUTPUT_QUIET ERROR_VARIABLE broken_errors)
if(NOT broken_status EQUAL 1)
  message(FATAL_ERROR "a file that is not JSON should exit with status 1, got '${broken_status}'")
endif()
string(FIND "${broken_errors}" "is not a usable recording" broken_position)
if(broken_position EQUAL -1)
  message(FATAL_ERROR "the message should say so:\n${broken_errors}")
endif()
execute_process(COMMAND "${CLI}" --replay "${WORK_DIR}/no-such-recording.json"
                RESULT_VARIABLE missing_recording_status OUTPUT_QUIET ERROR_QUIET)
if(NOT missing_recording_status EQUAL 1)
  message(FATAL_ERROR "a missing recording should exit with status 1, got '${missing_recording_status}'")
endif()
foreach(bad_arguments "--replay;x.json;--layout;demo" "--replay;x.json;--seed;3"
                      "--replay;x.json;--record;y.json" "--no-verify"
                      "--checkpoint-interval;10")
  execute_process(COMMAND "${CLI}" ${bad_arguments}
                  RESULT_VARIABLE bad_status OUTPUT_QUIET ERROR_QUIET)
  if(NOT bad_status EQUAL 2)
    message(FATAL_ERROR "'${bad_arguments}' should exit with status 2, got '${bad_status}'")
  endif()
endforeach()
execute_process(COMMAND "${CLI}" --record "${WORK_DIR}/no-such-folder/run.json" --ticks 5
                RESULT_VARIABLE nowhere_status OUTPUT_QUIET ERROR_QUIET)
if(NOT nowhere_status EQUAL 2)
  message(FATAL_ERROR "a recording that cannot be written should exit with status 2 at once, got '${nowhere_status}'")
endif()
file(REMOVE "${recording}")
