# Script-mode test: cmake -DCLI=<path to warehousesim_cli> -P RunHeadlessCli.cmake
# Runs the simulation with no window or display on every predefined layout, checks it
# completes, and checks that equal seeds give byte-identical output.
if(NOT CLI)
  message(FATAL_ERROR "pass -DCLI=<path to warehousesim_cli>")
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
