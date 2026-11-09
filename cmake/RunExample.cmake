# Script-mode test for one example map:
#   cmake -DCLI=<warehousesim_cli> -DMAP=<map.json> -DROBOTS=N -DSPAWN=N -DTICKS=N
#         -DWORK_DIR=<scratch directory> -P RunExample.cmake
# Runs the map headless with the audit on, then checks that it finished, did some work, gives the
# same output when run again and on planning threads, and that its recording replays to the same
# final state.
foreach(variable CLI MAP ROBOTS SPAWN TICKS WORK_DIR)
  if(NOT DEFINED ${variable})
    message(FATAL_ERROR "pass -D${variable}=...")
  endif()
endforeach()
file(MAKE_DIRECTORY "${WORK_DIR}")
get_filename_component(example "${MAP}" NAME_WE)
set(recording "${WORK_DIR}/${example}.recording.json")
set(run_arguments --map "${MAP}" --robots ${ROBOTS} --spawn ${SPAWN} --ticks ${TICKS} --seed 1)

execute_process(COMMAND "${CLI}" ${run_arguments} --audit --record "${recording}"
                RESULT_VARIABLE status OUTPUT_VARIABLE first ERROR_VARIABLE errors)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "${example}: exited with '${status}':\n${errors}")
endif()
string(FIND "${errors}" "no violations" audited)
if(audited EQUAL -1)
  message(FATAL_ERROR "${example}: the audit did not say 'no violations':\n${errors}")
endif()

string(JSON final_tick GET "${first}" tick)
string(JSON completed GET "${first}" completedTasks)
if(NOT final_tick EQUAL ${TICKS})
  message(FATAL_ERROR "${example}: ended at tick ${final_tick}, not ${TICKS}")
endif()
if(NOT completed GREATER 0)
  message(FATAL_ERROR "${example}: no task was completed in ${TICKS} ticks")
endif()

execute_process(COMMAND "${CLI}" ${run_arguments}
                RESULT_VARIABLE status OUTPUT_VARIABLE again ERROR_QUIET)
if(NOT status EQUAL 0 OR NOT again STREQUAL first)
  message(FATAL_ERROR "${example}: a second run did not give the same result")
endif()
execute_process(COMMAND "${CLI}" ${run_arguments} --threads 2
                RESULT_VARIABLE status OUTPUT_VARIABLE threaded ERROR_QUIET)
if(NOT status EQUAL 0 OR NOT threaded STREQUAL first)
  message(FATAL_ERROR "${example}: planning on 2 threads changed the result")
endif()

execute_process(COMMAND "${CLI}" --replay "${recording}"
                RESULT_VARIABLE status OUTPUT_VARIABLE replayed ERROR_VARIABLE errors)
if(NOT status EQUAL 0 OR NOT replayed STREQUAL first)
  message(FATAL_ERROR "${example}: the replay did not reproduce the run:\n${errors}")
endif()
file(REMOVE "${recording}")
message(STATUS "${example}: ${completed} tasks completed in ${TICKS} ticks")
