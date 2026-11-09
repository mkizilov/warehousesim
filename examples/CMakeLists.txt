# Every example map is run headless (see cmake/RunExample.cmake): audited step by step, repeated,
# repeated on planning threads, recorded and replayed. The numbers are the fleet, the ticks between
# new tasks and the length of the run that examples/README.md suggests for that map.
function(warehousesim_add_example name robots spawn ticks)
  add_test(NAME example_${name}
    COMMAND ${CMAKE_COMMAND}
            -DCLI=$<TARGET_FILE:warehousesim_cli>
            -DMAP=${CMAKE_CURRENT_SOURCE_DIR}/${name}.json
            -DROBOTS=${robots} -DSPAWN=${spawn} -DTICKS=${ticks}
            -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/${name}
            -P ${CMAKE_SOURCE_DIR}/cmake/RunExample.cmake)
endfunction()

warehousesim_add_example(small-depot 4 5 1500)
warehousesim_add_example(fulfilment-aisles 16 3 1500)
warehousesim_add_example(cross-dock 12 3 1500)
warehousesim_add_example(choke-point 10 4 1500)
warehousesim_add_example(single-lane 3 8 1500)
warehousesim_add_example(charging-hub 12 4 1500)
