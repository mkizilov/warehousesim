include(GoogleTest)

# Tests use a lighter warning set than the libraries: gtest's macros trip -Wconversion/-Wshadow.
function(warehousesim_add_test name)
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBS" ${ARGN})
  add_executable(${name} ${ARG_SOURCES})
  target_link_libraries(${name} PRIVATE ${ARG_LIBS} GTest::gtest_main)
  if(MSVC)
    target_compile_options(${name} PRIVATE /W4)
  else()
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic)
    if(WAREHOUSESIM_WARNINGS_AS_ERRORS)
      target_compile_options(${name} PRIVATE -Werror)
    endif()
  endif()
  gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
endfunction()

# Linked against core and world only. If world ever grew a dependency on the simulation,
# JSON or SDL, this target would stop building.
warehousesim_add_test(warehousesim_world_tests
  SOURCES
    core_test.cpp
    layouts_test.cpp
    occupancy_test.cpp
    warehouse_test.cpp
    world_model_test.cpp
  LIBS warehousesim::world)

warehousesim_add_test(warehousesim_tests
  SOURCES
    planning_test.cpp
    robots_test.cpp
    runner_test.cpp
    simulation_test.cpp
    tasks_test.cpp
  LIBS warehousesim::simulation)

if(TARGET warehousesim::rendering)
  target_sources(warehousesim_tests PRIVATE viewport_test.cpp)
  target_link_libraries(warehousesim_tests PRIVATE warehousesim::rendering)
endif()
