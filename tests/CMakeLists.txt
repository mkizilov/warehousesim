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

# Linked against core and robots only: the robot entity and its state machine need no world,
# planner, simulation, rendering or SDL.
warehousesim_add_test(warehousesim_robot_tests
  SOURCES robots_test.cpp
  LIBS warehousesim::robots)

# Camera, scene and view-controller tests need no SDL or display, so they build and run even
# when WAREHOUSESIM_BUILD_GUI is OFF.
warehousesim_add_test(warehousesim_tests
  SOURCES
    camera_test.cpp
    planning_test.cpp
    runner_test.cpp
    scene_test.cpp
    simulation_test.cpp
    tasks_test.cpp
    view_controller_test.cpp
  LIBS warehousesim::view warehousesim::simulation)

# The simulation must run with no window, display or SDL: the CLI does not link any of them.
add_test(NAME headless_cli_all_layouts
  COMMAND ${CMAKE_COMMAND} -DCLI=$<TARGET_FILE:warehousesim_cli>
          -P ${CMAKE_SOURCE_DIR}/cmake/RunHeadlessCli.cmake)
