include(GoogleTest)

# Tests use a lighter warning set than the libraries: gtest's macros trip -Wconversion/-Wshadow.
function(warehousesim_add_test name)
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBS" ${ARGN})
  add_executable(${name} ${ARG_SOURCES})
  target_link_libraries(${name} PRIVATE ${ARG_LIBS} GTest::gtest_main)
  target_include_directories(${name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")  # scenario_support.hpp
  if(MSVC)
    target_compile_options(${name} PRIVATE /W4)
  else()
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
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
    layout_check_test.cpp
    layouts_test.cpp
    occupancy_test.cpp
    warehouse_test.cpp
    world_model_test.cpp
  LIBS warehousesim::world)

# The map file format needs the world and JSON but no editor, simulation, rendering or SDL. The file
# tests write only below the working directory (the build tree under ctest), see scratch_dir.hpp.
warehousesim_add_test(warehousesim_mapio_tests
  SOURCES
    map_file_io_test.cpp
    map_file_test.cpp
  LIBS warehousesim::mapio)

# The editing model needs the world and the map file format but no simulation, view, rendering or SDL.
warehousesim_add_test(warehousesim_editor_tests
  SOURCES
    editor_files_test.cpp
    map_editor_test.cpp
  LIBS warehousesim::editor warehousesim::mapio)

# Linked against core, world and planning only: the path planner needs no robots, tasks,
# simulation, rendering or SDL. The reference search and map generators come from testing/.
warehousesim_add_test(warehousesim_planning_tests
  SOURCES
    astar_reference_test.cpp
    astar_scenarios_test.cpp
    planning_test.cpp
    space_time_astar_test.cpp
  LIBS warehousesim::planning warehousesim::testing)

# Counts heap allocations by replacing operator new in this executable, so it is not built under
# sanitizers (which interpose their own allocator).
if(NOT WAREHOUSESIM_SANITIZE)
  warehousesim_add_test(warehousesim_planning_alloc_tests
    SOURCES planning_alloc_test.cpp
    LIBS warehousesim::testing_allocation)
endif()

# Linked against core and robots only: the robot entity, its state machine and its motion model
# need no world, planner, simulation, rendering or SDL.
warehousesim_add_test(warehousesim_robot_tests
  SOURCES
    forecast_test.cpp
    motion_test.cpp
    robot_motion_test.cpp
    robots_test.cpp
  LIBS warehousesim::robots)

# The reservation table, traffic controller, cooperative router and progress monitor need the grid,
# robots and the planner but no tasks, simulation, rendering or SDL.
warehousesim_add_test(warehousesim_traffic_tests
  SOURCES
    cooperative_router_test.cpp
    progress_monitor_test.cpp
    reservation_table_test.cpp
    traffic_controller_test.cpp
  LIBS warehousesim::traffic)

# Task queue, estimators, generator and dispatcher need the planner and robots but no simulation,
# rendering or SDL. The reference search and map generators come from testing/.
warehousesim_add_test(warehousesim_task_tests
  SOURCES
    dispatcher_test.cpp
    distance_estimator_test.cpp
    task_generator_test.cpp
    task_queue_test.cpp
  LIBS warehousesim::tasks warehousesim::testing)

# Camera, scene and view-controller tests need no SDL or display, so they build and run even
# when WAREHOUSESIM_BUILD_GUI is OFF.
warehousesim_add_test(warehousesim_tests
  SOURCES
    camera_test.cpp
    collision_avoidance_test.cpp
    cooperative_planning_test.cpp
    editor_controller_test.cpp
    editor_scene_test.cpp
    interpolation_test.cpp
    map_simulation_test.cpp
    movement_test.cpp
    runner_test.cpp
    scene_test.cpp
    simulation_dispatch_test.cpp
    simulation_test.cpp
    view_controller_test.cpp
  LIBS warehousesim::view warehousesim::simulation warehousesim::mapio)

# The simulation must run with no window, display or SDL: the CLI does not link any of them.
add_test(NAME headless_cli_all_layouts
  COMMAND ${CMAKE_COMMAND} -DCLI=$<TARGET_FILE:warehousesim_cli>
          -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/headless_cli
          -P ${CMAKE_SOURCE_DIR}/cmake/RunHeadlessCli.cmake)
