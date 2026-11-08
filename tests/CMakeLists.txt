include(GoogleTest)

add_executable(warehousesim_tests
  core_test.cpp
  planning_test.cpp
  robots_test.cpp
  runner_test.cpp
  simulation_test.cpp
  tasks_test.cpp
  world_test.cpp)
target_link_libraries(warehousesim_tests PRIVATE warehousesim::simulation GTest::gtest_main)

if(TARGET warehousesim::rendering)
  target_sources(warehousesim_tests PRIVATE viewport_test.cpp)
  target_link_libraries(warehousesim_tests PRIVATE warehousesim::rendering)
endif()

# Tests use a lighter warning set: gtest's macros trip -Wconversion/-Wshadow.
if(MSVC)
  target_compile_options(warehousesim_tests PRIVATE /W4)
else()
  target_compile_options(warehousesim_tests PRIVATE -Wall -Wextra -Wpedantic)
  if(WAREHOUSESIM_WARNINGS_AS_ERRORS)
    target_compile_options(warehousesim_tests PRIVATE -Werror)
  endif()
endif()

gtest_discover_tests(warehousesim_tests DISCOVERY_TIMEOUT 60)
