add_executable(warehousesim_planning_bench planning_bench.cpp)
target_include_directories(warehousesim_planning_bench PRIVATE "${CMAKE_SOURCE_DIR}/apps/common")
target_link_libraries(warehousesim_planning_bench PRIVATE warehousesim::planning warehousesim::testing)
warehousesim_configure_target(warehousesim_planning_bench)

# Counting allocations replaces operator new, which sanitizers do not appreciate.
if(NOT WAREHOUSESIM_SANITIZE)
  target_link_libraries(warehousesim_planning_bench PRIVATE warehousesim::testing_allocation)
else()
  target_sources(warehousesim_planning_bench PRIVATE alloc_stub.cpp)
endif()

# A quick run on tiny maps; it also cross-checks A* against breadth-first search and fails on any
# mismatch. Use the full benchmark (no --quick) on a Release build for real numbers.
if(WAREHOUSESIM_BUILD_TESTS)
  add_test(NAME planning_bench_smoke COMMAND warehousesim_planning_bench --quick)
endif()

# Headless fleet benchmark: layouts x fleet sizes x fixed seeds, measured by the metrics module. All
# its results are simulated. Its --quick run checks that no two robots ever overlap or swap, that
# every task is accounted for and that a run reproduces exactly; it fails on any of them.
add_executable(warehousesim_fleet_bench fleet_bench.cpp)
target_include_directories(warehousesim_fleet_bench PRIVATE "${CMAKE_SOURCE_DIR}/apps/common")
target_link_libraries(warehousesim_fleet_bench PRIVATE warehousesim::simulation)
warehousesim_configure_target(warehousesim_fleet_bench)

if(WAREHOUSESIM_BUILD_TESTS)
  add_test(NAME fleet_bench_smoke
    COMMAND warehousesim_fleet_bench --quick
            --csv ${CMAKE_CURRENT_BINARY_DIR}/fleet_bench_smoke.csv)
endif()
