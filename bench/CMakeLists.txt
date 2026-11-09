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

# Wall-clock cost of a step with and without planning threads on fleets of a hundred robots and
# more. Its --quick run only checks that every thread count gives exactly the single-threaded
# states and routes; the timings are for a Release build on a quiet machine (see the README).
add_executable(warehousesim_scaling_bench scaling_bench.cpp)
target_include_directories(warehousesim_scaling_bench PRIVATE "${CMAKE_SOURCE_DIR}/apps/common")
target_link_libraries(warehousesim_scaling_bench PRIVATE warehousesim::simulation warehousesim::view)
warehousesim_configure_target(warehousesim_scaling_bench)

if(WAREHOUSESIM_BUILD_TESTS)
  add_test(NAME scaling_bench_smoke COMMAND warehousesim_scaling_bench --quick)
endif()

# Performance regression gate: a fixed set of scenarios against bench/regression_baseline.txt. What
# it compares is exact (tasks done, search work, state checksum, heap allocations a step), so it can
# run in the test suite; wall-clock time is reported, and compared only with times saved on the
# same machine. See docs/developer-guide.md (The regression gate).
add_executable(warehousesim_regression_bench regression_bench.cpp)
target_include_directories(warehousesim_regression_bench PRIVATE "${CMAKE_SOURCE_DIR}/apps/common")
target_link_libraries(warehousesim_regression_bench PRIVATE warehousesim::simulation)
target_compile_definitions(warehousesim_regression_bench PRIVATE
  WAREHOUSESIM_REGRESSION_BASELINE="${CMAKE_CURRENT_SOURCE_DIR}/regression_baseline.txt")
warehousesim_configure_target(warehousesim_regression_bench)
# Counting allocations replaces operator new, which sanitizers do not appreciate.
if(NOT WAREHOUSESIM_SANITIZE)
  target_link_libraries(warehousesim_regression_bench PRIVATE warehousesim::testing_allocation)
else()
  target_link_libraries(warehousesim_regression_bench PRIVATE warehousesim::testing)
  target_sources(warehousesim_regression_bench PRIVATE alloc_stub.cpp)
endif()
if(WAREHOUSESIM_BUILD_TESTS)
  add_test(NAME regression_bench_quick COMMAND warehousesim_regression_bench --quick --repeats 1)
endif()

if(WAREHOUSESIM_BUILD_TESTS)
  add_test(NAME fleet_bench_smoke
    COMMAND warehousesim_fleet_bench --quick
            --csv ${CMAKE_CURRENT_BINARY_DIR}/fleet_bench_smoke.csv)
endif()
