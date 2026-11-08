# Header-only helpers shared by the tests and the benchmarks: a slow reference shortest-path
# search, deterministic map generators, and an allocation counter. Nothing in src/ may depend on
# this directory.
add_library(warehousesim_testing INTERFACE)
add_library(warehousesim::testing ALIAS warehousesim_testing)
target_include_directories(warehousesim_testing INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(warehousesim_testing INTERFACE warehousesim::planning)

# Separate target because it replaces operator new for whichever executable links it. The
# replacement is left out of sanitizer builds, which interpose their own allocator.
add_library(warehousesim_testing_allocation INTERFACE)
add_library(warehousesim::testing_allocation ALIAS warehousesim_testing_allocation)
target_link_libraries(warehousesim_testing_allocation INTERFACE warehousesim::testing)
target_sources(warehousesim_testing_allocation INTERFACE
  "${CMAKE_CURRENT_SOURCE_DIR}/allocation_counter.cpp")
