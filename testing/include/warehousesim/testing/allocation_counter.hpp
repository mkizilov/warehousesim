#pragma once

#include <cstdint>

namespace warehousesim::testing {

// Number of calls to the global operator new since the program started. Only meaningful in a
// target that also compiles allocation_counter.cpp (which replaces the global allocation
// functions); see the warehousesim_testing_allocation CMake target.
[[nodiscard]] std::uint64_t allocationCount() noexcept;

}  // namespace warehousesim::testing
