// Stands in for testing/allocation_counter.cpp in sanitizer builds, where replacing operator new is
// avoided: the benchmark then reports 0 allocations instead of counting them.
#include "warehousesim/testing/allocation_counter.hpp"

namespace warehousesim::testing {

std::uint64_t allocationCount() noexcept { return 0; }

}  // namespace warehousesim::testing
