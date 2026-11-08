// Replaces the global allocation functions to count allocations. Compile this into an executable
// only when it needs to measure allocations (the allocation test and the planning benchmark).
#include "warehousesim/testing/allocation_counter.hpp"

#include <atomic>
#include <cstdlib>
#include <new>

namespace {

std::atomic<std::uint64_t> gAllocations{0};

void* allocate(std::size_t size) {
  gAllocations.fetch_add(1, std::memory_order_relaxed);
  if (void* memory = std::malloc(size == 0 ? 1 : size)) {
    return memory;
  }
  throw std::bad_alloc();
}

}  // namespace

namespace warehousesim::testing {

std::uint64_t allocationCount() noexcept { return gAllocations.load(std::memory_order_relaxed); }

}  // namespace warehousesim::testing

void* operator new(std::size_t size) { return allocate(size); }
void* operator new[](std::size_t size) { return allocate(size); }
void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t) noexcept { std::free(memory); }
