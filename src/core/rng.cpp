#include "warehousesim/core/rng.hpp"

namespace warehousesim::core {

std::uint64_t Rng::next() noexcept {
  state_ += 0x9E3779B97F4A7C15ULL;
  std::uint64_t z = state_;
  z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31U);
}

std::uint64_t Rng::below(std::uint64_t bound) noexcept {
  if (bound <= 1) {
    return 0;
  }
  // Reject the low `2^64 mod bound` values so every residue is equally likely.
  const std::uint64_t threshold = (~bound + 1U) % bound;
  std::uint64_t value = next();
  while (value < threshold) {
    value = next();
  }
  return value % bound;
}

}  // namespace warehousesim::core
