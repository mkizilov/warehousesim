#pragma once

#include <cstdint>

namespace warehousesim::core {

// SplitMix64. Its output is fully specified, so a seed reproduces the same
// sequence on every platform (std::*_distribution does not guarantee that).
class Rng {
 public:
  explicit Rng(std::uint64_t seed) noexcept : state_(seed) {}

  [[nodiscard]] std::uint64_t next() noexcept;

  // Unbiased value in [0, bound). Returns 0 when bound <= 1.
  [[nodiscard]] std::uint64_t below(std::uint64_t bound) noexcept;

  // Where the stream stands: two generators with equal state give equal numbers from here on. For
  // comparing runs, not for seeding.
  [[nodiscard]] std::uint64_t state() const noexcept { return state_; }

 private:
  std::uint64_t state_;
};

}  // namespace warehousesim::core
