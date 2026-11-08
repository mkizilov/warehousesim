#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

namespace warehousesim::core {

// 64-bit FNV-1a over a canonical byte stream, for telling whether two runs reached the same state.
// Every value is fed as eight little-endian bytes (or, for text, its length and then its bytes),
// whatever the host's byte order or the width of the type it came from, so equal state gives an
// equal checksum on every platform. A double goes in as its IEEE-754 bit pattern: the simulation's
// arithmetic is exact IEEE (see AGENTS.md), so equal runs give equal bits.
//
// It is not cryptographic. It detects accidental divergence between a run and its replay; it says
// nothing about whether a recording was tampered with.
class Checksum {
 public:
  static constexpr std::uint64_t kOffsetBasis = 14695981039346656037ULL;
  static constexpr std::uint64_t kPrime = 1099511628211ULL;

  template <std::integral T>
  constexpr void add(T value) noexcept {
    // Signed values are sign-extended first, so -1 is the same eight bytes from any width.
    if constexpr (std::is_signed_v<T>) {
      feed(static_cast<std::uint64_t>(static_cast<std::int64_t>(value)));
    } else {
      feed(static_cast<std::uint64_t>(value));
    }
  }

  template <typename E>
    requires std::is_enum_v<E>
  constexpr void add(E value) noexcept {
    add(static_cast<std::underlying_type_t<E>>(value));
  }

  constexpr void add(double value) noexcept { feed(std::bit_cast<std::uint64_t>(value)); }

  // The length first, so that ("ab", "c") and ("a", "bc") differ.
  constexpr void add(std::string_view text) noexcept {
    feed(static_cast<std::uint64_t>(text.size()));
    addBytes(text);
  }

  // The bytes as they are, with no length: what the published FNV test vectors are made of.
  constexpr void addBytes(std::string_view bytes) noexcept {
    for (const char byte : bytes) {
      state_ = (state_ ^ static_cast<unsigned char>(byte)) * kPrime;
    }
  }

  [[nodiscard]] constexpr std::uint64_t value() const noexcept { return state_; }

 private:
  constexpr void feed(std::uint64_t word) noexcept {
    for (std::size_t i = 0; i < 8; ++i) {
      state_ = (state_ ^ ((word >> (8 * i)) & 0xFFU)) * kPrime;
    }
  }

  std::uint64_t state_ = kOffsetBasis;
};

}  // namespace warehousesim::core
