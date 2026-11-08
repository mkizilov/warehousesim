#include "warehousesim/core/checksum.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <string_view>

#include "warehousesim/core/rng.hpp"

namespace {

using warehousesim::core::Checksum;
using warehousesim::core::Rng;

enum class Colour : std::uint8_t { Red = 1, Green = 2 };

std::uint64_t ofBytes(std::string_view bytes) {
  Checksum sum;
  sum.addBytes(bytes);
  return sum.value();
}

template <typename T>
std::uint64_t ofValue(T value) {
  Checksum sum;
  sum.add(value);
  return sum.value();
}

constexpr std::uint64_t constexprChecksumOfA() {
  Checksum sum;
  sum.addBytes("a");
  return sum.value();
}

// ---- the function itself ------------------------------------------------------------------------

TEST(ChecksumTest, MatchesThePublishedFnv1a64TestVectors) {
  EXPECT_EQ(ofBytes(""), 0xcbf29ce484222325ULL);
  EXPECT_EQ(ofBytes("a"), 0xaf63dc4c8601ec8cULL);
  EXPECT_EQ(ofBytes("foobar"), 0x85944171f73967e8ULL);
  EXPECT_EQ(Checksum{}.value(), Checksum::kOffsetBasis);
}

TEST(ChecksumTest, CanBeEvaluatedAtCompileTime) {
  static_assert(constexprChecksumOfA() == 0xaf63dc4c8601ec8cULL);
  EXPECT_EQ(constexprChecksumOfA(), 0xaf63dc4c8601ec8cULL);
}

// ---- a canonical byte stream --------------------------------------------------------------------

TEST(ChecksumTest, AnIntegerIsEightLittleEndianBytesWhateverTheHost) {
  EXPECT_EQ(ofValue(std::uint64_t{0x0102030405060708ULL}),
            ofBytes(std::string_view("\x08\x07\x06\x05\x04\x03\x02\x01", 8)));
  EXPECT_EQ(ofValue(std::uint64_t{0}), ofBytes(std::string_view("\0\0\0\0\0\0\0\0", 8)));
}

TEST(ChecksumTest, TheSameNumberIsTheSameWhateverTheWidthOfItsType) {
  EXPECT_EQ(ofValue(std::int8_t{-1}), ofValue(std::int64_t{-1}));
  EXPECT_EQ(ofValue(std::int16_t{-1}), ofValue(-1));
  EXPECT_EQ(ofValue(std::int32_t{-300}), ofValue(std::int64_t{-300}));
  EXPECT_EQ(ofValue(std::uint8_t{7}), ofValue(std::uint64_t{7}));
  EXPECT_EQ(ofValue(std::uint32_t{7}), ofValue(7));
  EXPECT_EQ(ofValue(true), ofValue(std::uint64_t{1}));
  EXPECT_EQ(ofValue(false), ofValue(std::uint64_t{0}));
}

TEST(ChecksumTest, SignedAndUnsignedValuesWithTheSameBitsAreToldApart) {
  // -1 is sign-extended, 0xFFFFFFFF is not: they are different numbers.
  EXPECT_NE(ofValue(-1), ofValue(std::uint32_t{0xFFFFFFFFU}));
  EXPECT_EQ(ofValue(-1), ofValue(std::uint64_t{0xFFFFFFFFFFFFFFFFULL}));
}

TEST(ChecksumTest, AnEnumIsItsUnderlyingNumber) {
  EXPECT_EQ(ofValue(Colour::Green), ofValue(std::uint8_t{2}));
  EXPECT_NE(ofValue(Colour::Green), ofValue(Colour::Red));
}

TEST(ChecksumTest, ADoubleIsItsIeeeBitPattern) {
  EXPECT_EQ(ofValue(1.5), ofValue(std::uint64_t{0x3FF8000000000000ULL}));
  EXPECT_NE(ofValue(0.0), ofValue(-0.0));  // different bits: different state
  EXPECT_NE(ofValue(0.1 + 0.2), ofValue(0.3));
  EXPECT_EQ(ofValue(std::numeric_limits<double>::infinity()),
            ofValue(std::uint64_t{0x7FF0000000000000ULL}));
}

TEST(ChecksumTest, TextCarriesItsLengthSoRunsOfTextCannotBlurTogether) {
  Checksum a;
  a.add(std::string_view("ab"));
  a.add(std::string_view("c"));
  Checksum b;
  b.add(std::string_view("a"));
  b.add(std::string_view("bc"));
  EXPECT_NE(a.value(), b.value());

  Checksum empty;
  empty.add(std::string_view(""));
  EXPECT_NE(empty.value(), Checksum{}.value());  // even nothing is something that was said
  Checksum viaString;
  viaString.add(std::string_view(std::string("ab")));
  Checksum viaLiteral;
  viaLiteral.add("ab");
  EXPECT_EQ(viaString.value(), viaLiteral.value());
}

TEST(ChecksumTest, ValuesFedInAnotherOrderGiveAnotherChecksum) {
  Checksum ab;
  ab.add(1);
  ab.add(2);
  Checksum ba;
  ba.add(2);
  ba.add(1);
  EXPECT_NE(ab.value(), ba.value());
}

TEST(ChecksumTest, EqualInputGivesEqualOutputAndAnyChangeShowsInTheOutput) {
  const auto run = [](std::uint64_t last) {
    Checksum sum;
    for (std::uint64_t i = 0; i < 100; ++i) {
      sum.add(i * i);
    }
    sum.add(last);
    return sum.value();
  };
  EXPECT_EQ(run(5), run(5));
  EXPECT_NE(run(5), run(6));
}

// ---- the random stream's state, which the simulation's checksum includes ------------------------

TEST(RngStateTest, StartsAtTheSeedAndMovesWithEveryNumber) {
  Rng rng(42);
  EXPECT_EQ(rng.state(), 42U);
  const std::uint64_t before = rng.state();
  (void)rng.next();
  EXPECT_NE(rng.state(), before);
}

TEST(RngStateTest, EqualStatesGiveEqualNumbersFromThenOn) {
  Rng a(7);
  Rng b(7);
  for (int i = 0; i < 10; ++i) {
    (void)a.next();
    (void)b.next();
  }
  ASSERT_EQ(a.state(), b.state());
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(a.below(1000), b.below(1000));
    EXPECT_EQ(a.state(), b.state());
  }
  (void)a.next();
  EXPECT_NE(a.state(), b.state());
}

}  // namespace
