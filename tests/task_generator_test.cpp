#include "warehousesim/tasks/task_generator.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/tasks/task.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::tasks::TaskGenerator;
using warehousesim::tasks::TaskPriority;

Point P(int x, int y) { return Point{x, y}; }

TEST(TaskGeneratorTest, RejectsEmptyLocationLists) {
  EXPECT_THROW(TaskGenerator({}, {P(0, 0)}, 1), std::invalid_argument);
  EXPECT_THROW(TaskGenerator({P(0, 0)}, {}, 1), std::invalid_argument);
}

TEST(TaskGeneratorTest, SameSeedReproducesRequestsAndStaysWithinLocations) {
  const std::vector<Point> pickups = {P(1, 0), P(2, 0), P(3, 0)};
  const std::vector<Point> dropoffs = {P(0, 5), P(1, 5)};
  TaskGenerator a(pickups, dropoffs, 99, true);
  TaskGenerator b(pickups, dropoffs, 99, true);
  const std::set<Point> allowedPickups(pickups.begin(), pickups.end());
  const std::set<Point> allowedDropoffs(dropoffs.begin(), dropoffs.end());
  for (int i = 0; i < 50; ++i) {
    const auto left = a.next();
    const auto right = b.next();
    EXPECT_EQ(left.pickup, right.pickup);
    EXPECT_EQ(left.dropoff, right.dropoff);
    EXPECT_TRUE(left.priority == right.priority);
    EXPECT_TRUE(allowedPickups.contains(left.pickup));
    EXPECT_TRUE(allowedDropoffs.contains(left.dropoff));
  }
}

TEST(TaskGeneratorTest, WithoutVaryingPriorityEveryRequestIsNormal) {
  TaskGenerator generator({P(0, 0), P(1, 0)}, {P(5, 5)}, 3);
  for (int i = 0; i < 200; ++i) {
    EXPECT_TRUE(generator.next().priority == TaskPriority::Normal);
  }
}

TEST(TaskGeneratorTest, VaryingPriorityDoesNotChangeTheFirstRequestsLocations) {
  // Priorities are drawn after the locations, so the first request is the same either way;
  // later ones diverge because the random stream has advanced by one extra draw each.
  const std::vector<Point> pickups = {P(1, 0), P(2, 0), P(3, 0), P(4, 0)};
  const std::vector<Point> dropoffs = {P(0, 5), P(1, 5), P(2, 5)};
  TaskGenerator plain(pickups, dropoffs, 17, false);
  TaskGenerator varied(pickups, dropoffs, 17, true);
  const auto a = plain.next();
  const auto b = varied.next();
  EXPECT_EQ(a.pickup, b.pickup);
  EXPECT_EQ(a.dropoff, b.dropoff);
}

TEST(TaskGeneratorTest, PriorityMixMatchesTheDocumentedWeights) {
  TaskGenerator generator({P(0, 0)}, {P(1, 0)}, 2024, true);
  std::array<int, 4> counts{};
  constexpr int kDraws = 20000;
  for (int i = 0; i < kDraws; ++i) {
    ++counts[static_cast<std::size_t>(generator.next().priority)];
  }
  // Expected 15% / 60% / 20% / 5%, with a generous tolerance for a fixed seed.
  EXPECT_NEAR(counts[0] / static_cast<double>(kDraws), 0.15, 0.015);
  EXPECT_NEAR(counts[1] / static_cast<double>(kDraws), 0.60, 0.015);
  EXPECT_NEAR(counts[2] / static_cast<double>(kDraws), 0.20, 0.015);
  EXPECT_NEAR(counts[3] / static_cast<double>(kDraws), 0.05, 0.015);
}

}  // namespace
