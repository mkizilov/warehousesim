#include "warehousesim/world/layout_check.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "warehousesim/world/grid_size.hpp"
#include "warehousesim/world/layouts.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::world::checkLayout;
using warehousesim::world::LayoutIssue;
using warehousesim::world::LayoutIssueKind;
using warehousesim::world::Warehouse;

Warehouse layout(const std::vector<std::string>& rows) {
  auto parsed = Warehouse::fromRows(rows);
  if (!parsed) {
    throw std::logic_error("bad test layout");
  }
  return *parsed;
}

std::vector<LayoutIssueKind> kindsOf(const std::vector<LayoutIssue>& issues) {
  std::vector<LayoutIssueKind> kinds;
  for (const LayoutIssue& issue : issues) {
    kinds.push_back(issue.kind);
  }
  return kinds;
}

TEST(LayoutCheckTest, EveryPredefinedLayoutIsReady) {
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    EXPECT_TRUE(checkLayout(info.make()).empty()) << info.name;
  }
}

TEST(LayoutCheckTest, ALayoutWithADropoffAndAPickupIsReady) {
  EXPECT_TRUE(checkLayout(layout({"P..D"})).empty());
}

TEST(LayoutCheckTest, ShelfAccessCountsAsASourceOfGoodsWhenThereIsNoPickup) {
  EXPECT_TRUE(checkLayout(layout({"S..D"})).empty());
}

TEST(LayoutCheckTest, AShelfThatSplitsTheFloorStillCountsAsASourceButLeavesTwoRegions) {
  const auto issues = checkLayout(layout({".S.D"}));
  EXPECT_EQ(kindsOf(issues), (std::vector<LayoutIssueKind>{LayoutIssueKind::DisconnectedRegions}));
}

TEST(LayoutCheckTest, ALayoutWithNoWalkableCellHasOnlyThatIssue) {
  const auto issues = checkLayout(layout({"#S", "S#"}));
  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues[0].kind, LayoutIssueKind::NoWalkableCells);
  EXPECT_TRUE(issues[0].blocksSimulation);
  EXPECT_FALSE(issues[0].where.has_value());
}

TEST(LayoutCheckTest, ALayoutWithoutADropoffCannotHostASimulation) {
  const auto issues = checkLayout(layout({"P..."}));
  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues[0].kind, LayoutIssueKind::NoDropoff);
  EXPECT_TRUE(warehousesim::world::blocksSimulation(issues));
}

TEST(LayoutCheckTest, ALayoutWithNothingToCollectCannotHostASimulation) {
  const auto issues = checkLayout(layout({"...D"}));
  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues[0].kind, LayoutIssueKind::NoGoodsSource);
  EXPECT_TRUE(warehousesim::world::blocksSimulation(issues));
}

TEST(LayoutCheckTest, AShelfWithNoFreeCellBesideItIsNotASourceOfGoods) {
  // The only free cell is the dropoff, which is not a free cell for shelf access.
  const auto issues = checkLayout(layout({"SD"}));
  EXPECT_EQ(kindsOf(issues), (std::vector<LayoutIssueKind>{LayoutIssueKind::NoGoodsSource}));
}

TEST(LayoutCheckTest, SeparateWalkableRegionsAreAWarningThatPointsAtTheSecondOne) {
  const auto issues = checkLayout(layout({"P.#.D"}));
  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues[0].kind, LayoutIssueKind::DisconnectedRegions);
  EXPECT_FALSE(issues[0].blocksSimulation);
  EXPECT_FALSE(warehousesim::world::blocksSimulation(issues));
  ASSERT_TRUE(issues[0].where.has_value());
  EXPECT_EQ(*issues[0].where, (Point{3, 0}));
  EXPECT_TRUE(issues[0].message.find("2 separate regions") != std::string::npos);
}

TEST(LayoutCheckTest, RegionsAreCountedAcrossRowsAndDiagonalsDoNotConnect) {
  const auto issues = checkLayout(layout({"P#.", "#.#", ".#D"}));
  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues[0].kind, LayoutIssueKind::DisconnectedRegions);
  EXPECT_TRUE(issues[0].message.find("5 separate regions") != std::string::npos);
}

TEST(LayoutCheckTest, IssuesComeInTheOrderOfTheirKind) {
  const auto issues = checkLayout(layout({".#."}));
  EXPECT_EQ(kindsOf(issues), (std::vector<LayoutIssueKind>{LayoutIssueKind::NoDropoff,
                                                           LayoutIssueKind::NoGoodsSource,
                                                           LayoutIssueKind::DisconnectedRegions}));
}

TEST(LayoutCheckTest, ALargeOpenLayoutIsCheckedWithoutRecursion) {
  Warehouse big(warehousesim::world::kMaxMapSide, warehousesim::world::kMaxMapSide);
  big.set({0, 0}, warehousesim::world::CellType::Pickup);
  big.set({1, 0}, warehousesim::world::CellType::Dropoff);
  EXPECT_TRUE(checkLayout(big).empty());
}

TEST(LayoutCheckTest, MapSidesAreValidFromOneToTheLimit) {
  using warehousesim::world::isValidMapSide;
  using warehousesim::world::kMaxMapSide;
  EXPECT_FALSE(isValidMapSide(0));
  EXPECT_TRUE(isValidMapSide(1));
  EXPECT_TRUE(isValidMapSide(kMaxMapSide));
  EXPECT_FALSE(isValidMapSide(kMaxMapSide + 1));
  EXPECT_FALSE(isValidMapSide(-5));
  EXPECT_FALSE(isValidMapSide(std::int64_t{1} << 40));
}

}  // namespace
