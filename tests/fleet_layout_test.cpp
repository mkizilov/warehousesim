#include "warehousesim/view/fleet_layout.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "warehousesim/core/rng.hpp"

namespace {

using warehousesim::view::computeLayout;
using warehousesim::view::FleetCounts;
using warehousesim::view::FleetLayout;
using warehousesim::view::hitTest;
using warehousesim::view::kHelpLines;
using warehousesim::view::kInspectorLines;
using warehousesim::view::kLegendLines;
using warehousesim::view::kMaxRobotRows;
using warehousesim::view::kPanelLineHeight;
using warehousesim::view::kPanelMinWindowHeight;
using warehousesim::view::kPanelMinWindowWidth;
using warehousesim::view::kPanelWidth;
using warehousesim::view::kPromptLines;
using warehousesim::view::PanelHit;
using warehousesim::view::ScreenPoint;
using warehousesim::view::ScreenRect;
using warehousesim::view::ScreenSize;

bool overlap(const ScreenRect& a, const ScreenRect& b) {
  return a.x < b.right() && b.x < a.right() && a.y < b.bottom() && b.y < a.bottom();
}

ScreenPoint centre(const ScreenRect& r) { return {r.x + r.width / 2.0, r.y + r.height / 2.0}; }

// ---- where the panel goes -----------------------------------------------------------------------

TEST(FleetLayoutTest, ThePanelTakesTheRightHandSideAndTheMapTheRest) {
  const FleetLayout layout = computeLayout(ScreenSize{1280, 720}, {4, 3, 2});
  ASSERT_TRUE(layout.hasPanel());
  EXPECT_EQ(layout.canvas.width + static_cast<int>(layout.panel.width), 1280);
  EXPECT_EQ(layout.canvas.height, 720);
  EXPECT_DOUBLE_EQ(layout.panel.x, static_cast<double>(layout.canvas.width));
  EXPECT_DOUBLE_EQ(layout.panel.width, kPanelWidth);
  EXPECT_DOUBLE_EQ(layout.panel.y, 0.0);
  EXPECT_DOUBLE_EQ(layout.panel.height, 720.0);
}

TEST(FleetLayoutTest, ATooSmallWindowHasNoPanelAndTheMapKeepsAllOfIt) {
  for (const ScreenSize size : {ScreenSize{719, 720}, ScreenSize{1280, 399}, ScreenSize{100, 100},
                                ScreenSize{0, 0}, ScreenSize{-5, -5}}) {
    const FleetLayout layout = computeLayout(size, {4, 3, 2});
    EXPECT_FALSE(layout.hasPanel()) << size.width << "x" << size.height;
    EXPECT_EQ(layout.canvas.width, std::max(size.width, 0));
    EXPECT_EQ(layout.canvas.height, std::max(size.height, 0));
    EXPECT_TRUE(layout.robotRows.empty());
    EXPECT_TRUE(layout.activeRows.empty());
    EXPECT_TRUE(layout.finishedRows.empty());
  }
}

TEST(FleetLayoutTest, TheSmallestWindowWithAPanelIsExactlyTheLimits) {
  EXPECT_TRUE(computeLayout(ScreenSize{static_cast<int>(kPanelMinWindowWidth),
                                       static_cast<int>(kPanelMinWindowHeight)},
                            {})
                  .hasPanel());
  EXPECT_FALSE(computeLayout(ScreenSize{static_cast<int>(kPanelMinWindowWidth) - 1,
                                        static_cast<int>(kPanelMinWindowHeight)},
                             {})
                   .hasPanel());
  EXPECT_FALSE(computeLayout(ScreenSize{static_cast<int>(kPanelMinWindowWidth),
                                        static_cast<int>(kPanelMinWindowHeight) - 1},
                             {})
                   .hasPanel());
}

TEST(FleetLayoutTest, TheBlocksOfThePanelRunTopToBottomInOrderAndEndInsideTheWindow) {
  const FleetLayout layout = computeLayout(ScreenSize{1280, 720}, {4, 6, 4});
  EXPECT_LT(layout.titleY, layout.robotsY);
  EXPECT_LT(layout.robotsY, layout.inspectorY);
  EXPECT_LT(layout.inspectorY, layout.activeY);
  EXPECT_LT(layout.activeY, layout.finishedY);
  EXPECT_LT(layout.finishedY, layout.promptY);
  EXPECT_LT(layout.promptY, layout.legendY);
  EXPECT_LT(layout.legendY, layout.helpY);
  EXPECT_LE(layout.helpY + kPanelLineHeight * static_cast<double>(kHelpLines), 720.0);
  // The prompt, the legend and the help are pinned to the bottom, back to back.
  EXPECT_DOUBLE_EQ(layout.promptY + kPanelLineHeight * static_cast<double>(kPromptLines),
                   layout.legendY);
  EXPECT_DOUBLE_EQ(layout.legendY + kPanelLineHeight * static_cast<double>(kLegendLines),
                   layout.helpY);
}

// ---- rows ---------------------------------------------------------------------------------------

TEST(FleetLayoutTest, EachRobotGetsARowUpToTheLimit) {
  EXPECT_EQ(computeLayout(ScreenSize{1280, 720}, {0, 0, 0}).robotRows.size(), 0U);
  EXPECT_EQ(computeLayout(ScreenSize{1280, 720}, {1, 0, 0}).robotRows.size(), 1U);
  EXPECT_EQ(computeLayout(ScreenSize{1280, 720}, {kMaxRobotRows, 0, 0}).robotRows.size(),
            kMaxRobotRows);
  EXPECT_EQ(computeLayout(ScreenSize{1280, 720}, {kMaxRobotRows + 5, 0, 0}).robotRows.size(),
            kMaxRobotRows);
}

TEST(FleetLayoutTest, TheRowsOfEachListAreOneLineHighStackedAndInsideThePanel) {
  const FleetLayout layout = computeLayout(ScreenSize{1280, 720}, {4, 5, 5});
  for (const auto* rows : {&layout.robotRows, &layout.activeRows, &layout.finishedRows}) {
    for (std::size_t i = 0; i < rows->size(); ++i) {
      const ScreenRect& row = (*rows)[i];
      EXPECT_DOUBLE_EQ(row.height, kPanelLineHeight);
      EXPECT_DOUBLE_EQ(row.x, layout.panel.x);
      EXPECT_DOUBLE_EQ(row.width, layout.panel.width);
      if (i > 0) {
        EXPECT_DOUBLE_EQ(row.y, (*rows)[i - 1].bottom());  // back to back
      }
    }
  }
}

TEST(FleetLayoutTest, EveryTaskGetsARowWhenThereIsRoom) {
  const FleetLayout layout = computeLayout(ScreenSize{1280, 900}, {4, 3, 2});
  EXPECT_EQ(layout.activeRows.size(), 3U);
  EXPECT_EQ(layout.finishedRows.size(), 2U);
}

TEST(FleetLayoutTest, WhenRoomRunsOutTheFinishedListGivesWayFirstButKeepsAThird) {
  // Plenty of both: the active list takes up to two thirds of the room, the finished list the rest.
  const FleetLayout layout = computeLayout(ScreenSize{1280, 520}, {4, 100, 100});
  EXPECT_GT(layout.activeRows.size(), 0U);
  EXPECT_GT(layout.finishedRows.size(), 0U);
  EXPECT_GT(layout.activeRows.size(), layout.finishedRows.size());
  const std::size_t total = layout.activeRows.size() + layout.finishedRows.size();
  EXPECT_EQ(layout.finishedRows.size(), total / 3);  // a third of the room, rounded down

  // Almost nothing waiting: the finished list uses the room the active list does not need.
  const FleetLayout quiet = computeLayout(ScreenSize{1280, 520}, {4, 1, 100});
  EXPECT_EQ(quiet.activeRows.size(), 1U);
  EXPECT_EQ(quiet.activeRows.size() + quiet.finishedRows.size(), total);
}

TEST(FleetLayoutTest, RowsNeverReachIntoThePromptOrBelow) {
  for (int height = 400; height <= 1400; height += 37) {
    const FleetLayout layout = computeLayout(ScreenSize{1280, height}, {8, 500, 500});
    for (const auto* rows : {&layout.activeRows, &layout.finishedRows}) {
      for (const ScreenRect& row : *rows) {
        EXPECT_LE(row.bottom(), layout.promptY + 1e-9) << height;
      }
    }
  }
}

TEST(FleetLayoutTest, NoTwoRowsOrBlocksOfThePanelOverlapAtAnyWindowSizeItShowsThePanelAt) {
  warehousesim::core::Rng rng(21);
  for (int round = 0; round < 300; ++round) {
    const ScreenSize size{720 + static_cast<int>(rng.below(1200)),
                          400 + static_cast<int>(rng.below(800))};
    const FleetCounts counts{rng.below(12), rng.below(60), rng.below(60)};
    const FleetLayout layout = computeLayout(size, counts);
    ASSERT_TRUE(layout.hasPanel());
    std::vector<ScreenRect> all;
    for (const auto* rows : {&layout.robotRows, &layout.activeRows, &layout.finishedRows}) {
      all.insert(all.end(), rows->begin(), rows->end());
    }
    for (std::size_t i = 0; i < all.size(); ++i) {
      EXPECT_GE(all[i].y, 0.0);
      EXPECT_LE(all[i].bottom(), static_cast<double>(size.height));
      EXPECT_GE(all[i].x, layout.panel.x);
      EXPECT_LE(all[i].right(), layout.panel.right() + 1e-9);
      for (std::size_t j = i + 1; j < all.size(); ++j) {
        EXPECT_FALSE(overlap(all[i], all[j]))
            << size.width << "x" << size.height << " rows " << i << "," << j;
      }
    }
    EXPECT_LE(layout.robotRows.size(), std::min(counts.robots, kMaxRobotRows));
    EXPECT_LE(layout.activeRows.size(), counts.activeTasks);
    EXPECT_LE(layout.finishedRows.size(), counts.finishedTasks);
    // The inspector sits between the robot list and the task lists, with its whole block.
    if (!layout.robotRows.empty()) {
      EXPECT_GE(layout.inspectorY, layout.robotRows.back().bottom());
    }
    EXPECT_GE(layout.activeY,
              layout.inspectorY + kPanelLineHeight * static_cast<double>(kInspectorLines));
  }
}

// ---- finding what was clicked -------------------------------------------------------------------

TEST(FleetLayoutTest, AClickInARowFindsThatRowOfThatList) {
  const FleetLayout layout = computeLayout(ScreenSize{1280, 900}, {3, 4, 2});
  for (std::size_t i = 0; i < layout.robotRows.size(); ++i) {
    EXPECT_EQ(hitTest(layout, centre(layout.robotRows[i])), (PanelHit{PanelHit::Kind::Robot, i}));
  }
  for (std::size_t i = 0; i < layout.activeRows.size(); ++i) {
    EXPECT_EQ(hitTest(layout, centre(layout.activeRows[i])),
              (PanelHit{PanelHit::Kind::ActiveTask, i}));
  }
  for (std::size_t i = 0; i < layout.finishedRows.size(); ++i) {
    EXPECT_EQ(hitTest(layout, centre(layout.finishedRows[i])),
              (PanelHit{PanelHit::Kind::FinishedTask, i}));
  }
}

TEST(FleetLayoutTest, TheEdgesOfARowBelongToItAndTheLineBelowTheLastDoesNotExist) {
  const FleetLayout layout = computeLayout(ScreenSize{1280, 900}, {3, 4, 2});
  const ScreenRect& row = layout.robotRows[1];
  EXPECT_EQ(hitTest(layout, {row.x, row.y}), (PanelHit{PanelHit::Kind::Robot, 1}));
  EXPECT_EQ(hitTest(layout, {row.right() - 0.01, row.bottom() - 0.01}),
            (PanelHit{PanelHit::Kind::Robot, 1}));
  EXPECT_FALSE(hitTest(layout, {row.right(), row.y}).has_value());
  const ScreenRect& last = layout.robotRows.back();
  EXPECT_FALSE(hitTest(layout, {last.x + 5.0, last.bottom() + 0.01})
                   .has_value());  // the gap before the next block
}

TEST(FleetLayoutTest, HeadersTheMapAndNonsenseHitNothing) {
  const FleetLayout layout = computeLayout(ScreenSize{1280, 900}, {3, 4, 2});
  EXPECT_FALSE(
      hitTest(layout, {layout.panel.x + 5.0, layout.robotsY + 1.0}).has_value());  // a header
  EXPECT_FALSE(hitTest(layout, {10.0, 10.0}).has_value());                         // the map
  EXPECT_FALSE(hitTest(layout, {std::numeric_limits<double>::quiet_NaN(), 10.0}).has_value());
  EXPECT_FALSE(hitTest(layout, {10.0, std::numeric_limits<double>::infinity()}).has_value());
  EXPECT_FALSE(hitTest(layout, {-1e300, -1e300}).has_value());
  EXPECT_FALSE(hitTest(computeLayout(ScreenSize{300, 300}, {3, 4, 2}), {10.0, 10.0}).has_value());
}

}  // namespace
