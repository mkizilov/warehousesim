#include "warehousesim/planning/space_time_astar.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::core::Tick;
using warehousesim::planning::SpaceTimeAStar;
using warehousesim::planning::SpaceTimeConstraints;
using warehousesim::planning::SpaceTimeOptions;
using warehousesim::planning::SpaceTimePlan;
using warehousesim::planning::SpaceTimeStatus;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }

Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

// A script of what is taken: cells for ranges of steps, and edges in one direction at one step.
class Script final : public SpaceTimeConstraints {
 public:
  // Nobody else may use `cell` during [from, to).
  void takeCell(Point cell, Tick from, Tick to) { cells_.push_back({cell, from, to}); }
  // The robot may not go from `from` to `to` during `step`.
  void takeEdge(Point from, Point to, Tick step) { edges_.insert({from, to, step}); }

  [[nodiscard]] bool cellFree(Point cell, Tick step) const override {
    return std::none_of(cells_.begin(), cells_.end(), [&](const Taken& t) {
      return t.cell == cell && t.from <= step && step < t.to;
    });
  }
  [[nodiscard]] bool edgeFree(Point from, Point to, Tick step) const override {
    return !edges_.contains(std::tuple<Point, Point, Tick>{from, to, step});
  }

 private:
  struct Taken {
    Point cell;
    Tick from;
    Tick to;
  };
  std::vector<Taken> cells_;
  std::set<std::tuple<Point, Point, Tick>> edges_;
};

constexpr Tick kForever = 1'000'000;

SpaceTimePlan search(const Warehouse& w, Point start, Point goal, const Script& script,
                     std::size_t window = 12, Tick startStep = 0, std::size_t maxWork = 0,
                     bool goalIsTerminal = false) {
  SpaceTimeAStar planner;
  return planner.search(w, start, goal, startStep, script,
                        SpaceTimeOptions{window, maxWork, goalIsTerminal});
}

bool visits(const SpaceTimePlan& plan, Point cell) {
  return std::find(plan.timeline.begin(), plan.timeline.end(), cell) != plan.timeline.end();
}

// Every step of a plan is a wait or a move to a neighbour, onto a walkable cell.
void expectLegal(const Warehouse& w, const SpaceTimePlan& plan, Point start) {
  ASSERT_FALSE(plan.timeline.empty());
  EXPECT_EQ(plan.timeline.front(), start);
  for (std::size_t i = 0; i < plan.timeline.size(); ++i) {
    EXPECT_TRUE(w.isWalkable(plan.timeline[i]));
    if (i > 0) {
      EXPECT_LE(warehousesim::core::manhattanDistance(plan.timeline[i - 1], plan.timeline[i]), 1);
    }
  }
  EXPECT_EQ(plan.moves + plan.waits, plan.duration());
}

// ---- the plain cases -----------------------------------------------------------------------

TEST(SpaceTimeAStarTest, WithNothingTakenTheRouteIsAShortestOneWithoutWaiting) {
  const Warehouse w = layout({"........", "..####..", "........"});
  const SpaceTimePlan plan = search(w, P(0, 0), P(7, 2), Script{});
  ASSERT_EQ(plan.status, SpaceTimeStatus::Found);
  expectLegal(w, plan, P(0, 0));
  EXPECT_EQ(plan.timeline.back(), P(7, 2));
  EXPECT_EQ(plan.duration(), 9U);  // Manhattan distance, and the wall is not in the way
  EXPECT_EQ(plan.waits, 0U);
  EXPECT_EQ(plan.remaining, 0);
  EXPECT_GT(plan.work, 0U);
}

TEST(SpaceTimeAStarTest, StartingOnTheGoalNeedsNoSteps) {
  const Warehouse w = layout({"...."});
  const SpaceTimePlan plan = search(w, P(2, 0), P(2, 0), Script{});
  ASSERT_EQ(plan.status, SpaceTimeStatus::Found);
  EXPECT_EQ(plan.duration(), 0U);
  EXPECT_EQ(plan.timeline.size(), 1U);
}

TEST(SpaceTimeAStarTest, RejectsAStartOrGoalThatIsNotWalkable) {
  const Warehouse w = layout({".#.", "..."});
  EXPECT_EQ(search(w, P(1, 0), P(2, 1), Script{}).status, SpaceTimeStatus::InvalidStart);
  EXPECT_EQ(search(w, P(0, 0), P(1, 0), Script{}).status, SpaceTimeStatus::InvalidGoal);
  EXPECT_EQ(search(w, P(-1, 0), P(2, 1), Script{}).status, SpaceTimeStatus::InvalidStart);
  EXPECT_EQ(search(w, P(0, 0), P(9, 9), Script{}).status, SpaceTimeStatus::InvalidGoal);
}

TEST(SpaceTimeAStarTest, AGoalWalledOffOnTheMapIsNoPlan) {
  const Warehouse w = layout({"..#..", "..#.."});
  const SpaceTimePlan plan = search(w, P(0, 0), P(4, 0), Script{});
  EXPECT_EQ(plan.status, SpaceTimeStatus::NoPlan);
  EXPECT_TRUE(plan.timeline.empty());
}

TEST(SpaceTimeAStarTest, ABadWindowIsRefused) {
  const Warehouse w = layout({"..."});
  SpaceTimeAStar planner;
  const Script script;
  EXPECT_THROW((void)planner.search(w, P(0, 0), P(2, 0), 0, script, SpaceTimeOptions{0, 0}),
               std::invalid_argument);
  EXPECT_THROW((void)planner.search(w, P(0, 0), P(2, 0), 0, script, SpaceTimeOptions{257, 0}),
               std::invalid_argument);
}

// ---- waiting and detouring -------------------------------------------------------------------

TEST(SpaceTimeAStarTest, AnotherRobotCrossingTheRouteMakesTheRobotWaitForItToPass) {
  const Warehouse w = layout({"......."});
  Script script;
  script.takeCell(P(3, 0), 2, 5);  // somebody is on (3, 0) during steps 2, 3 and 4
  const SpaceTimePlan plan = search(w, P(0, 0), P(6, 0), script);
  ASSERT_EQ(plan.status, SpaceTimeStatus::Found);
  expectLegal(w, plan, P(0, 0));
  // It cannot set foot on (3, 0) before it is free for two steps in a row (5 and 6), so it enters
  // at step 6: three steps of waiting.
  EXPECT_EQ(plan.duration(), 9U);
  EXPECT_EQ(plan.waits, 3U);
  EXPECT_EQ(plan.moves, 6U);
  for (std::size_t i = 0; i < plan.timeline.size(); ++i) {
    if (plan.timeline[i] == P(3, 0)) {
      EXPECT_GE(i, 5U);  // never holds it while it is taken
    }
  }
}

TEST(SpaceTimeAStarTest, ARouteBlockedForGoodIsGoneAroundWhenThereIsAWayRound) {
  const Warehouse w = layout({".....", ".....", "....."});
  Script script;
  script.takeCell(P(2, 1), 0, kForever);
  const SpaceTimePlan plan = search(w, P(0, 1), P(4, 1), script);
  ASSERT_EQ(plan.status, SpaceTimeStatus::Found);
  EXPECT_FALSE(visits(plan, P(2, 1)));
  EXPECT_EQ(plan.duration(), 6U);  // two cells longer than the straight line
  EXPECT_EQ(plan.waits, 0U);
}

TEST(SpaceTimeAStarTest, WhenWaitingAndDetouringTakeAsLongTheRobotWaitsInsteadOfWandering) {
  const Warehouse w = layout({".....", ".....", "....."});
  Script script;
  script.takeCell(P(2, 1), 1, 3);  // gone again by step 3
  const SpaceTimePlan plan = search(w, P(0, 1), P(4, 1), script);
  ASSERT_EQ(plan.status, SpaceTimeStatus::Found);
  // Straight on with two steps of waiting (6) is as fast as going round (6): fewer moves wins.
  EXPECT_EQ(plan.duration(), 6U);
  EXPECT_EQ(plan.moves, 4U);
  EXPECT_EQ(plan.waits, 2U);
}

TEST(SpaceTimeAStarTest, AnEdgeTakenInTheOtherDirectionIsNotUsedAtThatStep) {
  const Warehouse w = layout({"..."});
  Script script;
  script.takeEdge(P(0, 0), P(1, 0), 0);
  const SpaceTimePlan plan = search(w, P(0, 0), P(2, 0), script);
  ASSERT_EQ(plan.status, SpaceTimeStatus::Found);
  EXPECT_EQ(plan.duration(), 3U);
  EXPECT_EQ(plan.waits, 1U);
  EXPECT_EQ(plan.timeline[1], P(0, 0));  // it waited first
}

TEST(SpaceTimeAStarTest, MovingIntoACellNeedsItFreeForTheStepAfterTooBecauseTheRobotHoldsIt) {
  const Warehouse w = layout({"..."});
  Script script;
  script.takeCell(P(1, 0), 1, 2);  // taken during step 1 only
  const SpaceTimePlan plan = search(w, P(0, 0), P(2, 0), script);
  ASSERT_EQ(plan.status, SpaceTimeStatus::Found);
  // Moving at step 0 would hold (1, 0) during steps 0 and 1; moving at step 1 during 1 and 2: both
  // touch the taken step, so the earliest move is at step 2.
  EXPECT_EQ(plan.duration(), 4U);
  EXPECT_EQ(plan.waits, 2U);
}

TEST(SpaceTimeAStarTest, TheGoalMustBeKeptFreeToTheEndOfTheWindowSoTheRobotDoesNotParkInAWay) {
  const Warehouse w = layout({"......"});
  Script script;
  script.takeCell(P(5, 0), 8, 9);  // somebody needs the goal during step 8
  const SpaceTimePlan plan = search(w, P(0, 0), P(5, 0), script);
  ASSERT_EQ(plan.status, SpaceTimeStatus::Found);
  // Arriving by step 7 would mean being thrown out at step 8; the first arrival it can keep is by
  // a move at step 9 (free at 9 and 10), which ends at step 10.
  EXPECT_EQ(plan.duration(), 10U);
  EXPECT_EQ(plan.timeline[8], P(4, 0));
}

TEST(SpaceTimeAStarTest, StartingOnAGoalSomebodyWillNeedMeansLeavingAndComingBack) {
  const Warehouse w = layout({"...."});
  Script script;
  script.takeCell(P(1, 0), 5, 6);
  const SpaceTimePlan plan = search(w, P(1, 0), P(1, 0), script);
  ASSERT_EQ(plan.status, SpaceTimeStatus::Found);
  expectLegal(w, plan, P(1, 0));
  EXPECT_EQ(plan.duration(), 7U);  // back by a move at step 6 (free at 6 and 7)
  EXPECT_GT(plan.moves, 0U);
}

TEST(SpaceTimeAStarTest, WithATerminalGoalTheRouteNeverPassesThroughItOnTheWayElsewhere) {
  // Somebody needs (1, 0) from step 5 on, so it cannot be kept to the end of the window. A route
  // that is free to pass through it may do so and come back; a terminal goal may not.
  const Warehouse w = layout({"....", "...."});
  Script script;
  script.takeCell(P(1, 0), 5, kForever);
  const SpaceTimePlan open = search(w, P(0, 0), P(1, 0), script, 8);
  EXPECT_EQ(open.status, SpaceTimeStatus::Partial);  // it can never stay on it
  const SpaceTimePlan terminal = search(w, P(0, 0), P(1, 0), script, 8, 0, 0, true);
  ASSERT_EQ(terminal.status, SpaceTimeStatus::Partial);
  EXPECT_FALSE(visits(terminal, P(1, 0)));  // and so does not go through it either

  Script later;
  later.takeCell(P(1, 0), 5, 6);
  const SpaceTimePlan free = search(w, P(0, 0), P(1, 0), later, 10);
  const SpaceTimePlan stops = search(w, P(0, 0), P(1, 0), later, 10, 0, 0, true);
  ASSERT_EQ(free.status, SpaceTimeStatus::Found);
  ASSERT_EQ(stops.status, SpaceTimeStatus::Found);
  EXPECT_EQ(stops.duration(), free.duration());
  EXPECT_EQ(std::count(stops.timeline.begin(), stops.timeline.end(), P(1, 0)), 1);
  EXPECT_EQ(stops.timeline.back(), P(1, 0));
}

TEST(SpaceTimeAStarTest, AStartOnATerminalGoalThatCannotBeKeptHasNowhereToGo) {
  const Warehouse w = layout({"...."});
  Script script;
  script.takeCell(P(1, 0), 5, 6);
  const SpaceTimePlan open = search(w, P(1, 0), P(1, 0), script);
  EXPECT_EQ(open.status, SpaceTimeStatus::Found);  // may leave and come back
  const SpaceTimePlan terminal = search(w, P(1, 0), P(1, 0), script, 12, 0, 0, true);
  EXPECT_EQ(terminal.status, SpaceTimeStatus::NoPlan);
}

// ---- passing in a corridor -------------------------------------------------------------------

//   .......   one-cell-wide corridor (row 1) with a niche at (5, 0)
//   #####.#
Warehouse corridorWithNiche() { return layout({"#####.#", "......."}); }
Warehouse corridorWithoutNiche() { return layout({"#######", "......."}); }

// A robot driving east from (0, 1) at one cell per step and stopping at (6, 1): at step s it holds
// (s, 1) and the cell ahead of it.
Script eastboundRobot() {
  Script script;
  script.takeCell(P(0, 1), 0, 1);
  for (int x = 1; x < 6; ++x) {
    script.takeCell(P(x, 1), static_cast<Tick>(x - 1), static_cast<Tick>(x + 1));
    script.takeEdge(P(x, 1), P(x - 1, 1), static_cast<Tick>(x - 1));
  }
  script.takeCell(P(6, 1), 5, kForever);
  script.takeEdge(P(6, 1), P(5, 1), 5);
  return script;
}

TEST(SpaceTimeAStarTest, ARobotMeetingAnotherInACorridorStepsIntoTheNicheAndWaitsForItToPass) {
  const Warehouse w = corridorWithNiche();
  const SpaceTimePlan plan = search(w, P(6, 1), P(0, 1), eastboundRobot(), 16);
  ASSERT_EQ(plan.status, SpaceTimeStatus::Found);
  expectLegal(w, plan, P(6, 1));
  EXPECT_TRUE(visits(plan, P(5, 0)));  // the niche
  EXPECT_EQ(plan.duration(), 12U);
  EXPECT_EQ(plan.waits, 4U);
  EXPECT_EQ(plan.timeline.back(), P(0, 1));
}

TEST(SpaceTimeAStarTest, WithNoNicheNoWayToPassExistsAndTheSearchSaysSoInsteadOfInventingOne) {
  const Warehouse w = corridorWithoutNiche();
  const SpaceTimePlan plan = search(w, P(6, 1), P(0, 1), eastboundRobot(), 16);
  EXPECT_EQ(plan.status, SpaceTimeStatus::NoPlan);
}

// ---- the window, and the budget -----------------------------------------------------------------

TEST(SpaceTimeAStarTest, AGoalBeyondTheWindowGivesTheBestPlaceToBeWhenItEnds) {
  const Warehouse w = layout({std::string(30, '.')});
  const SpaceTimePlan plan = search(w, P(0, 0), P(29, 0), Script{}, 8);
  ASSERT_EQ(plan.status, SpaceTimeStatus::Partial);
  EXPECT_EQ(plan.duration(), 8U);
  EXPECT_EQ(plan.timeline.back(), P(8, 0));
  EXPECT_EQ(plan.remaining, 21);
}

TEST(SpaceTimeAStarTest, APartialPlanDodgesWhatIsTakenAndStillGetsAsNearAsItCan) {
  const Warehouse w = layout({std::string(20, '.'), std::string(20, '.')});
  Script script;
  script.takeCell(P(4, 0), 0, kForever);
  script.takeCell(P(5, 0), 0, kForever);
  const SpaceTimePlan plan = search(w, P(0, 0), P(19, 0), script, 10);
  ASSERT_EQ(plan.status, SpaceTimeStatus::Partial);
  // Ten steps, two of them spent getting round the block (a row down, and a column's worth of
  // distance back to the goal's row): 19 - 10 + 2 = 11 cells remain.
  EXPECT_EQ(plan.remaining, 11);
  EXPECT_FALSE(visits(plan, P(4, 0)));
}

TEST(SpaceTimeAStarTest, ABudgetStopsTheSearchAndNothingIsReturnedFromIt) {
  const Warehouse w = layout({std::string(40, '.'), std::string(40, '.'), std::string(40, '.')});
  Script script;
  script.takeCell(P(20, 1), 0, kForever);
  // The first search has to fill the distance field (120 cells), which 30 does not pay for.
  const SpaceTimePlan unaffordable = search(w, P(0, 1), P(39, 1), script, 64, 0, 30);
  EXPECT_EQ(unaffordable.status, SpaceTimeStatus::BudgetExceeded);
  EXPECT_TRUE(unaffordable.timeline.empty());
  EXPECT_EQ(unaffordable.work, 0U);  // and it did not start
  // With the field already known, the search itself is what runs out.
  SpaceTimeAStar planner;
  ASSERT_TRUE(planner.prepare(w, P(39, 1)).has_value());
  const SpaceTimePlan limited =
      planner.search(w, P(0, 1), P(39, 1), 0, script, SpaceTimeOptions{64, 30});
  EXPECT_EQ(limited.status, SpaceTimeStatus::BudgetExceeded);
  EXPECT_TRUE(limited.timeline.empty());
  EXPECT_GT(limited.work, 0U);
  EXPECT_LE(limited.work, 31U);
  const SpaceTimePlan free = search(w, P(0, 1), P(39, 1), script, 64);
  EXPECT_EQ(free.status, SpaceTimeStatus::Found);
  EXPECT_EQ(free.duration(), 41U);  // round the one blocked cell
  const SpaceTimePlan generous = search(w, P(0, 1), P(39, 1), script, 64, 0, free.work + 1);
  EXPECT_EQ(generous.status, SpaceTimeStatus::Found);
  EXPECT_TRUE(generous.timeline == free.timeline);
}

TEST(SpaceTimeAStarTest, PreparingTheDistancesCostsAFillOnceAndIsRefusedWhenItWouldCostTooMuch) {
  const Warehouse w = layout({"......", "......"});
  SpaceTimeAStar planner;
  EXPECT_FALSE(planner.prepare(w, P(5, 1), 5).has_value());  // twelve cells to visit
  const auto first = planner.prepare(w, P(5, 1), 12);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(*first, 12U);
  EXPECT_EQ(planner.prepare(w, P(5, 1), 1), std::optional<std::size_t>(0));  // cached: free
  EXPECT_EQ(planner.distance(w, P(0, 0), P(5, 1)), 6);
  EXPECT_EQ(planner.distance(w, P(5, 1), P(5, 1)), 0);
  EXPECT_EQ(planner.distance(w, P(9, 9), P(5, 1)), -1);
  const Warehouse walled = layout({"..#..", "..#.."});
  EXPECT_EQ(planner.distance(walled, P(0, 0), P(4, 1)), -1);
  EXPECT_THROW((void)planner.prepare(walled, P(2, 0)), std::invalid_argument);
}

TEST(SpaceTimeAStarTest, TheStartStepOnlyShiftsTimeNotTheRoute) {
  const Warehouse w = layout({".......", "......."});
  Script script;
  script.takeCell(P(3, 0), 102, 105);  // the same blockage as the 2..5 one, 100 steps later
  const SpaceTimePlan late = search(w, P(0, 0), P(6, 0), script, 12, 100);
  Script early;
  early.takeCell(P(3, 0), 2, 5);
  const SpaceTimePlan soon = search(w, P(0, 0), P(6, 0), early, 12, 0);
  ASSERT_EQ(late.status, SpaceTimeStatus::Found);
  EXPECT_TRUE(late.timeline == soon.timeline);
}

// ---- layout changes and determinism -------------------------------------------------------------

TEST(SpaceTimeAStarTest, ADistanceFieldIsNeverReusedOnAnotherLayout) {
  SpaceTimeAStar planner;
  const Script script;
  const Warehouse open = layout({".....", "....."});
  const Warehouse closed = layout({"..#..", "..#.."});
  EXPECT_EQ(planner.search(open, P(0, 0), P(4, 0), 0, script).status, SpaceTimeStatus::Found);
  EXPECT_EQ(planner.search(closed, P(0, 0), P(4, 0), 0, script).status, SpaceTimeStatus::NoPlan);
  EXPECT_EQ(planner.search(open, P(0, 0), P(4, 0), 0, script).status, SpaceTimeStatus::Found);
}

TEST(SpaceTimeAStarTest, TheSameQueryGivesTheSamePlanOnAReusedAndAFreshPlanner) {
  const Warehouse w = layout({"........", ".##..##.", "........", ".##..##.", "........"});
  Script script;
  script.takeCell(P(3, 2), 2, 6);
  script.takeCell(P(4, 2), 3, 5);
  script.takeEdge(P(2, 2), P(3, 2), 1);
  SpaceTimeAStar reused;
  const SpaceTimePlan first = reused.search(w, P(0, 2), P(7, 2), 0, script, SpaceTimeOptions{});
  (void)reused.search(w, P(7, 4), P(0, 0), 0, script, SpaceTimeOptions{});  // dirties its memory
  const SpaceTimePlan again = reused.search(w, P(0, 2), P(7, 2), 0, script, SpaceTimeOptions{});
  SpaceTimeAStar fresh;
  const SpaceTimePlan other = fresh.search(w, P(0, 2), P(7, 2), 0, script, SpaceTimeOptions{});
  ASSERT_EQ(first.status, SpaceTimeStatus::Found);
  EXPECT_TRUE(first.timeline == again.timeline);
  EXPECT_TRUE(first.timeline == other.timeline);
  EXPECT_EQ(first.work, other.work);
}

// ---- against an independent reference -----------------------------------------------------------

// Breadth-first over (cell, step) layers with the same rules, written separately from the search.
struct Reference {
  SpaceTimeStatus status = SpaceTimeStatus::NoPlan;
  std::size_t duration = 0;
  std::int32_t remaining = 0;
};

Reference reference(const Warehouse& w, Point start, Point goal, const Script& script,
                    std::size_t window, bool terminalGoal) {
  const auto size = w.size();
  // Distances to the goal on the empty map.
  std::vector<std::int32_t> dist(size.cellCount(), -1);
  std::vector<std::size_t> queue = {size.indexOf(goal)};
  dist[queue[0]] = 0;
  for (std::size_t head = 0; head < queue.size(); ++head) {
    const Point at = size.pointAt(queue[head]);
    for (const Point d : {P(0, 1), P(1, 0), P(0, -1), P(-1, 0)}) {
      const Point next = P(at.x + d.x, at.y + d.y);
      if (w.isWalkable(next) && dist[size.indexOf(next)] < 0) {
        dist[size.indexOf(next)] = dist[queue[head]] + 1;
        queue.push_back(size.indexOf(next));
      }
    }
  }
  if (dist[size.indexOf(start)] < 0) {
    return {SpaceTimeStatus::NoPlan, 0, 0};
  }
  std::set<std::size_t> layer = {size.indexOf(start)};
  for (std::size_t depth = 0; depth <= window; ++depth) {
    if (layer.contains(size.indexOf(goal))) {
      bool holds = true;
      for (std::size_t later = depth + 1; later <= window; ++later) {
        holds = holds && script.cellFree(goal, later);
      }
      if (holds) {
        return {SpaceTimeStatus::Found, depth, 0};
      }
      if (terminalGoal) {
        layer.erase(size.indexOf(goal));  // a dead end: it would have to leave and come back
      }
    }
    if (depth == window) {
      break;
    }
    std::set<std::size_t> next;
    for (const std::size_t index : layer) {
      const Point here = size.pointAt(index);
      if (script.cellFree(here, depth + 1)) {
        next.insert(index);
      }
      for (const Point d : {P(0, -1), P(1, 0), P(0, 1), P(-1, 0)}) {
        const Point there = P(here.x + d.x, here.y + d.y);
        if (w.isWalkable(there) && dist[size.indexOf(there)] >= 0 &&
            script.cellFree(there, depth) && script.cellFree(there, depth + 1) &&
            script.edgeFree(here, there, depth)) {
          next.insert(size.indexOf(there));
        }
      }
    }
    layer = std::move(next);
  }
  if (layer.empty()) {
    return {SpaceTimeStatus::NoPlan, 0, 0};
  }
  std::int32_t best = dist[*layer.begin()];
  for (const std::size_t index : layer) {
    best = std::min(best, dist[index]);
  }
  return {SpaceTimeStatus::Partial, window, best};
}

TEST(SpaceTimeAStarPropertyTest, OnRandomMapsAndRandomTakenCellsItMatchesAnIndependentSearch) {
  Rng rng(2026);
  int found = 0;
  int partial = 0;
  int none = 0;
  int waited = 0;
  for (int round = 0; round < 400; ++round) {
    const int width = 4 + static_cast<int>(rng.below(4));
    const int height = 3 + static_cast<int>(rng.below(4));
    std::vector<std::string> rows(static_cast<std::size_t>(height),
                                  std::string(static_cast<std::size_t>(width), '.'));
    for (auto& row : rows) {
      for (char& c : row) {
        if (rng.below(100) < 18) {
          c = '#';
        }
      }
    }
    rows[0][0] = '.';
    rows.back().back() = '.';
    const Warehouse w = layout(rows);
    const std::size_t window = 4 + rng.below(9);
    Script script;
    for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {
        for (Tick step = 1; step <= window + 1; ++step) {
          if (rng.below(100) < 9) {
            script.takeCell(P(x, y), step, step + 1);
          }
        }
        if (rng.below(100) < 6) {
          const Point to = P(x + 1, y);
          if (w.isWalkable(to)) {
            script.takeEdge(P(x, y), to, rng.below(window));
          }
        }
      }
    }
    const Point start = P(0, 0);
    const Point goal = P(width - 1, height - 1);
    const bool terminal = round % 2 == 1;
    const Reference expected = reference(w, start, goal, script, window, terminal);
    const SpaceTimePlan plan = search(w, start, goal, script, window, 0, 0, terminal);
    ASSERT_EQ(plan.status, expected.status) << "round " << round;
    if (plan.status == SpaceTimeStatus::Found) {
      ++found;
      ASSERT_EQ(plan.duration(), expected.duration) << "round " << round;
      EXPECT_EQ(plan.timeline.back(), goal);
      if (terminal) {
        EXPECT_EQ(std::count(plan.timeline.begin(), plan.timeline.end(), goal), 1);
      }
      waited += plan.waits > 0 ? 1 : 0;
    } else if (plan.status == SpaceTimeStatus::Partial) {
      ++partial;
      ASSERT_EQ(plan.remaining, expected.remaining) << "round " << round;
      EXPECT_EQ(plan.duration(), window);
    } else {
      ++none;
    }
    if (!plan.timeline.empty()) {
      expectLegal(w, plan, start);
      // The plan itself obeys the constraints it was given.
      for (std::size_t i = 1; i < plan.timeline.size(); ++i) {
        const Tick step = i - 1;
        if (plan.timeline[i] == plan.timeline[i - 1]) {
          ASSERT_TRUE(script.cellFree(plan.timeline[i], i)) << "round " << round;
        } else {
          ASSERT_TRUE(script.cellFree(plan.timeline[i], step)) << "round " << round;
          ASSERT_TRUE(script.cellFree(plan.timeline[i], i)) << "round " << round;
          ASSERT_TRUE(script.edgeFree(plan.timeline[i - 1], plan.timeline[i], step))
              << "round " << round;
        }
      }
    }
  }
  // Make sure the random instances really covered each outcome, and that waiting came up.
  EXPECT_GT(found, 80);
  EXPECT_GT(partial, 20);
  EXPECT_GT(none, 5);
  EXPECT_GT(waited, 30);
}

}  // namespace
