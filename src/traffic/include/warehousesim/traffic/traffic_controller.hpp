#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/traffic/reservation_table.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::traffic {

// What the table lets one robot do this step.
struct TrafficPlan {
  // How many leading cells of the robot's path it may be granted: all of them if its whole
  // forecast route is free of clashes, otherwise only the cells before the first one that is not.
  std::size_t clearedCells = 0;
  // What held it back: the first clash on the route it would have taken. Empty if nothing did.
  std::optional<Conflict> conflict;
  // The clash is on a cell the robot is already standing on or holds, so waiting cannot avoid it:
  // a robot with the right of way wants to pass through. The robot has to be moved or move itself.
  bool inTheWay = false;
  // The robot is at rest and cannot take its next cell yet. What it reserved is its whole route,
  // starting this many steps from now, instead of standing where it is. 0: it reserved the cells
  // before the clash (or, with no clash, everything).
  std::size_t delaySteps = 0;

  [[nodiscard]] bool waiting() const noexcept { return conflict.has_value(); }
};

// How often planning found each kind of clash.
struct TrafficStats {
  std::uint64_t blocked = 0;
  std::uint64_t vertex = 0;
  std::uint64_t swaps = 0;
  std::uint64_t inTheWay = 0;

  TrafficStats& operator+=(const TrafficStats& other) noexcept {
    blocked += other.blocked;
    vertex += other.vertex;
    swaps += other.swaps;
    inTheWay += other.inTheWay;
    return *this;
  }
  bool operator==(const TrafficStats&) const = default;
};

// Builds the reservation table for one simulation step and tells each robot how far it may go.
//
// Priority is the robot id, lowest first (or an order given to beginStep). Each step starts with
// beginStep(), which puts what is already a fact on the table: every robot's cell and the cells it
// holds, and robots standing still for the whole horizon. Then each robot with a route calls
// plan(), in order of priority (calling out of order throws). plan() forecasts the cells the robot
// would hold on every step of the horizon if nothing were in its way (robots::forecastHolds, the
// same motion code the simulation runs), and reserves them. Because robots plan in priority order,
// whatever a robot reserves was checked against every robot that outranks it, and nothing that
// outranks it has to give way to it.
//
// If the route clashes with a reservation already made, the robot is held back before the first
// cell that clashes: its plan is cut there and it reserves the cell it will wait on instead. The
// table only looks `horizon` steps ahead, so a robot also avoids waiting on any cell that a robot
// with priority has on its route at all: otherwise it would wait in a corridor that the other robot
// is about to drive down. If even waiting where it is clashes, the clash is on cells it already
// holds (TrafficPlan::inTheWay).
//
// The table is rebuilt from scratch every step, from the robots' real states, so it never goes
// stale for longer than a step. It only ever makes a robot wait earlier; it never grants a cell the
// world would refuse. It is a plan, not the truth: the world's claims stay the authority on who
// holds a cell, so a robot the table cannot help (inTheWay) is left to them. The table copies the
// walkable cells when it is built and does not see later edits to the layout.
class TrafficController {
 public:
  // Far enough to cover the braking distance and a few cells beyond. A forecast is only as good as
  // the assumption that nobody gets in anyone's way, so the further ahead it looks the more often
  // it holds a robot back for a clash that never comes (48 steps cost the demo hall about 12% of
  // its throughput and jammed one seed; 8 to 16 are indistinguishable from each other).
  static constexpr std::size_t kDefaultHorizon = 12;

  // Throws std::invalid_argument if the horizon is shorter than two steps.
  explicit TrafficController(const world::Warehouse& warehouse,
                             std::size_t horizon = kDefaultHorizon);

  // Starts step `now`: forgets the last step's reservations and reserves the facts. Throws
  // std::logic_error if two robots hold the same cell. Priority is the robot id, lowest first,
  // unless `priority` names some robots: those come first, in the order named, ahead of everybody
  // else (who stay in id order). Ids that are not in the fleet are ignored.
  void beginStep(core::Tick now, std::span<const robots::Robot> fleet,
                 std::span<const core::RobotId> priority = std::span<const core::RobotId>());

  // Reserves `robot`'s forecast route. Throws std::logic_error before beginStep(), or if robots do
  // not plan in order of priority within a step.
  [[nodiscard]] TrafficPlan plan(const robots::Robot& robot);
  // Plans the robot that was planned last once more, after its route or state changed (typically
  // because the cooperative router gave it a new route): what it reserved before is replaced.
  // Throws std::logic_error if any other robot was planned last, or none was.
  [[nodiscard]] TrafficPlan replan(const robots::Robot& robot);

  // Tells the table that `robot` was moved after beginStep() by something other than its own turn
  // (another robot made it step aside). What the table held for it is replaced by where it is now,
  // so that robots planning after this see it there. Throws std::logic_error before beginStep().
  void relocate(const robots::Robot& robot);

  [[nodiscard]] const ReservationTable& table() const noexcept { return table_; }
  [[nodiscard]] core::Tick now() const noexcept { return now_; }
  [[nodiscard]] std::size_t horizon() const noexcept { return horizon_; }
  [[nodiscard]] const TrafficStats& stats() const noexcept { return stats_; }

 private:
  [[nodiscard]] TrafficPlan planAndRecord(const robots::Robot& robot, bool again);
  [[nodiscard]] TrafficPlan planRoute(const robots::Robot& robot, bool again);
  // Reserves the robot's forecast route set off by the smallest delay (up to six steps) at which it
  // fits; returns the delay, or 0 and reserves nothing if there is none.
  [[nodiscard]] std::size_t reserveDelayed(const robots::Robot& robot);

  ReservationTable table_;
  // Every cell on the remaining route of each robot planned so far this step, however far ahead.
  std::unordered_map<core::Point, std::uint32_t, core::PointHash>
      routes_;                          // cell -> routes through it
  std::vector<core::Point> lastRoute_;  // what the robot planned last added to routes_
  std::size_t horizon_;
  core::Tick now_ = 0;
  bool began_ = false;
  std::optional<core::RobotId> lastPlanned_;
  std::vector<std::uint32_t> rank_;  // by robot id: 1 plans first
  TrafficStats stats_;
};

}  // namespace warehousesim::traffic
