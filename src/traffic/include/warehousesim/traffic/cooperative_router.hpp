#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <unordered_set>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/planning/path_planner.hpp"
#include "warehousesim/planning/space_time_astar.hpp"
#include "warehousesim/robots/forecast.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/traffic/reservation_table.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::traffic {

// What the reservation table leaves free to one robot, in the form the space-time search asks for:
// its own reservations never count against it, everybody else's do, and so does a wall.
class TableConstraints final : public planning::SpaceTimeConstraints {
 public:
  TableConstraints(const ReservationTable& table, core::RobotId self)
      : table_(table), self_(self) {}

  [[nodiscard]] bool cellFree(core::Point cell, core::Tick step) const override;
  [[nodiscard]] bool edgeFree(core::Point from, core::Point to, core::Tick step) const override;

 private:
  const ReservationTable& table_;
  core::RobotId self_;
};

struct CooperativeRouterOptions {
  // How many steps ahead to plan. Reservations only exist as far as the table's horizon.
  std::size_t window = 12;
  // Work (see planning::SpaceTimeOptions::maxWork) one call may spend: following the current route
  // plus the search. A call that cannot afford both says OutOfBudget.
  std::size_t maxWork = 1500;
  // A new route may be this many cells longer than the one it replaces when the old one could still
  // be followed (with waits) ...
  std::size_t maxDetourCells = 4;
  // ... and this many when it could not, so the robot has to get out of somebody's way.
  std::size_t maxDodgeCells = 12;
};

enum class RouteOutcomeKind : std::uint8_t {
  Skipped,      // nothing to plan: not driving, or the end of its granted cells is too far off
  Kept,         // the current route, with waits, is as good as anything found
  Rerouted,     // `path` replaces the robot's route
  NoPlan,       // everything is taken inside the window, or the rest of the way is cut off
  TooLong,      // a better way exists but is longer than the options allow
  OutOfBudget,  // the work ran out first
};

[[nodiscard]] const char* toString(RouteOutcomeKind kind) noexcept;

struct RouteOutcome {
  RouteOutcomeKind kind = RouteOutcomeKind::Skipped;
  // The robot's whole new route, granted cells included, ending at its destination. Only for
  // Rerouted.
  std::vector<core::Point> path;
  // The plan reached the window's end rather than the destination; the rest of `path` is the
  // shortest way on the empty map.
  bool partial = false;
  // The robot's old route could not be followed at all (it is in somebody's way).
  bool dodge = false;
  std::size_t waits = 0;  // waiting the plan does along the way
  std::size_t work = 0;
};

// Prioritized cooperative planning for one robot at a time. Robots take turns in priority order;
// each plans against the reservations of the ones that went before (the table), and then reserves
// its own. This class does the planning half: given the table as it stands for one robot, it looks
// for a route in space and time with planning::SpaceTimeAStar and says whether it beats what the
// robot is doing now.
//
// * What is compared: the robot's current route followed with waits (a small dynamic program over
//   the route's cells and the steps of the window; the robot may wait anywhere but never go back).
//   A new route replaces the old only if it gets there sooner inside the window, or, when neither
//   reaches the destination, ends the window nearer to it. Ties keep the old route, so robots do
//   not change their minds for nothing.
// * Where the plan starts: after the cells the robot has already been granted, at the step it is
//   forecast to be there (robots::forecastHolds), so a driving robot is never told to un-drive.
// * Beyond the window: a plan that does not reach the destination is extended with the shortest
//   route on the empty map (avoiding `avoid` where it can), which is not cooperative at all.
// * Waiting: waits in the plan are not stored anywhere. The route is the cells in order; the
//   TrafficController holds the robot back each step until the next cell is clear, which is the
//   same thing as the plan's waits as long as everybody keeps to their forecast.
//
// Deterministic: the same table, robot and options give the same outcome.
class CooperativeRouter {
 public:
  struct Request {
    const world::Warehouse& layout;
    const ReservationTable& table;
    const robots::Robot& robot;
    core::Tick now;
    planning::PathPlanner& tailPlanner;
    const planning::BlockedSet& avoid;
    CooperativeRouterOptions options;
    // If set, the cells to avoid are made by calling this, when (and only if) a plan falls short of
    // the destination and needs them, instead of being taken from `avoid`. Making the set can cost
    // more than the search it is for.
    const std::function<planning::BlockedSet()>* avoidLater = nullptr;
  };

  [[nodiscard]] RouteOutcome reroute(const Request& request);

 private:
  planning::SpaceTimeAStar search_;
  std::optional<robots::Robot>
      walkerSlot_;  // working copy for forecasts, kept to spare its storage
  robots::ForecastBuffer forecast_;
};

// How the current route fares when followed with waits: used by CooperativeRouter, public so that
// it can be tested on its own.
struct RouteFollow {
  bool feasible = false;       // the robot can be somewhere legal at the end of the window
  bool reaches = false;        // it reaches `rest.back()` and can stay there to the window's end
  std::size_t arrival = 0;     // steps until it does
  std::int32_t remaining = 0;  // otherwise, the distance on the empty map left at the window's end
  std::size_t work = 0;
};

// The robot starts on `start` at `startStep` and goes through the cells of `rest` in order, waiting
// where it must. `toGoal` says that `rest` ends at the destination (otherwise it is only the first
// part of the route, and the robot cannot arrive). `distance(cell)` is the distance to the
// destination on the empty map.
[[nodiscard]] RouteFollow followRoute(const planning::SpaceTimeConstraints& free, core::Point start,
                                      core::Tick startStep, const std::vector<core::Point>& rest,
                                      bool toGoal, std::size_t window,
                                      const std::function<std::int32_t(core::Point)>& distance);

}  // namespace warehousesim::traffic
