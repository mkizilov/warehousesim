#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"

namespace warehousesim::traffic {

// Notices robots that are trying to get somewhere and are not getting anywhere, and robots that
// wait for each other in a circle.
//
// Every step the caller says, for each robot, whether it wants to move, where it is, and who it is
// waiting for (the robot holding the cell it needs, or the one whose reservation keeps it back). A
// robot that wants to move and has not left the neighbourhood of where it was when it started
// waiting has its stuck count raised; any other robot starts again from zero. Counting the
// neighbourhood rather than "did not move" matters: two robots that keep stepping out of each
// other's way and back are going nowhere, though both move every few steps.
//
// A deadlock is a ring of robots, each waiting for the next, that have all been stuck for a while:
// no robot of the ring can go on until one of them gives way.
//
// This class only watches. What to do about a stuck robot or a deadlock is the caller's business
// (see Simulation). It is deterministic: robots are looked at in id order and every list it returns
// is in a fixed order.
class ProgressMonitor {
 public:
  // Robots are ids 1 to `robotCount`. A robot has got somewhere once it is more than `radius`
  // cells (Manhattan distance) from where it started waiting.
  explicit ProgressMonitor(std::size_t robotCount, std::int32_t radius = 2);

  // One robot's step. Throws std::out_of_range for an id outside 1 to robotCount.
  void record(core::RobotId robot, bool wantsToMove, core::Point position,
              std::optional<core::RobotId> waitsFor);
  // Forgets what has been counted for the robot (it has a new task, or has given up the old one).
  void reset(core::RobotId robot);

  [[nodiscard]] std::uint32_t stuckTicks(core::RobotId robot) const;
  [[nodiscard]] std::optional<core::RobotId> waitingFor(core::RobotId robot) const;

  // The robots stuck for at least `minTicks`, in id order.
  [[nodiscard]] std::vector<core::RobotId> stuck(std::uint32_t minTicks) const;

  // The rings of robots that wait for each other and have all been stuck for at least `minTicks`.
  // Each ring lists its robots in waiting order (each waits for the next, the last for the first)
  // starting with the lowest id; rings are ordered by that first robot.
  [[nodiscard]] std::vector<std::vector<core::RobotId>> deadlocks(std::uint32_t minTicks) const;

 private:
  [[nodiscard]] std::size_t index(core::RobotId robot) const;

  std::int32_t radius_;
  std::vector<std::uint32_t> stuck_;
  std::vector<std::optional<core::RobotId>> waitsFor_;
  std::vector<std::optional<core::Point>> anchor_;  // where the robot was when it began to wait
};

}  // namespace warehousesim::traffic
