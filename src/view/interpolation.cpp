#include "warehousesim/view/interpolation.hpp"

#include <algorithm>
#include <cmath>

#include "warehousesim/core/angle.hpp"
#include "warehousesim/core/vec2.hpp"

namespace warehousesim::view {

namespace {

const simulation::RobotSnapshot* find(const simulation::Snapshot& snapshot, core::RobotId id) {
  const auto it = std::ranges::find(snapshot.robots, id, &simulation::RobotSnapshot::id);
  return it == snapshot.robots.end() ? nullptr : &*it;
}

}  // namespace

simulation::Snapshot blend(const simulation::Snapshot& previous, const simulation::Snapshot& latest,
                           double alpha) {
  const double t = std::isfinite(alpha) ? std::clamp(alpha, 0.0, 1.0) : 1.0;
  simulation::Snapshot result = latest;
  for (simulation::RobotSnapshot& robot : result.robots) {
    const simulation::RobotSnapshot* before = find(previous, robot.id);
    const core::Vec2 to = robot.center();
    const double toHeading = robot.headingAngle();
    if (before == nullptr) {
      robot.pose = simulation::RobotPose{to, toHeading};
      continue;
    }
    const core::Vec2 from = before->center();
    const core::Vec2 gap = to - from;
    if (std::hypot(gap.x, gap.y) > kMaxInterpolatedDistance) {
      robot.pose = simulation::RobotPose{to, toHeading};
      continue;
    }
    robot.pose = simulation::RobotPose{core::lerp(from, to, t),
                                       core::lerpAngle(before->headingAngle(), toHeading, t)};
  }
  return result;
}

}  // namespace warehousesim::view
