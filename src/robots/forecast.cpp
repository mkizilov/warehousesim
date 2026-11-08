#include "warehousesim/robots/forecast.hpp"

#include <algorithm>

namespace warehousesim::robots {

namespace {

// Rounds of turn, grant and drive that one step can hold (as in Simulation::moveAlongPath).
constexpr int kMaxMotionRounds = 8;
// A step cannot hold more path cells than this: three granted ahead, and one passed per round.
constexpr std::size_t kMaxHeldCells = 16;

}  // namespace

UnobstructedStep stepUnobstructed(Robot& robot, double dt) {
  UnobstructedStep result;
  result.heldPrefix = robot.authority();
  double timeLeft = dt;
  for (int round = 0; round < kMaxMotionRounds; ++round) {
    if (!robot.hasPath()) {
      return result;
    }
    if (robot.state() == RobotState::Navigating) {
      timeLeft = robot.turnTowardNext(timeLeft);
      if (timeLeft <= 0.0) {
        (void)robot.advance(0.0);
        return result;
      }
      while (robot.authority() < robot.wantedAuthority(timeLeft)) {
        robot.extendAuthority();
      }
      result.heldPrefix = std::max(result.heldPrefix, result.passed + robot.authority());
    }
    result.passed += robot.advance(timeLeft);
    timeLeft = robot.unusedTime();
    if (timeLeft <= 0.0 || !robot.hasPath()) {
      return result;
    }
  }
  return result;
}

std::vector<std::vector<core::Point>> forecastHolds(Robot robot, std::size_t steps) {
  std::vector<std::vector<core::Point>> forecast;
  forecast.reserve(steps);
  for (std::size_t s = 0; s < steps; ++s) {
    std::vector<core::Point> holds = {robot.position()};
    if (robot.hasPath()) {
      // Remember the path as the step starts: the robot consumes it as it passes cells.
      const std::size_t remembered = std::min(robot.path().size(), kMaxHeldCells);
      const std::vector<core::Point> startPath(
          robot.path().begin(), robot.path().begin() + static_cast<std::ptrdiff_t>(remembered));
      const UnobstructedStep step = stepUnobstructed(robot, 1.0);
      const std::size_t held = std::min(step.heldPrefix, startPath.size());
      holds.insert(holds.end(), startPath.begin(),
                   startPath.begin() + static_cast<std::ptrdiff_t>(held));
    }
    forecast.push_back(std::move(holds));
  }
  return forecast;
}

}  // namespace warehousesim::robots
