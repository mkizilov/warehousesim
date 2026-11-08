#include "warehousesim/traffic/progress_monitor.hpp"

#include <algorithm>
#include <stdexcept>

namespace warehousesim::traffic {

using core::RobotId;

ProgressMonitor::ProgressMonitor(std::size_t robotCount, std::int32_t radius)
    : radius_(radius), stuck_(robotCount, 0), waitsFor_(robotCount), anchor_(robotCount) {}

std::size_t ProgressMonitor::index(RobotId robot) const {
  if (!robot.valid() || robot.value > stuck_.size()) {
    throw std::out_of_range("unknown robot id");
  }
  return robot.value - 1;
}

void ProgressMonitor::record(RobotId robot, bool wantsToMove, core::Point position,
                             std::optional<RobotId> waitsFor) {
  const std::size_t i = index(robot);
  if (wantsToMove && anchor_[i] && core::manhattanDistance(position, *anchor_[i]) <= radius_) {
    ++stuck_[i];
    waitsFor_[i] = waitsFor;
    return;
  }
  stuck_[i] = 0;
  waitsFor_[i].reset();
  // A robot that wants to move starts waiting here; one that does not has nothing to measure from.
  anchor_[i] = wantsToMove ? std::optional<core::Point>(position) : std::nullopt;
}

void ProgressMonitor::reset(RobotId robot) {
  const std::size_t i = index(robot);
  stuck_[i] = 0;
  waitsFor_[i].reset();
  anchor_[i].reset();
}

std::uint32_t ProgressMonitor::stuckTicks(RobotId robot) const { return stuck_[index(robot)]; }

std::optional<RobotId> ProgressMonitor::waitingFor(RobotId robot) const {
  return waitsFor_[index(robot)];
}

std::optional<core::Point> ProgressMonitor::anchor(RobotId robot) const {
  return anchor_[index(robot)];
}

std::vector<RobotId> ProgressMonitor::stuck(std::uint32_t minTicks) const {
  std::vector<RobotId> result;
  for (std::size_t i = 0; i < stuck_.size(); ++i) {
    if (stuck_[i] >= minTicks && stuck_[i] > 0) {
      result.push_back(RobotId{static_cast<std::uint32_t>(i + 1)});
    }
  }
  return result;
}

std::vector<std::vector<RobotId>> ProgressMonitor::deadlocks(std::uint32_t minTicks) const {
  // Each robot waits for at most one other, so the waiting forms a graph in which every node has
  // at most one way out: a walk either ends, joins a walk already done with, or closes a ring.
  enum class Mark : std::uint8_t { Unseen, OnWalk, Done };
  std::vector<Mark> marks(stuck_.size(), Mark::Unseen);
  const auto eligible = [&](std::size_t i) { return stuck_[i] > 0 && stuck_[i] >= minTicks; };

  std::vector<std::vector<RobotId>> rings;
  for (std::size_t first = 0; first < stuck_.size(); ++first) {
    if (marks[first] != Mark::Unseen || !eligible(first)) {
      continue;
    }
    std::vector<std::size_t> walk;
    std::size_t at = first;
    while (true) {
      marks[at] = Mark::OnWalk;
      walk.push_back(at);
      const std::optional<RobotId>& next = waitsFor_[at];
      if (!next || !next->valid() || next->value > stuck_.size() || !eligible(next->value - 1)) {
        break;
      }
      const std::size_t target = next->value - 1;
      if (marks[target] == Mark::OnWalk) {
        // A ring: from the first time `target` appeared on this walk to the end of it.
        const auto from = std::find(walk.begin(), walk.end(), target);
        std::vector<std::size_t> members(from, walk.end());
        const auto lowest = std::min_element(members.begin(), members.end());
        std::rotate(members.begin(), lowest, members.end());
        std::vector<RobotId> ring;
        for (const std::size_t member : members) {
          ring.push_back(RobotId{static_cast<std::uint32_t>(member + 1)});
        }
        rings.push_back(std::move(ring));
        break;
      }
      if (marks[target] == Mark::Done) {
        break;
      }
      at = target;
    }
    for (const std::size_t member : walk) {
      marks[member] = Mark::Done;
    }
  }
  std::sort(rings.begin(), rings.end(),
            [](const auto& a, const auto& b) { return a.front() < b.front(); });
  return rings;
}

}  // namespace warehousesim::traffic
