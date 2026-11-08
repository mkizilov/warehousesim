#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/tasks/distance_estimator.hpp"
#include "warehousesim/tasks/task_queue.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::tasks {

struct Assignment {
  core::RobotId robot;
  core::TaskId task;
  std::int32_t estimatedDistance = 0;  // robot -> pickup, as the dispatcher estimated it

  bool operator==(const Assignment&) const = default;
};

enum class RejectionReason : std::uint8_t {
  DropoffUnreachable,  // no route from the pickup to the dropoff
  PickupUnreachable,   // no robot in the fleet can ever reach the pickup
};

[[nodiscard]] const char* toString(RejectionReason reason) noexcept;

// A pending task that can never be done, so waiting would only keep it in the queue forever.
struct Rejection {
  core::TaskId task;
  RejectionReason reason;

  bool operator==(const Rejection&) const = default;
};

struct DispatchPlan {
  std::vector<Assignment> assignments;
  std::vector<Rejection> rejections;
};

// Decides which robot should take which pending task. Dispatchers are pure deciders: they read
// the queue and the fleet and propose a plan, but change neither. The caller applies the plan, and
// the queue's exclusive claim (TaskQueue::tryAssign) is what finally guarantees that no task ends
// up with two robots, whatever a dispatcher proposes.
class Dispatcher {
 public:
  Dispatcher() = default;
  Dispatcher(const Dispatcher&) = delete;
  Dispatcher& operator=(const Dispatcher&) = delete;
  virtual ~Dispatcher() = default;

  [[nodiscard]] virtual DispatchPlan plan(const world::Warehouse& warehouse,
                                          std::span<const robots::Robot> fleet,
                                          const TaskQueue& queue) = 0;
};

// Deterministic greedy dispatching.
//
//   * Tasks are taken in queue order: highest priority first, then oldest, then lowest id.
//   * Each task goes to the available robot with the smallest estimated path distance to its
//     pickup; ties go to the lowest robot id.
//   * A robot is available if it is idle and charged (Robot::isAvailable()); each receives at most
//     one task per plan, and each task goes to at most one robot.
//   * Robots that cannot reach a pickup are skipped for that task. A task whose delivery leg
//     (pickup -> dropoff) is impossible, or whose pickup no robot in the whole fleet can reach, is
//     rejected. A task that only busy robots can reach stays pending.
//
// Because the order is fixed by priority and age rather than by who is closest, an urgent task is
// never starved by a nearer, less important one; closeness only decides who serves it.
class GreedyDispatcher final : public Dispatcher {
 public:
  // Uses PathDistance when no estimator is given.
  explicit GreedyDispatcher(std::unique_ptr<DistanceEstimator> estimator = nullptr);

  [[nodiscard]] DispatchPlan plan(const world::Warehouse& warehouse,
                                  std::span<const robots::Robot> fleet,
                                  const TaskQueue& queue) override;

 private:
  std::unique_ptr<DistanceEstimator> estimator_;
};

}  // namespace warehousesim::tasks
