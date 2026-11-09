#include "warehousesim/planning/space_time_astar.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>
#include <utility>

namespace warehousesim::planning {

using core::Point;
using core::Tick;

namespace {

constexpr std::size_t kMaxWindow = 256;
// How many goals count as cached for the purpose of charging work (see distanceField): a goal is
// charged for when it is not among the last this many.
constexpr std::size_t kMaxCachedFields = 24;
// How many fields are kept to be reused, so that the charged ones are not swept again: at most this
// many, and at most what fits in this many bytes (but never fewer than the ones charged for), so a
// big map keeps a few and a small one many.
constexpr std::size_t kMaxStoredFields = 512;
constexpr std::size_t kStoredFieldBytes = 32U << 20U;

// North, east, south, west: the order neighbours are tried in.
constexpr std::array<Point, 4> kOffsets = {{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}};

// Which cells are walkable, as one number (FNV-1a), so a cached distance field is only reused on
// the layout it was computed for.
std::uint64_t layoutFingerprint(const world::Warehouse& warehouse) {
  const world::GridSize size = warehouse.size();
  std::uint64_t hash = 14695981039346656037ULL;
  const auto mix = [&hash](std::uint64_t value) {
    hash ^= value;
    hash *= 1099511628211ULL;
  };
  mix(static_cast<std::uint64_t>(size.width()));
  mix(static_cast<std::uint64_t>(size.height()));
  for (std::size_t i = 0; i < size.cellCount(); ++i) {
    mix(warehouse.isWalkable(size.pointAt(i)) ? 1U : 0U);
  }
  return hash;
}

// Larger f first would be a max-heap; std::push_heap wants "a comes after b".
struct After {
  template <class Entry>
  bool operator()(const Entry& a, const Entry& b) const noexcept {
    if (a.f != b.f) {
      return a.f > b.f;
    }
    if (a.h != b.h) {
      return a.h > b.h;
    }
    return a.id > b.id;
  }
};

}  // namespace

const char* toString(SpaceTimeStatus status) noexcept {
  switch (status) {
    case SpaceTimeStatus::Found:
      return "found";
    case SpaceTimeStatus::Partial:
      return "partial";
    case SpaceTimeStatus::NoPlan:
      return "no-plan";
    case SpaceTimeStatus::BudgetExceeded:
      return "budget-exceeded";
    case SpaceTimeStatus::InvalidStart:
      return "invalid-start";
    case SpaceTimeStatus::InvalidGoal:
      return "invalid-goal";
  }
  return "unknown";
}

const std::vector<std::int32_t>* SpaceTimeAStar::distanceField(const world::Warehouse& warehouse,
                                                               Point goal, std::size_t& work,
                                                               std::size_t maxWork) {
  const world::GridSize size = warehouse.size();
  // Looking at every cell for each question costs more than the questions themselves on a big
  // layout, so the fingerprint is only worked out again when the layout has changed.
  if (warehouse.version() != fingerprintVersion_) {
    fingerprint_ = layoutFingerprint(warehouse);
    fingerprintVersion_ = warehouse.version();
  }
  const std::uint64_t layout = fingerprint_;
  const std::size_t goalIndex = size.indexOf(goal);

  // Work is charged as if only the last kMaxCachedFields goals were remembered (`recent_`), which
  // is what the planning limits were tuned against. The fields themselves are kept for longer
  // (`fields_`), so that a goal that has to be charged for again is not swept again.
  const bool charged = std::none_of(recent_.begin(), recent_.end(),
                                    [&](const std::pair<std::uint64_t, std::size_t>& r) {
                                      return r.first == layout && r.second == goalIndex;
                                    });
  if (charged && maxWork != 0 && work + size.cellCount() > maxWork) {
    return nullptr;  // a fill visits every cell it can reach: more than this much, at worst
  }

  Field* stored = nullptr;
  for (Field& field : fields_) {
    if (field.layout == layout && field.goal == goalIndex) {
      stored = &field;
      break;
    }
  }
  if (stored == nullptr) {
    const std::size_t keep =
        std::clamp(kStoredFieldBytes / (size.cellCount() * sizeof(std::int32_t)), kMaxCachedFields,
                   kMaxStoredFields);
    while (fields_.size() >= keep) {
      fields_.erase(fields_.begin());
    }
    Field& field = fields_.emplace_back();
    field.layout = layout;
    field.goal = goalIndex;
    field.distance.assign(size.cellCount(), -1);
    queue_.clear();
    queue_.push_back(static_cast<std::uint32_t>(goalIndex));
    field.distance[goalIndex] = 0;
    for (std::size_t head = 0; head < queue_.size(); ++head) {
      const std::uint32_t index = queue_[head];
      const Point at = size.pointAt(index);
      for (const Point offset : kOffsets) {
        const Point next{at.x + offset.x, at.y + offset.y};
        if (!warehouse.isWalkable(next)) {
          continue;
        }
        const std::size_t nextIndex = size.indexOf(next);
        if (field.distance[nextIndex] < 0) {
          field.distance[nextIndex] = field.distance[index] + 1;
          queue_.push_back(static_cast<std::uint32_t>(nextIndex));
        }
      }
    }
    field.reached = queue_.size();
    ++sweeps_;
    stored = &field;
  }

  if (charged) {
    work += stored->reached;
    if (recent_.size() >= kMaxCachedFields) {
      recent_.erase(recent_.begin());
    }
    recent_.emplace_back(layout, goalIndex);
  }
  return &stored->distance;
}
std::optional<std::size_t> SpaceTimeAStar::prepare(const world::Warehouse& warehouse, Point goal,
                                                   std::size_t maxWork) {
  if (!warehouse.isWalkable(goal)) {
    throw std::invalid_argument("the goal must be a walkable cell");
  }
  std::size_t work = 0;
  if (distanceField(warehouse, goal, work, maxWork) == nullptr) {
    return std::nullopt;
  }
  return work;
}

std::int32_t SpaceTimeAStar::distance(const world::Warehouse& warehouse, Point from, Point goal) {
  if (!warehouse.isWalkable(from) || !warehouse.isWalkable(goal)) {
    return -1;
  }
  std::size_t ignored = 0;
  return (*distanceField(warehouse, goal, ignored, 0))[warehouse.size().indexOf(from)];
}

void SpaceTimeAStar::trace(std::uint32_t terminal, std::size_t cells, const world::GridSize& size,
                           SpaceTimePlan& plan) const {
  std::vector<Point> reversed;
  std::uint32_t id = terminal;
  while (true) {
    reversed.push_back(size.pointAt(id % cells));
    if (id < cells) {
      break;  // depth 0: the start
    }
    id = nodes_[id].parent;
  }
  plan.timeline.assign(reversed.rbegin(), reversed.rend());
  plan.moves = 0;
  plan.waits = 0;
  for (std::size_t i = 1; i < plan.timeline.size(); ++i) {
    (plan.timeline[i] == plan.timeline[i - 1] ? plan.waits : plan.moves) += 1;
  }
}

SpaceTimePlan SpaceTimeAStar::search(const world::Warehouse& warehouse, Point start, Point goal,
                                     Tick startStep, const SpaceTimeConstraints& constraints,
                                     const SpaceTimeOptions& options) {
  if (options.window == 0 || options.window > kMaxWindow) {
    throw std::invalid_argument("the search window must be between 1 and 256 steps");
  }
  SpaceTimePlan plan;
  if (!warehouse.isWalkable(start)) {
    plan.status = SpaceTimeStatus::InvalidStart;
    return plan;
  }
  if (!warehouse.isWalkable(goal)) {
    plan.status = SpaceTimeStatus::InvalidGoal;
    return plan;
  }

  const world::GridSize size = warehouse.size();
  const std::size_t cells = size.cellCount();
  const std::size_t window = options.window;
  const std::size_t stateCount = (window + 1) * cells;
  if (stateCount >= std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("the grid is too large to search in space and time");
  }

  const std::vector<std::int32_t>* fieldPointer =
      distanceField(warehouse, goal, plan.work, options.maxWork);
  if (fieldPointer == nullptr) {
    plan.status = SpaceTimeStatus::BudgetExceeded;
    return plan;
  }
  const std::vector<std::int32_t>& field = *fieldPointer;
  const std::size_t startIndex = size.indexOf(start);
  const std::size_t goalIndex = size.indexOf(goal);
  if (field[startIndex] < 0) {
    plan.status = SpaceTimeStatus::NoPlan;  // not even an empty map connects them
    return plan;
  }
  const auto overBudget = [&] { return options.maxWork != 0 && plan.work > options.maxWork; };
  if (overBudget()) {
    plan.status = SpaceTimeStatus::BudgetExceeded;
    return plan;
  }

  if (nodes_.size() < stateCount) {
    nodes_.resize(stateCount);
  }
  if (++generation_ == 0) {
    for (Node& node : nodes_) {
      node = Node{};
    }
    generation_ = 1;
  }
  open_.clear();

  const auto root = static_cast<std::uint32_t>(startIndex);
  nodes_[root] = Node{generation_, 0, root, 0};
  open_.push_back({field[startIndex], field[startIndex], root});

  // The goal is only reached for good if nobody needs it again before the window ends.
  const auto holdsGoal = [&](std::size_t depth) {
    for (std::size_t later = depth + 1; later <= window; ++later) {
      if (!constraints.cellFree(goal, startStep + later)) {
        return false;
      }
    }
    return true;
  };

  while (!open_.empty()) {
    std::pop_heap(open_.begin(), open_.end(), After{});
    const OpenEntry entry = open_.back();
    open_.pop_back();
    Node& node = nodes_[entry.id];
    if (node.closed == generation_) {
      continue;
    }
    node.closed = generation_;

    const std::size_t depth = entry.id / cells;
    const std::size_t index = entry.id % cells;
    if (index == goalIndex) {
      if (holdsGoal(depth)) {
        plan.status = SpaceTimeStatus::Found;
        trace(entry.id, cells, size, plan);
        plan.remaining = 0;
        return plan;
      }
      if (options.goalIsTerminal) {
        continue;  // it would have to leave and come back
      }
    }
    if (depth == window) {
      // The first state of the last layer off the open list has the smallest f, so the smallest h.
      plan.status = SpaceTimeStatus::Partial;
      trace(entry.id, cells, size, plan);
      plan.remaining = field[index];
      return plan;
    }
    ++plan.work;
    if (overBudget()) {
      plan.status = SpaceTimeStatus::BudgetExceeded;
      return plan;
    }

    const Point here = size.pointAt(index);
    const Tick step = startStep + depth;
    const auto push = [&](std::size_t nextIndex, bool isMove) {
      const auto id = static_cast<std::uint32_t>((depth + 1) * cells + nextIndex);
      const std::uint32_t moves = node.moves + (isMove ? 1U : 0U);
      Node& next = nodes_[id];
      if (next.stamp != generation_) {
        next = Node{generation_, 0, entry.id, moves};
        const std::int32_t h = field[nextIndex];
        open_.push_back({static_cast<std::int32_t>(depth + 1) + h, h, id});
        std::push_heap(open_.begin(), open_.end(), After{});
      } else if (next.closed != generation_ && moves < next.moves) {
        next.parent = entry.id;  // as fast, with fewer moves: prefer waiting to wandering
        next.moves = moves;
      }
    };

    for (const Point offset : kOffsets) {
      const Point there{here.x + offset.x, here.y + offset.y};
      if (!warehouse.isWalkable(there)) {
        continue;
      }
      const std::size_t thereIndex = size.indexOf(there);
      if (constraints.cellFree(there, step) && constraints.cellFree(there, step + 1) &&
          constraints.edgeFree(here, there, step)) {
        push(thereIndex, true);
      }
    }
    if (constraints.cellFree(here, step + 1)) {
      push(index, false);
    }
  }

  plan.status = SpaceTimeStatus::NoPlan;
  return plan;
}

}  // namespace warehousesim::planning
