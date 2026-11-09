#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/core/thread_pool.hpp"
#include "warehousesim/planning/astar_planner.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::planning {

// One question for the planner: the shortest way from `start` to `goal` on the empty map.
struct PathQuery {
  core::Point start;
  core::Point goal;
};

// The answer to one PathQuery, as AStarPlanner::search gives it.
struct PathAnswer {
  SearchStatus status = SearchStatus::Unreachable;
  std::int32_t cost = 0;       // steps, when found
  std::size_t expansions = 0;  // the work it took (AStarPlanner::work() counts the same)
  Path path;                   // when found, and only if the batch was asked to keep paths

  [[nodiscard]] bool found() const noexcept { return status == SearchStatus::Found; }
};

// Answers many independent path questions at once, on a bounded pool of worker threads.
//
// Each question is a pure function of the layout and its two cells, so the answers are the same
// whether they are worked out on one thread or spread over several, and in whatever order: they
// are written to the slot of their question, and the caller reads them in the order it asked. Which
// thread did what is invisible. That is the whole contract that lets a simulation use the pool and
// still be deterministic.
//
// * Immutable snapshot: the workers read a private copy of the layout, made when the layout last
//   changed (Warehouse::version), never the caller's. The caller keeps changing its own, as it
//   likes, between batches.
// * One writer: the executor changes nothing but the answers it is handed. Applying them (to
//   robots, tasks, the world) is left to the caller, on its own thread, in its own order.
// * Each thread has its own planner (and so its own scratch memory); nothing is shared but the
//   snapshot and the question and answer arrays, whose slots are each touched by one thread.
// * Small batches are not worth waking a thread for and run on the calling thread, with the same
//   planner code (minParallelQueries).
//
// Not thread-safe itself: run() is called by one thread at a time.
class PathBatchExecutor {
 public:
  struct Options {
    // Worker threads besides the calling thread. 0: everything runs on the calling thread.
    std::size_t workers = 0;
    // Batches with fewer questions than this run on the calling thread alone.
    std::size_t minParallelQueries = 16;
    // How many tasks may wait for a worker (see core::ThreadPool).
    std::size_t queueCapacity = 64;
  };
  struct Stats {
    std::uint64_t batches = 0;
    std::uint64_t parallelBatches = 0;  // ... of which were spread over the pool
    std::uint64_t queries = 0;
  };

  // Throws std::invalid_argument for more than 64 workers or a minParallelQueries of 0.
  explicit PathBatchExecutor(Options options);

  [[nodiscard]] const Options& options() const noexcept { return options_; }
  [[nodiscard]] const Stats& stats() const noexcept { return stats_; }

  // Answers every question on the empty map of `warehouse`: answers[i] is for queries[i]. With
  // `keepPaths` the routes of the ones found are kept in the answers, otherwise only their length.
  // `answers` is resized to fit; the paths it already holds are reused. Blocks until all are
  // answered. If a planner throws, the exception of the lowest question is rethrown.
  void run(const world::Warehouse& warehouse, std::span<const PathQuery> queries,
           std::vector<PathAnswer>& answers, bool keepPaths);

 private:
  Options options_;
  core::ThreadPool pool_;
  std::vector<std::unique_ptr<AStarPlanner>> planners_;  // by participant: 0 is the caller
  std::vector<Path> scratch_;                            // by participant: for lengths only
  std::shared_ptr<const world::Warehouse> snapshot_;
  BlockedSet noBlocked_;
  Stats stats_;
};

}  // namespace warehousesim::planning
