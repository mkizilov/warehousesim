#include "warehousesim/planning/path_batch.hpp"

#include <stdexcept>
#include <utility>

namespace warehousesim::planning {

namespace {

constexpr std::size_t kMaxWorkers = 64;

// Checked before the pool is made, so that a silly request starts no threads.
PathBatchExecutor::Options validated(PathBatchExecutor::Options options) {
  if (options.workers > kMaxWorkers) {
    throw std::invalid_argument("at most 64 planning threads");
  }
  if (options.minParallelQueries == 0) {
    throw std::invalid_argument("minParallelQueries must be at least 1");
  }
  return options;
}

}  // namespace

PathBatchExecutor::PathBatchExecutor(Options options)
    : options_(validated(options)), pool_(options_.workers, options_.queueCapacity) {
  planners_.reserve(options.workers + 1);
  for (std::size_t i = 0; i <= options.workers; ++i) {
    planners_.push_back(std::make_unique<AStarPlanner>());
  }
  scratch_.resize(options.workers + 1);
}

void PathBatchExecutor::run(const world::Warehouse& warehouse, std::span<const PathQuery> queries,
                            std::vector<PathAnswer>& answers, bool keepPaths) {
  answers.resize(queries.size());
  ++stats_.batches;
  stats_.queries += queries.size();
  if (queries.empty()) {
    return;
  }
  // The workers read this copy, which nothing changes until the next batch finds the layout
  // different; equal versions mean equal layouts.
  if (!snapshot_ || snapshot_->version() != warehouse.version()) {
    snapshot_ = std::make_shared<const world::Warehouse>(warehouse);
  }

  const world::Warehouse& layout = *snapshot_;
  const auto answer = [&](std::size_t participant, std::size_t index) {
    const PathQuery& query = queries[index];
    PathAnswer& slot = answers[index];
    Path& route = keepPaths ? slot.path : scratch_[participant];
    const SearchResult result =
        planners_[participant]->search(layout, query.start, query.goal, noBlocked_, route);
    slot.status = result.status;
    slot.cost = result.cost;
    slot.expansions = result.expansions;
    if (!keepPaths) {
      slot.path.clear();
    }
  };

  if (options_.workers > 0 && queries.size() >= options_.minParallelQueries) {
    ++stats_.parallelBatches;
    pool_.parallelFor(queries.size(), answer);
  } else {
    for (std::size_t i = 0; i < queries.size(); ++i) {
      answer(0, i);
    }
  }
}

}  // namespace warehousesim::planning
