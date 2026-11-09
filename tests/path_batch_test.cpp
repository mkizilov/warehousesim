#include "warehousesim/planning/path_batch.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/planning/astar_planner.hpp"
#include "warehousesim/testing/map_generators.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::planning::AStarPlanner;
using warehousesim::planning::BlockedSet;
using warehousesim::planning::Path;
using warehousesim::planning::PathAnswer;
using warehousesim::planning::PathBatchExecutor;
using warehousesim::planning::PathQuery;
using warehousesim::planning::SearchStatus;
using warehousesim::world::CellType;
using warehousesim::world::Warehouse;

Point randomCell(Rng& rng, const Warehouse& w) {
  return Point{static_cast<std::int32_t>(rng.below(static_cast<std::uint64_t>(w.width()))),
               static_cast<std::int32_t>(rng.below(static_cast<std::uint64_t>(w.height())))};
}

std::vector<PathQuery> randomQueries(Rng& rng, const Warehouse& w, std::size_t count) {
  std::vector<PathQuery> queries;
  for (std::size_t i = 0; i < count; ++i) {
    queries.push_back({randomCell(rng, w), randomCell(rng, w)});
  }
  return queries;
}

// What one planner on this thread says, question by question: the standard the batch is held to.
std::vector<PathAnswer> oneByOne(const Warehouse& w, const std::vector<PathQuery>& queries) {
  AStarPlanner planner;
  std::vector<PathAnswer> answers(queries.size());
  for (std::size_t i = 0; i < queries.size(); ++i) {
    const auto result =
        planner.search(w, queries[i].start, queries[i].goal, BlockedSet{}, answers[i].path);
    answers[i].status = result.status;
    answers[i].cost = result.cost;
    answers[i].expansions = result.expansions;
  }
  return answers;
}

void expectSame(const std::vector<PathAnswer>& got, const std::vector<PathAnswer>& want,
                const std::string& what) {
  ASSERT_EQ(got.size(), want.size()) << what;
  for (std::size_t i = 0; i < got.size(); ++i) {
    ASSERT_TRUE(got[i].status == want[i].status) << what << " question " << i;
    ASSERT_EQ(got[i].cost, want[i].cost) << what << " question " << i;
    ASSERT_EQ(got[i].expansions, want[i].expansions) << what << " question " << i;
    ASSERT_EQ(got[i].path, want[i].path) << what << " question " << i;
  }
}

TEST(PathBatchTest, AnswersAreTheOnesAPlannerGivesQuestionByQuestion) {
  Rng rng(21);
  for (int map = 0; map < 12; ++map) {
    const Warehouse w = warehousesim::testing::makeRandomObstacleMap(
        8 + static_cast<int>(rng.below(30)), 8 + static_cast<int>(rng.below(30)),
        static_cast<std::uint32_t>(rng.below(350)), rng);
    const auto queries = randomQueries(rng, w, 1 + rng.below(80));
    const auto want = oneByOne(w, queries);
    PathBatchExecutor executor({0, 1, 64});
    std::vector<PathAnswer> got;
    executor.run(w, queries, got, true);
    expectSame(got, want, "map " + std::to_string(map));
  }
}

TEST(PathBatchTest, EveryNumberOfWorkersGivesExactlyTheSameAnswers) {
  Rng rng(22);
  for (int map = 0; map < 6; ++map) {
    const Warehouse w = warehousesim::testing::makeRandomObstacleMap(40, 30, 220, rng);
    const auto queries = randomQueries(rng, w, 300);
    const auto want = oneByOne(w, queries);
    for (const std::size_t workers : {0U, 1U, 2U, 3U}) {
      PathBatchExecutor executor({workers, 1, 64});
      std::vector<PathAnswer> got;
      for (int repeat = 0; repeat < 3; ++repeat) {  // and again with the answers array reused
        executor.run(w, queries, got, true);
        expectSame(got, want, std::to_string(workers) + " workers, map " + std::to_string(map));
      }
    }
  }
}

TEST(PathBatchTest, TheOrderOfTheAnswersIsTheOrderOfTheQuestionsAndEveryStatusIsReported) {
  const Warehouse w = *Warehouse::fromRows({"..#..", "..#..", "....."});
  const std::vector<PathQuery> queries{
      {Point{0, 0}, Point{4, 0}},  // around the wall: found
      {Point{0, 0}, Point{0, 0}},  // already there: found, empty
      {Point{2, 0}, Point{0, 0}},  // starts in the wall
      {Point{0, 0}, Point{2, 1}},  // ends in the wall
      {Point{0, 0}, Point{9, 9}},  // off the grid
      {Point{4, 2}, Point{0, 2}},  // straight along the bottom
  };
  for (const std::size_t workers : {0U, 2U}) {
    PathBatchExecutor executor({workers, 1, 64});
    std::vector<PathAnswer> answers;
    executor.run(w, queries, answers, true);
    ASSERT_EQ(answers.size(), queries.size());
    EXPECT_TRUE(answers[0].status == SearchStatus::Found);
    EXPECT_EQ(answers[0].cost, 8);
    EXPECT_EQ(answers[0].path.size(), 8U);
    EXPECT_TRUE(answers[1].status == SearchStatus::Found);
    EXPECT_TRUE(answers[1].path.empty());
    EXPECT_TRUE(answers[2].status == SearchStatus::InvalidStart);
    EXPECT_TRUE(answers[3].status == SearchStatus::InvalidGoal);
    EXPECT_TRUE(answers[4].status == SearchStatus::InvalidGoal);
    EXPECT_FALSE(answers[2].found());
    EXPECT_TRUE(answers[5].found());
    EXPECT_EQ(answers[5].cost, 4);
  }
}

TEST(PathBatchTest, WithoutKeepPathsTheLengthIsKnownAndNoRouteIsKept) {
  Rng rng(23);
  const Warehouse w = warehousesim::testing::makeRandomObstacleMap(20, 20, 150, rng);
  const auto queries = randomQueries(rng, w, 60);
  const auto want = oneByOne(w, queries);
  PathBatchExecutor executor({2, 1, 64});
  std::vector<PathAnswer> got;
  for (const bool keep : {false, true, false}) {  // and the same answers array, reused
    executor.run(w, queries, got, keep);
    ASSERT_EQ(got.size(), want.size());
    std::size_t kept = 0;
    for (std::size_t i = 0; i < got.size(); ++i) {
      EXPECT_TRUE(got[i].status == want[i].status);
      EXPECT_EQ(got[i].cost, want[i].cost);
      EXPECT_EQ(got[i].path.empty(), !keep || want[i].path.empty()) << "keep " << keep;
      kept += got[i].path.empty() ? 0 : 1;
    }
    EXPECT_EQ(kept == 0, !keep);
  }
}

TEST(PathBatchTest, AnEmptyBatchAnswersNothingAndShrinksTheAnswers) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(5, 5);
  PathBatchExecutor executor({2, 1, 64});
  std::vector<PathAnswer> answers(7);
  executor.run(w, {}, answers, true);
  EXPECT_TRUE(answers.empty());
  EXPECT_EQ(executor.stats().batches, 1U);
  EXPECT_EQ(executor.stats().queries, 0U);
}

TEST(PathBatchTest, ABatchIsAnsweredFromACopyOfTheLayoutTheCallerCanChangeAfterwards) {
  Warehouse w = warehousesim::testing::makeOpenGrid(6, 1);
  const std::vector<PathQuery> queries{{Point{0, 0}, Point{5, 0}}};
  PathBatchExecutor executor({2, 1, 64});
  std::vector<PathAnswer> answers;

  executor.run(w, queries, answers, true);
  ASSERT_TRUE(answers[0].found());
  EXPECT_EQ(answers[0].cost, 5);

  // Wall off the corridor: the next batch must see it ...
  w.set(Point{3, 0}, CellType::Wall);
  executor.run(w, queries, answers, true);
  EXPECT_FALSE(answers[0].found()) << "a changed layout must not be answered from the old copy";

  // ... and opening it again must see that too, not remember the wall.
  w.set(Point{3, 0}, CellType::Free);
  executor.run(w, queries, answers, true);
  EXPECT_TRUE(answers[0].found());
  EXPECT_EQ(answers[0].cost, 5);
}

TEST(PathBatchTest, ACopyOfTheLayoutIsTheSameLayoutAndAnEditedOneIsNot) {
  Warehouse w = warehousesim::testing::makeOpenGrid(4, 4);
  const Warehouse copy = w;
  EXPECT_EQ(copy.version(), w.version());
  w.set(Point{1, 1}, CellType::Shelf);
  EXPECT_NE(copy.version(), w.version());
  const Warehouse another = warehousesim::testing::makeOpenGrid(4, 4);
  EXPECT_NE(another.version(), copy.version())
      << "two separately made layouts are never 'the same'";
}

TEST(PathBatchTest, SmallBatchesStayOnTheCallingThreadAndBigOnesGoToThePool) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(10, 10);
  Rng rng(24);
  PathBatchExecutor executor({2, 16, 64});
  std::vector<PathAnswer> answers;
  executor.run(w, randomQueries(rng, w, 15), answers, true);
  EXPECT_EQ(executor.stats().batches, 1U);
  EXPECT_EQ(executor.stats().parallelBatches, 0U);
  executor.run(w, randomQueries(rng, w, 16), answers, true);
  EXPECT_EQ(executor.stats().batches, 2U);
  EXPECT_EQ(executor.stats().parallelBatches, 1U);
  EXPECT_EQ(executor.stats().queries, 31U);

  PathBatchExecutor alone({0, 1, 64});
  alone.run(w, randomQueries(rng, w, 500), answers, true);
  EXPECT_EQ(alone.stats().parallelBatches, 0U) << "with no workers nothing is parallel";
}

TEST(PathBatchTest, SillyOptionsAreRefusedBeforeAnyThreadIsStarted) {
  EXPECT_THROW(PathBatchExecutor({65, 16, 64}), std::invalid_argument);
  EXPECT_THROW(PathBatchExecutor({2, 0, 64}), std::invalid_argument);
  EXPECT_NO_THROW(PathBatchExecutor({64, 16, 1}));
}

TEST(PathBatchTest, SeveralExecutorsCanRunAtTheSameTimeFromDifferentThreads) {
  Rng rng(25);
  const Warehouse w = warehousesim::testing::makeRandomObstacleMap(30, 30, 200, rng);
  const auto queries = randomQueries(rng, w, 200);
  const auto want = oneByOne(w, queries);
  std::vector<std::thread> threads;
  std::vector<int> ok(3, 0);
  for (std::size_t t = 0; t < 3; ++t) {
    threads.emplace_back([&, t] {
      PathBatchExecutor executor({2, 1, 8});
      std::vector<PathAnswer> got;
      bool same = true;
      for (int round = 0; round < 10; ++round) {
        executor.run(w, queries, got, true);
        for (std::size_t i = 0; i < got.size(); ++i) {
          same = same && got[i].path == want[i].path && got[i].cost == want[i].cost;
        }
      }
      ok[t] = same ? 1 : 0;
    });
  }
  for (std::thread& thread : threads) {
    thread.join();
  }
  EXPECT_EQ(ok, (std::vector<int>{1, 1, 1}));
}

}  // namespace
