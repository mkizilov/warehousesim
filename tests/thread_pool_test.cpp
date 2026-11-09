#include "warehousesim/core/thread_pool.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {

using std::chrono::milliseconds;
using warehousesim::core::ThreadPool;

// A gate that tasks can be made to wait at until the test lets them through.
class Gate {
 public:
  void open() {
    {
      const std::scoped_lock lock(mutex_);
      open_ = true;
    }
    changed_.notify_all();
  }
  void wait() {
    std::unique_lock lock(mutex_);
    changed_.wait(lock, [this] { return open_; });
  }

 private:
  std::mutex mutex_;
  std::condition_variable changed_;
  bool open_ = false;
};

bool waitUntil(const std::atomic<bool>& flag, milliseconds timeout = milliseconds(10000)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!flag.load()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::yield();
  }
  return true;
}

// ---- parallelFor --------------------------------------------------------------------------------

TEST(ThreadPoolTest, ParallelForCallsEveryIndexExactlyOnceWhateverTheShape) {
  for (const std::size_t workers : {0U, 1U, 2U, 3U}) {
    ThreadPool pool(workers);
    for (const std::size_t count : {0U, 1U, 2U, 3U, 17U, 1000U, 20000U}) {
      std::vector<std::atomic<int>> calls(count);
      pool.parallelFor(count, [&](std::size_t, std::size_t index) { calls[index].fetch_add(1); });
      for (std::size_t i = 0; i < count; ++i) {
        ASSERT_EQ(calls[i].load(), 1) << workers << " workers, " << count << " calls, index " << i;
      }
    }
  }
}

TEST(ThreadPoolTest, TheCallingThreadIsParticipantZeroAndWorkersAreOneToN) {
  ThreadPool pool(3);
  std::set<std::size_t> seen;
  std::mutex mutex;
  const std::thread::id caller = std::this_thread::get_id();
  std::atomic<int> onCaller{0};
  pool.parallelFor(5000, [&](std::size_t participant, std::size_t) {
    if (std::this_thread::get_id() == caller) {
      onCaller.fetch_add(1);
      EXPECT_EQ(participant, 0U);
    } else {
      EXPECT_GE(participant, 1U);
      EXPECT_LE(participant, 3U);
    }
    const std::scoped_lock lock(mutex);
    seen.insert(participant);
  });
  EXPECT_GT(onCaller.load(), 0) << "the caller works as well as waits";
  for (const std::size_t participant : seen) {
    EXPECT_LE(participant, 3U);
  }
}

TEST(ThreadPoolTest, NoTwoThreadsEverShareAParticipantNumber) {
  // Scratch memory is indexed by participant, so two threads using one number at once would race.
  ThreadPool pool(3);
  std::vector<std::atomic<int>> inUse(4);
  std::atomic<int> clashes{0};
  for (int round = 0; round < 50; ++round) {
    pool.parallelFor(200, [&](std::size_t participant, std::size_t) {
      if (inUse[participant].fetch_add(1) != 0) {
        clashes.fetch_add(1);
      }
      std::this_thread::yield();
      inUse[participant].fetch_sub(1);
    });
  }
  EXPECT_EQ(clashes.load(), 0);
}

TEST(ThreadPoolTest, APoolWithNoWorkersRunsEverythingOnTheCallingThread) {
  ThreadPool pool(0);
  EXPECT_EQ(pool.workers(), 0U);
  const std::thread::id caller = std::this_thread::get_id();
  int calls = 0;
  pool.parallelFor(50, [&](std::size_t participant, std::size_t) {
    EXPECT_EQ(std::this_thread::get_id(), caller);
    EXPECT_EQ(participant, 0U);
    ++calls;
  });
  EXPECT_EQ(calls, 50);
  EXPECT_THROW(pool.submit([] {}), std::runtime_error) << "there is nobody to run it";
}

TEST(ThreadPoolTest, TheResultDoesNotDependOnHowManyThreadsShareTheWork) {
  std::vector<std::uint64_t> reference;
  for (const std::size_t workers : {0U, 1U, 2U, 3U}) {
    ThreadPool pool(workers);
    std::vector<std::uint64_t> out(3000, 0);
    pool.parallelFor(out.size(), [&](std::size_t, std::size_t index) {
      std::uint64_t x = index + 1;
      for (int i = 0; i < 50; ++i) {
        x = x * 6364136223846793005ULL + 1442695040888963407ULL;
      }
      out[index] = x;
    });
    if (workers == 0) {
      reference = out;
    }
    EXPECT_EQ(out, reference) << workers << " workers";
  }
}

TEST(ThreadPoolTest, AnExceptionIsRethrownAfterEveryOtherCallHasRun) {
  ThreadPool pool(2);
  std::atomic<int> finished{0};
  try {
    pool.parallelFor(100, [&](std::size_t, std::size_t index) {
      if (index == 40 || index == 7 || index == 90) {
        throw std::runtime_error("index " + std::to_string(index));
      }
      finished.fetch_add(1);
    });
    FAIL() << "should have thrown";
  } catch (const std::runtime_error& error) {
    EXPECT_STREQ(error.what(), "index 7") << "the lowest index wins, whoever got there first";
  }
  EXPECT_EQ(finished.load(), 97);
  // And the pool is still good.
  std::atomic<int> after{0};
  pool.parallelFor(10, [&](std::size_t, std::size_t) { after.fetch_add(1); });
  EXPECT_EQ(after.load(), 10);
}

TEST(ThreadPoolTest, ParallelForFromATaskOfThePoolIsRefusedNotDeadlocked) {
  ThreadPool pool(2);
  std::atomic<bool> refused{false};
  std::atomic<bool> done{false};
  pool.submit([&] {
    try {
      pool.parallelFor(4, [](std::size_t, std::size_t) {});
    } catch (const std::logic_error&) {
      refused = true;
    }
    done = true;
  });
  ASSERT_TRUE(waitUntil(done));
  EXPECT_TRUE(refused.load());
}

// ---- submit and the bounded queue ---------------------------------------------------------------

TEST(ThreadPoolTest, EveryAcceptedTaskRunsBeforeShutdownReturns) {
  ThreadPool pool(2, 8);
  std::atomic<int> ran{0};
  for (int i = 0; i < 500; ++i) {
    pool.submit([&] { ran.fetch_add(1); });
  }
  pool.shutdown();
  EXPECT_EQ(ran.load(), 500);
  EXPECT_EQ(pool.tasksStarted(), 500U);
}

TEST(ThreadPoolTest, TheDestructorRunsWhatWasAcceptedAndJoinsTheThreads) {
  std::atomic<int> ran{0};
  {
    ThreadPool pool(2, 4);
    for (int i = 0; i < 200; ++i) {
      pool.submit([&] { ran.fetch_add(1); });
    }
  }  // if the destructor returned early, `ran` would be gone before the workers were done with it
  EXPECT_EQ(ran.load(), 200);
}

TEST(ThreadPoolTest, SubmittingAfterShutdownIsRefusedAndParallelForStillWorks) {
  ThreadPool pool(2);
  pool.shutdown();
  EXPECT_THROW(pool.submit([] {}), std::runtime_error);
  std::vector<int> out(30, 0);
  pool.parallelFor(out.size(), [&](std::size_t, std::size_t index) { out[index] = 1; });
  EXPECT_EQ(std::count(out.begin(), out.end(), 1), 30) << "the calling thread does it all";
}

TEST(ThreadPoolTest, ShutdownIsIdempotentAndSafeFromSeveralThreads) {
  // Every worker is held inside a task and the queue is full, so all the callers of shutdown() are
  // inside it, waiting to join the same threads, at the same time.
  ThreadPool pool(3, 4);
  Gate gate;
  std::atomic<int> inside{0};
  std::atomic<int> ran{0};
  for (int i = 0; i < 3; ++i) {
    pool.submit([&] {
      inside.fetch_add(1);
      gate.wait();
      ran.fetch_add(1);
    });
  }
  const auto deadline = std::chrono::steady_clock::now() + milliseconds(10000);
  while (inside.load() < 3 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  ASSERT_EQ(inside.load(), 3);
  for (int i = 0; i < 4; ++i) {
    pool.submit([&] { ran.fetch_add(1); });
  }
  std::vector<std::thread> callers;
  for (int i = 0; i < 4; ++i) {
    callers.emplace_back([&] { pool.shutdown(); });
  }
  std::this_thread::sleep_for(milliseconds(50));  // let them all get in
  gate.open();
  for (std::thread& caller : callers) {
    caller.join();
  }
  pool.shutdown();
  EXPECT_EQ(ran.load(), 7);
}

TEST(ThreadPoolTest, TheStartedCountCanBeReadWhileWorkersAreRunningTasks) {
  ThreadPool pool(2, 8);
  std::atomic<int> ran{0};
  std::thread producer([&] {
    for (int i = 0; i < 3000; ++i) {
      pool.submit([&] { ran.fetch_add(1); });
    }
  });
  std::size_t last = 0;
  const auto deadline = std::chrono::steady_clock::now() + milliseconds(10000);
  while (last < 3000 && std::chrono::steady_clock::now() < deadline) {
    const std::size_t now = pool.tasksStarted();
    ASSERT_GE(now, last) << "it never goes down";
    last = now;
  }
  producer.join();
  pool.shutdown();
  EXPECT_EQ(last, 3000U);
  EXPECT_EQ(ran.load(), 3000);
}

TEST(ThreadPoolTest, TheQueueIsBoundedAProducerWaitsWhenItIsFull) {
  ThreadPool pool(1, 2);
  Gate gate;
  std::atomic<bool> workerBusy{false};
  std::atomic<int> ran{0};
  pool.submit([&] {
    workerBusy = true;
    gate.wait();
    ran.fetch_add(1);
  });
  ASSERT_TRUE(waitUntil(workerBusy));  // the one worker is held; the queue is empty
  pool.submit([&] { ran.fetch_add(1); });
  pool.submit([&] { ran.fetch_add(1); });  // the queue holds two: it is full now

  std::atomic<bool> producerStarted{false};
  std::atomic<bool> producerReturned{false};
  std::thread producer([&] {
    producerStarted = true;
    pool.submit([&] { ran.fetch_add(1); });
    producerReturned = true;
  });
  ASSERT_TRUE(waitUntil(producerStarted));
  std::this_thread::sleep_for(milliseconds(50));  // only to see that something does not happen
  EXPECT_FALSE(producerReturned.load()) << "submit must wait while the queue is full";

  gate.open();
  producer.join();
  pool.shutdown();
  EXPECT_EQ(ran.load(), 4);
}

TEST(ThreadPoolTest, AProducerBlockedOnAFullQueueIsWokenByShutdownAndToldSo) {
  ThreadPool pool(1, 1);
  Gate gate;
  std::atomic<bool> workerBusy{false};
  std::atomic<int> ran{0};
  pool.submit([&] {
    workerBusy = true;
    gate.wait();
    ran.fetch_add(1);
  });
  ASSERT_TRUE(waitUntil(workerBusy));
  pool.submit([&] { ran.fetch_add(1); });  // fills the queue

  std::atomic<bool> producerStarted{false};
  std::atomic<bool> refused{false};
  std::atomic<bool> producerDone{false};
  std::thread producer([&] {
    producerStarted = true;
    try {
      pool.submit([&] { ran.fetch_add(100); });
    } catch (const std::runtime_error&) {
      refused = true;
    }
    producerDone = true;
  });
  ASSERT_TRUE(waitUntil(producerStarted));
  std::this_thread::sleep_for(milliseconds(30));  // let it block (it cannot be seen to)

  std::thread stopper([&] { pool.shutdown(); });
  ASSERT_TRUE(waitUntil(producerDone)) << "shutdown must wake a blocked producer";
  EXPECT_TRUE(refused.load());
  gate.open();
  stopper.join();
  producer.join();
  EXPECT_EQ(ran.load(), 2) << "the two accepted tasks ran; the refused one did not";
}

TEST(ThreadPoolTest, ManyProducersAndAQueueTooSmallForThemLoseNothingAndDoNotDeadlock) {
  ThreadPool pool(2, 3);
  std::atomic<int> ran{0};
  std::vector<std::thread> producers;
  for (int p = 0; p < 4; ++p) {
    producers.emplace_back([&] {
      for (int i = 0; i < 500; ++i) {
        pool.submit([&] { ran.fetch_add(1); });
      }
    });
  }
  for (std::thread& producer : producers) {
    producer.join();
  }
  pool.shutdown();
  EXPECT_EQ(ran.load(), 2000);
}

TEST(ThreadPoolTest, ParallelForsFromSeveralThreadsAtOnceEachGetTheirOwnResult) {
  ThreadPool pool(2, 4);
  std::vector<std::thread> callers;
  std::atomic<int> wrong{0};
  for (int c = 0; c < 3; ++c) {
    callers.emplace_back([&, c] {
      for (int round = 0; round < 40; ++round) {
        std::vector<int> out(64, -1);
        pool.parallelFor(out.size(), [&](std::size_t, std::size_t i) {
          out[i] = static_cast<int>(i) + c * 1000;
        });
        for (std::size_t i = 0; i < out.size(); ++i) {
          wrong += out[i] != static_cast<int>(i) + c * 1000 ? 1 : 0;
        }
      }
    });
  }
  for (std::thread& caller : callers) {
    caller.join();
  }
  EXPECT_EQ(wrong.load(), 0);
}

TEST(ThreadPoolTest, ABusyPoolCanBeDestroyedWhileWorkersAreStillInsideTasks) {
  // The workers are inside a task when the destructor starts; it must wait for them, not abandon
  // them or run the queue on the wrong thread.
  std::atomic<int> ran{0};
  Gate gate;
  std::atomic<bool> inside{false};
  std::thread releaser;
  {
    ThreadPool pool(2, 4);
    pool.submit([&] {
      inside = true;
      gate.wait();
      ran.fetch_add(1);
    });
    pool.submit([&] { ran.fetch_add(1); });
    ASSERT_TRUE(waitUntil(inside));
    releaser = std::thread([&] {
      std::this_thread::sleep_for(milliseconds(30));
      gate.open();
    });
  }
  releaser.join();
  EXPECT_EQ(ran.load(), 2);
}

TEST(ThreadPoolTest, AnImpossibleWorkerCountDoesNotLeaveThreadsBehind) {
  // Capacity of zero is raised to one.
  ThreadPool pool(1, 0);
  EXPECT_EQ(pool.queueCapacity(), 1U);
  std::atomic<int> ran{0};
  for (int i = 0; i < 20; ++i) {
    pool.submit([&] { ran.fetch_add(1); });
  }
  pool.shutdown();
  EXPECT_EQ(ran.load(), 20);
}

}  // namespace
