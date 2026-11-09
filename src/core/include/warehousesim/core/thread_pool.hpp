#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace warehousesim::core {

// A fixed number of worker threads fed from a bounded queue.
//
// * Bounded twice over: the number of threads is fixed when the pool is made, and so is the number
//   of tasks that may wait. submit() blocks while the queue is full instead of growing it.
// * Shutdown is safe from any thread and from the destructor: no new task is accepted, every task
//   that was accepted still runs, then the threads are joined. A producer blocked on a full queue
//   is woken and told the pool is shut down. Nothing is abandoned half way and nothing is left
//   running after the pool is gone.
// * parallelFor() is the usual way in: it hands a loop to the workers and the calling thread and
//   returns when every iteration has finished. What each iteration writes is the caller's business
//   (one slot per index keeps it free of races, and gives the same result however the iterations
//   happen to be spread over the threads).
//
// Tasks must not call parallelFor() or wait for the pool from a worker thread.
class ThreadPool {
 public:
  // `workers` may be 0: a pool with no threads accepts nothing but parallelFor() still works (the
  // calling thread does all of it). `queueCapacity` is at least 1.
  explicit ThreadPool(std::size_t workers, std::size_t queueCapacity = 64);
  ~ThreadPool();
  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  [[nodiscard]] std::size_t workers() const noexcept { return threads_.size(); }
  [[nodiscard]] std::size_t queueCapacity() const noexcept { return capacity_; }

  // Queues a task for a worker. Blocks while the queue is full. Throws std::runtime_error if the
  // pool has been shut down (or has no threads). An exception escaping the task ends the program,
  // as it would on any thread: use parallelFor() or wrap the task if it can throw.
  void submit(std::function<void()> task);

  // Calls body(participant, index) once for every index in [0, count), spread over the workers and
  // the calling thread, and returns when all calls have returned. `participant` is 0 for the
  // calling thread and 1..workers() for the workers, so that each can use its own scratch memory.
  // If calls throw, every other call still runs, and then the exception of the lowest index is
  // rethrown. Works on a pool that has been shut down (the calling thread does all of it).
  // Throws std::logic_error if called from one of the pool's own threads.
  void parallelFor(std::size_t count,
                   const std::function<void(std::size_t participant, std::size_t index)>& body);

  // Stops accepting tasks, lets the queue empty and joins the threads. Safe to call more than once
  // and from several threads; the destructor calls it.
  void shutdown();

  // How many tasks the workers have started since the pool was made (for tests and benchmarks).
  [[nodiscard]] std::size_t tasksStarted() const;

 private:
  void workerLoop(std::size_t participant);

  std::size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable notEmpty_;
  std::condition_variable notFull_;
  std::deque<std::function<void()>> queue_;
  bool stopping_ = false;
  std::size_t started_ = 0;
  std::vector<std::thread> threads_;
  std::mutex joinMutex_;  // makes shutdown() from two threads join once
};

}  // namespace warehousesim::core
