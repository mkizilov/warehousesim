#include "warehousesim/core/thread_pool.hpp"

#include <algorithm>
#include <atomic>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace warehousesim::core {

namespace {

// Which pool the current thread belongs to, if any, and as which participant.
thread_local const ThreadPool* currentPool = nullptr;
thread_local std::size_t currentParticipant = 0;

}  // namespace

ThreadPool::ThreadPool(std::size_t workers, std::size_t queueCapacity)
    : capacity_(std::max<std::size_t>(1, queueCapacity)) {
  threads_.reserve(workers);
  try {
    for (std::size_t i = 0; i < workers; ++i) {
      threads_.emplace_back([this, i] { workerLoop(i + 1); });
    }
  } catch (...) {
    shutdown();  // the threads that did start must not be left waiting for a pool that is gone
    throw;
  }
}

ThreadPool::~ThreadPool() { shutdown(); }

void ThreadPool::workerLoop(std::size_t participant) {
  currentPool = this;
  currentParticipant = participant;
  for (;;) {
    std::function<void()> task;
    {
      std::unique_lock lock(mutex_);
      notEmpty_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
      if (queue_.empty()) {
        return;  // shutting down, and everything accepted has been run
      }
      task = std::move(queue_.front());
      queue_.pop_front();
      ++started_;
    }
    notFull_.notify_one();
    task();
  }
}

void ThreadPool::submit(std::function<void()> task) {
  if (threads_.empty()) {
    throw std::runtime_error("the thread pool has no threads");
  }
  {
    std::unique_lock lock(mutex_);
    notFull_.wait(lock, [this] { return stopping_ || queue_.size() < capacity_; });
    if (stopping_) {
      throw std::runtime_error("the thread pool is shut down");
    }
    queue_.push_back(std::move(task));
  }
  notEmpty_.notify_one();
}

void ThreadPool::shutdown() {
  if (currentPool == this) {
    throw std::logic_error("a task cannot shut down the pool it runs on");
  }
  {
    const std::scoped_lock lock(mutex_);
    stopping_ = true;
  }
  notEmpty_.notify_all();
  notFull_.notify_all();
  const std::scoped_lock join(joinMutex_);
  for (std::thread& thread : threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

std::size_t ThreadPool::tasksStarted() const {
  const std::scoped_lock lock(mutex_);
  return started_;
}

void ThreadPool::parallelFor(
    std::size_t count,
    const std::function<void(std::size_t participant, std::size_t index)>& body) {
  if (currentPool == this) {
    throw std::logic_error("parallelFor cannot be called from a task of the pool it would use");
  }
  if (count == 0) {
    return;
  }

  // Shared with the helper tasks, which can outlive this call by a moment (one that was queued but
  // not started finds nothing left to do). They touch `body` only after claiming an index, and no
  // index can be claimed once all of them are done, which this call waits for.
  struct State {
    std::atomic<std::size_t> next{0};
    std::atomic<std::size_t> done{0};
    std::mutex mutex;
    std::condition_variable finished;
    std::exception_ptr error;
    std::size_t errorIndex = std::numeric_limits<std::size_t>::max();
  };
  const auto state = std::make_shared<State>();

  const auto drain = [state, count, bodyPointer = &body](std::size_t participant) {
    for (;;) {
      const std::size_t index = state->next.fetch_add(1, std::memory_order_relaxed);
      if (index >= count) {
        return;
      }
      try {
        (*bodyPointer)(participant, index);
      } catch (...) {
        const std::scoped_lock lock(state->mutex);
        if (index < state->errorIndex) {
          state->errorIndex = index;
          state->error = std::current_exception();
        }
      }
      if (state->done.fetch_add(1, std::memory_order_acq_rel) + 1 == count) {
        {
          const std::scoped_lock lock(state->mutex);
        }  // so that the waiter cannot miss the wake-up
        state->finished.notify_all();
      }
    }
  };

  const std::size_t helpers = std::min(threads_.size(), count - 1);
  for (std::size_t i = 0; i < helpers; ++i) {
    try {
      submit([drain] { drain(currentParticipant); });
    } catch (const std::runtime_error&) {
      break;  // shut down meanwhile: the calling thread does the rest
    }
  }
  drain(0);

  std::unique_lock lock(state->mutex);
  state->finished.wait(lock, [&] { return state->done.load(std::memory_order_acquire) == count; });
  if (state->error) {
    std::rethrow_exception(state->error);
  }
}

}  // namespace warehousesim::core
