// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef SINGLE_THREADED_TASK_QUEUE_H
#define SINGLE_THREADED_TASK_QUEUE_H

#include <future>
#include <queue>
#include <chrono>
#include <functional>

template <typename T>
class SingleThreadedTaskQueue
{

  // Will perform tasks on a single thread in first in first out order.

public:
  SingleThreadedTaskQueue(std::function<void(T)> task_func) : m_task(task_func)
  {
    // do nothing
  };

  SingleThreadedTaskQueue()                                          = delete;
  SingleThreadedTaskQueue(const SingleThreadedTaskQueue&)            = delete;
  SingleThreadedTaskQueue& operator=(const SingleThreadedTaskQueue&) = delete;

  ~SingleThreadedTaskQueue() = default;

  void addTask(T data);

  void manage();

private:
  std::function<void(T)> m_task;
  // std::thread m_thread;
  std::future<void> m_future;
  std::queue<T> m_queue;
};

template <typename T>
void SingleThreadedTaskQueue<T>::addTask(T data)
{
  m_queue.emplace(data);
};

template <typename T>
void SingleThreadedTaskQueue<T>::manage()
{
  using namespace std::chrono_literals;
  if (!m_future.valid() || m_future.wait_for(0ms) == std::future_status::ready)
  {
    if (m_queue.empty()) {
      // no more tasks, go into a holding pattern
      return;
    }
    m_future = std::async(std::launch::async, m_task, m_queue.front());
    m_queue.pop();
  }
};

#endif // SINGLE_THREADED_TASK_QUEUE_H

