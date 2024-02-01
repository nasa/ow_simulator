// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef SINGLE_THREADED_TASK_QUEUE_H
#define SINGLE_THREADED_TASK_QUEUE_H

#include <future>
#include <queue>
#include <chrono>
#include <functional>
#include <mutex>
#include <memory>
#include <algorithm>

template <typename T>
class SingleThreadedTaskQueue
{

  // Will perform tasks on a single thread in first in first out order.

public:
  SingleThreadedTaskQueue(std::function<void(T*)> task_func) : m_task(task_func)
  {
    // do nothing
  };

  SingleThreadedTaskQueue()                                          = delete;
  SingleThreadedTaskQueue(const SingleThreadedTaskQueue&)            = delete;
  SingleThreadedTaskQueue& operator=(const SingleThreadedTaskQueue&) = delete;

  ~SingleThreadedTaskQueue() = default;

  void addTask(T data);

private:
  void processTasks();

  std::unique_ptr<T> getNext();

  void spin();

  bool m_tasks_waiting;
  std::function<void(T*)> m_task;
  std::future<void> m_future;
  std::mutex m_queue_lock;
  std::queue<std::unique_ptr<T>> m_queue;
};

template <typename T>
void SingleThreadedTaskQueue<T>::addTask(T data)
{
  const std::lock_guard<std::mutex> lock(m_queue_lock);
  m_queue.push(std::make_unique<T>(data));
  spin();
};

template <typename T>
void SingleThreadedTaskQueue<T>::spin()
{
  using namespace std::chrono_literals;
  if (!m_future.valid()
      || m_future.wait_for(0ms) == std::future_status::ready) {
    m_future = std::async(std::launch::async,
                          &SingleThreadedTaskQueue<T>::processTasks,
                          this);
  }
};

template <typename T>
void SingleThreadedTaskQueue<T>::processTasks()
{
  while(!m_queue.empty()) {
    // perform task
    m_task(getNext().get());
  }
  // This function terminates putting the task queue into an idle state until
  // addTask is called.
}

template <typename T>
std::unique_ptr<T> SingleThreadedTaskQueue<T>::getNext()
{
  std::unique_ptr<T> data;
  // use a lock in case addTask is called asynchronously with this function
  const std::lock_guard<std::mutex> lock(m_queue_lock);
  // transfer ownership of the pointed to object
  std::swap(m_queue.front(), data);
  m_queue.pop();
  // return the pointer removed from the front of the queue
  return data;
}

#endif // SINGLE_THREADED_TASK_QUEUE_H

