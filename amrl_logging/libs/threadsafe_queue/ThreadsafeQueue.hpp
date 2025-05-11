/*
 * File:    ThreadsafeQueue.hpp
 * Author:  pdsherman-uva
 * Date:    Jan. 2023
 */

#pragma once

#include <memory>
#include <mutex>
#include <atomic>

namespace amrl {

///
/// Simple implementation for a thread-safe queue
/// that allows for concurrent push and pop operations
/// from different threads.
///
template<typename T>
class ThreadsafeQueue
{
private:
  /// Internal object to make up nodes in the queue
  /// Contains value and pointer to next item in queue
  struct Node
  {
    std::shared_ptr<T> data;
    std::unique_ptr<Node> next;
  };

public:
  /// Constructor
  ThreadsafeQueue(void);

  /// Object shouldn't be copied.
  ThreadsafeQueue(const ThreadsafeQueue&) = delete;

  /// Default Destrutor
  ~ThreadsafeQueue(void) = default;

  /// Pop front item off queue and return it.
  /// @note Will return nullptr if queue is empty
  /// @return Pointer to data at front of queue (or nullptr if empty)
  std::shared_ptr<T> pop(void);

  /// Push new item onto end of queue
  void push(T value);

  /// Check if the queue is empty
  /// @return True if no items currently in queue
  bool empty(void) const;

  /// @return Number of items currently stored in the queue.
  size_t size(void) const;

private:
  /// Get the raw pointer to tail.
  /// Used to compare against head for empty queue check
  Node *get_tail(void) const;

  /// Smart pointer to front of queue
  std::unique_ptr<Node> _head;

  /// Raw pointer to back of queue
  Node *_tail;

  /// For locking access to head node
  mutable std::mutex _head_mtx;

  /// For locking access to tail node
  mutable std::mutex _tail_mtx;

  /// Current size of the queue
  std::atomic<size_t> _curr_size;
};

template<typename T>
ThreadsafeQueue<T>::ThreadsafeQueue(void)
 : _head(new Node()), _tail(_head.get()), _curr_size(0) 
{}

template<typename T>
std::shared_ptr<T> ThreadsafeQueue<T>::pop(void)
{
  std::shared_ptr<T> data_ptr;

  {
    std::lock_guard<std::mutex> lck(_head_mtx);
    if(_head.get() == get_tail()) { return nullptr; }

    data_ptr = _head->data;
    _head    = std::move(_head->next);
  }
  _curr_size--;
  return data_ptr;
}

template<typename T>
void ThreadsafeQueue<T>::push(T value)
{
  std::unique_ptr<Node> new_tail(new Node());

  {
    std::lock_guard<std::mutex> lck(_tail_mtx);
    _tail->data = std::make_shared<T>(std::move(value));
    _tail->next = std::move(new_tail);
    _tail       = _tail->next.get();
  }
  _curr_size++;
}

template<typename T>
bool ThreadsafeQueue<T>::empty(void) const
{
  std::lock_guard<std::mutex> lck(_head_mtx);
  return _head.get() == get_tail();
}

template<typename T>
typename ThreadsafeQueue<T>::Node* ThreadsafeQueue<T>::get_tail(void) const
{
  std::lock_guard<std::mutex> lck(_tail_mtx);
  return _tail;
}

template<typename T>
size_t ThreadsafeQueue<T>::size(void) const
{
  return _curr_size.load();
}

} // namespace amrl
