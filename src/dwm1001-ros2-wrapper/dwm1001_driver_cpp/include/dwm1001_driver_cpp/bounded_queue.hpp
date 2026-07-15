// Copyright 2023 The Human and Intelligent Vehicle Ensembles (HIVE) Lab
// Copyright 2026 Gianmarco Guerriero (mivia_rover C++ port)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DWM1001_DRIVER_CPP__BOUNDED_QUEUE_HPP_
#define DWM1001_DRIVER_CPP__BOUNDED_QUEUE_HPP_

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace dwm1001_driver_cpp
{

// Thread-safe queue with a fixed maximum size. Mirrors Python's
// queue.Queue(maxsize=N): when full, pushing a new item drops the oldest
// one to make room, instead of blocking or raising.
template<typename T>
class BoundedQueue
{
public:
  explicit BoundedQueue(std::size_t max_size)
  : max_size_(max_size)
  {}

  void push(T value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (items_.size() >= max_size_) {
      items_.pop_front();
    }
    items_.push_back(std::move(value));
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    items_.clear();
  }

  std::optional<T> tryPop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (items_.empty()) {
      return std::nullopt;
    }
    T value = std::move(items_.front());
    items_.pop_front();
    return value;
  }

private:
  std::size_t max_size_;
  std::mutex mutex_;
  std::deque<T> items_;
};

}  // namespace dwm1001_driver_cpp

#endif  // DWM1001_DRIVER_CPP__BOUNDED_QUEUE_HPP_
