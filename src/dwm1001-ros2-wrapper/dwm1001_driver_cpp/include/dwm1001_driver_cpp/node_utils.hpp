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

#ifndef DWM1001_DRIVER_CPP__NODE_UTILS_HPP_
#define DWM1001_DRIVER_CPP__NODE_UTILS_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace dwm1001_driver_cpp
{

// DWM1001's fixed UART baud rate -- used as the hardcoded rate in
// active_tag_node.cpp/passive_tag_node.cpp (no baudrate parameter there)
// and as the "baudrate" parameter default in les_mode_tag_node.cpp/
// ranging_tag_node.cpp. One shared constant instead of the same literal
// typed out in four files.
constexpr int kDefaultBaudRate = 115200;

// Reader-thread retry pause after a serial read error, shared by all four
// nodes' serialReaderLoop(): avoids a busy-loop on a persistently
// disconnected device (our poll()-based SerialPort detects a hangup
// almost instantly, unlike a real read timeout).
constexpr std::chrono::milliseconds kReaderRetryDelay{50};

// Upper bound on the moving-average sample count ("samples"/"num_samples"
// parameter), shared by active_tag_node.cpp and passive_tag_node.cpp.
constexpr int kMaxSamples = 10;

// Processing-timer period for the les/lep line queue (100 Hz), shared by
// les_mode_tag_node.cpp and ranging_tag_node.cpp.
constexpr std::chrono::milliseconds kProcessTimerPeriod{10};

inline double nowSeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline double round3(double value)
{
  return std::round(value * 1000.0) / 1000.0;
}

// Every ROS parameter across this package's nodes is a read-only
// string/number set once at startup -- one factory instead of repeating
// the same 4-line ParameterDescriptor setup in every node.
inline rcl_interfaces::msg::ParameterDescriptor makeDescriptor(
  const std::string & description, uint8_t type)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.type = type;
  descriptor.read_only = true;
  return descriptor;
}

// Suppresses repeated log calls for the same ongoing condition (e.g. a
// persistent serial read failure): allow() returns true at most once per
// interval_s seconds.
class RateLimiter
{
public:
  explicit RateLimiter(double interval_s)
  : interval_s_(interval_s)
  {}

  bool allow()
  {
    double now = nowSeconds();
    if (now - last_t_ > interval_s_) {
      last_t_ = now;
      return true;
    }
    return false;
  }

private:
  double interval_s_;
  double last_t_ = 0.0;
};

}  // namespace dwm1001_driver_cpp

#endif  // DWM1001_DRIVER_CPP__NODE_UTILS_HPP_
