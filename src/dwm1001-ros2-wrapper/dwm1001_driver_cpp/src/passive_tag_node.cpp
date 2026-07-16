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

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dwm1001_driver_cpp/bounded_queue.hpp"
#include "dwm1001_driver_cpp/dwm1001_shell.hpp"
#include "dwm1001_driver_cpp/node_utils.hpp"
#include "dwm1001_driver_cpp/pos_report_parser.hpp"
#include "dwm1001_driver_cpp/serial_port.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using dwm1001_driver_cpp::Dwm1001Shell;
using dwm1001_driver_cpp::makeDescriptor;
using dwm1001_driver_cpp::ParsingError;
using dwm1001_driver_cpp::parsePassivePositionReport;
using dwm1001_driver_cpp::PassivePositionReport;
using dwm1001_driver_cpp::RateLimiter;
using dwm1001_driver_cpp::round3;
using dwm1001_driver_cpp::SerialError;
using dwm1001_driver_cpp::SerialPort;

std::vector<std::string> splitCommaList(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    out.push_back(item);
  }
  return out;
}

}  // namespace

// C++ port of passive_tag_node.py: listens for broadcasted position
// reports from any active tag (via 'lep'), publishing a PointStamped per
// discovered tag on ~/output/<tag_label>. No quality topic (the Python
// original doesn't publish one for passive tags either).
class PassiveTagNode : public rclcpp::Node
{
public:
  PassiveTagNode()
  : Node("dwm_passive")
  {
    declareParameters();

    serial_port_ = get_parameter("serial_port").as_string();
    openSerialPort();

    Dwm1001Shell shell(*serial_);
    shell.sendCommand("reset", 0.5);
    std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
    shell.enterShellMode(1.0);
    shell.sendCommand("lep", 0.5);
    serial_->flushInput();
    RCLCPP_INFO(get_logger(), "Started position reporting.");

    std::string ignore_tags_param = get_parameter("ignore_tags").as_string();
    if (!ignore_tags_param.empty()) {
      tags_to_ignore_ = splitCommaList(ignore_tags_param);
    }
    if (!tags_to_ignore_.empty()) {
      std::string joined;
      for (std::size_t i = 0; i < tags_to_ignore_.size(); ++i) {
        if (i > 0) {
          joined += ", ";
        }
        joined += "'" + tags_to_ignore_[i] + "'";
      }
      RCLCPP_INFO(get_logger(), "Ignoring tags: %s", joined.c_str());
    }

    int num_samples_param = static_cast<int>(get_parameter("num_samples").as_int());
    if (num_samples_param > 10) {
      RCLCPP_WARN(get_logger(), "Maximum number of samples is 10. Setting to maximum.");
      num_samples_ = 10;
    } else if (num_samples_param < 1) {
      RCLCPP_WARN(get_logger(), "Minimum number of samples is 1. Setting to minimum.");
      num_samples_ = 1;
    } else {
      num_samples_ = num_samples_param;
    }
    RCLCPP_INFO(get_logger(), "Listener set to average %d samples", num_samples_);

    running_ = true;
    reader_thread_ = std::thread(&PassiveTagNode::serialReaderLoop, this);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / 10.0), std::bind(&PassiveTagNode::timerCallback, this));
  }

  ~PassiveTagNode() override
  {
    running_ = false;
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
    if (serial_) {
      serial_->close();
    }
  }

private:
  void openSerialPort()
  {
    if (serial_port_.empty()) {
      RCLCPP_FATAL(get_logger(), "No serial port specified. Shutting down.");
      throw std::runtime_error("No serial port specified.");
    }

    serial_ = std::make_shared<SerialPort>();
    try {
      serial_->open(serial_port_, 115200, 2);
    } catch (const SerialError & e) {
      RCLCPP_FATAL(
        get_logger(), "Could not open serial port '%s': %s. Shutting down.",
        serial_port_.c_str(), e.what());
      throw std::runtime_error("Could not open serial port '" + serial_port_ + "'.");
    }

    RCLCPP_INFO(get_logger(), "Opened serial port: '%s'.", serial_port_.c_str());
  }

  void declareParameters()
  {
    using rcl_interfaces::msg::ParameterType;

    declare_parameter<std::string>(
      "serial_port", "",
      makeDescriptor(
        "Device file or COM port associated with DWM1001", ParameterType::PARAMETER_STRING));
    declare_parameter<std::string>(
      "ignore_tags", "",
      makeDescriptor(
        "DWM1001 tags to ignore. This node will not publish position reports coming from "
        "these tags. Expected format is comma-separated values. "
        "Example: 'DW1234,DW1235,DW1236'.",
        ParameterType::PARAMETER_STRING));
    declare_parameter<int>(
      "num_samples", 3,
      makeDescriptor(
        "The number of samples used to compute moving average.",
        ParameterType::PARAMETER_INTEGER));
  }

  // ----------------- Serial reading -----------------
  void serialReaderLoop()
  {
    while (running_.load()) {
      try {
        auto raw = serial_->readLine();
        if (!raw || raw->empty()) {
          continue;
        }
        line_queue_.push(std::move(*raw));
      } catch (const SerialError & e) {
        logReadErrorOnce(std::string("Serial read error: ") + e.what());
        // Not in the Python original -- avoids a busy-loop on a
        // persistently disconnected device, same reasoning as
        // ranging_tag_node.cpp / active_tag_node.cpp.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } catch (const std::exception & e) {
        // Defensive catch-all: nothing here should throw other than
        // SerialError, but letting anything else escape this detached
        // thread would call std::terminate() and kill the whole node.
        logReadErrorOnce(std::string("Unexpected serial read error: ") + e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  }

  void logReadErrorOnce(const std::string & msg)
  {
    if (read_error_rate_limiter_.allow()) {
      RCLCPP_WARN(get_logger(), "%s", msg.c_str());
    }
  }

  // ----------------- Timer: one report per tick -----------------
  void timerCallback()
  {
    auto maybe_line = line_queue_.tryPop();
    if (!maybe_line) {
      return;
    }

    PassivePositionReport report;
    try {
      report = parsePassivePositionReport(*maybe_line);
    } catch (const ParsingError &) {
      RCLCPP_WARN(get_logger(), "Could not parse position report. Skipping it.");
      return;
    }

    const std::string & tag_label = report.tag_name;

    for (const auto & ignored : tags_to_ignore_) {
      if (ignored == tag_label) {
        return;
      }
    }

    auto & buffer = position_buffers_[tag_label];
    buffer.push_back({report.position.x, report.position.y, report.position.z});
    while (static_cast<int>(buffer.size()) > num_samples_) {
      buffer.pop_front();
    }

    if (publishers_.find(tag_label) == publishers_.end()) {
      RCLCPP_INFO(
        get_logger(), "Discovered new active tag '%s'. Creating publisher.", tag_label.c_str());
      std::string tag_topic = "~/output/" + tag_label;
      publishers_[tag_label] = create_publisher<geometry_msgs::msg::PointStamped>(tag_topic, 1);
      updateTimerPeriod();
    }

    if (static_cast<int>(buffer.size()) == num_samples_) {
      double x_sum = 0.0;
      double y_sum = 0.0;
      double z_sum = 0.0;
      for (const auto & point : buffer) {
        x_sum += point[0];
        y_sum += point[1];
        z_sum += point[2];
      }

      geometry_msgs::msg::PointStamped msg;
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "dwm1001";
      msg.point.x = round3(x_sum / num_samples_);
      msg.point.y = round3(y_sum / num_samples_);
      msg.point.z = round3(z_sum / num_samples_);

      RCLCPP_DEBUG(
        get_logger(), "Average point for %s: [%.3f, %.3f, %.3f]",
        tag_label.c_str(), msg.point.x, msg.point.y, msg.point.z);

      publishers_[tag_label]->publish(msg);
    } else {
      RCLCPP_WARN(get_logger(), "Still buffering position samples...");
    }
  }

  // Recreates the wall timer with a period reflecting the current number
  // of discovered tags (mirrors rclpy's `self.timer.timer_period_ns = ...`
  // -- rclcpp has no in-place period setter, so a new timer replaces the
  // old one; safe to do from within the timer's own callback since the
  // executor holds its own reference to the in-flight invocation).
  void updateTimerPeriod()
  {
    std::size_t tag_count = publishers_.size();
    if (tag_count == 0) {
      return;
    }
    double period_s = 1.0 / (20.0 * static_cast<double>(tag_count));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_s), std::bind(&PassiveTagNode::timerCallback, this));
  }

  // ----------------- Parameters (cached) -----------------
  std::string serial_port_;
  std::vector<std::string> tags_to_ignore_;
  int num_samples_ = 3;

  // ----------------- Serial / reader thread -----------------
  std::shared_ptr<SerialPort> serial_;
  dwm1001_driver_cpp::BoundedQueue<std::string> line_queue_{10};
  std::atomic<bool> running_{false};
  std::thread reader_thread_;
  RateLimiter read_error_rate_limiter_{2.0};

  // ----------------- Per-tag state -----------------
  std::unordered_map<std::string, std::deque<std::array<double, 3>>> position_buffers_;
  std::unordered_map<std::string,
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> publishers_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<PassiveTagNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("dwm_passive"), "Fatal startup error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
