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
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "dwm1001_driver_cpp/bounded_queue.hpp"
#include "dwm1001_driver_cpp/dwm1001_shell.hpp"
#include "dwm1001_driver_cpp/node_utils.hpp"
#include "dwm1001_driver_cpp/pos_report_parser.hpp"
#include "dwm1001_driver_cpp/serial_port.hpp"
#include "dwm1001_msg/msg/float64_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using dwm1001_driver_cpp::Dwm1001Shell;
using dwm1001_driver_cpp::makeDescriptor;
using dwm1001_driver_cpp::ParsingError;
using dwm1001_driver_cpp::parseActivePositionReport;
using dwm1001_driver_cpp::PositionReport;
using dwm1001_driver_cpp::RateLimiter;
using dwm1001_driver_cpp::round3;
using dwm1001_driver_cpp::SerialError;
using dwm1001_driver_cpp::SerialPort;

}  // namespace

// C++ port of active_tag_node.py, talking to the DWM1001 shell protocol
// used by pydwm1001 (reset + enter shell + 'lep' -> "POS,x,y,z,quality"
// lines), rather than the 'les' text stream used by les_mode_tag/
// ranging_tag. Unlike the Python original -- which reads synchronously
// inside the ROS timer callback with no timeout, risking blocking the
// whole node forever -- this uses the same reader-thread + bounded-queue
// pattern as the other phases, with the timer popping at most one report
// per tick to preserve the original "one report per callback" semantics.
class ActiveTagNode : public rclcpp::Node
{
public:
  ActiveTagNode()
  : Node("dwm_active", rclcpp::NodeOptions().allow_undeclared_parameters(true))
  {
    declareParameters();

    int samples_param = static_cast<int>(get_parameter("samples").as_int());
    if (samples_param > 10) {
      RCLCPP_WARN(get_logger(), "Number of samples exceeds maximum value. Capping at 10.");
      samples_param = 10;
    } else if (samples_param < 1) {
      // active_tag_node.py has no lower-bound check at all: with
      // deque(maxlen=0), len(buffer)==maxlen is trivially true on every
      // callback and Python crashes with ZeroDivisionError computing the
      // average. Clamping here (matching passive_tag's symmetric clamp)
      // avoids the equivalent NaN-publishing failure mode in C++ instead
      // of reproducing a pre-existing Python bug nobody would want.
      RCLCPP_WARN(get_logger(), "Number of samples must be at least 1. Setting to minimum.");
      samples_param = 1;
    }
    RCLCPP_INFO(get_logger(), "Performing moving average with %d samples.", samples_param);
    max_samples_ = samples_param;

    serial_port_ = get_parameter("serial_port").as_string();
    RCLCPP_INFO(get_logger(), "Provided serial port: '%s'", serial_port_.c_str());

    tag_id_ = get_parameter("tag_id").as_string();

    openSerialPort();

    Dwm1001Shell shell(*serial_);
    // Mirror of pydwm1001.UartDwm1001.reset(): send_shell_command default
    // delay (0.5s) + an extra RESET_DELAY_PERIOD (0.1s).
    shell.sendCommand("reset", 0.5);
    std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
    // Mirror of enter_shell_mode(): SHELL_STARTUP_DELAY_PERIOD (1.0s).
    shell.enterShellMode(1.0);
    // Mirror of start_position_reporting(): send 'lep', then flush.
    shell.sendCommand("lep", 0.5);
    serial_->flushInput();
    RCLCPP_INFO(get_logger(), "Started position reporting.");

    std::string tag_topic = "~/output/" + tag_id_;
    std::string quality_topic = "~/output/" + tag_id_ + "_quality";
    point_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(tag_topic, 1);
    quality_publisher_ = create_publisher<dwm1001_msg::msg::Float64Stamped>(quality_topic, 1);
    RCLCPP_INFO(get_logger(), "Publishing DWM postions on %s", tag_topic.c_str());

    running_ = true;
    reader_thread_ = std::thread(&ActiveTagNode::serialReaderLoop, this);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / 25.0), std::bind(&ActiveTagNode::timerCallback, this));
  }

  ~ActiveTagNode() override
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
      // Hardcoded 115200 baud, 0.2s timeout -- baudrate isn't a parameter
      // in the Python driver for this node (unlike les_mode_tag/
      // ranging_tag).
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

    declare_parameter<int>(
      "samples", 1,
      makeDescriptor(
        "The number of samples used to perform the moving average. Max = 10",
        ParameterType::PARAMETER_INTEGER));
    declare_parameter<std::string>(
      "tag_id", "",
      makeDescriptor(
        "The ID for the particular DWM1001 device.", ParameterType::PARAMETER_STRING));
    declare_parameter<std::string>(
      "serial_port", "",
      makeDescriptor(
        "Device file or COM port associated with DWM1001", ParameterType::PARAMETER_STRING));
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
        // Not in the Python original (which has no timeout/thread at
        // all here) -- avoids a busy-loop on a persistently disconnected
        // device, same reasoning as ranging_tag_node.cpp.
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
      // No new report yet this tick. The Python original blocks until
      // one arrives; we just skip -- same eventual output, no stall.
      return;
    }

    PositionReport report;
    try {
      report = parseActivePositionReport(*maybe_line);
    } catch (const ParsingError &) {
      RCLCPP_WARN(get_logger(), "Could not parse position report. Skipping it.");
      return;
    }

    position_buffer_.push_back({report.x, report.y, report.z});
    while (static_cast<int>(position_buffer_.size()) > max_samples_) {
      position_buffer_.pop_front();
    }

    if (static_cast<int>(position_buffer_.size()) == max_samples_) {
      double x_sum = 0.0;
      double y_sum = 0.0;
      double z_sum = 0.0;
      for (const auto & point : position_buffer_) {
        x_sum += point[0];
        y_sum += point[1];
        z_sum += point[2];
      }

      rclcpp::Time time_stamp = this->get_clock()->now();

      geometry_msgs::msg::PointStamped msg;
      msg.header.stamp = time_stamp;
      msg.header.frame_id = "dwm1001";
      msg.point.x = round3(x_sum / max_samples_);
      msg.point.y = round3(y_sum / max_samples_);
      msg.point.z = round3(z_sum / max_samples_);

      dwm1001_msg::msg::Float64Stamped quality_msg;
      quality_msg.header.stamp = time_stamp;
      quality_msg.header.frame_id = "dwm1001";
      quality_msg.value = static_cast<double>(report.quality);

      quality_publisher_->publish(quality_msg);
      point_publisher_->publish(msg);
    } else {
      RCLCPP_WARN(get_logger(), "DWM1001 buffer not filled to %d yet.", max_samples_);
    }
  }

  // ----------------- Parameters (cached) -----------------
  std::string serial_port_;
  std::string tag_id_;
  int max_samples_ = 1;

  // ----------------- Serial / reader thread -----------------
  std::shared_ptr<SerialPort> serial_;
  dwm1001_driver_cpp::BoundedQueue<std::string> line_queue_{10};
  std::atomic<bool> running_{false};
  std::thread reader_thread_;
  RateLimiter read_error_rate_limiter_{2.0};

  // ----------------- Moving average buffer -----------------
  std::deque<std::array<double, 3>> position_buffer_;

  // ----------------- Publishers / timer -----------------
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_;
  rclcpp::Publisher<dwm1001_msg::msg::Float64Stamped>::SharedPtr quality_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<ActiveTagNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("dwm_active"), "Fatal startup error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
