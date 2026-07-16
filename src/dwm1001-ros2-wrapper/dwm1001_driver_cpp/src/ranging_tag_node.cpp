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

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "dwm1001_driver_cpp/bounded_queue.hpp"
#include "dwm1001_driver_cpp/les_line_parser.hpp"
#include "dwm1001_driver_cpp/node_utils.hpp"
#include "dwm1001_driver_cpp/serial_port.hpp"
#include "dwm1001_driver_cpp/string_utils.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace
{

using dwm1001_driver_cpp::AnchorReading;
using dwm1001_driver_cpp::makeDescriptor;
using dwm1001_driver_cpp::parseAnchors;
using dwm1001_driver_cpp::RateLimiter;
using dwm1001_driver_cpp::SerialError;
using dwm1001_driver_cpp::SerialPort;
using dwm1001_driver_cpp::strip;

}  // namespace

// C++ port of ranging_tag.py. Deliberately kept as the "raw" variant: no
// connection state machine, no auto-reconnect, no reset/shell sequence --
// just open the port, send 'les' once, and read. See les_mode_tag_node.cpp
// for the hardened variant with reconnection.
class DWMLesPublisherNode : public rclcpp::Node
{
public:
  DWMLesPublisherNode()
  : Node("dwm_les_publisher", rclcpp::NodeOptions().allow_undeclared_parameters(true))
  {
    declareParameters();

    serial_port_ = get_parameter("serial_port").as_string();
    baudrate_ = static_cast<int>(get_parameter("baudrate").as_int());
    frame_id_ = get_parameter("frame_id").as_string();
    send_les_on_start_ = get_parameter("send_les_on_start").as_bool();

    if (serial_port_.empty()) {
      RCLCPP_FATAL(get_logger(), "Parameter 'serial_port' is empty. Aborting.");
      throw std::runtime_error("Parameter 'serial_port' is empty.");
    }

    serial_ = std::make_shared<SerialPort>();
    // 1.0 s read timeout -> 10 deciseconds. Throws SerialError on failure,
    // caught and logged as FATAL by main() (mirrors the Python driver
    // treating open() failure as a hard startup error, unlike
    // les_mode_tag which tolerates it and relies on auto-reconnect).
    serial_->open(serial_port_, baudrate_, 10);

    RCLCPP_INFO(
      get_logger(), "Opened serial port '%s' @ %d baud.", serial_port_.c_str(), baudrate_);

    if (send_les_on_start_) {
      try {
        serial_->write("les\r\n");
        serial_->drain();
        RCLCPP_INFO(
          get_logger(), "Sent 'les' command to enable ranging + estimated position output.");
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to send 'les' command: %s", e.what());
      }
    }

    running_ = true;
    reader_thread_ = std::thread(&DWMLesPublisherNode::serialReaderLoop, this);

    process_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&DWMLesPublisherNode::processLines, this));
  }

  ~DWMLesPublisherNode() override
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
  struct AnchorPublishers
  {
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr range_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pos_pub;
  };

  // ----------------- Parameters -----------------
  void declareParameters()
  {
    using rcl_interfaces::msg::ParameterType;

    declare_parameter<std::string>(
      "serial_port", "/dev/ttyACM0",
      makeDescriptor(
        "Device/COM port for DWM1001 (e.g., /dev/ttyACM0, COM3)",
        ParameterType::PARAMETER_STRING));
    declare_parameter<int>(
      "baudrate", 115200,
      makeDescriptor("Baudrate for serial port.", ParameterType::PARAMETER_INTEGER));
    declare_parameter<std::string>(
      "frame_id", "dwm1001",
      makeDescriptor(
        "Frame id used for published PointStamped messages.",
        ParameterType::PARAMETER_STRING));
    declare_parameter<bool>(
      "send_les_on_start", true,
      makeDescriptor(
        "Send 'les' command on startup to (toggle) enable ranging stream.",
        ParameterType::PARAMETER_BOOL));
  }

  // ----------------- Serial reading -----------------
  void serialReaderLoop()
  {
    RCLCPP_INFO(get_logger(), "Starting serial reader thread.");
    while (running_.load()) {
      try {
        auto raw = serial_->readLine();
        if (!raw || raw->empty()) {
          continue;
        }
        std::string decoded = strip(*raw);
        if (decoded.empty()) {
          continue;
        }
        line_queue_.push(std::move(decoded));
      } catch (const SerialError & e) {
        logReadErrorOnce(std::string("Serial read error: ") + e.what());
        // No reconnection here (unlike les_mode_tag) -- just a short
        // pause so a persistently disconnected device doesn't spin the
        // loop at full CPU, since our poll()-based SerialPort detects a
        // hangup almost instantly instead of waiting out a real timeout.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } catch (const std::exception & e) {
        // Mirror of ranging_tag.py's reader loop catch-all "except
        // Exception": anything unexpected shouldn't be allowed to escape
        // this detached thread and call std::terminate().
        logReadErrorOnce(std::string("Unexpected serial read error: ") + e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  }

  // Avoids log spam while the device stays disconnected: with the 50ms
  // retry pause above, an unbounded warning would print ~20 times/second.
  void logReadErrorOnce(const std::string & msg)
  {
    if (read_error_rate_limiter_.allow()) {
      RCLCPP_WARN(get_logger(), "%s", msg.c_str());
    }
  }

  // ----------------- Parsing + Publish -----------------
  void processLines()
  {
    std::size_t processed = 0;
    while (true) {
      auto maybe_line = line_queue_.tryPop();
      if (!maybe_line) {
        break;
      }
      ++processed;

      auto readings = parseAnchors(*maybe_line);
      if (readings.empty()) {
        continue;
      }

      rclcpp::Time now_stamp = this->get_clock()->now();

      for (const AnchorReading & reading : readings) {
        AnchorPublishers & pubs = getOrCreateAnchorPubs(reading.id);

        std_msgs::msg::Float64 r_msg;
        r_msg.data = reading.distance;
        pubs.range_pub->publish(r_msg);

        geometry_msgs::msg::PointStamped p_msg;
        p_msg.header.stamp = now_stamp;
        p_msg.header.frame_id = frame_id_;
        p_msg.point.x = reading.x;
        p_msg.point.y = reading.y;
        p_msg.point.z = reading.z;
        pubs.pos_pub->publish(p_msg);
      }
    }

    if (processed > 0) {
      RCLCPP_DEBUG(get_logger(), "Processed %zu serial line(s).", processed);
    }
  }

  AnchorPublishers & getOrCreateAnchorPubs(const std::string & anchor_id)
  {
    auto it = anchor_pubs_.find(anchor_id);
    if (it != anchor_pubs_.end()) {
      return it->second;
    }

    std::string range_topic = "~/anchors/anchor_" + anchor_id + "/range";
    std::string pos_topic = "~/anchors/anchor_" + anchor_id + "/position";

    AnchorPublishers pubs;
    pubs.range_pub = create_publisher<std_msgs::msg::Float64>(range_topic, 10);
    pubs.pos_pub = create_publisher<geometry_msgs::msg::PointStamped>(pos_topic, 10);

    RCLCPP_INFO(
      get_logger(), "Created anchor publishers for %s: %s (Float64), %s (PointStamped)",
      anchor_id.c_str(), range_topic.c_str(), pos_topic.c_str());

    auto result = anchor_pubs_.emplace(anchor_id, std::move(pubs));
    return result.first->second;
  }

  // ----------------- Parameters (cached) -----------------
  std::string serial_port_;
  int baudrate_ = 0;
  std::string frame_id_;
  bool send_les_on_start_ = false;

  // ----------------- Serial / reader thread -----------------
  std::shared_ptr<SerialPort> serial_;
  dwm1001_driver_cpp::BoundedQueue<std::string> line_queue_{100};
  std::atomic<bool> running_{false};
  std::thread reader_thread_;
  RateLimiter read_error_rate_limiter_{2.0};

  // ----------------- Publishers -----------------
  std::unordered_map<std::string, AnchorPublishers> anchor_pubs_;

  // ----------------- Timer -----------------
  rclcpp::TimerBase::SharedPtr process_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<DWMLesPublisherNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("dwm_les_publisher"), "Fatal startup error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
