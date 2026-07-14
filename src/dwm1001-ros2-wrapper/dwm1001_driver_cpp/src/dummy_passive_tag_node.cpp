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

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class DummyPassiveTagNode : public rclcpp::Node
{
public:
  DummyPassiveTagNode()
  : Node("dummy_passive_tag")
  {
    declareParameters();

    tag_label_ = get_parameter("tag_label").as_string();
    if (tag_label_.empty()) {
      shutdownFatal("No DWM1001 tag label specified.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Started position reporting.");

    publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(tag_label_, 1);
    timer_ = create_wall_timer(100ms, std::bind(&DummyPassiveTagNode::timerCallback, this));
  }

private:
  void shutdownFatal(const std::string & message)
  {
    RCLCPP_FATAL(get_logger(), "%s Shutting down.", message.c_str());
    std::exit(1);
  }

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor tag_label_descriptor;
    tag_label_descriptor.description = "DWM1001 tag label";
    tag_label_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    tag_label_descriptor.read_only = true;

    declare_parameter<std::string>("tag_label", "", tag_label_descriptor);
  }

  void timerCallback()
  {
    auto time_stamp = get_clock()->now();

    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = time_stamp;
    msg.header.frame_id = "dwm1001";
    msg.point.x = 1.2;
    msg.point.y = 2.3;
    msg.point.z = 3.4;

    publisher_->publish(msg);
  }

  std::string tag_label_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPassiveTagNode>());
  rclcpp::shutdown();
  return 0;
}
