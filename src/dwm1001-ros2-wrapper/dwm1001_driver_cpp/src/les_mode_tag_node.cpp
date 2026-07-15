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

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <unordered_map>

#include "dwm1001_driver_cpp/bounded_queue.hpp"
#include "dwm1001_driver_cpp/dwm1001_shell.hpp"
#include "dwm1001_driver_cpp/serial_port.hpp"
#include "dwm1001_msg/msg/float64_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using dwm1001_driver_cpp::Dwm1001Shell;
using dwm1001_driver_cpp::SerialError;
using dwm1001_driver_cpp::SerialPort;

// Matches anchor blocks such as "1783[1.00,2.00,1.17]=2.87": anchor id,
// its known position, and the measured range. Groups: 1=id, 2=x, 3=y,
// 4=z, 5=distance. std::regex (ECMAScript grammar) has no named capture
// groups, unlike Python's re.
const std::regex kAnchorPattern(
  R"(([0-9A-Fa-f]{4})\s*\[\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*)"
  R"((-?\d+(?:\.\d+)?)\s*\]\s*=\s*(-?\d+(?:\.\d+)?))");

// Matches the tag position estimate, e.g. "est[1.23,4.56,0.10,80]".
// Groups: 1=x, 2=y, 3=z, 4=quality.
const std::regex kEstPattern(
  R"(est\[\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*)"
  R"((-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\])");

std::string lstrip(const std::string & s)
{
  std::size_t i = 0;
  while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) {
    ++i;
  }
  return s.substr(i);
}

std::string rstrip(const std::string & s)
{
  std::size_t i = s.size();
  while (i > 0 && std::isspace(static_cast<unsigned char>(s[i - 1]))) {
    --i;
  }
  return s.substr(0, i);
}

std::string strip(const std::string & s)
{
  return rstrip(lstrip(s));
}

std::string lstripChar(const std::string & s, char c)
{
  std::size_t i = 0;
  while (i < s.size() && s[i] == c) {
    ++i;
  }
  return s.substr(i);
}

std::string toUpper(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return std::toupper(c);});
  return s;
}

double nowSeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

class DWM1001LesPublisherNode : public rclcpp::Node
{
public:
  DWM1001LesPublisherNode()
  : Node("dwm1001_les_mode", rclcpp::NodeOptions().allow_undeclared_parameters(true))
  {
    declareParameters();

    serial_port_ = get_parameter("serial_port").as_string();
    baudrate_ = static_cast<int>(get_parameter("baudrate").as_int());
    frame_id_ = get_parameter("frame_id").as_string();
    send_les_on_start_ = get_parameter("send_les_on_start").as_bool();

    reconnect_enabled_ = get_parameter("reconnect_enabled").as_bool();
    reconnect_initial_delay_s_ = get_parameter("reconnect_initial_delay_s").as_double();
    reconnect_max_delay_s_ = get_parameter("reconnect_max_delay_s").as_double();
    reconnect_backoff_ = get_parameter("reconnect_backoff").as_double();
    reconnect_timer_period_s_ = get_parameter("reconnect_timer_period_s").as_double();

    if (serial_port_.empty()) {
      RCLCPP_FATAL(get_logger(), "Parameter 'serial_port' is empty. Aborting.");
      throw std::runtime_error("Parameter 'serial_port' is empty.");
    }

    state_ = ConnState::Disconnected;
    current_reconnect_delay_s_ = reconnect_initial_delay_s_;

    tag_pos_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("~/tag/position", 10);
    tag_quality_pub_ = create_publisher<dwm1001_msg::msg::Float64Stamped>("~/tag/quality", 10);

    if (!connectAndInit()) {
      RCLCPP_WARN(
        get_logger(),
        "Initial serial connection failed. Auto-reconnect is %s.",
        reconnect_enabled_ ? "ENABLED" : "DISABLED");
    }

    running_ = true;
    reader_thread_ = std::thread(&DWM1001LesPublisherNode::serialReaderLoop, this);

    process_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&DWM1001LesPublisherNode::processLines, this));
    reconnect_timer_ = create_wall_timer(
      std::chrono::duration<double>(reconnect_timer_period_s_),
      std::bind(&DWM1001LesPublisherNode::reconnectWatchdog, this));
  }

  ~DWM1001LesPublisherNode() override
  {
    running_ = false;
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }

    std::shared_ptr<SerialPort> old_serial;
    {
      std::lock_guard<std::mutex> lock(ser_mutex_);
      old_serial = std::move(serial_);
      serial_.reset();
      state_ = ConnState::Disconnected;
    }
    old_serial.reset();
  }

private:
  enum class ConnState { Connected, Disconnected, Connecting };

  struct AnchorPublishers
  {
    rclcpp::Publisher<dwm1001_msg::msg::Float64Stamped>::SharedPtr range_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pos_pub;
  };

  // ----------------- Parameters -----------------
  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor serial_port_desc;
    serial_port_desc.description =
      "Device/COM port for DWM1001 (e.g., /dev/ttyACM0 or /dev/serial/by-id/...)";
    serial_port_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    serial_port_desc.read_only = true;

    rcl_interfaces::msg::ParameterDescriptor baudrate_desc;
    baudrate_desc.description = "Baudrate for serial port.";
    baudrate_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    baudrate_desc.read_only = true;

    rcl_interfaces::msg::ParameterDescriptor frame_id_desc;
    frame_id_desc.description = "Frame id used for published PointStamped messages.";
    frame_id_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    frame_id_desc.read_only = true;

    rcl_interfaces::msg::ParameterDescriptor send_les_desc;
    send_les_desc.description =
      "Send 'les' command on startup to enable ranging stream (after reset+shell).";
    send_les_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    send_les_desc.read_only = true;

    rcl_interfaces::msg::ParameterDescriptor reconnect_enabled_desc;
    reconnect_enabled_desc.description =
      "Enable auto-reconnect on serial failures/disconnections.";
    reconnect_enabled_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    reconnect_enabled_desc.read_only = true;

    rcl_interfaces::msg::ParameterDescriptor reconnect_initial_delay_desc;
    reconnect_initial_delay_desc.description = "Initial reconnect delay (seconds).";
    reconnect_initial_delay_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    reconnect_initial_delay_desc.read_only = true;

    rcl_interfaces::msg::ParameterDescriptor reconnect_max_delay_desc;
    reconnect_max_delay_desc.description = "Maximum reconnect delay (seconds).";
    reconnect_max_delay_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    reconnect_max_delay_desc.read_only = true;

    rcl_interfaces::msg::ParameterDescriptor reconnect_backoff_desc;
    reconnect_backoff_desc.description =
      "Backoff multiplier applied after each failed reconnect attempt.";
    reconnect_backoff_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    reconnect_backoff_desc.read_only = true;

    rcl_interfaces::msg::ParameterDescriptor reconnect_timer_period_desc;
    reconnect_timer_period_desc.description =
      "Watchdog timer period for reconnect attempts (seconds).";
    reconnect_timer_period_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    reconnect_timer_period_desc.read_only = true;

    declare_parameter<std::string>(
      "serial_port", "/dev/serial/by-id/usb-SEGGER_J-Link_000760176108-if00", serial_port_desc);
    declare_parameter<int>("baudrate", 115200, baudrate_desc);
    declare_parameter<std::string>("frame_id", "dwm1001", frame_id_desc);
    declare_parameter<bool>("send_les_on_start", true, send_les_desc);

    declare_parameter<bool>("reconnect_enabled", true, reconnect_enabled_desc);
    declare_parameter<double>("reconnect_initial_delay_s", 0.5, reconnect_initial_delay_desc);
    declare_parameter<double>("reconnect_max_delay_s", 10.0, reconnect_max_delay_desc);
    declare_parameter<double>("reconnect_backoff", 1.6, reconnect_backoff_desc);
    declare_parameter<double>(
      "reconnect_timer_period_s", 0.2, reconnect_timer_period_desc);
  }

  // ----------------- Serial connection / init -----------------
  bool connectAndInit()
  {
    {
      std::lock_guard<std::mutex> lock(ser_mutex_);
      if (state_ == ConnState::Connecting) {
        return false;
      }
      state_ = ConnState::Connecting;
    }

    auto new_serial = std::make_shared<SerialPort>();
    try {
      new_serial->open(serial_port_, baudrate_, 2);  // 0.2 s timeout -> 2 deciseconds

      try {
        new_serial->setDtr(true);
        new_serial->setRts(false);
      } catch (const SerialError &) {
        // Best effort, mirrors the Python driver's bare except/pass here.
      }

      std::shared_ptr<SerialPort> old_serial;
      {
        std::lock_guard<std::mutex> lock(ser_mutex_);
        old_serial = std::move(serial_);
        serial_ = new_serial;
        state_ = ConnState::Connected;
        current_reconnect_delay_s_ = reconnect_initial_delay_s_;
        next_reconnect_attempt_t_ = 0.0;
      }
      old_serial.reset();  // closes the previous port, outside the lock

      RCLCPP_INFO(
        get_logger(), "Opened serial port '%s' @ %d baud.",
        serial_port_.c_str(), baudrate_);

      resetAndEnterShell();
      if (send_les_on_start_) {
        startLesStream();
      } else {
        RCLCPP_WARN(
          get_logger(),
          "send_les_on_start=False: not sending 'les'. "
          "Ensure LES mode is active on the DWM1001 tag.");
      }

      line_queue_.clear();

      RCLCPP_INFO(get_logger(), "Serial connected and initialized. Node is operational.");
      return true;
    } catch (const SerialError & e) {
      {
        std::lock_guard<std::mutex> lock(ser_mutex_);
        state_ = ConnState::Disconnected;
      }
      logDisconnectOnce(
        "Cannot open serial port '" + serial_port_ + "': " + e.what());
      return false;
    }
  }

  void markDisconnected(const std::string & reason)
  {
    std::shared_ptr<SerialPort> old_serial;
    {
      std::lock_guard<std::mutex> lock(ser_mutex_);
      if (state_ == ConnState::Disconnected) {
        return;
      }
      state_ = ConnState::Disconnected;
      old_serial = std::move(serial_);
      serial_.reset();

      double now = nowSeconds();
      if (next_reconnect_attempt_t_ <= 0.0) {
        next_reconnect_attempt_t_ = now + current_reconnect_delay_s_;
      }
    }
    old_serial.reset();  // closes the port, outside the lock

    logDisconnectOnce(reason);
  }

  void logDisconnectOnce(const std::string & msg)
  {
    double now = nowSeconds();
    if (now - last_disconnect_log_t_ > 2.0) {
      RCLCPP_WARN(get_logger(), "[SERIAL DISCONNECTED] %s", msg.c_str());
      last_disconnect_log_t_ = now;
    }
  }

  // ----------------- Shell control -----------------
  void sendShellCommand(const std::string & cmd, double delay_seconds)
  {
    std::shared_ptr<SerialPort> serial;
    ConnState state;
    {
      std::lock_guard<std::mutex> lock(ser_mutex_);
      serial = serial_;
      state = state_;
    }
    if (state != ConnState::Connected || !serial) {
      return;
    }
    try {
      Dwm1001Shell shell(*serial);
      shell.sendCommand(cmd, delay_seconds);
    } catch (const SerialError & e) {
      markDisconnected("Failed to send shell command '" + cmd + "': " + e.what());
    }
  }

  void resetAndEnterShell()
  {
    RCLCPP_INFO(get_logger(), "Resetting DWM1001...");
    sendShellCommand("reset", 0.2);
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0));

    RCLCPP_INFO(get_logger(), "Entering shell mode...");

    std::shared_ptr<SerialPort> serial;
    ConnState state;
    {
      std::lock_guard<std::mutex> lock(ser_mutex_);
      serial = serial_;
      state = state_;
    }
    if (state != ConnState::Connected || !serial) {
      return;
    }
    try {
      Dwm1001Shell shell(*serial);
      shell.enterShellMode(1.0);
    } catch (const SerialError & e) {
      markDisconnected(std::string("Failed to enter shell mode: ") + e.what());
    }
  }

  void startLesStream()
  {
    RCLCPP_INFO(get_logger(), "Starting LES stream (les)...");
    sendShellCommand("les", 0.2);

    std::shared_ptr<SerialPort> serial;
    ConnState state;
    {
      std::lock_guard<std::mutex> lock(ser_mutex_);
      serial = serial_;
      state = state_;
    }
    if (state != ConnState::Connected || !serial) {
      return;
    }

    try {
      serial->flushInput();
    } catch (const SerialError &) {
      // best effort
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
    try {
      int waiting = serial->bytesWaiting();
      std::string peek = serial->readRaw(waiting > 0 ? static_cast<std::size_t>(waiting) : 1);
      if (!peek.empty()) {
        RCLCPP_INFO(get_logger(), "LES: received initial %zu bytes.", peek.size());
      }
    } catch (const SerialError & e) {
      markDisconnected(std::string("LES start/peek failed: ") + e.what());
    }
  }

  // ----------------- Reconnect watchdog -----------------
  void reconnectWatchdog()
  {
    if (!reconnect_enabled_) {
      return;
    }

    ConnState state;
    double next_t;
    double delay;
    {
      std::lock_guard<std::mutex> lock(ser_mutex_);
      state = state_;
      next_t = next_reconnect_attempt_t_;
      delay = current_reconnect_delay_s_;
    }

    if (state != ConnState::Disconnected) {
      return;
    }

    double now = nowSeconds();
    if (next_t > 0.0 && now < next_t) {
      return;
    }

    if (connectAndInit()) {
      return;
    }

    double new_delay;
    {
      std::lock_guard<std::mutex> lock(ser_mutex_);
      new_delay = std::min(
        reconnect_max_delay_s_,
        std::max(reconnect_initial_delay_s_, delay) * reconnect_backoff_);
      current_reconnect_delay_s_ = new_delay;
      next_reconnect_attempt_t_ = nowSeconds() + new_delay;
    }

    RCLCPP_WARN(get_logger(), "Reconnect failed. Next attempt in %.2f s.", new_delay);
  }

  // ----------------- Serial reading -----------------
  void serialReaderLoop()
  {
    RCLCPP_INFO(get_logger(), "Starting serial reader thread (readLine).");
    while (running_.load()) {
      std::shared_ptr<SerialPort> serial;
      ConnState state;
      {
        std::lock_guard<std::mutex> lock(ser_mutex_);
        serial = serial_;
        state = state_;
      }

      if (state != ConnState::Connected || !serial) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }

      try {
        auto raw = serial->readLine();
        if (!raw || raw->empty()) {
          continue;
        }
        std::string decoded = strip(*raw);
        if (decoded.empty()) {
          continue;
        }
        line_queue_.push(decoded);
      } catch (const SerialError & e) {
        markDisconnected(std::string("Serial read error: ") + e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  }

  // ----------------- Parsing + Publish -----------------
  void processLines()
  {
    {
      std::lock_guard<std::mutex> lock(ser_mutex_);
      if (state_ != ConnState::Connected) {
        return;
      }
    }

    std::size_t processed = 0;
    while (true) {
      auto maybe_line = line_queue_.tryPop();
      if (!maybe_line) {
        break;
      }
      ++processed;

      std::string line = lstrip(*maybe_line);
      if (!line.empty() && line.front() == '#') {
        line = strip(lstripChar(line, '#'));
      }

      rclcpp::Time now_stamp = this->get_clock()->now();

      for (auto it = std::sregex_iterator(line.begin(), line.end(), kAnchorPattern);
        it != std::sregex_iterator(); ++it)
      {
        const std::smatch & m = *it;
        std::string anchor_id = toUpper(m[1].str());
        double x = std::stod(m[2].str());
        double y = std::stod(m[3].str());
        double z = std::stod(m[4].str());
        double dist = std::stod(m[5].str());

        AnchorPublishers & pubs = getOrCreateAnchorPubs(anchor_id);

        dwm1001_msg::msg::Float64Stamped r_msg;
        r_msg.header.stamp = now_stamp;
        r_msg.header.frame_id = frame_id_;
        r_msg.value = dist;
        pubs.range_pub->publish(r_msg);

        geometry_msgs::msg::PointStamped p_msg;
        p_msg.header.stamp = now_stamp;
        p_msg.header.frame_id = frame_id_;
        p_msg.point.x = x;
        p_msg.point.y = y;
        p_msg.point.z = z;
        pubs.pos_pub->publish(p_msg);
      }

      std::smatch est_match;
      if (std::regex_search(line, est_match, kEstPattern)) {
        double tx = std::stod(est_match[1].str());
        double ty = std::stod(est_match[2].str());
        double tz = std::stod(est_match[3].str());
        double q = std::stod(est_match[4].str());

        geometry_msgs::msg::PointStamped t_msg;
        t_msg.header.stamp = now_stamp;
        t_msg.header.frame_id = frame_id_;
        t_msg.point.x = tx;
        t_msg.point.y = ty;
        t_msg.point.z = tz;
        tag_pos_pub_->publish(t_msg);

        dwm1001_msg::msg::Float64Stamped q_msg;
        q_msg.header.stamp = now_stamp;
        q_msg.header.frame_id = frame_id_;
        q_msg.value = q;
        tag_quality_pub_->publish(q_msg);
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
    pubs.range_pub = create_publisher<dwm1001_msg::msg::Float64Stamped>(range_topic, 10);
    pubs.pos_pub = create_publisher<geometry_msgs::msg::PointStamped>(pos_topic, 10);

    RCLCPP_INFO(
      get_logger(), "Created publishers for anchor %s: %s, %s",
      anchor_id.c_str(), range_topic.c_str(), pos_topic.c_str());

    auto result = anchor_pubs_.emplace(anchor_id, std::move(pubs));
    return result.first->second;
  }

  // ----------------- Parameters (cached) -----------------
  std::string serial_port_;
  int baudrate_ = 0;
  std::string frame_id_;
  bool send_les_on_start_ = false;

  bool reconnect_enabled_ = false;
  double reconnect_initial_delay_s_ = 0.0;
  double reconnect_max_delay_s_ = 0.0;
  double reconnect_backoff_ = 0.0;
  double reconnect_timer_period_s_ = 0.0;

  // ----------------- Connection state (guarded by ser_mutex_) -----------------
  std::mutex ser_mutex_;
  std::shared_ptr<SerialPort> serial_;
  ConnState state_ = ConnState::Disconnected;
  double last_disconnect_log_t_ = 0.0;
  double next_reconnect_attempt_t_ = 0.0;
  double current_reconnect_delay_s_ = 0.0;

  // ----------------- Reader thread / queue -----------------
  dwm1001_driver_cpp::BoundedQueue<std::string> line_queue_{500};
  std::atomic<bool> running_{false};
  std::thread reader_thread_;

  // ----------------- Publishers -----------------
  std::unordered_map<std::string, AnchorPublishers> anchor_pubs_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr tag_pos_pub_;
  rclcpp::Publisher<dwm1001_msg::msg::Float64Stamped>::SharedPtr tag_quality_pub_;

  // ----------------- Timers -----------------
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<DWM1001LesPublisherNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("dwm1001_les_mode"), "Fatal startup error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
