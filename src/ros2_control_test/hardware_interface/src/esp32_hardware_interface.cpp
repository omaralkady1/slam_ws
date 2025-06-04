// hardware_interface/src/esp32_hardware_interface.cpp

#include "../include/ros2_control_test/esp32_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_test
{

hardware_interface::CallbackReturn ESP32HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from hardware config
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);
  cmd_vel_topic_ = info_.hardware_parameters.count("cmd_vel_topic") ? 
    info_.hardware_parameters["cmd_vel_topic"] : "/cmd_vel";
  joint_states_topic_ = info_.hardware_parameters.count("joint_states_topic") ? 
    info_.hardware_parameters["joint_states_topic"] : "/joint_states";
  diagnostic_topic_ = info_.hardware_parameters.count("diagnostic_topic") ? 
    info_.hardware_parameters["diagnostic_topic"] : "/esp32/diagnostics";
  timeout_ms_ = info_.hardware_parameters.count("timeout_ms") ? 
    std::stoi(info_.hardware_parameters["timeout_ms"]) : 500;

  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
              "Hardware parameters - wheel_radius: %f, wheel_separation: %f", 
              wheel_radius_, wheel_separation_);
  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
              "Topics - cmd_vel: %s, joint_states: %s, diagnostics: %s",
              cmd_vel_topic_.c_str(), joint_states_topic_.c_str(), diagnostic_topic_.c_str());

  // Initialize state
  joint_state_received_ = false;
  read_calls_ = 0;
  write_calls_ = 0;
  missed_reads_ = 0;
  successful_writes_ = 0;

  // Check if we should use mock hardware
  use_mock_hardware_ = info_.hardware_parameters.count("use_mock_hardware") ? 
    (info_.hardware_parameters.at("use_mock_hardware") == "true") : false;

  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
              "Initializing ESP32 hardware interface in %s mode", 
              use_mock_hardware_ ? "MOCK" : "REAL");

  // Setup joint variables
  joint_names_.resize(info_.joints.size());
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  mock_velocities_.resize(info_.joints.size(), 0.0);

  // Store joint names
  for (size_t i = 0; i < info_.joints.size(); i++) {
    joint_names_[i] = info_.joints[i].name;
    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
                "Registered joint %ld: %s", i, joint_names_[i].c_str());
    
    // Check command interfaces
    if (info_.joints[i].command_interfaces.size() != 1 ||
        info_.joints[i].command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ESP32HardwareInterface"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.", 
                   joint_names_[i].c_str(), info_.joints[i].command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check state interfaces
    if (info_.joints[i].state_interfaces.size() != 2)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ESP32HardwareInterface"),
                   "Joint '%s' has %zu state interface. 2 expected.", 
                   joint_names_[i].c_str(), info_.joints[i].state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Configuring...");

  // Create a non-lifecycle node for communication
  node_ = std::make_shared<rclcpp::Node>("esp32_hardware_interface_node");

  // Create publisher for velocity commands
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::QoS(10).reliable());

  // Create status publisher for diagnostics
  status_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/ros2_control/esp32_status", rclcpp::QoS(10).reliable());

  if (!use_mock_hardware_) {
    // Real hardware mode - create subscribers
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(10).reliable(), 
      std::bind(&ESP32HardwareInterface::joint_state_callback, this, std::placeholders::_1));

    diagnostic_sub_ = node_->create_subscription<std_msgs::msg::String>(
      diagnostic_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&ESP32HardwareInterface::diagnostic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
                "Configured for REAL hardware - waiting for ESP32 connection");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
                "Configured for MOCK hardware - simulation mode");
  }

  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ESP32HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ESP32HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ESP32HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Activating....");

  // Initialize values
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    hw_commands_[i] = 0.0;
    mock_velocities_[i] = 0.0;
  }

  last_joint_state_time_ = node_->now();

  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Deactivating....");

  // Send zero velocity to stop the robot
  geometry_msgs::msg::Twist zero_vel;
  zero_vel.linear.x = 0.0;
  zero_vel.angular.z = 0.0;
  vel_pub_->publish(zero_vel);
  
  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ESP32HardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  read_calls_++;
  
  if (use_mock_hardware_) {
    // Mock hardware simulation
    simulate_esp32_hardware(period);
    return hardware_interface::return_type::OK;
  }
  
  // Real hardware logic
  // Process any pending callbacks
  rclcpp::spin_some(node_);
  
  // Check for timeout
  auto time_since_last_msg = time - last_joint_state_time_;
  bool timeout = time_since_last_msg.seconds() > (timeout_ms_ / 1000.0);
  
  if (timeout && joint_state_received_) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ESP32HardwareInterface"), 
                         *node_->get_clock(), 1000,
                         "No joint state received for %.2f seconds", 
                         time_since_last_msg.seconds());
    missed_reads_++;
  }

  // Copy data from the latest joint state message if available
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (latest_joint_state_) {
    // Match joint names and copy data
    for (size_t i = 0; i < latest_joint_state_->name.size(); i++) {
      for (size_t j = 0; j < joint_names_.size(); j++) {
        if (latest_joint_state_->name[i] == joint_names_[j]) {
          hw_positions_[j] = latest_joint_state_->position[i];
          hw_velocities_[j] = latest_joint_state_->velocity[i];
          if (latest_joint_state_->effort.size() > i) {
            hw_efforts_[j] = latest_joint_state_->effort[i];
          }
          break;
        }
      }
    }
  }

  // Publish status every 100 reads
  if (read_calls_ % 100 == 0) {
    std_msgs::msg::String status_msg;
    std::stringstream ss;
    ss << "ESP32 Hardware Interface Status: "
       << "reads=" << read_calls_ 
       << ", writes=" << write_calls_
       << ", missed=" << missed_reads_
       << ", successful=" << successful_writes_
       << ", connected=" << (joint_state_received_ ? "yes" : "no")
       << ", timeout=" << (timeout ? "yes" : "no");
    
    if (!latest_diagnostic_.empty()) {
      ss << ", diagnostic=" << latest_diagnostic_;
    }
    
    status_msg.data = ss.str();
    status_pub_->publish(status_msg);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ESP32HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  write_calls_++;
  
  if (use_mock_hardware_) {
    // In mock mode, just store commands
    for (size_t i = 0; i < hw_commands_.size(); i++) {
      mock_velocities_[i] = hw_commands_[i];
    }
    return hardware_interface::return_type::OK;
  }

  // Real hardware logic
  // Find indices for each wheel
  size_t fl_idx = 0, fr_idx = 1, rl_idx = 2, rr_idx = 3;
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (joint_names_[i].find("front_left") != std::string::npos) {
      fl_idx = i;
    } else if (joint_names_[i].find("front_right") != std::string::npos) {
      fr_idx = i;
    } else if (joint_names_[i].find("rear_left") != std::string::npos) {
      rl_idx = i;
    } else if (joint_names_[i].find("rear_right") != std::string::npos) {
      rr_idx = i;
    }
  }
  
  // Average left and right wheel commands
  double left_wheel_vel = (hw_commands_[fl_idx] + hw_commands_[rl_idx]) / 2.0;
  double right_wheel_vel = (hw_commands_[fr_idx] + hw_commands_[rr_idx]) / 2.0;
  
  // Convert to linear and angular velocities
  double linear_x = wheel_radius_ * (left_wheel_vel + right_wheel_vel) / 2.0;
  double angular_z = wheel_radius_ * (right_wheel_vel - left_wheel_vel) / wheel_separation_;
  
  // Create and publish Twist message
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_x;
  cmd_vel.angular.z = angular_z;
  vel_pub_->publish(cmd_vel);
  
  successful_writes_++;
  
  // Log every 50 writes
  if (write_calls_ % 50 == 0) {
    RCLCPP_DEBUG(rclcpp::get_logger("ESP32HardwareInterface"),
                 "Sent cmd_vel: linear_x=%.3f, angular_z=%.3f (from wheel vels: L=%.3f, R=%.3f)",
                 linear_x, angular_z, left_wheel_vel, right_wheel_vel);
  }
  
  return hardware_interface::return_type::OK;
}

void ESP32HardwareInterface::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_joint_state_ = msg;
  joint_state_received_ = true;
  last_joint_state_time_ = node_->now();
  
  // Log first message
  static bool first_msg = true;
  if (first_msg) {
    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
                "First joint state received from ESP32!");
    first_msg = false;
  }
}

void ESP32HardwareInterface::diagnostic_callback(
  const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(diagnostic_mutex_);
  latest_diagnostic_ = msg->data;
  
  RCLCPP_DEBUG(rclcpp::get_logger("ESP32HardwareInterface"), 
               "ESP32 diagnostic: %s", msg->data.c_str());
}

void ESP32HardwareInterface::simulate_esp32_hardware(const rclcpp::Duration & period)
{
  // Simple simulation: integrate commanded velocities
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    // Update position based on velocity
    hw_positions_[i] += mock_velocities_[i] * period.seconds();
    
    // Simulate velocity response with some lag
    double alpha = 0.95; // Response factor
    hw_velocities_[i] = alpha * hw_velocities_[i] + (1.0 - alpha) * hw_commands_[i];
  }
}

}  // namespace ros2_control_test

// Register this hardware interface as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_test::ESP32HardwareInterface,
  hardware_interface::SystemInterface
)