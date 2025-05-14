#include "my_slam_pkg/esp32_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_slam_pkg
{

hardware_interface::CallbackReturn ESP32HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check if we should use simulated hardware
  use_sim_hardware_ = info_.hardware_parameters.count("use_sim_hardware") ? 
    (info_.hardware_parameters.at("use_sim_hardware") == "true") : false;

  if (use_sim_hardware_) {
    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Using simulated hardware");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Using real hardware");
  }

  // Setup joint variables
  joint_names_.resize(info_.joints.size());
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Store joint names
  for (size_t i = 0; i < info_.joints.size(); i++) {
    joint_names_[i] = info_.joints[i].name;
    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Joint %ld: %s", i, joint_names_[i].c_str());
  }

  // Get parameters from URDF
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);
  cmd_vel_topic_ = info_.hardware_parameters.count("cmd_vel_topic") ? 
    info_.hardware_parameters["cmd_vel_topic"] : "/cmd_vel";
  joint_states_topic_ = info_.hardware_parameters.count("joint_states_topic") ? 
    info_.hardware_parameters["joint_states_topic"] : "/joint_states";

  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
              "Wheel radius: %f, separation: %f", wheel_radius_, wheel_separation_);
  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
              "Using topics - cmd_vel: %s, joint_states: %s",
              cmd_vel_topic_.c_str(), joint_states_topic_.c_str());

  joint_state_received_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create a non-lifecycle node for communication
  node_ = std::make_shared<rclcpp::Node>("esp32_hardware_interface");

  // Create publisher for velocity commands
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::QoS(10).reliable());

  // Create subscription for joint states
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, rclcpp::QoS(10).reliable(), 
    std::bind(&ESP32HardwareInterface::joint_state_callback, this, std::placeholders::_1));

  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Hardware interface configured");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ESP32HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Export position, velocity state interfaces for each joint
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ESP32HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  // Export velocity command interfaces for each joint
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ESP32HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default values for joint states
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0.0;
    }
    if (std::isnan(hw_velocities_[i])) {
      hw_velocities_[i] = 0.0;
    }
    if (std::isnan(hw_efforts_[i])) {
      hw_efforts_[i] = 0.0;
    }
    hw_commands_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Hardware interface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Send zero velocity to stop the robot
  geometry_msgs::msg::Twist zero_vel;
  zero_vel.linear.x = 0.0;
  zero_vel.angular.z = 0.0;
  vel_pub_->publish(zero_vel);
  
  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Hardware interface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ESP32HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (use_sim_hardware_) {
    // Simulation logic - simple position and velocity integration
    for (size_t i = 0; i < hw_positions_.size(); i++) {
      // Update position based on velocity and period
      hw_positions_[i] += hw_commands_[i] * period.seconds();
      // Keep velocity same as commanded
      hw_velocities_[i] = hw_commands_[i];
    }
    return hardware_interface::return_type::OK;
  }
  
  // Real hardware logic
  // Process any pending callbacks from the joint_state_sub_
  rclcpp::spin_some(node_);
  
  // Check if we've received joint states
  if (!joint_state_received_) {
    RCLCPP_DEBUG(rclcpp::get_logger("ESP32HardwareInterface"), "No joint state received yet");
    return hardware_interface::return_type::OK;
  }

  // Copy data from the latest joint state message
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

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ESP32HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_sim_hardware_) {
    // In simulation mode, we don't need to publish commands
    // The simulated state is updated in the read function
    return hardware_interface::return_type::OK;
  }

  // Real hardware logic
  // Calculate linear and angular velocities from wheel commands
  // Assuming differential drive kinematics with 4 wheels (2 on each side)
  
  // For simplicity, assume:
  // front_left and rear_left wheels are controlled together
  // front_right and rear_right wheels are controlled together
  
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
  
  return hardware_interface::return_type::OK;
}

void ESP32HardwareInterface::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_joint_state_ = msg;
  joint_state_received_ = true;
}

}  // namespace my_slam_pkg

// Register this hardware interface as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  my_slam_pkg::ESP32HardwareInterface,
  hardware_interface::SystemInterface
)