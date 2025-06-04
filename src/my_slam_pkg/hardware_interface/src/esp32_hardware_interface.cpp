
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
    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Using simulated ESP32 hardware");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Using real ESP32 hardware via micro-ROS");
  }

  // Setup joint variables - must match your URDF joint names
  joint_names_.resize(info_.joints.size());
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Store joint names and verify they match expected pattern
  for (size_t i = 0; i < info_.joints.size(); i++) {
    joint_names_[i] = info_.joints[i].name;
    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Joint %ld: %s", i, joint_names_[i].c_str());
  }

  // Get robot parameters from hardware parameters
  if (info_.hardware_parameters.find("wheel_radius") != info_.hardware_parameters.end()) {
    wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
  } else {
    wheel_radius_ = 0.05;  // Default from your config
    RCLCPP_WARN(rclcpp::get_logger("ESP32HardwareInterface"), "Using default wheel_radius: %f", wheel_radius_);
  }

  if (info_.hardware_parameters.find("wheel_separation") != info_.hardware_parameters.end()) {
    wheel_separation_ = std::stod(info_.hardware_parameters.at("wheel_separation"));
  } else {
    wheel_separation_ = 0.34;  // Default from your config
    RCLCPP_WARN(rclcpp::get_logger("ESP32HardwareInterface"), "Using default wheel_separation: %f", wheel_separation_);
  }

  // Get topic names
  cmd_vel_topic_ = info_.hardware_parameters.count("cmd_vel_topic") ? 
    info_.hardware_parameters.at("cmd_vel_topic") : "/cmd_vel";
  joint_states_topic_ = info_.hardware_parameters.count("joint_states_topic") ? 
    info_.hardware_parameters.at("joint_states_topic") : "/joint_states";

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
  node_ = std::make_shared<rclcpp::Node>("esp32_hardware_interface_node");

  if (!use_sim_hardware_) {
    // For real hardware: Create publisher for velocity commands to ESP32
    vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10).reliable());

    // Create subscription for joint states from ESP32
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(10).reliable(), 
      std::bind(&ESP32HardwareInterface::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
                "Configured for real ESP32 hardware communication");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
                "Configured for simulated ESP32 hardware");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ESP32HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Export position, velocity, and effort state interfaces for each joint
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
  if (!use_sim_hardware_ && vel_pub_) {
    // Send zero velocity to stop the robot
    geometry_msgs::msg::Twist zero_vel;
    zero_vel.linear.x = 0.0;
    zero_vel.angular.z = 0.0;
    vel_pub_->publish(zero_vel);
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Hardware interface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ESP32HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (use_sim_hardware_) {
    // Simulation logic - simple integration
    for (size_t i = 0; i < hw_positions_.size(); i++) {
      // Update position based on velocity and period
      hw_positions_[i] += hw_commands_[i] * period.seconds();
      // Set velocity to match commanded velocity
      hw_velocities_[i] = hw_commands_[i];
      // No effort calculation in simulation
      hw_efforts_[i] = 0.0;
    }
    return hardware_interface::return_type::OK;
  }
  
  // Real hardware logic
  // Process any pending callbacks from the joint_state_sub_
  if (node_) {
    rclcpp::spin_some(node_);
  }
  
  // Check if we've received joint states from ESP32
  if (!joint_state_received_) {
    RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("ESP32HardwareInterface"), 
                          *node_->get_clock(), 1000, "No joint state received from ESP32 yet");
    return hardware_interface::return_type::OK;
  }

  // Copy data from the latest joint state message
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (latest_joint_state_ && latest_joint_state_->name.size() >= 4) {
    // Map joint states from ESP32 to our joint order
    // ESP32 publishes in order: [front_left, front_right, rear_left, rear_right]
    for (size_t esp32_idx = 0; esp32_idx < latest_joint_state_->name.size() && esp32_idx < 4; esp32_idx++) {
      // Find corresponding joint in our configuration
      for (size_t our_idx = 0; our_idx < joint_names_.size(); our_idx++) {
        if (latest_joint_state_->name[esp32_idx] == joint_names_[our_idx]) {
          if (esp32_idx < latest_joint_state_->position.size()) {
            hw_positions_[our_idx] = latest_joint_state_->position[esp32_idx];
          }
          if (esp32_idx < latest_joint_state_->velocity.size()) {
            hw_velocities_[our_idx] = latest_joint_state_->velocity[esp32_idx];
          }
          if (esp32_idx < latest_joint_state_->effort.size()) {
            hw_efforts_[our_idx] = latest_joint_state_->effort[esp32_idx];
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
    return hardware_interface::return_type::OK;
  }

  if (!vel_pub_) {
    return hardware_interface::return_type::ERROR;
  }

  // Convert joint velocity commands to differential drive kinematics
  // Find wheel velocities by name to ensure correct mapping
  double fl_vel = 0.0, fr_vel = 0.0, rl_vel = 0.0, rr_vel = 0.0;
  
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (joint_names_[i] == "front_left_wheel_joint") {
      fl_vel = hw_commands_[i];
    } else if (joint_names_[i] == "front_right_wheel_joint") {
      fr_vel = hw_commands_[i];
    } else if (joint_names_[i] == "rear_left_wheel_joint") {
      rl_vel = hw_commands_[i];
    } else if (joint_names_[i] == "rear_right_wheel_joint") {
      rr_vel = hw_commands_[i];
    }
  }
  
  // Average left and right wheel commands for differential drive
  double left_wheel_vel = (fl_vel + rl_vel) / 2.0;
  double right_wheel_vel = (fr_vel + rr_vel) / 2.0;
  
  // Convert wheel velocities to robot linear and angular velocities
  double linear_x = wheel_radius_ * (left_wheel_vel + right_wheel_vel) / 2.0;
  double angular_z = wheel_radius_ * (right_wheel_vel - left_wheel_vel) / wheel_separation_;
  
  // Create and publish Twist message to ESP32
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_x;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
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