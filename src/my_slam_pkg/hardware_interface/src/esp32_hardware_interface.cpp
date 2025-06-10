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
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

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
    info_.hardware_parameters.at("cmd_vel_topic") : "/cmd_vel_stamped";
  joint_states_topic_ = info_.hardware_parameters.count("joint_states_topic") ? 
    info_.hardware_parameters.at("joint_states_topic") : "/joint_states";
  odom_topic_ = info_.hardware_parameters.count("odom_topic") ? 
    info_.hardware_parameters.at("odom_topic") : "/odom";

  // Get control mode
  use_stamped_vel_ = info_.hardware_parameters.count("use_stamped_vel") ? 
    (info_.hardware_parameters.at("use_stamped_vel") == "true") : true;

  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
              "Wheel radius: %f, separation: %f", wheel_radius_, wheel_separation_);
  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
              "Using topics - cmd_vel: %s, joint_states: %s, odom: %s",
              cmd_vel_topic_.c_str(), joint_states_topic_.c_str(), odom_topic_.c_str());

  joint_state_received_ = false;
  odometry_received_ = false;
  last_command_time_ = std::chrono::steady_clock::now();
  
  // Initialize encoder direction correction flags
  // These match the ESP32 firmware configuration
  encoder_direction_[0] = 1.0;  // FL - normal
  encoder_direction_[1] = -1.0; // FR - reversed (typical for right wheels)
  encoder_direction_[2] = 1.0;  // RL - normal  
  encoder_direction_[3] = -1.0; // RR - reversed (typical for right wheels)
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create a non-lifecycle node for communication
  node_ = std::make_shared<rclcpp::Node>("esp32_hardware_interface_node");

  if (!use_sim_hardware_) {
    // For real hardware: Create publisher for velocity commands to ESP32
    if (use_stamped_vel_) {
      vel_stamped_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic_, rclcpp::QoS(10).reliable());
    } else {
      vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::QoS(10).reliable());
    }

    // Create subscription for joint states from ESP32
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(10).reliable(), 
      std::bind(&ESP32HardwareInterface::joint_state_callback, this, std::placeholders::_1));

    // Create subscription for odometry from ESP32
    odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&ESP32HardwareInterface::odometry_callback, this, std::placeholders::_1));

    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
                "Configured for real ESP32 hardware communication with PID control");
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
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  // Initialize timing
  last_command_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Hardware interface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!use_sim_hardware_) {
    // Send zero velocity to stop the robot
    if (use_stamped_vel_ && vel_stamped_pub_) {
      geometry_msgs::msg::TwistStamped zero_vel;
      zero_vel.header.stamp = node_->now();
      zero_vel.twist.linear.x = 0.0;
      zero_vel.twist.angular.z = 0.0;
      vel_stamped_pub_->publish(zero_vel);
    } else if (vel_pub_) {
      geometry_msgs::msg::Twist zero_vel;
      zero_vel.linear.x = 0.0;
      zero_vel.angular.z = 0.0;
      vel_pub_->publish(zero_vel);
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Hardware interface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ESP32HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (use_sim_hardware_) {
    // Enhanced simulation with PID-like response
    for (size_t i = 0; i < hw_positions_.size(); i++) {
      // Simulate motor response with first-order dynamics
      double target_vel = hw_commands_[i];
      double current_vel = hw_velocities_[i];
      
      // Time constant for motor response (seconds)
      double tau = 0.1;
      double alpha = period.seconds() / (tau + period.seconds());
      
      // Update velocity with exponential response
      hw_velocities_[i] = current_vel + alpha * (target_vel - current_vel);
      
      // Update position based on velocity
      hw_positions_[i] += hw_velocities_[i] * period.seconds();
      
      // Simulate effort proportional to acceleration
      double acceleration = (hw_velocities_[i] - current_vel) / period.seconds();
      hw_efforts_[i] = acceleration * 0.1; // Arbitrary scaling
    }
    return hardware_interface::return_type::OK;
  }
  
  // Real hardware logic
  // Process any pending callbacks
  if (node_) {
    rclcpp::spin_some(node_);
  }
  
  // Check if we've received joint states from ESP32
  if (!joint_state_received_) {
    static auto last_warn_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_warn_time).count() > 2) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ESP32HardwareInterface"), 
                           *node_->get_clock(), 2000, "No joint state received from ESP32 yet");
      last_warn_time = now;
    }
    return hardware_interface::return_type::OK;
  }

  // Copy data from the latest joint state message
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (latest_joint_state_ && latest_joint_state_->name.size() >= 4) {
    // Map ESP32 joint states to our hardware interface
    for (size_t esp32_idx = 0; esp32_idx < latest_joint_state_->name.size() && esp32_idx < 4; esp32_idx++) {
      // Find corresponding joint in our configuration
      for (size_t our_idx = 0; our_idx < joint_names_.size(); our_idx++) {
        if (latest_joint_state_->name[esp32_idx] == joint_names_[our_idx]) {
          // The ESP32 firmware now handles direction correction internally
          // so we can directly use the values
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
    
    // Diagnostic output
    static auto last_diag = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_diag).count() > 5) {
      RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
                  "Joint states - Positions: [%.3f, %.3f, %.3f, %.3f] rad, "
                  "Velocities: [%.3f, %.3f, %.3f, %.3f] rad/s",
                  hw_positions_[0], hw_positions_[1], hw_positions_[2], hw_positions_[3],
                  hw_velocities_[0], hw_velocities_[1], hw_velocities_[2], hw_velocities_[3]);
      
      if (odometry_received_) {
        RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
                    "Odometry received from ESP32 - Position: (%.3f, %.3f), Orientation: %.3f rad",
                    latest_odometry_->pose.pose.position.x,
                    latest_odometry_->pose.pose.position.y,
                    2.0 * atan2(latest_odometry_->pose.pose.orientation.z,
                               latest_odometry_->pose.pose.orientation.w));
      }
      last_diag = now;
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

  if (!vel_pub_ && !vel_stamped_pub_) {
    return hardware_interface::return_type::ERROR;
  }

  // Rate limiting: Don't send commands too frequently
  auto now = std::chrono::steady_clock::now();
  auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time_);
  if (time_since_last.count() < 20) { // Limit to 50Hz max
    return hardware_interface::return_type::OK;
  }
  last_command_time_ = now;

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
  
  // Average left and right wheel commands
  double left_wheel_vel = (fl_vel + rl_vel) / 2.0;
  double right_wheel_vel = (fr_vel + rr_vel) / 2.0;
  
  // Convert to robot velocities
  double linear_x = wheel_radius_ * (left_wheel_vel + right_wheel_vel) / 2.0;
  double angular_z = wheel_radius_ * (right_wheel_vel - left_wheel_vel) / wheel_separation_;
  
  // Apply velocity limits for safety
  linear_x = std::max(-0.6, std::min(0.6, linear_x));   // Max 0.6 m/s (from YAML)
  angular_z = std::max(-1.0, std::min(1.0, angular_z)); // Max 1.0 rad/s (from YAML)
  
  // Publish command
  if (use_stamped_vel_ && vel_stamped_pub_) {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.twist.linear.x = linear_x;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.linear.z = 0.0;
    cmd_vel.twist.angular.x = 0.0;
    cmd_vel.twist.angular.y = 0.0;
    cmd_vel.twist.angular.z = angular_z;
    vel_stamped_pub_->publish(cmd_vel);
  } else if (vel_pub_) {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_x;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = angular_z;
    vel_pub_->publish(cmd_vel);
  }
  
  // Debug output
  static auto last_debug = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::seconds>(now - last_debug).count() > 1) {
    if (std::abs(linear_x) > 0.01 || std::abs(angular_z) > 0.01) {
      RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
                   "Sending cmd_vel: linear_x=%.3f m/s, angular_z=%.3f rad/s "
                   "(from wheel commands: FL=%.3f, FR=%.3f, RL=%.3f, RR=%.3f rad/s)", 
                   linear_x, angular_z, fl_vel, fr_vel, rl_vel, rr_vel);
      last_debug = now;
    }
  }
  
  return hardware_interface::return_type::OK;
}

void ESP32HardwareInterface::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_joint_state_ = msg;
  joint_state_received_ = true;
  
  // Debug first few messages
  static int message_count = 0;
  if (++message_count <= 3) {
    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
                "Received joint state #%d with %ld joints", message_count, msg->name.size());
  }
}

void ESP32HardwareInterface::odometry_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odometry_mutex_);
  latest_odometry_ = msg;
  odometry_received_ = true;
  
  // Debug first few messages
  static int message_count = 0;
  if (++message_count <= 3) {
    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), 
                "Received odometry #%d from ESP32 - x: %.3f, y: %.3f", 
                message_count, msg->pose.pose.position.x, msg->pose.pose.position.y);
  }
}

}  // namespace my_slam_pkg

// Register this hardware interface as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  my_slam_pkg::ESP32HardwareInterface,
  hardware_interface::SystemInterface
)