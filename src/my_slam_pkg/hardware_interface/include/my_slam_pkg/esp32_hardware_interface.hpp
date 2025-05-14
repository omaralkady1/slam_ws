#ifndef MY_SLAM_PKG_ESP32_HARDWARE_INTERFACE_HPP
#define MY_SLAM_PKG_ESP32_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Messages for micro-ROS communication
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace my_slam_pkg
{

class ESP32HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ESP32HardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Callback for joint state messages from micro-ROS
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Flag to indicate if we're using simulated hardware
  bool use_sim_hardware_;

  // Wheel joints
  std::vector<std::string> joint_names_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // ROS interfaces
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Wheel parameters from URDF
  double wheel_radius_;
  double wheel_separation_;

  // Message buffer for thread safety
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::mutex joint_state_mutex_;
  bool joint_state_received_;

  // Parameters
  std::string cmd_vel_topic_;
  std::string joint_states_topic_;
};

}  // namespace my_slam_pkg

#endif  // MY_SLAM_PKG_ESP32_HARDWARE_INTERFACE_HPP