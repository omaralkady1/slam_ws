// hardware_interface/include/ros2_control_test/esp32_hardware_interface.hpp

#ifndef ROS2_CONTROL_TEST_ESP32_HARDWARE_INTERFACE_HPP
#define ROS2_CONTROL_TEST_ESP32_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>
#include <mutex>

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
#include "std_msgs/msg/string.hpp"

namespace ros2_control_test
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
  
  // Callback for diagnostic messages
  void diagnostic_callback(const std_msgs::msg::String::SharedPtr msg);

  // Mock hardware simulation
  void simulate_esp32_hardware(const rclcpp::Duration & period);

  // Flag to indicate if we're using mock hardware
  bool use_mock_hardware_;

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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr diagnostic_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // Wheel parameters from URDF
  double wheel_radius_;
  double wheel_separation_;

  // Message buffer for thread safety
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::mutex joint_state_mutex_;
  bool joint_state_received_;
  rclcpp::Time last_joint_state_time_;
  
  // Diagnostic data
  std::string latest_diagnostic_;
  std::mutex diagnostic_mutex_;

  // Parameters
  std::string cmd_vel_topic_;
  std::string joint_states_topic_;
  std::string diagnostic_topic_;
  int timeout_ms_;
  
  // Performance tracking
  size_t read_calls_;
  size_t write_calls_;
  size_t missed_reads_;
  size_t successful_writes_;
  
  // For mock hardware simulation
  std::vector<double> mock_velocities_;
};

}  // namespace ros2_control_test

#endif  // ROS2_CONTROL_TEST_ESP32_HARDWARE_INTERFACE_HPP