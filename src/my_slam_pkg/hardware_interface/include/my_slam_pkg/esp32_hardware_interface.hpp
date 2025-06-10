#ifndef MY_SLAM_PKG_ESP32_HARDWARE_INTERFACE_HPP
#define MY_SLAM_PKG_ESP32_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <chrono>

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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace my_slam_pkg
{

class ESP32HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ESP32HardwareInterface)

  /**
   * @brief Initialize the hardware interface from hardware description
   * @param info Hardware description from URDF
   * @return Success or error
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Configure the hardware interface
   * @param previous_state Previous lifecycle state
   * @return Success or error
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Export state interfaces for reading hardware state
   * @return Vector of state interfaces
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Export command interfaces for sending commands to hardware
   * @return Vector of command interfaces
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Activate the hardware interface
   * @param previous_state Previous lifecycle state
   * @return Success or error
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate the hardware interface
   * @param previous_state Previous lifecycle state
   * @return Success or error
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Read hardware state from ESP32
   * @param time Current time
   * @param period Time since last read
   * @return Success or error
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Write commands to ESP32 hardware
   * @param time Current time
   * @param period Time since last write
   * @return Success or error
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /**
   * @brief Callback for joint state messages from ESP32 micro-ROS
   * @param msg Joint state message
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief Callback for odometry messages from ESP32 micro-ROS
   * @param msg Odometry message
   */
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Configuration flags
  bool use_sim_hardware_;
  bool use_stamped_vel_;

  // Joint configuration
  std::vector<std::string> joint_names_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Encoder direction correction for proper SLAM odometry
  double encoder_direction_[4]; // Multipliers for each wheel: +1 or -1

  // ROS2 communication interfaces
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_stamped_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  // Robot physical parameters (from URDF/config)
  double wheel_radius_;
  double wheel_separation_;

  // Communication topics
  std::string cmd_vel_topic_;
  std::string joint_states_topic_;
  std::string odom_topic_;

  // Thread-safe message handling
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  nav_msgs::msg::Odometry::SharedPtr latest_odometry_;
  std::mutex joint_state_mutex_;
  std::mutex odometry_mutex_;
  bool joint_state_received_;
  bool odometry_received_;

  // Rate limiting for commands
  std::chrono::steady_clock::time_point last_command_time_;

  // Constants for joint indices (matching ESP32 firmware)
  static constexpr size_t FRONT_LEFT_WHEEL = 0;
  static constexpr size_t FRONT_RIGHT_WHEEL = 1;
  static constexpr size_t REAR_LEFT_WHEEL = 2;
  static constexpr size_t REAR_RIGHT_WHEEL = 3;
  static constexpr size_t NUM_JOINTS = 4;
};

}  // namespace my_slam_pkg

#endif  // MY_SLAM_PKG_ESP32_HARDWARE_INTERFACE_HPP