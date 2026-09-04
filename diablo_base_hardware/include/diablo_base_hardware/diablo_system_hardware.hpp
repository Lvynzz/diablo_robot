#ifndef DIABLO_BASE_HARDWARE__DIABLO_SYSTEM_HARDWARE_HPP_
#define DIABLO_BASE_HARDWARE__DIABLO_SYSTEM_HARDWARE_HPP_

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "motion_msgs/msg/leg_motors.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diablo_base_hardware
{

/**
 * @brief ros2_control adapter for the Diablo vehicle-level motion API.
 *
 * diff_drive_controller exposes left/right wheel velocity interfaces.  Diablo
 * instead accepts one vehicle-level forward velocity and one yaw-rate command
 * on /diablo/MotionCmd.  This SystemInterface performs that kinematic
 * conversion and exposes wheel feedback from /diablo/sensor/Motors.
 */
class DiabloSystemHardware final : public hardware_interface::SystemInterface
{
public:
  DiabloSystemHardware() = default;
  ~DiabloSystemHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void motors_callback(const motion_msgs::msg::LegMotors::SharedPtr msg);
  void publish_motion_command(double forward, double yaw_rate);
  void publish_crawl_command();

  static double parameter_as_double(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    double default_value,
    const rclcpp::Logger & logger);

  static std::string parameter_as_string(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    const std::string & default_value);

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr motion_publisher_;
  rclcpp::Subscription<motion_msgs::msg::LegMotors>::SharedPtr motors_subscription_;

  std::mutex feedback_mutex_;
  bool have_feedback_{false};
  rclcpp::Time last_feedback_time_;
  double left_raw_position_{0.0};
  double right_raw_position_{0.0};
  double left_raw_velocity_{0.0};
  double right_raw_velocity_{0.0};
  int left_revolutions_{0};
  int right_revolutions_{0};

  std::vector<double> commands_;
  std::vector<double> states_position_;
  std::vector<double> states_velocity_;
  std::size_t left_joint_index_{0};
  std::size_t right_joint_index_{0};

  std::string motion_cmd_topic_;
  std::string motors_topic_;
  double wheel_radius_{0.105};
  double track_width_{0.3751};
  double left_feedback_sign_{1.0};
  double right_feedback_sign_{-1.0};
  double max_forward_{1.0};
  double max_yaw_rate_{1.0};
  double crawl_up_{1.0};
  double feedback_timeout_ms_{1000.0};
  double command_publish_rate_{25.0};
  std::chrono::steady_clock::time_point last_command_publish_time_{};
  bool command_publish_initialized_{false};
  bool active_{false};
};

}  // namespace diablo_base_hardware

#endif  // DIABLO_BASE_HARDWARE__DIABLO_SYSTEM_HARDWARE_HPP_
