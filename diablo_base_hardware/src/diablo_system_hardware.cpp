#include "diablo_base_hardware/diablo_system_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace diablo_base_hardware
{

namespace
{
constexpr double kTwoPi = 6.283185307179586476925286766559;

bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const hardware_interface::InterfaceInfo & interface) {
      return interface.name == name;
    });
}

double clamp_symmetric(double value, double limit)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  if (!std::isfinite(limit) || limit <= 0.0) {
    return value;
  }
  return std::max(-limit, std::min(limit, value));
}
}  // namespace

DiabloSystemHardware::~DiabloSystemHardware()
{
  if (active_ && rclcpp::ok()) {
    publish_motion_command(0.0, 0.0);
  }
  if (executor_ && node_) {
    executor_->remove_node(node_->get_node_base_interface());
  }
}

hardware_interface::CallbackReturn DiabloSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("diablo_base_hardware"),
      "Expected exactly two wheel joints, received %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto logger = rclcpp::get_logger("diablo_base_hardware");
  bool found_left = false;
  bool found_right = false;
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const auto & joint = info_.joints[index];
    if (!has_interface(joint.command_interfaces, hardware_interface::HW_IF_VELOCITY)) {
      RCLCPP_ERROR(
        logger, "Joint '%s' must export a velocity command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(
        logger, "Joint '%s' must export position and velocity state interfaces",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.name == "left_wheel_joint") {
      left_joint_index_ = index;
      found_left = true;
    } else if (joint.name == "right_wheel_joint") {
      right_joint_index_ = index;
      found_right = true;
    } else {
      RCLCPP_ERROR(
        logger,
        "Unexpected joint '%s'; expected left_wheel_joint and right_wheel_joint",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (!found_left || !found_right) {
    RCLCPP_ERROR(logger, "Both left_wheel_joint and right_wheel_joint are required");
    return hardware_interface::CallbackReturn::ERROR;
  }

  commands_.assign(info_.joints.size(), 0.0);
  states_position_.assign(info_.joints.size(), 0.0);
  states_velocity_.assign(info_.joints.size(), 0.0);

  motion_cmd_topic_ = parameter_as_string(
    info_, "motion_cmd_topic", "/diablo/MotionCmd");
  motors_topic_ = parameter_as_string(
    info_, "motors_topic", "/diablo/sensor/Motors");
  wheel_radius_ = parameter_as_double(info_, "wheel_radius", wheel_radius_, logger);
  track_width_ = parameter_as_double(info_, "track_width", track_width_, logger);
  left_feedback_sign_ = parameter_as_double(
    info_, "left_feedback_sign", left_feedback_sign_, logger);
  right_feedback_sign_ = parameter_as_double(
    info_, "right_feedback_sign", right_feedback_sign_, logger);
  max_forward_ = parameter_as_double(info_, "max_forward", max_forward_, logger);
  max_yaw_rate_ = parameter_as_double(info_, "max_yaw_rate", max_yaw_rate_, logger);
  crawl_up_ = parameter_as_double(info_, "crawl_up", crawl_up_, logger);
  feedback_timeout_ms_ = parameter_as_double(
    info_, "feedback_timeout_ms", feedback_timeout_ms_, logger);
  command_publish_rate_ = parameter_as_double(
    info_, "command_publish_rate", command_publish_rate_, logger);

  if (!std::isfinite(command_publish_rate_) || command_publish_rate_ <= 0.0) {
    RCLCPP_WARN(
      logger,
      "command_publish_rate must be positive; using %.1f Hz",
      25.0);
    command_publish_rate_ = 25.0;
  }

  if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0 ||
    !std::isfinite(track_width_) || track_width_ <= 0.0)
  {
    RCLCPP_ERROR(logger, "wheel_radius and track_width must be positive finite values");
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ = std::make_shared<rclcpp::Node>("diablo_base_hardware");
  motion_publisher_ = node_->create_publisher<motion_msgs::msg::MotionCtrl>(
    motion_cmd_topic_, rclcpp::QoS(10));
  motors_subscription_ = node_->create_subscription<motion_msgs::msg::LegMotors>(
    motors_topic_, rclcpp::QoS(10),
    std::bind(&DiabloSystemHardware::motors_callback, this, std::placeholders::_1));

  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_->get_node_base_interface());

  RCLCPP_INFO(
    logger,
    "Initialized Diablo base adapter: cmd='%s', feedback='%s', radius=%.4f m, track=%.4f m",
    motion_cmd_topic_.c_str(), motors_topic_.c_str(), wheel_radius_, track_width_);
  RCLCPP_INFO(
    logger,
    "Feedback signs: left=%.1f, right=%.1f; command limits: forward=%.3f, yaw=%.3f; publish rate=%.1f Hz",
    left_feedback_sign_, right_feedback_sign_, max_forward_, max_yaw_rate_,
    command_publish_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DiabloSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(states_position_.size() * 2);
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    state_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &states_position_[index]);
    state_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_VELOCITY, &states_velocity_[index]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiabloSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(commands_.size());
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    command_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_VELOCITY, &commands_[index]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn DiabloSystemHardware::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  std::fill(commands_.begin(), commands_.end(), 0.0);
  active_ = true;
  command_publish_initialized_ = false;
  publish_crawl_command();
  RCLCPP_INFO(node_->get_logger(), "Diablo base activated; crawl-mode command sent once");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiabloSystemHardware::on_deactivate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  if (node_ && rclcpp::ok()) {
    publish_motion_command(0.0, 0.0);
  }
  command_publish_initialized_ = false;
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiabloSystemHardware::read(
  [[maybe_unused]] const rclcpp::Time & time,
  [[maybe_unused]] const rclcpp::Duration & period)
{
  if (!executor_ || !node_) {
    return hardware_interface::return_type::ERROR;
  }
  executor_->spin_some();

  double left_position = 0.0;
  double right_position = 0.0;
  double left_velocity = 0.0;
  double right_velocity = 0.0;
  bool have_feedback = false;
  bool feedback_is_stale = false;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    have_feedback = have_feedback_;
    if (have_feedback_) {
      left_position = left_feedback_sign_ *
        (left_raw_position_ + static_cast<double>(left_revolutions_) * kTwoPi);
      right_position = right_feedback_sign_ *
        (right_raw_position_ + static_cast<double>(right_revolutions_) * kTwoPi);
      left_velocity = left_feedback_sign_ * left_raw_velocity_;
      right_velocity = right_feedback_sign_ * right_raw_velocity_;
      if (feedback_timeout_ms_ > 0.0) {
        feedback_is_stale =
          (node_->now() - last_feedback_time_).seconds() * 1000.0 > feedback_timeout_ms_;
      }
    }
  }

  if (!have_feedback) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "No %s feedback received yet; wheel states remain zero", motors_topic_.c_str());
    return hardware_interface::return_type::OK;
  }
  if (feedback_is_stale) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Diablo motor feedback is stale (>%g ms)", feedback_timeout_ms_);
    publish_motion_command(0.0, 0.0);
    return hardware_interface::return_type::ERROR;
  }

  states_position_[left_joint_index_] = left_position;
  states_position_[right_joint_index_] = right_position;
  states_velocity_[left_joint_index_] = left_velocity;
  states_velocity_[right_joint_index_] = right_velocity;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiabloSystemHardware::write(
  [[maybe_unused]] const rclcpp::Time & time,
  [[maybe_unused]] const rclcpp::Duration & period)
{
  if (!motion_publisher_ || !active_) {
    return hardware_interface::return_type::ERROR;
  }

  const double left_wheel_velocity = commands_[left_joint_index_];
  const double right_wheel_velocity = commands_[right_joint_index_];
  const double linear = wheel_radius_ *
    (left_wheel_velocity + right_wheel_velocity) * 0.5;
  const double angular = wheel_radius_ *
    (right_wheel_velocity - left_wheel_velocity) / track_width_;

  const auto now = std::chrono::steady_clock::now();
  const double publish_period = 1.0 / command_publish_rate_;
  if (command_publish_initialized_ &&
    std::chrono::duration<double>(now - last_command_publish_time_).count() < publish_period)
  {
    return hardware_interface::return_type::OK;
  }

  publish_motion_command(
    clamp_symmetric(linear, max_forward_),
    clamp_symmetric(angular, max_yaw_rate_));
  last_command_publish_time_ = now;
  command_publish_initialized_ = true;
  return hardware_interface::return_type::OK;
}

void DiabloSystemHardware::motors_callback(
  const motion_msgs::msg::LegMotors::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  left_raw_position_ = msg->left_wheel_pos;
  right_raw_position_ = msg->right_wheel_pos;
  left_raw_velocity_ = msg->left_wheel_vel;
  right_raw_velocity_ = msg->right_wheel_vel;
  left_revolutions_ = msg->left_wheel_enc_rev;
  right_revolutions_ = msg->right_wheel_enc_rev;
  last_feedback_time_ = node_->now();
  have_feedback_ = true;
}

void DiabloSystemHardware::publish_motion_command(double forward, double yaw_rate)
{
  if (!motion_publisher_) {
    return;
  }
  motion_msgs::msg::MotionCtrl message;
  message.mode_mark = false;
  message.value.forward = forward;
  message.value.left = yaw_rate;
  message.value.up = crawl_up_;
  message.value.roll = 0.0;
  message.value.pitch = 0.0;
  message.value.leg_split = 0.0;
  motion_publisher_->publish(message);
}

void DiabloSystemHardware::publish_crawl_command()
{
  if (!motion_publisher_) {
    return;
  }
  motion_msgs::msg::MotionCtrl message;
  message.mode_mark = true;
  message.mode.stand_mode = false;
  message.mode.jump_mode = false;
  message.mode.split_mode = false;
  message.mode.pitch_ctrl_mode = false;
  message.mode.roll_ctrl_mode = false;
  message.mode.height_ctrl_mode = false;
  message.value.up = crawl_up_;
  motion_publisher_->publish(message);
}

double DiabloSystemHardware::parameter_as_double(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  double default_value,
  const rclcpp::Logger & logger)
{
  const auto iterator = info.hardware_parameters.find(name);
  if (iterator == info.hardware_parameters.end()) {
    return default_value;
  }
  try {
    return std::stod(iterator->second);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      logger, "Invalid value '%s' for parameter '%s': %s; using %.6f",
      iterator->second.c_str(), name.c_str(), error.what(), default_value);
    return default_value;
  }
}

std::string DiabloSystemHardware::parameter_as_string(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const std::string & default_value)
{
  const auto iterator = info.hardware_parameters.find(name);
  return iterator == info.hardware_parameters.end() ? default_value : iterator->second;
}

}  // namespace diablo_base_hardware

PLUGINLIB_EXPORT_CLASS(
  diablo_base_hardware::DiabloSystemHardware,
  hardware_interface::SystemInterface)
