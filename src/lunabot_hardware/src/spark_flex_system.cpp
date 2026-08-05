// Copyright 2027 Lunabot. Licensed under the MIT License.

#include "lunabot_hardware/spark_flex_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lunabot_hardware
{
namespace
{
constexpr auto kLogger = "SparkFlexSystem";
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

// ---------------------------------------------------------------------------
// Parameter helpers
// ---------------------------------------------------------------------------

std::string SparkFlexSystem::get_hardware_parameter(
  const std::string & name, const std::string & fallback) const
{
  const auto it = info_.hardware_parameters.find(name);
  return it != info_.hardware_parameters.end() ? it->second : fallback;
}

double SparkFlexSystem::get_hardware_parameter(const std::string & name, double fallback) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return fallback;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLogger), "hardware parameter '%s' is not a number ('%s'); using %f",
      name.c_str(), it->second.c_str(), fallback);
    return fallback;
  }
}

bool SparkFlexSystem::get_hardware_parameter(const std::string & name, bool fallback) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return fallback;
  }
  return it->second == "true" || it->second == "True" || it->second == "1";
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn SparkFlexSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // No hardware is touched here. on_init runs during URDF parsing, long
  // before anyone has decided the robot should be live, and opening a CAN
  // socket at that point makes `ros2 control list_hardware_components` a
  // side-effecting command.
  params_.can_interface = get_hardware_parameter("can_interface", std::string("can0"));
  params_.use_motor_feedback = get_hardware_parameter("use_motor_feedback", false);
  params_.max_wheel_rad_s = get_hardware_parameter("max_wheel_rad_s", 16.0);
  params_.max_duty_cycle = get_hardware_parameter("max_duty_cycle", 0.8);
  params_.gear_ratio = get_hardware_parameter("gear_ratio", 20.0);
  params_.ramp_rate = get_hardware_parameter("ramp_rate", 0.1);

  if (params_.max_wheel_rad_s <= 0.0) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kLogger), "max_wheel_rad_s must be positive, got %f",
      params_.max_wheel_rad_s);
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::size_t joint_count = info_.joints.size();
  hw_commands_velocity_.assign(joint_count, kNaN);
  hw_states_position_.assign(joint_count, kNaN);
  hw_states_velocity_.assign(joint_count, kNaN);
  motors_.clear();
  motors_.reserve(joint_count);

  std::vector<int> seen_can_ids;

  for (const auto & joint : info_.joints) {
    // Validate against what diff_drive_controller will ask for. Failing here
    // produces one clear message; failing later produces a controller that
    // refuses to activate for reasons that read as a controller bug.
    if (
      joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "joint '%s' must have exactly one command interface, 'velocity'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_position = false;
    bool has_velocity = false;
    for (const auto & state : joint.state_interfaces) {
      has_position |= state.name == hardware_interface::HW_IF_POSITION;
      has_velocity |= state.name == hardware_interface::HW_IF_VELOCITY;
    }
    if (!has_position || !has_velocity) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "joint '%s' must export both 'position' and 'velocity' state interfaces",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto can_id_it = joint.parameters.find("can_id");
    if (can_id_it == joint.parameters.end()) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger), "joint '%s' has no 'can_id' parameter", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    SparkFlexMotor::Config config;
    config.joint_name = joint.name;
    try {
      config.can_id = std::stoi(can_id_it->second);
    } catch (const std::exception &) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger), "joint '%s' has a non-integer can_id '%s'", joint.name.c_str(),
        can_id_it->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Two motors on one CAN address is a wiring mistake that presents as one
    // wheel not responding, which looks like a dead motor.
    if (std::find(seen_can_ids.begin(), seen_can_ids.end(), config.can_id) != seen_can_ids.end()) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger), "CAN id %d is assigned to more than one joint", config.can_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    seen_can_ids.push_back(config.can_id);

    const auto invert_it = joint.parameters.find("invert");
    config.invert =
      invert_it != joint.parameters.end() &&
      (invert_it->second == "true" || invert_it->second == "True" || invert_it->second == "1");

    config.max_wheel_rad_s = params_.max_wheel_rad_s;
    config.max_duty_cycle = params_.max_duty_cycle;
    config.gear_ratio = params_.gear_ratio;
    config.ramp_rate = params_.ramp_rate;

    motors_.emplace_back(std::move(config));
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger), "initialised %zu motors on '%s' (motor feedback %s)",
    motors_.size(), params_.can_interface.c_str(),
    params_.use_motor_feedback ? "enabled" : "DISABLED - odometry is dead reckoning");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SparkFlexSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Opening the bus here rather than in a constructor is the whole point of
  // the lifecycle: a down CAN interface becomes a clean transition failure
  // that controller_manager reports, instead of an exception escaping a
  // constructor with a stack trace that never mentions CAN. The 2026 node
  // threw from its constructor.
  for (auto & motor : motors_) {
    try {
      motor.connect(params_.can_interface);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(rclcpp::get_logger(kLogger), "%s", e.what());
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "is the interface up?  sudo ip link set %s up type can bitrate 1000000",
        params_.can_interface.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger), "connected to %zu SparkFlex controllers on %s", motors_.size(),
    params_.can_interface.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SparkFlexSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (params_.use_motor_feedback) {
    // Refuse rather than silently produce NaN states, which would propagate
    // into diff_drive_controller and out into /odom and TF as NaN poses --
    // and NaN in a transform poisons the whole tree in a way that is very
    // hard to trace back to here.
    RCLCPP_FATAL(
      rclcpp::get_logger(kLogger),
      "use_motor_feedback is true, but SparkFlexMotor::read_velocity() is not wired to a "
      "sparkcan getter yet. See the instructions in src/spark_flex_motor.cpp, or set "
      "use_motor_feedback to false in the ros2_control xacro.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & motor : motors_) {
    try {
      motor.configure();
    } catch (const std::exception & e) {
      RCLCPP_FATAL(rclcpp::get_logger(kLogger), "failed to configure motor: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Start from a known state. Leaving these NaN means the first read() feeds
  // NaN to the controller before the first write() has happened.
  std::fill(hw_commands_velocity_.begin(), hw_commands_velocity_.end(), 0.0);
  std::fill(hw_states_position_.begin(), hw_states_position_.end(), 0.0);
  std::fill(hw_states_velocity_.begin(), hw_states_velocity_.end(), 0.0);

  for (auto & motor : motors_) {
    motor.stop();
  }

  active_ = true;
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SparkFlexSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  for (auto & motor : motors_) {
    motor.stop();
    // One last heartbeat so the controllers see a deliberate zero rather than
    // a bus that went quiet, which they would treat as a fault.
    motor.heartbeat();
  }
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "deactivated, motors stopped and braked");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SparkFlexSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  motors_.clear();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interfaces
// ---------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface> SparkFlexSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> SparkFlexSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]);
  }
  return interfaces;
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

hardware_interface::return_type SparkFlexSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  for (std::size_t i = 0; i < motors_.size(); ++i) {
    if (params_.use_motor_feedback) {
      hw_states_velocity_[i] = motors_[i].read_velocity();
    } else {
      // ============= DEAD RECKONING FROM COMMAND =============
      // No encoders, so "measured" velocity is the commanded velocity echoed
      // back and position is its integral.
      //
      // This is not an approximation that degrades gracefully. The moment a
      // wheel slips -- which on regolith is continuously -- reported motion
      // and actual motion diverge with no bound and nothing to correct them.
      // /odom will confidently report a straight line while the rover sits
      // spinning in place.
      //
      // Everything downstream that trusts /odom inherits this. That is the
      // argument for the EKF and for visual odometry, not a reason to
      // pretend the number is better than it is.
      // =======================================================
      const double command =
        std::isfinite(hw_commands_velocity_[i]) ? hw_commands_velocity_[i] : 0.0;
      hw_states_velocity_[i] = command;
    }

    if (std::isfinite(hw_states_velocity_[i]) && dt > 0.0) {
      hw_states_position_[i] += hw_states_velocity_[i] * dt;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SparkFlexSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }

  for (std::size_t i = 0; i < motors_.size(); ++i) {
    const double command = std::isfinite(hw_commands_velocity_[i]) ? hw_commands_velocity_[i] : 0.0;
    motors_[i].set_velocity(command);
  }

  // The heartbeat the SparkFlex controllers need roughly every 50 ms.
  //
  // No separate timer, and that is deliberate. controller_manager calls
  // write() at update_rate (100 Hz, i.e. every 10 ms) whenever this component
  // is active, regardless of whether any controller is running -- so the
  // heartbeat cannot drift out of sync with the control loop the way the
  // 2026 node's independent 50 ms wall timer could.
  //
  // If update_rate in controllers.yaml ever drops below ~40 Hz, this stops
  // being true and the motors will fault out.
  for (auto & motor : motors_) {
    motor.heartbeat();
  }

  return hardware_interface::return_type::OK;
}

}  // namespace lunabot_hardware

PLUGINLIB_EXPORT_CLASS(lunabot_hardware::SparkFlexSystem, hardware_interface::SystemInterface)
