// Copyright 2027 Lunabot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "lunabot_hardware/spark_flex_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
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
constexpr float kNaNf = std::numeric_limits<float>::quiet_NaN();

/// Node name for the component's own node, which exists only to own the
/// /drive/status publisher. Fixed rather than derived from info_.name so it
/// is a legal node name whatever the ros2_control block is called.
constexpr auto kStatusNodeName = "spark_flex_system";
constexpr auto kStatusTopic = "/drive/status";
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
  params_.status_publish_rate = get_hardware_parameter("status_publish_rate", 20.0);
  params_.command_timeout = get_hardware_parameter("command_timeout", 0.5);

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
  last_commands_.assign(joint_count, kNaN);
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

  start_status_publisher();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void SparkFlexSystem::start_status_publisher()
{
  if (params_.status_publish_rate <= 0.0) {
    RCLCPP_INFO(
      rclcpp::get_logger(kLogger), "status_publish_rate is 0, so %s is not published",
      kStatusTopic);
    return;
  }

  // A hardware component is normally loaded by controller_manager, where
  // rclcpp is long since initialised. It can also be loaded by a bare
  // ResourceManager -- which is exactly what test_load_spark_flex_system does
  // -- and constructing a node there throws rclcpp::ContextNotInitialized out
  // of a lifecycle callback. Telemetry is not worth failing a transition
  // over, so the check is here rather than a try/catch around the throw.
  if (!rclcpp::ok()) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLogger), "rclcpp is not initialised, so %s is not published",
      kStatusTopic);
    return;
  }

  status_node_ = std::make_shared<rclcpp::Node>(kStatusNodeName);
  auto publisher =
    status_node_->create_publisher<lunabot_msgs::msg::DriveStatus>(kStatusTopic, rclcpp::QoS(10));
  status_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<lunabot_msgs::msg::DriveStatus>>(publisher);

  // Everything that cannot change after configuration is written once, here.
  // publish_status() then touches only the fields that vary, which is what
  // keeps it allocation-free in the control loop: the motors vector is sized
  // now and never resized again.
  auto & msg = status_publisher_->msg_;
  msg.can_interface = params_.can_interface;
  msg.motors.resize(motors_.size());
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    auto & motor = msg.motors[i];
    motor.joint_name = motors_[i].config().joint_name;
    motor.can_id = static_cast<std::uint8_t>(motors_[i].config().can_id);

    // NaN, not zero, and not a sentinel. MotorStatus.msg's rule is that a
    // field the controller cannot report is NaN and the consumer checks --
    // so a plot of bus voltage shows a gap rather than a convincing 0 V.
    //
    // These four stay NaN for now whatever use_motor_feedback says, because
    // SparkFlexMotor exposes no getter for any of them. Wiring one is the
    // same job as wiring read_velocity(); see docs/HARDWARE_CAN.md.
    motor.velocity = kNaNf;
    motor.position = kNaNf;
    motor.bus_voltage = kNaNf;
    motor.output_current = kNaNf;
    motor.temperature = kNaNf;
    motor.fault_bits = 0;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger), "publishing %s at %.1f Hz", kStatusTopic,
    params_.status_publish_rate);
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
  std::fill(last_commands_.begin(), last_commands_.end(), 0.0);

  // Both clocks are learned from the first write() rather than seeded here:
  // on_activate has no time argument, and an rclcpp::Time default-constructs
  // against the system clock, which cannot be subtracted from a ROS-time
  // stamp without throwing.
  have_command_time_ = false;
  have_status_time_ = false;

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
  // Publisher before node: it holds a shared_ptr to a publisher created from
  // that node, and RealtimePublisher's destructor joins its own thread.
  status_publisher_.reset();
  status_node_.reset();
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
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }

  bool commanded = !have_command_time_;
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    const double command = std::isfinite(hw_commands_velocity_[i]) ? hw_commands_velocity_[i] : 0.0;
    motors_[i].set_velocity(command);

    // "The drivetrain was asked to do something" -- see publish_status() for
    // why that is the strongest statement available here, and why it is not
    // the same as "a command arrived".
    commanded |= (command != 0.0) || (command != last_commands_[i]);
    last_commands_[i] = command;
  }

  if (commanded) {
    last_command_time_ = time;
    have_command_time_ = true;
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

  publish_status(time);

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

void SparkFlexSystem::publish_status(const rclcpp::Time & time)
{
  if (!status_publisher_) {
    return;
  }

  const double status_period = 1.0 / params_.status_publish_rate;
  if (have_status_time_ && (time - last_status_time_).seconds() < status_period) {
    return;
  }

  // trylock() and not lock(): this runs in the control loop, ten milliseconds
  // of which also has to carry a CAN write and a heartbeat to four motor
  // controllers. A dropped status sample is invisible on a plot; a late
  // heartbeat faults the drivetrain.
  if (!status_publisher_->trylock()) {
    return;
  }

  last_status_time_ = time;
  have_status_time_ = true;

  // Only the fields that vary are touched from here on. can_interface, the
  // joint names and the CAN ids were written once in start_status_publisher()
  // -- assigning a std::string every cycle is the one thing in this function
  // that could allocate.
  auto & msg = status_publisher_->msg_;
  msg.header.stamp = time;
  msg.motor_feedback_active = params_.use_motor_feedback;

  // ============ WHAT time_since_last_command CAN AND CANNOT MEAN ============
  // A ros2_control command interface is a bare double in shared memory. It
  // carries no timestamp and no writer identity, so a controller writing the
  // same value every cycle and a controller that has died are byte-identical
  // from in here. There is no way to measure "seconds since a command
  // arrived", and reporting a number that claims to be that would be the
  // same class of lie as reporting echoed commands as measured velocity.
  //
  // What is measurable is the last cycle in which the drivetrain was asked
  // to move -- a non-zero command, or any change of command. So this is
  // seconds since the rover was last asked to do something, and its useful
  // reading is the one dashboards actually want: a rising value means
  // nothing is driving this robot.
  //
  // The cost of the definition is one false positive: an operator holding a
  // deliberate, sustained zero looks the same as a dead controller. That is
  // the correct trade -- the failure it does catch is silent, and the one it
  // confuses is not.
  // =========================================================================
  msg.time_since_last_command =
    have_command_time_ ? static_cast<float>((time - last_command_time_).seconds()) : 0.0F;

  // The hardware layer has no watchdog of its own any more -- cmd_vel_timeout
  // in diff_drive_controller replaced it, and command_timeout defaults to the
  // same 0.5 s. So this reports the controller's watchdog as seen from below:
  // the outputs are zero and have been for longer than the controller would
  // have tolerated silence. See docs/HARDWARE_CAN.md.
  msg.watchdog_triggered = msg.time_since_last_command > params_.command_timeout;

  for (std::size_t i = 0; i < motors_.size(); ++i) {
    auto & motor = msg.motors[i];
    motor.applied_duty_cycle = static_cast<float>(motors_[i].applied_duty_cycle());

    // Only when the numbers are measurements. With use_motor_feedback false,
    // hw_states_* hold the commanded velocity echoed back, and copying that
    // into a field called `velocity` would put a fabricated number on a
    // telemetry topic whose whole purpose is to be trusted. It stays NaN,
    // as MotorStatus.msg says it must.
    if (params_.use_motor_feedback) {
      motor.velocity = static_cast<float>(hw_states_velocity_[i]);
      motor.position = static_cast<float>(hw_states_position_[i]);
    }
  }

  status_publisher_->unlockAndPublish();
}

}  // namespace lunabot_hardware

PLUGINLIB_EXPORT_CLASS(lunabot_hardware::SparkFlexSystem, hardware_interface::SystemInterface)
