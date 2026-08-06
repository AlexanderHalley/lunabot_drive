// Copyright 2027 Lunabot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef LUNABOT_HARDWARE__SPARK_FLEX_SYSTEM_HPP_
#define LUNABOT_HARDWARE__SPARK_FLEX_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lunabot_hardware/spark_flex_motor.hpp"
#include "lunabot_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace lunabot_hardware
{

/// ros2_control system interface for the 4WD SparkFlex drivetrain.
///
/// Replaces the 2026 drive_node.cpp, which subscribed to cmd_vel and wrote
/// duty cycle straight to the motors. That design had no place to put
/// odometry, so SLAM and Nav2 were both blocked on rewriting it.
///
/// Exports, per wheel joint:
///   command: velocity (rad/s)
///   state:   position (rad), velocity (rad/s)
///
/// diff_drive_controller consumes exactly that. Position state is what allows
/// position_feedback: true in controllers.yaml; without it the controller
/// integrates velocity and drifts faster.
///
/// ==================== THE OPEN-LOOP HOLE ====================
/// The 2026 drivetrain has no encoders, so this runs with
/// use_motor_feedback: false, where read() echoes the commanded velocity back
/// as though it were measured.
///
/// The resulting /odom has the right topology, the right frames and the right
/// units, and it is WRONG under any wheel slip -- which on regolith is always.
/// It is dead reckoning from command, not from measurement.
///
/// This is documented rather than hidden because it is the reason
/// robot_localization ships in the skeleton disabled rather than deferred:
/// wheel odometry is known-bad before the first test, so the fusion path needs
/// wiring and exercising in sim from day one. See docs/HARDWARE_CAN.md.
/// =============================================================
class SparkFlexSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SparkFlexSystem)

  // NOTE ON API VERSION: these signatures target hardware_interface 4.x as
  // shipped in ROS 2 Jazzy. Rolling and Kilted moved to on_export_state_
  // interfaces() and HardwareComponentInterfaceParams. Do not copy examples
  // from the Rolling documentation into this file -- they will compile
  // against a different base class and fail to load at runtime with a
  // pluginlib error that does not mention the version.

  LUNABOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  LUNABOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  LUNABOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  LUNABOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  LUNABOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  LUNABOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  LUNABOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  LUNABOT_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  LUNABOT_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Everything read out of the URDF's <hardware> block.
  struct Parameters
  {
    std::string can_interface = "can0";
    bool use_motor_feedback = false;
    double max_wheel_rad_s = 16.0;
    double max_duty_cycle = 0.8;
    double gear_ratio = 20.0;
    double ramp_rate = 0.1;
  };

  Parameters params_;
  std::vector<SparkFlexMotor> motors_;

  // Indexed in the order the joints appear in the URDF. Kept as separate
  // vectors rather than a struct-of-arrays because export_*_interfaces()
  // hands out raw pointers into them, so the storage must be stable and
  // trivially addressable.
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  /// True once on_activate has run. Guards write() against touching a bus
  /// that has been cleaned up.
  bool active_ = false;

  double get_hardware_parameter(const std::string & name, double fallback) const;
  bool get_hardware_parameter(const std::string & name, bool fallback) const;
  std::string get_hardware_parameter(const std::string & name, const std::string & fallback) const;
};

}  // namespace lunabot_hardware

#endif  // LUNABOT_HARDWARE__SPARK_FLEX_SYSTEM_HPP_
