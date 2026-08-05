// Copyright 2027 Lunabot. Licensed under the MIT License.

#ifndef LUNABOT_HARDWARE__SPARK_FLEX_MOTOR_HPP_
#define LUNABOT_HARDWARE__SPARK_FLEX_MOTOR_HPP_

#include <memory>
#include <string>

#include "lunabot_hardware/visibility_control.h"

// Forward-declared so this header does not drag SparkFlex.hpp into every
// translation unit that includes it. That matters more than usual here:
// sparkcan is built from source (see lunabot.repos) and is the one dependency
// most likely to be missing on a given machine, so the blast radius of its
// header should stay as small as possible.
class SparkFlex;

namespace lunabot_hardware
{

/// One SparkFlex motor controller on a SocketCAN bus.
///
/// Wraps the sparkcan API so that everything version-sensitive about that
/// library lives behind one class. The 2026 drive node called sparkcan
/// directly from its ROS callbacks, which meant an API change upstream
/// touched the control logic.
///
/// Owns the underlying device. Not copyable; move-only.
class SparkFlexMotor
{
public:
  /// Configuration read from the URDF's <ros2_control> block.
  struct Config
  {
    std::string joint_name;
    int can_id = 0;

    /// True for the right-hand side. On the 2026 rover this was hardcoded in
    /// C++ as right_front_->SetInverted(true), so rewiring the robot required
    /// recompiling it. It is now a per-joint URDF parameter.
    bool invert = false;

    /// Wheel rad/s at full duty cycle. Sets the scale of the entire velocity
    /// command path: a command of max_wheel_rad_s maps to duty 1.0 before
    /// clamping.
    double max_wheel_rad_s = 16.0;

    /// Duty ceiling, [0, 1]. 0.8 on the 2026 robot -- headroom for the
    /// controller and mercy for the gearboxes.
    double max_duty_cycle = 0.8;

    /// Motor shaft revolutions per wheel revolution. Telemetry is reported at
    /// the motor, commands are given at the wheel.
    double gear_ratio = 20.0;

    /// Seconds from zero to full output. Carried over from the 2026 node's
    /// SetRampRate(0.1); it is what stopped the rover shock-loading its
    /// gearboxes on a step command.
    double ramp_rate = 0.1;
  };

  LUNABOT_HARDWARE_PUBLIC explicit SparkFlexMotor(Config config);
  LUNABOT_HARDWARE_PUBLIC ~SparkFlexMotor();

  SparkFlexMotor(const SparkFlexMotor &) = delete;
  SparkFlexMotor & operator=(const SparkFlexMotor &) = delete;
  LUNABOT_HARDWARE_PUBLIC SparkFlexMotor(SparkFlexMotor &&) noexcept;
  LUNABOT_HARDWARE_PUBLIC SparkFlexMotor & operator=(SparkFlexMotor &&) noexcept;

  /// Open the CAN device. Throws std::runtime_error on failure.
  ///
  /// Separate from the constructor so on_configure() can report a clean
  /// CallbackReturn::ERROR instead of throwing out of a constructor the way
  /// the 2026 node did -- that produced a stack trace with no indication that
  /// the real problem was a CAN interface that was down.
  LUNABOT_HARDWARE_PUBLIC void connect(const std::string & can_interface);

  /// Apply idle/motor/sensor mode, ramp rate and inversion.
  ///
  /// Deliberately does NOT burn the settings to flash. Flash has a finite
  /// write endurance and this runs on every activation; the 2026 code carried
  /// the same warning.
  LUNABOT_HARDWARE_PUBLIC void configure();

  /// Command a wheel velocity in rad/s. Converts to duty cycle, applies
  /// inversion, and clamps to max_duty_cycle.
  LUNABOT_HARDWARE_PUBLIC void set_velocity(double wheel_rad_s);

  /// Zero the output. Brake mode stays engaged.
  LUNABOT_HARDWARE_PUBLIC void stop();

  /// Keep-alive frame. The controllers fault out without one roughly every
  /// 50 ms, so this must be called from write() on every control cycle.
  LUNABOT_HARDWARE_PUBLIC void heartbeat();

  /// Measured wheel velocity in rad/s, or NaN when telemetry is unavailable.
  ///
  /// VERIFY the underlying sparkcan getter before trusting this. The 2026
  /// robot never read from the bus at all, so no getter in this library has
  /// been exercised against real hardware by this team.
  LUNABOT_HARDWARE_PUBLIC double read_velocity();

  /// Duty cycle last written, after inversion and clamping. This is what
  /// actually went on the wire.
  LUNABOT_HARDWARE_PUBLIC double applied_duty_cycle() const { return applied_duty_; }

  LUNABOT_HARDWARE_PUBLIC const Config & config() const { return config_; }
  LUNABOT_HARDWARE_PUBLIC bool connected() const { return device_ != nullptr; }

private:
  Config config_;
  std::unique_ptr<SparkFlex> device_;
  double applied_duty_ = 0.0;
};

}  // namespace lunabot_hardware

#endif  // LUNABOT_HARDWARE__SPARK_FLEX_MOTOR_HPP_
