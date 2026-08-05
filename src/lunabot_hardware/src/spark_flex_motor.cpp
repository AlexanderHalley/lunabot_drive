// Copyright 2027 Lunabot. Licensed under the MIT License.

#include "lunabot_hardware/spark_flex_motor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "SparkFlex.hpp"

namespace lunabot_hardware
{

SparkFlexMotor::SparkFlexMotor(Config config) : config_(std::move(config)) {}

SparkFlexMotor::~SparkFlexMotor() = default;
SparkFlexMotor::SparkFlexMotor(SparkFlexMotor &&) noexcept = default;
SparkFlexMotor & SparkFlexMotor::operator=(SparkFlexMotor &&) noexcept = default;

void SparkFlexMotor::connect(const std::string & can_interface)
{
  try {
    device_ = std::make_unique<SparkFlex>(can_interface, config_.can_id);
  } catch (const std::exception & e) {
    // Rethrow with the context the caller actually needs. sparkcan's own
    // message is typically an errno string that does not mention which motor
    // or which bus, which turns a one-line fix into a guessing game.
    throw std::runtime_error(
      "failed to open SparkFlex id " + std::to_string(config_.can_id) + " for joint '" +
      config_.joint_name + "' on interface '" + can_interface + "': " + e.what());
  }
}

void SparkFlexMotor::configure()
{
  if (!device_) {
    throw std::runtime_error("configure() called before connect() for " + config_.joint_name);
  }

  // Ported verbatim from the 2026 drive_node.cpp configure_motors(). These
  // four settings are the accumulated hardware knowledge of last season and
  // should not be changed casually:
  //
  //   kBrake       a coasting rover on a slope keeps going. Brake mode is
  //                also what makes the watchdog stop mean anything.
  //   kBrushless   } what the drivetrain physically is. Wrong values here
  //   kHallSensor  } produce a motor that stutters or does not turn at all.
  //   ramp_rate    a step command breaks traction, and a rover that has
  //                broken traction is a rover whose odometry is fiction.
  device_->SetIdleMode(IdleMode::kBrake);
  device_->SetMotorType(MotorType::kBrushless);
  device_->SetSensorType(SensorType::kHallSensor);
  device_->SetRampRate(config_.ramp_rate);
  device_->SetInverted(config_.invert);

  // Deliberately NOT burning to flash. Flash has finite write endurance and
  // this runs on every activation. Persist these once during bench setup, not
  // from the control loop. The 2026 code carried the same warning.
}

void SparkFlexMotor::set_velocity(double wheel_rad_s)
{
  if (!device_) {
    return;
  }

  // rad/s -> duty cycle. This is a scale factor, not a control loop: the
  // SparkFlex is running in open-loop duty mode, so the relationship between
  // duty and actual speed depends on load. max_wheel_rad_s is the speed at
  // full duty under no load, which means the rover runs slower than commanded
  // under load and there is nothing here to notice or correct for that.
  //
  // Closing this loop needs encoders. See the header's open-loop note.
  const double normalised = wheel_rad_s / config_.max_wheel_rad_s;
  const double clamped = std::clamp(normalised, -config_.max_duty_cycle, config_.max_duty_cycle);

  // Inversion is applied by the controller itself via SetInverted() in
  // configure(), so the duty written here is in the joint's own frame. Do not
  // negate again -- doing both is a full season of "why does it only turn one
  // way".
  applied_duty_ = clamped;
  device_->SetDutyCycle(applied_duty_);
}

void SparkFlexMotor::stop()
{
  applied_duty_ = 0.0;
  if (device_) {
    device_->SetDutyCycle(0.0);
  }
}

void SparkFlexMotor::heartbeat()
{
  if (device_) {
    device_->Heartbeat();
  }
}

double SparkFlexMotor::read_velocity()
{
  // ==================== NOT WIRED UP ====================
  // The 2026 drivetrain had no encoders and the 2026 code never read from the
  // bus, so no sparkcan getter has ever been exercised by this team. Guessing
  // at a method name here would produce either a compile error or, worse,
  // plausible-looking numbers in the wrong units.
  //
  // SparkFlexSystem refuses to activate with use_motor_feedback:=true while
  // this returns NaN, so the failure is loud rather than silent odometry.
  //
  // To wire it up:
  //   1. Confirm the getter and its units against your sparkcan build.
  //      Likely candidates: GetVelocity(), GetAppliedOutput(), or a
  //      periodic-status accessor. Units are usually motor RPM, not rad/s.
  //   2. Replace the return below with something like:
  //
  //        const double motor_rpm = device_->GetVelocity();
  //        return motor_rpm * 2.0 * M_PI / 60.0 / config_.gear_ratio;
  //
  //   3. Delete the guard in SparkFlexSystem::on_activate.
  //   4. Verify the sign against SetInverted -- if the controller reports
  //      pre-inversion velocity, the right side reads backwards and the
  //      rover will appear to drive in circles.
  //   5. Set use_motor_feedback:=true in the ros2_control xacro.
  // ======================================================
  return std::numeric_limits<double>::quiet_NaN();
}

}  // namespace lunabot_hardware
