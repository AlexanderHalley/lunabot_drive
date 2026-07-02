// SparkFlex ros2_control SystemInterface (SKELETON).
//
// This is the Phase-3 home of the 2026 drive_node.cpp CAN logic. The lifecycle
// contract is wired here; the bodies are TODOs so it compiles-by-intent and
// documents exactly what to port. Do NOT put drivetrain kinematics here — that
// lives in diff_drive_controller now.
#include "lunabot_hardware/sparkflex_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lunabot_hardware
{
using CallbackReturn = hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn SparkFlexSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  can_interface_ = info_.hardware_parameters.count("can_interface")
                     ? info_.hardware_parameters.at("can_interface") : "can0";
  // TODO: parse "left_ids"/"right_ids" CSV params into left_ids_/right_ids_.

  const auto n = info_.joints.size();
  hw_commands_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_positions_.assign(n, 0.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SparkFlexSystem::on_configure(const rclcpp_lifecycle::State &)
{
  // TODO: port configure_motors() from drive_node.cpp:
  //   construct SparkFlex per id on can_interface_, set IdleMode::kBrake,
  //   kBrushless, kHallSensor, ramp rate, and invert the right side.
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SparkFlexSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface> SparkFlexSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return ifaces;
}

CallbackReturn SparkFlexSystem::on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn SparkFlexSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // TODO: port stop_motors() — SetDutyCycle(0) on all motors.
  return CallbackReturn::SUCCESS;
}

return_type SparkFlexSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO: read hall-sensor velocity/position from each SparkFlex ->
  //       hw_velocities_[i], integrate/accumulate hw_positions_[i].
  return return_type::OK;
}

return_type SparkFlexSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO: map hw_commands_[i] (rad/s) -> SetDutyCycle/velocity per motor,
  //       and call Heartbeat() on every motor (was the 50ms timer in 2026).
  return return_type::OK;
}

}  // namespace lunabot_hardware

PLUGINLIB_EXPORT_CLASS(lunabot_hardware::SparkFlexSystem, hardware_interface::SystemInterface)
