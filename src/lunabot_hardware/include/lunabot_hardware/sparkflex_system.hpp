// SparkFlex ros2_control SystemInterface (SKELETON).
// Replaces the 2026 drive_node.cpp; see README.md for the migration map.
#ifndef LUNABOT_HARDWARE__SPARKFLEX_SYSTEM_HPP_
#define LUNABOT_HARDWARE__SPARKFLEX_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

// #include "SparkFlex.hpp"  // from sparkcan, as in 2026 drive_node.cpp

namespace lunabot_hardware
{

class SparkFlexSystem : public hardware_interface::SystemInterface
{
public:
  // Parse joints/params from the <ros2_control> URDF tag.
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // Open CAN, construct + configure SparkFlex motors (was configure_motors()).
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;  // stop motors

  // read(): pull hall-sensor feedback -> hw_positions_/hw_velocities_.
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // write(): send hw_commands_ to motors + heartbeat every cycle.
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string can_interface_;
  std::vector<int> left_ids_;
  std::vector<int> right_ids_;

  // Per-joint command/state buffers (ros2_control reads/writes these by ref).
  std::vector<double> hw_commands_;   // velocity command per wheel joint
  std::vector<double> hw_velocities_; // measured
  std::vector<double> hw_positions_;  // measured

  // std::vector<std::unique_ptr<SparkFlex>> motors_;  // TODO
};

}  // namespace lunabot_hardware

#endif  // LUNABOT_HARDWARE__SPARKFLEX_SYSTEM_HPP_
