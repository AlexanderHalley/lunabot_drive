// Copyright 2027 Lunabot. Licensed under the MIT License.
//
// Loads SparkFlexSystem through pluginlib against a synthetic URDF and
// exercises on_init.
//
// No SocketCAN and no motors, so this runs in CI. That matters: on_init is
// where every "the robot does not move and there is no error" bug in the
// hardware layer originates, and it is the only part of the real hardware
// path that can be covered automatically.
//
// Note what is NOT covered: on_configure opens the bus, so anything past
// on_init needs vcan0. See docs/HARDWARE_CAN.md for the candump procedure.

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "test_assets.hpp"

using lunabot_hardware_test::kDuplicateCanIds;
using lunabot_hardware_test::kMissingCanId;
using lunabot_hardware_test::kMissingPositionState;
using lunabot_hardware_test::kValidSystem;
using lunabot_hardware_test::kWrongCommandInterface;

namespace
{

std::string wrap(const std::string & ros2_control_block)
{
  return ros2_control_test_assets::urdf_head + ros2_control_block +
         ros2_control_test_assets::urdf_tail;
}

}  // namespace

class SparkFlexSystemTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // ResourceManager has no default constructor in Jazzy: it wants a clock
    // and a logger, which is why this is built in SetUp from a node rather
    // than being a plain member. Getting that wrong presents as gtest
    // complaining that the FIXTURE's constructor is deleted, which points at
    // the test rather than at the thing that changed.
    node_ = std::make_shared<rclcpp::Node>("test_spark_flex_system");
    rm_ = std::make_unique<hardware_interface::ResourceManager>(
      node_->get_node_clock_interface(), node_->get_node_logging_interface());
  }

  rclcpp::Node::SharedPtr node_;

  // load_urdf(..., validate_interfaces=false) so the resource manager parses
  // and instantiates the component without requiring an activated lifecycle.
  std::unique_ptr<hardware_interface::ResourceManager> rm_;
};

TEST_F(SparkFlexSystemTest, plugin_loads_from_a_valid_description)
{
  EXPECT_NO_THROW(rm_->load_urdf(wrap(kValidSystem), false));
  EXPECT_TRUE(rm_->is_urdf_already_loaded());
}

TEST_F(SparkFlexSystemTest, exports_the_interfaces_diff_drive_controller_claims)
{
  ASSERT_NO_THROW(rm_->load_urdf(wrap(kValidSystem), false));

  // Four wheels: one velocity command each, position and velocity state each.
  for (const auto & joint : {
         "front_left_wheel_joint",
         "front_right_wheel_joint",
         "rear_left_wheel_joint",
         "rear_right_wheel_joint",
       }) {
    EXPECT_TRUE(rm_->command_interface_exists(std::string(joint) + "/velocity")) << joint;
    EXPECT_TRUE(rm_->state_interface_exists(std::string(joint) + "/position")) << joint;
    EXPECT_TRUE(rm_->state_interface_exists(std::string(joint) + "/velocity")) << joint;
  }

  EXPECT_EQ(rm_->command_interface_keys().size(), 4u);
  EXPECT_EQ(rm_->state_interface_keys().size(), 8u);
}

TEST_F(SparkFlexSystemTest, does_not_export_a_position_command)
{
  ASSERT_NO_THROW(rm_->load_urdf(wrap(kValidSystem), false));

  // A position command interface would let someone activate a position
  // controller against a drivetrain that physically cannot hold a position.
  EXPECT_FALSE(rm_->command_interface_exists("front_left_wheel_joint/position"));
}

TEST_F(SparkFlexSystemTest, rejects_duplicate_can_ids)
{
  // Two motors on one address presents as one dead wheel on the real robot.
  // Catching it at parse time turns an afternoon of wiring inspection into
  // one log line.
  EXPECT_THROW(rm_->load_urdf(wrap(kDuplicateCanIds), false), std::exception);
}

TEST_F(SparkFlexSystemTest, rejects_a_joint_without_a_can_id)
{
  // Defaulting to 0 would address whichever controller sits at id 0.
  EXPECT_THROW(rm_->load_urdf(wrap(kMissingCanId), false), std::exception);
}

TEST_F(SparkFlexSystemTest, rejects_a_non_velocity_command_interface)
{
  EXPECT_THROW(rm_->load_urdf(wrap(kWrongCommandInterface), false), std::exception);
}

TEST_F(SparkFlexSystemTest, rejects_a_joint_without_position_state)
{
  // Without position state the controller silently falls back to integrating
  // velocity, so odometry degrades with no error anywhere.
  EXPECT_THROW(rm_->load_urdf(wrap(kMissingPositionState), false), std::exception);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  // Required before the fixture constructs its node.
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
