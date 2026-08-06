// Copyright 2027 Lunabot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
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
#include "test_assets.hpp"

using lunabot_hardware_test::kDuplicateCanIds;
using lunabot_hardware_test::kMissingCanId;
using lunabot_hardware_test::kMissingPositionState;
using lunabot_hardware_test::kValidSystem;
using lunabot_hardware_test::kWrongCommandInterface;

namespace
{

// The URDF the block is wrapped in has to declare the same joints the block
// names: ResourceManager cross-checks them and throws "Joint '...' not found
// in URDF" before the plugin is ever loaded. See test_assets.hpp.
std::string wrap(const std::string & ros2_control_block)
{
  return lunabot_hardware_test::kUrdfHead + ros2_control_block + lunabot_hardware_test::kUrdfTail;
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

  // load_and_initialize_components parses the description and runs on_init on
  // each component, which is exactly the part of the real hardware path that
  // can be covered without a CAN bus. It REPORTS failure rather than throwing
  // it -- see the note on the rejects_* tests below.
  bool load(const std::string & ros2_control_block)
  {
    return rm_->load_and_initialize_components(wrap(ros2_control_block));
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<hardware_interface::ResourceManager> rm_;
};

TEST_F(SparkFlexSystemTest, plugin_loads_from_a_valid_description)
{
  EXPECT_TRUE(load(kValidSystem));
  EXPECT_TRUE(rm_->are_components_initialized());
}

TEST_F(SparkFlexSystemTest, exports_the_interfaces_diff_drive_controller_claims)
{
  ASSERT_TRUE(load(kValidSystem));

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
  ASSERT_TRUE(load(kValidSystem));

  // A position command interface would let someone activate a position
  // controller against a drivetrain that physically cannot hold a position.
  EXPECT_FALSE(rm_->command_interface_exists("front_left_wheel_joint/position"));
}

// The four rejects_* tests below assert on a RETURN VALUE, not on an
// exception. load_and_initialize_components catches whatever on_init raises
// and reports false, so EXPECT_THROW here would fail even though the
// component correctly refused the description. What is being pinned is that
// each bad description is refused at all -- every one of them is a fault that
// otherwise presents as a wheel that does not turn, with nothing in the log.

TEST_F(SparkFlexSystemTest, rejects_duplicate_can_ids)
{
  // Two motors on one address presents as one dead wheel on the real robot.
  // Catching it at parse time turns an afternoon of wiring inspection into
  // one log line.
  EXPECT_FALSE(load(kDuplicateCanIds));
}

TEST_F(SparkFlexSystemTest, rejects_a_joint_without_a_can_id)
{
  // Defaulting to 0 would address whichever controller sits at id 0.
  EXPECT_FALSE(load(kMissingCanId));
}

TEST_F(SparkFlexSystemTest, rejects_a_non_velocity_command_interface)
{
  EXPECT_FALSE(load(kWrongCommandInterface));
}

TEST_F(SparkFlexSystemTest, rejects_a_joint_without_position_state)
{
  // Without position state the controller silently falls back to integrating
  // velocity, so odometry degrades with no error anywhere.
  EXPECT_FALSE(load(kMissingPositionState));
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
