// Copyright 2027 Lunabot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// Synthetic URDFs for the loader test.
//
// These are hand-written rather than generated from lunabot_description on
// purpose: this test is about the PLUGIN's contract with ros2_control, and it
// should keep passing when someone changes a wheel diameter. The description
// package has its own test for whether the real URDF matches this shape
// (test_urdf_parses.py).

#ifndef TEST_ASSETS_HPP_
#define TEST_ASSETS_HPP_

#include <string>

namespace lunabot_hardware_test
{

// ==================== Why this file carries a whole URDF ====================
// It used to wrap the <ros2_control> blocks below in
// ros2_control_test_assets::urdf_head, whose robot has joints named joint1
// and joint2. ResourceManager now cross-checks every joint named in a
// <ros2_control> block against the URDF model, so all seven tests died with
//
//     C++ exception with description "Joint 'front_left_wheel_joint' not
//     found in URDF"
//
// before reaching the plugin at all. The message points at our joint name,
// which reads as a typo in the block below rather than as the wrapper
// describing a different robot.
//
// So the robot is spelled out here: four wheels on a box, the smallest thing
// the four joint names can hang off. It is still deliberately NOT generated
// from lunabot_description -- this test is about the PLUGIN's contract with
// ros2_control and must keep passing when someone changes a wheel diameter.
// ============================================================================

// Wheels are `continuous`, not `revolute`: a driven wheel has no end stop, and
// a revolute joint would hand the joint limiter a position range to clamp
// against in a test that is not about limits. Every <ros2_control> block below
// names a subset of these four joints and never anything outside them.
const auto kUrdfHead =
  R"(<?xml version="1.0" encoding="utf-8"?>
<robot name="LunabotTestRobot">
  <link name="base_link">
    <inertial>
      <mass value="25.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5"/>
    </inertial>
  </link>

  <link name="front_left_wheel_link">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>
  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="front_left_wheel_link"/>
    <origin rpy="0 0 0" xyz="0.22 0.25 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="40.0" velocity="16.0"/>
  </joint>

  <link name="front_right_wheel_link">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>
  <joint name="front_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="front_right_wheel_link"/>
    <origin rpy="0 0 0" xyz="0.22 -0.25 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="40.0" velocity="16.0"/>
  </joint>

  <link name="rear_left_wheel_link">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>
  <joint name="rear_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_left_wheel_link"/>
    <origin rpy="0 0 0" xyz="-0.22 0.25 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="40.0" velocity="16.0"/>
  </joint>

  <link name="rear_right_wheel_link">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>
  <joint name="rear_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_right_wheel_link"/>
    <origin rpy="0 0 0" xyz="-0.22 -0.25 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="40.0" velocity="16.0"/>
  </joint>
)";

const auto kUrdfTail = "</robot>\n";

const auto kValidSystem =
  R"(
  <ros2_control name="LunabotSystem" type="system">
    <hardware>
      <plugin>lunabot_hardware/SparkFlexSystem</plugin>
      <param name="can_interface">vcan0</param>
      <param name="use_motor_feedback">false</param>
      <param name="max_wheel_rad_s">16.0</param>
      <param name="max_duty_cycle">0.8</param>
      <param name="gear_ratio">20.0</param>
      <param name="ramp_rate">0.1</param>
    </hardware>
    <joint name="front_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="can_id">2</param>
      <param name="invert">false</param>
    </joint>
    <joint name="front_right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="can_id">1</param>
      <param name="invert">true</param>
    </joint>
    <joint name="rear_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="can_id">3</param>
      <param name="invert">false</param>
    </joint>
    <joint name="rear_right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="can_id">4</param>
      <param name="invert">true</param>
    </joint>
  </ros2_control>
  )";

// Two motors on one CAN address. Presents on the real robot as a single wheel
// not responding, which reads as a dead motor rather than a config error.
const auto kDuplicateCanIds =
  R"(
  <ros2_control name="LunabotSystem" type="system">
    <hardware>
      <plugin>lunabot_hardware/SparkFlexSystem</plugin>
      <param name="can_interface">vcan0</param>
    </hardware>
    <joint name="front_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="can_id">2</param>
    </joint>
    <joint name="front_right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="can_id">2</param>
    </joint>
  </ros2_control>
  )";

// A joint with no can_id. The plugin cannot guess one, and defaulting to 0
// would address whichever controller happens to be at id 0.
const auto kMissingCanId =
  R"(
  <ros2_control name="LunabotSystem" type="system">
    <hardware>
      <plugin>lunabot_hardware/SparkFlexSystem</plugin>
    </hardware>
    <joint name="front_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  )";

// Position command instead of velocity. diff_drive_controller would fail to
// claim its interfaces, and the error it produces points at the controller.
const auto kWrongCommandInterface =
  R"(
  <ros2_control name="LunabotSystem" type="system">
    <hardware>
      <plugin>lunabot_hardware/SparkFlexSystem</plugin>
    </hardware>
    <joint name="front_left_wheel_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="can_id">2</param>
    </joint>
  </ros2_control>
  )";

// No position state interface. Silently degrades odometry: the controller
// falls back to integrating velocity and drifts faster, with no error.
const auto kMissingPositionState =
  R"(
  <ros2_control name="LunabotSystem" type="system">
    <hardware>
      <plugin>lunabot_hardware/SparkFlexSystem</plugin>
    </hardware>
    <joint name="front_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <param name="can_id">2</param>
    </joint>
  </ros2_control>
  )";

}  // namespace lunabot_hardware_test

#endif  // TEST_ASSETS_HPP_
