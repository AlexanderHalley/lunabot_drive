// ~/ros2_ws/src/lunabot_drive/src/drive_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// TODO: For odometry, add these includes:
// #include <nav_msgs/msg/odometry.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
#include "SparkFlex.hpp"
#include <memory>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class DriveNode : public rclcpp::Node
{
public:
    DriveNode() : Node("drive_node")
    {
        // Declare parameters
        declare_parameter("can_interface", "can0");
        declare_parameter("left_front_id", 1);
        declare_parameter("right_front_id", 2);
        declare_parameter("left_rear_id", 3);
        declare_parameter("right_rear_id", 4);
        declare_parameter("wheel_base", 0.762);    // meters (distance between left/right wheels)
        declare_parameter("wheel_radius", 0.1778); // meters (7 inches, from URDF)
        declare_parameter("gear_ratio", 100.0);      // 5x5x4 gearboxes
        declare_parameter("max_duty_cycle", 0.8);
        declare_parameter("joint_state_rate", 50.0); // Hz

        // Get parameters
        can_interface_ = get_parameter("can_interface").as_string();
        wheel_base_ = get_parameter("wheel_base").as_double();
        wheel_radius_ = get_parameter("wheel_radius").as_double();
        gear_ratio_ = get_parameter("gear_ratio").as_double();
        max_duty_ = get_parameter("max_duty_cycle").as_double();
        double joint_state_rate = get_parameter("joint_state_rate").as_double();

        int lf_id = get_parameter("left_front_id").as_int();
        int rf_id = get_parameter("right_front_id").as_int();
        int lr_id = get_parameter("left_rear_id").as_int();
        int rr_id = get_parameter("right_rear_id").as_int();

        // Initialize motors
        try {
            left_front_ = std::make_unique<SparkFlex>(can_interface_, lf_id);
            right_front_ = std::make_unique<SparkFlex>(can_interface_, rf_id);
            left_rear_ = std::make_unique<SparkFlex>(can_interface_, lr_id);
            right_rear_ = std::make_unique<SparkFlex>(can_interface_, rr_id);

            configure_motors();
            RCLCPP_INFO(get_logger(), "Motors initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize motors: %s", e.what());
            throw;
        }

        // Subscribe to cmd_vel
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&DriveNode::cmd_vel_callback, this, std::placeholders::_1));

        // Publisher for joint states (wheel positions/velocities for visualization and odometry)
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // TODO: For odometry, add these publishers:
        // odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // Initialize odometry state variables:
        // x_ = 0.0; y_ = 0.0; theta_ = 0.0;
        // last_left_pos_ = 0.0; last_right_pos_ = 0.0;
        // For sensor fusion with IMU, subscribe to the OAK-D IMU:
        // imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("oak/imu/data", 10, ...);
        // Use an Extended Kalman Filter (robot_localization package) to fuse wheel odometry with IMU

        // Heartbeat timer (motors need regular heartbeat)
        heartbeat_timer_ = create_wall_timer(
            50ms, std::bind(&DriveNode::heartbeat_callback, this));

        // Joint state publisher timer
        auto joint_state_period = std::chrono::duration<double>(1.0 / joint_state_rate);
        joint_state_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(joint_state_period),
            std::bind(&DriveNode::publish_joint_states, this));

        // Watchdog timer (stop if no commands received)
        watchdog_timer_ = create_wall_timer(
            100ms, std::bind(&DriveNode::watchdog_callback, this));

        RCLCPP_INFO(get_logger(), "Drive node started");
    }

    ~DriveNode()
    {
        stop_motors();
    }

private:
    void configure_motors()
    {
        for (auto* motor : {left_front_.get(), right_front_.get(),
                           left_rear_.get(), right_rear_.get()}) {
            motor->SetIdleMode(IdleMode::kBrake);
            motor->SetMotorType(MotorType::kBrushless);
            motor->SetSensorType(SensorType::kHallSensor);
            motor->SetRampRate(0.1);
            // Don't burn flash every startup - do this once during setup
        }

        // Invert right side motors for tank drive
        right_front_->SetInverted(false);  // Motor 1
        right_rear_->SetInverted(false);   // Motor 4
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_time_ = now();

        // Differential drive calculation
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        // Convert to left/right wheel speeds
        double left_speed = linear - (angular * wheel_base_ / 2.0);
        double right_speed = linear + (angular * wheel_base_ / 2.0);

        // Normalize and clamp to max duty cycle
        double max_speed = std::max(std::abs(left_speed), std::abs(right_speed));
        if (max_speed > max_duty_) {
            left_speed = (left_speed / max_speed) * max_duty_;
            right_speed = (right_speed / max_speed) * max_duty_;
        }

        target_left_ = std::clamp(left_speed, -max_duty_, max_duty_);
        target_right_ = std::clamp(right_speed, -max_duty_, max_duty_);

        // Apply to motors
        left_front_->SetDutyCycle(target_left_);
        left_rear_->SetDutyCycle(target_left_);
        right_front_->SetDutyCycle(target_right_);
        right_rear_->SetDutyCycle(target_right_);
    }

    void heartbeat_callback()
    {
        left_front_->Heartbeat();
        right_front_->Heartbeat();
        left_rear_->Heartbeat();
        right_rear_->Heartbeat();
    }

    void watchdog_callback()
    {
        // Stop motors if no command received for 500ms
        auto elapsed = now() - last_cmd_time_;
        if (elapsed.seconds() > 0.5) {
            stop_motors();
        }
    }

    void publish_joint_states()
    {
        // Read encoder feedback from each motor
        // GetPosition() returns motor rotations, GetVelocity() returns RPM
        // Convert to wheel radians and rad/s for ROS joint states

        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now();

        // Joint names must match URDF joint names exactly
        msg.name = {
            "left_front_wheel_joint",
            "right_front_wheel_joint",
            "left_wheel_joint",    // rear left in URDF
            "right_wheel_joint"    // rear right in URDF
        };

        // Convert motor rotations to wheel radians
        // wheel_radians = motor_rotations * (2*pi) / gear_ratio
        double motor_to_wheel_rad = (2.0 * M_PI) / gear_ratio_;

        // Convert motor RPM to wheel rad/s
        // wheel_rad_s = motor_rpm * (2*pi/60) / gear_ratio
        double rpm_to_rad_s = (2.0 * M_PI / 60.0) / gear_ratio_;

        // Read positions (motor rotations -> wheel radians)
        double lf_pos = left_front_->GetPosition() * motor_to_wheel_rad;
        double rf_pos = right_front_->GetPosition() * motor_to_wheel_rad;
        double lr_pos = left_rear_->GetPosition() * motor_to_wheel_rad;
        double rr_pos = right_rear_->GetPosition() * motor_to_wheel_rad;

        // Read velocities (motor RPM -> wheel rad/s)
        double lf_vel = left_front_->GetVelocity() * rpm_to_rad_s;
        double rf_vel = right_front_->GetVelocity() * rpm_to_rad_s;
        double lr_vel = left_rear_->GetVelocity() * rpm_to_rad_s;
        double rr_vel = right_rear_->GetVelocity() * rpm_to_rad_s;

        msg.position = {lf_pos, rf_pos, lr_pos, rr_pos};
        msg.velocity = {lf_vel, rf_vel, lr_vel, rr_vel};
        msg.effort = {};  // Not measuring torque

        joint_state_pub_->publish(msg);

        // TODO: For odometry, add differential drive kinematics here:
        // 1. Compute wheel displacement since last update:
        //    double left_delta = (lf_pos + lr_pos) / 2.0 - last_left_pos_;
        //    double right_delta = (rf_pos + rr_pos) / 2.0 - last_right_pos_;
        // 2. Convert to linear/angular displacement:
        //    double linear = wheel_radius_ * (left_delta + right_delta) / 2.0;
        //    double angular = wheel_radius_ * (right_delta - left_delta) / wheel_base_;
        // 3. Update pose:
        //    theta_ += angular;
        //    x_ += linear * cos(theta_);
        //    y_ += linear * sin(theta_);
        // 4. Publish odom message and tf (odom -> base_link)
        // 5. For better accuracy, fuse with OAK-D IMU using robot_localization EKF
    }

    void stop_motors()
    {
        target_left_ = 0.0;
        target_right_ = 0.0;
        left_front_->SetDutyCycle(0.0);
        left_rear_->SetDutyCycle(0.0);
        right_front_->SetDutyCycle(0.0);
        right_rear_->SetDutyCycle(0.0);
    }

    // Motors
    std::unique_ptr<SparkFlex> left_front_;
    std::unique_ptr<SparkFlex> right_front_;
    std::unique_ptr<SparkFlex> left_rear_;
    std::unique_ptr<SparkFlex> right_rear_;

    // Parameters
    std::string can_interface_;
    double wheel_base_;
    double wheel_radius_;
    double gear_ratio_;
    double max_duty_;

    // State
    double target_left_ = 0.0;
    double target_right_ = 0.0;
    rclcpp::Time last_cmd_time_;

    // TODO: For odometry, add pose state:
    // double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    // double last_left_pos_ = 0.0, last_right_pos_ = 0.0;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    // TODO: For odometry:
    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveNode>());
    rclcpp::shutdown();
    return 0;
}
