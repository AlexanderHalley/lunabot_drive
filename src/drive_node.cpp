// ~/ros2_ws/src/lunabot_drive/src/drive_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
        declare_parameter("wheel_base", 0.5);  // meters
        declare_parameter("max_duty_cycle", 0.8);

        // Odometry parameters
        declare_parameter("wheel_radius", 0.075);  // meters
        declare_parameter("encoder_cpr", 42);       // hall sensor counts per revolution (NEO Vortex)
        declare_parameter("gear_ratio", 1.0);
        declare_parameter("odom_rate", 30.0);       // Hz
        declare_parameter("odom_frame_id", "odom");
        declare_parameter("odom_child_frame_id", "base_link");

        // Get parameters
        can_interface_ = get_parameter("can_interface").as_string();
        wheel_base_ = get_parameter("wheel_base").as_double();
        max_duty_ = get_parameter("max_duty_cycle").as_double();
        wheel_radius_ = get_parameter("wheel_radius").as_double();
        encoder_cpr_ = get_parameter("encoder_cpr").as_int();
        gear_ratio_ = get_parameter("gear_ratio").as_double();
        double odom_rate = get_parameter("odom_rate").as_double();
        odom_frame_id_ = get_parameter("odom_frame_id").as_string();
        odom_child_frame_id_ = get_parameter("odom_child_frame_id").as_string();

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

        // Odometry publisher
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);

        // Heartbeat timer (motors need regular heartbeat)
        heartbeat_timer_ = create_wall_timer(
            50ms, std::bind(&DriveNode::heartbeat_callback, this));

        // Watchdog timer (stop if no commands received)
        watchdog_timer_ = create_wall_timer(
            100ms, std::bind(&DriveNode::watchdog_callback, this));

        // Odometry timer
        auto odom_period = std::chrono::duration<double>(1.0 / odom_rate);
        odom_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(odom_period),
            std::bind(&DriveNode::odom_callback, this));

        RCLCPP_INFO(get_logger(), "Drive node started with wheel odometry at %.1f Hz", odom_rate);
    }

    ~DriveNode()
    {
        stop_motors();
    }

private:
    void configure_motors()
    {
        // Compute conversion factors so GetPosition() returns meters
        // and GetVelocity() returns m/s
        double pos_factor = (1.0 / gear_ratio_) * 2.0 * M_PI * wheel_radius_;
        double vel_factor = pos_factor / 60.0;  // RPM -> m/s

        for (auto* motor : {left_front_.get(), right_front_.get(),
                           left_rear_.get(), right_rear_.get()}) {
            motor->SetIdleMode(IdleMode::kBrake);
            motor->SetMotorType(MotorType::kBrushless);
            motor->SetSensorType(SensorType::kHallSensor);
            motor->SetRampRate(0.1);
            motor->SetPositionConversionFactor(pos_factor);
            motor->SetVelocityConversionFactor(vel_factor);
            motor->SetPeriodicStatus1Period(20);
            motor->SetPeriodicStatus2Period(20);
        }

        // Invert right side motors for tank drive
        right_front_->SetInverted(true);
        right_rear_->SetInverted(true);
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

    void odom_callback()
    {
        // Read encoder positions from front motors (meters, after conversion factor)
        double left_pos = left_front_->GetPosition();
        double right_pos = right_front_->GetPosition();

        if (!odom_initialized_) {
            prev_left_pos_ = left_pos;
            prev_right_pos_ = right_pos;
            odom_initialized_ = true;
            return;
        }

        // Compute distance deltas
        double dl = left_pos - prev_left_pos_;
        double dr = right_pos - prev_right_pos_;
        prev_left_pos_ = left_pos;
        prev_right_pos_ = right_pos;

        // Midpoint integration (2nd-order accurate)
        double d_center = (dl + dr) / 2.0;
        double d_theta = (dr - dl) / wheel_base_;

        // Integrate using midpoint angle for better accuracy
        double mid_theta = theta_ + d_theta / 2.0;
        x_ += d_center * std::cos(mid_theta);
        y_ += d_center * std::sin(mid_theta);
        theta_ += d_theta;

        // Normalize theta to [-pi, pi]
        theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

        // Read velocities from front motors (m/s, after conversion factor)
        double left_vel = left_front_->GetVelocity();
        double right_vel = right_front_->GetVelocity();
        double linear_vel = (left_vel + right_vel) / 2.0;
        double angular_vel = (right_vel - left_vel) / wheel_base_;

        // Build odometry message
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now();
        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = odom_child_frame_id_;

        // Pose
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        // Pose covariance [x, y, z, roll, pitch, yaw] - 6x6 row-major
        // Low confidence for x/y, high for unmeasured DOFs, moderate for yaw
        odom_msg.pose.covariance[0]  = 0.01;   // x
        odom_msg.pose.covariance[7]  = 0.01;   // y
        odom_msg.pose.covariance[14] = 1e6;    // z (unmeasured)
        odom_msg.pose.covariance[21] = 1e6;    // roll (unmeasured)
        odom_msg.pose.covariance[28] = 1e6;    // pitch (unmeasured)
        odom_msg.pose.covariance[35] = 0.05;   // yaw

        // Twist (in child frame = base_link)
        odom_msg.twist.twist.linear.x = linear_vel;
        odom_msg.twist.twist.angular.z = angular_vel;

        // Twist covariance
        odom_msg.twist.covariance[0]  = 0.01;   // vx
        odom_msg.twist.covariance[7]  = 0.01;   // vy
        odom_msg.twist.covariance[14] = 1e6;    // vz (unmeasured)
        odom_msg.twist.covariance[21] = 1e6;    // roll rate (unmeasured)
        odom_msg.twist.covariance[28] = 1e6;    // pitch rate (unmeasured)
        odom_msg.twist.covariance[35] = 0.05;   // yaw rate

        odom_pub_->publish(odom_msg);
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
    double max_duty_;
    double wheel_radius_;
    int encoder_cpr_;
    double gear_ratio_;
    std::string odom_frame_id_;
    std::string odom_child_frame_id_;

    // Drive state
    double target_left_ = 0.0;
    double target_right_ = 0.0;
    rclcpp::Time last_cmd_time_;

    // Odometry state
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    double prev_left_pos_ = 0.0;
    double prev_right_pos_ = 0.0;
    bool odom_initialized_ = false;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveNode>());
    rclcpp::shutdown();
    return 0;
}
