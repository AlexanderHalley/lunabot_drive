// ~/ros2_ws/src/lunabot_drive/src/drive_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "SparkFlex.hpp"
#include <memory>
#include <chrono>

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

        // Get parameters
        can_interface_ = get_parameter("can_interface").as_string();
        wheel_base_ = get_parameter("wheel_base").as_double();
        max_duty_ = get_parameter("max_duty_cycle").as_double();

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

        // Heartbeat timer (motors need regular heartbeat)
        heartbeat_timer_ = create_wall_timer(
            50ms, std::bind(&DriveNode::heartbeat_callback, this));

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

    // State
    double target_left_ = 0.0;
    double target_right_ = 0.0;
    rclcpp::Time last_cmd_time_;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveNode>());
    rclcpp::shutdown();
    return 0;
}
