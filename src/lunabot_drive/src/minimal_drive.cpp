// minimal_drive.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class MinimalDrive : public rclcpp::Node
{
public:
    MinimalDrive() : Node("minimal_drive")
    {
        RCLCPP_INFO(get_logger(), "Starting minimal drive...");
        
        // Subscribe to joy
        sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            [this](sensor_msgs::msg::Joy::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "JOY: axes[0]=%.2f axes[1]=%.2f buttons[4]=%d", 
                           msg->axes[0], msg->axes[1], msg->buttons[4]);
            });
        
        RCLCPP_INFO(get_logger(), "Subscribed to /joy");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalDrive>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
