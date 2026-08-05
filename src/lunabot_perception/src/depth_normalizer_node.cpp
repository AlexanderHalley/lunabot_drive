// Copyright 2027 Lunabot. Licensed under the MIT License.
//
// Converts depth images from 16UC1 millimetres to 32FC1 metres.
//
// ==================== WHY THIS EXISTS ====================
// The same logical topic carries different encodings depending on where the
// data comes from:
//
//   OAK-D S2 via depthai      16UC1, millimetres
//   Isaac Sim depth annotator 32FC1, metres
//
// rtabmap happens to accept both, which is exactly what makes this dangerous:
// everything works until something we wrote reads the depth image directly
// and is silently off by a factor of a thousand. A 1000x error in depth does
// not look like a bug, it looks like a calibration problem.
//
// Running this on the real robot means everything downstream sees ONE
// encoding. It is not needed in sim and should not be started there.
//
// Note that the boulder detector sidesteps the issue entirely by consuming
// the point cloud, which is metres in both cases. This node is for anything
// that genuinely wants the depth image.
// =========================================================

#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace lunabot_perception
{

class DepthNormalizerNode : public rclcpp::Node
{
public:
  explicit DepthNormalizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("depth_normalizer", options)
  {
    publisher_ = create_publisher<sensor_msgs::msg::Image>("depth_out", rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "depth_in", rclcpp::SensorDataQoS(),
      std::bind(&DepthNormalizerNode::on_image, this, std::placeholders::_1));
  }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    // Already metres. Pass it through untouched rather than erroring, so the
    // same launch configuration works in sim and on hardware and nobody has
    // to remember to disable this node.
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      publisher_->publish(*msg);
      return;
    }

    if (msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "unhandled depth encoding '%s'; expected 16UC1 or 32FC1",
        msg->encoding.c_str());
      return;
    }

    sensor_msgs::msg::Image out;
    out.header = msg->header;
    out.height = msg->height;
    out.width = msg->width;
    out.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    out.is_bigendian = msg->is_bigendian;
    out.step = msg->width * sizeof(float);
    out.data.resize(static_cast<std::size_t>(out.step) * out.height);

    auto * dst = reinterpret_cast<float *>(out.data.data());

    for (uint32_t row = 0; row < msg->height; ++row) {
      // Read row by row using the source step. Depth images are frequently
      // padded, so assuming step == width * 2 gives a sheared image.
      const auto * src = reinterpret_cast<const uint16_t *>(
        msg->data.data() + static_cast<std::size_t>(row) * msg->step);

      for (uint32_t col = 0; col < msg->width; ++col) {
        const uint16_t millimetres = src[col];
        // 0 means "no return" in 16UC1 depth, not "zero distance". The 32FC1
        // convention for that is NaN. Mapping it to 0.0 would place a
        // phantom obstacle at the camera's optical centre.
        dst[static_cast<std::size_t>(row) * msg->width + col] =
          millimetres == 0 ? std::numeric_limits<float>::quiet_NaN()
                           : static_cast<float>(millimetres) * 0.001f;
      }
    }

    publisher_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

}  // namespace lunabot_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lunabot_perception::DepthNormalizerNode>());
  rclcpp::shutdown();
  return 0;
}
