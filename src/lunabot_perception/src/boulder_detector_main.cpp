// Copyright 2027 Lunabot. Licensed under the MIT License.
//
// Standalone entry point. The node is also registered as a component, so it
// can be loaded into a container alongside the camera driver to avoid
// serialising point clouds between processes -- worth doing on the Pi once
// the pipeline is real, and not worth the indirection while it is a stub.

#include <memory>

#include "lunabot_perception/boulder_detector_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lunabot_perception::BoulderDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
