// Copyright 2027 Lunabot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "lunabot_navigation/unseen_shadow_layer.hpp"

#include <algorithm>
#include <stdexcept>

#include "lunabot_navigation/unseen_shadow.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace lunabot_navigation
{

static_assert(kLethal == nav2_costmap_2d::LETHAL_OBSTACLE, "lethal cost changed");
static_assert(kUnknown == nav2_costmap_2d::NO_INFORMATION, "unknown cost changed");

void UnseenShadowLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("shadow_depth", rclcpp::ParameterValue(0.45));
  const auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("UnseenShadowLayer: failed to lock node");
  }
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".shadow_depth", shadow_depth_);
  if (!layered_costmap_->isTrackingUnknown()) {
    // Without unknown cells there are no shadows to find: say so, once.
    RCLCPP_WARN(
      logger_, "%s: track_unknown_space is false, so this layer does nothing", name_.c_str());
  }
  RCLCPP_INFO(logger_, "%s: shadow depth %.2f m", name_.c_str(), shadow_depth_);
  current_ = true;
}

void UnseenShadowLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  // A change anywhere in the window can move a shadow up to shadow_depth
  // beyond it, and a source just outside it can cast one in.
  *min_x -= shadow_depth_;
  *min_y -= shadow_depth_;
  *max_x += shadow_depth_;
  *max_y += shadow_depth_;
}

void UnseenShadowLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  min_i = std::max(min_i, 0);
  min_j = std::max(min_j, 0);
  max_i = std::min(max_i, static_cast<int>(master_grid.getSizeInCellsX()));
  max_j = std::min(max_j, static_cast<int>(master_grid.getSizeInCellsY()));
  markUnseenShadows(
    master_grid.getCharMap(), static_cast<int>(master_grid.getSizeInCellsX()), min_i, min_j, max_i,
    max_j, shadow_depth_ / master_grid.getResolution());
}

}  // namespace lunabot_navigation

PLUGINLIB_EXPORT_CLASS(lunabot_navigation::UnseenShadowLayer, nav2_costmap_2d::Layer)
