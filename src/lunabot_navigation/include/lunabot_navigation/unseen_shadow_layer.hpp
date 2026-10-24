// Copyright 2027 Lunabot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef LUNABOT_NAVIGATION__UNSEEN_SHADOW_LAYER_HPP_
#define LUNABOT_NAVIGATION__UNSEEN_SHADOW_LAYER_HPP_

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"

namespace lunabot_navigation
{

/// Costmap layer: the unseen space behind a seen obstacle surface is obstacle.
///
/// Goes AFTER the layers that mark (static, obstacle) and BEFORE inflation,
/// in a costmap with track_unknown_space: true. See markUnseenShadows() and
/// docs/NAVIGATION.md, "Rocks behind their seen faces".
///
///   shadow_depth (m, default 0.45): how far behind a seen surface unseen
///     space counts as obstacle. The largest rock is ~0.40 m deep.
class UnseenShadowLayer : public nav2_costmap_2d::Layer
{
public:
  void reset() override {}
  bool isClearable() override { return false; }

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;

protected:
  void onInitialize() override;

private:
  double shadow_depth_{0.45};
};

}  // namespace lunabot_navigation

#endif  // LUNABOT_NAVIGATION__UNSEEN_SHADOW_LAYER_HPP_
