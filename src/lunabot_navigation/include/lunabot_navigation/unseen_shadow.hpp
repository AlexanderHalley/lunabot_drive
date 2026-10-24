// Copyright 2027 Lunabot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef LUNABOT_NAVIGATION__UNSEEN_SHADOW_HPP_
#define LUNABOT_NAVIGATION__UNSEEN_SHADOW_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

namespace lunabot_navigation
{

constexpr std::uint8_t kLethal = 254;   // nav2_costmap_2d::LETHAL_OBSTACLE
constexpr std::uint8_t kUnknown = 255;  // nav2_costmap_2d::NO_INFORMATION

/// Mark the unseen space behind seen obstacle surfaces as obstacle.
///
/// A camera marks only the faces of an obstacle turned towards it. What is
/// behind a face was never seen, so it is unknown -- and a planner allowed
/// into unknown space plans straight through the back of the rock. This
/// marks LETHAL every UNKNOWN cell reachable from a LETHAL cell through
/// UNKNOWN cells only, within `max_cells` (straight-line distance from that
/// lethal cell).
///
/// It never touches a cell with any other value. In particular a cell seen
/// to be free stops the spread, so nothing is ever marked in front of a
/// seen surface (where the rover is, and where it may need to turn), and
/// unknown space far from any obstacle stays unknown, so goals beyond the
/// camera's range still plan. It does not guess shapes: how far the shadow
/// reaches is the one parameter, the largest obstacle depth expected.
///
/// Works on the window [min_i, max_i) x [min_j, max_j) of a row-major grid
/// `size_x` wide. Lethal cells outside the window are not sources. Returns
/// the number of cells marked.
inline std::size_t markUnseenShadows(
  std::uint8_t * grid, int size_x, int min_i, int min_j, int max_i, int max_j, double max_cells)
{
  if (min_i >= max_i || min_j >= max_j || max_cells <= 0.0) {
    return 0;
  }
  const int w = max_i - min_i;
  const int h = max_j - min_j;
  const double limit2 = max_cells * max_cells;

  // Closest source first, like the inflation layer: a cell belongs to the
  // NEAREST lethal cell, whichever reaches it first in steps. A plain BFS
  // lets the first-reaching source claim it and cuts the reach short.
  std::vector<int> source(static_cast<std::size_t>(w) * h, -1);
  std::vector<double> best(static_cast<std::size_t>(w) * h, limit2 + 1.0);
  using Entry = std::pair<double, int>;  // (squared distance to source, local index)
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  for (int j = 0; j < h; ++j) {
    for (int i = 0; i < w; ++i) {
      const int local = j * w + i;
      if (grid[(min_j + j) * size_x + (min_i + i)] == kLethal) {
        source[local] = local;
        best[local] = 0.0;
        queue.emplace(0.0, local);
      }
    }
  }

  std::vector<int> marked;
  static constexpr int kDi[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static constexpr int kDj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  while (!queue.empty()) {
    const auto [d2, local] = queue.top();
    queue.pop();
    if (d2 > best[local]) {
      continue;  // stale entry: a closer source got here since
    }
    const int i = local % w;
    const int j = local / w;
    const int src = source[local];
    const int si = src % w;
    const int sj = src / w;
    for (int k = 0; k < 8; ++k) {
      const int ni = i + kDi[k];
      const int nj = j + kDj[k];
      if (ni < 0 || nj < 0 || ni >= w || nj >= h) {
        continue;
      }
      const int next = nj * w + ni;
      if (grid[(min_j + nj) * size_x + (min_i + ni)] != kUnknown) {
        continue;  // seen (free or anything else): the shadow stops here
      }
      const double di = ni - si;
      const double dj = nj - sj;
      const double nd2 = di * di + dj * dj;
      if (nd2 > limit2 || nd2 >= best[next]) {
        continue;
      }
      if (source[next] == -1) {
        marked.push_back(next);
      }
      source[next] = src;
      best[next] = nd2;
      queue.emplace(nd2, next);
    }
  }
  for (const int local : marked) {
    grid[(min_j + local / w) * size_x + (min_i + local % w)] = kLethal;
  }
  return marked.size();
}

}  // namespace lunabot_navigation

#endif  // LUNABOT_NAVIGATION__UNSEEN_SHADOW_HPP_
