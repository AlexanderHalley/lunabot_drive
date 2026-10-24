// Copyright 2027 Lunabot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "lunabot_navigation/unseen_shadow.hpp"

using lunabot_navigation::kLethal;
using lunabot_navigation::kUnknown;
using lunabot_navigation::markUnseenShadows;

namespace
{

// '#' lethal, '?' unknown, '.' free. Row 0 is the first string.
struct Grid
{
  explicit Grid(const std::vector<std::string> & rows)
  : w(static_cast<int>(rows[0].size())), h(static_cast<int>(rows.size()))
  {
    for (const auto & row : rows) {
      for (const char c : row) {
        cells.push_back(c == '#' ? kLethal : c == '?' ? kUnknown : 0);
      }
    }
  }
  std::size_t mark(double depth) { return markUnseenShadows(cells.data(), w, 0, 0, w, h, depth); }
  std::string row(int j) const
  {
    std::string out;
    for (int i = 0; i < w; ++i) {
      const auto v = cells[j * w + i];
      out += v == kLethal ? '#' : v == kUnknown ? '?' : '.';
    }
    return out;
  }
  int w, h;
  std::vector<std::uint8_t> cells;
};

}  // namespace

// A rock seen from the left: its face is marked, its back is unknown.
TEST(UnseenShadow, FillsTheUnseenBackOfARockUpToTheDepth)
{
  Grid g({
    "....??????",
    "....#?????",
    "....#?????",
    "....??????",
  });
  g.mark(3.0);
  EXPECT_EQ(g.row(1), "....####??");
  EXPECT_EQ(g.row(2), "....####??");
  EXPECT_EQ(g.row(0), "....###???");  // diagonal reach, still within 3 cells
}

TEST(UnseenShadow, NeverMarksSeenFreeSpace)
{
  Grid g({
    "..........",
    "....#.....",
    "..........",
  });
  EXPECT_EQ(g.mark(5.0), 0u);
  EXPECT_EQ(g.row(1), "....#.....");
}

// Seen-free cells stop the spread: unknown space on the far side of a free
// gap is not reached, however close it is.
TEST(UnseenShadow, SeenFreeSpaceStopsTheShadow)
{
  Grid g({
    "#.?",
  });
  EXPECT_EQ(g.mark(5.0), 0u);
  EXPECT_EQ(g.row(0), "#.?");
}

// Unknown space far from any obstacle stays unknown: goals out there still plan.
TEST(UnseenShadow, FarUnknownStaysUnknown)
{
  Grid g({
    "#?????????",
  });
  g.mark(2.0);
  EXPECT_EQ(g.row(0), "###???????");
}

TEST(UnseenShadow, OnlyTheWindowIsTouched)
{
  Grid g({
    "#????",
    "?????",
  });
  // Window excludes column 0, so its lethal cell is not a source.
  EXPECT_EQ(markUnseenShadows(g.cells.data(), g.w, 1, 0, g.w, g.h, 3.0), 0u);
}
