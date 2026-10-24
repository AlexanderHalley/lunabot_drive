# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""AprilTags on the arena walls, for finding the rover's start pose.

Isaac-free, like layout.py: the tag geometry and bit patterns are checked in
ordinary CI (test_fiducials.py checks the patterns against OpenCV's own
tag36h11 dictionary). arena.py draws them out of thin boxes, one per run of
black cells, so no texture files are needed.

A tag's pose follows OpenCV's marker convention, which is what the detector
(lunabot_navigation fiducial_localizer.py) estimates: origin at the tag's
centre, +Z out of the wall towards the viewer, +Y up, +X to the viewer's
right. `size` is the black square's side -- the length the detector needs.

The white margin is TWO cells, not the usual one: OpenCV's detector misses a
one-cell margin on a darker wall once a cell is more than ~8 px across, which
is exactly the close range of the start spin (~27 px). Print them that way.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# tag36h11, ids 0 and 1, as OpenCV draws them with a one-cell black border
# (cv2.aruco.drawMarker(DICT_APRILTAG_36h11, id, 8, borderBits=1)).
# '1' = white. Row 0 is the top.
TAG36H11 = {
    0: (
        '00000000',
        '00010000',
        '00110100',
        '00001010',
        '00001100',
        '01011100',
        '01010110',
        '00000000',
    ),
    1: (
        '00000000',
        '01001000',
        '01011010',
        '00001100',
        '00011110',
        '01110100',
        '00110110',
        '00000000',
    ),
}
CELLS = 8  # including the black border
MARGIN_CELLS = 2  # white, all round; see above


@dataclass(frozen=True)
class Fiducial:
    """One tag on a wall, in the arena (world) frame."""

    tag_id: int
    x: float
    y: float
    z: float
    """Centre of the tag, metres. x, y on the wall's inside face."""

    facing: float
    """Direction the tag faces (its +Z), radians from +X in the ground plane."""

    size: float = 0.16
    """Side of the black square, metres."""

    def axes(self):
        """(X, Y, Z) unit vectors of the tag frame, in the arena frame."""
        c, s = math.cos(self.facing), math.sin(self.facing)
        return (-s, c, 0.0), (0.0, 0.0, 1.0), (c, s, 0.0)

    def corners(self):
        """Return the black square's corners in the arena frame.

        In OpenCV's order: top-left, top-right, bottom-right, bottom-left, as
        the viewer sees it.
        """
        (xx, xy, _), _, _ = self.axes()
        h = self.size / 2.0
        out = []
        for u, v in ((-h, h), (h, h), (h, -h), (-h, -h)):
            out.append((self.x + u * xx, self.y + u * xy, self.z + v))
        return tuple(out)


def black_runs(tag_id: int):
    """Black cells merged into horizontal runs: (row, first_col, length).

    Row 0 is the top, col 0 the viewer's left. One box per run keeps the
    prim count down (tag 0: 22 boxes rather than 52).
    """
    runs = []
    for r, row in enumerate(TAG36H11[tag_id]):
        c = 0
        while c < CELLS:
            if row[c] == '0':
                start = c
                while c < CELLS and row[c] == '0':
                    c += 1
                runs.append((r, start, c - start))
            else:
                c += 1
    return runs
