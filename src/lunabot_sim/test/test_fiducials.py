# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The wall AprilTags: right patterns, drawn the right way round, in the right place."""

import math

import numpy as np
import pytest

from lunabot_sim.scene import fiducials
from lunabot_sim.scene.layout import UCF


def _cv2_aruco():
    cv2 = pytest.importorskip('cv2')
    if not hasattr(cv2, 'aruco'):
        pytest.skip('OpenCV without aruco')
    return cv2


def _image_from_runs(tag_id, cell=24):
    """Rebuild the tag the way arena.py does -- white square, black runs -- as pixels."""
    m = fiducials.MARGIN_CELLS
    n = fiducials.CELLS + 2 * m
    img = np.full((n * cell, n * cell), 255, np.uint8)
    for row, col, length in fiducials.black_runs(tag_id):
        y, x = (row + m) * cell, (col + m) * cell
        img[y : y + cell, x : x + length * cell] = 0
    # A wall darker than the tag's white, at close range (big cells).
    return np.pad(img, 3 * cell, constant_values=130)


@pytest.mark.parametrize('tag_id', sorted(fiducials.TAG36H11))
def test_patterns_match_opencv(tag_id):
    cv2 = _cv2_aruco()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    drawn = cv2.aruco.drawMarker(dictionary, tag_id, fiducials.CELLS, borderBits=1)
    ours = np.array([[255 if b == '1' else 0 for b in row] for row in fiducials.TAG36H11[tag_id]])
    assert (drawn == ours).all()


@pytest.mark.parametrize('tag_id', sorted(fiducials.TAG36H11))
def test_drawn_runs_detect_as_the_right_tag_upright(tag_id):
    cv2 = _cv2_aruco()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    corners, ids, _ = cv2.aruco.detectMarkers(_image_from_runs(tag_id), dictionary)
    assert ids is not None and ids.ravel().tolist() == [tag_id]
    tl, tr, br, bl = corners[0].reshape(4, 2)
    assert tl[0] < tr[0] and tl[1] < bl[1]  # not mirrored, not rotated


def test_tag_axes_are_right_handed_and_face_out_of_the_wall():
    for tag in UCF.fiducials:
        x, y, z = (np.array(a) for a in tag.axes())
        assert np.allclose(np.cross(x, y), z)
        # Facing into the arena: a point 1 m in front of the tag is on the floor.
        assert UCF.floor.contains(tag.x + z[0], tag.y + z[1])


def test_ucf_tags_one_on_each_wall_of_the_start_corner():
    by_id = {t.tag_id: t for t in UCF.fiducials}
    assert sorted(by_id) == [0, 1]
    assert by_id[0].x == UCF.floor.x_min and by_id[0].facing == 0.0
    assert by_id[1].y == UCF.floor.y_min and by_id[1].facing == pytest.approx(math.pi / 2)
    for tag in UCF.fiducials:
        margin = tag.size / 2 + fiducials.MARGIN_CELLS * tag.size / fiducials.CELLS
        assert UCF.start_zone.contains(tag.x, tag.y)
        assert tag.z - margin > 0.0
        for post in UCF.posts:  # not hidden behind or inside a corner post
            assert not post.footprint().contains(tag.x, tag.y, margin=margin)


def test_corners_are_in_opencv_order():
    tag = fiducials.Fiducial(0, x=0.0, y=0.0, z=0.5, facing=0.0, size=0.2)
    tl, tr, br, bl = tag.corners()
    # Facing +X, the viewer looks along -X: their right is +Y.
    assert tr[1] > tl[1] and tl[2] > bl[2]
    assert br == pytest.approx((0.0, 0.1, 0.4))


def test_start_area_is_inside_the_start_zone_and_clear_of_walls():
    area = UCF.start_area()
    assert area.inside(UCF.start_zone)
    assert area.x_min - UCF.floor.x_min >= 0.63 and area.y_min - UCF.floor.y_min >= 0.63
    assert area.size[0] > 0.5 and area.size[1] > 0.5
