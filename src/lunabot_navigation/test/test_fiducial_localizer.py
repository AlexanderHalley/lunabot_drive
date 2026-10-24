#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The start fix: tags rendered into synthetic images, found, and solved.

Pure Python + OpenCV; the node imports rclpy only inside main().
"""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

cv2 = pytest.importorskip('cv2')
if not hasattr(cv2, 'aruco'):
    pytest.skip('OpenCV without aruco', allow_module_level=True)

PKG = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('fiducial_localizer', PKG / 'scripts' /
                                              'fiducial_localizer.py')
loc = importlib.util.module_from_spec(spec)
sys.modules['fiducial_localizer'] = loc
spec.loader.exec_module(loc)

PARAMS = yaml.safe_load((PKG / 'config' / 'fiducials_ucf.yaml').read_text())
PARAMS = PARAMS['fiducial_localizer']['ros__parameters']
IDS = PARAMS['tag_ids']
SIZE = PARAMS['tag_size']
TAGS = {i: loc.tag_in_arena(*PARAMS['tag_poses'][4 * n : 4 * n + 4]) for n, i in enumerate(IDS)}

W, H = 640, 400
K = np.array([[466.0, 0, 320], [0, 466.0, 200], [0, 0, 1]])
D = np.zeros(5)
# base_link -> optical frame: 0.4065 ahead, 0.10 up, 15 deg down (lunabot.urdf.xacro);
# base_link 0.15 m above the ground.
PITCH = math.radians(15)


def camera_in_base():
    body = np.eye(4)  # camera body frame: x fwd, z up, pitched down
    c, s = math.cos(PITCH), math.sin(PITCH)
    body[:3, :3] = ((c, 0, s), (0, 1, 0), (-s, 0, c))
    body[:3, 3] = (0.4065, 0.0, 0.10)
    optical = np.eye(4)
    optical[:3, :3] = ((0, 0, 1), (-1, 0, 0), (0, -1, 0))  # z fwd, x right, y down
    return body @ optical


def render(t_arena_cam):
    """The scene's tags as the camera would see them: a grey wall, white tags."""
    img = np.full((H, W), 120, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for tag_id, t_arena_tag in TAGS.items():
        cells = 8 + 2 * 2
        px = 20
        marker = np.full((cells * px, cells * px), 255, np.uint8)
        marker[2 * px : 10 * px, 2 * px : 10 * px] = cv2.aruco.drawMarker(d, tag_id, 8 * px)
        h = SIZE / 2 * cells / 8
        square = np.array([(-h, h, 0), (h, h, 0), (h, -h, 0), (-h, -h, 0), (0, 0, 0)], float)
        t_cam_tag = np.linalg.inv(t_arena_cam) @ t_arena_tag
        pts = (t_cam_tag[:3, :3] @ square.T).T + t_cam_tag[:3, 3]
        if (pts[:, 2] < 0.05).any():
            continue
        uv = (K @ pts.T).T
        uv = uv[:, :2] / uv[:, 2:]
        src = np.array([(0, 0), (cells * px, 0), (cells * px, cells * px), (0, cells * px)],
                       np.float32)
        warp = cv2.getPerspectiveTransform(src, uv[:4].astype(np.float32))
        patch = cv2.warpPerspective(marker, warp, (W, H), flags=cv2.INTER_LINEAR,
                                    borderValue=0)
        mask = cv2.warpPerspective(np.full_like(marker, 255), warp, (W, H),
                                   flags=cv2.INTER_NEAREST, borderValue=0)
        img[mask > 0] = patch[mask > 0]
    return img


def survey(start, steps=36, noise=0.0, rng=None):
    """Spin on the spot at `start` (arena x, y, yaw) and collect map-frame sightings."""
    t_arena_map = loc.planar_matrix(*start)
    cam = camera_in_base()
    observations = []
    for k in range(steps):
        t_map_base = loc.planar_matrix(0.0, 0.0, 2 * math.pi * k / steps)
        t_map_base[2, 3] = 0.15
        t_map_cam = t_map_base @ cam
        img = render(t_arena_map @ t_map_cam)
        if noise and rng is not None:
            img = np.clip(img + rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8)
        for tag_id, t_cam_tag, _, _ in loc.detect(img, K, D, SIZE, IDS):
            observations.append((tag_id, t_map_cam @ t_cam_tag))
    return observations


@pytest.mark.parametrize(
    'start',
    [(0.0, 0.0, 0.0), (0.3, 0.2, math.radians(130)), (0.6, -0.3, math.radians(-75)),
     (-0.1, 0.4, math.radians(200))],
)
def test_a_full_spin_recovers_the_start_pose(start):
    obs = survey(start, noise=4.0, rng=np.random.default_rng(1))
    assert {i for i, _ in obs} == set(IDS), 'both tags should be seen during a full turn'
    x, y, yaw, method, _ = loc.solve(obs, TAGS)
    assert method == 'tag_centres'
    assert math.hypot(x - start[0], y - start[1]) < 0.03
    assert abs(math.degrees(math.remainder(yaw - start[2], math.tau))) < 1.0


def test_one_tag_still_gives_a_fix():
    obs = [o for o in survey((0.2, 0.1, 0.5)) if o[0] == IDS[0]]
    x, y, yaw, method, _ = loc.solve(obs, TAGS)
    assert method == 'single_tag'
    assert math.hypot(x - 0.2, y - 0.1) < 0.05
    assert abs(math.degrees(math.remainder(yaw - 0.5, math.tau))) < 3.0


def test_no_sightings_no_fix():
    assert loc.solve([], TAGS) is None


def test_other_tag_ids_are_ignored():
    img = render(loc.planar_matrix(0, 0, math.pi) @ camera_in_base())
    assert loc.detect(img, K, D, SIZE, ids=[7]) == []


def test_config_matches_the_sim_arena():
    fiducials = _sim_layout().UCF.fiducials
    assert sorted(t.tag_id for t in fiducials) == sorted(IDS)
    for tag in fiducials:
        n = IDS.index(tag.tag_id)
        assert PARAMS['tag_poses'][4 * n : 4 * n + 4] == pytest.approx(
            [tag.x, tag.y, tag.z, tag.facing], abs=1e-6)
        assert tag.size == pytest.approx(SIZE)


def _sim_layout():
    """lunabot_sim's arena layout, from the installed package or this repo's source.

    It is Isaac-free, but not installed into the ROS environment colcon tests in,
    and skipping would defeat the cross-check.
    """
    sys.path.append(str(PKG.parent / 'lunabot_sim'))
    return pytest.importorskip('lunabot_sim.scene.layout')
