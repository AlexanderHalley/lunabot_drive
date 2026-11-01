#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Mission control's pieces agree with each other.

The Foxglove layout, the bridge's field profile, mission.py's services and
topics, twist_mux's e-stop lock and the recording preset each name the same
topics by string, in five files. A rename in one leaves a panel that stays
empty, or a button the bridge refuses, with no error anywhere -- so the
names are cross-checked here. Plus the operator camera's encoding.

Pure Python: no ROS graph, no Foxglove.
"""

import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
CONFIG = PKG / 'config'
LAYOUT = json.loads((CONFIG / 'foxglove' / 'mission_control.json').read_text())
PANELS = LAYOUT['configById']


def _params(path):
    return yaml.safe_load(path.read_text())['/**']['ros__parameters']


FIELD = _params(CONFIG / 'foxglove' / 'field.yaml')
RECORDED = set(yaml.safe_load((CONFIG / 'record_topics.yaml').read_text())['topics'])
TWIST_MUX = yaml.safe_load((CONFIG / 'twist_mux.yaml').read_text())['twist_mux']['ros__parameters']


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module  # dataclasses look the module up by name
    spec.loader.exec_module(module)
    return module


mission = _load('mission', PKG.parent / 'lunabot_navigation' / 'scripts' / 'mission.py')
camera = _load('operator_camera', PKG / 'scripts' / 'operator_camera.py')
e_stop = _load('operator_e_stop', PKG / 'scripts' / 'operator_e_stop.py')


def _kind(panel_id):
    return panel_id.split('!')[0]


def _topic(path):
    """'/mission/mode.data' -> '/mission/mode' (a message path's topic)."""
    return re.match(r'(/[\w/]+)', path).group(1)


def _subscribed_topics():
    topics = set()
    for panel_id, config in PANELS.items():
        kind = _kind(panel_id)
        if kind == '3D':
            topics |= {t for t, s in config['topics'].items() if s.get('visible')}
            topics.add('/tf')
        elif kind == 'Plot':
            topics |= {_topic(p['value']) for p in config['paths']}
        elif kind == 'Indicator':
            topics.add(_topic(config['path']))
        elif kind == 'RawMessages':
            topics.add(_topic(config['topicPath']))
        elif kind == 'Image':
            topics.add(config['imageMode']['imageTopic'])
        elif kind == 'DiagnosticSummary':
            topics.add(config['topicToRender'])
    return topics


def _allowed(name, patterns):
    return any(re.search(p, name) for p in patterns)


def test_the_layout_tree_holds_exactly_the_configured_panels():
    def leaves(node):
        if isinstance(node, str):
            return [node]
        return leaves(node['first']) + leaves(node['second'])

    assert sorted(leaves(LAYOUT['layout'])) == sorted(PANELS)


def test_every_panel_topic_gets_through_the_field_profile():
    blocked = sorted(t for t in _subscribed_topics() if not _allowed(t, FIELD['topic_whitelist']))
    assert not blocked, f'field.yaml would leave these panels empty: {blocked}'


def test_every_panel_topic_is_recorded():
    """Load the layout on a bag of the run and every panel has data."""
    missing = sorted(_subscribed_topics() - RECORDED - {'/robot_description'} - {'/tf'})
    assert not missing, f'record_topics.yaml misses {missing}'
    assert {'/tf', '/tf_static', '/robot_description'} <= RECORDED


def _buttons(service):
    return [
        c for i, c in PANELS.items() if _kind(i) == 'CallService' and c['serviceName'] == service
    ]


def test_the_buttons_are_services_the_field_profile_allows():
    services = {c['serviceName'] for i, c in PANELS.items() if _kind(i) == 'CallService'}
    assert services == {f'/mission/{name}' for name in mission.COMMANDS} | {e_stop.SERVICE}
    for service in services:
        assert _allowed(service, FIELD['service_whitelist']), service
    assert 'services' in FIELD['capabilities']
    assert not [i for i in PANELS if _kind(i) == 'Publish'], 'the field profile cannot publish'


def test_the_e_stop_buttons_reach_twist_mux_and_the_mission():
    buttons = _buttons(e_stop.SERVICE)
    assert sorted(json.loads(b['requestPayload'])['data'] for b in buttons) == [False, True]
    lock = TWIST_MUX['locks']['e_stop']
    assert e_stop.TOPIC == mission.E_STOP_TOPIC == '/' + lock['topic']
    assert PANELS['Indicator!estop']['path'] == e_stop.TOPIC + '.data'


def test_the_field_profile_lets_the_laptop_publish_nothing():
    """No clientPublish at all: bridge 3.x does not enforce client_topic_whitelist.

    Measured 2026-10-31 (bridge 3.6.0): with clientPublish on and the whitelist
    ['^/e_stop$'], a client's /cmd_vel_joy messages all reached ROS. Without
    the capability the bridge refuses the advertisement.
    """
    assert 'clientPublish' not in FIELD['capabilities']
    assert 'parameters' not in FIELD['capabilities']
    assert not _allowed('/lifecycle_manager_navigation/manage_nodes', FIELD['service_whitelist'])


def test_the_field_profile_carries_no_raw_camera_or_cloud():
    for heavy in ('/oak_d/rgb/image_raw', '/oak_d/points', '/local_costmap/costmap'):
        assert not _allowed(heavy, FIELD['topic_whitelist']), heavy


def test_the_e_stop_outranks_every_velocity_source_teleop_included():
    """A latching lock (timeout 0): one message holds it until released."""
    lock = TWIST_MUX['locks']['e_stop']
    assert lock['timeout'] == 0.0
    assert all(lock['priority'] > t['priority'] for t in TWIST_MUX['topics'].values())


def test_the_mode_indicator_has_a_colour_for_every_mode():
    rules = PANELS['Indicator!mode']['rules']
    modes = {'READY', 'RUNNING', 'PAUSED', 'E-STOP', 'ABORTED', 'FAILED', 'COMPLETE'}
    assert {r['rawValue'] for r in rules} == modes
    assert PANELS['Indicator!mode']['path'] == mission.MODE_TOPIC + '.data'
    assert PANELS['RawMessages!state']['topicPath'] == mission.STATE_TOPIC + '.data'


def test_the_layout_shows_the_operator_camera():
    assert PANELS['Image!camera']['imageMode']['imageTopic'] == camera.OUTPUT
    assert camera.OUTPUT in RECORDED


def test_no_raw_images_are_recorded():
    assert not [t for t in RECORDED if t.endswith(('image_raw', 'image_rect', '/points'))]


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8', 'rgba8', 'bgra8'])
def test_the_operator_camera_keeps_colours_in_bgr_order(encoding):
    np = pytest.importorskip('numpy')
    channels = 4 if encoding.endswith('a8') else 3
    red = {'rgb8': [255, 0, 0], 'bgr8': [0, 0, 255], 'rgba8': [255, 0, 0, 255]}
    red['bgra8'] = [0, 0, 255, 255]
    width, height, step = 8, 4, 8 * channels + 4  # a padded row, as some drivers send
    rows = np.zeros((height, step), np.uint8)
    rows[:, : width * channels] = np.tile(red[encoding], width)
    bgr = camera.to_bgr(rows.tobytes(), encoding, width, height, step)
    assert bgr.shape == (height, width, 3)
    assert bgr[0, 0].tolist() == [0, 0, 255]


def test_the_operator_camera_shrinks_to_its_width_and_stays_small():
    np = pytest.importorskip('numpy')
    cv2 = pytest.importorskip('cv2')
    rng = np.random.default_rng(0)
    frame = rng.integers(0, 255, (400, 640, 3), np.uint8)  # noise: JPEG's worst case
    jpeg = camera.shrink_and_encode(frame, 424, 50)
    decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (265, 424, 3)
    assert len(jpeg) < 60_000
    small = frame[:100, :200]
    assert cv2.imdecode(
        np.frombuffer(camera.shrink_and_encode(small, 424, 50), np.uint8), cv2.IMREAD_COLOR
    ).shape == (100, 200, 3)  # never enlarged
