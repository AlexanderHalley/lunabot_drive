#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""excavation.launch.py's arguments are its own.

Launch arguments are shared down every include: robot.launch.py's scope held
slam.launch.py's `backend:=rtabmap`, this file's `backend` argument (choices:
model) read it, and the whole stack failed at start in Isaac. Pure text: no
launch import, so it runs anywhere.
"""

import re
from pathlib import Path

SRC = Path(__file__).resolve().parents[2]
OURS = SRC / 'lunabot_excavation' / 'launch' / 'excavation.launch.py'
DECLARED = re.compile(r"DeclareLaunchArgument\(\s*'([^']+)'")


def _arguments(path):
    return set(DECLARED.findall(path.read_text()))


def test_no_other_launch_file_declares_an_excavation_argument():
    ours = _arguments(OURS) - {'use_sim_time'}
    assert ours == {'excavator_backend'}
    for path in SRC.glob('*/launch/*.launch.py'):
        if path != OURS:
            assert not ours & _arguments(path), (
                f'{path.name} also declares {ours & _arguments(path)}'
            )
