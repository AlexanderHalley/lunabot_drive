#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The excavator core on the model backend: sequences, limits, cancel, e-stop, faults.

Pure Python: the node imports rclpy only inside main().
"""

import importlib.util
import math
import sys
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('excavator', PKG / 'scripts' / 'excavator.py')
ex = importlib.util.module_from_spec(spec)
sys.modules['excavator'] = ex  # dataclasses look the module up by name
spec.loader.exec_module(ex)

CONFIG = yaml.safe_load((PKG / 'config' / 'excavator.yaml').read_text())['excavator'][
    'ros__parameters'
]
MSG_DIR = PKG.parent / 'lunabot_msgs'


def run(core, seconds, dt=0.05, until=None):
    """Tick for `seconds`, or until `until(core)`; return the phases seen, in order."""
    phases = []
    for _ in range(round(seconds / dt)):
        core.tick(dt)
        phase = core.phase()
        if phase and (not phases or phases[-1] != phase):
            phases.append(phase)
        if until and until(core):
            break
    return phases


def model_core(**model):
    return ex.Excavator(ex.ModelBackend(**model), move_timeout=CONFIG['move_timeout'])


def test_a_dig_lowers_digs_for_its_time_raises_and_ends_stowed():
    core = model_core()
    job = core.start('excavate', max_seconds=5.0)
    assert core.state() == 'EXCAVATING'
    phases = run(core, 30.0, until=lambda c: c.job.done)
    assert phases == ['LOWERING', 'DIGGING', 'RAISING']  # RETURNING passed at once: flat
    assert job.outcome == ex.TIME_LIMIT
    assert job.seconds == pytest.approx(5.0, abs=0.06)
    assert core.state() == 'STOWED'
    assert core.backend.fill == pytest.approx(5.0 * 0.12, abs=0.02)  # carried, not spilled


def test_with_a_measured_fill_the_dig_ends_when_full():
    core = model_core(fill_measured=True)
    job = core.start('excavate', max_seconds=60.0, target_fill=0.6)
    run(core, 30.0, until=lambda c: c.job.done)
    assert job.outcome == ex.FULL
    assert job.seconds == pytest.approx(0.6 / 0.12, abs=0.1)
    assert core.state() == 'STOWED'


def test_without_a_measured_fill_the_target_is_ignored():
    core = model_core()  # the model's fill is an estimate
    job = core.start('excavate', max_seconds=12.0, target_fill=0.3)
    run(core, 30.0, until=lambda c: c.job.done)
    assert job.outcome == ex.TIME_LIMIT and job.seconds == pytest.approx(12.0, abs=0.06)


def test_a_dump_tips_pours_and_comes_back_flat():
    core = model_core(fill_measured=True)
    core.backend.fill = 1.0
    job = core.start('deposit', max_seconds=10.0)
    phases = run(core, 30.0, until=lambda c: c.job.done)
    assert phases == ['TIPPING', 'POURING', 'RETURNING']
    assert job.outcome == ex.EMPTY and core.backend.fill == 0.0
    assert core.state() == 'STOWED'


def test_a_dump_with_the_drum_down_raises_it_first():
    core = model_core()
    core.backend.arm = 1.0
    core.start('deposit', max_seconds=1.0)
    assert run(core, 30.0, until=lambda c: c.job.done)[0] == 'RAISING'
    assert core.state() == 'STOWED'


def test_cancel_holds_where_it_is_and_a_new_goal_carries_on():
    core = model_core()
    job = core.start('excavate', max_seconds=6.0)
    run(core, 5.0)  # lowered (2.4 s) and digging
    core.cancel()
    assert job.outcome == ex.CANCELED
    arm, spent = core.backend.arm, job.seconds
    run(core, 3.0)
    assert core.backend.arm == arm and core.backend.drum == 0.0
    assert core.state() == 'HOLDING'  # not safe to drive
    rest = core.start('excavate', max_seconds=6.0 - spent)
    phases = run(core, 30.0, until=lambda c: c.job.done)
    assert phases[0] == 'DIGGING'  # already down: no lowering again
    assert spent + rest.seconds == pytest.approx(6.0, abs=0.1)


def test_the_e_stop_holds_ends_the_goal_and_refuses_new_ones_until_released():
    core = model_core()
    job = core.start('excavate', max_seconds=8.0)
    run(core, 1.0)  # mid-lowering
    core.set_e_stop(True)
    assert job.outcome == ex.E_STOP
    arm = core.backend.arm
    run(core, 3.0)
    assert core.backend.arm == arm and 0.0 < arm < 1.0
    assert core.start('excavate', max_seconds=1.0) is None
    assert core.stow() is None
    core.set_e_stop(False)
    run(core, 2.0)
    assert core.backend.arm == arm  # release does not move anything by itself
    assert core.start('excavate', max_seconds=1.0) is not None


def test_one_job_at_a_time_but_stow_preempts():
    core = model_core()
    dig = core.start('excavate', max_seconds=8.0)
    assert core.start('deposit', max_seconds=1.0) is None
    run(core, 4.0)
    stow = core.stow()
    assert dig.outcome == ex.CANCELED and stow is not None
    run(core, 10.0, until=lambda c: c.job.done)
    assert stow.done and core.state() == 'STOWED'


def test_a_stroke_that_never_arrives_is_a_fault_and_holds():
    core = model_core()
    core.backend.stuck = 'arm'
    job = core.start('excavate', max_seconds=5.0)
    run(core, CONFIG['move_timeout'] + 2.0, until=lambda c: c.job.done)
    assert job.outcome == ex.FAULT and 'LOWERING' in job.message
    assert core.state() == 'FAULT'
    core.backend.stuck = ''
    assert core.stow() is not None  # stow clears it
    run(core, 10.0, until=lambda c: c.job.done)
    assert core.state() == 'STOWED'


def test_a_dig_with_no_time_left_just_stows():
    core = model_core()
    core.backend.arm = 0.6  # cancelled on the way up, say
    job = core.start('excavate', max_seconds=0.0)
    phases = run(core, 20.0, until=lambda c: c.job.done)
    assert phases == ['RAISING']  # not lowered again to dig nothing
    assert job.outcome == ex.TIME_LIMIT and job.seconds == 0.0
    assert core.state() == 'STOWED'


def test_the_states_match_the_message_constants():
    text = (MSG_DIR / 'msg' / 'ExcavatorStatus.msg').read_text()
    for i, name in enumerate(ex.STATES):
        assert f'uint8 {name} = {i}' in text


@pytest.mark.parametrize(
    ('action', 'outcomes'),
    [
        ('Excavate', (ex.FULL, ex.TIME_LIMIT, ex.CANCELED, ex.E_STOP, ex.FAULT)),
        ('Deposit', (ex.EMPTY, ex.TIME_LIMIT, ex.CANCELED, ex.E_STOP, ex.FAULT)),
    ],
)
def test_every_outcome_is_an_action_result_constant(action, outcomes):
    text = (MSG_DIR / 'action' / f'{action}.action').read_text()
    for name in outcomes:
        assert f'uint8 {name} = ' in text


def test_the_config_builds_the_model_with_every_parameter():
    model = CONFIG['model']
    assert set(model) == set(ex.MODEL_PARAMS)
    backend = ex.make_backend(CONFIG['backend'], model)
    assert backend.name == 'model'
    assert math.isclose(backend.arm_rate, 1.0 / model['arm_seconds'])
    with pytest.raises(ValueError):
        ex.make_backend('spark', {})
