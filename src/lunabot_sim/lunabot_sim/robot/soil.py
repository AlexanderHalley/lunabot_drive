# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Wheel-on-sand forces, in place of PhysX friction.

Why this exists: the 2026 rover could NOT turn on the spot in sand -- not
slowly, not at all -- yet drove wide arcs without trouble. The 2027 design has
nearly the same wheelbase/track ratio (1.12 vs 1.14), so expect the same.

PhysX friction cannot produce that. Its friction is isotropic Coulomb: any
sideways slip, however small, meets the full mu*N at once. A wide arc needs a
little sideways scrub at the front and rear wheels, so in PhysX a wide arc
meets the same resistance as a pivot does. With the 2027 geometry Isaac gave
48% of the commanded yaw rate in place and 3% in an arc -- the reverse of the
field.

Sand behaves like a tyre, not like a block on a table. Sideways resistance
grows with the SLIP ANGLE (the angle between where the wheel points and where
it travels) and only saturates at large angles, where the wheel is bulldozing
sand sideways. Forward traction grows with the slip RATIO. So:

  * A wide arc has small slip angles at every wheel (about atan(L/2 / R)),
    little sideways resistance, and turns.
  * A pivot has slip angles of atan(L/T), ~48 deg for this chassis, at any
    yaw rate at all. Sideways resistance is saturated, and when that
    saturated resistance (times the wheelbase) beats the most traction the
    wheels can find (times the track), the rover does not turn. The wheels
    dig instead. That is the field behaviour.

How it is applied: the wheel colliders get a frictionless material, so PhysX
only supplies the normal force; every step these forces are applied at each
wheel's contact point. A force at the contact point also loads the wheel
joint (F*r about the axle), so the drives feel the ground as they should.

This module is numpy only, so the turning behaviour is tested in ordinary CI
(test/test_soil.py) without Isaac. robot/traction.py is the Isaac glue.

Scope: flat ground at z = 0 (terrain.py's scope) and a static load split.
Craters will need a ground height lookup and real normal loads.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SoilConfig:
    # ==================== Calibration ====================
    # Fitted so the 2027 chassis (L 0.62, T 0.554 m) cannot pivot and turns a
    # tightest arc of about 1.5 m radius. 1.5 m is an ASSUMPTION: the 2026 rover's
    # real minimum arc radius in sand has not been measured. Measure it
    # (drive the tightest arc that still turns, loaded and empty), then refit
    # alpha_0 -- test_soil.py's arc tests say what moves.

    mu_long: float = 0.6
    """Peak forward traction / normal load. Regolith simulant, ~0.5-0.7."""

    mu_lat: float = 0.85
    """Peak sideways resistance / normal load. Higher than mu_long because a
    sinking wheel driven sideways bulldozes the sand in front of its sidewall.
    The pivot locks when mu_lat * wheelbase clearly exceeds mu_long * track."""

    kappa_0: float = 0.15
    """Slip ratio at which traction reaches ~76% of its peak (tanh(1)). Sand
    needs a lot of slip to push against; rigid ground would be ~0.03."""

    alpha_0: float = 0.5
    """Slip angle, rad, at which sideways resistance reaches ~76% of its peak.
    Sets the minimum arc radius: 0.35 gives ~2.5 m, 0.5 ~1.5 m, 0.7 ~0.9 m.
    Raised too far (~1.0 with mu_lat 0.75) and the pivot starts to creep."""

    rolling_resistance: float = 0.05
    """Fraction of the normal load that resists rolling (compaction)."""

    v_eps: float = 0.003
    """m/s. Keeps the slip ratio and angle defined at standstill."""

    contact_tolerance: float = 0.02
    """m. A wheel whose centre is within radius + this of the ground is on it."""

    # ==================== Sinkage ====================
    # Sand gives. Applied in Isaac as a compliant (spring-damper) contact on
    # the wheel colliders. Without it the ground is rigid and a drop-centre
    # six-wheeler balances on three wheels, chattering across the diagonal
    # (Isaac: roll rate 0.2 rad/s average, standing still) and pivoting more
    # easily than it would in sand, where the end wheels sink in and carry
    # their share. Sized so that with the 1.5 cm drop the end wheels and the
    # middle axle split the weight about evenly: each wheel sinks ~1.5 cm.

    sinkage_stiffness: float = 560.0
    """N/m per contact POINT. PhysX makes about five per wheel on the ground
    plane (measured: 2800 here sank a wheel 1.2 cm under 167 N, ~13900 N/m),
    so ~2800 N/m per wheel. Lower: deeper sinkage, more even loads."""

    sinkage_damping: float = 36.0
    """N s/m per contact point; about critical per wheel, five points each."""


def wheel_forces(
    config: SoilConfig,
    centre: np.ndarray,
    axis: np.ndarray,
    v_centre: np.ndarray,
    omega: np.ndarray,
    radius: float,
    mass: float,
    yaw_inertia: float,
    dt: float,
    loads: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Sand forces on each wheel, world frame.

    Per-wheel arrays, shape (n, 3):
      centre      wheel centre position
      axis        unit spin axis (the joint axis, +y of the wheel link)
      v_centre    linear velocity of the wheel centre
      omega       angular velocity of the wheel body (spin included)
    mass, yaw_inertia: the whole rover's, about the middle of the wheels.

    loads, optional: each wheel's normal force, N. In Isaac these come from
    the ground contacts, which is what makes a drop-centre six-wheeler work:
    the middle axle carries more and the ends scrub less. Without them the
    weight is split evenly over the wheels near the ground.

    Returns (force, contact point): apply each force AT its point.
    """
    forward, lateral, point, slip, v_x = _kinematics(centre, axis, v_centre, omega, radius)
    surface = v_x - slip  # rim speed, omega * r

    on_ground = centre[:, 2] < radius + config.contact_tolerance
    if loads is None:
        load = np.where(on_ground, mass * _G / max(int(on_ground.sum()), 1), 0.0)
    else:
        load = np.where(on_ground, np.maximum(np.asarray(loads, dtype=float), 0.0), 0.0)
    # The mass each wheel would have to stop on its own, for the step caps.
    share = load / _G

    # ---- Along the wheel: traction and rolling resistance ----
    # Slip ratio: + driving (rim faster than travel), - braking.
    kappa = -slip / np.maximum(np.maximum(np.abs(v_x), np.abs(surface)), config.v_eps)
    f_trac = config.mu_long * load * np.tanh(kappa / config.kappa_0)
    f_roll = -config.rolling_resistance * load * np.tanh(v_x / config.v_eps)

    # ---- Across the wheel: side force from the slip angle ----
    # Evaluated at the velocity the rover WILL have once traction acts, not
    # the one it has. From standstill the slip angle is otherwise zero, side
    # force zero, and traction gets one free step of rotation every time --
    # a pivot that should be locked creeps.
    along = (f_trac + f_roll)[:, None] * forward
    v_pred = _predict(centre, v_centre, omega, along, point, mass, yaw_inertia, dt)
    v_px = np.einsum('ij,ij->i', v_pred, forward)
    v_py = np.einsum('ij,ij->i', v_pred, lateral)
    alpha = np.arctan2(v_py, np.abs(v_px) + config.v_eps)
    f_side = -config.mu_lat * load * np.tanh(alpha / config.alpha_0)

    # Combined slip: traction and side force share one friction ellipse.
    use = np.hypot(f_trac / config.mu_long, f_side / config.mu_lat) / np.maximum(load, 1e-9)
    scale = np.where(use > 1.0, 1.0 / np.maximum(use, 1e-9), 1.0)
    f_trac *= scale
    f_side *= scale

    # Never push harder than it takes to stop the motion in one step. The
    # force curves are near vertical at standstill; integrated explicitly they
    # are a very stiff spring and the rover buzzes in place.
    cap = share / dt
    f_trac = np.clip(f_trac, -cap * np.abs(slip), cap * np.abs(slip))
    f_roll = np.clip(f_roll, -cap * np.abs(v_x), cap * np.abs(v_x))

    # Side forces resist motion; they may stop it but never reverse it. Done
    # for the rover as a whole, because what they hold in a pivot is a yaw
    # moment, which no single wheel's velocity measures.
    # Judged against the FINAL traction: the ellipse has just cut it, and a
    # side force sized to cancel the uncut traction would kick the rover
    # backwards.
    along = (f_trac + f_roll)[:, None] * forward
    v_pred = _predict(centre, v_centre, omega, along, point, mass, yaw_inertia, dt)
    side = f_side[:, None] * lateral
    side *= _no_reversal(centre, v_pred, side, point, mass, yaw_inertia, dt)

    force = along + side
    return force, point


_G = 9.81


def _kinematics(centre, axis, v_centre, omega, radius):
    z = np.array([0.0, 0.0, 1.0])
    forward = np.cross(axis, z)
    forward /= np.maximum(np.linalg.norm(forward, axis=1, keepdims=True), 1e-9)
    lateral = np.cross(z, forward)

    r_contact = np.tile(-radius * z, (len(centre), 1))
    point = centre + r_contact
    v_contact = v_centre + np.cross(omega, r_contact)

    slip = np.einsum('ij,ij->i', v_contact, forward)  # contact point over ground
    v_x = np.einsum('ij,ij->i', v_centre, forward)
    return forward, lateral, point, slip, v_x


def _planar(centre, v_centre, omega):
    """Rover body state in the plane, about the middle of the wheels."""
    mid = centre[:, :2].mean(axis=0)
    arm = centre[:, :2] - mid
    yaw_rate = omega[:, 2].mean()  # the spin axes are horizontal
    v_mid = v_centre[:, :2].mean(axis=0)
    return arm, v_mid, yaw_rate


def _wrench(centre, force, point):
    mid = centre[:, :2].mean(axis=0)
    lever = point[:, :2] - mid
    torque = np.sum(lever[:, 0] * force[:, 1] - lever[:, 1] * force[:, 0])
    return force[:, :2].sum(axis=0), torque


def _predict(centre, v_centre, omega, force, point, mass, yaw_inertia, dt):
    """Wheel centre velocities after one step under `force`, rover rigid."""
    arm, v_mid, yaw_rate = _planar(centre, v_centre, omega)
    f, torque = _wrench(centre, force, point)
    v_mid = v_mid + f / mass * dt
    yaw_rate = yaw_rate + torque / yaw_inertia * dt
    v = v_mid + yaw_rate * np.c_[-arm[:, 1], arm[:, 0]]
    return np.c_[v, np.zeros(len(centre))]


def _no_reversal(centre, v_pred, side, point, mass, yaw_inertia, dt):
    """Largest scale in [0, 1] at which `side` reverses no body velocity."""
    _, v_mid, _ = _planar(centre, v_pred, np.zeros((len(centre), 3)))
    arm = centre[:, :2] - centre[:, :2].mean(axis=0)
    # Yaw rate from the predicted velocities (least squares over the wheels).
    yaw_rate = np.sum(arm[:, 0] * v_pred[:, 1] - arm[:, 1] * v_pred[:, 0]) / max(
        np.sum(arm**2), 1e-9
    )
    f, torque = _wrench(centre, side, point)

    scale = 1.0
    for momentum, push in (
        (mass * v_mid[0], f[0] * dt),
        (mass * v_mid[1], f[1] * dt),
        (yaw_inertia * yaw_rate, torque * dt),
    ):
        if push * momentum < 0 and abs(push) > abs(momentum):
            scale = min(scale, abs(momentum) / abs(push))
    return scale
