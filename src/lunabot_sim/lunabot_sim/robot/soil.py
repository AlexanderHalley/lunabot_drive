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

    alpha_0: float = 0.34
    """Slip angle, rad, at which sideways resistance reaches ~76% of its peak.
    Sets the minimum arc radius, the same at any speed: 0.34 gives ~1.5 m,
    0.4 ~1.3 m, 0.5 ~1.0 m. (Fitted as 0.5 before 2026-10-31, when the side
    force was judged after traction alone; that made arcs wider, and wider
    the slower the rover went.)"""

    rolling_resistance: float = 0.05
    """Fraction of the normal load that resists rolling (compaction)."""

    v_eps: float = 0.001
    """m/s. Keeps the slip ratio and angle defined at standstill. Below it
    the sand acts like a thick fluid: a locked pivot creeps at about
    1.6 * v_eps rad/s (0.0016 here, 0.1 deg/s)."""

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
    # The mass each wheel would have to stop on its own, for the rolling cap.
    share = load / _G

    # ---- Along the wheel: rolling resistance (small, explicit) ----
    f_roll = -config.rolling_resistance * load * np.tanh(v_x / config.v_eps)
    f_roll = np.clip(f_roll, -share / dt * np.abs(v_x), share / dt * np.abs(v_x))

    # ---- Traction (slip ratio) and side force (slip angle), implicit ----
    # Both are worked out at the velocity the rover WILL have at the end of
    # the step, with the forces themselves acting (backward Euler, every
    # wheel coupled through the rigid body). Two things depend on that:
    #   * a locked pivot stays locked: from standstill the slip angle is
    #     otherwise zero, side force zero, and traction gets a free step of
    #     rotation every time -- the pivot creeps;
    #   * turning does not depend on speed. Judging the side force after
    #     traction alone acts (the old way) added a fixed, speed-independent
    #     error to every slip angle -- the yaw that traction's moment would
    #     add in one step if nothing resisted it -- so at 240 Hz slow arcs came
    #     out far too wide (1.5 m commanded: 3.9 m at 0.075 m/s, 1.6 at 0.3).
    f_trac, f_side = _implicit_forces(
        config,
        load,
        slip,
        v_x,
        surface,
        v_centre,
        forward,
        lateral,
        point,
        centre,
        f_roll,
        mass,
        yaw_inertia,
        dt,
    )

    # Combined slip: traction and side force share one friction ellipse.
    use = np.hypot(f_trac / config.mu_long, f_side / config.mu_lat) / np.maximum(load, 1e-9)
    scale = np.where(use > 1.0, 1.0 / np.maximum(use, 1e-9), 1.0)
    f_trac *= scale
    f_side *= scale

    # No separate guard against side forces reversing the motion: forces
    # judged at the end-of-step velocity oppose that velocity, so they stop
    # motion without reversing it. (The old guard, judged against traction
    # alone, trimmed balanced side forces and let a locked pivot creep.)
    force = (f_trac + f_roll)[:, None] * forward + f_side[:, None] * lateral
    return force, point


_G = 9.81


_NEWTON_ITERATIONS = 20


def _implicit_forces(
    config,
    load,
    slip,
    v_x,
    surface,
    v_centre,
    forward,
    lateral,
    point,
    centre,
    f_roll,
    mass,
    yaw_inertia,
    dt,
):
    """Traction and side force per wheel at the END-of-step velocities.

    Unknowns F = [traction; side], 2n. The end-of-step slip (along) and side
    velocity (across) of each wheel are linear in F through the rigid planar
    rover: c = c0 + dt * W @ F, W the contact mobility. Solve F = phi(c) by
    Newton; the force laws oppose motion, so the system is well conditioned
    however steep the curves get at standstill. The wheels hold their spin.
    """
    n = len(load)
    mid = centre[:, :2].mean(axis=0)
    lever = point[:, :2] - mid
    dirs = np.concatenate([forward[:, :2], lateral[:, :2]])
    arms = np.concatenate([lever, lever])
    moment = arms[:, 0] * dirs[:, 1] - arms[:, 1] * dirs[:, 0]
    mobility = dt * (dirs @ dirs.T / mass + np.outer(moment, moment) / yaw_inertia)

    v_ref = np.maximum(np.maximum(np.abs(v_x), np.abs(surface)), config.v_eps)
    k_trac = 1.0 / (v_ref * config.kappa_0)
    trac_max = config.mu_long * load
    side_max = config.mu_lat * load
    c0 = np.concatenate([slip, np.einsum('ij,ij->i', v_centre, lateral)])
    c0 = c0 + mobility[:, :n] @ f_roll

    eye = np.eye(2 * n)
    bound = np.concatenate([trac_max, side_max])

    def phi(forces):
        """Force laws at the end-of-step velocities, and their Jacobian."""
        c = c0 + mobility @ forces
        s, u = c[:n], c[n:]
        # Traction: + driving (rim faster than travel), - braking.
        t = np.tanh(-s * k_trac)
        dt_ds = -trac_max * (1.0 - t * t) * k_trac
        # Side force from the slip angle; the wheel's forward speed is
        # s + surface (the rim speed is held).
        w = s + surface
        den = np.abs(w) + config.v_eps
        a = np.tanh(np.arctan2(u, den) / config.alpha_0)
        ds_dalpha = -side_max * (1.0 - a * a) / config.alpha_0
        rr = u * u + den * den
        jac = np.zeros((2 * n, 2 * n))
        jac[np.arange(n), np.arange(n)] = dt_ds
        jac[n + np.arange(n), np.arange(n)] = ds_dalpha * -u * np.sign(w) / rr
        jac[n + np.arange(n), n + np.arange(n)] = ds_dalpha * den / rr
        return np.concatenate([trac_max * t, -side_max * a]), jac @ mobility

    # Newton with a backtracking line search: near standstill the curves are
    # nearly vertical and a full step can overshoot into the clamp and back.
    forces = np.zeros(2 * n)
    target, d_phi = phi(forces)
    residual = forces - target
    tolerance = 1e-6 * (1.0 + load.max(initial=0.0))
    for _ in range(_NEWTON_ITERATIONS):
        if np.max(np.abs(residual)) < tolerance:
            break
        step = np.linalg.solve(eye - d_phi, -residual)
        norm = np.linalg.norm(residual)
        lam = 1.0
        while True:
            trial = np.clip(forces + lam * step, -bound, bound)
            trial_target, trial_d_phi = phi(trial)
            trial_residual = trial - trial_target
            if np.linalg.norm(trial_residual) < norm or lam < 1e-3:
                break
            lam *= 0.5
        forces, target, d_phi, residual = trial, trial_target, trial_d_phi, trial_residual
    return forces[:n], forces[n:]


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
