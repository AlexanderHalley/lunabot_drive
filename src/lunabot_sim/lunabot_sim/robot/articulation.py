"""Configure the rover's articulation for VELOCITY control.

This is the module most likely to need a debugging session, and it is worth
knowing why in advance.

`topic_based_ros2_control` publishes a sensor_msgs/JointState with BOTH the
position and velocity arrays populated. Isaac's ROS2SubscribeJointState node
decides whether to drive position or velocity targets based on which arrays
are non-empty -- so with both present it may well drive position, and the
rover will lurch toward an absolute wheel angle instead of spinning at a rate.

Two defences, both applied here:

  1. Wheel drives are configured with stiffness 0 and high damping. A drive
     with zero stiffness cannot hold a position at all; it can only track a
     velocity. Even if a position target arrives, it does nothing.
  2. WHEEL_JOINTS is stated explicitly rather than inferred, so a joint that
     silently keeps its importer defaults is a visible omission.

If the rover in sim responds to /cmd_vel by snapping to an angle and stopping,
this is the file.
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)

# Must match lunabot_description/urdf/lunabot.urdf.xacro exactly. These same
# strings appear in controllers.yaml.
WHEEL_JOINTS = (
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
)


def configure(articulation_path: str, max_effort: float = 40.0, damping: float = 1.0e4):
    """Set every wheel joint to velocity drive.

    max_effort is the joint's torque limit from properties.xacro. Too low and
    the rover cannot climb; too high and it can lift its own front wheels off
    the ground under acceleration.

    damping is the velocity-drive gain. High, because a low value gives a
    drive that tracks the commanded velocity so sluggishly that the control
    loop looks broken. Stiffness stays at zero -- see the module docstring.
    """
    from pxr import UsdPhysics
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    configured = []

    for joint_name in WHEEL_JOINTS:
        prim = _find_joint(stage, articulation_path, joint_name)
        if prim is None:
            logger.error(
                'joint %s not found under %s. Wheel names must match the URDF exactly; '
                'note that merge_fixed_joints in importer.py removes FIXED joints only, '
                'so a missing revolute joint is a real mismatch.',
                joint_name,
                articulation_path,
            )
            continue

        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
        drive.CreateTypeAttr().Set('force')
        # Zero stiffness: the drive physically cannot hold a position, so a
        # stray position target from the ROS bridge is harmless.
        drive.CreateStiffnessAttr().Set(0.0)
        drive.CreateDampingAttr().Set(damping)
        drive.CreateMaxForceAttr().Set(max_effort)
        drive.CreateTargetVelocityAttr().Set(0.0)

        configured.append(joint_name)

    if len(configured) != len(WHEEL_JOINTS):
        raise RuntimeError(
            f'configured {len(configured)} of {len(WHEEL_JOINTS)} wheel drives: '
            f'{configured}. A partially driven articulation will move, badly, and '
            'look like a tuning problem.'
        )

    logger.info('configured %d wheel joints for velocity drive', len(configured))
    return configured


def _find_joint(stage, root_path: str, joint_name: str):
    """Locate a joint prim by name anywhere under the articulation root.

    Searched rather than constructed from a path template because the
    importer's prim layout is not guaranteed stable across Isaac versions.
    """
    from pxr import Usd

    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        raise RuntimeError(f'no prim at {root_path}; did the URDF import succeed?')

    for prim in Usd.PrimRange(root):
        if prim.GetName() == joint_name:
            return prim
    return None


def set_solver_iterations(articulation_path: str, position: int = 32, velocity: int = 16):
    """Raise the articulation solver's iteration counts.

    Defaults are tuned for arms, not for a four-wheeled vehicle in continuous
    contact with a friction surface. Low iteration counts show up as wheels
    that sink into the ground, jitter at rest, or slide under braking -- all
    of which read as a physics-material problem and are not one.
    """
    from pxr import PhysxSchema
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(articulation_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f'no prim at {articulation_path}')

    api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
    api.CreateSolverPositionIterationCountAttr().Set(position)
    api.CreateSolverVelocityIterationCountAttr().Set(velocity)
    api.CreateEnabledSelfCollisionsAttr().Set(False)
    logger.info('articulation solver iterations set to %d/%d', position, velocity)
