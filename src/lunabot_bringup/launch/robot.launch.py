# Copyright 2027 Lunabot. Licensed under the MIT License.

"""The one launch file you type.

    ros2 launch lunabot_bringup robot.launch.py hw:=mock rviz:=true

Everything else in this package is included from here. Composing your own
combination of the sub-launch files is fine for debugging, but this file is
where the argument defaults and the mutual exclusions live -- in particular
which node owns odom -> base_link, which is the single easiest thing to get
wrong in the whole stack.

Sub-launch files grow onto this as the packages land: camera and teleop, then
SLAM, then perception, then navigation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

# SetParameter is a launch_ros action, not a launch one: it sets a ROS
# parameter on every Node in scope, which plain launch knows nothing about.
# Importing it from launch.actions raises ImportError while the module is
# being loaded, so robot.launch.py -- the only launch file anyone types --
# does not load at all, and every test that constructs it fails with the same
# unrelated-looking message.
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'hw',
        default_value='mock',
        choices=['mock', 'sim', 'real'],
        description=(
            'Hardware backend. mock runs anywhere with no hardware and no GPU; '
            'sim expects Isaac Sim to be running already; real expects SocketCAN.'
        ),
    ),
    DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='SocketCAN device for hw:=real. Use vcan0 to test without motors.',
    ),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description=(
            'Use /clock instead of the wall clock. Set true whenever hw:=sim. '
            'Isaac must already be publishing /clock or every node blocks at time zero.'
        ),
    ),
    DeclareLaunchArgument(
        'odom_source',
        default_value='wheel',
        choices=['wheel', 'visual', 'ekf'],
        description=(
            'Who publishes odom -> base_link. Exactly one node may. '
            'wheel = diff_drive_controller, visual = cuVSLAM, ekf = robot_localization.'
        ),
    ),
    DeclareLaunchArgument(
        'slam',
        default_value='none',
        choices=['none', 'rtabmap', 'cuvslam'],
        description=(
            'SLAM backend. rtabmap runs on CPU anywhere; cuvslam needs an NVIDIA '
            'GPU and is scaffolding (see docs/SLAM.md). Whichever runs owns map -> odom.'
        ),
    ),
    DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Localise against an existing map instead of building one.',
    ),
    DeclareLaunchArgument(
        'camera',
        default_value='auto',
        choices=['auto', 'true', 'false'],
        description=(
            'Start the OAK-D driver. auto means "only when hw is real" -- under '
            'hw:=sim Isaac publishes the camera topics itself, and starting the '
            'driver too would put two publishers on every camera topic.'
        ),
    ),
    DeclareLaunchArgument(
        'camera_profile',
        default_value='default',
        choices=['default', 'rgb_only', 'pointcloud', 'stereo_rect'],
        description='Camera configuration. stereo_rect is required by the cuVSLAM backend.',
    ),
    DeclareLaunchArgument(
        'teleop',
        default_value='true',
        description='Start joy, teleop_twist_joy and twist_mux.',
    ),
    DeclareLaunchArgument(
        'joy',
        default_value='true',
        description='Start the joystick nodes. False runs twist_mux alone, for Nav2-only driving.',
    ),
    DeclareLaunchArgument(
        'perception',
        default_value='false',
        description=(
            'Start boulder detection. Consumes /oak_d/points, so it needs either '
            'the camera or Isaac publishing.'
        ),
    ),
    DeclareLaunchArgument(
        'nav',
        default_value='false',
        description=(
            'Start Nav2. Needs the map frame, so pair it with slam:=rtabmap -- '
            'nav:=true on its own leaves bt_navigator waiting for a transform '
            'that never arrives. Nav2 drives /cmd_vel_nav; teleop still wins.'
        ),
    ),
    DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz2.',
    ),
    DeclareLaunchArgument(
        'rviz_config',
        default_value='slam',
        choices=['description', 'slam'],
        description='Which RViz config to load.',
    ),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    bringup = FindPackageShare('lunabot_bringup')
    slam_pkg = FindPackageShare('lunabot_slam')

    def include(package, name, condition=None, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([package, 'launch', name])),
            condition=condition,
            launch_arguments={'use_sim_time': use_sim_time, **kwargs}.items(),
        )

    odom_source = LaunchConfiguration('odom_source')

    return LaunchDescription(
        ARGUMENTS
        + [
            # Applies use_sim_time to every Node action in this launch scope.
            # Included descriptions do NOT inherit it, which is why include()
            # passes it explicitly as well. Belt and braces on purpose: a node
            # that misses use_sim_time under sim does not error, it just
            # behaves strangely, and RViz is the usual casualty.
            SetParameter(name='use_sim_time', value=use_sim_time),
            include(
                bringup,
                'description.launch.py',
                hardware=LaunchConfiguration('hw'),
                can_interface=LaunchConfiguration('can_interface'),
            ),
            include(
                bringup,
                'control.launch.py',
                # diff_drive_controller owns odom -> base_link only when it is
                # the odom source. Exactly one node may publish it.
                enable_odom_tf=_equals(odom_source, 'wheel'),
            ),
            include(
                bringup,
                'camera.launch.py',
                condition=IfCondition(
                    _camera_enabled(LaunchConfiguration('camera'), LaunchConfiguration('hw'))
                ),
                profile=LaunchConfiguration('camera_profile'),
            ),
            include(
                bringup,
                'teleop.launch.py',
                condition=IfCondition(LaunchConfiguration('teleop')),
                joy=LaunchConfiguration('joy'),
            ),
            # EKF, only when it is the odom source.
            include(
                bringup,
                'localization.launch.py',
                condition=IfCondition(_equals(odom_source, 'ekf')),
            ),
            # SLAM lives in its own package, so it does not go through
            # include()'s bringup-relative path.
            include(
                slam_pkg,
                'slam.launch.py',
                condition=UnlessCondition(_equals(LaunchConfiguration('slam'), 'none')),
                backend=LaunchConfiguration('slam'),
                localization=LaunchConfiguration('localization'),
                # The backend publishes odom -> base_link only when it is the
                # odom source. Otherwise it owns map -> odom and nothing else.
                publish_odom_tf=_equals(odom_source, 'visual'),
            ),
            include(
                FindPackageShare('lunabot_perception'),
                'perception.launch.py',
                condition=IfCondition(LaunchConfiguration('perception')),
                # Depth normalisation is a real-hardware concern only: the
                # OAK-D publishes 16UC1 millimetres, Isaac publishes 32FC1
                # metres. Converting in sim would be converting data that is
                # already correct.
                normalize_depth=_equals(LaunchConfiguration('hw'), 'real'),
            ),
            # Nav2 last of the functional stack, because it consumes what
            # everything above produces: /odom from the control stack, map ->
            # odom from SLAM, /oak_d/points into both costmaps.
            #
            # The slam:=none pairing is NOT blocked here. It is a real thing to
            # want -- bringing Nav2 up against a bag, or against an external
            # map -> odom publisher -- and this file has always documented the
            # mutual exclusions rather than policed them.
            include(
                FindPackageShare('lunabot_navigation'),
                'navigation.launch.py',
                condition=IfCondition(LaunchConfiguration('nav')),
            ),
            include(
                bringup,
                'rviz.launch.py',
                condition=IfCondition(LaunchConfiguration('rviz')),
                rviz_config=LaunchConfiguration('rviz_config'),
            ),
        ]
    )


def _equals(configuration, value):
    """Return 'true' when the launch configuration equals value, else 'false'.

    PythonExpression rather than a plain Python comparison: LaunchConfiguration
    values are not resolved until launch time, so `configuration == value` here
    would compare a substitution object to a string and be false always.

    Returning a string rather than using LaunchConfigurationEquals because the
    result is also passed down as a launch ARGUMENT, not only used as a
    condition.
    """
    return PythonExpression(["'true' if '", configuration, "' == '", value, "' else 'false'"])


def _camera_enabled(camera, hw):
    """Resolve camera:=auto against the hardware backend.

    auto means real hardware only. Under hw:=sim, Isaac's ROS2CameraHelper
    graphs already publish the camera topics, so also starting the driver
    would put two publishers on each -- and it would fail anyway, because
    there is no camera plugged in.
    """
    return PythonExpression(
        [
            "'true' if ('",
            camera,
            "' == 'true' or ('",
            camera,
            "' == 'auto' and '",
            hw,
            "' == 'real')) else 'false'",
        ]
    )
