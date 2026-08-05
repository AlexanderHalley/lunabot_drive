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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetParameter
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    pkg = FindPackageShare('lunabot_bringup')

    def include(name, condition=None, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', name])),
            condition=condition,
            launch_arguments={'use_sim_time': use_sim_time, **kwargs}.items(),
        )

    return LaunchDescription(
        ARGUMENTS
        + [
            # Applies use_sim_time to every Node action in this launch scope.
            # Included descriptions do NOT inherit it, which is why each
            # include() above passes it explicitly as well. Belt and braces on
            # purpose: a node that misses use_sim_time under sim does not
            # error, it just behaves strangely.
            SetParameter(name='use_sim_time', value=use_sim_time),
            include(
                'description.launch.py',
                hardware=LaunchConfiguration('hw'),
                can_interface=LaunchConfiguration('can_interface'),
            ),
            include(
                'control.launch.py',
                # diff_drive_controller publishes odom -> base_link only when
                # nothing else is claiming it. The other two sources arrive
                # with the SLAM and localisation packages; until then anything
                # but odom_source:=wheel leaves the transform unpublished, and
                # TF will say so loudly.
                enable_odom_tf=_odom_tf_from(LaunchConfiguration('odom_source')),
            ),
            include(
                'camera.launch.py',
                condition=IfCondition(
                    _camera_enabled(
                        LaunchConfiguration('camera'), LaunchConfiguration('hw')
                    )
                ),
                profile=LaunchConfiguration('camera_profile'),
            ),
            include(
                'teleop.launch.py',
                condition=IfCondition(LaunchConfiguration('teleop')),
                joy=LaunchConfiguration('joy'),
            ),
            include(
                'rviz.launch.py',
                condition=IfCondition(LaunchConfiguration('rviz')),
                rviz_config=LaunchConfiguration('rviz_config'),
            ),
        ]
    )


def _camera_enabled(camera, hw):
    """Resolve camera:=auto against the hardware backend.

    auto means "real hardware only". Under hw:=sim, Isaac's ROS2CameraHelper
    graphs already publish the camera topics, so also starting the driver
    would put two publishers on each of them -- and the driver would fail
    anyway, because there is no camera plugged in.
    """
    from launch.substitutions import PythonExpression

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


def _odom_tf_from(odom_source):
    """'true' when odom_source is wheel, 'false' otherwise.

    PythonExpression is the only way to compute this at launch time --
    LaunchConfiguration values are not resolved until then, so a plain
    `== 'wheel'` in Python here would compare a substitution object to a
    string and be false always.
    """
    from launch.substitutions import PythonExpression

    return PythonExpression(["'true' if '", odom_source, "' == 'wheel' else 'false'"])
