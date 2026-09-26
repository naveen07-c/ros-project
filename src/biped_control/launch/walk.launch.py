import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_gait_node(context):
    """gait_generator node with an optional params-file override.

    gait_gen_params (string, default ''): path to a YAML file with a
    'gait_generator: ros__parameters:' block. Values there win over the
    inline defaults below - the no-rebuild A/B tuning hook (AT-11).
    """
    gait_params_file = LaunchConfiguration('gait_gen_params').perform(context)
    params = [{'use_sim_time': True,
               'step_length': LaunchConfiguration('step_length'),
               'step_period': LaunchConfiguration('step_period'),
               'crouch_deg': LaunchConfiguration('crouch_deg'),
               # 3.0 s crouch-in: the AT-05-proven 2.0 s crouch is
               # 15 deg; the gait approach is a deeper crouch and
               # 1.5 s was marginally fast enough to tip (goal never
               # converged -> watchdog ESTOP).
               'approach_time': 3.0,
               'walk_distance_limit': LaunchConfiguration(
                   'walk_distance_limit')}]
    if gait_params_file:
        params.append(gait_params_file)
    return [Node(
        package='biped_control',
        executable='gait_generator',
        output='screen',
        parameters=params)]


def generate_launch_description():
    """M6 acceptance stack (AT-06/AT-07/AT-10):

    Gazebo (spawn) -> controller spawners -> gait_generator +
    gait_odometry. Start/stop walking with std_msgs/Bool on
    /gait_command (true = walk, false = stop).
    """
    ctrl_share = get_package_share_directory('biped_control')
    bringup_share = get_package_share_directory('biped_bringup')

    use_gui = LaunchConfiguration('use_gui')
    use_gui_arg = DeclareLaunchArgument('use_gui', default_value='false')

    with_payload_arg = DeclareLaunchArgument(
        'with_payload', default_value='true')

    step_length = LaunchConfiguration('step_length')
    step_length_arg = DeclareLaunchArgument(
        'step_length', default_value='0.06',
        description='Per-step swing advance in meters (guard-clamped)')

    crouch_deg = LaunchConfiguration('crouch_deg')
    crouch_deg_arg = DeclareLaunchArgument(
        'crouch_deg', default_value='20.0',
        description='Gait crouch depth in degrees. The partial lateral '
                    'shift (support_shift) keeps every leg chain within '
                    'reach even at this shallow crouch, close to the '
                    'AT-05-verified 15 deg stand; the old full roll-over '
                    'geometry forced >= 25 deg and silently clamped the '
                    'touchdown chain. (AT-11 iteration 8)')

    step_period = LaunchConfiguration('step_period')
    step_period_arg = DeclareLaunchArgument(
        'step_period', default_value='10.0',
        description='Seconds per step (min 1.0). 10.0: the body-over-foot '
                    'inverted pendulum has T ~= 1.55 s at h = 0.6 m; a '
                    'weight shift needs >= 2T (3 s) to be quasi-static '
                    '(AT-11 diagnosis: faster shifts resonate the mode).',
                    )

    walk_limit = LaunchConfiguration('walk_distance_limit')
    walk_limit_arg = DeclareLaunchArgument(
        'walk_distance_limit', default_value='0.0',
        description='Auto-stop after this many meters (0 = unlimited)')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'use_gui': use_gui,
                          'with_payload': LaunchConfiguration(
                              'with_payload')}.items())

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ctrl_share, 'launch', 'controllers.launch.py')))

    gait_generator = OpaqueFunction(function=_make_gait_node)

    gait_odometry = Node(
        package='biped_control',
        executable='gait_odometry',
        output='screen',
        parameters=[{'use_sim_time': True}])

    # M9 (AT-09): shared RViz include (robot model, TF, M8 sensor
    # markers, odometry axes). contact_markers runs unconditionally -
    # its cost is negligible and /contact_markers stays inspectable
    # even without the GUI.
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false')

    contact_markers = Node(
        package='biped_control',
        executable='contact_markers',
        output='screen',
        parameters=[{'use_sim_time': True}])

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ctrl_share, 'launch', 'rviz.launch.py')),
        launch_arguments={'use_rviz': use_rviz}.items())

    gait_params_arg = DeclareLaunchArgument(
        'gait_gen_params', default_value='',
        description='Optional YAML overriding gait_generator node params '
                    '(no-rebuild tuning hook)')

    return LaunchDescription([
        use_gui_arg, with_payload_arg, step_length_arg, crouch_deg_arg,
        step_period_arg, gait_params_arg,
        walk_limit_arg, use_rviz_arg, gazebo, controllers, gait_generator,
        gait_odometry, contact_markers, rviz])
