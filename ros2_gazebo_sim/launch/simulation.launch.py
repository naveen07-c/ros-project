import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('biped_gazebo_sim')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(pkg_share, 'urdf', 'biped.urdf')
    default_world = os.path.join(pkg_share, 'worlds', 'terrain.sdf')
    controller_config = os.path.join(pkg_share, 'config', 'controllers.yaml')
    gait_config = os.path.join(pkg_share, 'config', 'gait.yaml')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    walk = LaunchConfiguration('walk')
    terrain_x = LaunchConfiguration('terrain_x')
    terrain_z = LaunchConfiguration('terrain_z')
    foot_lift = LaunchConfiguration('foot_lift')
    kp_pitch = LaunchConfiguration('kp_pitch')
    kd_pitch = LaunchConfiguration('kd_pitch')
    step_time = LaunchConfiguration('step_time')
    speed = LaunchConfiguration('speed')
    edge_stop = LaunchConfiguration('edge_stop')
    edge_lookahead = LaunchConfiguration('edge_lookahead')
    edge_drop = LaunchConfiguration('edge_drop')
    ankle_slope_sign = LaunchConfiguration('ankle_slope_sign')
    slope_zmp_gain = LaunchConfiguration('slope_zmp_gain')
    slope_speed_factor = LaunchConfiguration('slope_speed_factor')
    slope_speed_thresh = LaunchConfiguration('slope_speed_thresh')

    # Load the URDF and point the gz_ros2_control plugin at the installed
    # controller configuration file.
    with open(urdf_path, 'r') as f:
        robot_description = f.read().replace('@CONTROLLER_CONFIG@', controller_config)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v3 ', world],
        }.items(),
        condition=UnlessCondition(headless),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v3 ', world],
        }.items(),
        condition=IfCondition(headless),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'biped_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.72',
        ],
    )

    # Bridge the Gazebo clock into ROS so time works in the simulation.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    # Bridge the base odometry used by the walking/balance controller.
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        output='screen',
        arguments=[
            '/model/biped_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/model/biped_robot/odometry', '/odom'),
        ],
    )

    # Bridge the pelvis IMU (Phase 1 state estimation).
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        output='screen',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
    )

    # The gz_ros2_control plugin starts controller_manager automatically.
    js_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # Generates the walking gait and streams it to leg_controller.
    gait_controller = Node(
        package='biped_gazebo_sim',
        executable='gait_controller.py',
        name='gait_controller',
        output='screen',
        parameters=[
            gait_config,
            {
                'use_sim_time': True,
                'walk_enable': ParameterValue(walk, value_type=bool),
                'terrain_x': terrain_x,
                'terrain_z': terrain_z,
                'foot_lift': ParameterValue(foot_lift, value_type=float),
                'kp_pitch': ParameterValue(kp_pitch, value_type=float),
                'kd_pitch': ParameterValue(kd_pitch, value_type=float),
                'step_time': ParameterValue(step_time, value_type=float),
                'speed': ParameterValue(speed, value_type=float),
                'edge_stop': ParameterValue(edge_stop, value_type=bool),
                'edge_lookahead': ParameterValue(edge_lookahead, value_type=float),
                'edge_drop': ParameterValue(edge_drop, value_type=float),
                'ankle_slope_sign': ParameterValue(ankle_slope_sign, value_type=float),
                'slope_zmp_gain': ParameterValue(slope_zmp_gain, value_type=float),
                'slope_speed_factor': ParameterValue(slope_speed_factor, value_type=float),
                'slope_speed_thresh': ParameterValue(slope_speed_thresh, value_type=float),
            },
        ],
    )

    # Let Gazebo resolve the robot's mesh URIs (package://biped_gazebo_sim/...,
    # which gz rewrites to model://). gz searches GZ_SIM_RESOURCE_PATH for a
    # directory named biped_gazebo_sim, so add the share dir that contains it.
    gz_resource_dir = os.path.dirname(pkg_share)
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gz_resource_dir + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    return LaunchDescription([
        set_gz_resource_path,
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the Gazebo world SDF file.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo server only (no GUI).',
        ),
        DeclareLaunchArgument(
            'walk',
            default_value='true',
            description='Enable the walking gait (false keeps the robot standing).',
        ),
        DeclareLaunchArgument(
            'terrain_x',
            default_value='',
            description='Comma-separated ground-height breakpoints (x, m).',
        ),
        DeclareLaunchArgument(
            'terrain_z',
            default_value='',
            description='Comma-separated ground heights (z, m) matching terrain_x.',
        ),
        DeclareLaunchArgument(
            'foot_lift',
            default_value='0.04',
            description='Swing-foot clearance (m). Raise it to clear stair risers.',
        ),
        DeclareLaunchArgument(
            'kp_pitch',
            default_value='0.0',
            description='Pitch-feedback proportional gain (0 disables it).',
        ),
        DeclareLaunchArgument(
            'kd_pitch',
            default_value='0.0',
            description='Pitch-feedback derivative gain (0 disables it).',
        ),
        DeclareLaunchArgument(
            'step_time',
            default_value='1.0',
            description='Seconds per step (half cycle). Raise it to slow the gait.',
        ),
        DeclareLaunchArgument(
            'speed',
            default_value='0.10',
            description='Forward walking speed (m/s). Sets the stride = speed*step_time.',
        ),
        DeclareLaunchArgument(
            'edge_stop',
            default_value='true',
            description='Fall-safe: halt before a drop-off detected ahead.',
        ),
        DeclareLaunchArgument(
            'edge_lookahead',
            default_value='0.30',
            description='Distance ahead of the base probed for an edge (m).',
        ),
        DeclareLaunchArgument(
            'edge_drop',
            default_value='0.15',
            description='Down-step size that triggers the edge stop (m).',
        ),
        DeclareLaunchArgument(
            'ankle_slope_sign',
            default_value='-1.0',
            description='Sign that flattens the sole against the local slope.',
        ),
        DeclareLaunchArgument(
            'slope_zmp_gain',
            default_value='0.0',
            description='ZMP lean scale on ramps (1.0 = z_c*sin(alpha)).',
        ),
        DeclareLaunchArgument(
            'slope_speed_factor',
            default_value='0.5',
            description='Stride scale used while on a genuine incline.',
        ),
        DeclareLaunchArgument(
            'slope_speed_thresh',
            default_value='0.03',
            description='|slope| above which the stride is shortened.',
        ),

        gz_sim,
        gz_sim_headless,
        clock_bridge,
        odom_bridge,
        imu_bridge,
        TimerAction(period=3.0, actions=[spawn_entity]),
        TimerAction(period=4.0, actions=[robot_state_publisher]),
        TimerAction(period=6.0, actions=[js_broadcaster_spawner]),
        TimerAction(period=7.0, actions=[leg_controller_spawner]),
        TimerAction(period=10.0, actions=[gait_controller]),
    ])
