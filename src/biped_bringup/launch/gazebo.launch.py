import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():
    bringup_share = get_package_share_directory('biped_bringup')
    desc_share = get_package_share_directory('biped_description')

    world_arg = DeclareLaunchArgument(
        'world', default_value=os.path.join(bringup_share, 'worlds', 'flat_world.sdf'))

    # AT-05 requires balance tests with and without the 1 kg payload
    # (xacro arg toggles the payload_link, PROJECT_REQUIREMENTS §4).
    with_payload = LaunchConfiguration('with_payload')
    with_payload_arg = DeclareLaunchArgument(
        'with_payload', default_value='true')

    # gz_sim.launch.py takes gz_args only: '-s' = server-only (headless).
    # A boolean string here would reach gz sim as a bogus positional file
    # argument (the M3/M4 fix), so GUI choice selects between two includes.
    use_gui = LaunchConfiguration('use_gui')
    use_gui_arg = DeclareLaunchArgument('use_gui', default_value='false')

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        condition=UnlessCondition(use_gui),
        launch_arguments={'gz_args': ['-r -s "', LaunchConfiguration('world'), '"']}.items())

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        condition=IfCondition(use_gui),
        launch_arguments={'gz_args': ['-r "', LaunchConfiguration('world'), '"']}.items())

    robot_description = ParameterValue(
        Command(['xacro ', shlex.quote(os.path.join(desc_share, 'urdf', 'biped.urdf.xacro')),
                 ' with_payload:=', with_payload]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'publish_frequency': 30.0}])

    # Spawn standing height: hip 0.55 + pelvis_hip_drop 0.05 (ASSUMPTIONS, audit-verified)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'biped',
                   '-z', '0.62',
                   '-allow_renaming', 'false'],
        output='screen')

    # M8 (PROJECT_REQUIREMENTS §6): Gazebo -> ROS bridges for the URDF
    # sensors (sensors.xacro). The clock stays on its own lazy bridge;
    # sensors share a second bridge. IMU is gz.msgs.IMU -> sensor_msgs/Imu,
    # contacts are gz.msgs.Contacts -> ros_gz_interfaces/Contacts.
    sensor_bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/left_foot_contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/right_foot_contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        output='screen')

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')

    # M7 (AT-07): identity map -> odom anchor. gait_odometry initializes
    # odom at the spawn pose, so while no localization exists (out of
    # scope per PROJECT_REQUIREMENTS section 7 - no LiDAR, no SLAM)
    # map == odom. If localization is ever added, replace this static
    # transform with a dynamic map -> odom estimate.
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        world_arg, with_payload_arg, use_gui_arg, gazebo_headless, gazebo_gui,
        robot_state_publisher, spawn, clock_bridge, sensor_bridges,
        map_odom_tf])
