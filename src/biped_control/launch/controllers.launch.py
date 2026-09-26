import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ctrl_share = get_package_share_directory('biped_control')

    controllers_file = os.path.join(
        ctrl_share, 'config', 'biped_controllers.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    cm_timeout_arg = DeclareLaunchArgument(
        'controller_manager_timeout', default_value='30')

    # Spawners block until the controller_manager (inside Gazebo via
    # gz_ros2_control) is up, then load + activate their controller.
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', LaunchConfiguration(
                'controller_manager_timeout'),
        ],
        parameters=[{'use_sim_time': use_sim_time}])

    jtc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', LaunchConfiguration(
                'controller_manager_timeout'),
        ],
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        use_sim_time_arg, cm_timeout_arg, jsb_spawner, jtc_spawner])
