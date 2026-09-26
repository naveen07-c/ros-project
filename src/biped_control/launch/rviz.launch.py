import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """M9 (AT-09): RViz2 with the project config (robot model, TF, M8
    sensor markers, odometry axes). Optional in every acceptance stack.

    RViz renders on wall time (no /clock subscription), so use_sim_time
    is intentionally NOT set here.
    """
    ctrl_share = get_package_share_directory('biped_control')

    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(
            ctrl_share, 'config', 'rviz', 'biped.rviz')])

    return LaunchDescription([use_rviz_arg, rviz])
