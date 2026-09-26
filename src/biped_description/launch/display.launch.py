import os

from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():
    # Works both from the source tree and the install dir (same relative layout)
    default_model = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        'urdf', 'biped.urdf.xacro')

    model_arg = DeclareLaunchArgument(
        'model', default_value=default_model,
        description='Path to the robot xacro')

    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz2 alongside robot_state_publisher')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'publish_frequency': 30.0}])

    # Zero/default joint positions so the full TF tree is published
    # even before a real controller broadcasts /joint_states.
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            'rviz', 'biped_display.rviz')])

    return LaunchDescription([model_arg, use_rviz_arg, rsp, jsp, rviz])
