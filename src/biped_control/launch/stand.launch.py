import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """M5 acceptance stack (AT-04/AT-05):

    Gazebo (spawn) -> controller spawners -> stand_hold node.

    Prerequisite: the ros2_control apt packages must be installed
      sudo apt install ros-jazzy-gz-ros2-control \\
           ros-jazzy-joint-trajectory-controller ros-jazzy-ros2controlcli
    (verified in the M1 report; biped_control builds without them but the
    controller_manager plugin only exists once they are present).
    """
    bringup_share = get_package_share_directory('biped_bringup')

    use_gui = LaunchConfiguration('use_gui')
    use_gui_arg = DeclareLaunchArgument('use_gui', default_value='false')

    crouch_deg = LaunchConfiguration('crouch_deg')
    crouch_deg_arg = DeclareLaunchArgument(
        'crouch_deg', default_value='15.0',
        description='Crouch depth in degrees; 0 = hold the straight pose')

    transition_time = LaunchConfiguration('transition_time')
    transition_time_arg = DeclareLaunchArgument(
        'transition_time', default_value='2.0',
        description='Seconds to move from spawn pose into the hold pose')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'use_gui': use_gui,
                          'with_payload': LaunchConfiguration('with_payload')}.items())

    with_payload_arg = DeclareLaunchArgument(
        'with_payload', default_value='true',
        description='Attach the 1 kg payload link (AT-05 with/without test)')

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('biped_control'),
            'launch', 'controllers.launch.py')))

    stand_hold = Node(
        package='biped_control',
        executable='stand_hold',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'crouch_deg': crouch_deg,
                     'transition_time': transition_time}])

    # Optional RViz with the M9 project config (model, TF, M8 sensor
    # markers) for visual verification of the hold pose.
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('biped_control'),
            'launch', 'rviz.launch.py')),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items())

    return LaunchDescription([
        use_gui_arg, with_payload_arg, crouch_deg_arg, transition_time_arg,
        use_rviz_arg, gazebo, controllers, stand_hold, rviz])
