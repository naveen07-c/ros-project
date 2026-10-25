"""CAD walking + stair animation. Explicitly kinematic, starts automatically."""
import importlib.util
import os
import shutil
import tempfile
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def prepare(context):
    share = get_package_share_directory('biped_bringup')
    spec = importlib.util.spec_from_file_location('cad_demo', os.path.join(share, 'scripts', 'cad_demo.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    output = tempfile.mkdtemp(prefix='biped-cad-demo-')
    try:
        world = module.generate(get_package_share_directory('biped_description'), share, output)
    except Exception:
        shutil.rmtree(output)
        raise
    gazebo = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))
    def cleanup(context):
        shutil.rmtree(output, ignore_errors=True)
        return []
    plugin_path = os.path.join(get_package_prefix('biped_bringup'), 'lib')
    return [
        LogInfo(msg='Robot.STEP walking + stairs: KINEMATIC DEMO (no dynamic balancing). Completes in 52 simulation seconds.'),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path+os.pathsep+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        IncludeLaunchDescription(gazebo, condition=IfCondition(LaunchConfiguration('use_gui')),
                                 launch_arguments={'gz_args': '-r "'+world+'"'}.items()),
        IncludeLaunchDescription(gazebo, condition=UnlessCondition(LaunchConfiguration('use_gui')),
                                 launch_arguments={'gz_args': '-r -s "'+world+'"'}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('use_gui', default_value='true'), OpaqueFunction(function=prepare)])
