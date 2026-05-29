import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('xgo_description')
    gz_sim_vendor_prefix = get_package_prefix('gz_sim_vendor')
    world_file = os.path.join(pkg_share, 'worlds', 'real_objects_world.sdf')
    mover_script = os.path.join(pkg_share, 'scripts', 'move_real_objects.py')
    gazebo_fast_launch = os.path.join(pkg_share, 'launch', 'gazebo_fast.launch.py')
    gz_system_plugins = os.path.join(
        gz_sim_vendor_prefix,
        'opt',
        'gz_sim_vendor',
        'lib',
        'gz-sim-8',
        'plugins',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo GUI. Set false for server-only simulation.',
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=gz_system_plugins,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_fast_launch),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'world': world_file,
            }.items(),
        ),
        ExecuteProcess(
            cmd=['python3', mover_script],
            name='move_real_objects',
            output='log',
        ),
    ])
