import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('xgo_description')
    world_file = os.path.join(pkg_share, 'worlds', 'real_objects_world.sdf')
    gazebo_fast_launch = os.path.join(pkg_share, 'launch', 'gazebo_fast.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo GUI. Set false for server-only simulation.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_fast_launch),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'world': world_file,
            }.items(),
        ),
    ])
