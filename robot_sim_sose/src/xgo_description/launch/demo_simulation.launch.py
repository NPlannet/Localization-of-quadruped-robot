import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


SUPPORTED_METHODS = ('slam_toolbox', 'cartographer', 'rtabmap')


def select_demo(context):
    method = LaunchConfiguration('slam_method').perform(context).strip().lower()
    if method not in SUPPORTED_METHODS:
        choices = ', '.join(SUPPORTED_METHODS)
        raise RuntimeError(
            f"Unsupported slam_method '{method}'. Choose one of: {choices}."
        )

    pkg_share = get_package_share_directory('xgo_description')
    launch_dir = os.path.join(pkg_share, 'launch')

    if method == 'slam_toolbox':
        launch_file = 'real_objects.launch.py'
        launch_arguments = {
            'gui': LaunchConfiguration('gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }
    elif method == 'cartographer':
        launch_file = 'real_objects_cartographer.launch.py'
        launch_arguments = {
            'gui': LaunchConfiguration('gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'cartographer_scan_topic': LaunchConfiguration('scan_topic'),
        }
    else:
        launch_file = 'real_objects_rtabmap.launch.py'
        launch_arguments = {
            'gui': LaunchConfiguration('gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rtabmap_scan_topic': LaunchConfiguration('scan_topic'),
            'rtabmap_use_camera': LaunchConfiguration('rtabmap_use_camera'),
            'rtabmap_database_path': LaunchConfiguration(
                'rtabmap_database_path'
            ),
            'rtabmap_delete_database_on_start': LaunchConfiguration(
                'rtabmap_delete_database_on_start'
            ),
        }

    return [
        LogInfo(msg=f'Starting simulation demo with {method}.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, launch_file)
            ),
            launch_arguments=launch_arguments.items(),
        ),
    ]


def generate_launch_description():
    workspace = os.environ.get('WORKSPACE', '/workspaces/robot_sim_sose')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_method',
            default_value='slam_toolbox',
            description='Mapping algorithm: slam_toolbox, cartographer, or rtabmap.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo GUI.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description=(
                'Start RViz in this launch. The documented demo workflow '
                'normally runs RViz in a separate terminal.'
            ),
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_filtered',
            description='LaserScan input for Cartographer and RTAB-Map.',
        ),
        DeclareLaunchArgument(
            'rtabmap_use_camera',
            default_value='true',
            description='Use the simulated RGB camera for RTAB-Map loop recognition.',
        ),
        DeclareLaunchArgument(
            'rtabmap_database_path',
            default_value=os.path.join(
                workspace, 'maps', 'rtabmap_simulation.db'
            ),
            description='RTAB-Map database written by the simulation demo.',
        ),
        DeclareLaunchArgument(
            'rtabmap_delete_database_on_start',
            default_value='true',
            description='Start the RTAB-Map demo with an empty database.',
        ),
        OpaqueFunction(function=select_demo),
    ])
