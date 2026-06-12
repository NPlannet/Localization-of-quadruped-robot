import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('xgo_description')
    config_dir = os.path.join(pkg_share, 'config')
    default_world = os.path.join(pkg_share, 'worlds', 'slam_test_world.sdf')

    # 1. Gazebo + Robot
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                pkg_share,
                'launch',
                'gazebo_fast.launch.py'
            )
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
        }.items()
    )

    # 2. SLAM Toolbox – verzögert starten damit Gazebo zuerst hochfährt
    slam = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch',
                        'online_async_launch.py'
                    )
                ]),
                launch_arguments={
                    'slam_params_file': os.path.join(config_dir, 'slam_toolbox.yaml'),
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )

    # 3. Nav2 – noch später starten damit SLAM zuerst läuft
    nav2 = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    )
                ]),
                launch_arguments={
                    'use_sim_time': 'True',
                    'params_file': os.path.join(config_dir, 'nav2_params.yaml')
                }.items()
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the Gazebo world SDF file.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo GUI. Set false for server-only simulation.',
        ),
        gazebo,
        slam,
        nav2,
    ])
