import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    pkg_share = get_package_share_directory('xgo_description')
    config_dir = os.path.join(pkg_share, 'config')
    world_file = os.path.join(pkg_share, 'worlds', 'real_objects_world.sdf')
    gazebo_fast_launch = os.path.join(pkg_share, 'launch', 'gazebo_fast.launch.py')
    
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_fast_launch),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'world': world_file,
        }.items(),
    )
    
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

    rviz = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=[
                    '-d', os.path.join(pkg_share, 'rviz', 'slam_mapping.rviz')
                ],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('use_rviz')),
            ),
        ],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo GUI. Set false for server-only simulation.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz with the SLAM configuration.',
        ),
        gazebo,
        slam,
        rviz,
        nav2,
    ])
