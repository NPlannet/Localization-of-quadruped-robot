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

    # Erzeugt /scan_filtered aus dem rohen Lidar-Scan (ersetzt slam_toolbox-Input).
    # Frueh starten, damit Cartographer beim Start bereits Daten bekommt.
    map_patch = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='dynamic_scan_filter',
                executable='map_patch_node',
                name='map_patch_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    cartographer = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    '-configuration_directory',
                    LaunchConfiguration('cartographer_config_dir'),
                    '-configuration_basename',
                    LaunchConfiguration('cartographer_config_basename'),
                ],
                remappings=[
                    ('scan', LaunchConfiguration('cartographer_scan_topic')),
                    ('odom', '/odom'),
                ],
            ),
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    '-resolution',
                    LaunchConfiguration('occupancy_grid_resolution'),
                    '-publish_period_sec',
                    LaunchConfiguration('occupancy_publish_period'),
                ],
                condition=IfCondition(LaunchConfiguration('use_occupancy_grid')),
            ),
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
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'slam_mapping.rviz')],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('use_rviz')),
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo GUI. Set false for server-only simulation.',
        ),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=config_dir,
            description='Directory containing the Cartographer Lua configuration.',
        ),
        DeclareLaunchArgument(
            'cartographer_config_basename',
            default_value='cartographer_robot_2d_live.lua',
            description='Cartographer Lua configuration file basename.',
        ),
        DeclareLaunchArgument(
            'cartographer_scan_topic',
            default_value='/scan_filtered',
            description='LaserScan topic fed into Cartographer (produced by map_patch_node).',
        ),
        DeclareLaunchArgument(
            'use_occupancy_grid',
            default_value='true',
            description='Start the Cartographer occupancy grid publisher.',
        ),
        DeclareLaunchArgument(
            'occupancy_grid_resolution',
            default_value='0.05',
            description='Resolution for cartographer_occupancy_grid_node.',
        ),
        DeclareLaunchArgument(
            'occupancy_publish_period',
            default_value='1.0',
            description='Map publish period in seconds for cartographer_occupancy_grid_node.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with the SLAM config.',
        ),
        gazebo,
        map_patch,
        cartographer,
        rviz,
        nav2,
    ])