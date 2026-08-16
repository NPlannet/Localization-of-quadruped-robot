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
    driver_share = get_package_share_directory('xgo_driver_bridge')
    config_dir = os.path.join(pkg_share, 'config')
    world_file = os.path.join(pkg_share, 'worlds', 'real_objects_world.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo_fast.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'world': world_file,
        }.items(),
    )

    rtabmap = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        driver_share, 'launch', 'rtabmap_robot.launch.py'
                    )
                ),
                launch_arguments={
                    'scan_topic': LaunchConfiguration('rtabmap_scan_topic'),
                    'qos_scan': '1',
                    'odom_topic': '/odom',
                    'camera_topic': '/camera/image_raw',
                    'camera_info_topic': '/camera/camera_info',
                    'use_camera': LaunchConfiguration('rtabmap_use_camera'),
                    'use_sim_time': 'true',
                    'database_path': LaunchConfiguration(
                        'rtabmap_database_path'
                    ),
                    'delete_database_on_start': LaunchConfiguration(
                        'rtabmap_delete_database_on_start'
                    ),
                }.items(),
            ),
        ],
    )

    nav2 = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'navigation_launch.py',
                    )
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': os.path.join(config_dir, 'nav2_params.yaml'),
                }.items(),
            ),
        ],
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

    workspace = os.environ.get('WORKSPACE', '/workspaces/robot_sim_sose')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'rtabmap_scan_topic', default_value='/scan_filtered'
        ),
        DeclareLaunchArgument('rtabmap_use_camera', default_value='true'),
        DeclareLaunchArgument(
            'rtabmap_database_path',
            default_value=os.path.join(
                workspace, 'maps', 'rtabmap_simulation.db'
            ),
        ),
        DeclareLaunchArgument(
            'rtabmap_delete_database_on_start', default_value='true'
        ),
        gazebo,
        rtabmap,
        rviz,
        nav2,
    ])
