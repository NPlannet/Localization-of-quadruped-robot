import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bridge_share = get_package_share_directory('xgo_driver_bridge')

    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool,
    )

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory',
            LaunchConfiguration('configuration_directory'),
            '-configuration_basename',
            LaunchConfiguration('configuration_basename'),
        ],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
            ('odom', LaunchConfiguration('odom_topic')),
            ('imu', LaunchConfiguration('imu_topic')),
        ],
    )

    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution',
            LaunchConfiguration('occupancy_grid_resolution'),
            '-publish_period_sec',
            LaunchConfiguration('occupancy_publish_period'),
        ],
        condition=IfCondition(LaunchConfiguration('use_occupancy_grid')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=os.path.join(bridge_share, 'config'),
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='cartographer_robot_2d.lua',
        ),
        DeclareLaunchArgument('scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_occupancy_grid', default_value='true'),
        DeclareLaunchArgument('occupancy_grid_resolution', default_value='0.05'),
        DeclareLaunchArgument('occupancy_publish_period', default_value='1.0'),
        cartographer,
        occupancy_grid,
    ])
