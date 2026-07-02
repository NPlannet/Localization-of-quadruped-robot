from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS0'),
        DeclareLaunchArgument('version', default_value='xgomini'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        Node(
            package='xgo_driver_bridge',
            executable='xgo_bridge_node',
            name='xgo_driver_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'version': LaunchConfiguration('version'),
                'enable_motion': ParameterValue(
                    LaunchConfiguration('enable_motion'),
                    value_type=bool,
                ),
            }],
        ),
    ])
