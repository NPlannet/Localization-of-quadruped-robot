from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('version', default_value='xgomini'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument(
            'imu_read_mode',
            default_value='orientation_registers',
            description=(
                'IMU source: orientation_registers for the tested roll/pitch/yaw '
                'register path or combined for SDK read_imu().'
            ),
        ),
        Node(
            package='xgo_driver_bridge',
            executable='xgo_bridge_node',
            name='xgo_driver_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'version': LaunchConfiguration('version'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'imu_read_mode': LaunchConfiguration('imu_read_mode'),
                'enable_motion': ParameterValue(
                    LaunchConfiguration('enable_motion'),
                    value_type=bool,
                ),
            }],
        ),
    ])
