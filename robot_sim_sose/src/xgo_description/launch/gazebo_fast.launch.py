import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('xgo_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'xgo.urdf')
    world_file = LaunchConfiguration('world').perform(context)

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes')
    gz_args = f'-r {world_file}' if gui else f'-r -s {world_file}'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'xgo_robot', '-string', robot_desc, '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen',
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',##########
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    dynamic_scan_filter = Node(
        package='dynamic_scan_filter',
        executable='dynamic_scan_filter_node',
        name='dynamic_scan_filter',
        output='screen',
    )
    
    return [gz_sim, robot_state_publisher, spawn_entity, bridge, dynamic_scan_filter]


def generate_launch_description():
    pkg_share = get_package_share_directory('xgo_description')
    pkg_prefix = get_package_prefix('xgo_description')
    gz_sim_vendor_prefix = get_package_prefix('gz_sim_vendor')
    default_world = os.path.join(pkg_share, 'worlds', 'slam_test_world.sdf')
    # default_world = os.path.join(pkg_share, 'worlds', 'real_objects_world.sdf')
    gz_system_plugins = os.path.join(
        gz_sim_vendor_prefix,
        'opt',
        'gz_sim_vendor',
        'lib',
        'gz-sim-8',
        'plugins',
    )
    custom_plugin_dir = os.path.join(pkg_prefix, 'lib')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo GUI. Set false for server-only simulation.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the Gazebo world SDF file.',
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=[custom_plugin_dir, ':', gz_system_plugins],
        ),
        OpaqueFunction(function=launch_setup),
    ])
