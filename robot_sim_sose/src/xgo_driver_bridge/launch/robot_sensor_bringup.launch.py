import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def bool_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def slam_method_condition(method):
    return IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('slam_method'), "' == '", method, "'",
        ])
    )


def validate_slam_arguments(context):
    method = LaunchConfiguration('slam_method').perform(context).strip()
    scan_topic = LaunchConfiguration('slam_scan_topic').perform(context).strip()
    allowed_methods = {'none', 'slam_toolbox', 'cartographer', 'rtabmap'}

    if method not in allowed_methods:
        choices = ', '.join(sorted(allowed_methods))
        raise RuntimeError(
            f"Unsupported slam_method '{method}'. Choose one of: {choices}."
        )

    actions = [LogInfo(msg=f"Selected SLAM method: {method}")]
    filter_enabled = (
        LaunchConfiguration('start_dynamic_filter')
        .perform(context)
        .strip()
        .lower()
        in {'1', 'true', 'yes', 'on'}
    )
    camera_enabled = (
        LaunchConfiguration('start_camera')
        .perform(context)
        .strip()
        .lower()
        in {'1', 'true', 'yes', 'on'}
    )
    rtabmap_camera_enabled = (
        LaunchConfiguration('rtabmap_use_camera')
        .perform(context)
        .strip()
        .lower()
        in {'1', 'true', 'yes', 'on'}
    )

    if method != 'none' and scan_topic == '/scan_filtered' and not filter_enabled:
        actions.append(
            LogInfo(
                msg=(
                    'WARNING: SLAM is configured for /scan_filtered, but the '
                    'dynamic filter is disabled. Use slam_scan_topic:=/scan '
                    'or enable start_dynamic_filter.'
                )
            )
        )

    if method == 'rtabmap' and rtabmap_camera_enabled and not camera_enabled:
        actions.append(
            LogInfo(
                msg=(
                    'WARNING: RTAB-Map RGB input is enabled, but camera_ros is '
                    'disabled. Set rtabmap_use_camera:=false or '
                    'start_camera:=true.'
                )
            )
        )

    return actions


def generate_launch_description():
    bridge_share = get_package_share_directory('xgo_driver_bridge')
    config_dir = os.path.join(bridge_share, 'config')

    xgo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, 'launch', 'xgo_bridge.launch.py')
        ),
        launch_arguments={
            'port': LaunchConfiguration('xgo_port'),
            'version': LaunchConfiguration('xgo_version'),
            'enable_motion': LaunchConfiguration('enable_motion'),
            'imu_topic': '/imu/data',
            'imu_read_mode': LaunchConfiguration('xgo_imu_read_mode'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_xgo_bridge')),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ldlidar_stl_ros2'),
                'launch',
                'ld19.launch.py',
            )
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_port'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_lidar')),
    )

    camera = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        output='screen',
        parameters=[
            LaunchConfiguration('camera_params_file'),
            {'use_sim_time': bool_parameter('use_sim_time')},
        ],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('image_raw/compressed', '/camera/image_raw/compressed'),
            ('camera_info', '/camera/camera_info'),
        ],
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_optical',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--roll', '-1.57079632679',
            '--pitch', '0.0',
            '--yaw', '-1.57079632679',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_optical_frame',
        ],
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    dynamic_filter = Node(
        package='dynamic_scan_filter',
        executable='dynamic_scan_filter_node',
        name='dynamic_scan_filter',
        output='screen',
        parameters=[
            LaunchConfiguration('dynamic_filter_params_file'),
            {
                'input_scan_topic': LaunchConfiguration('filter_input_topic'),
                'output_scan_topic': LaunchConfiguration('filter_output_topic'),
                'tracking_frame': LaunchConfiguration('filter_tracking_frame'),
                'use_sim_time': bool_parameter('use_sim_time'),
            },
        ],
        condition=IfCondition(LaunchConfiguration('start_dynamic_filter')),
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[
            LaunchConfiguration('foxglove_params_file'),
            {
                'port': ParameterValue(
                    LaunchConfiguration('foxglove_port'),
                    value_type=int,
                ),
                'use_sim_time': bool_parameter('use_sim_time'),
            },
        ],
        condition=IfCondition(LaunchConfiguration('start_foxglove')),
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bridge_share,
                'launch',
                'slam_toolbox_robot.launch.py',
            )
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('slam_toolbox_params_file'),
            'scan_topic': LaunchConfiguration('slam_scan_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=slam_method_condition('slam_toolbox'),
    )

    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bridge_share,
                'launch',
                'cartographer_robot.launch.py',
            )
        ),
        launch_arguments={
            'configuration_directory': LaunchConfiguration(
                'cartographer_config_directory'
            ),
            'configuration_basename': LaunchConfiguration(
                'cartographer_config_basename'
            ),
            'scan_topic': LaunchConfiguration('slam_scan_topic'),
            'odom_topic': '/odom',
            'imu_topic': '/imu/data',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_occupancy_grid': LaunchConfiguration(
                'cartographer_use_occupancy_grid'
            ),
        }.items(),
        condition=slam_method_condition('cartographer'),
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bridge_share,
                'launch',
                'rtabmap_robot.launch.py',
            )
        ),
        launch_arguments={
            'scan_topic': LaunchConfiguration('slam_scan_topic'),
            'odom_topic': '/odom',
            'camera_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'use_camera': LaunchConfiguration('rtabmap_use_camera'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'database_path': LaunchConfiguration('rtabmap_database_path'),
            'delete_database_on_start': LaunchConfiguration(
                'rtabmap_delete_database_on_start'
            ),
            'detection_rate': LaunchConfiguration('rtabmap_detection_rate'),
            'visual_loop_threshold': LaunchConfiguration(
                'rtabmap_visual_loop_threshold'
            ),
        }.items(),
        condition=slam_method_condition('rtabmap'),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'xgo_port',
            default_value='/dev/ttyAMA0',
            description='Serial port for the XGO Mini2 controller and IMU.',
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the LD19 LiDAR.',
        ),
        DeclareLaunchArgument(
            'xgo_version',
            default_value='xgomini',
            description='Robot model string passed to the official XGO SDK.',
        ),
        DeclareLaunchArgument(
            'xgo_imu_read_mode',
            default_value='orientation_registers',
            description=(
                'XGO IMU source. Use orientation_registers for Mini2 M-5.1.1.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_motion',
            default_value='true',
            description=(
                'Allow /cmd_vel to command the robot and feed commanded '
                'velocity into odometry. The launch itself sends no commands.'
            ),
        ),
        DeclareLaunchArgument(
            'start_xgo_bridge',
            default_value='true',
            description='Publish /imu/data, /battery_state, /odom and odom TF.',
        ),
        DeclareLaunchArgument(
            'start_lidar',
            default_value='true',
            description='Publish the raw LD19 scan on /scan.',
        ),
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Publish image, compressed image and CameraInfo topics.',
        ),
        DeclareLaunchArgument(
            'start_dynamic_filter',
            default_value='true',
            description='Publish a filtered copy of /scan on /scan_filtered.',
        ),
        DeclareLaunchArgument(
            'start_foxglove',
            default_value='true',
            description='Expose evaluation topics to Lichtblick over WebSocket.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the ROS simulation clock. Keep false on the robot.',
        ),
        DeclareLaunchArgument(
            'camera_params_file',
            default_value=os.path.join(config_dir, 'camera_ros_robot.yaml'),
            description='camera_ros parameter file.',
        ),
        DeclareLaunchArgument(
            'camera_x',
            default_value='0.14',
            description='Camera optical centre X position relative to base_link.',
        ),
        DeclareLaunchArgument(
            'camera_y',
            default_value='0.0',
            description='Camera optical centre Y position relative to base_link.',
        ),
        DeclareLaunchArgument(
            'camera_z',
            default_value='0.10',
            description='Camera optical centre Z position relative to base_link.',
        ),
        DeclareLaunchArgument(
            'dynamic_filter_params_file',
            default_value=os.path.join(
                config_dir,
                'dynamic_scan_filter_robot.yaml',
            ),
            description='Dynamic filter tuning for the live LD19 stream.',
        ),
        DeclareLaunchArgument(
            'filter_input_topic',
            default_value='/scan',
            description='Live LaserScan input for the dynamic filter.',
        ),
        DeclareLaunchArgument(
            'filter_output_topic',
            default_value='/scan_filtered',
            description='Filtered LaserScan output.',
        ),
        DeclareLaunchArgument(
            'filter_tracking_frame',
            default_value='odom',
            description='Stable frame used to track moving scan clusters.',
        ),
        DeclareLaunchArgument(
            'foxglove_params_file',
            default_value=os.path.join(
                config_dir,
                'foxglove_bridge_robot.yaml',
            ),
            description='Topic whitelist and compression settings for Lichtblick.',
        ),
        DeclareLaunchArgument(
            'foxglove_port',
            default_value='8766',
            description='Foxglove WebSocket port used by Lichtblick.',
        ),
        DeclareLaunchArgument(
            'slam_method',
            default_value='none',
            description=(
                'Mapping algorithm to start: none, slam_toolbox, '
                'cartographer, or rtabmap.'
            ),
        ),
        DeclareLaunchArgument(
            'slam_scan_topic',
            default_value='/scan_filtered',
            description=(
                'LaserScan consumed by the selected SLAM algorithm. Use '
                '/scan for raw or /scan_filtered for the dynamic filter.'
            ),
        ),
        DeclareLaunchArgument(
            'slam_toolbox_params_file',
            default_value=os.path.join(
                config_dir,
                'slam_toolbox_robot.yaml',
            ),
            description='SLAM Toolbox live-robot parameter file.',
        ),
        DeclareLaunchArgument(
            'cartographer_config_directory',
            default_value=config_dir,
            description='Directory containing Cartographer Lua configuration.',
        ),
        DeclareLaunchArgument(
            'cartographer_config_basename',
            default_value='cartographer_robot_2d.lua',
            description='Cartographer Lua configuration filename.',
        ),
        DeclareLaunchArgument(
            'cartographer_use_occupancy_grid',
            default_value='true',
            description='Publish Cartographer submaps as nav_msgs/OccupancyGrid.',
        ),
        DeclareLaunchArgument(
            'rtabmap_use_camera',
            default_value='true',
            description='Use RGB images for RTAB-Map visual loop recognition.',
        ),
        DeclareLaunchArgument(
            'rtabmap_database_path',
            default_value=os.path.join(
                os.environ.get('WORKSPACE', '/workspaces/robot_sim_sose'),
                'maps',
                'rtabmap_robot.db',
            ),
            description='Persistent RTAB-Map database file.',
        ),
        DeclareLaunchArgument(
            'rtabmap_delete_database_on_start',
            default_value='false',
            description=(
                'Delete the selected RTAB-Map database before this run. '
                'Enable for an independent evaluation run.'
            ),
        ),
        DeclareLaunchArgument(
            'rtabmap_detection_rate',
            default_value='2.0',
            description='RTAB-Map memory/loop-detection update rate in Hz.',
        ),
        DeclareLaunchArgument(
            'rtabmap_visual_loop_threshold',
            default_value='0.11',
            description='RTAB-Map visual loop-closure acceptance threshold.',
        ),
        OpaqueFunction(function=validate_slam_arguments),
        LogInfo(
            msg=[
                'Starting robot evaluation stack. SLAM: ',
                LaunchConfiguration('slam_method'),
                '; raw LiDAR: /scan; ',
                'filtered LiDAR: ',
                LaunchConfiguration('filter_output_topic'),
                '; camera: /camera/image_raw/compressed; IMU: /imu/data; ',
                'odometry: /odom.',
            ]
        ),
        xgo_launch,
        lidar_launch,
        camera,
        camera_transform,
        dynamic_filter,
        foxglove_bridge,
        slam_toolbox,
        cartographer,
        rtabmap,
    ])
