import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('xgo_description')
    config_dir = os.path.join(pkg_share, 'config')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_mapping.rviz')
    workspace = os.environ.get(
        'WORKSPACE',
        os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..')),
    )
    dataset_dir = os.path.join(workspace, 'evaluation', 'datasets', 'w1')
    results_dir = os.path.join(workspace, 'evaluation', 'results', 'w1')
    default_bag_path = os.path.join(dataset_dir, 'bag')
    default_waypoints_path = os.path.join(dataset_dir, 'waypoints.json')
    default_eval_output_filtered = os.path.join(
        results_dir,
        'metrics',
        'w1_cartographer_filtered_waypoint_eval.json',
    )
    default_eval_output_raw = os.path.join(
        results_dir,
        'metrics',
        'w1_cartographer_raw_waypoint_eval.json',
    )
    default_qos_overrides = os.path.join(config_dir, 'bag_play_qos_overrides.yaml')
    default_dynamic_filter_params = os.path.join(config_dir, 'dynamic_scan_filter_bag_replay.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value=default_bag_path,
            description='Path to a rosbag2 directory that contains metadata.yaml.',
        ),
        DeclareLaunchArgument(
            'playback_rate',
            default_value='1.0',
            description='Playback rate passed to ros2 bag play.',
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=default_waypoints_path,
            description='Ground-truth waypoint sidecar JSON aligned with the bag timestamps.',
        ),
        DeclareLaunchArgument(
            'evaluation_output_path',
            default_value=PythonExpression([
                repr(default_eval_output_filtered),
                " if '",
                LaunchConfiguration('use_dynamic_filter'),
                "' == 'true' else ",
                repr(default_eval_output_raw),
            ]),
            description='Output JSON path for waypoint localization metrics.',
        ),
        DeclareLaunchArgument(
            'qos_overrides_path',
            default_value=default_qos_overrides,
            description='QoS override YAML passed to ros2 bag play.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with the SLAM config.',
        ),
        DeclareLaunchArgument(
            'use_image_view',
            default_value='true',
            description='Start rqt_image_view on the republished raw camera topic.',
        ),
        DeclareLaunchArgument(
            'use_dynamic_filter',
            default_value='true',
            description='Filter dynamic LiDAR returns before feeding scans into Cartographer.',
        ),
        DeclareLaunchArgument(
            'use_offline_odom',
            default_value='true',
            description='Synthesize /odom and odom->base_link TF from replayed velocity and IMU topics.',
        ),
        DeclareLaunchArgument(
            'offline_twist_topic',
            default_value='/xgo/applied_vel',
            description='TwistStamped topic used to synthesize odometry during bag replay.',
        ),
        DeclareLaunchArgument(
            'offline_imu_topic',
            default_value='/imu/data',
            description='IMU topic used to recover heading during bag replay.',
        ),
        DeclareLaunchArgument(
            'use_waypoint_evaluator',
            default_value='true',
            description='Evaluate map-frame localization accuracy against the waypoint sidecar.',
        ),
        DeclareLaunchArgument(
            'evaluation_settling_window_sec',
            default_value='1.0',
            description='Centered time window used to obtain a stable pose at each waypoint.',
        ),
        DeclareLaunchArgument(
            'evaluation_window_sample_count',
            default_value='11',
            description='Number of TF samples taken across each waypoint settling window.',
        ),
        DeclareLaunchArgument(
            'evaluation_min_window_samples',
            default_value='5',
            description='Minimum successful TF samples required to score a waypoint.',
        ),
        DeclareLaunchArgument(
            'evaluation_alignment_mode',
            default_value='se2',
            description="Coordinate-frame alignment: 'se2' or 'none'.",
        ),
        DeclareLaunchArgument(
            'republish_camera',
            default_value='true',
            description='Republish the recorded compressed camera stream as raw images.',
        ),
        DeclareLaunchArgument(
            'camera_compressed_topic',
            default_value='/camera/image_raw/compressed',
            description='Compressed camera topic recorded in the bag.',
        ),
        DeclareLaunchArgument(
            'camera_raw_topic',
            default_value='/camera/image_raw',
            description='Raw camera topic published by image_transport republish.',
        ),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=config_dir,
            description='Directory containing the Cartographer Lua configuration.',
        ),
        DeclareLaunchArgument(
            'cartographer_config_basename',
            default_value='cartographer_replay_2d.lua',
            description='Cartographer Lua configuration file basename.',
        ),
        DeclareLaunchArgument(
            'dynamic_filter_params_file',
            default_value=default_dynamic_filter_params,
            description='Dynamic scan filter parameter file used during bag replay.',
        ),
        DeclareLaunchArgument(
            'cartographer_scan_topic',
            default_value=PythonExpression([
                "'/scan_filtered' if '",
                LaunchConfiguration('use_dynamic_filter'),
                "' == 'true' else '/scan_normalized'",
            ]),
            description='LaserScan topic fed into Cartographer.',
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
        Node(
            package='dynamic_scan_filter',
            executable='scan_normalizer_node',
            name='scan_normalizer',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='dynamic_scan_filter',
            executable='dynamic_scan_filter_node',
            name='dynamic_scan_filter',
            output='screen',
            parameters=[
                LaunchConfiguration('dynamic_filter_params_file'),
                {'use_sim_time': True},
            ],
            condition=IfCondition(LaunchConfiguration('use_dynamic_filter')),
        ),
        Node(
            package='xgo_driver_bridge',
            executable='xgo_offline_odom_node',
            name='xgo_offline_odom',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'input_twist_topic': LaunchConfiguration('offline_twist_topic'),
                    'input_imu_topic': LaunchConfiguration('offline_imu_topic'),
                }
            ],
            condition=IfCondition(LaunchConfiguration('use_offline_odom')),
        ),
        Node(
            package='xgo_driver_bridge',
            executable='waypoint_accuracy_node',
            name='waypoint_accuracy_monitor',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'waypoints_file': LaunchConfiguration('waypoints_file'),
                    'output_path': LaunchConfiguration('evaluation_output_path'),
                    'settling_window_sec': LaunchConfiguration(
                        'evaluation_settling_window_sec'
                    ),
                    'window_sample_count': LaunchConfiguration(
                        'evaluation_window_sample_count'
                    ),
                    'min_window_samples': LaunchConfiguration(
                        'evaluation_min_window_samples'
                    ),
                    'alignment_mode': LaunchConfiguration('evaluation_alignment_mode'),
                }
            ],
            condition=IfCondition(LaunchConfiguration('use_waypoint_evaluator')),
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='camera_image_republisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'in_transport': 'compressed',
                    'out_transport': 'raw',
                }
            ],
            remappings=[
                ('in/compressed', LaunchConfiguration('camera_compressed_topic')),
                ('out', LaunchConfiguration('camera_raw_topic')),
            ],
            condition=IfCondition(LaunchConfiguration('republish_camera')),
        ),
        TimerAction(
            period=1.0,
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
            ],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config],
                    parameters=[{'use_sim_time': True}],
                    condition=IfCondition(LaunchConfiguration('use_rviz')),
                ),
                Node(
                    package='rqt_image_view',
                    executable='rqt_image_view',
                    name='camera_image_view',
                    output='screen',
                    arguments=[LaunchConfiguration('camera_raw_topic')],
                    parameters=[{'use_sim_time': True}],
                    condition=IfCondition(LaunchConfiguration('use_image_view')),
                ),
            ],
        ),
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2',
                        'bag',
                        'play',
                        LaunchConfiguration('bag_path'),
                        '--clock',
                        '--rate',
                        LaunchConfiguration('playback_rate'),
                        '--qos-profile-overrides-path',
                        LaunchConfiguration('qos_overrides_path'),
                        '--disable-keyboard-controls',
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
