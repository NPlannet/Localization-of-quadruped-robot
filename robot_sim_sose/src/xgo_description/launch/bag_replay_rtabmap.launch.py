"""Evaluate RTAB-Map on W1 with 2D LiDAR and optional RGB loop recognition."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('xgo_description')
    config_dir = os.path.join(pkg_share, 'config')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_mapping.rviz')
    camera_calibration = os.path.join(
        config_dir,
        'xgo_camera_640x480_approx.yaml',
    )
    workspace = os.environ.get(
        'WORKSPACE',
        os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..')),
    )
    dataset_dir = os.path.join(workspace, 'evaluation', 'datasets', 'w1')
    results_dir = os.path.join(workspace, 'evaluation', 'results', 'w1')

    metric_paths = {
        (True, True): os.path.join(
            results_dir, 'metrics', 'w1_rtabmap_rgb_filtered_waypoint_eval.json'
        ),
        (True, False): os.path.join(
            results_dir, 'metrics', 'w1_rtabmap_rgb_raw_waypoint_eval.json'
        ),
        (False, True): os.path.join(
            results_dir, 'metrics', 'w1_rtabmap_lidar_filtered_waypoint_eval.json'
        ),
        (False, False): os.path.join(
            results_dir, 'metrics', 'w1_rtabmap_lidar_raw_waypoint_eval.json'
        ),
    }
    database_paths = {
        (True, True): os.path.join(results_dir, 'databases', 'w1_rtabmap_rgb_filtered.db'),
        (True, False): os.path.join(results_dir, 'databases', 'w1_rtabmap_rgb_raw.db'),
        (False, True): os.path.join(results_dir, 'databases', 'w1_rtabmap_lidar_filtered.db'),
        (False, False): os.path.join(results_dir, 'databases', 'w1_rtabmap_lidar_raw.db'),
    }

    def four_way_default(paths):
        return PythonExpression([
            repr(paths[(True, True)]),
            " if '", LaunchConfiguration('use_camera'), "' == 'true' and '",
            LaunchConfiguration('use_dynamic_filter'), "' == 'true' else ",
            repr(paths[(True, False)]),
            " if '", LaunchConfiguration('use_camera'), "' == 'true' else ",
            repr(paths[(False, True)]),
            " if '", LaunchConfiguration('use_dynamic_filter'), "' == 'true' else ",
            repr(paths[(False, False)]),
        ])

    rtabmap_parameters = {
        'use_sim_time': True,
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': '',
        'publish_tf': True,
        'subscribe_depth': False,
        'subscribe_rgbd': False,
        'subscribe_stereo': False,
        'subscribe_rgb': LaunchConfiguration('use_camera'),
        'subscribe_scan': True,
        'subscribe_scan_cloud': False,
        'approx_sync': True,
        'approx_sync_max_interval': 0.10,
        'topic_queue_size': 100,
        'sync_queue_size': 100,
        'qos_image': 1,
        'qos_camera_info': 1,
        'qos_scan': 1,
        'qos_odom': 1,
        'wait_for_transform': 1.0,
        'database_path': LaunchConfiguration('database_path'),

        # External /odom supplies motion. RGB creates appearance hypotheses;
        # scan ICP checks/refines their metric transformation.
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        'Mem/ImagePreDecimation': '1',
        'Mem/ImagePostDecimation': '1',
        'Kp/DetectorStrategy': '8',
        'Kp/MaxFeatures': '500',
        'Rtabmap/DetectionRate': ParameterValue(
            LaunchConfiguration('detection_rate'), value_type=str
        ),
        'Rtabmap/LoopThr': ParameterValue(
            LaunchConfiguration('visual_loop_threshold'), value_type=str
        ),
        'RGBD/LinearUpdate': '0.10',
        'RGBD/AngularUpdate': '0.10',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ForceOdom3DoF': 'true',
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'Icp/PointToPlane': 'false',
        'Icp/MaxCorrespondenceDistance': '0.20',
        'Icp/CorrespondenceRatio': '0.10',
        'Icp/MaxTranslation': '1.0',
        'Icp/MaxRotation': '3.14',

        # Construct the published 2D occupancy map only from LaserScan data.
        'Grid/Sensor': '0',
        'Grid/3D': 'false',
        'Grid/CellSize': '0.05',
        'Grid/RangeMin': '0.05',
        'Grid/RangeMax': '12.0',
        'Grid/Scan2dUnknownSpaceFilled': 'true',
    }

    sensor_remappings = [
        ('rgb/image', LaunchConfiguration('camera_raw_topic')),
        ('rgb/camera_info', LaunchConfiguration('calibrated_camera_info_topic')),
        ('scan', LaunchConfiguration('rtabmap_scan_topic')),
        ('odom', '/odom'),
        ('map', '/map'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value=os.path.join(dataset_dir, 'bag'),
            description='Path to the W1 rosbag2 directory.',
        ),
        DeclareLaunchArgument('playback_rate', default_value='1.0'),
        DeclareLaunchArgument(
            'qos_overrides_path',
            default_value=os.path.join(config_dir, 'bag_play_qos_overrides.yaml'),
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=os.path.join(dataset_dir, 'waypoints.json'),
        ),
        DeclareLaunchArgument('use_dynamic_filter', default_value='true'),
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='Use RGB appearance descriptors for loop-closure detection.',
        ),
        DeclareLaunchArgument(
            'rtabmap_scan_topic',
            default_value=PythonExpression([
                "'/scan_filtered' if '",
                LaunchConfiguration('use_dynamic_filter'),
                "' == 'true' else '/scan_normalized'",
            ]),
        ),
        DeclareLaunchArgument(
            'dynamic_filter_params_file',
            default_value=os.path.join(
                config_dir, 'dynamic_scan_filter_bag_replay.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'camera_compressed_topic',
            default_value='/camera/image_raw/compressed',
        ),
        DeclareLaunchArgument('camera_raw_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument(
            'calibrated_camera_info_topic',
            default_value='/camera/camera_info_rtabmap',
        ),
        DeclareLaunchArgument(
            'camera_calibration_file',
            default_value=camera_calibration,
            description='Approximate W1 calibration; replace after calibrating the lens.',
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value=four_way_default(database_paths),
            description='RTAB-Map database saved after the run.',
        ),
        DeclareLaunchArgument(
            'evaluation_output_path',
            default_value=four_way_default(metric_paths),
        ),
        DeclareLaunchArgument(
            'detection_rate',
            default_value='2.0',
            description='RTAB-Map graph/visual detection rate in Hz.',
        ),
        DeclareLaunchArgument(
            'visual_loop_threshold',
            default_value='0.11',
            description='RTAB-Map appearance loop-closure acceptance threshold.',
        ),
        DeclareLaunchArgument('use_waypoint_evaluator', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('use_image_view', default_value='false'),
        DeclareLaunchArgument('evaluation_settling_window_sec', default_value='1.0'),
        DeclareLaunchArgument('evaluation_window_sample_count', default_value='11'),
        DeclareLaunchArgument('evaluation_min_window_samples', default_value='5'),
        DeclareLaunchArgument('evaluation_alignment_mode', default_value='se2'),

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
            parameters=[{
                'use_sim_time': True,
                'input_twist_topic': '/xgo/applied_vel',
                'input_imu_topic': '/imu/data',
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_transform',
            arguments=[
                '--x', '0.14', '--y', '0.0', '--z', '0.10',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
            ],
            condition=IfCondition(LaunchConfiguration('use_camera')),
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='rtabmap_camera_republisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'in_transport': 'compressed',
                'out_transport': 'raw',
            }],
            remappings=[
                ('in/compressed', LaunchConfiguration('camera_compressed_topic')),
                ('out', LaunchConfiguration('camera_raw_topic')),
            ],
            condition=IfCondition(LaunchConfiguration('use_camera')),
        ),
        Node(
            package='rtabmap_util',
            executable='yaml_to_camera_info.py',
            name='rtabmap_camera_info',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_path': LaunchConfiguration('camera_calibration_file'),
                'frame_id': 'camera_link',
            }],
            remappings=[
                ('image', LaunchConfiguration('camera_raw_topic')),
                (
                    'camera_info',
                    LaunchConfiguration('calibrated_camera_info_topic'),
                ),
            ],
            condition=IfCondition(LaunchConfiguration('use_camera')),
        ),
        TimerAction(
            period=1.0,
            actions=[Node(
                package='rtabmap_slam',
                executable='rtabmap',
                namespace='rtabmap',
                name='rtabmap',
                output='screen',
                parameters=[rtabmap_parameters],
                remappings=sensor_remappings,
                arguments=['--delete_db_on_start'],
            )],
        ),
        Node(
            package='xgo_driver_bridge',
            executable='waypoint_accuracy_node',
            name='waypoint_accuracy_monitor',
            output='screen',
            parameters=[{
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
            }],
            condition=IfCondition(LaunchConfiguration('use_waypoint_evaluator')),
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
                    package='rtabmap_viz',
                    executable='rtabmap_viz',
                    namespace='rtabmap',
                    name='rtabmap_viz',
                    output='screen',
                    parameters=[rtabmap_parameters],
                    remappings=sensor_remappings,
                    condition=IfCondition(LaunchConfiguration('use_rtabmap_viz')),
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
            actions=[ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play', LaunchConfiguration('bag_path'),
                    '--clock', '--rate', LaunchConfiguration('playback_rate'),
                    '--qos-profile-overrides-path',
                    LaunchConfiguration('qos_overrides_path'),
                    '--disable-keyboard-controls',
                ],
                output='screen',
            )],
        ),
    ])
