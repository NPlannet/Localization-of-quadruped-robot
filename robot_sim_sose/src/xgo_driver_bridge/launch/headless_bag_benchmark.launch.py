"""Run an input-only rosbag through a freshly started SLAM pipeline.

This launch intentionally replays an allowlist of original sensor topics. It
never replays recorded odometry, dynamic-filter output, regular TF, maps, or
SLAM diagnostics, so every derived value is recomputed during the benchmark.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def as_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def launch_setup(context):
    bridge_share = get_package_share_directory('xgo_driver_bridge')
    launch_dir = os.path.join(bridge_share, 'launch')
    config_dir = os.path.join(bridge_share, 'config')

    method = LaunchConfiguration('slam_method').perform(context).strip()
    if method not in {'slam_toolbox', 'cartographer', 'rtabmap'}:
        raise RuntimeError(
            'slam_method must be slam_toolbox, cartographer, or rtabmap; '
            f'got {method!r}.'
        )

    bag_path = os.path.abspath(
        os.path.expanduser(LaunchConfiguration('bag_path').perform(context))
    )
    if not os.path.isfile(os.path.join(bag_path, 'metadata.yaml')):
        raise RuntimeError(
            f'Bag path must contain metadata.yaml: {bag_path}'
        )

    use_filter = as_bool(
        LaunchConfiguration('use_dynamic_filter').perform(context)
    )
    normalize_scan = as_bool(
        LaunchConfiguration('normalize_scan').perform(context)
    )
    use_camera = as_bool(LaunchConfiguration('use_camera').perform(context))
    use_evaluator = as_bool(
        LaunchConfiguration('use_waypoint_evaluator').perform(context)
    )
    start_foxglove = as_bool(
        LaunchConfiguration('start_foxglove').perform(context)
    )

    if use_camera and method != 'rtabmap':
        raise RuntimeError(
            'use_camera is supported only for the RTAB-Map benchmark. '
            'Keep it false for SLAM Toolbox and Cartographer CPU trials.'
        )

    waypoints_file = os.path.abspath(
        os.path.expanduser(
            LaunchConfiguration('waypoints_file').perform(context)
        )
    )
    if use_evaluator and not os.path.isfile(waypoints_file):
        raise RuntimeError(
            f'Waypoint evaluation requested but file is missing: {waypoints_file}'
        )

    original_scan_topic = '/scan'
    normalized_scan_topic = '/scan_normalized'
    filter_output_topic = '/scan_filtered'
    filter_input_topic = (
        normalized_scan_topic if normalize_scan else original_scan_topic
    )
    mapper_scan_topic = (
        filter_output_topic if use_filter else filter_input_topic
    )
    
    velocity_scale = float(LaunchConfiguration('velocity_scale').perform(context))
    if velocity_scale != 1.0:
        odom_input_topic = '/xgo/applied_vel_scaled'
    else:
        odom_input_topic = '/xgo/applied_vel'

    actions = [
        LogInfo(
            msg=(
                f'Headless bag benchmark: method={method}, '
                f'scan={mapper_scan_topic}, camera={use_camera}, '
                f'foxglove={start_foxglove}. Recorded /odom, /tf, '
                '/scan_filtered, /map, and SLAM outputs are excluded.'
            )
        ),
    ]
    
    if velocity_scale != 1.0:
        actions.append(
            Node(
                package='xgo_driver_bridge',
                executable='velocity_scale_node',
                name='velocity_scale_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'input_topic': '/xgo/applied_vel',
                    'output_topic': '/xgo/applied_vel_scaled',
                    'scale': velocity_scale,
                }],
            )
        )
        
    actions.append(
        Node(
            package='xgo_driver_bridge',
            executable='xgo_offline_odom_node',
            name='xgo_offline_odom',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'input_twist_topic': odom_input_topic,
                'input_imu_topic': '/imu/data',
                'odom_topic': '/odom',
                'publish_tf': True,
            }],
        ),
    )
        
        

    if normalize_scan:
        actions.append(
            Node(
                package='dynamic_scan_filter',
                executable='scan_normalizer_node',
                name='scan_normalizer',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'input_scan_topic': original_scan_topic,
                    'output_scan_topic': normalized_scan_topic,
                }],
            )
        )

    if use_filter:
        actions.append(
            Node(
                package='dynamic_scan_filter',
                executable='dynamic_scan_filter_node',
                name='dynamic_scan_filter',
                output='screen',
                parameters=[
                    LaunchConfiguration('dynamic_filter_params_file'),
                    {
                        'use_sim_time': True,
                        'input_scan_topic': filter_input_topic,
                        'output_scan_topic': filter_output_topic,
                        'tracking_frame': 'odom',
                    },
                ],
            )
        )

    if method == 'slam_toolbox':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'slam_toolbox_robot.launch.py')
                ),
                launch_arguments={
                    'params_file': LaunchConfiguration(
                        'slam_toolbox_params_file'
                    ),
                    'scan_topic': mapper_scan_topic,
                    'use_sim_time': 'true',
                }.items(),
            )
        )
    elif method == 'cartographer':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'cartographer_robot.launch.py')
                ),
                launch_arguments={
                    'configuration_directory': config_dir,
                    'configuration_basename': LaunchConfiguration(
                        'cartographer_config_basename'
                    ),
                    'scan_topic': mapper_scan_topic,
                    'odom_topic': '/odom',
                    'imu_topic': '/imu/data',
                    'use_sim_time': 'true',
                    'use_occupancy_grid': 'true',
                }.items(),
            )
        )
    else:
        if use_camera:
            actions.extend([
                Node(
                    package='image_transport',
                    executable='republish',
                    name='benchmark_camera_republisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'in_transport': 'compressed',
                        'out_transport': 'raw',
                        'qos_overrides./camera/image_raw/compressed.subscription.reliability': 'best_effort',
                        'qos_overrides./camera/image_raw/compressed.subscription.durability': 'volatile',
                    }],
                    remappings=[
                        ('in/compressed', '/camera/image_raw/compressed'),
                        ('out', '/camera/image_raw'),
                    ],
                ),
                Node(
                    package='rtabmap_util',
                    executable='yaml_to_camera_info.py',
                    name='benchmark_camera_info',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'yaml_path': LaunchConfiguration(
                            'camera_calibration_file'
                        ),
                        'frame_id': 'camera_optical_frame',
                    }],
                    remappings=[
                        ('image', '/camera/image_raw'),
                        ('camera_info', '/camera/camera_info_benchmark'),
                    ],
                ),
            ])
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'rtabmap_robot.launch.py')
                ),
                launch_arguments={
                    'scan_topic': mapper_scan_topic,
                    'odom_topic': '/odom',
                    'camera_topic': '/camera/image_raw',
                    'camera_info_topic': '/camera/camera_info_benchmark',
                    'use_camera': 'true' if use_camera else 'false',
                    'use_sim_time': 'true',
                    'database_path': LaunchConfiguration(
                        'rtabmap_database_path'
                    ),
                    'delete_database_on_start': 'true',
                    'detection_rate': LaunchConfiguration(
                        'rtabmap_detection_rate'
                    ),
                    'visual_loop_threshold': LaunchConfiguration(
                        'rtabmap_visual_loop_threshold'
                    ),
                }.items(),
            )
        )

    if use_evaluator:
        actions.append(
            Node(
                package='xgo_driver_bridge',
                executable='waypoint_accuracy_node',
                name='waypoint_accuracy_monitor',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'waypoints_file': waypoints_file,
                    'output_path': LaunchConfiguration(
                        'evaluation_output_path'
                    ),
                    'settling_window_sec': LaunchConfiguration(
                        'evaluation_settling_window_sec'
                    ),
                    'window_sample_count': LaunchConfiguration(
                        'evaluation_window_sample_count'
                    ),
                    'min_window_samples': LaunchConfiguration(
                        'evaluation_min_window_samples'
                    ),
                    'alignment_mode': LaunchConfiguration(
                        'evaluation_alignment_mode'
                    ),
                }],
            )
        )

    if start_foxglove:
        actions.append(
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen',
                parameters=[
                    LaunchConfiguration('foxglove_params_file'),
                    {
                        'port': int(
                            LaunchConfiguration('foxglove_port').perform(context)
                        ),
                        'use_sim_time': True,
                    },
                ],
            )
        )

    # Only original sensor inputs are replayed. In particular, this excludes
    # recorded /odom, /tf, /scan_filtered, /map, and every mapper output.
    replay_topics = [
        '/scan',
        '/imu/data',
        '/xgo/applied_vel',
        '/tf_static',
    ]
    if use_camera:
        replay_topics.append('/camera/image_raw/compressed')

    bag_player = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'play',
            bag_path,
            '--clock',
            '--rate',
            LaunchConfiguration('playback_rate'),
            '--qos-profile-overrides-path',
            LaunchConfiguration('qos_overrides_path'),
            '--disable-keyboard-controls',
            '--topics',
            *replay_topics,
        ],
        output='screen',
    )
    shutdown_delay = float(
        LaunchConfiguration('post_playback_delay_sec').perform(context)
    )
    actions.extend([
        RegisterEventHandler(
            OnProcessExit(
                target_action=bag_player,
                on_exit=[
                    LogInfo(
                        msg=(
                            'Bag playback finished; allowing '
                            f'{shutdown_delay:.1f}s for queued processing.'
                        )
                    ),
                    TimerAction(
                        period=shutdown_delay,
                        actions=[
                            EmitEvent(
                                event=Shutdown(
                                    reason='Headless bag benchmark complete.'
                                )
                            )
                        ],
                    ),
                ],
            )
        ),
        TimerAction(period=5.0, actions=[bag_player]),
    ])
    return actions


def generate_launch_description():
    bridge_share = get_package_share_directory('xgo_driver_bridge')
    config_dir = os.path.join(bridge_share, 'config')
    workspace = os.environ.get('WORKSPACE', '/workspaces/robot_sim_sose')

    return LaunchDescription([
        DeclareLaunchArgument('bag_path'),
        DeclareLaunchArgument(
            'slam_method',
            description='slam_toolbox, cartographer, or rtabmap',
        ),
        DeclareLaunchArgument('use_dynamic_filter', default_value='false'),
        DeclareLaunchArgument(
            'normalize_scan',
            default_value='false',
            description='Enable only for irregular legacy scans such as W1.',
        ),
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('playback_rate', default_value='1.0'),
        DeclareLaunchArgument('post_playback_delay_sec', default_value='5.0'),
        DeclareLaunchArgument(
            'qos_overrides_path',
            default_value=os.path.join(
                config_dir, 'bag_benchmark_play_qos.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'dynamic_filter_params_file',
            default_value=os.path.join(
                config_dir, 'dynamic_scan_filter_robot.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'slam_toolbox_params_file',
            default_value=os.path.join(
                config_dir, 'slam_toolbox_robot.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'cartographer_config_basename',
            default_value='cartographer_robot_2d.lua',
        ),
        DeclareLaunchArgument(
            'rtabmap_database_path',
            default_value=os.path.join(
                workspace,
                'evaluation',
                'runs',
                'bag_benchmark_rtabmap.db',
            ),
        ),
        DeclareLaunchArgument('rtabmap_detection_rate', default_value='2.0'),
        DeclareLaunchArgument(
            'rtabmap_visual_loop_threshold', default_value='0.11'
        ),
        DeclareLaunchArgument(
            'camera_calibration_file',
            default_value=os.path.join(
                config_dir, 'xgo_camera_640x480_approx.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'use_waypoint_evaluator', default_value='false'
        ),
        DeclareLaunchArgument('waypoints_file', default_value=''),
        DeclareLaunchArgument('evaluation_output_path', default_value=''),
        DeclareLaunchArgument(
            'evaluation_settling_window_sec', default_value='1.0'
        ),
        DeclareLaunchArgument(
            'evaluation_window_sample_count', default_value='11'
        ),
        DeclareLaunchArgument(
            'evaluation_min_window_samples', default_value='5'
        ),
        DeclareLaunchArgument('evaluation_alignment_mode', default_value='se2'),
        DeclareLaunchArgument('start_foxglove', default_value='false'),
        DeclareLaunchArgument('foxglove_port', default_value='8766'),
        DeclareLaunchArgument(
            'foxglove_params_file',
            default_value=os.path.join(
                config_dir, 'foxglove_bridge_robot.yaml'
            ),
        ),
        DeclareLaunchArgument('velocity_scale', default_value='1.0'),
        OpaqueFunction(function=launch_setup),
    ])
