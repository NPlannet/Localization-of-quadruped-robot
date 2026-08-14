import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def prepare_database_directory(context):
    database_path = os.path.abspath(
        os.path.expanduser(
            LaunchConfiguration('database_path').perform(context)
        )
    )
    database_directory = os.path.dirname(database_path)

    try:
        os.makedirs(database_directory, exist_ok=True)
    except OSError as exc:
        raise RuntimeError(
            'Cannot create the RTAB-Map database directory '
            f'{database_directory!r}: {exc}'
        ) from exc

    if not os.access(database_directory, os.W_OK):
        raise RuntimeError(
            'RTAB-Map database directory is not writable: '
            f'{database_directory}'
        )

    return [
        LogInfo(msg=f'RTAB-Map database: {database_path}'),
    ]


def generate_launch_description():
    workspace = os.environ.get('WORKSPACE', '/workspaces/robot_sim_sose')

    parameters = {
        'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'),
            value_type=bool,
        ),
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': '',
        'publish_tf': True,
        'subscribe_depth': False,
        'subscribe_rgbd': False,
        'subscribe_stereo': False,
        'subscribe_rgb': ParameterValue(
            LaunchConfiguration('use_camera'),
            value_type=bool,
        ),
        'subscribe_scan': True,
        'subscribe_scan_cloud': False,
        'approx_sync': True,
        'approx_sync_max_interval': 0.10,
        'topic_queue_size': 50,
        'sync_queue_size': 30,
        'qos_image': 1,
        'qos_camera_info': 1,
        'qos_scan': ParameterValue(
            LaunchConfiguration('qos_scan'),
            value_type=int,
        ),
        'qos_odom': 1,
        'wait_for_transform': 1.0,
        'database_path': LaunchConfiguration('database_path'),
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        'Mem/ImagePreDecimation': '1',
        'Mem/ImagePostDecimation': '1',
        'Kp/DetectorStrategy': '8',
        'Kp/MaxFeatures': '500',
        'Rtabmap/DetectionRate': ParameterValue(
            LaunchConfiguration('detection_rate'),
            value_type=str,
        ),
        'Rtabmap/LoopThr': ParameterValue(
            LaunchConfiguration('visual_loop_threshold'),
            value_type=str,
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
        'Grid/Sensor': '0',
        'Grid/3D': 'false',
        'Grid/CellSize': '0.05',
        'Grid/RangeMin': '0.05',
        'Grid/RangeMax': '12.0',
        'Grid/Scan2dUnknownSpaceFilled': 'true',
    }

    remappings = [
        ('rgb/image', LaunchConfiguration('camera_topic')),
        ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
        ('scan', LaunchConfiguration('scan_topic')),
        ('odom', LaunchConfiguration('odom_topic')),
        ('map', '/map'),
    ]

    common = {
        'package': 'rtabmap_slam',
        'executable': 'rtabmap',
        'namespace': 'rtabmap',
        'name': 'rtabmap',
        'output': 'screen',
        'parameters': [parameters],
        'remappings': remappings,
    }

    preserve_database = Node(
        **common,
        condition=UnlessCondition(
            LaunchConfiguration('delete_database_on_start')
        ),
    )
    reset_database = Node(
        **common,
        arguments=['--delete_db_on_start'],
        condition=IfCondition(
            LaunchConfiguration('delete_database_on_start')
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument(
            'qos_scan',
            default_value='1',
            description=(
                'rtabmap_ros QoS enum for its scan subscriber: 1=reliable, '
                '2=best_effort. Must match the reliability of whatever '
                'topic scan_topic points at, or the subscription silently '
                'never receives data. Default (1/reliable) matches '
                '/scan_filtered, which dynamic_scan_filter republishes with '
                "rclcpp's reliable default. Raw /scan bag playback is "
                'forced to best_effort by bag_benchmark_play_qos.yaml, so '
                'callers pointing scan_topic at /scan must pass 2 here.'
            ),
        ),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera_info',
        ),
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='Use RGB appearance descriptors for loop recognition.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'database_path',
            default_value=os.path.join(
                workspace,
                'maps',
                'rtabmap_robot.db',
            ),
        ),
        DeclareLaunchArgument(
            'delete_database_on_start',
            default_value='false',
            description='Explicitly erase the selected RTAB-Map database.',
        ),
        DeclareLaunchArgument('detection_rate', default_value='2.0'),
        DeclareLaunchArgument('visual_loop_threshold', default_value='0.11'),
        OpaqueFunction(function=prepare_database_directory),
        preserve_database,
        reset_database,
    ])