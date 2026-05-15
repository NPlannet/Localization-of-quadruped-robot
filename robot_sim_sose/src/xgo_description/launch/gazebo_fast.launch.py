import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


COLLISION_BOXES = {
    'arm_calf.STL': ('0.08740 0.03665 0.04032', '0.03350 0.01858 0.01550'),
    'arm_thigh.STL': ('0.07250 0.03720 0.02264', '-0.02975 0.01860 -0.00162'),
    'bl_calf.STL': ('0.03439 0.03665 0.09049', '-0.01254 -0.01882 -0.03324'),
    'bl_foot.STL': ('0.02000 0.01100 0.01707', '0.00000 0.00000 -0.00146'),
    'bl_hip.STL': ('0.03670 0.05397 0.02965', '-0.01830 0.02232 0.00733'),
    'bl_thigh.STL': ('0.02264 0.03720 0.07250', '-0.00162 -0.01860 -0.02975'),
    'body.STL': ('0.21260 0.06592 0.05904', '-0.00108 -0.00001 0.01239'),
    'br_calf.STL': ('0.03439 0.03665 0.09049', '-0.01254 0.01882 -0.03324'),
    'br_foot.STL': ('0.02000 0.01100 0.01707', '0.00000 0.00000 -0.00146'),
    'br_hip.STL': ('0.03670 0.05397 0.02965', '-0.01830 -0.02232 0.00733'),
    'br_thigh.STL': ('0.02294 0.03720 0.07250', '-0.00147 0.01860 -0.02975'),
    'fl_calf.STL': ('0.03439 0.03665 0.09049', '-0.01254 -0.01882 -0.03324'),
    'fl_foot.STL': ('0.02000 0.01100 0.01707', '0.00000 0.00000 -0.00146'),
    'fl_hip.STL': ('0.03670 0.05397 0.02965', '-0.01880 0.02233 0.00733'),
    'fl_thigh.STL': ('0.02264 0.03720 0.07250', '-0.00162 -0.01860 -0.02975'),
    'fr_calf.STL': ('0.03439 0.03665 0.09049', '-0.01254 0.01882 -0.03324'),
    'fr_foot.STL': ('0.02000 0.01100 0.01707', '0.00000 0.00000 -0.00146'),
    'fr_hip.STL': ('0.03670 0.05397 0.02965', '-0.01880 -0.02232 0.00733'),
    'fr_thigh.STL': ('0.02294 0.03720 0.07250', '-0.00147 0.01860 -0.02975'),
    'hand_left.STL': ('0.03715 0.03500 0.02455', '0.01752 -0.00430 0.00862'),
    'hand_right.STL': ('0.03715 0.03500 0.02455', '0.01752 0.00430 -0.00862'),
}


def use_box_collisions(robot_desc):
    root = ET.fromstring(robot_desc)

    for collision in root.findall('.//collision'):
        mesh = collision.find('./geometry/mesh')
        if mesh is None:
            continue

        mesh_file = os.path.basename(mesh.attrib.get('filename', ''))
        if mesh_file not in COLLISION_BOXES:
            continue

        size, center = COLLISION_BOXES[mesh_file]
        geometry = collision.find('geometry')
        geometry.clear()
        ET.SubElement(geometry, 'box', {'size': size})

        origin = collision.find('origin')
        if origin is None:
            ET.SubElement(collision, 'origin', {'xyz': center, 'rpy': '0 0 0'})
        else:
            origin.set('xyz', center)

    return ET.tostring(root, encoding='unicode')


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('xgo_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'xgo.urdf')
    world_file = LaunchConfiguration('world').perform(context)

    with open(urdf_file, 'r') as infp:
        robot_desc = use_box_collisions(infp.read())

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
        arguments=['-name', 'xgo_robot', '-string', robot_desc],
        output='screen',
    )

    return [gz_sim, robot_state_publisher, spawn_entity]


def generate_launch_description():
    pkg_share = get_package_share_directory('xgo_description')
    default_world = os.path.join(pkg_share, 'worlds', 'slam_test_world.sdf')

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
            name='GZ_SIM_RESOURCE_PATH',
            value=[os.path.join(pkg_share, '..')],
        ),
        OpaqueFunction(function=launch_setup),
    ])
