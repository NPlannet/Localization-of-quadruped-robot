import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable

def generate_launch_description():

    pkg_share = get_package_share_directory('xgo_description')

    # Add this action to set the path automatically
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_share, '..')]
    )
    
    pkg_share = get_package_share_directory('xgo_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'xgo.urdf')

    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Gazebo Sim Launch (New for Jazzy)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 3. Spawn the robot in Gazebo Sim (New executable name)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'xgo_robot', '-string', robot_desc],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,  # Add this here
        gz_sim,
        robot_state_publisher,
        spawn_entity
    ])