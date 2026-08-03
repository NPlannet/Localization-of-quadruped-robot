from glob import glob
from setuptools import find_packages, setup

package_name = 'xgo_driver_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml') + glob('config/*.lua'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='ROS 2 bridge for the XGO Mini2 Python SDK.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'xgo_bridge_node = xgo_driver_bridge.xgo_bridge_node:main',
            'xgo_offline_odom_node = xgo_driver_bridge.offline_odom_node:main',
            'waypoint_accuracy_node = xgo_driver_bridge.waypoint_accuracy_node:main',
        ],
    },
)
