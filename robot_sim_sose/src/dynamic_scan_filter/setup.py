from setuptools import find_packages, setup

package_name = 'dynamic_scan_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'dynamic_scan_filter_node = dynamic_scan_filter.dynamic_scan_filter_node:main',
        ],
    },
)
