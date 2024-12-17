from setuptools import setup
import os
from glob import glob

package_name = 'robot_catch_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'std_msgs',
        'geometry_msgs',
        'vision_msgs',
        'stereo_msgs',
        'cv_bridge',
        'image_transport',
        'depthai',
        'camera_info_manager',
    ],
    zip_safe=True,
    maintainer='Shiwen Yang',
    maintainer_email='sy796@cornell.edu',
    description='A ROS2 package for an interactive cat-and-mouse game',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_catch_node = robot_catch_pkg.robot_catch_node:main',
        ],
    },
)
