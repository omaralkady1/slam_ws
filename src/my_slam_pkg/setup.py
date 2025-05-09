#!/usr/bin/env python3
# Add system path to help IDE resolving imports
import sys
import os
from glob import glob

try:
    from setuptools import setup, find_packages
except ImportError:
    # Fallback to distutils if setuptools is not available
    from distutils.core import setup

package_name = 'my_slam_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.tf_transformations'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A package for SLAM implementation with ROS2 control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_goal_sender = my_slam_pkg.nav_goal_sender:main',
            'send_waypoints = my_slam_pkg.send_waypoints:main',
            'nav_goal_listener = my_slam_pkg.nav_goal_listener:main',
            'nav_goal_client = my_slam_pkg.nav_goal_client:main',
            'nav_visualization = my_slam_pkg.nav_visualization:main',
            'navigate_to_coordinates = my_slam_pkg.navigate_to_coordinates:main',
        ],
    },
)