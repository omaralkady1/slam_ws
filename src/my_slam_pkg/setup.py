from setuptools import setup
import os
from glob import glob

package_name = 'my_slam_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Include configuration files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Include map files
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omar',
    maintainer_email='your_email@example.com',
    description='SLAM and Navigation package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
