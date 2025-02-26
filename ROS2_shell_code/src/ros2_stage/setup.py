import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_stage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')), # Holds Gazebo .world and .inc files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='klotzl',
    maintainer_email='klotzl@oregonstate.edu',
    description='osu_stage ported to ROS2',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
