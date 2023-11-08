import os
from setuptools import setup
from glob import glob

package_name = 'lab0_r2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), 
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), 

        ('share/' + package_name + '/config', ['config/circler.rviz2']),
        # If you have other directories or files like messages or services definitions,
        # they should be listed here too.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lk',
    maintainer_email='lk@oregonstate.edu',
    description='Lab 0 package adjusted for ros2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circler = lab0_r2.circler:main',
            'publisher = lab0_r2.publisher:main',
            'subscriber = lab0_r2.subscriber:main',
        ],
    },
)

