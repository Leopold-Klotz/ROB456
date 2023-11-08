from setuptools import setup

package_name = 'lab0_r2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Directory 'config' and 'launch' are not Python packages, but we still need
        # to install them for use with the package. They contain configuration and 
        # launch files.
        ('share/' + package_name + '/config', ['config/circler.rviz']),
        ('share/' + package_name + '/launch', ['launch/circle.launch', 'launch/test.launch']),
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
            'circler = lab0_r2.circler.py',
            'publisher = lab0_r2.publisher.py',
            'subscriber = lab0_r2.subscriber.py',
        ],
    },
)
