import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marcel Zawadzki',
    maintainer_email='01158344@pw.edu.pl',
    description='Package implementing custom lidar driver for ROS',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scanner = rover_lidar.rover_lidar_scanner:main',
            'controller = rover_lidar.rover_lidar_controller:main',
            'fake_odom = rover_lidar.rover_lidar_fake_odom:main',
        ],
    },
)
