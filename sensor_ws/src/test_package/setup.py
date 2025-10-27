from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'test_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krri',
    maintainer_email='jungjjang44@g.skku.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'image_publisher = test_package.imagePublisher:main',
            'cam_lidar_calibration_node = test_package.cam2LidarCalibrationNode:main',
            'test = test_package.test:main',
        ],
    },
)
