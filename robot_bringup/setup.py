from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Include config files (URDF, EKF yaml)
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Navid Noroozi',
    maintainer_email='navidnoroozi@gmail.com',
    description='Approach 1 — custom serial bridge for 2WD robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge_node = robot_bringup.serial_bridge_node:main',
            'mpu6050_node       = robot_bringup.mpu6050_node:main',
        ],
    },
)
