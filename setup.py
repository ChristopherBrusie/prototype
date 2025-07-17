from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'prototype'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), #added
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='chris@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_node = prototype.motor_driver_node:main',
            'teleop_twist_keyboard = prototype.teleop_twist_keyboard:main',
            'imu_node = prototype.imu_node:main',
        ],
    },
)
