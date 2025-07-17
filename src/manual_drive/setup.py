
from setuptools import setup
import os
from glob import glob

package_name = 'manual_drive'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Manual drive control for turtlesim',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emergency_stop_node = manual_drive.emergency_stop_node:main',
            'keyboard_teleop_node = manual_drive.keyboard_teleop_node:main',
            'safety_monitor_node = manual_drive.safety_monitor_node:main',
        ],
    },
)
