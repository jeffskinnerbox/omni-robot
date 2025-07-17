from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_gamepad_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Turtle gamepad controller with emergency stop',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_gamepad_controller = turtle_gamepad_2.turtle_gamepad_controller:main',
            'emergency_stop_monitor = turtle_gamepad_2.emergency_stop_monitor:main',
        ],
    },
)
