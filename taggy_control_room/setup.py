from setuptools import setup
import os
from glob import glob

package_name = 'taggy_control_room'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/control_room', glob('control_room/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manolis Efthymiou',
    maintainer_email='manolis@example.com',
    description='TAGGY Control Room - Voice commands and web UI',
    license='MIT',
    entry_points={
        'console_scripts': [
            'voice_command_node = taggy_control_room.voice_command_node:main',
            'mission_monitor_node = taggy_control_room.mission_monitor_node:main',
        ],
    }
)