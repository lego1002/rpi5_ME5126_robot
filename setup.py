#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from setuptools import setup, find_packages

here = os.path.abspath(os.path.dirname(__file__))
package_name = 'rpi5_robot_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'launch', 'config', 'rviz2']),
    data_files=[
        # Package manifest
        (f'share/{package_name}', [
            'package.xml',
        ]),

        # Configuration files
        (f'share/{package_name}/config', [
            os.path.join('config', 'ekf.yaml'),
            os.path.join('config', 'nav2_params.yaml'),
        ]),

        # Launch files
        (f'share/{package_name}/launch', [
            os.path.join('launch', 'rpi5_localization_launch.py'),
            os.path.join('launch', 'rpi5_cant_move_sim_launch.py'),
            os.path.join('launch', 'nav2_bringup_with_rviz_launch.py'),
            os.path.join('launch', 'rpi5_odometry_rviz2_launch.py'),
            os.path.join('launch', 'nav2_bringup_navigation_launch.py'),
        ]),

        # Scripts
        (f'share/{package_name}/scripts', [
            os.path.join('scripts', 'map_generator.py'),
            os.path.join('scripts', 'input_goal.py'),
        ]),

        # RViz2 configurations
        (f'share/{package_name}/rviz2', [
            os.path.join('rviz2', 'rpi5_nav2.rviz'),
            os.path.join('rviz2', 'rpi5_cant_move.rviz'),
        ]),
    ],
    install_requires=[
        'setuptools',
        'gpiod',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='lego_Dai',
    maintainer_email='gdteyuj123@gmail.com',
    description='RPi5 localization + motor control',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'rpi5_encoder_node       = rpi5_robot_localization.rpi5_encoder_node:main',
            'rpi5_imu_node           = rpi5_robot_localization.rpi5_imu_node:main',
            'rpi5_serial_bridge_node = rpi5_robot_localization.rpi5_serial_bridge_node:main',
            'rpi5_odometry_node      = rpi5_robot_localization.rpi5_odometry_node:main',
            'rpi5_control_node       = rpi5_robot_localization.rpi5_control_node:main',
        ],
    },
)
