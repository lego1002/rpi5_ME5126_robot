#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def _generate_map(context, *args, **kwargs):
    pkg = get_package_share_directory('rpi5_robot_localization')
    script = os.path.join(pkg, 'scripts', 'map_generator.py')
    # 直接呼叫你的 map_generator.py
    subprocess.run(['python3', script], check=True)
    return []

def generate_launch_description():
    pkg = get_package_share_directory('rpi5_robot_localization')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    map_yaml = '/tmp/rpi5_auto_map/auto_map.yaml'
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    rviz_cfg   = os.path.join(pkg, 'rviz2', 'rpi5_nav2.rviz')

    # 1) 自動產圖
    gen_map = OpaqueFunction(function=_generate_map)

    # 2) Encoder 與 Odometry，並把 /wheel/odom remap 到 /odom
    enc_node = Node(
        package='rpi5_robot_localization',
        executable='rpi5_encoder_node',
        name='rpi5_encoder_node',
        output='screen',
        parameters=[{'pulses_per_rev': 388}],
    )
    odom_node = Node(
        package='rpi5_robot_localization',
        executable='rpi5_odometry_node',
        name='rpi5_odometry_node',
        output='screen',
        parameters=[
            {'ppr': 388},
            {'wheel_radius': 0.04},
            {'wheel_base': 0.17},
        ],
        remappings=[
            ('/wheel/odom', '/odom'),
        ],
    )

    # 3) cmd_vel → Arduino
    serial_node = Node(
        package='rpi5_robot_localization',
        executable='rpi5_serial_bridge_node',
        name='rpi5_serial_bridge_node',
        output='screen',
        parameters=[
            {'port': '/dev/ttyACM0'},
            {'baudrate': 115200},
        ],
    )

    # 4) Nav2 官方 bringup
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav2_params,
            'use_sim_time': 'false',
        }.items()
    )

    # 5) RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
    )

    return LaunchDescription([
        gen_map,
        enc_node,
        odom_node,
        serial_node,
        nav2,
        rviz_node,
    ])
