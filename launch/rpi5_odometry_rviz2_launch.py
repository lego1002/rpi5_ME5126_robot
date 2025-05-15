#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('rpi5_robot_localization')
    map_yaml = '/tmp/rpi5_auto_map/auto_map.yaml'
    rviz_cfg = os.path.join(pkg, 'rviz2', 'rpi5_nav2.rviz')

    return LaunchDescription([
        # 1) 產生格子地圖
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg, 'scripts', 'map_generator.py')],
            output='screen'
        ),

        # 2) Map Server
        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[{'yaml_filename': map_yaml}],
        ),

        # 3) static TF: map → odom，起點 (0,90px)=>(0m,0.90m)，並加 90° 旋轉讓車頭對齊 X+
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom', output='screen',
            arguments=[
                '0', '0.90', '0',        # x=0m, y=0.90m, z=0m
                '0', '0', '0.7071', '0.7071',  # qx=0,qy=0,qz=sin45°,qw=cos45°
                'map', 'odom'
            ],
        ),

        # 4) Encoder & Wheel→Odometry
        Node(
            package='rpi5_robot_localization', executable='rpi5_encoder_node',
            name='rpi5_encoder_node', output='screen',
            parameters=[{'pulses_per_rev': 388}],
        ),
        Node(
            package='differential_drive', executable='diff_tf',
            name='wheel_odom_node', output='screen',
            parameters=[{
                'rate': 30.0,
                'ticks_meter': 388/(2*3.14159*0.04),
                'base_width': 0.17,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
            }],
            remappings=[
                ('lwheel', '/wheel/encoder_left'),
                ('rwheel', '/wheel/encoder_right'),
                ('odom',   '/wheel/odom'),
            ],
        ),
        Node(
            package='rpi5_robot_localization', executable='rpi5_odometry_node',
            name='rpi5_odometry_node', output='screen',
            parameters=[
                {'ppr': 388},
                {'wheel_radius': 0.04},
                {'wheel_base': 0.17},
            ],
        ),

        # 5) Serial Bridge (cmd_vel → PWM)
        Node(
            package='rpi5_robot_localization', executable='rpi5_serial_bridge_node',
            name='rpi5_serial_bridge_node', output='screen',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baudrate': 115200},
                {'wheel_base': 0.17},
                {'max_pwm': 255},
                {'max_linear_speed': 0.2},
                {'max_angular_speed': 1.0},
            ],
        ),

        # 6) GUI for goal/stop/speed
        Node(
            package='rpi5_robot_localization', executable='input_goal',
            name='input_goal', output='screen',
        ),

        # 7) RViz2
        Node(
            package='rviz2', executable='rviz2',
            name='rviz2', output='screen',
            arguments=['-d', rviz_cfg],
        ),
    ])
