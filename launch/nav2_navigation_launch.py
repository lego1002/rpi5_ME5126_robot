#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('rpi5_robot_localization')

    # 地图要放的临时目录
    map_dir = '/tmp/rpi5_auto_map'
    map_yaml = os.path.join(map_dir, 'auto_map.yaml')

    # 1) 动态生成地图（PGM + YAML）
    map_gen = ExecuteProcess(
        cmd=[
            'python3', os.path.join(pkg_share, 'scripts', 'map_generator.py')
        ],
        output='screen'
    )

    # 2) nav2 参数文件（可以通过 --params_file 参数覆盖）
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 parameters file'
    )

    # 3) RViz 配置
    rviz_cfg = os.path.join(pkg_share, 'rviz2', 'rpi5_nav2.rviz')

    # 4) 当 map_gen 完成后，再启动后面的 Nav2 + RViz
    bringup = RegisterEventHandler(
        OnProcessExit(
            target_action=map_gen,
            on_exit=[
                # map_server
                Node(
                    package='nav2_map_server', executable='map_server',
                    name='map_server', output='screen',
                    parameters=[{'yaml_filename': map_yaml}],
                    remappings=[('/odom', '/wheel/odom')],
                ),
                # AMCL
                Node(
                    package='nav2_amcl', executable='amcl',
                    name='amcl', output='screen',
                    parameters=[LaunchConfiguration('params_file')],
                    remappings=[('/odom', '/wheel/odom')],
                ),
                # planner_server
                Node(
                    package='nav2_planner', executable='planner_server',
                    name='planner_server', output='screen',
                    parameters=[LaunchConfiguration('params_file')],
                    remappings=[('/odom', '/wheel/odom')],
                ),
                # controller_server
                Node(
                    package='nav2_controller', executable='controller_server',
                    name='controller_server', output='screen',
                    parameters=[LaunchConfiguration('params_file')],
                    remappings=[
                        ('/odom', '/wheel/odom'),
                        ('/cmd_vel', '/cmd_vel'),
                    ],
                ),
                # bt_navigator
                Node(
                    package='nav2_bt_navigator', executable='bt_navigator',
                    name='bt_navigator', output='screen',
                    parameters=[LaunchConfiguration('params_file')],
                    remappings=[('/odom', '/wheel/odom')],
                ),
                # lifecycle_manager
                Node(
                    package='nav2_lifecycle_manager', executable='lifecycle_manager',
                    name='lifecycle_manager_navigation', output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'autostart': True,
                        'node_names': [
                            'map_server',
                            'amcl',
                            'planner_server',
                            'controller_server',
                            'bt_navigator'
                        ]
                    }],
                    remappings=[('/odom', '/wheel/odom')],
                ),
                # RViz
                Node(
                    package='rviz2', executable='rviz2',
                    name='rviz2', output='screen',
                    arguments=['-d', rviz_cfg],
                ),
            ]
        )
    )

    return LaunchDescription([
        map_gen,
        params_arg,
        bringup,
    ])
