#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('rpi5_robot_localization')

    # 1) 參數：地圖、Nav2 設定、RViz config
    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml',
        default_value='/tmp/rpi5_auto_map/auto_map.yaml',
        description='YAML map file'
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
        description='Nav2 parameters file'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg, 'rviz2', 'rpi5_nav2.rviz'),
        description='RViz config file'
    )

    # 2) 先用 Python script 自動產生地圖
    map_gen = ExecuteProcess(
        cmd=['python3', os.path.join(pkg, 'scripts', 'map_generator.py')],
        output='screen'
    )

    # 3) RPi5 自製 pipeline：Encoder -> Odometry -> SerialBridge
    encoder = Node(
        package='rpi5_robot_localization', executable='rpi5_encoder_node',
        name='rpi5_encoder_node', output='screen',
        parameters=[{'pulses_per_rev': 388}],
    )
    odom = Node(
        package='rpi5_robot_localization', executable='rpi5_odometry_node',
        name='rpi5_odometry_node', output='screen',
        parameters=[
            {'ppr': 388},
            {'wheel_radius': 0.04},
            {'wheel_base': 0.17},
        ],
    )
    serial_bridge = Node(
        package='rpi5_robot_localization', executable='rpi5_serial_bridge_node',
        name='rpi5_serial_bridge_node', output='screen',
        parameters=[
            {'port': '/dev/ttyACM0'},
            {'baudrate': 115200},
            # 以下供動態調參或 rqt_reconfigure
            {'max_pwm': 255},
            {'max_linear_speed': 0.2},
            {'max_angular_speed': 1.0},
        ],
    )

    # 4) Nav2 核心節點
    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_yaml')}],
    )

    # 靜態 TF：把 map 座標系對齊到 odom，並設定車子起點為 (0,0.90,0)
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_to_odom', output='screen',
        arguments=[
            '0', '0.90', '0',   # x=0m, y=0.90m, z=0
            '0', '0', '0', '1', # quaternion (無旋轉)
            'map', 'odom'
        ],
    )

    # AMCL
    amcl = Node(
        package='nav2_amcl', executable='amcl',
        name='amcl', output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': False},
        ],
    )
    # 規劃器
    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': False},
        ],
    )
    # 控制器
    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': False},
        ],
    )
    # 行為樹導航
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': False},
        ],
    )
    # Lifecycle Manager（自動啟動）
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'bt_navigator',
            ],
        }],
    )

    # 5) RViz2
    rviz2 = Node(
        package='rviz2', executable='rviz2',
        name='rviz2', output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        # launch args
        declare_map_yaml,
        declare_params,
        declare_rviz,
        # pipeline
        map_gen,
        encoder,
        odom,
        serial_bridge,
        # Nav2 nodes
        map_server,
        static_tf,
        amcl,
        planner,
        controller,
        bt_navigator,
        lifecycle_manager,
        # RViz
        rviz2,
    ])
