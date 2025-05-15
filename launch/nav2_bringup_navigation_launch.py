#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = get_package_share_directory('rpi5_robot_localization')

    # 1) Launch arguments
    map_yaml_arg    = DeclareLaunchArgument(
        'map', default_value='/tmp/rpi5_auto_map/auto_map.yaml',
        description='Full path to map yaml file')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameter file')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    map_yaml     = LaunchConfiguration('map')
    params_file  = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 2) Rewrite YAML to inject use_sim_time and namespace (none)
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )

    # 3) Nav2 core nodes
    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[{'yaml_filename': map_yaml}, configured_params],
    )

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_to_odom', output='screen',
        arguments=['0','0','0','0','0','0','map','odom'],
    )

    amcl = Node(
        package='nav2_amcl', executable='amcl',
        name='amcl', output='screen',
        parameters=[configured_params],
        remappings=[('/odom','/wheel/odom')],
    )

    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[configured_params],
        remappings=[('/odom','/wheel/odom')],
    )

    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[configured_params],
        remappings=[('/odom','/wheel/odom'),
                    ('/cmd_vel','/cmd_vel')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[configured_params],
        remappings=[('/odom','/wheel/odom')],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[configured_params,
                    {'autostart': True,
                     'node_names': [
                         'map_server',
                         'amcl',
                         'planner_server',
                         'controller_server',
                         'bt_navigator'
                     ]}]
    )

    return LaunchDescription([
        # arguments
        map_yaml_arg,
        params_file_arg,
        use_sim_time_arg,
        # nodes
        map_server,
        static_tf,
        amcl,
        planner_server,
        controller_server,
        bt_navigator,
        lifecycle_manager,
    ])
