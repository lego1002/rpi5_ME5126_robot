#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share       = get_package_share_directory('rpi5_robot_localization')
    default_map     = '/tmp/rpi5_auto_map/auto_map.yaml'
    default_params  = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # 1) LaunchArguments
    namespace     = LaunchConfiguration('namespace',    default='')
    use_sim_time  = LaunchConfiguration('use_sim_time', default='false')
    autostart     = LaunchConfiguration('autostart',    default='true')
    map_yaml      = LaunchConfiguration('map',          default=default_map)
    params_file   = LaunchConfiguration('params_file',  default=default_params)

    declare_namespace     = DeclareLaunchArgument('namespace',    default_value='',               description='Top-level namespace')
    declare_use_sim_time  = DeclareLaunchArgument('use_sim_time', default_value='false',          description='Use simulation (Gazebo) clock if true')
    declare_autostart     = DeclareLaunchArgument('autostart',    default_value='true',           description='Automatically startup the nav2 stack')
    declare_map           = DeclareLaunchArgument('map',          default_value=default_map,       description='Full path to map yaml file to load')
    declare_params_file   = DeclareLaunchArgument('params_file',  default_value=default_params,    description='Full path to the ROS2 parameters file to use')

    # 2) 用 RewrittenYaml 把 use_sim_time/autostart 寫入所有節點
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart':    autostart
    }
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # 3) 環境變數：即時輸出日誌
    env = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # 4) 各 Nav2 節點（記得把 /odom remap 到 /wheel/odom）
    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[configured_params],
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
        remappings=[('/odom', '/wheel/odom')],
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[configured_params],
        remappings=[('/odom', '/wheel/odom')],
    )
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[configured_params],
        remappings=[
            ('/odom',    '/wheel/odom'),
            ('/cmd_vel', '/cmd_vel'),
        ],
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[configured_params],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[configured_params],
        remappings=[('/odom', '/wheel/odom')],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[configured_params],
    )

    # 5) 組裝並回傳 LaunchDescription
    ld = LaunchDescription()

    ld.add_action(env)
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_map)
    ld.add_action(declare_params_file)

    ld.add_action(map_server)
    ld.add_action(static_tf)
    ld.add_action(amcl)
    ld.add_action(planner_server)
    ld.add_action(controller_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(lifecycle_manager)

    return ld
