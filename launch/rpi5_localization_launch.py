#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ekf_params = {
        'frequency': 30.0,
        'sensor_timeout': 1.0,
        'two_d_mode': True,
        'publish_tf': True,
        'map_frame': 'odom',
        'odom_frame': 'odom',
        'base_link_frame': 'base_link',
        'odom0': '/wheel/odom',
        'odom0_config': [True, False, False,
                         False, False, True,
                         True, False, False],
        'odom0_queue_size': 5,
        'odom0_relative': True,
        'imu0': '/imu/data',
        'imu0_config': [False, False, False,
                        False, False, True,
                        False, False, False,
                        False, False, True],
        'imu0_queue_size': 5,
        'imu0_remove_gravitational_acceleration': True,
    }

    return LaunchDescription([
        # 1) Encoder
        Node(
            package='rpi5_robot_localization',
            executable='rpi5_encoder_node',
            name='rpi5_encoder_node',
            output='screen',
            parameters=[{'pulses_per_rev': 388}],
        ),

        # 2) IMU
        Node(
            package='rpi5_robot_localization',
            executable='rpi5_imu_node',
            name='rpi5_imu_node',
            output='screen',
            parameters=[{'i2c_bus': 1}, {'address': 0x68}],
        ),

        # 3) Odometry：從編碼器直接算瞬時 + 累積
        Node(
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
                ('/wheel/encoder_left',  '/wheel/encoder_left'),
                ('/wheel/encoder_right', '/wheel/encoder_right'),
                ('/wheel/odom',          '/wheel/odom'),
                ('/wheel/odom_accum',    '/wheel/odom_accum'),
            ],
        ),
        

        # 4) EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='rpi5_ekf_filter_node',
            output='screen',
            parameters=[{'ros__parameters': ekf_params}],
        ),

        '''
        # 5) Control
        Node(
            package='rpi5_robot_localization',
            executable='rpi5_control_node',
            name='rpi5_control_node',
            output='screen',
        ),
        '''

        # 6) Serial bridge
        Node(
            package='rpi5_robot_localization',
            executable='rpi5_serial_bridge_node',
            name='rpi5_serial_bridge_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baudrate': 115200},
                {'wheel_radius': 0.04},
                {'wheel_base': 0.17},
            ],
        ),
    ])