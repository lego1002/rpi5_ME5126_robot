from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
          package='rpi5_robot_localization',
          executable='rpi5_encoder_node',
          name='rpi5_encoder_node',
          output='screen',
          parameters=[{'pulses_per_rev': 388}],
        ),
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
        ),
    ])
