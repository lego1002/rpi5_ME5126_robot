#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('rpi5_control_node')
        # Declare parameters with default values
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('stop_tolerance', 0.02)  # 停車容差 (公尺)

        # Get initial parameter values
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.stop_tolerance = self.get_parameter('stop_tolerance').value

        # Add callback for dynamic parameter updates
        self.add_on_set_parameters_callback(self.param_callback)

        # Subscribe to odometry (wheel odom or filtered odom)
        self.create_subscription(
            Odometry,
            '/wheel/odom',    # 若使用 EKF，改成 '/odometry/filtered'
            self.odom_callback,
            10
        )

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def param_callback(self, params):
        # Update internal parameters when changed
        for param in params:
            if param.name == 'target_x' and param.type_ == param.Type.DOUBLE:
                self.target_x = param.value
                self.get_logger().info(f'Updated target_x to {self.target_x}')
            elif param.name == 'target_y' and param.type_ == param.Type.DOUBLE:
                self.target_y = param.value
                self.get_logger().info(f'Updated target_y to {self.target_y}')
            elif param.name == 'kp_linear' and param.type_ == param.Type.DOUBLE:
                self.kp_linear = param.value
                self.get_logger().info(f'Updated kp_linear to {self.kp_linear}')
            elif param.name == 'kp_angular' and param.type_ == param.Type.DOUBLE:
                self.kp_angular = param.value
                self.get_logger().info(f'Updated kp_angular to {self.kp_angular}')
            elif param.name == 'stop_tolerance' and param.type_ == param.Type.DOUBLE:
                self.stop_tolerance = param.value
                self.get_logger().info(f'Updated stop_tolerance to {self.stop_tolerance}')
        return SetParametersResult(successful=True)

    def odom_callback(self, msg: Odometry):
        # Extract current position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Compute distance error
        dx = self.target_x - x
        dy = self.target_y - y
        dist_error = math.hypot(dx, dy)

        # Deadzone: if within stop tolerance, publish zero and return
        if dist_error < self.stop_tolerance:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return

        # Compute yaw from quaternion
        ori = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (ori.w * ori.z + ori.x * ori.y),
            1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        )

        # Compute desired heading and yaw error
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - yaw)

        # Compute control commands (P controller)
        cmd = Twist()
        cmd.linear.x = self.kp_linear * dist_error
        cmd.angular.z = self.kp_angular * yaw_error

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        # Wrap angle to [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
