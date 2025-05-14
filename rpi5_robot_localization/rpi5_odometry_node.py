#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class OdometryNode(Node):
    def __init__(self):
        super().__init__('rpi5_odometry_node')
        # �Ѽ�
        self.declare_parameter('ppr', 388)
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('wheel_base', 0.17)
        self.declare_parameter('publish_frequency', 20.0)

        self.ppr = self.get_parameter('ppr').value
        self.r   = self.get_parameter('wheel_radius').value
        self.b   = self.get_parameter('wheel_base').value
        freq    = self.get_parameter('publish_frequency').value

        # ���A
        self.x = self.y = self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # �Ȧs�W�q
        self.delta_l = 0
        self.delta_r = 0

        # �q�\���k���W�q
        self.create_subscription(Int32, '/wheel/encoder_left',  self.cb_left,  10)
        self.create_subscription(Int32, '/wheel/encoder_right', self.cb_right, 10)

        # Path & TF
        self.pub_path = self.create_publisher(Path, '/wheel/odom_accum', 10)
        self.tf_b     = TransformBroadcaster(self)
        self.path     = Path()
        self.path.header.frame_id = 'odom'

        # �w�ɤ@���p��]���A�b callback �̺�^
        period = 1.0 / freq
        self.create_timer(period, self._process)

    def cb_left(self,  msg: Int32):
        # �֥[��Ȧs�A���ߨ��
        self.delta_l += msg.data

    def cb_right(self, msg: Int32):
        self.delta_r += msg.data

    def _process(self):
        # 1) �p��
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        # 2) �W�q�߽��ন���l���L���Z��
        dl = 2 * math.pi * self.r * self.delta_l / self.ppr
        dr = 2 * math.pi * self.r * self.delta_r / self.ppr

        # �M���w�ιL���W�q
        self.delta_l = 0
        self.delta_r = 0

        # 3) �u�t�סB���t��
        v = (dr + dl) / 2.0 / dt
        w = (dr - dl) / self.b / dt

        # 4) �ֿn��m
        self.yaw += w * dt
        self.x   += v * dt * math.cos(self.yaw)
        self.y   += v * dt * math.sin(self.yaw)

        # 5) ��s Path
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.z = math.sin(self.yaw/2.0)
        pose.pose.orientation.w = math.cos(self.yaw/2.0)
        self.path.header.stamp = now.to_msg()
        self.path.poses.append(pose)
        self.pub_path.publish(self.path)

        # 6) broadcast TF
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z    = pose.pose.orientation.z
        tf.transform.rotation.w    = pose.pose.orientation.w
        self.tf_b.sendTransform(tf)

        # 7) log
        self.get_logger().info(
            f"x={self.x:.3f}, y={self.y:.3f}, dl={dl:.3f}, dr={dr:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()
