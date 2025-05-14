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
        # 參數
        self.declare_parameter('ppr', 388)
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('wheel_base', 0.17)
        self.declare_parameter('publish_frequency', 20.0)

        self.ppr = self.get_parameter('ppr').value
        self.r   = self.get_parameter('wheel_radius').value
        self.b   = self.get_parameter('wheel_base').value
        freq    = self.get_parameter('publish_frequency').value

        # 狀態
        self.x = self.y = self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # 暫存增量
        self.delta_l = 0
        self.delta_r = 0

        # 訂閱左右輪增量
        self.create_subscription(Int32, '/wheel/encoder_left',  self.cb_left,  10)
        self.create_subscription(Int32, '/wheel/encoder_right', self.cb_right, 10)

        # Path & TF
        self.pub_path = self.create_publisher(Path, '/wheel/odom_accum', 10)
        self.tf_b     = TransformBroadcaster(self)
        self.path     = Path()
        self.path.header.frame_id = 'odom'

        # 定時一次計算（不再在 callback 裡算）
        period = 1.0 / freq
        self.create_timer(period, self._process)

    def cb_left(self,  msg: Int32):
        # 累加到暫存，不立刻算
        self.delta_l += msg.data

    def cb_right(self, msg: Int32):
        self.delta_r += msg.data

    def _process(self):
        # 1) 計時
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        # 2) 增量脈衝轉成輪子走過的距離
        dl = 2 * math.pi * self.r * self.delta_l / self.ppr
        dr = 2 * math.pi * self.r * self.delta_r / self.ppr

        # 清掉已用過的增量
        self.delta_l = 0
        self.delta_r = 0

        # 3) 線速度、角速度
        v = (dr + dl) / 2.0 / dt
        w = (dr - dl) / self.b / dt

        # 4) 累積位置
        self.yaw += w * dt
        self.x   += v * dt * math.cos(self.yaw)
        self.y   += v * dt * math.sin(self.yaw)

        # 5) 更新 Path
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
