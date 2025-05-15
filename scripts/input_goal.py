#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading, math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import PySimpleGUI as sg
import tf2_ros
from tf2_ros import TransformException

class GoalController(Node):
    def __init__(self):
        super().__init__('input_goal')
        # Publisher cmd_vel
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF listener
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 參數
        self.Kp_ang      = 2.0      # 方向 P gain
        self.dist_thresh = 0.05     # 0.05 m 以內算到
        self.ang_thresh  = 0.1      # 0.1 rad 以內視為對齊
        self.max_v       = 0.2      # 初始線速度 (m/s)

        self.goal = None
        self.running = True

        # 啟 GUI
        threading.Thread(target=self._build_gui, daemon=True).start()

        # 控制迴圈 10 Hz
        self.create_timer(0.1, self._control_loop)

    def _build_gui(self):
        layout = [
            [sg.Text('目標 X (m):'), sg.Input('0.0', size=(8,1), key='-X-')],
            [sg.Text('目標 Y (m):'), sg.Input('0.0', size=(8,1), key='-Y-')],
            [sg.Text('最大速度 (m/s):'), sg.Slider(range=(0,1), default_value=self.max_v,
                                                  resolution=0.01, orientation='h', key='-V-')],
            [sg.Button('Go'), sg.Button('Stop'), sg.Button('Exit')],
        ]
        window = sg.Window('Goal Controller', layout)

        while True:
            event, vals = window.read(timeout=100)
            if event in (sg.WIN_CLOSED, 'Exit'):
                self.running = False
                break

            if event == 'Go':
                try:
                    x = float(vals['-X-']); y = float(vals['-Y-'])
                    self.max_v = float(vals['-V-'])
                    self.goal  = (x, y)
                    self.get_logger().info(f'設定新目標: ({x:.2f}, {y:.2f}), max_v={self.max_v:.2f}')
                except ValueError:
                    pass

            if event == 'Stop':
                self.goal = None
                twist = Twist()
                self.pub.publish(twist)
                self.get_logger().info('停止馬達')

        window.close()
        rclpy.shutdown()

    def _control_loop(self):
        if not self.running:
            return

        if self.goal is None:
            return

        # 1) 拿當前 base_link 在 odom 座標下的位置 & yaw
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
        except TransformException:
            return

        x_cur = trans.transform.translation.x
        y_cur = trans.transform.translation.y
        q = trans.transform.rotation
        # yaw = atan2(2*w*z, 1-2*z^2)
        yaw = math.atan2(2.0*(q.w*q.z), 1.0-2.0*(q.z*q.z))

        # 2) 計算目標向量
        xg, yg = self.goal
        dx = xg - x_cur
        dy = yg - y_cur
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)

        # 3) 計算 yaw 誤差 [-π,π]
        err_yaw = desired_yaw - yaw
        err_yaw = (err_yaw + math.pi) % (2*math.pi) - math.pi

        twist = Twist()

        # 4) 如果尚未對齊，就只旋轉
        if abs(err_yaw) > self.ang_thresh:
            twist.angular.z = self.Kp_ang * err_yaw
            twist.linear.x  = 0.0

        # 5) 向齊後、距離大於閾值才前進
        elif dist > self.dist_thresh:
            twist.angular.z = 0.0
            twist.linear.x  = min(self.max_v, dist)

        # 6) 都到位了，就停下並清目標
        else:
            twist.angular.z = 0.0
            twist.linear.x  = 0.0
            self.get_logger().info('已到達目標')
            self.goal = None

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GoalController()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
