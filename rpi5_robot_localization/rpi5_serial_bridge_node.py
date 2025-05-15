#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def clamp(x, lo, hi):
    if x < lo:   return lo
    if x > hi:   return hi
    return x

class SerialBridge(Node):
    def __init__(self):
        super().__init__('rpi5_serial_bridge_node')

        # 參數宣告
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_base', 0.17)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('max_linear_speed', 0.3)    # [m/s]
        self.declare_parameter('max_angular_speed', 1.0)   # [rad/s]

        # 讀參數
        port            = self.get_parameter('port').value
        baudrate        = self.get_parameter('baudrate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_pwm    = self.get_parameter('max_pwm').value
        self.max_v      = self.get_parameter('max_linear_speed').value
        self.max_w      = self.get_parameter('max_angular_speed').value

        # 開啟序列埠
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"已開啟 Serial port：{port} @ {baudrate}bps")
        except Exception as e:
            self.get_logger().error(f"無法開啟 Serial port：{e}")
            raise

        # 訂閱 cmd_vel
        self.create_subscription(
            Twist, '/cmd_vel', self.cb_cmd_vel, 10)

    def cb_cmd_vel(self, msg: Twist):
        # 1) 把 cmd_vel 限幅到 max_v、max_w
        v = clamp(msg.linear.x,  -self.max_v, self.max_v)
        w = clamp(msg.angular.z, -self.max_w, self.max_w)

        # 2) 轉為左右輪速度 (m/s)
        #    v_l = v - (w * L/2) ; v_r = v + (w * L/2)
        v_l = v - w * (self.wheel_base / 2.0)
        v_r = v + w * (self.wheel_base / 2.0)

        # 3) 對映到 PWM
        # pwm = (v_i / max_v) * max_pwm
        l_pwm = int(clamp(v_l / self.max_v * self.max_pwm,
                          -self.max_pwm, self.max_pwm))
        r_pwm = int(clamp(v_r / self.max_v * self.max_pwm,
                          -self.max_pwm, self.max_pwm))

        # 4) 印出 debug
        self.get_logger().info(
            f"cmd_vel → v={v:.3f} w={w:.3f} 轉換 → L_pwm={l_pwm}, R_pwm={r_pwm}")

        # 5) 傳送給 Arduino
        data = f"{l_pwm},{r_pwm}\n".encode('utf-8')
        try:
            self.ser.write(data)
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f"Serial 寫入失敗：{e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()