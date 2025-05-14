#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.04)  
        self.declare_parameter('wheel_base',   0.17)  

        port  = self.get_parameter('port').value
        baud  = self.get_parameter('baudrate').value
        self.r  = self.get_parameter('wheel_radius').value
        self.L  = self.get_parameter('wheel_base').value

        self.ser = serial.Serial(port, baud, timeout=1)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

    def cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        wl = (2*v - w*self.L) / (2*self.r)
        wr = (2*v + w*self.L) / (2*self.r)

        s = f"{wl:.3f},{wr:.3f}\n"
        self.ser.write(s.encode())

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()
