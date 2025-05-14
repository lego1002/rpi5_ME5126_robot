#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
import smbus
import time
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu

class ImuNode(Node):
    def __init__(self):
        super().__init__('rpi5_imu_node')
        # I2C parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('address', 0x68)
        bus_num = self.get_parameter('i2c_bus').value
        self.addr = self.get_parameter('address').value
        self.get_logger().info(f'Opening I2C bus {bus_num} at address 0x{self.addr:X}')
        self.bus = smbus.SMBus(bus_num)

        # Wake MPU6050
        try:
            self.bus.write_byte_data(self.addr, 0x6B, 0x00)
        except OSError as e:
            self.get_logger().warn(f'I2C wake failed: {e}')

        # Calibrate biases
        time.sleep(0.1)
        self.bias_acc = [0.0, 0.0, 0.0]
        self.bias_gyro = [0.0, 0.0, 0.0]
        self.calibrate()

        # Publisher
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        # Timer
        self.create_timer(0.02, self._timer_cb)

    def calibrate(self):
        self.get_logger().info('Starting MPU6050 calibration...')
        acc_sum = [0, 0, 0]
        gyro_sum = [0, 0, 0]
        N = 100
        for _ in range(N):
            ax, ay, az, gx, gy, gz = self.read_raw_all()
            acc_sum[0] += ax; acc_sum[1] += ay; acc_sum[2] += az
            gyro_sum[0] += gx; gyro_sum[1] += gy; gyro_sum[2] += gz
            time.sleep(0.01)
        self.bias_acc = [v / N for v in acc_sum]
        self.bias_gyro = [v / N for v in gyro_sum]
        self.get_logger().info(f'Bias acc={self.bias_acc}, gyro={self.bias_gyro}')

    def _read_raw(self, reg):
        # Read two bytes and combine into signed int
        for _ in range(3):
            try:
                high = self.bus.read_byte_data(self.addr, reg)
                low  = self.bus.read_byte_data(self.addr, reg + 1)
                value = (high << 8) | low
                if value & 0x8000:
                    value = -((65535 - value) + 1)
                return value
            except OSError as e:
                self.get_logger().warn(f'I2C read reg 0x{reg:X} failed: {e}, retrying...')
                time.sleep(0.005)
        # After retries, raise to avoid silent failure
        raise RuntimeError('Failed to read from I2C after retries')

    def read_raw_all(self):
        ax = self._read_raw(0x3B) - self.bias_acc[0]
        ay = self._read_raw(0x3D) - self.bias_acc[1]
        az = self._read_raw(0x3F) - self.bias_acc[2]
        gx = self._read_raw(0x43) - self.bias_gyro[0]
        gy = self._read_raw(0x45) - self.bias_gyro[1]
        gz = self._read_raw(0x47) - self.bias_gyro[2]
        return ax, ay, az, gx, gy, gz

    def _timer_cb(self):
        try:
            ax, ay, az, gx, gy, gz = self.read_raw_all()
        except Exception as e:
            self.get_logger().error(f'Failed to read IMU data: {e}')
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        # Convert raw to SI units
        imu_msg.linear_acceleration.x = ax * 9.80665 / 16384.0
        imu_msg.linear_acceleration.y = ay * 9.80665 / 16384.0
        imu_msg.linear_acceleration.z = az * 9.80665 / 16384.0
        imu_msg.angular_velocity.x = gx * (250.0/32768.0) * (math.pi/180.0)
        imu_msg.angular_velocity.y = gy * (250.0/32768.0) * (math.pi/180.0)
        imu_msg.angular_velocity.z = gz * (250.0/32768.0) * (math.pi/180.0)
        self.pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
