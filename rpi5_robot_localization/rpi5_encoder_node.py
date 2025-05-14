#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gpiod
import threading
import math

class EncoderNode(Node):
    def __init__(self):
        super().__init__('rpi5_encoder_node')
        # Parameters: pulses per revolution and publish rate
        self.declare_parameter('pulses_per_rev', 388)
        self.declare_parameter('publish_frequency', 10.0)
        self.ppr = self.get_parameter('pulses_per_rev').value
        freq = self.get_parameter('publish_frequency').value

        # Configure gpio lines on gpiochip4
        chip = gpiod.Chip('gpiochip4')
        self.line_left_a  = chip.get_line(22)
        self.line_left_b  = chip.get_line(23)
        self.line_right_a = chip.get_line(6)
        self.line_right_b = chip.get_line(5)

        # Request events for channel A, input-only for B
        self.line_left_a.request(consumer='enc_la', type=gpiod.LINE_REQ_EV_BOTH_EDGES)
        self.line_left_b.request(consumer='enc_lb', type=gpiod.LINE_REQ_DIR_IN)
        self.line_right_a.request(consumer='enc_ra', type=gpiod.LINE_REQ_EV_BOTH_EDGES)
        self.line_right_b.request(consumer='enc_rb', type=gpiod.LINE_REQ_DIR_IN)

        # Counters and lock
        self.counts = {'left': 0, 'right': 0}
        self.last_counts = {'left': 0, 'right': 0}
        self._lock = threading.Lock()

        # Start monitoring threads for left and right channels
        threading.Thread(target=self._monitor, args=(self.line_left_a, self.line_left_b, 'left'), daemon=True).start()
        threading.Thread(target=self._monitor, args=(self.line_right_a, self.line_right_b, 'right'), daemon=True).start()

        # Publishers for encoder pulse deltas
        self.pub_l = self.create_publisher(Int32, '/wheel/encoder_left', 10)
        self.pub_r = self.create_publisher(Int32, '/wheel/encoder_right', 10)

        # Timer to publish pulse deltas at the specified frequency
        timer_period = 1.0 / freq
        self.create_timer(timer_period, self._timer_cb)

    def _monitor(self, line_a, line_b, side):
        # Wait for edge events on channel A and update count based on channel B state
        while rclpy.ok():
            if line_a.event_wait(1):  # wait up to 1 second
                _ = line_a.event_read()
                a_val = line_a.get_value()
                b_val = line_b.get_value()
                delta = 1 if (a_val != b_val) else -1
                with self._lock:
                    self.counts[side] += delta

    def _timer_cb(self):
        # Compute pulse deltas since last publish
        with self._lock:
            dl = self.counts['left']  - self.last_counts['left']
            dr = self.counts['right'] - self.last_counts['right']
            self.last_counts['left']  = self.counts['left']
            self.last_counts['right'] = self.counts['right']

        # Publish the deltas (incremental pulses)
        self.pub_l.publish(Int32(data=dl))
        self.pub_r.publish(Int32(data=dr))


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
