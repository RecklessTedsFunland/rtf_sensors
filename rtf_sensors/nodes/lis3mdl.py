# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2020 Kevin Walchko
# see LICENSE for full details
##############################################


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
# from colorama import Fore
import board
import busio
import adafruit_lis3mdl
from collections import deque


class rtf_lis3mdl(Node):
    def __init__(self, i2c=None):
        super().__init__('rtf_lis3mdl')

        if i2c is None:
            self.i2c = busio.I2C(board.SCL, board.SDA)
        else:
            self.i2c = i2c

        self.lis = adafruit_lis3mdl.LIS3MDL(self.i2c) # 155 Hz, 4 gauss, continuous
        self.lis.data_rate = adafruit_lis3mdl.Rate.RATE_155_HZ

        self.timer_mag  = self.create_timer(1/10, self.callback)
        self.pub_mag = self.create_publisher(MagneticField, 'mag', 10)

        self.mags_bias = self.declare_parameter('mags_bias', [0.,0.,0.])

        frame_id = self.declare_parameter('frame_id', "base_imu_link").value

        self.mag_msg = MagneticField()
        self.mag_msg.header.frame_id = frame_id

        mc = 0.01
        self.mag_msg.magnetic_field_covariance = [
             mc, 0.0, 0.0,
            0.0,  mc, 0.0,
            0.0, 0.0,  mc
        ]

        self.calibrate = False

    def callback(self):
        m = self.lis.magnetic

        if self.calibrate:
            msg = {
                "timestamp": self.get_clock().now().nanoseconds*1e-9,
                "m": m,
            }
            self.deque.append(msg)

        self.mag_msg.header.stamp = self.get_clock().now().to_msg()
        self.mag_msg.magnetic_field.x = m[0]
        self.mag_msg.magnetic_field.y = m[1]
        self.mag_msg.magnetic_field.z = m[2]

        self.pub_mag.publish(self.mag_msg)

    def enable_calibration(self):
        self.calibrate = True
        self.deque = deque()
