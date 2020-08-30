# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2020 Kevin Walchko
# see LICENSE for full details
##############################################


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, FluidPressure
# from colorama import Fore
import board
import busio
import adafruit_lps2x


class rtf_lps22(Node):
    def __init__(self, i2c=None):
        super().__init__('rtf_lps22')


        if i2c is None:
            self.i2c = busio.I2C(board.SCL, board.SDA)
        else:
            self.i2c = i2c

        self.lps = adafruit_lps2x.LPS22(self.i2c)

        self.timer_temp = self.create_timer(1.0, self.callback)

        self.pub_temp = self.create_publisher(Temperature, 'temp', 10)
        self.pub_pressure = self.create_publisher(FluidPressure, 'pressure', 10)

        frame_id = self.declare_parameter('frame_id', "base_imu_link").value

        self.temp_msg = Temperature()
        self.temp_msg.header.frame_id = frame_id
        self.temp_msg.variance = 0.01

        self.pressure_msg = FluidPressure()
        self.pressure_msg.header.frame_id = frame_id
        self.pressure_msg.variance = 0.01

    def callback(self):
        stamp = self.get_clock().now().to_msg()
        t = self.lps.temperature
        self.temp_msg.header.stamp = stamp
        self.temp_msg.temperature = t # C
        self.pub_temp.publish(self.temp_msg)

        p = self.lps.pressure
        self.pressure_msg.header.stamp = stamp
        self.pressure_msg.fluid_pressure = p*100 # Pa
        self.pub_pressure.publish(self.pressure_msg)
