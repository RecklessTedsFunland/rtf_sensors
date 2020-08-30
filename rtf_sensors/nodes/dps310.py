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
import adafruit_dps310


# class rtf_dps310(Node):
#     def __init__(self, i2c=None):
#         super().__init__('rtf_dps310')
#
#
#         if i2c is None:
#             self.i2c = busio.I2C(board.SCL, board.SDA)
#         else:
#             self.i2c = i2c
#
#         self.sensor = adafruit_dps310.DPS310(self.i2c)

class rtf_PT(Node):
    def __init__(self):
        super().__init__('rtf_dps310')

        rate = 1.0

        self.timer = self.create_timer(1.0/rate, self.callback)

        self.pub_temp = self.create_publisher(Temperature, 'temperature', 10)
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
        t = self.sensor.temperature
        self.temp_msg.header.stamp = stamp
        self.temp_msg.temperature = t # C
        self.pub_temp.publish(self.temp_msg)

        p = self.sensor.pressure
        self.pressure_msg.header.stamp = stamp
        self.pressure_msg.fluid_pressure = p*100 # Pa
        self.pub_pressure.publish(self.pressure_msg)


class rtf_dps310(rtf_PT):
    def __init__(self, i2c=None):
        super().__init__()

        if i2c is None:
            self.i2c = busio.I2C(board.SCL, board.SDA)
        else:
            self.i2c = i2c

        self.sensor = adafruit_dps310.DPS310(self.i2c)
