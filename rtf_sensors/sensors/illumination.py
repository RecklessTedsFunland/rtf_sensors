# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2020 Kevin Walchko
# see LICENSE for full details
##############################################
# pip3 install adafruit-circuitpython-bh1750
#
#  https://www.waveformlighting.com/home-residential/what-is-the-difference-between-lux-and-lumens
#
# - Lux is a measure of illuminance, the total amount of light that falls
#   on a surface
#
# - Lumens is a measure of luminous flux, the total amount of light
#   emitted in all directions.

from rclpy.node import Node
from sensor_msgs.msg import Illuminance
from colorama import Fore
import board
import busio
import adafruit_bh1750
import attr


# @attr.s(slots=True)
# class Parameters:
#     @classmethod
#     def add_values(cls, params):
#         cls.doc = {}
#         for name, value, doc_str in params:
#             setattr(cls, name, value)
#             cls.doc[value] = doc_str
#
#     @classmethod
#     def is_valid(cls, value):
#         return value in cls.doc
#
#
# class Rate(Parameters):
#     pass
#
# Rate.add_values(
#     (
#         ("r1Hz", 1.0, "Sample once per second"),
#         ("r2Hz", 0.5, "Sample twice a second"),
#         ("r5Hz", 0.2, "Sample 5 times a second")
#     )
# )


from collections import namedtuple

Hz = namedtuple("Hz", "hertz_1 hertz_2 hertz_3 hertz_4 hertz_5")
Rate = Hz(1.0, 0.5, 1/3, 1/4, 1/5)

class rtf_bh1750(Node):
    def __init__(self, i2c=None):
        super().__init__('rtf_bh1750')

        if i2c is None:
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        else:
            self.i2c = i2c

        self.sensor = adafruit_bh1750.BH1750(i2c)
        # resolution: LOW(4 lx), MEDIUM(1 lx), HIGH (0.5 lx)
        self.sensor.resolution = adafruit_bh1750.Resolution.HIGH

        # at high and medium, it can take <= 180msec to collect data
        self.timer = self.create_timer(Rate.hertz_5, self.callback)
        # self.timer = self.create_timer(0.2, self.callback)

        self.pub = self.create_publisher(Illuminance, 'illuminance', 10)
        # self.pub_pressure = self.create_publisher(FluidPressure, 'pressure', 10)

        frame_id = self.declare_parameter('frame_id', "base_imu_link").value

        self.msg = Illuminance()
        self.msg.header.frame_id = frame_id
        self.msg.variance = 0.01

    def callback(self):
        try:
            stamp = self.get_clock().now().to_msg()
            illum = self.sensor.lux
            self.msg.header.stamp = stamp
            self.msg.illuminance = illum # lux
            self.pub.publish(self.msg)

        except Exception as e:
            print(f"{Fore.RED}*** {e} ***{Fore.RESET}")

        # p = self.lps.pressure
        # self.pressure_msg.header.stamp = stamp
        # self.pressure_msg.fluid_pressure = p*100 # Pa
        # self.pub_pressure.publish(self.pressure_msg)
