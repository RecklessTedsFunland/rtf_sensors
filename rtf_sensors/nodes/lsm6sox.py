# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2016 Kevin Walchko
# see LICENSE for full details
##############################################


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
# from colorama import Fore
import board
import busio
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import AccelRange, GyroRange, Rate
from squaternion import Quaternion
from collections import deque


# class Base:
#     def __init__(self, time_frame=100):
#         # print(">> Base init")
#         # super(Base, self).__init__()
#         # super().__init__()
#         self.cnt = 0
#         self.__hz = 0
#         self.last = time.monotonic()
#         self.window = time_frame
#         # self.deque = deque()
#
#     @property
#     def hz(self):
#         return self.__hz
#
#     def hz_update(self):
#         self.cnt += 1
#
#         if self.cnt % self.window == 0:
#             now = time.monotonic()
#             self.__hz = self.window/(now - self.last)
#             self.last = now
#             self.cnt = 0
#             print(f">> {self.__hz}hz")


class rtf_lsm6sox(Node):
    def __init__(self, i2c=None):
        super().__init__('rtf_lsm6sox')
        # Base.__init__(self, 70)

        self.timer_imu  = self.create_timer(1/100, self.callback)

        if i2c is None:
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        else:
            self.i2c = i2c

        # 'RANGE_1000_DPS', 'RANGE_125_DPS', 'RANGE_2000_DPS', 'RANGE_250_DPS', 'RANGE_4000_DPS', 'RANGE_500_DPS'
        self.imu = LSM6DSOX(self.i2c)
        self.imu.accelerometer_range = AccelRange.RANGE_4G  # pylint: disable=no-member
        self.imu.accelerometer_data_rate = Rate.RATE_104_HZ
        self.imu.gyro_range = GyroRange.RANGE_2000_DPS
        self.imu.gyro_data_rate = Rate.RATE_104_HZ

        frame_id = self.declare_parameter('frame_id', "base_imu_link").value
        self.accels_bias = self.declare_parameter('accels_bias', [0.,0.,0.])
        self.gyros_bias = self.declare_parameter('gyros_bias', [0.,0.,0.])

        self.pub_imu = self.create_publisher(Imu, 'imu', 10)

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = frame_id

        ac, oc, vc = (0.01, 0.001, 0.01)
        self.imu_msg.linear_acceleration_covariance = [
             ac, 0.0, 0.0,
            0.0,  ac, 0.0,
            0.0, 0.0,  ac
        ]
        self.imu_msg.orientation_covariance = [
             oc, 0.0, 0.0,
            0.0,  oc, 0.0,
            0.0, 0.0,  oc
        ]
        self.imu_msg.angular_velocity_covariance = [
             vc, 0.0, 0.0,
            0.0,  vc, 0.0,
            0.0, 0.0,  vc
        ]

        self.calibrate = False

    def callback(self):
        # self.hz_update()

        a = self.imu.acceleration
        g = self.imu.gyro

        if self.calibrate:
            msg = {
                "timestamp": self.get_clock().now().nanoseconds*1e-9,
                "a": a,
                "g": g
            }
            self.deque.append(msg)
            if not self.continue_pub:
                return

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.linear_acceleration.x = a[0]
        self.imu_msg.linear_acceleration.y = a[1]
        self.imu_msg.linear_acceleration.z = a[2]
        self.imu_msg.angular_velocity.x = g[0]
        self.imu_msg.angular_velocity.y = g[1]
        self.imu_msg.angular_velocity.z = g[2]

        # generally I would also have the magnetometer to determine heading
        # not sure I like this solution/hack :P
        # q = self.compass.get_quaternion(a, None)
        q = Quaternion()

        self.imu_msg.orientation.x = q.x
        self.imu_msg.orientation.y = q.y
        self.imu_msg.orientation.z = q.z
        self.imu_msg.orientation.w = q.w

        self.pub_imu.publish(self.imu_msg)

    def enable_calibration(self, window, continue_pub=True):
        """
        Creates a couple class members:
        - timer_calibrate: calls every *window* telling you to rotate calibration cube
        - deque: creates deque with fast append to store data
        - continue_pub: after pushing data to deque, do you still want to publish data [default=True]?
        """
        self.calibrate = True
        self.continue_pub = continue_pub
        self.deque = deque()
        self.timer_calibrate  = self.create_timer(window, self.cb_calibration)

    def cb_calibration(self):
        ret = input(">> Setup new orientation, press any key to continue or ctrl-c to stop")
        # print(ret)

        self.timer_calibrate.reset()
