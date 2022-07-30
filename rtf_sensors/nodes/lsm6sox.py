# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2016 Kevin Walchko
# see LICENSE for full details
##############################################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import board
    import busio
except ImportError:
    # print(">> pip install -U Adafruit-Blinka")
    pass

try:
    from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
    from adafruit_lsm6ds import AccelRange, GyroRange, Rate
except ImportError:
    # print(">> pip install -U adafruit-circuitpython-lsm6ds")
    pass

from squaternion import Quaternion
from ..filters.mahony import Mahony

class rtf_IMU(Node):
    def __init__(self, name, i2c=None):
        super().__init__(name)
        # print("rtf_imu")

        self.filter = Mahony()

        self.timer_imu  = self.create_timer(1/100, self.callback)

        if i2c is None:
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        else:
            self.i2c = i2c

        self.imu = None

        frame_id = self.declare_parameter('frame_id', "base_imu_link").value
        self.accels_bias = self.declare_parameter('accels_bias', [0.,0.,0.])
        self.gyros_bias = self.declare_parameter('gyros_bias', [0.,0.,0.])

        self.pub_imu = self.create_publisher(Imu, 'imu', 10)

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = frame_id

        self.imu_msg.linear_acceleration_covariance = [0.0]*9
        self.imu_msg.orientation_covariance = [0.0]*9
        self.imu_msg.angular_velocity_covariance = [0.0]*9

    def callback(self):
        a = self.imu.acceleration
        g = self.imu.gyro

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
        # q = Quaternion()
        q = self.filter.update(a,g,0.01)

        # ROS is JPL and mine is Hamilton quaternion
        # qJPL = qHamilton.conjugate
        self.imu_msg.orientation.x = -q.x
        self.imu_msg.orientation.y = -q.y
        self.imu_msg.orientation.z = -q.z
        self.imu_msg.orientation.w = q.w

        self.pub_imu.publish(self.imu_msg)


class rtf_lsm6sox(rtf_IMU):
    def __init__(self, i2c=None):
        super().__init__('rtf_lsm6sox', i2c)
        # print("rtf_lsm6sox")

        # 'RANGE_1000_DPS', 'RANGE_125_DPS', 'RANGE_2000_DPS', 'RANGE_250_DPS',
        # 'RANGE_4000_DPS', 'RANGE_500_DPS'
        self.imu = LSM6DSOX(self.i2c)
        self.imu.accelerometer_range = AccelRange.RANGE_2G
        self.imu.accelerometer_data_rate = Rate.RATE_104_HZ
        self.imu.gyro_range = GyroRange.RANGE_2000_DPS
        self.imu.gyro_data_rate = Rate.RATE_104_HZ
