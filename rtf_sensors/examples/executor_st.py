#!/usr/bin/env python3
# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2020 Kevin Walchko
# see LICENSE for full details
##############################################

"""
discrete LPF or HPF
from collections import deque

c = deque([n coefficients], maxlen=n)
d = deque([n data pts], maxlen=n)

bob = lambda e,f: sum([a*b for a,b in zip(e,f)])
bob(c,d)
"""
import os
if 'BLINKA_MCP2221' in os.environ.keys():
    pass
else:
    os.environ['BLINKA_MCP2221'] = "1"

import rclpy
from rtf_sensors.nodes.lsm6sox import rtf_lsm6sox
from rtf_sensors.nodes.lis3mdl import rtf_lis3mdl
from rtf_sensors.nodes.dps310  import rtf_dps310

# from rclpy.exceptions import ParameterNotDeclaredException
# from rcl_interfaces.msg import ParameterType
from rclpy.executors import SingleThreadedExecutor

import board
import busio


def main(args=None):
    rclpy.init(args=args)

    i2c = busio.I2C(board.SCL, board.SDA)

    imu = rtf_lsm6sox(i2c)
    mag = rtf_lis3mdl(i2c)
    temp = rtf_dps310(i2c)

    executor = SingleThreadedExecutor()
    executor.add_node(imu)
    executor.add_node(mag)
    executor.add_node(temp)

    try:
        executor.spin()

    except KeyboardInterrupt:
        print("\nbye ...\n")

    finally:
        executor.shutdown()
        imu.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
