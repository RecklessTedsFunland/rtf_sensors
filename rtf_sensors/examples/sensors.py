#!/usr/bin/env python3
# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2020 Kevin Walchko
# see LICENSE for full details
##############################################

import os
if 'BLINKA_MCP2221' in os.environ.keys():
    pass
else:
    os.environ['BLINKA_MCP2221'] = "1"

import rclpy
import board
import busio
from rtf_sensors.nodes.mlx90640 import rtf_mlx90640


def camera_ir(args=None):
    rclpy.init(args=args)

    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    sensor = rtf_mlx90640(i2c)
    # mag.enable_calibration()

    try:
        rclpy.spin(sensor)
    except KeyboardInterrupt:
        print("bye ...")
    finally:
        sensor.destroy_node()
        rclpy.shutdown()

# def camera_ir():
#     i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
#     camera = rtf_mlx90640(i2c)
#
#     main_loop(camera)
