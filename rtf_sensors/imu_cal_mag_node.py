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
from rtf_sensors.nodes.lis3mdl import rtf_lis3mdl
import board
import busio
from slurm import storage


def main(args=None):
    rclpy.init(args=args)

    i2c = busio.I2C(board.SCL, board.SDA)
    mag = rtf_lis3mdl(i2c)
    mag.enable_calibration()

    try:
        rclpy.spin(mag)
    except KeyboardInterrupt:
        print("bye ...")
    finally:
        storage.write("lis3mdl-cal-data.pickle", mag.deque)
        mag.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
