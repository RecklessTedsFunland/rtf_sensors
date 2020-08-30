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
from rtf_sensors.nodes.lsm6sox import rtf_lsm6sox
import board
import busio
from slurm import storage


def main(args=None):
    rclpy.init(args=args)

    i2c = busio.I2C(board.SCL, board.SDA)
    imu = rtf_lsm6sox(i2c)
    imu.enable_calibration(10.0)

    try:
        rclpy.spin(imu)
    except KeyboardInterrupt:
        print("bye ...")
    finally:
        storage.write("lsm6sox-cal-data.pickle", imu.deque)
        imu.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
