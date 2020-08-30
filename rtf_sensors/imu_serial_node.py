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
# import time
# import numpy as np
from rtf_sensors.imu_node import RTFIMU

def main(args=None):
    rclpy.init(args=args)

    node = RTFIMU()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n\nbye ...\n")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
