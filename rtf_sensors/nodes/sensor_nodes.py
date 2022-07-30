# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2020 Kevin Walchko
# see LICENSE for full details
##############################################
import rclpy
from rclpy.executors import SingleThreadedExecutor
# from rclpy.exceptions import ParameterNotDeclaredException
# from rcl_interfaces.msg import ParameterType

from ..sensors.acceleration_gyro    import rtf_lsm6sox
from ..sensors.magnetic             import rtf_lis3mdl
from ..sensors.pressure_temperature import rtf_dps310
from ..sensors.pressure_temperature import rtf_bmp390


def run(sensorClass, args=None):
    rclpy.init(args=args)
    sensor = sensorClass()

    executor = SingleThreadedExecutor()
    executor.add_node(sensor)

    try:
        executor.spin()

    except KeyboardInterrupt:
        print("\n>> Received ctrl-c ... bye\n")

    finally:
        executor.shutdown()
        sensor.destroy_node()

# Pressure/Temperature -----------------
def rtf_bmp390_node(args=None):
    run(rtf_bmp390, args)

def rtf_dsp310_node(args=None):
    run(rtf_dsp310, args)

# Magnetic ------------------------------
def rtf_lis3mdl_node(args=None):
    run(rtf_lis3mdl, args)

# Accel/Gyro ----------------------------
def rtf_lsm6sox_node(args=None):
    run(rtf_lsm6sox, args)
