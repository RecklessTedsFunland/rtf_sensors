# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2016 Kevin Walchko
# see LICENSE for full details
##############################################
import attr
from math import cos, sin, pi, atan2, asin, sqrt
from squaternion import Quaternion
# from ins_nav.utils import RAD2DEG, DEG2RAD
# from ins_nav.utils import normalize3
from enum import IntFlag


RAD2DEG = 180/pi
DEG2RAD = pi/180

# Angle = IntFlag("Angle", "degrees radians quaternion")

@attr.s(slots=True)
class TiltCompensatedCompass:
    """
    A tilt compensated compass is basically just taking the magnetometer
    readings and adjusting them based of how the sensor is oriented (tilted).
    We use the accelerometer to determine the orientation relative to the
    ground. Obviously, if the sensor is under some sort of gravity loading,
    say movng fast (falling), then this won't work correctly.

    Also, the in sensor inputs are expected to have already been adjusted
    for biases and other issues (hard/soft iron errors).
    """

    # angle_units = attr.ib(default=Angle.degrees)

    # def __init__(self, angle_units=Angle.degrees):
    #     """
    #     angle_units: degrees, radians, quaternion
    #     quaternion: default is (1,0,0,0), but you can set it to something else
    #     """
    #     # Mx points to North
    #     # self.imu = imu
    #     self.angle_units = angle_units
    def normalize(self, x, y, z):
        """Return a unit vector"""
        norm = sqrt(x * x + y * y + z * z)

        # already a unit vector
        if norm == 1.0:
            return (x, y, z)

        inorm = 1.0/norm
        if inorm > 1e-6:
            x *= inorm
            y *= inorm
            z *= inorm
        else:
            raise ZeroDivisionError(f'norm({x:.4f}, {y:.4f}, {z:.4f},) = {inorm:.6f}')
        return (x, y, z,)

    def get_quaternion(self, accel, mag):
        r,p,y = self.get_euler(accel, mag)
        return Quaternion().from_euler(r,p,y, degrees=False)

    def get_euler(self, accel, mag=None):
        """

        """

        try:
            if mag is None:
                mx,my,mz = [0.,0.,1.0]
            else:
                mx, my, mz = self.normalize(*mag)
            ax, ay, az = self.normalize(*accel)

            pitch = asin(-ax)

            if abs(pitch) >= pi/2:
                roll = 0.0
            else:
                roll = asin(ay/cos(pitch))

            # mx, my, mz = mag
            x = mx*cos(pitch)+mz*sin(pitch)
            y = mx*sin(roll)*sin(pitch)+my*cos(roll)-mz*sin(roll)*cos(pitch)
            heading = atan2(y, x)

            # wrap heading between 0 and 360 degrees
            if heading > 2*pi:
                heading -= 2*pi
            elif heading < 0:
                heading += 2*pi

            # if self.angle_units == Angle.degrees:
            #     roll    *= RAD2DEG
            #     pitch   *= RAD2DEG
            #     heading *= RAD2DEG
            # elif self.angle_units == Angle.quaternion:
            #     return Quaternion.from_euler(roll, pitch, heading)

            return (roll, pitch, heading,)

        except ZeroDivisionError as e:
            print('Error', e)
            # if self.angle_units == Angle.quaternion:
            #     return Quaternion(1, 0, 0, 0)
            # else:
            return (0.0, 0.0, 0.0,)
