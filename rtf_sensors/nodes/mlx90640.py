# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2020 Kevin Walchko
# see LICENSE for full details
##############################################


# import rclpy
from rclpy.node import Node
import board
import busio
from collections import deque
from colorama import Fore
from pyftdi.i2c import I2cNackError
import adafruit_mlx90640
from rtf_interfaces import ImageIR


class rtf_mlx90640(Node):
    
    pixel_colors = {
        25: f"{Fore.BLACK}\u2588",
        27: f"{Fore.BLUE}\u2588",
        29: f"{Fore.CYAN}\u2588",
        31: f"{Fore.GREEN}\u2588",
        33: f"{Fore.YELLOW}\u2588",
        35: f"{Fore.RED}\u2588",
    }
    
    self.row = deque([f"{Fore.BLACK}\u2588"]*32,32)
        
    def __init__(self, i2c=None):
        super().__init__('rtf_mlx90640')

        if i2c is None:
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        else:
            self.i2c = i2c
            
        self.camera = adafruit_mlx90640.MLX90640(i2c)
        self.camera.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ
        self.frame = [0] * 24*32  # replace with deque?
        
        self.timer = self.create_timer(1/2, self.callback)
        
        self.pub_imu = self.create_publisher(ImageIR, 'image_ir', 10)
        self.msg = ImageIR()
        self.msg.header.frame_id = self.declare_parameter('frame_id', "base_imu_link").value
        self.msg.height = 24 # rows
        self.msg.width  = 32 # cols
        
    def callback(self):
        try:
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.camera.getFrame(self.frame)
            self.msg.data = self.frame
            self.pub.publish(self.msg)
        except ValueError as e:
            print(e)
            # these happen, no biggie - retry
            continue
        except I2cNackError as e:
            print(e)
            continue
            
    def scale(self, t):
        """
        Translate an IR pixel temperature into a simple ASCII color code for printing.
        """
#         u = "\u2588"
#         s = {
#             25: f"{Fore.BLACK}{u}",
#             27: f"{Fore.BLUE}{u}",
#             29: f"{Fore.CYAN}{u}",
#             31: f"{Fore.GREEN}{u}",
#             33: f"{Fore.YELLOW}{u}",
#             35: f"{Fore.RED}{u}",
#         }
        for k, v in self.pixel_colors.items():
            if t < k:
                return v
        return f"{Fore.RED}\u2588"
    
    def print(self):
        """
        Print the frame to the command line.
        """
        for i, t in enumerate(self.frame):
            if i%32 == 0:
                print("".join(self.row))

            self.row.append(self.scale(t))
        print(Fore.RESET)
        
