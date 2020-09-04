# Reckless Ted's Funland Sensors (RTF_Sensors)

Here are some ROS2 drivers to publish Adafruit senor information:

- Acceleration and Gyroscope
    - [lsm6dsox](https://www.adafruit.com/product/4517)
    - [lsm6ds33](https://www.adafruit.com/product/4485)
- Magnetometer
    - [lis3mdl](https://www.adafruit.com/product/4517)
- Pressure and Temperature
    - [dps310](https://www.adafruit.com/product/4494)
    - [lps22](https://www.adafruit.com/product/4633)
- Infrared Camera
    - [mlx90640](https://www.adafruit.com/product/4469)
- Light
    - [bh1750](https://www.adafruit.com/product/4681)

To help with development of the I2C ROS2 drivers, I use Adafruit's
[MCP2221A](https://www.adafruit.com/product/4471) to develop on my laptop with.
It works pretty good, but you can't get the same performance as a Raspberry Pi
can talking to the sensors using native I2C interface.

```
import os
if 'BLINKA_MCP2221' in os.environ.keys():
    pass
else:
    os.environ['BLINKA_MCP2221'] = "1"
```

## To Build

```
git clone https://github.com/RecklessTedsFunland/rtf_sensors.git
colcon build --symlink-install --packages-select rtf_sensors
source install/setup.bash

# now run an example
ros2 run rtf_sensors executor_st
```

## Configuration Files

General layout for the `yaml` file, but different sensors have different
layouts:

```
lsm6sox:
  accel:
    data:
    - 0.10124958887273922
    - 2.419810569214096e-05
    - -0.0007916959137182327
    - -0.0008555830320481476
    - 0.10181139633682373
    - 0.001510699098028933
    - 0.0003137800905107515
    - -0.0015629026939724747
    - 0.10216486651751974
    - 0.004015925094103206
    - 0.008329605644502025
    - -0.024763812324729968
    dtype: float64
    shape:
    - 4
    - 3
  gyro:
    data:
    - 1.0
    - 0.0
    - 0.0
    - 0.0
    - 1.0
    - 0.0
    - 0.0
    - 0.0
    - 1.0
    - 0.0001524393485238419
    - -0.024156762226978153
    - -0.010172667688119676
    dtype: float64
    shape:
    - 4
    - 3
  timestamp: 2020-08-30 12:32:45.540809
```

# To Do

- [ ] Pass parameters via `yaml` file
- Examples for sensors
- Calibration Jupyter notebooks for:
    - [x] Accelerometer
    - [x] Gyroscope
    - [x] Magnetometer
    - [ ] Temperature?
    - [ ] Pressure?


# References

- Conventions for IMU Sensor Drivers: [REP-0145](http://docs.ros.org/independent/api/rep/html/rep-0145.html)

# MIT License

**Copyright (c) 2020 Reckless Ted's Funland**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
