# Reckless Ted's Funland Sensors (RTF_Sensors)


## Configuration Files

General layout for the `yaml` file, but different sensors have different
layouts:

```
sensor1:
    rate: int_hz # for both sample and publish
    range: float
    bias:
    - x
    - y
    - z
    cov:
    - 1
    - 2
    - 3
    - 4
    - 5
    - 6
    - 7
    - 8
    - 9
sensor2:
    ...
```

# To Do

- [ ] Pass parameters via `yaml` file
- Examples for sensors
- Calibration Jupyter notebooks for:
    - [ ] Accelerometer
    - [ ] Gyroscope
    - [ ] Magnetometer
    - [ ] Temperature?
    - [ ] Pressure?
