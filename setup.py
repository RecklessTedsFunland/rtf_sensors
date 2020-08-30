from setuptools import setup

package_name = 'rtf_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'squaternion',
        'pyyaml',
        'numpy',
        'slurm',
        'adafruit_circuitpython_lsm6ds',
        'adafruit_circuitpython_lis3mdl',
        'adafruit_circuitpython_lps2x',
        'adafruit_circuitpython_dps310',
        'sensor_msgs'
    ],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='walchko@users.noreply.github.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={  # colcon build --symlink-install
        'console_scripts': [
            'i2c_imu = rtf_sensors.imu_serial_node:main',
            'i2c_cal_imu = rtf_sensors.imu_calibration_node:main',
            'executor_st = rtf_sensors.examples.executor_st:main'
        ],
    },
)
