# Script to test IMU module functionality

from pyb import Pin, I2C
from IMU_sensor import IMU
from os import listdir
from time import sleep_ms

# Initialize and configure pins
Left_nSLP = Pin('PB3', mode=Pin.OUT_PP)
Right_nSLP = Pin('PC9', mode=Pin.OUT_PP)
sda = Pin('PB14', mode=Pin.ALT, alt=4)
scl = Pin('PB13', mode=Pin.ALT, alt=4)
# reset = Pin('PB15', mode=Pin.OUT_PP)

# Ensure motors are disabled
def disable_motors():
    Left_nSLP.low()
    Right_nSLP.low()
    print("Motors disabled.")

disable_motors()

# Initialize I2C interface
i2c = I2C(2, mode=I2C.CONTROLLER, baudrate=400000)

# Initialize IMU object
imu = IMU(i2c)
imu.set_operation_mode("config")  # start in config mode

filelist = listdir()

# Check for existing IMU calibration file
if "imu_cal.bin" in filelist:
    print("IMU calibration file 'imu_cal.bin' found.")
    imu.write_calibration_coeffs()
else:
    print("No IMU calibration file found.")

imu.set_operation_mode("ndof")  # set to fusion mode


print("Starting IMU test...")

while True:
    heading, roll, pitch = imu.read_euler_angles()
    print("Heading: {:.2f} deg, Roll: {:.2f} deg, Pitch: {:.2f} deg".format(heading, roll, pitch))
    wx, wy, wz = imu.read_angular_velocity()
    print("Angular Velocity - wx: {:.2f} deg/s, wy: {:.2f} deg/s, wz: {:.2f} deg/s".format(wx, wy, wz))
    sleep_ms(500)
