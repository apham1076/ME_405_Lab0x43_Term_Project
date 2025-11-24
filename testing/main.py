# Script to test IMU module functionality

from pyb import Pin, I2C, ExtInt
from IMU_sensor import IMU
from os import listdir
from time import sleep_ms, ticks_us, ticks_diff
import gc

# Initialize and configure pins
Left_nSLP = Pin('PB3', mode=Pin.OUT_PP)
Right_nSLP = Pin('PC9', mode=Pin.OUT_PP)
# sda = Pin('PB14', mode=Pin.ALT, alt=4)
# scl = Pin('PB13', mode=Pin.ALT, alt=4)
# reset = Pin('PB15', mode=Pin.OUT_PP)

# Initialize I2C interface
# i2c = I2C(2, mode=I2C.CONTROLLER, baudrate=400000)

# Initialize IMU object
# imu = IMU(i2c)
# imu.set_operation_mode("config")  # start in config mode

# Ensure motors are disabled
def disable_motors():
    Left_nSLP.low()
    Right_nSLP.low()
    print("Motors disabled.")

disable_motors()



# filelist = listdir()

# Check for existing IMU calibration file
# if "imu_cal.bin" in filelist:
#     print("IMU calibration file 'imu_cal.bin' found.")
#     imu.write_calibration_coeffs()  
# else:
#     print("No IMU calibration file found.")

# imu.set_operation_mode("ndof")  # set to fusion mode

# Configure external interrupt on INT pin



print("Starting IMU test...")

gc.collect()  # Run garbage collector to free up memory

# while True:
#     imu._read_reg(imu.reg.EULER_DATA_ALL,debug=True)
#     sleep_ms(500)
