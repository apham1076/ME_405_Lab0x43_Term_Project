# Script to test IMU module functionality

from pyb import Pin, I2C, ExtInt
from IMU_sensor import IMU
from os import listdir
from time import sleep_ms, ticks_us, ticks_diff
import gc

# Initialize I2C interface
sda = Pin('PB14', mode=Pin.ALT, alt=4)
scl = Pin('PB13', mode=Pin.ALT, alt=4)
i2c = I2C(2, mode=I2C.CONTROLLER, baudrate=400000)

# Initialize and configure motor pins
Left_nSLP = Pin('PB3', mode=Pin.OUT_PP)
Right_nSLP = Pin('PC9', mode=Pin.OUT_PP)

# Initialize IMU object
imu = IMU(i2c)
imu.set_operation_mode("config")  # start in config mode

# Ensure motors are disabled
def disable_motors():
    Left_nSLP.low()
    Right_nSLP.low()
    print("Motors disabled.")

disable_motors()



filelist = listdir()

# Check for existing IMU calibration file
if "imu_cal.bin" in filelist:
    print("IMU calibration file 'imu_cal.bin' found.")
    imu.write_calibration_coeffs()  
else:
    print("No IMU calibration file found.")

imu.set_operation_mode("ndof")  # set to fusion mode
sleep_ms(1000)  # wait for mode switch

# Configure external interrupt on INT pin



print("Starting IMU test...")

gc.collect()  # Run garbage collector to free up memory

# while True:
#     cal_stat = imu.read_calibration_status()
#     print("Calibration status (sys, gyr, acc, mag):", cal_stat)
#     # t1 = ticks_us()
#     # imu._read_reg(imu.reg.ACC_DATA_ALL)
#     # t2 = ticks_us()
#     # print("Acceleration (mg): X={0}, Y={1}, Z={2}".format(ax, ay, az))
#     # print("Time to read acceleration (us):", ticks_diff(t2, t1))
#     sleep_ms(500)
def callback(line):
    print("line =", line)
    extint.disable()

extint = ExtInt('H0', ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback)
extint2 = ExtInt('H1', ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback)
extint3 = ExtInt('C10', ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback)
extint4 = ExtInt('C11', ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback)
extint5 = ExtInt('C12', ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback)
extint6 = ExtInt('D2', ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback)