from pyb import Pin, ExtInt

# Initialize and configure motor pins
Left_nSLP = Pin('PB3', mode=Pin.OUT_PP)
Right_nSLP = Pin('PC9', mode=Pin.OUT_PP)

# Ensure motors are disabled
def disable_motors():
    Left_nSLP.low()
    Right_nSLP.low()
    print("Motors disabled.")

disable_motors()

def callback(line):
    print("line =", line)

extint = ExtInt('H0', ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback)
extint = ExtInt('H1', ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback)
extint = ExtInt('C10', ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback)