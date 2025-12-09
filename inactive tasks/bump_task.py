# bump_task.py
#
# ==============================================================================
# BumpTask
# ------------------------------------------------------------------------------
# Task to handle bump sensor events and set abort flag
# ==============================================================================

from pyb import ExtInt, Pin
from time import ticks_ms, ticks_diff

class BumpTask:
    """Task to handle bump sensor events and set abort flag"""

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self, abort, bump_pin='H0'):
        # Share to set abort flag
        self.abort = abort
        self.bump = 0
        self.bump_pin = bump_pin

        # Configure bump sensor interrupt
        def callback(line):
            print("Bump sensor triggered on line =", line)
            self.extint.disable()  # Disable further interrupts to avoid multiple triggers
            self.bump = 1   
            self.abort.put(1)  # Set abort flag when bump sensor is triggered

        self.extint = ExtInt(bump_pin, ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback)

    def run(self):
        while True:
            if self.bump == 1:
                start = ticks_ms()
                yield
                while self.bump == 1:
                    now = ticks_ms()
                    if ticks_diff(now, start) > 2000:  # 2 seconds debounce
                        self.bump = 0
                        self.extint.enable()  # Re-enable interrupt after debounce
                        print("Bump sensor re-enabled after debounce.")
                    yield
            yield