from pyb import Pin, Timer
from time import ticks_us, ticks_diff   # Use to get dt value in update()
import math

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class. Provides position and velocity.'''

    # --------------------------------------------------------------------------
    # CONSTANTS
    GEAR_RATIO = 3952/33  # Gear ratio of motor to wheel (~120)
    CPR_MOTOR = 12 # Counts per rev of the motor shaft (before gearbox)
    CPR_WHEEL = GEAR_RATIO*CPR_MOTOR  # Counts per rev of the wheel (~1440)
    RAD_PER_COUNT = 2 * math.pi / CPR_WHEEL  # Radians per count
    WHEEL_RADIUS_MM = 35  # Wheel radius in mm
    # --------------------------------------------------------------------------

    def __init__(self,
                 tim_num: int,
                 chA_pin: Pin,
                 chB_pin: Pin):
        '''Initializes an Encoder object'''
        self.AR = 65535         # Auto-reload value for 16-bit timer
        self.tim = Timer(tim_num, prescaler=0, period=self.AR) # freq is ignored
        self.chA_pin = Pin(chA_pin)
        self.chB_pin = Pin(chB_pin)
        self.tim.channel(1, mode=Timer.ENC_AB, pin=self.chA_pin)
        self.tim.channel(2, mode=Timer.ENC_AB, pin=self.chB_pin)

        # Internal states
        self.position_counts   = 0     # Total accumulated position counts
        self.prev_count = self.tim.counter() # initialize from current counter
        self.delta = 0      # Change in count between last two updates
        self.prev_time = ticks_us()    # Time from most recent update
        self.dt         = 0     # Amount of time between last two updates
        self.velocity_counts_per_s = 0  # Velocity in counts per second
        
    # --------------------------------------------------------------------------
    def update(self):
        '''Update encoder count and compute velocity.'''

        # Read current count and compute delta
        curr_count = self.tim.counter() # read current count from timer
        delta = curr_count - self.prev_count # delta is change in counts

        # Check if delta is out of range (handle 16-bit over/underflow)
        half_range = (self.AR + 1) // 2  # 32768 for 16-bit timer
        if delta < -half_range:
            delta += self.AR + 1
        elif delta > half_range:
            delta -= self.AR + 1
        
        # Update count-based position
        self.position_counts += delta # update position
        
        # Compute time since last update
        curr_time = ticks_us()
        dt = ticks_diff(curr_time, self.prev_time)
        
        # Update dt
        self.dt = dt

        # Compute raw velocity (counts/s)
        if self.dt != 0:
            self.velocity_counts_per_s = delta*1e6 / self.dt
        else: # this shouldn't happen, but just in case, avoid division by zero
            self.velocity_counts_per_s = 0

        # Save for next update
        self.delta = delta
        self.prev_count = curr_count
        self.prev_time = curr_time

    # --------------------------------------------------------------------------
    def zero(self):
        '''Zero the encoder position.'''
        self.position_counts = 0
        self.prev_count = self.tim.counter()
        self.prev_time = ticks_us()

    # --------------------------------------------------------------------------
    def get_position_counts(self):
        '''Return position in counts.'''
        return self.position_counts  # position [counts]
    
    # --------------------------------------------------------------------------
    def get_position_rad(self):
        '''Return position in radians.'''
        return self.position_counts * self.RAD_PER_COUNT  # position [rad]

    # --------------------------------------------------------------------------
    def get_position_mm(self):
        '''Return position in millimeters.'''
        return self.get_position_rad() * self.WHEEL_RADIUS_MM  # position [mm]

    # --------------------------------------------------------------------------
    def get_velocity_counts_s(self):
        '''Return velocity in counts per second.'''
        return self.velocity_counts_per_s  # velocity [counts/s]
    
    # --------------------------------------------------------------------------
    def get_velocity_rad_s(self):
        '''Return velocity in radians per second.'''
        return self.get_velocity_counts_s() * self.RAD_PER_COUNT  # velocity [rad/s]

    # --------------------------------------------------------------------------
    def get_velocity_mm_s(self):
        '''Return velocity in millimeters per second.'''
        return self.get_velocity_rad_s() * self.WHEEL_RADIUS_MM  # velocity [mm/s]