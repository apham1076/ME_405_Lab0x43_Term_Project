# closed_loop.py
# ==============================================================================
# PI Velocity Controller for Romi motors (in rad/s)
# ==============================================================================

from time import ticks_diff, ticks_ms
from math import pi

class ClosedLoop:
    """Proportional-Integral (PI) controller for velocity control in rad/s."""

    def __init__(self,
                 kp, ki,
                 sp_sh,
                 battery,
                 effort_limits=(-100, 100)):
        
        self.kp = kp
        self.ki = ki
        self.sp_sh = sp_sh
        self.effort_min, self.effort_max = effort_limits
        self.integrator = 0.0
        self.last_time = ticks_ms()
        self.output = 0.0
        self.last_output = 0.0

        self.battery = battery  # battery object for droop compensation

    def reset(self):
        """Reset controller integrator and state."""
        self.integrator = 0.0
        self.output = 0.0
        self.last_time = ticks_ms()

    def run(self, fb):
        """Compute control output (effort %) from velocity feedback and measured battery voltage."""
        now = ticks_ms()
        dt_ms = ticks_diff(now, self.last_time)
        
        # If task paused too long, hold last output instead of cutting to zero
        if dt_ms > 1000:
            self.last_time = now # reset time
            return self.last_output # hold last output
        
        dt = dt_ms / 1000.0  # Convert to seconds
        self.last_time = now

        # Convert velocity from count/s to rad/s
        cpr = 1440
        fb *= 2*pi / cpr
        
        # --- Core PI control ---
        error = self.sp_sh.get() - fb
        self.integrator += error * dt
        
        # Clamp integrator to reasonable bounds based on output limits (prevent windup)
        # max_integral = (self.effort_max - self.kp.get() * error) / (self.ki.get() + 1e-6)
        # min_integral = (self.effort_min - self.kp.get() * error) / (self.ki.get() + 1e-6)
        # self.integrator = max(min(self.integrator, max_integral), min_integral)
        
        # Base PI output (without droop compensation)
        u = self.kp.get() * error + self.ki.get() * self.integrator

        # --- Battery droop compensation ---
        if self.battery is not None:
            gain = self.battery.droop_gain()  # Calls the battery method to compute a gain for droop compensation
        else:
            gain = 1.0  # No compensation
        
        # Compute final output with droop compensation
        u *= gain # apply droop gain block to controller output
        
        # Final clamp on output (clamp effort to safe limits)
        u = max(min(u, self.effort_max), self.effort_min)

        # print(f"CL Controller | SP: {self.sp_sh.get():.2f}, FB: {fb:.2f}, Error: {error:.2f}, Effort: {u:.2f}, Gain: {gain:.3f}")
        
        self.output = u
        self.last_output = u
        return u