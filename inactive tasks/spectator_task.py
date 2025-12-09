# spectator_task.py
#
# ==============================================================================
# SpectatorTask
# ------------------------------------------------------------------------------
# This task predicts the absolute position of the robot using encoder data
# ==============================================================================

from ulab import numpy as np

class SpectatorTask:
    """Estimates the absolute position of the robot using encoder data."""

    # The states of the FSM
    S0_INIT = 0
    S1_ESTIMATING = 1

    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self, start,
                 left_pos_sh, right_pos_sh, total_s_sh,
                 abs_x_sh, abs_y_sh, abs_theta_sh):

        # Shares
        self.left_pos_sh = left_pos_sh
        self.right_pos_sh = right_pos_sh
        self.total_s_sh = total_s_sh
        self.abs_x_sh = abs_x_sh
        self.abs_y_sh = abs_y_sh
        self.abs_theta_sh = abs_theta_sh

        # Parameters
        self.r = 35  # wheel radius (mm)
        self.w = 141  # distance between wheels (mm)
        self.count2rad = 2*3.14159/1440   # Conversion factor

        self.start = start

        self.state = self.S0_INIT

    def run(self):
        """Main method for the SpectatorTask FSM."""
        state = self.S0_INIT

        while True:
            if state == self.S0_INIT:
                # Wait for start signal
                if self.start.get():
                    # Initialize absolute position and orientation
                    self.abs_x_sh.put(0.0)
                    self.abs_y_sh.put(0.0)
                    self.total_s_sh.put(0.0)
                    self.prev_left_pos = self.left_pos_sh.get()
                    self.prev_right_pos = self.right_pos_sh.get()
                    self.abs_theta_sh.put(0.0)
                    state = self.S1_ESTIMATING

            elif state == self.S1_ESTIMATING:
                # Read encoder positions
                left_pos = self.left_pos_sh.get()
                right_pos = self.right_pos_sh.get()

                # Compute total displacement
                total_s = (left_pos + right_pos) * self.count2rad * self.r / 2.0

                # Compute changes in encoder positions
                delta_sL = (left_pos - self.prev_left_pos) * self.count2rad * self.r
                delta_sR = (right_pos - self.prev_right_pos) * self.count2rad * self.r
    
                # Compute absolute position and orientation
                # (This is a placeholder for actual kinematic calculations)
                delta_s = (delta_sL + delta_sR) / 2.0
                delta_theta = (delta_sR - delta_sL) / self.w

                # Update previous positions
                abs_x = self.abs_x_sh.get() + delta_s * np.cos(self.abs_theta_sh.get() + delta_theta / 2.0)
                abs_y = self.abs_y_sh.get() + delta_s * np.sin(self.abs_theta_sh.get() + delta_theta / 2.0)
                abs_theta = self.abs_theta_sh.get() + delta_theta

                # Update shares
                self.total_s_sh.put(total_s)
                self.abs_x_sh.put(abs_x)
                self.abs_y_sh.put(abs_y)
                self.abs_theta_sh.put(abs_theta)
                print(f"Total_S: {total_s:.2f} mm, Abs X: {abs_x:.2f} mm, Abs Y: {abs_y:.2f} mm, Abs Theta: {abs_theta:.2f} rad")

            yield self.state # Yield control to allow other tasks to run
