# steering_task.py
# ==============================================================================
# SteeringTask (outer loop for closed-loop line following)
# ------------------------------------------------------------------------------
# Uses the IR sensor array to compute a steering correction and publish left
# right motor VELOCITY setpoints (to maintain the inner closed-loop control)
# for the MotorControlTask.
# ==============================================================================

from pyb import millis

class SteeringTask:
    """Outer-loop controller for closed-loop line following."""

    # FSM States
    S0_INIT = 0
    S1_WAIT_ENABLE = 1
    S2_FOLLOW = 2
    S3_LOST = 3
    S4_HEADING = 4

    def __init__(self, ir_array, battery,
                 control_mode, mtr_enable,
                 left_sp_sh, right_sp_sh,
                 k_line, lf_target, bias,
                 abs_x_sh, abs_y_sh, abs_theta_sh,
                 nav_target_x_sh, nav_target_y_sh, nav_speed_sh, heading, k_heading, heading_setpoint):
        # Hardware
        self.ir = ir_array
        self.battery = battery

        # Shares
        self.left_sp_sh = left_sp_sh # share for left motor velocity setpoint
        self.right_sp_sh = right_sp_sh # share for right motor velocity setpoint
        self.control_mode = control_mode # 0: effort, 1: velocity, 2: line-following, 3: waypoint navigation
        self.mtr_enable = mtr_enable # share for motor enable flag
        self.k_line_sh = k_line # share for line-following gain
        self.lf_target_sh = lf_target # share for line-following target speed

        self.k_heading = k_heading # gain for heading control
        self.heading = heading # share for current heading
        self.heading_setpoint = heading_setpoint # share for desired heading

        # Local copies (to be updated on S1-->S2 transition)
        self.bias = bias # centroid bias to influence line 
        self.abs_x_sh = abs_x_sh
        self.abs_y_sh = abs_y_sh
        self.abs_theta_sh = abs_theta_sh
        self.nav_target_x_sh = nav_target_x_sh
        self.nav_target_y_sh = nav_target_y_sh
        self.nav_speed_sh = nav_speed_sh

        # Lost-line behavior
        self.search_speed = 0.5 # fraction of v_target to creep forward while searching
        self.reacquire_thresh = 0.05  # minimum sum(norm) threshold to consider the line as "seen"

        # FSM state initialization
        self.state = self.S0_INIT

        # Timing
        self.last_time = millis()

        self.first = 1

    # --------------------------------------------------------------------------
    ### HELPER FUNCTIONS
    # --------------------------------------------------------------------------
    def _compute_clamp_bound(self):
        """Compute the maximum speed clamp."""
        idx_min = min(self.ir.sensor_index)
        idx_max = max(self.ir.sensor_index)
        half_span = 0.5 * (idx_max - idx_min) if idx_max > idx_min else 1.0
        max_sp = abs(self.lf_target_sh.get()) + abs(self.k_line_sh.get()) * half_span
        return max(max_sp, 1.0) # avoid zero clamp

    # --------------------------------------------------------------------------
    def _publish(self, v_left, v_right):
        max_sp = self._compute_clamp_bound()
        v_left = max(min(v_left, max_sp), -max_sp)
        v_right = max(min(v_right, max_sp), -max_sp)
        self.left_sp_sh.put(v_left)
        self.right_sp_sh.put(v_right)

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        while True:
            # S0: INIT ---------------------------------------------------------
            if self.state == self.S0_INIT:
                self.bias.put(0.0)
                self._publish(0.0, 0.0) # ensure motors are stopped
                self.state = self.S1_WAIT_ENABLE

            # S1: WAIT FOR ENABLE ----------------------------------------------
            elif self.state == self.S1_WAIT_ENABLE:
                self._publish(0.0, 0.0) # ensure motors are stopped
                if self.control_mode.get() == 2: # if line following enabled
                    self.state = self.S2_FOLLOW # go to FOLLOW state
                elif self.control_mode.get() == 3:
                    self.state = self.S4_HEADING # go to FOLLOW state

            # S2: FOLLOW LINE --------------------------------------------------
            elif self.state == self.S2_FOLLOW:
                if self.control_mode.get() == 2:
                    if self.mtr_enable.get():
                        # print("SteeringTask: Line-following active.")
                        # Read gains and recalculate clamp
                        centroid, seen = self.ir.get_centroid() # get line centroid
                        if not seen:
                            # No line detected -> go to LOST
                            self.state = self.S3_LOST
                        else:
                            center = self.ir.center_index() # ideal centroid index
                            # Normalize error to [-1, 1] based on sensor index span
                            idx_min = min(self.ir.sensor_index)
                            idx_max = max(self.ir.sensor_index)
                            half_span = 0.5 * (idx_max - idx_min) if idx_max > idx_min else 1.0
                            error_norm = (centroid - center) / half_span + self.bias.get() # -1 (far left) to +1 (far right)

                            correction = self.k_line_sh.get() * error_norm # steering correction
                            v_left = self.lf_target_sh.get() + correction # correct steering
                            v_right = self.lf_target_sh.get() - correction # correct steering
                            # print(f"SteeringTask: centroid={centroid:.2f}, error_norm={error_norm:.2f}, correction={correction:.2f}, v_left={v_left:.2f}, v_right={v_right:.2f}")
                            self._publish(v_left, v_right)
                else:
                    # print("SteeringTask: Line-following disabled.")
                    self._publish(0.0, 0.0) # ensure motors are stopped
                    self.state = self.S1_WAIT_ENABLE # go to WAIT ENABLE

            # S3: LOST LINE --------------------------------------------------
            elif self.state == self.S3_LOST:
                if self.control_mode.get() != 2: # if line following disabled
                    self._publish(0.0, 0.0) # ensure motors are stopped
                    self.state = self.S1_WAIT_ENABLE # go to WAIT ENABLE state
                else:
                    # Gentle creep to reacquire (or set both to 0.0 if you prefer a stop)
                    v = self.search_speed * self.lf_target_sh.get()
                    self._publish(v, v)

                    # Quick check for line reacquisition
                    # (use read() to avoid computing centroid every tick)
                    norm = self.ir.read()
                    if sum(norm) > 0.05: # reacquire threshold
                        self.state = self.S2_FOLLOW # consider line reacquired

            # S4: FOLLOW HEADING ------------------------------------------------
            elif self.state == self.S4_HEADING:
                if self.control_mode.get() == 3:
                    if self.mtr_enable.get():
                        error_norm = (self.heading.get() - self.heading_setpoint.get()) / 180
                        correction = self.k_heading.get() * error_norm # steering correction
                        v_left = self.lf_target_sh.get() + correction # correct steering
                        v_right = self.lf_target_sh.get() - correction # correct steering
                        self._publish(v_left, v_right)

                else:
                    self._publish(0.0, 0.0) # ensure motors are stopped
                    self.state = self.S1_WAIT_ENABLE # go to WAIT ENABLE state

            yield self.state