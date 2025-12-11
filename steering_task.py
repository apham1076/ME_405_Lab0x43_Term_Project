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

    def __init__(self, ir_array, battery,
                 control_mode, mtr_enable,
                 left_sp_sh, right_sp_sh,
                 k_line, lf_target, bias):
        # Hardware
        self.ir = ir_array
        self.battery = battery

        # Shares
        self.left_sp_sh = left_sp_sh # share for left motor velocity setpoint
        self.right_sp_sh = right_sp_sh # share for right motor velocity setpoint
        self.control_mode = control_mode # 0: effort, 1: velocity, 2: line-following
        self.mtr_enable = mtr_enable # share for motor enable flag
        self.k_line_sh = k_line # share for line-following gain
        self.lf_target_sh = lf_target # share for line-following target speed
        # Local copies (to be updated on S1-->S2 transition)
        self.k_line_param = 0.0 # cached line-following gain
        self.v_target_param = 0.0 # cached nominal translational speed
        self._have_params = False # flag to indicate if params have been loaded, set true when S2 entered
        self.bias = bias # centroid bias to influence line following

        # Lost-line behavior
        self.search_speed = 0.5 # fraction of v_target to creep forward while searching
        self.reacquire_thresh = 0.05  # minimum sum(norm) threshold to consider the line as "seen"

        # FSM state initialization
        self.state = self.S0_INIT

        # Timing
        self.last_time = millis()

    # --------------------------------------------------------------------------
    ### HELPER FUNCTIONS
    # --------------------------------------------------------------------------
    def _compute_clamp_bound(self):
        """Compute the maximum speed clamp based on current parameters."""
        idx_min = min(self.ir.sensor_index)
        idx_max = max(self.ir.sensor_index)
        half_span = 0.5 * (idx_max - idx_min) if idx_max > idx_min else 1.0
        max_sp = abs(self.v_target_param) + abs(self.k_line_param) * half_span
        return max(max_sp, 1.0) # avoid zero clamp

    # --------------------------------------------------------------------------
    def _publish(self, v_left, v_right):
        max_sp = self._compute_clamp_bound()
        v_left = max(min(v_left, max_sp), -max_sp)
        v_right = max(min(v_right, max_sp), -max_sp)
        self.left_sp_sh.put(v_left)
        self.right_sp_sh.put(v_right)
        # print("Max speed clamp: {:.2f}, Published v_left: {:.2f}, v_right: {:.2f}".format(max_sp, v_left, v_right))

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

            # S2: FOLLOW LINE -------------------------------------------------
            elif self.state == self.S2_FOLLOW:

                # If motors are disabled, set the param flag False so that fresh params are loaded on next entry (after next enable)
                if not self.mtr_enable.get():
                    self._have_params = False # reset param flag
                    self._publish(0.0, 0.0) # ensure motors are stopped
                    # Stay in S2_FOLLOW (don't transition to S1) since control_mode may still be 2
                    yield self.state
                    continue

                # On entry to S2, load parameters (or reload if have_params was set False)
                if not self._have_params:
                    self.k_line_param = self.k_line_sh.get()
                    self.v_target_param = self.lf_target_sh.get()
                    self._have_params = True

                if self.control_mode.get() != 2: # if line following disabled
                    self._publish(0.0, 0.0) # ensure motors are stopped
                    self._have_params = False # reset param flag
                    self.state = self.S1_WAIT_ENABLE # go to WAIT ENABLE
                else:
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

                        correction = self.k_line_param * error_norm # steering correction
                        v_left = self.v_target_param + correction # correct steering
                        v_right = self.v_target_param - correction # correct steering
                        # print(f"Centroid: {centroid:.2f}, Error norm: {error_norm:.3f}, Correction: {correction:.3f}, v_left: {v_left:.2f}, v_right: {v_right:.2f}")
                        self._publish(v_left, v_right)

            # S3: LOST LINE --------------------------------------------------
            elif self.state == self.S3_LOST:
                if self.control_mode.get() != 2: # if line following disabled
                    self._publish(0.0, 0.0) # ensure motors are stopped
                    self._have_params = False # reset param flag
                    self.state = self.S1_WAIT_ENABLE # go to WAIT ENABLE state
                else:
                    # Gentle creep to reacquire (or set both to 0.0 if you prefer a stop)
                    v = self.search_speed * self.v_target_param
                    self._publish(v, v)

                    # Quick check for line reacquisition
                    # (use read() to avoid computing centroid every tick)
                    norm = self.ir.read()
                    if sum(norm) > 0.05: # reacquire threshold
                        self.state = self.S2_FOLLOW # consider line reacquired

            yield self.state