# spectator_task.py
#
# ==============================================================================
# SpectatorTask
# ------------------------------------------------------------------------------
# This task predicts the absolute position of the robot using encoder data
# ==============================================================================

import math

class SpectatorTask:
    """Estimates the absolute position of the robot using encoder data."""

    # --------------------------------------------------------------------------
    # CONSTANTS FOR UNIT CONVERSIONS
    GEAR_RATIO = 3952/33  # Gear ratio of motor to wheel (~120)
    CPR_MOTOR = 12 # Counts per rev of the motor shaft (before gearbox)
    CPR_WHEEL = GEAR_RATIO*CPR_MOTOR  # Counts per rev of the wheel (~1440)
    RAD_PER_COUNT = 2 * math.pi / CPR_WHEEL  # Radians per count
    WHEEL_RADIUS_MM = 35  # Wheel radius in mm
    WHEEL_BASE_MM = 141  # Distance between wheels in mm
    # --------------------------------------------------------------------------

    # The states of the FSM
    S0_INIT = 0
    S1_WAITING = 1
    S2_ESTIMATING = 2

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self, run_observer,
                 left_pos_sh, right_pos_sh, total_s_sh,
                 abs_x_sh, abs_y_sh, abs_theta_sh,
                 x0_mm=0.0, y0_mm=0.0, theta0_rad=0.0):

        # Shares
        self.left_pos_sh = left_pos_sh
        self.right_pos_sh = right_pos_sh
        self.total_s_sh = total_s_sh
        self.abs_x_sh = abs_x_sh
        self.abs_y_sh = abs_y_sh
        self.abs_theta_sh = abs_theta_sh

        # World-frame initial pose (able to be changed)
        self.x0_mm = float(x0_mm)
        self.y0_mm = float(y0_mm)
        self.theta0_rad = float(theta0_rad)

        # Internal pose state (world frame)
        self.x_mm = self.x0_mm
        self.y_mm = self.y0_mm
        self.theta_rad = self.theta0_rad

        # Flags
        self.run_observer = run_observer

        self.state = self.S0_INIT # Start in the INIT state

    # --------------------------------------------------------------------------
    ### HELPER FUNCTIONS
    # --------------------------------------------------------------------------
    def set_initial_pose(self, x_mm, y_mm, theta_rad=0.0):
        """!
        Set the initial pose of the robot in the world/game coordinate frame.

        This pose will be used the next time the observer is started.
        For example, if the track origin is at (0, 0) and Romi's starting
        position is (300 mm, 150 mm) with heading pi/2 rad, you would call:

            spectator.set_initial_pose(300.0, 150.0, math.pi/2)

        before run_observer is asserted.
        """
        self.x0_mm      = float(x_mm)
        self.y0_mm      = float(y_mm)
        self.theta0_rad = float(theta_rad)

    # --------------------------------------------------------------------------
    def _reset_odometry(self):
        """!
        At the start of a run, reset the accumulated pose and previous encoder counts.
        """

        # Set pose to the configured initial world-frame pose
        self.x_mm      = self.x0_mm
        self.y_mm      = self.y0_mm
        self.theta_rad = self.theta0_rad

        # Push those to the shares so other tasks see the reset immediately
        self.abs_x_sh.put(self.x_mm)
        self.abs_y_sh.put(self.y_mm)
        self.abs_theta_sh.put(self.theta_rad)

        # Reset center-line distance
        if self.total_s_sh is not None:
            self.total_s_sh.put(0.0)

        # Read current encoder positions to define our baselines
        left_counts  = self.left_pos_sh.get()
        right_counts = self.right_pos_sh.get()

        # Baselines for "since start of run" and "since last step"
        self.start_left_counts   = left_counts
        self.start_right_counts  = right_counts
        self.prev_left_counts    = left_counts
        self.prev_right_counts   = right_counts


    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        while True:
            ### 0: INIT STATE --------------------------------------------------
            if self.state == self.S0_INIT:
                # Initialize pose in world frame based on configured initial pose
                self._reset_odometry()
                # Transition to WAITING state
                self.state = self.S1_WAITING
                
            ### 1: WAITING STATE -----------------------------------------------
            elif self.state == self.S1_WAITING:
                # Wait until another task tells us to start observing
                if self.run_observer.get():
                    # When starting a new run, reset the pose and baselines
                    self._reset_odometry()
                    # Transition to ESTIMATING state
                    self.state = self.S2_ESTIMATING
                    
            ### 2: ESTIMATING STATE --------------------------------------------
            elif self.state == self.S2_ESTIMATING:

                # If told to stop observing, transition to WAITING state
                if not self.run_observer.get():
                    self.state = self.S1_WAITING
                    continue  # Skip the rest of this loop iteration

                # Read current encoder positions [counts]
                left_counts = self.left_pos_sh.get()
                right_counts = self.right_pos_sh.get()

                # Incremental counts since last update
                d_counts_L = left_counts - self.prev_left_counts
                d_counts_R = right_counts - self.prev_right_counts

                # Convert to wheel displacements [mm]
                d_sL = d_counts_L * self.RAD_PER_COUNT * self.WHEEL_RADIUS_MM
                d_sR = d_counts_R * self.RAD_PER_COUNT * self.WHEEL_RADIUS_MM

                # Compute the incremental displacement at Romi's center [mm]
                d_s = (d_sL + d_sR) / 2.0

                # Compute the incremental change in heading [rad]
                d_theta = (d_sR - d_sL) / self.WHEEL_BASE_MM

                # Assume that during this small time step interval, Romi's heading changes linearly; thus, the average heading is:
                theta_mid = self.theta_rad + (d_theta / 2.0)

                # Integrate pose in the world frame
                self.x_mm     += d_s * math.cos(theta_mid)
                self.y_mm     += d_s * math.sin(theta_mid)
                self.theta_rad += d_theta

                # Update previous encoder readings [counts] for next iteration
                self.prev_left_counts  = left_counts
                self.prev_right_counts = right_counts

                # Calculate the total distance traveled along the center line since the start of the run [mm]
                center_counts = ((left_counts - self.start_left_counts) + (right_counts - self.start_right_counts)) / 2.0
                total_s_mm = center_counts * self.RAD_PER_COUNT * self.WHEEL_RADIUS_MM

                # Update shares with new pose estimates
                self.total_s_sh.put(total_s_mm)
                self.abs_x_sh.put(self.x_mm)
                self.abs_y_sh.put(self.y_mm)
                self.abs_theta_sh.put(self.theta_rad)

                # For debugging: print the estimated pose
                # print(f"{total_s_mm}, {self.x_mm}, {self.y_mm}, {self.theta_rad}")

            yield self.state # Yield control to allow other tasks to run