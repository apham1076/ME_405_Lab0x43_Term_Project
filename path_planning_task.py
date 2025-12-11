# path_planning_task.py
#
# ==============================================================================
# PathPlanningTask
# ------------------------------------------------------------------------------
# This task is responsible for planning the path of the robot
# ==============================================================================

import math
class PathPlanningTask:
    """This task is responsible for planning the path of the robot."""

    # Examples states of the FSM
    S0_INIT = 0
    S1_SEG1 = 1
    S2_SEG2 = 2
    S3_SEG3 = 3
    S4_SEG4 = 4
    S5_SEG5 = 5
    S6_DONE = 6

    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self, planning, bias, total_s_sh, abs_x_sh, abs_y_sh, abs_theta_sh, kp, ki, k_line, lf_target, control_mode, abort, mtr_enable, setpoint):

        # Flags
        self.planning = planning
        self.abort = abort
        self.mtr_enable = mtr_enable

        # Shares
        self.total_s_sh = total_s_sh
        self.abs_x_sh = abs_x_sh
        self.abs_y_sh = abs_y_sh
        self.abs_theta_sh = abs_theta_sh
        self.setpoint = setpoint
        
        # Parameters
        self.bias = bias
        self.kp = kp
        self.ki = ki
        self.k_line = k_line
        self.lf_target = lf_target
        self.control_mode = control_mode
        
        self.threshold = [650, 700, 900, 1050]  # Example thresholds for total displacement
        self.first = 1
        self.checkpoint = 0

        self.planning.put(0)  # Initialize planning flag to 0
        self.state = self.S0_INIT # start the FSM in the INIT state

    # --------------------------------------------------------------------------
    ### HELPER FUNCTIONS
    # --------------------------------------------------------------------------
    


    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        """Main method for the PathPlanningTask FSM."""
        self.state = self.S0_INIT

        while True:
            # S0: INIT ---------------------------------------------------------
            if self.state == self.S0_INIT:
                # Initialization code here
                if self.planning.get() and self.mtr_enable.get():
                    # Set line-following gains
                    self.kp.put(6.0)
                    self.ki.put(0.0)
                    self.k_line.put(6.0)
                    self.lf_target.put(6.0)
                    self.control_mode.put(2)  # Line-following mode
                    print("Path Planning: Initialized")
                    self.state = self.S1_SEG1
            # S1: SEGMENT 1 ----------------------------------------------------
            elif self.state == self.S1_SEG1:
                if self.first == 1:
                    print("Path Planning: Segment 1 started.")
                    self.first = 0

                if self.total_s_sh.get() > self.threshold[0]:
                    if self.checkpoint < 1:
                        # Intersection maneuver
                        print("Checkpoint 1 reached.")
                        self.checkpoint += 1
                    # Turn right at checkpoint
                    self.bias.put(0.6)

                if self.total_s_sh.get() > self.threshold[1]:
                    self.first = 1
                    # Continue line-following
                    self.bias.put(0.0)
                    self.state = self.S2_SEG2
            # S2: SEGMENT 2 ----------------------------------------------------
            elif self.state == self.S2_SEG2:
                if self.first == 1:
                    print("Path Planning: Segment 2 started.")
                    self.first = 0

                if self.total_s_sh.get() > self.threshold[2]:
                    if self.checkpoint < 2:
                        # Diamond maneuver
                        print("Checkpoint 2 reached.")
                        # Turn off line-following and pass through diamond
                        self.setpoint.put(4.0)
                        self.control_mode.put(1)  # Velocity mode
                        self.checkpoint += 1
                
                if self.total_s_sh.get() > self.threshold[3]:
                    # Turn line-following back on
                    self.control_mode.put(2)  # Line-following mode
                    self.first = 1
                    self.state = self.S3_SEG3
            # S3: SEGMENT 3 ----------------------------------------------------
            elif self.state == self.S3_SEG3:
                if self.first == 1:
                    print("Path Planning: Segment 3 started.")
                    self.first = 0
                    
            elif self.state == self.S6_DONE:
                print("Path Planning: Done.")
                self.planning.put(0)
                self.abort.put(1)
                self.state = self.S0_INIT

            yield self.state # Yield control to allow other tasks to run
