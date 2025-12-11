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

    CHECKPOINTS = {
        0: (100.0, 800.0),
        1: (950.0, 425.0),
        2: (1400.0, 800.0),
        3: (1350.0, 100.0),
        4: (800.0, 175.0),
        5: (175.0, 300.0),
        6: (100.0, 800.0)
    }

    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self, planning, bias, total_s_sh, abs_x_sh, abs_y_sh, abs_theta_sh, kp, ki, k_line, lf_target, control_mode, abort, mtr_enable, setpoint, stream_data):

        # Flags
        self.planning = planning
        self.abort = abort
        self.mtr_enable = mtr_enable
        self.stream_data = stream_data

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
        
        self.threshold = [580, 720, 910, 1070, 1550, 1750, 2000, 3140, 3640]  # Example thresholds for total displacement
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
                    self.stream_data.put(0)  # DISABLE data streaming
                    # Set line-following gains
                    self.kp.put(6.0)
                    self.ki.put(0.0)
                    self.k_line.put(7.0)
                    self.lf_target.put(10.0)
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
                    # Slow down for turn
                    self.lf_target.put(6.0)
                    # Turn right at checkpoint
                    self.bias.put(0.5)

                if self.total_s_sh.get() > self.threshold[1]:
                    self.first = 1
                    # Continue line-following but now with no bias
                    self.lf_target.put(7.0)
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
                        self.setpoint.put(7.0)
                        self.control_mode.put(1)  # Velocity mode
                        self.checkpoint += 1
                
                if self.total_s_sh.get() > self.threshold[3]:
                    # semi-circle maneuver
                    # Turn line-following back on
                    self.bias.put(-0.6)
                    self.lf_target.put(9.0)
                    self.control_mode.put(2)  # Line-following mode
                    self.first = 1
                    self.state = self.S3_SEG3
            # S3: SEGMENT 3 ----------------------------------------------------
            elif self.state == self.S3_SEG3:
                if self.first == 1:
                    print("Path Planning: Segment 3 started.")
                    self.first = 0

                if self.total_s_sh.get() > self.threshold[4]:
                    # dashed-line maneuver
                    self.bias.put(0.0)
                    self.k_line.put(5.0)
                    self.lf_target.put(8.0)
                    if self.checkpoint < 3:
                        print("Checkpoint 3 reached.")
                        self.checkpoint += 1
                    
                if self.total_s_sh.get() > self.threshold[5]:
                    # small turn to CP#2 maneuver
                    # Keep line-following on
                    self.k_line.put(7.0) # raise k_line for tight turn
                    self.first = 1
                    self.state = self.S4_SEG4
            # S4: SEGMENT 4 ----------------------------------------------------
            elif self.state == self.S4_SEG4:
                if self.first == 1:
                    print("Path Planning: Segment 4 started.")
                    self.first = 0

                if self.total_s_sh.get() > self.threshold[6]:
                    if self.checkpoint < 4:
                        print("Checkpoint 4 reached.")
                        self.checkpoint += 1

                if self.total_s_sh.get() > self.threshold[7]:
                    # large turn to CP#3 maneuver
                    # keep line-following on
                    self.k_line.put(7.0) # keep k_line high for fast turn
                    self.lf_target.put(10.0) # fast
                    self.first = 1
                    self.state = self.S6_DONE
                    
            # S6: DONE ---------------------------------------------------------
            elif self.state == self.S6_DONE:
                self.abort.put(1)  # debugging stop
                print("Path Planning: Done.")
                self.planning.put(0)
                self.abort.put(1)
                self.state = self.S0_INIT

            yield self.state # Yield control to allow other tasks to run
