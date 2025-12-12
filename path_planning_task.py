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
    S6_SEG6 = 6
    S9_DONE = 9

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
    def __init__(self, planning, bias, total_s_sh, abs_x_sh, abs_y_sh, abs_theta_sh, kp, ki, k_line, lf_target, control_mode, driving_mode, abort, mtr_enable, setpoint, stream_data, heading, heading_setpoint, k_heading, eff):

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
        self.heading = heading
        self.heading_setpoint = heading_setpoint
        self.k_heading = k_heading
        self.eff = eff

        
        # Parameters
        self.bias = bias
        self.kp = kp
        self.ki = ki
        self.k_line = k_line
        self.lf_target = lf_target
        self.control_mode = control_mode
        self.driving_mode = driving_mode
        
        self.threshold = [580, 720, 910, 1070, 1550, 1750, 2000, 3140, 3640, 4220]  # Example thresholds for total displacement
        self.first = 1
        self.checkpoint = 0
        self.pivotting = False
        self.maneuver = 0

        self.planning.put(0)  # Initialize planning flag to 0
        self.state = self.S0_INIT # start the FSM in the INIT state

    # --------------------------------------------------------------------------
    ### HELPER FUNCTIONS
    # --------------------------------------------------------------------------
    
    def pivot_in_place(self, effort, target):
        error = target - self.heading.get()
        # print("Pivoting to", target, "degrees. Current heading:", self.heading.get(), "Error:", error)
        self.control_mode.put(0)  # Set to effort control mode
        self.driving_mode.put(1)  # Set to pivot mode
        if abs(error) > 5.0:
            if error > 0:
                self.eff.put(-effort)
            else:
                self.eff.put(effort)
        else:
            print("Leaving pivot function. Heading ", self.heading.get())
            self.eff.put(0)
            self.pivotting = False
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
                    self.total_s_sh.put(0.0)  # Reset total displacement
                    print("Path Planning: Initialized")
                    self.state = self.S1_SEG1
            # S1: SEGMENT 1 ----------------------------------------------------
            elif self.state == self.S1_SEG1:
                if self.first == 1:
                    print("Path Planning: Segment 1 started.")
                    print("Heading:", self.heading.get())
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
                    print("Heading:", self.heading.get())
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
                    print("Heading:", self.heading.get())
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
                    print("Heading:", self.heading.get())
                    self.first = 0

                if self.total_s_sh.get() > self.threshold[6]:
                    if self.checkpoint < 4:
                        print("Checkpoint 4 reached.")
                        self.checkpoint += 1

                if self.total_s_sh.get() > self.threshold[7]:
                    # large turn to CP#3 maneuver
                    # keep line-following on
                    # self.k_line.put(7.0) # keep k_line high for fast turn
                    # self.lf_target.put(10.0) # fast
                    self.control_mode.put(0)
                    self.pivotting = True
                    self.first = 1
                    self.state = self.S5_SEG5

            # S5: SEGMENT 5 ----------------------------------------------------
            elif self.state == self.S5_SEG5:
                if self.first == 1:
                    print("Path Planning: Segment 5 started.")
                    print("Heading:", self.heading.get())
                    self.pivotting = True
                    self.first = 0
                
                if self.maneuver == 0:
                    if self.pivotting:
                        self.pivot_in_place(effort=10, target=163.3)  # Pivot to 120 degrees
                    else:
                        self.maneuver += 1
                        self.pivotting = True
                        print("First pivot complete.")

                if self.maneuver == 1:
                    if self.checkpoint < 5:
                        print("Checkpoint 5 reached.")
                        self.checkpoint += 1
                        # Do control on heading
                        self.kp.put(4.0)
                        self.ki.put(0.0)
                        self.k_heading.put(6.0)
                        self.lf_target.put(4.0)
                        self.heading_setpoint.put(163.3)
                        self.control_mode.put(3)

                    if self.total_s_sh.get() > self.threshold[8]:
                        self.maneuver += 1
                        self.pivotting = True
                        print("Heading control complete. Preparing for second pivot.")
                
                if self.maneuver == 2:
                    if self.pivotting:
                        self.pivot_in_place(effort=10, target=180)  # Pivot to 0 degrees
                    else:
                        self.maneuver += 1
                        self.pivotting = True
                        print("Second pivot complete.")
                
                if self.total_s_sh.get() > self.threshold[9]:
                    if self.checkpoint < 6:
                        print("Checkpoint 6 reached.")
                        self.checkpoint += 1
                        self.heading_setpoint.put(180.0)
                        self.control_mode.put(3)
                        self.maneuver += 1
                        self.first = 1
                        self.state = self.S6_SEG6

            elif self.state == self.S6_SEG6:
                if self.first == 1:
                    print("Path Planning: Segment 6 started.")
                    print("Heading:", self.heading.get())
                    self.first = 0

                # Add any specific logic for Segment 6 here

                if self.total_s_sh.get() > self.threshold[9]:
                    self.first = 1
                    self.state = self.S9_DONE


                    
            # S6: DONE ---------------------------------------------------------
            elif self.state == self.S9_DONE:
                self.abort.put(1)  # debugging stop
                print("Path Planning: Done.")
                self.planning.put(0)
                self.abort.put(1)
                self.state = self.S0_INIT

            yield self.state # Yield control to allow other tasks to run
