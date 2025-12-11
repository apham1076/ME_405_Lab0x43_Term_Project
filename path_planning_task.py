# path_planning_task.py
#
# ==============================================================================
# PathPlanningTask
# ------------------------------------------------------------------------------
# This task is responsible for planning the path of the robot
# ==============================================================================

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
    def __init__(self, planning, bias, total_s_sh, kp, ki, k_line, lf_target, control_mode, abort, mtr_enable):

        # Flags
        self.planning = planning
        self.abort = abort
        self.mtr_enable = mtr_enable

        # Shares
        self.total_s_sh = total_s_sh
        
        # Parameters
        self.bias = bias
        self.kp = kp
        self.ki = ki
        self.k_line = k_line
        self.lf_target = lf_target
        self.control_mode = control_mode
        
        self.threshold = [600, 900, 1200]  # Example thresholds for total displacement
        self.first = 1
        self.checkpoint = 0

        self.state = self.S0_INIT

    def run(self):
        """Main method for the PathPlanningTask FSM."""
        state = self.S0_INIT

        while True:
            if state == self.S0_INIT:
                # Initialization code here
                if self.planning.get():
                    print("Starting run...")
                    self.abort.put(0) # clear abort flag
                    self.state = self.S1_SEG1
                
            elif state == self.S1_SEG1:
                if self.first:
                    print("Path Planning: Segment 1 started.")
                    self.mtr_enable.put(1) # ensure motors are enabled
                    # Set line-following gains
                    self.kp.put(8.0)
                    self.ki.put(0.0)
                    self.k_line.put(4.0)
                    self.lf_target.put(6)
                    self.control_mode.put(2)  # Line-following mode
                    self.first = 0

                if self.threshold[0] < self.total_s_sh.get() < self.threshold[1]:
                    if self.checkpoint < 1:
                        print("Checkpoint 1 reached.")
                        self.checkpoint += 1
                    # Turn right at checkpoint
                    self.bias.put(-3.0)                
                else:
                    # Continue line-following
                    self.bias.put(0.0)
                    self.first = 1
                    state = self.S2_SEG2
            
            elif state == self.S2_SEG2:
                if self.first:
                    print("Path Planning: Segment 2 started.")
                    # Turn off line-following, pass through diamond
                    self.kp.put(6.0)
                    self.ki.put(0.0)
                    self.control_mode.put(1)  # Velocity mode
                    self.first = 0

                if self.threshold[1] < self.total_s_sh.get() < self.threshold[2]:
                    if self.checkpoint < 2:
                        print("Checkpoint 2 reached.")
                        self.checkpoint += 1
                else:
                    # Turn line-following back on
                    self.control_mode.put(2)  # Line-following mode
                    self.first = 1
                    state = self.S3_SEG3

            elif state == self.S3_SEG3:
                pass

            yield self.state # Yield control to allow other tasks to run
