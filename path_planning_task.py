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

    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self, param1, param2, param3, param4):

        # Shares
        
        # Parameters
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
        self.param4 = param4

        self.state = self.S0_INIT

    def run(self):
        """Main method for the TemplateTask FSM."""
        state = self.S0_INIT

        while True:
            if state == self.S0_INIT:
                # Initialization code here
                state = self.S1_SEG1

            elif state == self.S1_SEG1:
                # Running code here
                state = self.S2_SEG2
            
            elif state == self.S2_SEG2:
                # Running code here
                state = self.S3_SEG3

            yield self.state # Yield control to allow other tasks to run
