# template.py
#
# ==============================================================================
# TasknameTask
# ------------------------------------------------------------------------------
# This is a template for creating new task classes.
# ==============================================================================

class TasknameTask:
    """Description of what the task does."""

    # Examples states of the FSM
    S0_INIT = 0
    S1_WAITING = 1
    S2_RUNNING = 2

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
                state = self.S1_WAITING

            elif state == self.S1_WAITING:
                # Waiting code here
                state = self.S2_RUNNING

            elif state == self.S2_RUNNING:
                # Running code here
                state = self.S1_WAITING

            yield self.state # Yield control to allow other tasks to run
