# steering_task.py
#
# ==============================================================================
# SteeringTask
# ------------------------------------------------------------------------------
# Write in a description...
# ==============================================================================

### TO-DO:

class SteeringTask:
    """Reads from IR sensors to get centroid and does Closed-loop control to have Romi follow a line"""

    ### The states of the FSM
    S0_INIT = 0
    S1_WAIT_FOR_START_COLLECTING = 1
    S2_COLLECTING_DATA = 2
    S3_COLLECTION_DONE = 3

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self, attr1, attr2, attr3):

        # Flags
        

        # Queues
        

        # Shares
        
        # ensure FSM starts in state S0_INIT
        self.state = self.S0_INIT

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        while True: # run infinite iterations of the FSM
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):

                # Clear queues
                

                self.state = self.S1_WAIT_FOR_START_COLLECTING # set next state

            ### 1: WAITING STATE -----------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_START_COLLECTING):
                
                self.state = self.S2_COLLECTING_DATA # set next state

            ### 2: COLLECTING STATE --------------------------------------------
            elif (self.state == self.S2_COLLECTING_DATA):
                # print("Collecting data...")
                
                
                self.state = self.S3_COLLECTION_DONE
            
            ### 3: DONE STATE --------------------------------------------------
            elif (self.state == self.S3_COLLECTION_DONE):
                # Wait for col_start to be cleared
                
                
                self.state = self.S1_WAIT_FOR_START_COLLECTING

            yield self.state