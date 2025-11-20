# state_estimation_task.py
#
# ==============================================================================
# StateEstimationTask
# ------------------------------------------------------------------------------
# This task predicts the future state of the system using a system model and sensor inputs
# ==============================================================================

from ulab import numpy as np

class StateEstimationTask:

    # The states of the FSM
    S0_INIT = 0
    S1_WAIT_FOR_ENABLE = 1
    S2_RUN = 2

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh, psi_sh, psi_dot_sh, left_eff_sh, right_eff_sh, battery, imu):

        # Shares
        self.left_pos_sh = left_pos_sh
        self.right_pos_sh = right_pos_sh
        self.left_vel_sh = left_vel_sh
        self.right_vel_sh = right_vel_sh
        self.psi_sh = psi_sh
        self.psi_dot_sh = psi_dot_sh
        self.left_eff_sh = left_eff_sh
        self.right_eff_sh = right_eff_sh
        self.battery = battery
        self.imu = imu

        # Queues

        # Flags

        # Controllers

        self.radius = 0.035  # wheel radius (m)
        self.wheelbase = 0.141  # distance between wheels (m)

        self.state = self.S0_INIT # ensure FSM starts in state S0_INIT

    # --------------------------------------------------------------------------
    ### HELPER FUNCTIONS
    # --------------------------------------------------------------------------

    def update_observer_eqn(x_k, u_star):
        '''!@brief      Implements the state equations for the open loop system
            @param t    The value of time for a given simulation step
            @param x    The value of the state vector for a given simulation step
            @return     A tuple containing both the derivative of the state vector
                        and the output vector for a given simulation step
        '''

        # Placeholder parameters (to be replaced with actual system parameters)
        r = 0.03       # wheel radius (m)
        w = 0.15       # wheelbase (m)
        K = 3.4      # motor gain (rad/s per V)
        tau = 0.05    # motor time constant (s)
        
        A_D = np.array([[0], [0], [0.1331], [0],
                      [0], [0], [0.1331], [0],
                      [0], [0], [0], [0],
                      [0], [0], [0], [0]])

        B_D = np.array([[0.0406], [0.0373], [-0.0666], [-0.0666], [0], [-2.0123],
                      [0.0373], [0.0406], [-0.0666], [-0.0666], [0], [2.0123],
                      [0], [0], [0.5], [0.5], [0], [0],
                      [0], [0], [-0.0698], [0.0698], [0.9902], [0.0001]])
        
        C = np.array([[0], [0], [1], [-w/2],
                      [0], [0], [1], [w/2],
                      [0], [0], [0], [1],
                      [-r/w], [r/w], [0], [0]])
        
        # D = np.array([[0], [0],
        #               [0], [0],
        #               [0], [0],
        #               [0], [0]])

        x_kplus1 = np.dot(A_D, x_k) + np.dot(B_D, u_star)
    
        y = np.dot(C, x_k)
        
        return x_kplus1, y
    
    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        while True: # run infinite iterations of the FSM
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):
                self.V_nom = self.battery.read_voltage()

                self.state = self.S1_WAIT_FOR_ENABLE # set next state

            ### 1: WAITING STATE -----------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_ENABLE):


                self.state = self.S2_RUN # set next state
            
            ### 2: RUN STATE ---------------------------------------------------
            elif (self.state == self.S2_RUN):

                s_L = self.left_pos_sh.get() * self.radius # Left wheel displacement (m)
                s_R = self.right_pos_sh.get() * self.radius # Right wheel displacement (m)
                s = (s_L + s_R) / 2.0                       # Romi center displacement (m)
                omega_L = self.left_vel_sh.get()            # Left wheel angular velocity (rad/s)
                omega_R = self.right_vel_sh.get()           # Right wheel angular velocity (rad/s)
                psi = self.psi_sh.get()                     # Yaw angle (deg)
                psi_dot = self.psi_dot_sh.get()             # Yaw rate (deg/s)

                psi = self.imu.read_euler_angles()[0] * (3.14159 / 180.0)  # in rad
                psi_dot = self.imu.read_angular_velocity()[2] * (3.14159 / 180.0)  # in rad/s


                V_L = self.left_eff_sh.get() * self.V_nom / 100.0
                V_R = self.right_eff_sh.get() * self.V_nom / 100.0

                x_k = np.array([[omega_L],
                                     [omega_R], 
                                     [s],
                                     [psi]])
                
                u_star = np.array([[V_L],
                                   [V_R],
                                   [s_L],
                                   [s_R],
                                   [psi],
                                   [psi_dot]])
                
                x_kplus1, y = self.update_observer_eqn(x_k, u_star)

                # Update values

                self.state = self.S2_RUN # remain in this state
            
            yield self.state