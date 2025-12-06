# state_estimation_task.py
#
# ==============================================================================
# StateEstimationTask
# ------------------------------------------------------------------------------
# This task predicts the future state of the system using a system model and sensor inputs
# ==============================================================================

from pyb import millis
from ulab import numpy as np
from math import pi

class StateEstimationTask:
    """Estimates the state of the robot using sensor inputs and a system model."""

    # The states of the FSM
    S0_INIT = 0
    S1_ESTIMATING = 1

    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self,
                 run_observer,
                 start_time, obsv_data_ready,
                 obsv_time_sh, left_pos_sh, right_pos_sh, 
                 left_vel_sh, right_vel_sh,
                 psi_sh, psi_dot_sh, left_eff_sh, right_eff_sh,
                 battery,
                 obsv_sL_sh, obsv_sR_sh, obsv_psi_sh, obsv_psi_dot_sh,
                 obsv_left_vel_sh, obsv_right_vel_sh, obsv_s_sh, obsv_yaw_sh):

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

        self.start_time = start_time
        self.obsv_time_sh = obsv_time_sh
        self.obsv_sL_sh = obsv_sL_sh
        self.obsv_sR_sh = obsv_sR_sh
        self.obsv_psi_sh = obsv_psi_sh
        self.obsv_psi_dot_sh = obsv_psi_dot_sh
        self.obsv_left_vel_sh = obsv_left_vel_sh
        self.obsv_right_vel_sh = obsv_right_vel_sh
        self.obsv_s_sh = obsv_s_sh
        self.obsv_yaw_sh = obsv_yaw_sh

        # Flags
        self.run_observer = run_observer
        self.obsv_data_ready = obsv_data_ready

        self.r = 0.035  # wheel radius (m)
        self.w = 0.141  # distance between wheels (m)
        self.count2rad = 2*pi/1440   # Conversion factor

        self.V_nom = self.battery.read_voltage()

        self.A_D = np.array([[0, 0, 0.1331*100, 0],
                             [0, 0, 0.1331, 0],
                             [0, 0, 0, 0],
                             [0, 0, 0, 0]])

        self.B_D = np.array([[0.0406, 0.0373, -0.0666*100, -0.0666*100, 0, -2.0123],
                             [0.0373, 0.0406, -0.0666, -0.0666, 0, 2.0123],
                             [0, 0, 0.5, 0.5, 0, 0],
                             [0, 0, -0.0698, 0.0698, 0.9902, 0.0001]])
        
        self.C = np.array([[0, 0, 1, -self.w/2],
                           [0, 0, 1, self.w/2],
                           [0, 0, 0, 1],
                           [-self.r/self.w, self.r/self.w, 0, 0]])
        
        self.x_k = np.array([[0],
                             [0], 
                             [0],
                             [0]])
        
        self.x_kplus1 = np.array([[0],
                                 [0],
                                 [0],
                                 [0]])
        
        self.y_k = np.array([[0],
                           [0],
                           [0],
                           [0]])

        self.state = self.S0_INIT # ensure FSM starts in state S0_INIT
    
    def run(self):
        while True: # run infinite iterations of the FSM
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):
                # Wait for signal from motor task to start observing
                run_observer = self.run_observer.get()
                if run_observer:
                    motor_task_start_time = self.start_time.get()
                    self.state = self.S1_ESTIMATING # set next state
                else:
                    yield self.state
                    continue

            ### 1: ESTIMATING STATE --------------------------------------------
            elif (self.state == self.S1_ESTIMATING):
                # Determine input vector, u
                V_L = self.left_eff_sh.get() * self.V_nom / 100.0
                V_R = self.right_eff_sh.get() * self.V_nom / 100.0

                # Determine values for output state vector, y
                s_L = self.left_pos_sh.get() * self.count2rad * self.r # Left wheel displacement (m)
                s_R = self.right_pos_sh.get() * self.count2rad * self.r # Right wheel displacement (m)
                psi = self.psi_sh.get() / 1000
                psi_dot = self.psi_dot_sh.get() / 1000

                # print("State Estimation Task: V_L =", V_L, "V_R =", V_R, "s_L =", s_L, "s_R =", s_R, "psi =", psi, "psi_dot =", psi_dot)
                # Form u_star
                u_star = np.array([[V_L],
                                    [V_R],
                                    [s_L],
                                    [s_R],
                                    [psi],
                                    [psi_dot]])
                
                # Predict next state
                self.x_kplus1 = np.dot(self.A_D, self.x_k) + np.dot(self.B_D, u_star)
                # Determine output for current state
                self.y_k = np.dot(self.C, self.x_k)

                # Put time in share
                t = millis() - motor_task_start_time
                self.obsv_time_sh.put(int(t))
                
                # Scale values
                # obsv_sL = self.y_k[0,0] * 1e3       # Has units m / 1000
                # obsv_sR = self.y_k[1,0] * 1e3       # Has units m / 1000
                # obsv_psi = self.y_k[2,0] * 1e6          # Has units rad / 1e6
                # obsv_psi_dot = self.y_k[3,0] * 1e6      # Has units rad/s / 1e6

                # # Put observer values in shares
                # self.obsv_sL_sh.put(int(obsv_sL))
                # self.obsv_sR_sh.put(int(obsv_sR))
                # self.obsv_psi_sh.put(int(obsv_psi))
                # self.obsv_psi_dot_sh.put(int(obsv_psi_dot))

                # Get estimated state
                obsv_left_vel = self.x_kplus1[0,0] * 1e3    # Has units rad/s / 1e3
                obsv_right_vel = self.x_kplus1[1,0] * 1e3   # Has units rad/s / 1e3
                obsv_s = self.x_kplus1[2,0] * 1e3           # Has units m / 1e3
                obsv_yaw = self.x_kplus1[3,0] * 1e3         # Has units rad / 1e3

                self.obsv_left_vel_sh.put(int(obsv_left_vel))
                self.obsv_right_vel_sh.put(int(obsv_right_vel))
                self.obsv_s_sh.put(int(obsv_s))
                self.obsv_yaw_sh.put(int(obsv_yaw))
                
                # Set flag for data task
                self.obsv_data_ready.put(1)

                # Update state for next iteration
                self.x_k = self.x_kplus1
                
            yield self.state