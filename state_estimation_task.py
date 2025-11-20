# state_estimation_task.py
#
# ==============================================================================
# StateEstimationTask
# ------------------------------------------------------------------------------
# This task predicts the future state of the system using a system model and sensor inputs
# ==============================================================================

from pyb import millis
from ulab import numpy as np

class StateEstimationTask:
    """Estimates the state of the robot using sensor inputs and a system model."""

    # The states of the FSM
    S0_INIT = 0
    S1_ESTIMATING = 1

    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self, start_time, obsv_time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh,
                 psi_sh, psi_dot_sh, left_eff_sh, right_eff_sh,
                 battery, imu,
                 obsv_sL_sh, obsv_sR_sh, obsv_psi_sh, obsv_psi_dot_sh,
                 obsv_left_vel_sh, obsv_right_vel_sh, obsv_s_sh, obsv_s, obsv_yaw_sh):

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

        self.r = 0.035  # wheel radius (m)
        self.w = 0.141  # distance between wheels (m)

        self.V_nom = self.battery.read_voltage()

        # Set initial yaw angle from IMU

        self.A_D = np.array([[0], [0], [0.1331], [0],
                             [0], [0], [0.1331], [0],
                             [0], [0], [0], [0],
                             [0], [0], [0], [0]])

        self.B_D = np.array([[0.0406], [0.0373], [-0.0666], [-0.0666], [0], [-2.0123],
                             [0.0373], [0.0406], [-0.0666], [-0.0666], [0], [2.0123],
                             [0], [0], [0.5], [0.5], [0], [0],
                             [0], [0], [-0.0698], [0.0698], [0.9902], [0.0001]])
        
        self.C = np.array([[0], [0], [1], [-self.w/2],
                           [0], [0], [1], [self.w/2],
                           [0], [0], [0], [1],
                           [-self.r/self.w], [self.r/self.w], [0], [0]])
        
        self.x_k = np.array([[0],
                             [0], 
                             [0],
                             [0]])
        
        self.y = np.array([[0],
                           [0],
                           [0],
                           [0]])
        
        self.state = self.S0_INIT # ensure FSM starts in state S0_INIT
    
    def run(self):
        while True:
            if (self.state == self.S0_INIT):
                # Set initial yaw angle
                s_L = self.left_pos_sh.get() * self.r  # initial left wheel displacement (m)
                s_R = self.right_pos_sh.get() * self.r  # initial right wheel displacement (m)
                psi = (s_R - s_L) / self.w  # initial yaw angle from wheel odometry (rad)
                psi_meas = self.imu.read_euler_angles()[0] * (3.14159 / 180.0)  # initial yaw angle in rad
                self.psi_offset = psi_meas - psi

                # Get initial time
                self.t0 = self.start_time.get()

                self.state = self.S1_ESTIMATING # set next state

            elif (self.state == self.S1_ESTIMATING):

                # Determine values for output state vector, y
                s_L = self.left_pos_sh.get() * self.r # Left wheel displacement (m)
                s_R = self.right_pos_sh.get() * self.r # Right wheel displacement (m)
                psi = self.imu.read_euler_angles()[0] * (3.14159 / 180.0)  # yaw angle in rad
                psi -= self.psi_offset  # Subtract offset
                psi_dot = self.imu.read_angular_velocity()[2] * (3.14159 / 180.0)  # yaw rate in rad/s

                # Store away yaw angle and yaw rate from IMU
                self.psi_sh.put(psi)
                self.psi_dot_sh.put(psi_dot)

                # Determine input vector, u
                V_L = self.left_eff_sh.get() * self.V_nom / 100.0
                V_R = self.right_eff_sh.get() * self.V_nom / 100.0
                
                # Form u_star
                u_star = np.array([[V_L],
                                    [V_R],
                                    [s_L],
                                    [s_R],
                                    [psi],
                                    [psi_dot]])
                
                # Predict next state
                self.x_kplus1 = np.dot(self.A_D, self.x_k) + np.dot(self.B_D, u_star)
                # Determine present output
                self.y_k = np.dot(self.C, self.x_k)

                # Put observer values in shares
                self.obsv_sL_sh.put(float(self.y_k[0]))
                self.obsv_sR_sh.put(float(self.y_k[1]))
                self.obsv_psi_sh.put(float(self.y_k[2]))
                self.obsv_psi_dot_sh.put(float(self.y_k[3]))

                self.obsv_left_vel_sh.put(float(self.x_kplus1[0]))
                self.obsv_right_vel_sh.put(float(self.x_kplus1[1]))
                self.obsv_s_sh.put(float(self.x_kplus1[2]))
                self.obsv_yaw_sh.put(float(self.x_kplus1[3]))

                t = millis() - self.t0
                self.obsv_time_sh.put(t)

                # Update state for next iteration
                self.x_k = self.x_kplus1
                
                yield self.state