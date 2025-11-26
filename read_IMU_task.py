
class ReadIMUTask:
    """Handles reading from IMU sensor and updating shares so state_estimation_task can run faster"""
    S0_INIT = 0
    S1_IDLE = 1
    S2_READING = 2

    def __init__(self, imu, left_pos_sh, right_pos_sh, psi_sh, psi_dot_sh, read_IMU_flg):
        

        self.imu = imu
        self.left_pos_sh = left_pos_sh
        self.right_pos_sh = right_pos_sh
        self.psi_sh = psi_sh
        self.psi_dot_sh = psi_dot_sh
        self.read_IMU_flg = read_IMU_flg
        self.r = 0.035  # wheel radius (m)
        self.w = 0.141  # distance between wheels (m)

        self.state = self.S0_INIT

    def run(self):
        while True:
            if (self.state == self.S0_INIT):
                # Set initial yaw angle
                s_L = self.left_pos_sh.get() * self.r  # initial left wheel displacement (m)
                s_R = self.right_pos_sh.get() * self.r  # initial right wheel displacement (m)
                psi = (s_R - s_L) / self.w  # initial yaw angle from wheel odometry (rad)
                self.psi_offset = self.imu.read_euler_angles()[0] * (3.14159 / 180.0) - psi # find the yaw offset
                
                self.state = self.S1_IDLE
            
            elif (self.state == self.S1_IDLE):
                # Wait for flag to begin reading
                # if self.read_IMU_flg.get():
                #     self.state = self.S2_READING

                self.state = self.S2_READING

            elif (self.state == self.S2_READING):
                # if not self.read_IMU_flg.get():
                #     self.state = self.S1_IDLE
                #     continue

                psi = self.imu.read_euler_angles()[0] * (3.14159 / 180.0)  # yaw angle in rad
                psi -= self.psi_offset  # Subtract offset
                psi *= 1000  # Scale up psi and store as an integer
                psi_dot = self.imu.read_angular_velocity()[2] * (3.14159 / 180.0)  # yaw rate in rad/s
                psi_dot *= 1000  # Scale up psi_dot and store as an integer

                self.psi_sh.put(int(psi))
                self.psi_dot_sh.put(int(psi_dot))

            yield self.state


