# motor_task.py
#
# ==============================================================================
# MotorControlTask
# ------------------------------------------------------------------------------
# This task periodically reads both encoders, computes position and velocity,
# updates time/pos/vel shares, and applies the latest motor efforts.
# ==============================================================================

# To-do: Determine limits for Yaw controller, need to define midpoint location,
#       need to define nominal translational velocity, define Romi parameters like trackwdith and wheel radius,
#       double check yaw rate to velocity calculations

from pyb import millis
from closed_loop import ClosedLoop

class MotorControlTask:
    """Handles reading from encoders and actuating motors, updates encoder position and velocity"""

    # The states of the FSM
    S0_INIT = 0
    S1_WAIT_FOR_EFF = 1
    S2_RUN = 2

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self,
                 left_motor, right_motor,
                 left_encoder, right_encoder,
                 IR_arr,
                 eff, mtr_enable, abort, mode, setpoint, kp, ki, control_mode,
                 time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh,
                 kpy, kiy):

        # Hardware
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.left_encoder = left_encoder
        self.right_encoder = right_encoder
        self.IR_arr = IR_arr

        # Shares
        self.eff = eff
        self.mode = mode
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.kpy = kpy
        self.kiy = kiy
        self.control_mode = control_mode
        
        # Queues
        self.time_sh = time_sh
        self.left_pos_sh = left_pos_sh
        self.right_pos_sh = right_pos_sh
        self.left_vel_sh = left_vel_sh
        self.right_vel_sh = right_vel_sh

        # Flags
        self.mtr_enable = mtr_enable
        self.abort = abort

        # Controllers
        self.left_controller = ClosedLoop(effort_limits=(-100, 100))
        self.right_controller = ClosedLoop(effort_limits=(-100, 100))
        self.yaw_controller = ClosedLoop(yaw_limits=(-1, 1))
        
        self.t0 = 0 # zero the start time offset
        self.state = self.S0_INIT # ensure FSM starts in state S0_INIT

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        while True: # run infinite iterations of the FSM
            ### INIT STATE -----------------------------------------------------
            if (self.state == self.S0_INIT):
                # Zero encoders, disable motors, and initialize variables
                self.left_encoder.zero()
                self.right_encoder.zero()
                self.left_motor.disable()
                self.right_motor.disable()

                # Clear command and shares
                self.eff.put(0)
                self.mtr_enable.put(0)

                # Reset controllers
                self.left_controller.reset()
                self.right_controller.reset()
                self.yaw_controller.reset()

                # Set gains
                self.left_controller.set_gains(self.kp.get(), self.ki.get())
                self.right_controller.set_gains(self.kp.get(), self.ki.get())
                self.yaw_controller.set_gains(self.kpy.get(), self.kiy.get())

                self.state = self.S1_WAIT_FOR_EFF # set next state

            ### WAITING STATE --------------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_EFF):
                if self.mtr_enable.get():
                    self.left_encoder.zero()
                    self.right_encoder.zero()
                    self.t0 = millis() # a timestamp to zero the time right when the motors are enabled
                    self.left_motor.enable()
                    self.right_motor.enable()
                    
                    self.state = self.S2_RUN # set next state
            
            ### RUN STATE ------------------------------------------------------
            elif (self.state == self.S2_RUN):

                # Update encoders
                self.left_encoder.update()
                self.right_encoder.update()

                # Get current velocities and positions
                pL = self.left_encoder.get_position()
                pR = self.right_encoder.get_position()
                vL = self.left_encoder.get_velocity()
                vR = self.right_encoder.get_velocity()

                # Run steering controller
                self._update_setpoint()

                # Update setpoints
                self.left_controller.set_setpoint(self.left_vel_setpoint)
                self.right_controller.set_setpoint(self.right_vel_setpoint)
                
                # Calculate control efforts
                left_effort = self.left_controller.run(vL)
                right_effort = self.right_controller.run(vR)
                
                # print(f"Setpoint: {setpoint}, Left Vel: {vL}, Right Vel: {vR}, Left Effort: {left_effort}, Right Effort: {right_effort}")

                # Apply efforts
                self.left_motor.set_effort(left_effort)
                self.right_motor.set_effort(right_effort)

                # Calculate the exact timestamp of the measurement
                t = millis() - self.t0

                # Write data samples to shares (for other tasks using them)
                # Ensure values being put are integers
                self.time_sh.put(int(t))
                self.left_pos_sh.put(float(pL))
                self.right_pos_sh.put(float(pR))
                self.left_vel_sh.put(float(vL))
                self.right_vel_sh.put(float(vR))
                
                # print("Motor Task Shares:")
                # print(self.time_sh.get())
                # print(self.left_pos_sh.get())
                # print(self.right_pos_sh.get())
                # print(self.left_vel_sh.get())
                # print(self.right_vel_sh.get())

                yield self.state

                # Disable if mtr_enable flag is cleared or abort is triggered
                if not self.mtr_enable.get() or self.abort.get():
                    self.left_motor.disable()
                    self.right_motor.disable()
                    self.abort.put(0)  # Reset abort flag after handling it
                    # Reset controllers
                    self.left_controller.reset()
                    self.right_controller.reset()
                    self.state = self.S1_WAIT_FOR_EFF
            
            yield self.state

# --------------------------------------------------------------------------
### Helper functions
# --------------------------------------------------------------------------
    def _update_setpoint(self):
        # Reads from sensors and get centroid
        xc = self.IR_arr.get_centroid()

        # Use centroid to get a yaw rate
        yaw_rate = self.yaw_controller.run(xc)

        # Update velocitiy setpoint values
        # yaw > 0 is turn left, yaw < 0 is turn right
        self.left_vel_setpoint -= yaw_rate * self.trackwidth / 2
        self.right_vel_setpoint += yaw_rate * self.trackwidth / 2