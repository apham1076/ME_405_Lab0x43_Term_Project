# motor_task.py
#
# ==============================================================================
# MotorControlTask
# ------------------------------------------------------------------------------
# This task periodically reads both encoders, computes position and velocity,
# updates time/pos/vel shares, and applies the latest motor efforts.
# ==============================================================================

from pyb import millis
from closed_loop import ClosedLoop

class MotorControlTask:
    """Handles reading from encoders and actuating motors, updates encoder position and velocity"""

    # The states of the FSM
    S0_INIT = 0
    S1_WAIT_FOR_ENABLE = 1
    S2_RUN = 2

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self,
                 left_motor, right_motor,
                 left_encoder, right_encoder,
                 battery,
                 eff, mtr_enable, motor_data_ready, run_observer, abort, driving_mode, setpoint, kp, ki, control_mode,
                 start_time,
                 time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh,
                 left_sp_sh, right_sp_sh, left_eff_sh, right_eff_sh):

        # Hardware
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.left_encoder = left_encoder
        self.right_encoder = right_encoder
        self.battery = battery

        # Shares
        self.eff = eff
        self.driving_mode = driving_mode # 1=straight, 2=pivot, 3=arc
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.control_mode = control_mode # 0=effort, 1=velocity, 2=line follow
        self.left_sp_sh = left_sp_sh
        self.right_sp_sh = right_sp_sh
        self.left_eff_sh = left_eff_sh
        self.right_eff_sh = right_eff_sh
        self.start_time = start_time

        # Queues
        self.time_sh = time_sh
        self.left_pos_sh = left_pos_sh
        self.right_pos_sh = right_pos_sh
        self.left_vel_sh = left_vel_sh
        self.right_vel_sh = right_vel_sh

        # Flags
        self.mtr_enable = mtr_enable
        self.abort = abort
        self.motor_data_ready = motor_data_ready
        self.run_observer = run_observer

        # Controllers
        self.left_controller = ClosedLoop(self.kp, self.ki, self.left_sp_sh, self.battery, effort_limits=(-100, 100))
        self.right_controller = ClosedLoop(self.kp, self.ki, self.right_sp_sh, self.battery, effort_limits=(-100, 100))

        self.t0 = 0 # zero the start time offset
        self.state = self.S0_INIT # ensure FSM starts in state S0_INIT
    
    # --------------------------------------------------------------------------
    ### HELPER FUNCTIONS
    # --------------------------------------------------------------------------
    ### Split setpoints for left and right motors based on driving mode
    def _split_setpoints(self, driving_mode_val, setpoint):
        """
        mode 0: straight  -> (spL, spR) = ( sp,  sp)
        mode 1: pivot     -> (spL, spR) = ( sp, -sp)
        mode 2: arc       -> (spL, spR) = ( sp, sp*RATIO )
        """
        if driving_mode_val == 0:   # straight
            return float(setpoint), float(setpoint)
        elif driving_mode_val == 1: # pivot in place
            return float(setpoint), -float(setpoint)
        else:               # arc (simple fixed ratio; refine later if desired)
            RATIO = 0.6
            return float(setpoint), float(setpoint) * RATIO

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        while True: # run infinite iterations of the FSM
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):
                # Zero encoders, disable motors, and initialize variables
                self.left_encoder.zero()
                self.right_encoder.zero()
                self.left_motor.disable()
                self.right_motor.disable()

                # Clear command and shares
                self.eff.put(0)
                self.mtr_enable.put(0)
                self.run_observer.put(0)  # Disable state estimator

                # Set default modes
                self.driving_mode.put(1)
                self.control_mode.put(0)

                # Reset controllers
                self.left_controller.reset()
                self.right_controller.reset()

                self.state = self.S1_WAIT_FOR_ENABLE # set next state

            ### 1: WAITING STATE -----------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_ENABLE):
                if self.mtr_enable.get():

                    if self.battery is not None:
                        try:
                            gain = self.battery.refresh()
                            print(f"Measured battery: {self.battery._cached_voltage:.2f} V, applying droop gain: {gain:.3f}")
                        except Exception as e:
                            print(f"Battery refresh failed: {e}")

                    # Zero encoders
                    self.left_encoder.zero()
                    self.right_encoder.zero()

                    # Log a timestamp to zero the time right when the motors are enabled
                    self.t0 = millis()
                    self.start_time.put(self.t0)

                    # Enable motors
                    self.left_motor.enable()
                    self.right_motor.enable()
                    
                    # Enable state estimator on transition from WAIT to RUN
                    self.run_observer.put(1)
                    
                    self.state = self.S2_RUN # set next state
            
            ### 2: RUN STATE ---------------------------------------------------
            elif (self.state == self.S2_RUN):
                # Check for abort signal
                if self.abort.get(): # (abort is a share)
                    self.left_motor.disable()
                    self.right_motor.disable()
                    self.left_controller.reset()
                    self.right_controller.reset()
                    self.run_observer.put(0)  # Disable state estimator on transition from RUN to WAIT
                    self.abort.put(0)  # Reset abort flag after handling it
                    self.mtr_enable.put(0)  # Clear enable flag
                    self.state = self.S1_WAIT_FOR_ENABLE
                    continue

                # Update encoders
                self.left_encoder.update()
                self.right_encoder.update()

                # --- Get raw data for storing to the shares ---
                # Calculate the exact timestamp of the measurements
                t = millis() - self.t0
                # Get current positions and velocities (in raw units, counts and counts/s, for data streaming)
                left_pos = self.left_encoder.get_position("counts")
                right_pos = self.right_encoder.get_position("counts")
                left_vel = self.left_encoder.get_velocity("counts/s")
                right_vel = self.right_encoder.get_velocity("counts/s")

                # --------------------------------------------------------------
                ### Determine left and right efforts based on control mode
                # 0 = Effort; 1 = Velocity; 2 = Line Follow
                # --------------------------------------------------------------
                    # ----------------------------------------------------------
                    # MODE 0: EFFORT (open loop control)
                if self.control_mode.get() == 0:
                    left_eff, right_eff = self._split_setpoints(self.driving_mode.get(), self.eff.get())

                    # ----------------------------------------------------------
                    # MODE 1: VELOCITY (closed-loop velocity control)
                elif self.control_mode.get() == 1:
                    self.left_sp_sh.put(float(self.setpoint.get()))
                    self.right_sp_sh.put(float(self.setpoint.get()))
                    # Calculate control efforts
                    left_eff = self.left_controller.run(left_vel)
                    right_eff = self.right_controller.run(right_vel)

                    # ----------------------------------------------------------
                    # MODE 2: LINE FOLLOWING (outer + inner loop)
                elif self.control_mode.get() == 2:
                    left_eff = self.left_controller.run(left_vel)
                    right_eff = self.right_controller.run(right_vel)

                # --------------------------------------------------------------
                # Apply efforts
                self.left_motor.set_effort(float(left_eff))
                self.right_motor.set_effort(float(right_eff))
                # --------------------------------------------------------------

                # Write data samples to shares (for other tasks using them)
                self.time_sh.put(int(t))
                self.left_pos_sh.put(int(left_pos))
                self.right_pos_sh.put(int(right_pos))
                self.left_vel_sh.put(int(left_vel))
                self.right_vel_sh.put(int(right_vel))
                
                # Store efforts in shares for monitoring
                self.left_eff_sh.put(float(left_eff))
                self.right_eff_sh.put(float(right_eff))

                # Set flag for data task
                self.motor_data_ready.put(1)
            
            yield self.state