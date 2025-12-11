# ui_task.py
#
# ==============================================================================
# UITask
# ------------------------------------------------------------------------------
# Reads user input from Bluetooth UART (PC test.py), interprets commands, and
# sets control flags and shares accordingly.
# Allows the user to:
#   - 
#   -
#   - 
# ==============================================================================

from time import ticks_ms, ticks_diff

class UITask:
    """Reads user input from Bluetooth UART, interprets commands, and sets control flags."""

    # The states of the FSM
    S0_INIT = 0
    S1_WAIT_FOR_CMD = 1
    S2_PROCESS_CMD = 2
    S3_MONITOR_TEST = 3
    S4_IMU_CALIBRATION = 4
    S5_IR_CALIBRATION = 5

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self,
                 mtr_enable, stream_data, abort,
                 eff, driving_mode, setpoint, kp, ki, control_mode,
                 uart, battery, imu, ir_array,
                 k_line, lf_target, planning, game_origin_mode):
        
        # Flags
        self.mtr_enable = mtr_enable
        self.stream_data = stream_data
        self.abort = abort
        self.planning = planning
        self.game_origin_mode = game_origin_mode
        
        # Shares
        self.eff = eff
        self.driving_mode = driving_mode # 0: straight line, 1: pivot, 2: arc
        self.setpoint = setpoint  # velocity setpoint
        self.kp = kp  # proportional gain
        self.ki = ki  # integral gain
        self.k_line = k_line  # line following K_line gain
        self.lf_target = lf_target  # line following target velocity
        self.control_mode = control_mode  # 0: effort, 1: velocity, 2: line following

        # Hardware
        self.ser = uart # Serial UART object
        self.battery = battery # Battery object
        self.imu = imu # IMU object
        self.ir = ir_array # IR sensor array object

        # Initial values
        self.state = self.S0_INIT
        self.cmd_buf = ''  # buffer for incoming command
        self.prev_time = 0 # previous time for timing

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        """Generator that checks PuTTY for commands and updates flags"""
        while True: # run infinite iterations of the FSM
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):
                self.stream_data.put(1)  # DEFAULT: streaming ON
                self.driving_mode.put(0) # DEFAULT: straight line mode
                self.planning.put(0)     # DEFAULT: path planning OFF
                self.abort.put(0)
                self.prev_time = ticks_ms()
                self.state = self.S1_WAIT_FOR_CMD

            ### 1: WAITING STATE -----------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_CMD):
                # Wait for user input (read available bytes non-blocking)
                if self.ser.any():
                    try:
                        self.cmd_buf = self.ser.read(1).decode()
                        self.state = self.S2_PROCESS_CMD # set next state
                    except Exception:
                        pass # Handle decoding errors gracefully
            
            ### 2: PROCESS COMMAND STATE ---------------------------------------
            elif self.state == self.S2_PROCESS_CMD:

                cmd = self.cmd_buf

                # 'e': EFFORT
                # 'e' + 2 digits (00 to 99) or 'e' + 'a' for 100%
                if cmd == 'e' and self.ser.any() >= 1:
                    val_str = self.ser.read(1).decode()
                    if val_str.isdigit() or val_str == 'a':
                        val = 10 * (int(val_str) if val_str.isdigit() else 10)  # '00'-'99' → 0–99%, 'a'→100%
                        self.eff.put(val)
                        self.control_mode.put(0)  # Effort mode
                        print(f"Effort set: {val}%")
                    else:
                        print("Invalid effort command format")

                # 'v': VELOCITY
                # 'v' + 4 digits for setpoint + 4 digits for Kp + 4 digits for Ki
                elif cmd == 'v' and self.ser.any() >= 12:
                    try:
                        val_str = self.ser.read(12).decode()
                        sp = int(val_str[0:4]) / 100.0
                        kp = int(val_str[4:8]) / 100.0
                        ki = int(val_str[8:12]) / 100.0
                        self.setpoint.put(sp)
                        self.kp.put(kp)
                        self.ki.put(ki)
                        self.control_mode.put(1)  # Velocity mode
                        print(f"Received velocity params: SP={sp}, Kp={kp}, Ki={ki}")
                    except ValueError:
                        print("Invalid velocity command format")

                # 'l': LINE-FOLLOWING
                # 'l' + 4 digits for Kp + 4 digits for Ki + 4 digits for K_line + 4 digits for target velocity
                elif cmd == 'l' and self.ser.any() >= 16:
                    try:
                        val_str = self.ser.read(16).decode()
                        kp = int(val_str[0:4]) / 100.0
                        ki = int(val_str[4:8]) / 100.0
                        k_line = int(val_str[8:12]) / 100.0
                        v_target = int(val_str[12:16]) / 100.0
                        self.kp.put(kp)
                        self.ki.put(ki)
                        self.k_line.put(k_line)
                        self.lf_target.put(v_target)
                        self.control_mode.put(2)  # Line-following mode
                        print(f"Received LF params: Kp={kp}, Ki={ki}, K_line={k_line}, Target Velocity={v_target}")
                    except ValueError:
                        print("Invalid line-following command format")

                # 'r': RUN
                elif cmd == 'r':
                    if not self.mtr_enable.get():
                        print("Starting run...")
                        self.abort.put(0) # clear abort flag
                        self.mtr_enable.put(1) # set motor enable flag
                        self.ser.write(b'q') # Tell PC that run started
                        self.state = self.S3_MONITOR_TEST # go to monitor state

                # 'k': KILL (STOP)
                elif cmd == 'k':
                    print("Kill command.")
                    self.abort.put(1)  # Set abort flag first
                    self.mtr_enable.put(0)  # Then disable motors

                # 's': STREAMING TOGGLE
                elif cmd == 's':
                    if self.stream_data.get():
                        print("Stopping data stream.")
                        self.stream_data.put(0) # Turn OFF streaming
                    else:
                        print("Starting data stream.")
                        self.stream_data.put(1) # Turn ON streaming

                # 'n': TOGGLE DRIVING MODE (0->1->2->0)
                # 0: straight line, 1: pivot, 2: arc
                elif cmd == 'n':
                    if not self.mtr_enable.get(): # only change when stopped
                        current_mode = self.driving_mode.get() or 0
                        if current_mode == 0:
                            new_mode = 1
                            print("Driving mode set to: Pivot")
                        elif current_mode == 1:
                            new_mode = 2
                            print("Driving mode set to: Arc")
                        else:
                            new_mode = 0
                            print("Driving mode set to: Straight")
                        self.driving_mode.put(new_mode)
                
                # 'z': TOGGLE PATH PLANNING MODE
                elif cmd == 'z':   # Toggle path planning mode
                    if self.planning.get():
                        self.planning.put(0)
                        self.game_origin_mode.put(0)  # Clear game origin mode when path planning is disabled
                        print("Path planning mode DISABLED.")
                    else:
                        self.planning.put(1)
                        self.game_origin_mode.put(1)  # Set game origin mode when path planning is enabled
                        print("Path planning mode ENABLED.")
                
                # 'v': BATTERY VOLTAGE
                elif cmd == 'V':
                    voltage_msg = f"{self.battery.read_voltage():.2f}\n"
                    self.ser.write(voltage_msg.encode()) # send over Bluetooth
                
                # 'i': IMU CALIBRATION
                elif cmd == 'i':
                    print("IMU calibration command received.")
                    self.state = self.S4_IMU_CALIBRATION
                
                # IR CALIBRATION
                elif cmd == 'f':
                    print("IR sensor calibration command received.")
                    self.state = self.S5_IR_CALIBRATION

                else:
                    pass

                if self.state == self.S2_PROCESS_CMD: # if state wasn't changed
                    self.state = self.S1_WAIT_FOR_CMD # set back to waiting

            ### 3: MONITOR TEST STATE ------------------------------------------
            elif self.state == self.S3_MONITOR_TEST:
                # Check for Kill command during run and handle it
                if self.ser.any():
                    try:
                        key = self.ser.read(1).decode().lower()
                        if 'k' in key:
                            print("Stopping test due to kill.")
                            self.abort.put(1) # Set abort flag first
                            self.mtr_enable.put(0) # Then disable motors
                            self.ser.write(b'q')  # Tell PC test is done
                            self.state = self.S1_WAIT_FOR_CMD # set back to waiting
                        if 's' in key:
                            if self.stream_data.get():
                                print("Stopping data stream during test.")
                                self.stream_data.put(0) # Turn OFF streaming
                            else:
                                print("Starting data stream during test.")
                                self.stream_data.put(1) # Turn ON streaming
                    except Exception:
                        pass # Handle decoding errors gracefully

                # If no kill command, check for normal test completion
                if not self.mtr_enable.get():
                    # Motors have already been disabled
                    print("Test complete.")
                    self.abort.put(1) # Set abort flag??
                    self.state = self.S1_WAIT_FOR_CMD # return to waiting

            ### 4: IMU Calibration State ---------------------------------------
            elif self.state == self.S4_IMU_CALIBRATION:
                self.imu.set_operation_mode("ndof") # set to NDOF mode
                self.curr_time = ticks_ms()
                if ticks_diff(self.curr_time, self.prev_time) > 2000:
                    self.prev_time = self.curr_time
                    imu_status_bytes = self.imu.read_calibration_status()
                    print("IMU Calibration Status (sys, gyr, acc, mag):", imu_status_bytes)
                    # stay in this state until 'j' is received
                if self.ser.any():
                    if self.ser.read(1).decode() == 'j':
                        self.imu.read_calibration_coeffs()
                        print("IMU calibration done.")
                        self.state = self.S1_WAIT_FOR_CMD # return to waiting
            
            ### 5: IR Calibration State ----------------------------------------
            elif self.state == self.S5_IR_CALIBRATION:
                if self.ser.any():
                    cmd = self.ser.read(1).decode()
                    # 'w': WHITE CALIBRATION
                    if cmd == 'w':   # Calibrate on WHITE background
                        print("Calibrating white background...")
                        self.ir.calibrate('w')
                        # stay in this state until after 'b' calibration
                    # 'b': BLACK CALIBRATION
                    elif cmd == 'b':   # Calibrate on BLACK background
                        print("Calibrating black background...")
                        self.ir.calibrate('b')
                        print("IR calibration done.")
                        self.state = self.S1_WAIT_FOR_CMD # return to waiting

            yield self.state