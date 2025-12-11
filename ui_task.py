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
    S1_WAIT_FOR_COMMAND = 1
    S2_PROCESS_COMMAND = 2
    S3_MONITOR_TEST = 3
    S4_CALIBRATION = 4

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self,
                 mtr_enable, stream_data, abort,
                 eff, driving_mode, setpoint, kp, ki, control_mode,
                 uart, battery, imu,
                 ir_cmd, k_line, lf_target):
        
        # Flags
        self.mtr_enable = mtr_enable
        self.stream_data = stream_data
        self.abort = abort
        
        # Shares
        self.eff = eff
        self.driving_mode = driving_mode # 0: straight line, 1: pivot, 2: arc
        self.setpoint = setpoint  # Share for velocity setpoint
        self.kp = kp  # Share for proportional gain
        self.ki = ki  # Share for integral gain
        self.k_line = k_line  # Share for line following K_line gain
        self.lf_target = lf_target  # Share for line following target velocity
        self.control_mode = control_mode  # Share for control mode (effort/velocity/line-follow)
        self.ir_cmd = ir_cmd  # Share for IR command

        # Hardware
        self.ser = uart # Serial UART object
        self.battery = battery # Battery object
        self.imu = imu # IMU object

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
                self.abort.put(0)
                self.prev_time = ticks_ms()
                self.state = self.S1_WAIT_FOR_COMMAND

            ### 1: WAITING STATE -----------------------------------------------
            elif (self.state == self.S1_WAIT_FOR_COMMAND):
                # Wait for user input (read available bytes non-blocking)
                if self.ser.any():
                    try:
                        self.cmd_buf = self.ser.read(1).decode()
                        self.state = self.S2_PROCESS_COMMAND # set next state
                    except Exception:
                        pass # Handle decoding errors gracefully
            
            ### 2: PROCESS COMMAND STATE ---------------------------------------
            elif self.state == self.S2_PROCESS_COMMAND:
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
                        print(f"Velocity params set: SP={sp}, Kp={kp}, Ki={ki}")
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
                        print(f"LF params set: Kp={kp}, Ki={ki}, K_line={k_line}, Target Velocity={v_target}")
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
                
                # 'i': IMU calibration command
                elif cmd == 'i':
                    print("IMU calibration command received.")
                    self.imu.set_operation_mode("ndof")
                    self.state = self.S4_CALIBRATION
                
                # 'v': print current battery voltage
                elif cmd == 'V':
                    voltage_msg = f"{self.battery.read_voltage():.2f}\n"
                    self.ser.write(voltage_msg.encode()) # send over Bluetooth
                
                # 'w': WHITE calibration command
                elif cmd == 'w':   # Calibrate on WHITE background
                    if self.ir_cmd: self.ir_cmd.put(1)

                # 'b': BLACK calibration command
                elif cmd == 'b':   # Calibrate on BLACK line
                    if self.ir_cmd: self.ir_cmd.put(2)

                else:
                    pass

                if self.state == self.S2_PROCESS_COMMAND: # if state wasn't changed
                    self.state = self.S1_WAIT_FOR_COMMAND # set back to waiting

            ### 3: MONITOR TEST STATE ------------------------------------------
            elif self.state == self.S3_MONITOR_TEST:
                # Check for Kill command during run and handle it
                if self.ser.any():
                    try:
                        key = self.ser.read(1).decode().lower()
                        if 'k' in key:
                            print("Stopping test")
                            self.abort.put(1) # Set abort flag first
                            self.mtr_enable.put(0) # Then disable motors
                            self.ser.write(b'q')  # Tell PC test is done
                            self.state = self.S1_WAIT_FOR_COMMAND # set back to waiting
                    except Exception:
                        pass # Handle decoding errors gracefully

                # If no kill command, check for normal test completion
                if not self.mtr_enable.get():
                    # Motors have already been disabled
                    print("Test complete.")
                    self.abort.put(1) # Set abort flag??
                    self.state = self.S1_WAIT_FOR_COMMAND # return to waiting

            ### 4: IMU Calibration State ---------------------------------------
            elif self.state == self.S4_CALIBRATION:
                self.curr_time = ticks_ms()
                if ticks_diff(self.curr_time, self.prev_time) > 1000:
                    self.prev_time = self.curr_time
                    imu_status_bytes = self.imu.read_calibration_status()
                    print("IMU Calibration Status (sys, gyr, acc, mag):", imu_status_bytes)
                if self.ser.any():
                    if self.ser.read(1).decode() == 'j':
                        self.imu.read_calibration_coeffs()
                        print("IMU calibration done.")
                        self.state = self.S1_WAIT_FOR_COMMAND
                
            yield self.state