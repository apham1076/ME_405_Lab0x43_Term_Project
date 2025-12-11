# stream_task.py
#
# ==============================================================================
# StreamTask (BLUETOOTH version)
# ------------------------------------------------------------------------------
# Waits for stream_data flag and motor_data_ready flag, then streams CSV lines 
# (one at a time) over Bluetooth to the PC.
# ==============================================================================

import time

class StreamTask:
    """Streams data (in CSV format) over Bluetooth if enabled."""

    # The states of the FSM
    S0_INIT = 0
    S1_WAIT_FOR_TRIGGER = 1
    S2_STREAM_DATA = 2

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self,
                 eff, stream_data, uart,
                 control_mode, setpoint, kp, ki, k_line, lf_target,
                 time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh,
                 motor_data_ready, abort, total_s_sh, abs_x_sh, abs_y_sh, abs_theta_sh):

        # Serial interface (Bluetooth UART)
        self.ser = uart

        # Flags
        self.stream_data = stream_data
        self.motor_data_ready = motor_data_ready
        self.abort = abort
        
        # Control parameters
        # self.eff = eff
        # self.control_mode = control_mode
        # self.setpoint = setpoint
        # self.kp = kp
        # self.ki = ki
        # self.k_line = k_line
        # self.lf_target = lf_target

        # Shares
        self.time_sh = time_sh
        self.left_pos_sh = left_pos_sh
        self.right_pos_sh = right_pos_sh
        self.left_vel_sh = left_vel_sh
        self.right_vel_sh = right_vel_sh
        # self.total_s_sh = total_s_sh
        # self.abs_x_sh = abs_x_sh
        # self.abs_y_sh = abs_y_sh
        # self.abs_theta_sh = abs_theta_sh
       
        # Initial values
        self.sent_end = 0
        self.lines_sent = 0
        self.state = self.S0_INIT # start the FSM in the INIT state

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        """Finite state machine for data streaming."""
        while True: # run infinite iterations of the state machine
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):
                self.stream_data.put(1)  # DEFAULT: streaming ON
                self.sent_end = 0 # flag to track if END marker has been sent
                self.lines_sent = 0 # initialize line counter to zero
                self.state = self.S1_WAIT_FOR_TRIGGER # set next state

            ### 1: WAIT FOR TRIGGER STATE --------------------------------------
            elif (self.state == self.S1_WAIT_FOR_TRIGGER):
                # Wait until some other task (e.g. UI) has turned on streaming
                if self.stream_data.get():
                    print("Stream Task: starting live data streaming...")
                    self.state = self.S2_STREAM_DATA # set next state
                
            ### 2: STREAM DATA STATE -------------------------------------------
            elif (self.state == self.S2_STREAM_DATA):
                # 1) If ABORT, send END once and reset counter, but leave stream_data alone (so streaming can stay "armed" for next test)
                if self.abort.get():
                    if self.sent_end == 0:
                        self.ser.write(b"<S>#END<E>\n") # explicit end marker
                        self.sent_end = 1
                    self.lines_sent = 0 # reset line counter for next stream

                # 2) If streaming is turned OFF, send END once, reset counter, and go back to WAIT_FOR_TRIGGER until re-enabled
                elif not self.stream_data.get():
                    if self.sent_end == 0:
                        self.ser.write(b"<S>#END<E>\n") # explicit end marker
                        self.sent_end = 1
                    self.lines_sent = 0 # reset line counter for next stream
                    print("Stream Task: live data streaming ended.")
                    self.state = self.S1_WAIT_FOR_TRIGGER # set next state

                # 3) Normal streaming: only send when new motor data is ready to avoid duplicates
                elif self.motor_data_ready.get():
                    self.sent_end = 0 # we have new data, so clear the END sent flag; next time abort/streaming off happens, we'll need to send END again exactly once
                    # Read the relevant data from the shares
                    t = self.time_sh.get()
                    pL = self.left_pos_sh.get()
                    pR = self.right_pos_sh.get()
                    vL = self.left_vel_sh.get()
                    vR = self.right_vel_sh.get()

                    # Put it all into a CSV-style line stamped with the index
                    payload = f"{self.lines_sent},{t},{pL},{pR},{vL},{vR}"
                    framed = f"<S>{payload}<E>" # framed packet to send; start and end delimeters help process data on the PC side
                    # Send the line over Bluetooth
                    self.ser.write(framed.encode() + b"\n")
                    
                    self.lines_sent += 1 # Increment the line counter
                    self.motor_data_ready.put(0) # Clear the data-ready flag (avoid duplicates)

            yield self.state # yield the current state for the scheduler