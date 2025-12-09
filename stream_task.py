# stream_task.py
#
# ==============================================================================
# StreamTask (BLUETOOTH version)
# ------------------------------------------------------------------------------
# Waits for a stream command (stream_data flag) and the data collection
# completion (col_done flag). Then, buffer data in CSV format over Bluetooth one
# line at a time.
# ==============================================================================


import time

class StreamTask:
    """Streams data in CSV format over Bluetooth serial port to PC."""

    # The states of the FSM
    S0_INIT = 0
    S1_WAIT_FOR_TRIGGER = 1
    S2_STREAM_DATA = 2
    S3_DONE = 3

    # --------------------------------------------------------------------------
    ### Initialize the object's attributes
    # --------------------------------------------------------------------------
    def __init__(self,
                    eff, col_done, stream_data, uart,
                    time_q, left_pos_q, right_pos_q, left_vel_q, right_vel_q,
                    control_mode, setpoint, kp, ki, k_line, lf_target,
                    time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh,
                    motor_data_ready, abort):

         # Serial interface (Bluetooth UART)
        self.ser = uart

        # Flags
        self.col_done = col_done
        self.stream_data = stream_data
        self.motor_data_ready = motor_data_ready
        self.abort = abort
        
        # Control parameters
        self.eff = eff
        self.control_mode = control_mode
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.k_line = k_line
        self.lf_target = lf_target

        # Shares
        self.time_sh = time_sh
        self.left_pos_sh = left_pos_sh
        self.right_pos_sh = right_pos_sh
        self.left_vel_sh = left_vel_sh
        self.right_vel_sh = right_vel_sh

        # Queues
        self.time_q = time_q
        self.left_pos_q = left_pos_q
        self.right_pos_q = right_pos_q
        self.left_vel_q = left_vel_q
        self.right_vel_q = right_vel_q

        self.lines_sent = 0

       
        # ensure FSM starts in state S0_INIT
        self.state = self.S0_INIT

    # --------------------------------------------------------------------------
    ### FINITE STATE MACHINE
    # --------------------------------------------------------------------------
    def run(self):
        """Finite state machine for data streaming."""
        while True: # run infinite iterations of the state machine
            ### 0: INIT STATE --------------------------------------------------
            if (self.state == self.S0_INIT):
                # Clear stream flags
                self.stream_data.put(0)
                self.state = self.S1_WAIT_FOR_TRIGGER # set next state

            ### 1: WAIT FOR TRIGGER STATE --------------------------------------
            elif (self.state == self.S1_WAIT_FOR_TRIGGER):
                if self.stream_data.get():
                    print("Stream Task: starting data stream...")
                    self.state = self.S2_STREAM_DATA # set next state
                
            ### 2: STREAM DATA STATE -------------------------------------------
            elif (self.state == self.S2_STREAM_DATA):
                if self.motor_data_ready.get() and not self.abort.get():
                    t = self.time_sh.get()
                    pL = self.left_pos_sh.get()
                    pR = self.right_pos_sh.get()
                    vL = self.left_vel_sh.get()
                    vR = self.right_vel_sh.get()
                    # Put it all into a CSV-style line stamped with the index
                    payload = f"{self.lines_sent},{t},{pL},{pR},{vL},{vR}"
                    framed = f"<S>{payload}<E>" # these start and end delimeters will help us process data on the PC side
                    # Send the line ove r Bluetooth
                    self.ser.write(framed.encode() + b"\n")
                    self.lines_sent += 1
                else:
                    # Send end of stream marker
                    self.ser.write(b"<S>#END<E>\n") # explicit end marker for
                    # Reset flags
                    self.stream_data.put(0)
                    self.lines_sent = 0
                    self.state = self.S1_WAIT_FOR_TRIGGER # go back to wait state
                    
                self.motor_data_ready.put(0)

            yield self.state