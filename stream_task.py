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
                    obsv_time_q, obsv_sL_q, obsv_sR_q, obsv_psi_q, obsv_psi_dot_q,
                    obsv_left_vel_q, obsv_right_vel_q, obsv_s_q, obsv_yaw_q,
                    control_mode, setpoint, kp, ki, k_line, lf_target,
                    time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh,
                    motor_data_ready):

         # Serial interface (Bluetooth UART)
        self.ser = uart

        # Flags
        self.col_done = col_done
        self.stream_data = stream_data
        self.motor_data_ready = motor_data_ready
        
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

        self.obsv_time_q = obsv_time_q
        self.obsv_sL_q = obsv_sL_q
        self.obsv_sR_q = obsv_sR_q
        self.obsv_psi_q = obsv_psi_q
        self.obsv_psi_dot_q = obsv_psi_dot_q

        self.obsv_left_vel_q = obsv_left_vel_q
        self.obsv_right_vel_q = obsv_right_vel_q
        self.obsv_s_q = obsv_s_q
        self.obsv_yaw_q = obsv_yaw_q

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

                # # Get control mode info to send to PC
                # mode_val = int(self.control_mode.get())
                # if mode_val == 0:
                #     mode = "E"  # effort mode
                #     control_val = self.eff.get()
                # elif mode_val == 1:
                #     mode = "V"  # velocity mode
                #     control_val = self.setpoint.get()
                # else:
                #     mode = "L"  # line follow mode
                #     control_val = self.lf_target.get()

                # # Determine the number of samples actually in the queues
                # _size = self.time_q.num_in()

                # # Send a line with the control mode and parameters
                # if mode == "E":
                #     line = f"{mode},{control_val},{_size}\r\n"
                # elif mode == "V":
                #     line = f"{mode},{control_val},{self.kp.get():.2f},{self.ki.get():.2f},{_size}\r\n"
                # else:
                #     line = f"{mode},{control_val},{self.kp.get():.2f},{self.ki.get():.2f},{self.k_line.get():.2f},{_size}\r\n"

                # self.ser.write(line.encode()) # write the line over Bluetooth
                # yield self.state # write only one line then yield

                # # while we still have items in queue
                # index = 0 # sample index number (sent to PC for alignment)
                # while self.time_q.any():
                #     # Get items from the queues
                #     t = self.time_q.get()
                #     pL = self.left_pos_q.get()
                #     pR = self.right_pos_q.get()
                #     vL = self.left_vel_q.get()
                #     vR = self.right_vel_q.get()
                #     # Put it all into a CSV-style line stamped with the index
                #     # Build a framed message with delimiters for better PC parsing
                #     payload = f"{index},{t},{pL},{pR},{vL},{vR}"

                #     framed = f"<S>{payload}<E>" # these start and end delimeters will help us process data on the PC side
                #     # Send the line over Bluetooth
                #     self.ser.write(framed.encode() + b"\n")
                #     # Increment the sample index
                #     index += 1
                #     # Only write one line, then yield
                #     yield self.state
                
                # # Once all samples have been sent, mark the end of the stream
                # self.ser.write(b"<S>#END1<E>\n") # explicit end marker for PC
                # yield self.state

                # index = 0 # sample index number (sent to PC for alignment)
                # while self.obsv_time_q.any():
                #     # Get items from the queues
                #     t = self.obsv_time_q.get()
                #     # sL = self.obsv_sL_q.get()
                #     # sR = self.obsv_sR_q.get()
                #     # psi = self.obsv_psi_q.get()
                #     # psi_dot = self.obsv_psi_dot_q.get()
                #     omega_L = self.obsv_left_vel_q.get()
                #     omega_R = self.obsv_right_vel_q.get()
                #     s = self.obsv_s_q.get()
                #     yaw = self.obsv_yaw_q.get()
                #     # Put it all into a CSV-style line stamped with the index
                #     # Build a framed message with delimiters for better PC parsing
                #     # payload = f"{index},{t},{sL},{sR},{psi},{psi_dot}"
                #     payload = f"{index},{t},{omega_L},{omega_R},{s},{yaw}"
                #     # payload = f"{index},{t},{sL:.3f},{sR:.3f},{psi:.3f},{psi_dot:.3f}"

                #     framed = f"<S>{payload}<E>" # these start and end delimeters will help us process data on the PC side
                #     # Send the line over Bluetooth
                #     self.ser.write(framed.encode() + b"\n")
                #     # Increment the sample index
                #     index += 1
                #     # Only write one line, then yield

                if self.motor_data_ready.get():
                    self.motor_data_ready.put(0)
                    if self.lines_sent < 300:
                        t = self.time_sh.get()
                        pL = self.left_pos_sh.get()
                        pR = self.right_pos_sh.get()
                        vL = self.left_vel_sh.get()
                        vR = self.right_vel_sh.get()
                        # Put it all into a CSV-style line stamped with the index
                        payload = f"{self.lines_sent},{t},{pL},{pR},{vL},{vR}"
                        print(payload)
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

                    yield self.state

            yield self.state