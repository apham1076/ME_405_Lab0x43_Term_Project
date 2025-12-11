# main.py
# ==============================================================================
# ME 405 TERM PROJECT: MAIN FILE
# ==============================================================================
# Each task runs cooperatively under Dr. JR Ridgley's cotask scheduler.
# Communication between tasks happens exclusively via task_share objects
# (Shares and Queues), never through global variables.
# ------------------------------------------------------------------------------
# Tasks:
#
#   UITask: handles Bluetooth serial communication from PC; sets flags and motor
#           effort commands.
#
#   MotorControlTask: reads encoder, updates motor outputs, writes latest
#                     samples (time/pos/vel) to shares.
#
#   DataCollectionTask: moves data from shares into queues to be used by
#                       StreamTask
#
#   StreamTask: buffers data stored in queues over bluetooth serial connection
#               back to PC
#
#   SteeringTask: reads IR sensor array, computes steering correction, and
#                 sends commands to motor control task.
#
#   StateEstimationTask: predicts the future state of the system using a system #                        model and sensor inputs.

# ==============================================================================
# IMPORTS:
# ==============================================================================
import gc
import cotask
import task_share
from pyb import Pin, UART, I2C
from motor import Motor
from encoder import Encoder
from battery_droop import Battery
from IR_sensor import IRArray
from motor_task import MotorControlTask
from ui_task import UITask
from stream_task import StreamTask
from steering_task import SteeringTask
from IMU_sensor import IMU
from gc_task import GCTask
from spectator_task import SpectatorTask
from path_planning_task import PathPlanningTask
# from data_task import DataCollectionTask
# from state_estimation_task import StateEstimationTask
# from read_IMU_task import ReadIMUTask
# from bump_task import BumpTask
from os import listdir

# ==============================================================================
# MAIN FUNCTION:
# ==============================================================================
def main():
    print("\r\n=== ME405 Scheduler Start ===")
    # MAX_SAMPLES = 5
    # OBSV_SAMPLES = MAX_SAMPLES // 2  # Observer collects half the samples
    
    # ==========================================================================
    # HARDWARE SETUP:
    # ==========================================================================
    # Create motor and encoder objects
          # ARGUMENTS: |  PWM pin  |  DIR pin  |  nSLP pin | timer | channel |
    left_motor =  Motor(Pin.cpu.B5, Pin.cpu.B3, Pin.cpu.B4,    3,       2    )
    right_motor = Motor(Pin.cpu.B8, Pin.cpu.B9, Pin.cpu.C9,    4,       3    )
          # ARGUMENTS:     | CH.A pin | CH.B pin  | timer |
    left_encoder =  Encoder(Pin.cpu.A8, Pin.cpu.A9,   1   )
    right_encoder = Encoder(Pin.cpu.C6, Pin.cpu.C7,   8   )
    #---------------------------------------------------------------------------
    # Create battery measurement object
        # ARGUMENTS: | V_BATT PIN |
    battery = Battery( Pin.cpu.C2 )
    #---------------------------------------------------------------------------
    # Create UART object
        # ARGUMENTS: | UART# | BAUDRATE |
    uart =       UART(   1   ,  115200  )
    #---------------------------------------------------------------------------
    # Create IR sensor array object
    # Define the physical pins we are *currently* using, left-to-right order.
    # These are the MCU pins connected to the array.
    IR_pins = [ Pin.cpu.A0,  # physical index 1  (leftmost used)
                Pin.cpu.A6,  # physical index 2
                Pin.cpu.A1,  # physical index 3
                Pin.cpu.A7,  # physical index 4
                Pin.cpu.A4,  # physical index 5
                Pin.cpu.C4,  # physical index 6 (might be broken)
                Pin.cpu.B0,  # physical index 7
                Pin.cpu.B1,  # physical index 8
                Pin.cpu.C1,  # physical index 9
                Pin.cpu.C3,  # physical index 10
                Pin.cpu.C0 ] # physical index 11 (rightmost used)
    # Map each pin to the sensor index printed on the IR sensor board.
    # When we later have the full array (e.g., 1...11), we can update this list to match the pins.
    IR_board_indices = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
    # Indicate the number of samples to average for calibration
    IR_samples = 100
    # Pick a timer number for ADC sampling
    IR_timer = 6
    # Putting it all together to create the IR array object
        # ARGUMENTS:  |  timer  | #samples | IR_pins | sensor_indices |
    ir_array = IRArray(IR_timer, IR_samples, IR_pins, IR_board_indices)
    #---------------------------------------------------------------------------
    # Create IMU object
    sda = Pin('PB14', mode=Pin.ALT, alt=4)
    scl = Pin('PB13', mode=Pin.ALT, alt=4)
    i2c = I2C(2, I2C.CONTROLLER, baudrate=400000)
    imu = IMU(i2c)
    imu.set_operation_mode("config")  # start in config mode

    # ==========================================================================
    # CHECK FOR CALIBRATION FILES:
    # ==========================================================================
    filelist = listdir()
    # Check for existing IMU calibration file
    if "imu_cal.bin" in filelist:
        print("IMU calibration file 'imu_cal.bin' found.")
        imu.write_calibration_coeffs()
    else:
        print("No IMU calibration file found.")

    # Check for existing IR calibration file
    if "IR_cal.txt" in filelist:
        print("IR calibration file 'IR_cal.txt' found.")
        ir_array.set_calibration()
    else:
        print("No IR calibration file found.")

    # ==========================================================================
    # SETUP SHARES AND QUEUES:
    # ==========================================================================
    # ------------------------------- SHARES -----------------------------------
    # --------------------------------------------------------------------------
    # Streaming shares...
    time_sh = task_share.Share('H', name='Time share')
    left_pos_sh = task_share.Share('h', name= 'Left motor position share')
    right_pos_sh = task_share.Share('h', name= 'Right motor position share')
    left_vel_sh = task_share.Share('h', name= 'Left motor velocity share')
    right_vel_sh = task_share.Share('h', name= 'Right motor velocity share')
    # --------------------------------------------------------------------------
    # Motor control shares...
    start_time_sh = task_share.Share('L', name='Start Time Share')
    eff = task_share.Share('f', name='Requested Effort') # float effort percent
    setpoint = task_share.Share('f', name='Velocity Setpoint') # 'h' for signed 16-bit to handle larger velocity values
    kp = task_share.Share('f', name='Prop. Gain') # 'f' for float to store Kp
    ki = task_share.Share('f', name='Integral Gain') # 'f' for float to store Ki
    left_eff_sh = task_share.Share('f', name='Left Motor Effort Share')
    right_eff_sh = task_share.Share('f', name='Right Motor Effort Share')
    # --------------------------------------------------------------------------
    # Driving mode and control mode shares...
    driving_mode = task_share.Share('B', name='Driving Mode')
    # driving mode --- 0: straight line, 1: pivot, 2: arc
    control_mode = task_share.Share('B', name='Control Mode')
    # control mode --- 0: effort, 1: velocity, 2: line follow
    # --------------------------------------------------------------------------
    # Line following shares...
    left_sp_sh = task_share.Share('f', name='LF Left Setpoint')
    right_sp_sh = task_share.Share('f', name='LF Right Setpoint')
    ir_cmd = task_share.Share('B', name='IR Calibrate Cmd')
    k_line = task_share.Share('f', name='LineFollow K_line')
    lf_target = task_share.Share('f', name='LineFollow Target')
    bias = task_share.Share('f', name='LineFollow Centroid Bias')
    # Initialize line following shares...
    left_sp_sh.put(0.0)
    right_sp_sh.put(0.0)
    ir_cmd.put(0)
    k_line.put(0.0)
    lf_target.put(0.0)
    bias.put(0.0)
    # --------------------------------------------------------------------------
    # State Estimation shares...
    # psi_sh = task_share.Share('f', name='Yaw Angle Share')
    # psi_dot_sh = task_share.Share('f', name='Yaw Rate Share')
    # obsv_time_sh = task_share.Share('H', name='Observed Time Share')
    # obsv_sL_sh = task_share.Share('h', name='Observed Left Displacement Share')
    # obsv_sR_sh = task_share.Share('h', name='Observed Right Displacement Share')
    # obsv_psi_sh = task_share.Share('h', name='Observed Yaw Angle Share')
    # obsv_psi_dot_sh = task_share.Share('h', name='Observed Yaw Rate Share')
    # obsv_left_vel_sh = task_share.Share('h', name='Observed Left Velocity Share')
    # obsv_right_vel_sh = task_share.Share('h', name='Observed Right Velocity Share')
    # obsv_s_sh = task_share.Share('h', name='Observed Linear Displacement Share')
    # obsv_yaw_sh = task_share.Share('h', name='Observed Yaw Share')
    # --------------------------------------------------------------------------
    # Boolean flags (shares)...
    mtr_enable = task_share.Share('B', name='Motor Enable Flag')
    stream_data = task_share.Share('B', name='Stream Data Flag')
    abort = task_share.Share('B', name='Abort Flag')
    run_observer = task_share.Share('B', name='Run Observer Flag')
    read_IMU_flg = task_share.Share('B', name='Read IMU flag')
    motor_data_ready = task_share.Share('B', name='Motor Data Ready Flag')
    obsv_data_ready = task_share.Share('B', name='Observer Data Ready Flag')
    planning = task_share.Share('B', name='Path Planning Mode Flag')
    # --------------------------------------------------------------------------
    # Spectator Task shares...
    total_s_sh = task_share.Share('f', name='Total Displacement Share')
    abs_x_sh = task_share.Share('f', name='Absolute X Position Share')
    abs_y_sh = task_share.Share('f', name='Absolute Y Position Share')
    abs_theta_sh = task_share.Share('f', name='Absolute Theta Share')
    #
    # ------------------------------- QUEUES -----------------------------------
    # (none for now)

    # ==========================================================================
    # CREATE TASK OBJECTS (since tasks are written as classes):
    # ==========================================================================
    ui_task_obj = UITask(mtr_enable, stream_data, abort,
                         eff, driving_mode, setpoint, kp, ki, control_mode,
                         uart, battery, imu,
                         ir_cmd, k_line, lf_target, planning)

    motor_task_obj = MotorControlTask(left_motor, right_motor,
                                      left_encoder, right_encoder,
                                      battery,
                                      eff, mtr_enable, motor_data_ready, run_observer, abort, driving_mode, setpoint, kp, ki, control_mode, start_time_sh,
                                      time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh,
                                      left_sp_sh, right_sp_sh, left_eff_sh, right_eff_sh)

    stream_task_obj = StreamTask(eff, stream_data, uart,
                                 control_mode, setpoint, kp, ki, k_line, lf_target,
                                 time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh,
                                 motor_data_ready, abort)

    steering_task_obj = SteeringTask(ir_array, battery,
                                 control_mode, ir_cmd,
                                 left_sp_sh, right_sp_sh,
                                 k_line, lf_target, bias)

    gc_task_obj = GCTask()

    spectator_task_obj = SpectatorTask(run_observer,
                                       left_pos_sh, right_pos_sh, total_s_sh,
                                       abs_x_sh, abs_y_sh, abs_theta_sh)
    
    path_planning_task_obj = PathPlanningTask(planning, bias, total_s_sh, kp, ki, k_line, lf_target, control_mode, abort, mtr_enable)

    # data_task_obj = DataCollectionTask(col_start, col_done,
    #                                    mtr_enable, abort, motor_data_ready, obsv_data_ready,
    #                                    time_q, left_pos_q, right_pos_q, left_vel_q, right_vel_q,
    #                                    obsv_time_q, obsv_sL_q, obsv_sR_q, obsv_psi_q, obsv_psi_dot_q,
    #                                    time_sh, left_pos_sh, right_pos_sh, left_vel_sh, right_vel_sh, obsv_time_sh, obsv_sL_sh, obsv_sR_sh, obsv_psi_sh, obsv_psi_dot_sh,
    #                                    obsv_left_vel_sh, obsv_right_vel_sh, obsv_s_sh, obsv_yaw_sh,
    #                                    obsv_left_vel_q, obsv_right_vel_q, obsv_s_q, obsv_yaw_q)

    # state_estimation_task_obj = StateEstimationTask(start_time_sh,
    #                                                 run_observer,
    #                                                 obsv_data_ready,
    #                                                 obsv_time_sh, 
    #                                                 left_pos_sh, right_pos_sh,
    #                                                 left_vel_sh, right_vel_sh,
    #                                                 psi_sh, psi_dot_sh,
    #                                                 left_eff_sh, right_eff_sh,
    #                                                 battery,
    #                                                 obsv_sL_sh, obsv_sR_sh, obsv_psi_sh, obsv_psi_dot_sh, obsv_left_vel_sh, obsv_right_vel_sh, obsv_s_sh, obsv_yaw_sh)

    # read_IMU_task_obj = ReadIMUTask(imu, left_pos_sh, right_pos_sh, psi_sh, psi_dot_sh, read_IMU_flg)
    
    # bump_task_obj = BumpTask(abort, bump_pin='H0')

    # ==========================================================================
    # CREATE cotask.Task WRAPPERS:
    # ==========================================================================
	# (If trace is enabled for any task, memory will be allocated for state transition tracing, and the application will run out of memory after a while and quit. Therefore, use tracing only for debugging and set trace to False when it's not needed)

    _motor_task = cotask.Task(motor_task_obj.run, name='Motor Control Task', priority=3, period=20, profile=True, trace=False)

    _ui_task = cotask.Task(ui_task_obj.run, name='User Interface Task', priority=0, period=100, profile=True, trace=False)

    _stream_task = cotask.Task(stream_task_obj.run, name='Stream Task', priority=1, period=20, profile=True, trace=False)

    _steering_task = cotask.Task(steering_task_obj.run, name='Steering Task', priority=2, period=40, profile=True, trace=False)

    _gc_task = cotask.Task(gc_task_obj.run, name='Garbage Collector Task', priority=0, period=100, profile=True, trace=False)

    _spectator_task = cotask.Task(spectator_task_obj.run, name='Spectator Task', priority=2, period=20, profile=True, trace=False)

    _path_planning_task = cotask.Task(path_planning_task_obj.run, name='Path Planning Task', priority=2, period=40, profile=True, trace=False)

    # _data_collection_task = cotask.Task(data_task_obj.run, name='Data Collection Task', priority=2, period=20, profile=True, trace=False)

    # _state_estimation_task = cotask.Task(state_estimation_task_obj.run, name='State Estimation Task', priority=2, period=20, profile=True, trace=False)

    # _read_IMU_task = cotask.Task(read_IMU_task_obj.run, name='Read IMU Task', priority=1, period=20, profile=True, trace=False)

    # _bump_task = cotask.Task(bump_task_obj.run, name='Bump Task', priority=4, period=20, profile=True, trace=False)

    # ==========================================================================
    # ADD TASKS TO SCHEDULER LIST:
    # ==========================================================================
	# Now add (append) the tasks to the scheduler list
    cotask.task_list.append(_motor_task)
    cotask.task_list.append(_ui_task)
    cotask.task_list.append(_stream_task)
    cotask.task_list.append(_steering_task)
    cotask.task_list.append(_gc_task)
    cotask.task_list.append(_spectator_task)
    cotask.task_list.append(_path_planning_task)
    # cotask.task_list.append(_data_collection_task)
    # cotask.task_list.append(_state_estimation_task)
    # cotask.task_list.append(_read_IMU_task)
    # cotask.task_list.append(_bump_task)

    # ==========================================================================
    # RUN THE SCHEDULER:
    # ==========================================================================
	# Run the memory garbage collector to ensure memory is as defragmented as possible before the real-time scheduler is started
    gc.collect()
    # The scheduler is ready to start
    
    # --------------------------------------------------------------------------
	# Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
    print("Scheduler running... Press Ctrl-C to halt.\r\n")
    while True: # attempt to run infinite iterations of the scheduler
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt: # if ^C pressed, stop motors and exit
            left_motor.disable()
            right_motor.disable()
            print("Keyboard interrupt detected, stopping motors and halting scheduler.")
            break
        except: # if another error occurs, stop motors and raise error
            left_motor.disable()
            right_motor.disable()
            print("Unexpected error in scheduler, stopping motors.")
            raise

    # Diagnostics on exit: print a table of task data and a table of shared information data
    print("\n=== Scheduler Halted ===")
    print('\n' + str(cotask.task_list))
    # print(task_share.show_all())
    # print(_ui_task.get_trace())
    # print(_motor_task.get_trace())
    # print(_data_collection_task.get_trace())
    # print(_stream_task.get_trace())
    # print(_steering_task.get_trace())
    # print(_read_IMU_task.get_trace())
    # print(_state_estimation_task.get_trace())

# ==============================================================================
# RUN THE "MAIN" FUNCTION:
# ==============================================================================
if __name__ == "__main__":
	main()