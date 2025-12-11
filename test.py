# Script to run on PC to run motor characterization test and perform data collection

from serial import Serial, SerialException
from time import sleep
import matplotlib.pyplot as plt
import msvcrt
import pandas as pd
import numpy as np
import os
import queue as local_queue
import time
import math

key = ''                    # Stores value of key pressed as a string
running = False             # Set when motor testing is progress
streaming = False           # True when we are in the middle of receiving streamed data frames
print_stream_frames = True  # Toggle printing of incoming data frames
stream_expected = True      # MCU streaming service is enabled flag (default ON since MCU starts with streaming enabled)
first_frame = True          # Set on first data stream to create run entry
runs = {}                   # Dict to contain runs
run_count = 0               # Number of runs
frame_buffer = ""           # Buffer for delimiter-framed lines
control_mode = 0            # 0 = effort mode, 1 = velocity mode, 2 = line follow mode
driving_mode = 0            # 0 = straight line, 1 = arc, 2 = pivot
effort = 0                  # Current effort value
setpoint = 0                # Current velocity setpoint in rad/s
kp = 0.0                    # Current proportional gain
ki = 0.0                    # Current integral gain
k_line = 0.0                # Current line following gain
first = True
done = False
# mode = 1                  # 1, 2, 3 = straight, pivot, arc
queue = local_queue.Queue() # Queue to hold tests
queuing = False             # Set when queuing tests
creating_run = False        # Set when creating a new run
test_origin = "manual"      # tracks how the current test started: "manual" or "queue"

control_mode_dict = {0: "effort", 1: "velocity", 2: "line following"}

driving_mode_dict = {0: "straight", 1: "pivot", 2: "arc"}

user_prompt = '''\r\nCommand keys:
    t      : Select a test to run: Effort, Velocity, Line Following
    u      : Queue tests
    r      : Run test
    k      : Kill (stop) motors
    n      : Toggle driving mode (straight, pivot, arc)
    s      : Toggle live data streaming ON/OFF
    p      : Toggle printing of streamed data frames
    d      : Save complete data to CSV and optionally create plots
    b      : Check battery voltage (prints to this terminal)
    c      : Calibrate sensors: IR sensors or IMU
    h      : Help / show this menu
    ctrl-c : Interrupt this program
'''

# --------------------------------------------------------------------------
# CONSTANTS FOR UNIT CONVERSIONS
GEAR_RATIO = 3952/33  # Gear ratio of motor to wheel (~120)
CPR_MOTOR = 12 # Counts per rev of the motor shaft (before gearbox)
CPR_WHEEL = GEAR_RATIO*CPR_MOTOR  # Counts per rev of the wheel (~1440)
RAD_PER_COUNT = 2 * math.pi / CPR_WHEEL  # Radians per count
WHEEL_RADIUS_MM = 35  # Wheel radius in mm
# --------------------------------------------------------------------------

# ==============================================================================
# HELPER FUNCTIONS
# ==============================================================================

# Map an effort percentage (0-100) to the single-character key
def eff_to_key(eff):
    if eff == 100:
        return 'a'
    # assume efforts are multiples of 10
    digit = int(eff // 10)
    return str(digit)
# ------------------------------------------------------------------------------
# Map a single-character key to an effort percentage (0-100)
def key_to_eff(key):
    if key == 'a':
        return 100
    if key.isdigit():
        digit = int(key)
        return digit * 10
    return None
# ------------------------------------------------------------------------------
# Function to create dictionary for storing data from one run
def create_run(control_mode, control_val, driving_mode, run_num, size):
    time = np.zeros(size)
    p1 = np.zeros(size)
    p2 = np.zeros(size)
    v1 = np.zeros(size)
    v2 = np.zeros(size)
    ctrl = np.zeros(size)  # Store control value (effort or setpoint)

    # obsv_size = size // 2
    # obsv_time = np.zeros(obsv_size)
    # sL = np.zeros(obsv_size)
    # sR = np.zeros(obsv_size)
    # psi = np.zeros(obsv_size)
    # psi_dot = np.zeros(obsv_size)
    # omega_L = np.zeros(obsv_size)
    # omega_R = np.zeros(obsv_size)
    # s = np.zeros(obsv_size)
    # yaw = np.zeros(obsv_size)

    df = pd.DataFrame({
        "_time": time,
        "_control": ctrl,
        "_left_pos": p1,
        "_right_pos": p2,
        "_left_vel": v1,
        "_right_vel": v2
    })

    # df2 = pd.DataFrame({
    #     "_obsv_time": obsv_time,
    #     "_obsv_sL": sL,
    #     "_obsv_sR": sR,
    #     "_obsv_psi": psi,
    #     "_obsv_psi_dot": psi_dot,
    #     "_obsv_left_vel": omega_L,
    #     "_obsv_right_vel": omega_R,
    #     "_obsv_s": s,
    #     "_obsv_yaw": yaw
    # })

    return {"control_val": control_val, "run_num": run_num, "control_mode": control_mode_dict[control_mode], "driving_mode": driving_mode_dict[driving_mode], "size": size, "motor_data": df}

# ------------------------------------------------------------------------------
# Function to clean DataFrame by removing all-zero rows except leading zeros
def clean_data(df, cols=None, mode='all'):
    """
    Clean DataFrame by removing all zero rows except initial zeros up to the first non-zero data.
    
    Parameters:
      df: pandas.DataFrame
      cols: list of columns to check. If None, defaults to all motor_data columns.
      mode: 'all' -> remove rows where ALL specified cols are zero;
            'any' -> remove rows where ANY specified cols are zero.

    Returns: (cleaned_df, removed_count)
    """
    if cols is None:
        cols = ["_time", "_left_pos", "_right_pos", "_left_vel", "_right_vel"]

    # Ensure columns exist
    cols = [c for c in cols if c in df.columns]
    if not cols:
        return df, 0

    # Create boolean mask for zero-rows according to mode
    if mode == 'any':
        zero_mask = (df[cols] == 0).any(axis=1)
    else:
        zero_mask = (df[cols] == 0).all(axis=1)

    # Find the indices of non-zero rows
    nonzero_idx = (~zero_mask).to_numpy().nonzero()[0]
    
    if len(nonzero_idx) == 0:
        # All rows are zero -> keep first row only
        if len(df) <= 1:
            return df.reset_index(drop=True), 0
        cleaned = df.iloc[:1].reset_index(drop=True)
        removed = len(df) - 1
        return cleaned, removed

    # Find first and last non-zero indices
    first_nonzero = int(nonzero_idx[0])
    last_nonzero = int(nonzero_idx[-1])

    # Keep all rows from start up to first non-zero (preserve leading zeros)
    # plus all non-zero rows up to the last non-zero row
    cleaned = df.iloc[:last_nonzero + 1].copy()
    
    # Remove any zero rows between first and last non-zero (excluding leading zeros)
    if first_nonzero > 0:  # If we have leading zeros
        # Keep all rows up to first_nonzero (leading zeros)
        # Then only keep non-zero rows after that
        mask = pd.Series(True, index=cleaned.index)
        mask[first_nonzero:] = ~zero_mask[first_nonzero:last_nonzero + 1]
        cleaned = cleaned[mask].reset_index(drop=True)
    else:
        # No leading zeros, just keep non-zero rows
        cleaned = cleaned[~zero_mask[:last_nonzero + 1]].reset_index(drop=True)

    removed = len(df) - len(cleaned)
    return cleaned, removed

# ------------------------------------------------------------------------------
# Function to perform START / ACK handshake for data streaming
def start_stream_handshake(ser, timeout=2.0, retries=3):
    """Send START and wait for ACK from MCU. Returns True if ACK received."""
    for attempt in range(retries):
        try:
            ser.write(b'#START\n')
            ser.flush()
        except Exception:
            return False

        start = time.time()
        while time.time() - start < timeout:
            try:
                line = ser.readline().decode(errors='ignore').strip()
            except Exception:
                line = ''
            if not line:
                continue
            if line == '$ACK':
                return True
            # ignore other lines
        # retry
    return False

# ==============================================================================
# MAIN PROGRAM
# ==============================================================================

# Create 'runs' directory if it doesn't exist
try:
    os.makedirs('runs', exist_ok=True)
    # print("Output directory 'runs' is ready")
except Exception as e:
    print(f"Warning: Could not create 'runs' directory: {e}")

# Establish Bluetooth connection
try:
    ser = Serial('COM9', baudrate=115200, timeout=1)
except SerialException:
    print("Unable to connect to port")

print("Begin Program:")
print(user_prompt)

while True:
    try:
        # Check for key pressed
        if msvcrt.kbhit():
            key = msvcrt.getch().decode().lower()

            if key == 't':
                if running:
                    print("Cannot select test while test is running")
                elif streaming:
                    print("Cannot select test while data is streaming")
                else:
                    print("Select test to run:")
                    print("  e: Effort mode")
                    print("  v: Velocity mode")
                    print("  l: Line Following mode")
                    selected = input("Enter choice (e, v, l, or q to quit): ")
                    if selected == 'e':
                        control_mode = 0
                        print("Selected Effort mode")
                        # Accept a single key (0-9 or 'a') or an integer percentage 0-100
                        key_in = input("Enter effort key (0-9 or 'a' for 100%) or percent (0-100): ").strip()
                        # Only use key_to_eff for single characters
                        if len(key_in) == 1:
                            eff_val = key_to_eff(key_in)
                            if eff_val is None:
                                print("Invalid effort entry. Use 0-9, 'a', or a number 0-100.")
                                continue
                        else:
                            # Multi-digit input: parse as integer percent directly
                            try:
                                eff_val = int(key_in)
                                if eff_val < 0 or eff_val > 100:
                                    raise ValueError()
                            except Exception:
                                print("Invalid effort entry. Use 0-9, 'a', or a number 0-100.")
                                continue
                        effort = eff_val
                        line = 'e' + eff_to_key(effort)
                    elif selected == 'v':
                        control_mode = 1
                        print("Selected Velocity mode")
                        gains = input("Enter setpoint (mm/s), Kp, and Ki separated by a comma (e.g., 40,1.5,0.1): ")
                        try:
                            sp_str, kp_str, ki_str = gains.split(',')
                            # Convert user's setpoint from mm/s to rad/s
                            setpoint_mm_s = float(sp_str) # mm/s
                            setpoint_rad_s = setpoint_mm_s / WHEEL_RADIUS_MM  # rad/s
                            kp = float(kp_str)
                            ki = float(ki_str)
                            # Scale setpoint and gains (*100) for transmission
                            setpoint_scaled = int(setpoint_rad_s * 100)
                            kp_scaled = int(kp * 100)
                            ki_scaled = int(ki * 100)
                            # Semd as 4-digit integers
                            line = f'v{setpoint_scaled:04d}{kp_scaled:04d}{ki_scaled:04d}'
                            # Store setpoint in mm/s for the data log
                            setpoint = setpoint_mm_s
                        except ValueError:
                            print("Invalid format. Please enter two numbers separated by a comma.")

                    elif selected == 'l':
                        control_mode = 2
                        print("Selected Line Following mode")
                        gains = input("Enter Kp, Ki, K_line, and target (mm/s) separated by commas (e.g., 1.5,0.1,2.0,0.5): ")
                        try:
                            kp_str, ki_str, k_line_str, target_str = gains.split(',')
                            # Convert user's setpoint from mm/s to rad/s
                            setpoint_mm_s = float(target_str) # mm/s
                            setpoint_rad_s = setpoint_mm_s / WHEEL_RADIUS_MM  # rad/s
                            kp = float(kp_str)
                            ki = float(ki_str)
                            k_line = float(k_line_str)
                            # Scale setpoint and gains (*100) for transmission
                            target_scaled = int(setpoint_rad_s * 100)
                            kp_scaled = int(kp * 100)
                            ki_scaled = int(ki * 100)
                            k_line_scaled = int(k_line * 100)
                            line = f'l{kp_scaled:04d}{ki_scaled:04d}{k_line_scaled:04d}{target_scaled:04d}'
                            # Store setpoint in mm/s for the data log
                            setpoint = setpoint_mm_s
                        except ValueError:
                            print("Invalid format. Please enter four numbers separated by commas.")
                    
                    elif selected == 'q':
                        print("Test selection cancelled.")
                        continue
                    else:
                        print("Invalid selection. Enter 'e', 'v', 'l', or 'q' to quit.")

                    # Send configuration line to Romi    
                    ser.write(line.encode())
                    print("Command sent to Romi. Hit 'r' to run the test.")

            elif key == 'u':
                if running:
                    print("Cannot add tests to queue while test is running")
                elif streaming:
                    print("Cannot add tests to queue while data is streaming")
                else:
                    print(("Select from the following:"))
                    print("  l: View items in queue")
                    print("  c: Clear queue")
                    print("  a: Add test to queue")
                    selected = input("Enter choice (l, c, a, or q to quit): ")
                    if selected == 'l':
                        if queue.is_empty():
                            print("Queue is empty.")
                        else:
                            print("Current queue:")
                            for i in queue.items:
                                # Format for test is (mode, param1, param2, ...)
                                print(i)
                    
                    elif selected == 'c':
                        while not queue.is_empty():
                            queue.dequeue()
                        print("Queue cleared.")
                    elif selected == 'a':
                        # For now only allow adding effort tests
                        eff = input("Enter effort test in the following format: start, end, step (e.g., 0,100,10): ")
                        try:
                            start_str, end_str, step_str = eff.split(',')
                            start = int(start_str)
                            end = int(end_str)
                            step = int(step_str)
                            if start < 0 or end > 100 or step <= 0 or start > end:
                                raise ValueError("Invalid range or step")
                            # Add tests to queue
                            for e in range(start, end + 1, step):
                                queue.enqueue( ('effort', e) )
                        except ValueError as e:
                            print(f"Invalid format: {e}")
                        queuing = True if not queue.is_empty() else False
                    elif selected == 'q':
                        print("Queue operation cancelled.")
                        continue
                    
            elif key == 'r':
                if running:
                    print("Test is already running")
                elif streaming:
                    print("Data is streaming.")
                else:
                    print("Starting test...")
                    # Flush buffer
                    # Clear out any stale flags (like #END from toggling streaming off)
                    try:
                        while ser.in_waiting:
                            ser.read(ser.in_waiting)
                    except Exception:
                        pass
                    running = True
                    first = True
                    ser.write(b'r')
            
            # Send a 'z' to Romi to tell it to enter path planning mode
            elif key == 'z':
                if running:
                    print("Cannot enter path planning mode while test is running")
                elif streaming:
                    print("Cannot enter path planning mode while data is streaming")
                else:
                    print("Entering path planning mode...")
                    ser.write(b'z')

            elif key == 'k':
                if running:
                    # Kill motors
                    ser.write(b'k')
                    running = False
                    streaming = False
                    queuing = False
                    print("End test. Stop motors")
                    # Flush any pending MCU responses (like #END) so they don't interfere with the next test.
                    try:
                        sleep(0.2)
                        while ser.in_waiting:
                            ser.read(ser.in_waiting)
                    except Exception:
                        pass
                else:
                    print("Motors are already off")

            elif key == 's':
                if stream_expected:
                    print("Requesting MCU to STOP streaming...")
                    ser.write(b's') # Toggles MCU streaming OFF
                    stream_expected = False
                else:
                    print("Requesting MCU to START streaming...")
                    ser.write(b's') # Toggles MCU streaming ON
                    stream_expected = True
            
            elif key == 'p':
                print_stream_frames = not print_stream_frames
                print(f"Printing streamed frames: {print_stream_frames}")

            elif key == 'n':
                if running:
                    print("Test is already running, cannot toggle driving mode")
                elif streaming:
                    print("Data is already streaming.")
                else:
                    print("Toggle driving mode")
                    driving_mode = (driving_mode + 1) % 3
                    ser.write(b'n')

            elif key == 'c':
                if running:
                    print("Cannot calibrate while test is running")
                elif streaming:
                    print("Cannot calibrate while data is streaming")
                else:
                    print("Select calibration to perform:")
                    print("  1: IR Sensors")
                    print("  2: IMU")
                    selected = input("Enter choice (1 or 2): ")
                    if selected == '1':
                        # print("IR sensor calibration command sent to Romi.")
                        key = input("Place Romi on white surface and press any key to continue...")
                        ser.write(b'w')
                        print("White calibration done. Now place Romi on black surface.")
                        key = input("Place Romi on black surface and press any key to continue...")
                        ser.write(b'b')
                        print("Black calibration done.")
                    elif selected == '2':
                        # print("IMU calibration command sent to Romi.")
                        ser.write(b'i')
                        key = input("Follow IMU calibration procedure and press any key to continue...")
                        ser.write(b'j')
                        print("IMU calibration command sent.")
                    else:
                        print("Invalid selection. Enter '1' or '2'.")

            elif key == 'b':
                # Request battery voltage (firmware will respond with a number and newline)
                if streaming:
                    print("Battery query deferred (currently streaming)")
                else:
                    ser.write(b'V')
                    # try to read a short reply
                    sleep(0.05)
                    try:
                        line = ser.readline().decode().strip()
                        if line:
                            print(f"Battery voltage: {line} V")
                        else:
                            print("Requested battery voltage; waiting for response...")
                    except Exception:
                        print("Requested battery voltage.")

            elif key == 'd':
                # Save all data to CSV and optionally create plots
                # Ensure base 'runs' and subfolders exist
                try:
                    os.makedirs('runs', exist_ok=True)
                    csv_dir = os.path.join('runs', 'csvs')
                    plots_dir = os.path.join('runs', 'plots')
                    os.makedirs(csv_dir, exist_ok=True)
                    os.makedirs(plots_dir, exist_ok=True)
                except Exception as e:
                    print(f"Warning: could not create output directories: {e}")
                
                # Save data to CSV
                for run_name, meta in runs.items():
                    # Skip runs that have no metadata (e.g., created with streaming disabled)
                    if meta is None:
                        print(f"Skipping {run_name}: no data recorded (streaming was OFF).")
                        continue

                    df1 = meta.get('motor_data')
                    df2 = meta.get('obsv_data', None)  # may not exist

                    if df1 is None:
                        print(f"Skipping {run_name}: no motor_data present.")
                        continue

                    # If observer data exists, concat; otherwise just use df1
                    if df2 is not None:
                        df = pd.concat([df1, df2], axis=1)
                    else:
                        df = df1

                    if _control_mode == "effort":
                        run_code = 'E'
                    elif _control_mode == "velocity":
                        run_code = 'V'
                    else:
                        run_code = 'LF'

                    if _driving_mode == "straight":
                        driving_code = "STR"
                    elif _driving_mode == "pivot":
                        driving_code = "PV"
                    else:
                        driving_code = "ARC"

                    base_name = f"{run_name}_{run_code}_{_control_val}_{driving_code}"
                    try:
                        csv_name = os.path.join(csv_dir, base_name + ".csv")
                        # cols = ["_time", "_left_pos", "_right_pos", "_left_vel", "_right_vel", "_obsv_time", "_obsv_sL", "_obsv_sR", "_obsv_psi", "_obsv_psi_dot"]
                        cols = ["_time", "_left_pos", "_right_pos", "_left_vel", "_right_vel", "_obsv_time", "_obsv_left_vel", "_obsv_right_vel", "_obsv_s", "_obsv_yaw"]
                        cols = [c for c in cols if c in df.columns]
                        df[cols].to_csv(csv_name, index=False)
                        print(f"Saved complete motor data to {csv_name}")
                    except Exception as e:
                        print(f"Failed to save CSV for {run_name}: {e}")

                continue
            elif key == 'h':
                # Print helpful prompt
                print(user_prompt)

            # Clear any remaining input
            while msvcrt.kbhit():
                msvcrt.getch()

            
        # If not running a test and queuing tests, start the next test
        elif not running and not streaming and queuing:
            if not queue.is_empty():
                next_test = queue.dequeue()
                mode = next_test[0]
                if mode == 'effort':
                    effort = next_test[1]
                    line = 'e' + eff_to_key(effort)
                    control_mode = 0
                    print(f"Queued next {mode} test: {effort}%")
                else:
                    print(f"Unknown test mode in queue: {mode}")
                    continue
                # Send command to Romi 
                ser.write(line.encode())
                sleep(0.2)
                # Start test
                ser.write(b'r')
                first = True
                running = True
        
        # Handle serial input
        else:
            # Check for bytes waiting
            if ser.in_waiting:
                # print("\r\nWe are reading you loud and clear!")
                if running:
                    if first:
                         # Determine what kind of run to create
                        if control_mode == 0:
                            mode_name = "effort"
                            params = None
                        elif control_mode == 1:
                            mode_name = "velocity"
                            params = {'setpoint': setpoint, 'kp': kp, 'ki': ki}
                        else:
                            mode_name = "line_following"
                            params = {'setpoint': setpoint, 'kp': kp, 'ki': ki, 'k_line': k_line}

                        sample_size = 200    # Default sample size
                        if not stream_expected:
                            sample_size = 0    # No data expected
                        # Create new run for motor data ONLY if we expect streaming from MCU
                        run_count += 1
                        run_name = f'run{run_count}'
                        if stream_expected:
                            runs[run_name] = create_run(control_mode, effort, driving_mode, run_count, sample_size)
                            if params:
                                runs[run_name]['params'] = params
                            print(f"Run {run_count} created in {mode_name} mode (size={sample_size})")
                        else:
                            runs[run_name] = None
                            print(f"Run {run_count} started in {mode_name} mode with streaming DISABLED (no DataFrame).")

                        streaming = True
                        first = False

                    elif streaming:
                        # Read all the available bytes in the UART buffer
                        chunk = ser.read(ser.in_waiting or 1).decode()
                        if chunk:
                            frame_buffer += chunk

                        # For complete <S> ... <E> frames, extract lines
                        while "<S>" in frame_buffer and "<E>" in frame_buffer:

                            start = frame_buffer.find("<S>")
                            end = frame_buffer.find("<E>", start)

                            if end == -1: # if there is no <E> found, break
                                break # incomplete frame

                            # Extract the inside contents
                            frame = frame_buffer[start+3 : end]
                            frame_buffer = frame_buffer[end+3:]  # Remove processed frame

                            # END OF STREAM CHECK
                            if frame == "#END":
                                print("Received end of stream marker from Romi. Hit 'd' to print data.")

                                running = False
                                streaming = False
                                frame_buffer = ""  # Clear buffer
                                sleep(0.2)
                                break

                            # For a normal frame, parse the CSV payload
                            try:
                                idx_str, time_s, left_pos, right_pos, left_vel, right_vel = frame.split(',')
                                idx = int(idx_str)
                            except ValueError:
                                print(f"[Rejected] Bad frame contents: '{frame}'")
                                continue

                            # Print the frame if enabled
                            if print_stream_frames:
                                print(f"{frame}")

                            # Store the values only if we have a DataFrame for this run
                            if stream_expected and runs.get(run_name) is not None:
                                df = runs[run_name]["motor_data"]
                                df.loc[idx, "_time"]      = float(time_s)
                                df.loc[idx, "_left_pos"]  = float(left_pos)
                                df.loc[idx, "_right_pos"] = float(right_pos)
                                df.loc[idx, "_left_vel"]  = float(left_vel)
                                df.loc[idx, "_right_vel"] = float(right_vel)

                else:
                    pass
            else:
                pass

    except KeyboardInterrupt:
        break

if 'ser' in locals() and getattr(ser, 'is_open', False):
    try:
        ser.write(b'k')         # Kill motors
    except Exception:
        pass
    try:
        ser.close()                 # Close port
    except Exception:
        pass
else:
    # If ser isn't available, nothing to close
    pass
print("Program interrupted")