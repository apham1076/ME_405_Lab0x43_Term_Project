# Script to run on PC to run motor characterization test and perform data collection

# Last modified: 10-25-25 7:30pm

# # ************* NOTES **************

# ************* TO-DO **************
# Debug driving mode
# Plot data for several runs
# Figure out why data is so jaggedy (message Charlie on Piazza)
# Debug data streaming: use START and ACK


from serial import Serial, SerialException
from time import sleep
import matplotlib.pyplot as plt
import msvcrt
import pandas as pd
import numpy as np
import os

key = ''                    # Stores value of key pressed as a string
running = False             # Set when motor testing is progress
streaming = False           # Set when data streaming is in progress
runs = {}                   # Dict to contain runs
run_count = 0               # Number of runs
override = False            # Set if user wants to overwrite existing run
_continue = False           # Second option, if user doesn't want to overwrite
question = False           # Set when option to override is presented
effort_mode = True         # True for effort mode, False for velocity mode
current_setpoint = 0       # Current velocity setpoint in ticks/second
first = True
done = False
mode = 1                    # 1, 2, 3 = straight, pivot, arc

user_prompt = '''\r\nCommand keys:
    e      : Toggle between Effort mode and Velocity mode
    0-9,a  : Set effort (0-100%) for open-loop control [Effort mode]
    t      : Set velocity setpoint (ticks/s) for closed-loop control [Velocity mode]
    p      : Set proportional gain (Kp) for controller [Velocity mode]
    i      : Set integral gain (Ki) for controller [Velocity mode]
    g      : GO (start test)
    k      : Kill (stop) motors
    r      : Run automated sequence (0 to 100% by 10%)
    x      : Plot all runs' left velocity
    s      : Stream data
    ----  o      : Override existing run
    m      : Toggle mode (Set to straight by default)
    d      : Print CSV data to Terminal
    h      : Help / show this menu
    ctrl-c : Interrupt this program\r\n'''

# Function to create dictionary for storing data from one run
def create_run(control_val, size):
    time = np.zeros(size)
    p1 = np.zeros(size)
    p2 = np.zeros(size)
    v1 = np.zeros(size)
    v2 = np.zeros(size)
    ctrl = np.zeros(size)  # Store control value (effort or setpoint)

    df = pd.DataFrame({
        "_time": time,
        "_control": ctrl,
        "_left_pos": p1,
        "_right_pos": p2,
        "_left_vel": v1,
        "_right_vel": v2
    })

    return {"control_val": control_val, "size": size, "motor_data": df}

# Function to check if run has been created for a particular effort value
def has_eff(runs, eff_value):
    for run in runs.values():
        if run["eff"] == eff_value:
            return True    
    return False


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


# Map an effort percentage (0-100) to the single-character key expected by the Romi
def eff_to_key(eff):
    if eff == 100:
        return 'a'
    # assume efforts are multiples of 10
    digit = int(eff // 10)
    return str(digit)


# Automated run sequence: for each effort in 'efforts' send the effort key and 'g' to start
# then wait for the Romi to send a 'q' (test done), request streaming ('s'), and collect the
# streamed data into the global `runs` dict (same format as interactive streaming).
def auto_run_sequence(efforts):
    global ser, runs, run_count
    try:
        for eff in efforts:
            key = eff_to_key(eff)
            print(f"Starting automated test for {eff}% (key='{key}')")
            # set effort
            # clear any stale input, then write the key. For 100% ('a') give the Romi more time to process.
            try:
                # pyserial method to clear input buffer
                ser.reset_input_buffer()
            except Exception:
                pass
            ser.write(key.encode())
            if key == 'a':
                sleep(0.5)
            else:
                sleep(0.2)
            # start test
            ser.write(b'g')
            print("Test started, waiting for completion signal from Romi...")

            # wait for 'q' from Romi that indicates the test is done
            while True:
                if ser.in_waiting:
                    ch = ser.read().decode(errors='ignore')
                    if ch == 'q':
                        print("Test complete signal received")
                        break
                else:
                    sleep(0.05)

            # request streaming
            ser.write(b's')
            sleep(0.1)

            # Read header line with effort and size
            header = ser.readline().decode().strip()
            try:
                eff_recv, size = header.split(',')
                eff_recv = int(eff_recv)
                size = int(size)
            except Exception as e:
                print(f"Failed to parse header '{header}': {e}")
                continue

            run_count += 1
            run_name = f'run{run_count}'
            runs[run_name] = create_run(eff_recv, size)
            print(f"Created {run_name} (eff={eff_recv}, size={size})")

            line_num = 0
            while line_num <= size - 10:
                line = ser.readline().decode().strip()
                if not line:
                    continue
                spam = line.split(',')
                try:
                    time_s, left_pos, right_pos, left_vel, right_vel = spam
                    runs[run_name]["motor_data"].loc[line_num, "_time"] = float(time_s)
                    runs[run_name]["motor_data"].loc[line_num, "_left_pos"] = float(left_pos)
                    runs[run_name]["motor_data"].loc[line_num, "_right_pos"] = float(right_pos)
                    runs[run_name]["motor_data"].loc[line_num, "_left_vel"] = float(left_vel)
                    runs[run_name]["motor_data"].loc[line_num, "_right_vel"] = float(right_vel)
                    line_num += 1
                except ValueError:
                    print(f"Line {line_num} rejected. Contents: {spam}")
                    line_num += 1

            print(f"Finished streaming for {run_name}")
            
            if run_count == 10:
                print(f"Automated test is finished ({run_count} runs completed). Hit 'x' to plot data.")



    except KeyboardInterrupt:
        print("\nAutomated sequence interrupted by user (Ctrl-C). Sending kill and closing serial port.")
        try:
            if ser and hasattr(ser, 'is_open') and ser.is_open:
                try:
                    ser.write(b'k')
                except Exception:
                    pass
                try:
                    ser.close()
                except Exception as e:
                    print(f"Error closing serial port: {e}")
                else:
                    print("Serial port closed.")
        finally:
            return



# Create 'runs' directory if it doesn't exist
try:
    os.makedirs('runs', exist_ok=True)
    # print("Output directory 'runs' is ready")
except Exception as e:
    print(f"Warning: Could not create 'runs' directory: {e}")

# Establish Bluetooth connection
try:
    ser = Serial('COM8', baudrate=115200, timeout=1)
except SerialException:
    print("Unable to connect to port")

# ser = Serial('COM8', baudrate=115200, timeout=1)

print("Begin Program:")
print(user_prompt)

while True:
    try:
        # Check for key pressed
        if msvcrt.kbhit():
            key = msvcrt.getch().decode()
            # print(f"{key} was pressed")
            if key.isdigit() or key == 'a':
                if running:
                    print("Cannot set effort at this time. Try again when test is finished")
                elif streaming:
                    print("Cannot set effort at this time. Data streaming in progress")
                else:
                    ser.write(key.encode())
                    if key == 'a':
                        print(f"Effort value set to 100%. Enter 'g' to begin test.")
                    else:
                        print(f"Effort value set to {key}0%. Enter 'g' to begin test.")
            elif key == 'g':
                if running:
                    print("Test is already running")
                elif streaming:
                    print("Data is streaming.")
                else:
                    print("Starting test...")
                    running = True
                    ser.write(b'g')
            elif key == 'k':
                if running:
                    # Kill motors
                    ser.write(b'k')
                    print("End test. Stop motors")
                else:
                    print("Motors are already off")
            elif key == 's':
                if running:
                    print("Test is already running, cannot stream data now.")
                elif streaming:
                    print("Data is already streaming.")
                else:
                    # Tell Romi to stream data to PC
                    ser.write(b's')

                    # Flush serial port
                    if ser.in_waiting:
                        ser.read()

                    print("Data streaming to PC...")
                    streaming = True

                # elif not question:
                #     # Check if run has already been made
                #     if has_eff(runs, eff):
                #             print(f"A run has already been created for an effort value of {eff}")
                #             print("Do you want to override it? Override: 'o', Do nothing: 'p'")
                #             question = True
                #     else:
                #         # Create the dict, no run with this effort has been made yet
                #         run_count += 1
                #         run_name = f'run{run_count}'
                #         runs[run_name] = create_run(eff, size)
            
            elif key == 'm':
                if not running and not streaming:
                    print("Toggle mode")
                    ser.write(b'm')
                    if mode == 1:
                        print("Romi will drive in a straight line.")
                        mode = 2
                    elif mode == 2:
                        print("Romi will pivot in place.")
                        mode = 3
                    else:
                        mode = 1
                        print("Romi will follow an arc.")

            elif key == 'e':
                if running:
                    print("Cannot change control mode while test is running")
                elif streaming:
                    print("Cannot change control mode while streaming")
                else:
                    ser.write(b'e')
                    effort_mode = not effort_mode
                    mode_str = "effort" if effort_mode else "velocity"
                    print(f"Switched to {mode_str} control mode")

            elif key == 't':
                if running:
                    print("Cannot set velocity setpoint while test is running")
                elif streaming:
                    print("Cannot set velocity setpoint while streaming")
                elif effort_mode:
                    print("Must be in velocity mode to set setpoint")
                else:
                    try:
                        # Get setpoint from user
                        setpoint = input("Enter velocity setpoint (ticks/s): ")
                        setpoint = int(setpoint)
                        # Format: 'pXXXX' for positive, 'nXXXX' for negative
                        cmd = 'p' if setpoint >= 0 else 'n'  # 'p' for positive, 'n' for negative
                        cmd += f"{abs(setpoint):04d}"  # Always send magnitude as 4 digits
                        ser.write(cmd.encode())
                        current_setpoint = setpoint
                        sign_str = "+" if setpoint >= 0 else "-"
                        print(f"Velocity setpoint set to {sign_str}{abs(setpoint)} ticks/s")
                    except ValueError:
                        print("Invalid input. Please enter an integer.")

            elif key == 'i':
                if running:
                    print("Cannot set Ki while test is running")
                elif streaming:
                    print("Cannot set Ki while streaming")
                else:
                    try:
                        # Get Ki from user
                        ki = input("Enter integral gain (Ki): ")
                        ki = float(ki)
                        # Send Ki to Romi - format: 'iXXXX' where XXXX is Ki*100
                        ki_int = int(ki * 100)  # Scale up by 100 to send as integer
                        cmd = f"i{abs(ki_int):04d}"
                        ser.write(cmd.encode())
                        print(f"Integral gain set to {ki}")
                    except ValueError:
                        print("Invalid input. Please enter a number.")

            elif key == 'p':
                if running:
                    print("Cannot set Kp while test is running")
                elif streaming:
                    print("Cannot set Kp while streaming")
                else:
                    try:
                        # Get Kp from user
                        kp = input("Enter proportional gain (Kp): ")
                        kp = float(kp)
                        # Send Kp to Romi - format: 'pXXXX' where XXXX is Kp*100
                        kp_int = int(kp * 100)  # Scale up by 100 to send as integer
                        cmd = f"p{abs(kp_int):04d}"
                        ser.write(cmd.encode())
                        print(f"Proportional gain set to {kp}")
                    except ValueError:
                        print("Invalid input. Please enter a number.")

            elif key == 'd':
                # Plot and save the latest run's motor_data to CSV
                if run_count == 0:
                    print("No runs available to display or save.")
                else:
                    run_name = f'run{run_count}'
                    df = runs[run_name]["motor_data"]
                    # Clean data: remove zeros except leading zeros
                    df_clean, removed = clean_data(df, mode='all')
                    if removed:
                        print(f"Removed {removed} all-zero rows from {run_name} before plotting/saving")
                    x = df_clean["_time"]
                    y = df_clean["_left_vel"]
                    plt.plot(x, y)
                    plt.xlabel("Time")
                    plt.ylabel("Left velocity")
                    eff_val = runs[run_name].get('eff', runs[run_name].get('eff:', 'unknown'))
                    # instead of plt.show()
                    plt.savefig(f"runs/{run_name}_effort_{eff_val}_left_vel.png")
                    plt.close()

                    # Save motor_data to CSV inside the 'runs' folder. Try to get effort value from either 'eff' or 'eff:' key.
                    # try:
                    #     os.makedirs('runs', exist_ok=True)
                    #     filename = os.path.join('runs', f"{run_name}_eff{eff_val}.csv")
                    #     df_clean.to_csv(filename, index=False)
                    #     print(f"Saved motor_data to {filename}")
                    # except Exception as e:
                    #     print(f"Failed to save CSV: {e}")
                    
                    filename = os.path.join('runs', f"{run_name}_eff{eff_val}.csv")
                    df_clean.to_csv(filename, index=False)
                    print(f"Saved motor_data to {filename}")
                    print(user_prompt)

            elif key == 'r':
                # Run automated sequence for efforts 0..100 step 10
                print("Starting automated batch: efforts 0 to 100 by 10")
                efforts = list(range(0, 91, 10))
                auto_run_sequence(efforts)

            elif key == 'x':
                # Plot all runs at once
                if not runs:
                    print("No runs available to plot.")
                else:
                    plt.figure()
                    for run_name, meta in runs.items():
                        df = meta["motor_data"]
                        # Clean data: remove zeros except leading zeros
                        df_clean, removed = clean_data(df, mode='all')
                        if removed:
                            print(f"Removed {removed} all-zero rows from {run_name} before plotting")
                            
                        mode = meta.get('mode', 'E')  # Default to effort mode for backward compatibility
                        control_val = meta.get('control_val', 'unknown')
                        
                        if mode == 'V':  # Velocity mode
                            kp = meta.get('params', {}).get('kp', 0)
                            ki = meta.get('params', {}).get('ki', 0)
                            label = f"{run_name} (V) sp={control_val} Kp={kp:.2f} Ki={ki:.2f}"
                        else:  # Effort mode
                            label = f"{run_name} (E) eff={control_val}"
                            
                        plt.plot(df_clean["_time"], df_clean["_left_vel"], label=label)
                        
                    plt.xlabel("Time")
                    plt.ylabel("Left velocity")
                    plt.legend()
                    # instead of plt.show()
                    plt.savefig(f"runs/{run_name}_velocity_response.png")
                    plt.close()
            
            elif key == 'h':
                # Print helpful prompt
                print(user_prompt)
            else:
                print(f"Unknown command '{key}'. Press 'h' for help.")

            # Clear any remaining input
            if ser.in_waiting:  
                ser.read()
        
        else:
            # Check for bytes waiting
            if ser.in_waiting:
                # print("\r\nWe are reading you loud and clear!")
                if running:
                    ch = ser.read().decode()
                    if ch == 'q':
                        print("Testing is done. Press 's' to stream data")
                        print(user_prompt)
                        running = False
                elif streaming:
                    # print("ve r streeming")
                    if first:
                        # Read and parse header line
                        header = ser.readline().decode().strip().split(",")
                        mode = header[0]  # 'E' for effort, 'V' for velocity
                        
                        if mode == 'E':
                            control_val = int(header[1])  # effort value
                            size = int(header[2])
                            params = None
                        else:  # Velocity mode
                            control_val = float(header[1])  # setpoint value
                            kp = float(header[2])
                            ki = float(header[3])
                            size = int(header[4])
                            params = {'kp': kp, 'ki': ki}
                        
                        # Create new run
                        run_count += 1
                        run_name = f'run{run_count}'
                        runs[run_name] = create_run(control_val, size)
                        if params:
                            runs[run_name]['params'] = params
                        runs[run_name]['mode'] = mode
                        print(f"Run {run_name} created in {mode} mode")
                        # print(size)
                        line_num = 0
                        first = False
                    # elif done:
                    #     first = True
                    #     streaming = False
                    #     done = False
                    else:
                        if line_num <= size - 4:
                            # Read line by line
                            try:
                                spam = ser.readline().decode().strip().split(",")
                                time, control, left_pos, right_pos, left_vel, right_vel = spam

                                runs[run_name]["motor_data"].loc[line_num,"_time"] = float(time)
                                runs[run_name]["motor_data"].loc[line_num,"_control"] = float(control)
                                runs[run_name]["motor_data"].loc[line_num, "_left_pos"] = float(left_pos)
                                runs[run_name]["motor_data"].loc[line_num, "_right_pos"] = float(right_pos)
                                runs[run_name]["motor_data"].loc[line_num, "_left_vel"] = float(left_vel)
                                runs[run_name]["motor_data"].loc[line_num, "_right_vel"] = float(right_vel)
                                line_num += 1
                                # print(line_num)
                            except ValueError:
                                print(f"Line {line_num} rejected. Contents: {spam}")
                                line_num +=1

                        else:
                            print("Data streaming complete. Hit 'd' to print data.")
                            first = True
                            streaming = False
                            sleep(0.5)
                            print(user_prompt)
                else:
                    pass

                    # if not question:
                    #     # Ask user if want to override existing run
                    #     if has_eff(runs, eff):
                    #         print(f"A run has already been created for an effort value of {eff}")
                    #         print("Do you want to override it? Override: 'o', Do nothing: 'p'")
                    #         question = True
                    #     else:
                    #         # Create the dict, no run with this effort has been made yet
                    #         run_count += 1
                    #         run_name = f'run{run_count}'
                    #         runs[run_name] = create_run(eff, size)
                    # else:
                    #     # Create dict object to store data for run
                    #     # First, check if run with this effort has already been created
                    #     if override:
                    #         # Delete run and create new dictionary, do not need to change run_count
                    #         print(f"Run {run_count} was overwritten.") 
                    #         run_name = f'run{run_count}'
                    #         del runs[run_name]
                    #     elif _continue:
                    #         # Do nothing, prompt user with help menu again
                    #         print(user_prompt)

                    #     streaming = False
                    #     override = False
                    #     _continue = False
                    #     question = False
                    #     pass

            else:
                pass

    except KeyboardInterrupt:
        break

if ser.is_open:
    ser.write(b'k')         # Kill motors
ser.close()                 # Close port
print("Program interrupted")