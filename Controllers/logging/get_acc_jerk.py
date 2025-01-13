import os
import pandas as pd
from scipy.signal import butter, filtfilt

def low_pass_filter(data, cutoff, fs, order=4):
    nyquist = 0.5 * fs  # Nyquist frequency
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def ensure_directory_exists(file_path):
    """Ensure the directory for the file path exists."""
    directory = os.path.dirname(file_path)
    if directory and not os.path.exists(directory):
        os.makedirs(directory)

def process_velocity_data(velocity_file, acceleration_file, jerk_file, cutoff_freq, sampling_rate):
    # Load velocity data
    velocity_data = pd.read_csv(velocity_file, sep=" ", header=None, names=["Time", "Value1", "Velocity"])

    # 1. Calculate actual acceleration
    velocity_data['Acceleration'] = velocity_data['Velocity'].diff() / velocity_data['Time'].diff()

    # 2. Apply LPF to actual acceleration
    velocity_data['Filtered_Acceleration'] = low_pass_filter(
        velocity_data['Acceleration'].fillna(0), cutoff_freq, sampling_rate
    )

    # 3. Calculate actual jerk using filtered acceleration
    velocity_data['Jerk'] = velocity_data['Filtered_Acceleration'].diff() / velocity_data['Time'].diff()

    # 4. Apply LPF to actual jerk
    velocity_data['Filtered_Jerk'] = low_pass_filter(
        velocity_data['Jerk'].fillna(0), cutoff_freq, sampling_rate
    )

    # Load acceleration file and update the 3rd column
    acceleration_data = pd.read_csv(acceleration_file, sep=" ", header=None, names=["Time", "Value1", "Acceleration"])
    acceleration_data['Acceleration'] = velocity_data['Filtered_Acceleration'][:len(acceleration_data)].fillna(0).values

    # Save updated acceleration file
    updated_acceleration_file = acceleration_file
    ensure_directory_exists(updated_acceleration_file)
    acceleration_data.to_csv(updated_acceleration_file, sep=" ", header=False, index=False)

    # Load jerk file and update the 3rd column
    jerk_data = pd.read_csv(jerk_file, sep=" ", header=None, names=["Time", "Value1", "Jerk"])
    jerk_data['Jerk'] = velocity_data['Filtered_Jerk'][:len(jerk_data)].fillna(0).values

    # Save updated jerk file
    updated_jerk_file = jerk_file
    ensure_directory_exists(updated_jerk_file)
    jerk_data.to_csv(updated_jerk_file, sep=" ", header=False, index=False)

    return updated_acceleration_file, updated_jerk_file

# Example usage
if __name__ == "__main__":

    dir = "csp_zero_vel/"
    velocity_file = dir + "csp_zero_vel_vel03.txt"
    acceleration_file = dir + "csp_zero_vel_acc03.txt"
    jerk_file = dir + "csp_zero_vel_jerk03.txt"

    # dir = "csv_prev_vel/"
    # velocity_file = dir + "csv_prev_vel_vel02.txt"
    # acceleration_file = dir + "csv_prev_vel_acc02.txt"
    # jerk_file = dir + "csv_prev_vel_jerk02.txt"

    # dir = "csp_predict/"
    # velocity_file = dir + "csp_predict_vel02.txt"
    # acceleration_file = dir + "csp_predict_acc02.txt"
    # jerk_file = dir + "csp_predict_jerk02.txt"

    cutoff_freq = 5  # Hz
    sampling_rate = 1000  # Hz

    updated_acc_file, updated_jerk_file = process_velocity_data(
        velocity_file, acceleration_file, jerk_file, cutoff_freq, sampling_rate
    )

    print(f"Updated acceleration file saved as: {updated_acc_file}")
    print(f"Updated jerk file saved as: {updated_jerk_file}")
