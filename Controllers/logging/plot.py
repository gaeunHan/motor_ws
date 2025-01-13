import matplotlib.pyplot as plt
import numpy as np

# Load position and velocity data
# position_data = np.loadtxt('csp_zero_vel/csp_zero_vel_pos02.txt')
# velocity_data = np.loadtxt('csp_zero_vel/csp_zero_vel_vel02.txt')
# accel_data = np.loadtxt('csp_zero_vel/csp_zero_vel_acc02.txt')
# jerk_data = np.loadtxt('csp_zero_vel/csp_zero_vel_jerk02.txt')

# position_data = np.loadtxt('csv_prev_vel/csv_prev_vel_pos01.txt')
# velocity_data = np.loadtxt('csv_prev_vel/csv_prev_vel_vel01.txt')
# accel_data = np.loadtxt('csv_prev_vel/csv_prev_vel_acc01.txt')
# jerk_data = np.loadtxt('csv_prev_vel/csv_prev_vel_jerk01.txt')

position_data = np.loadtxt('csp_predict/csp_predict_pos01.txt')
velocity_data = np.loadtxt('csp_predict/csp_predict_vel01.txt')
accel_data = np.loadtxt('csp_predict/csp_predict_acc01.txt')
jerk_data = np.loadtxt('csp_predict/csp_predict_jerk01.txt')

# Extract columns from position data
timestamp_pos = position_data[:, 0]
reference_position = position_data[:, 1]
actual_position = position_data[:, 2]

# Extract columns from velocity data
timestamp_vel = velocity_data[:, 0]
reference_velocity = velocity_data[:, 1]
actual_velocity = velocity_data[:, 2]

# Extract accel data
timestamp_accel = accel_data[:, 0]
reference_accel = accel_data[:, 1]
actual_accel = accel_data[:, 2]

# Extract jerk data
timestamp_jerk = jerk_data[:, 0]
reference_jerk = jerk_data[:, 1]
actual_jerk = jerk_data[:, 2]

# Calculate errors
position_error = reference_position - actual_position
velocity_error = reference_velocity - actual_velocity
accel_error = reference_accel - actual_accel
jerk_error = reference_jerk - actual_jerk

# Find maximum values for position error data
position_mean_error = np.mean(position_error)
position_max_error = position_error[np.argmax(np.abs(position_error))]

# Find maximum values for jerk data (absolute value based)
jerk_max_reference = reference_jerk[np.argmax(np.abs(reference_jerk))]
jerk_max_actual = actual_jerk[np.argmax(np.abs(actual_jerk))]

# Print calculated results
print("Position error data: Mean =", position_mean_error, ", Max =", position_max_error)
print("Jerk data: Max Reference Value =", jerk_max_reference, ", Max Actual Value =", jerk_max_actual)

# Create a figure with a (4, 2) layout
plt.figure(figsize=(16, 12))

# Plot position data
plt.subplot(4, 2, 1)
plt.plot(timestamp_pos, reference_position, label='Reference Position', linestyle='-', color='blue')
plt.plot(timestamp_pos, actual_position, label='Actual Position', linestyle='--', color='cyan')
plt.title('Position Data', fontsize=32)  # Title font size
plt.xlabel('Timestamp (s)', fontsize=28)  # X-axis label font size
plt.ylabel('Position [deg]', fontsize=28)  # Y-axis label font size
plt.xlim(0, 10)
plt.ylim(-10, 50)
plt.legend(fontsize=20, loc='upper right')  # Legend font size
plt.grid(True)
plt.tick_params(axis='both', which='major', labelsize=24)  # Tick label font size

# Plot position error
plt.subplot(4, 2, 2)
plt.plot(timestamp_pos, position_error, label='Position Error', linestyle='-', color='red')
plt.title('Position Error', fontsize=32)
plt.xlabel('Timestamp (s)', fontsize=28)
plt.ylabel('Error [deg]', fontsize=28)
plt.xlim(0, 10)
plt.ylim(-0.5, 0.5)
plt.grid(True)
plt.tick_params(axis='both', which='major', labelsize=24)

# Plot velocity data
plt.subplot(4, 2, 3)
plt.plot(timestamp_vel, reference_velocity, label='Reference Velocity', linestyle='-', color='green')
plt.plot(timestamp_vel, actual_velocity, label='Actual Velocity', linestyle='--', color='lime')
plt.title('Velocity Data', fontsize=32)
plt.xlabel('Timestamp (s)', fontsize=28)
plt.ylabel('Velocity [rpm]', fontsize=28)
plt.xlim(0, 10)
plt.ylim(-15, 20)
plt.legend(fontsize=20, loc='upper right')  # Legend font size
plt.grid(True)
plt.tick_params(axis='both', which='major', labelsize=24)

# Plot velocity error
plt.subplot(4, 2, 4)
plt.plot(timestamp_vel, velocity_error, label='Velocity Error', linestyle='-', color='red')
plt.title('Velocity Error', fontsize=32)
plt.xlabel('Timestamp (s)', fontsize=28)
plt.ylabel('Error [rpm]', fontsize=28)
plt.xlim(0, 10)
plt.ylim(-3, 3)
plt.grid(True)
plt.tick_params(axis='both', which='major', labelsize=24)

# Plot accel data
plt.subplot(4, 2, 5)
plt.plot(timestamp_accel, reference_accel, label='Reference Acceleration', linestyle='-', color='purple')
plt.plot(timestamp_accel, actual_accel, label='Actual Acceleration', linestyle='--', color='plum')
plt.title('Acceleration Data', fontsize=32)
plt.xlabel('Timestamp (s)', fontsize=28)
plt.ylabel('Acceleration [rpm/s]', fontsize=28)
plt.xlim(0, 10)
plt.ylim(-50, 50)
plt.legend(fontsize=20, loc='upper right')  # Legend font size
plt.grid(True)
plt.tick_params(axis='both', which='major', labelsize=24)

# Plot acceleration error
plt.subplot(4, 2, 6)
plt.plot(timestamp_accel, accel_error, label='Acceleration Error', linestyle='-', color='red')
plt.title('Acceleration Error', fontsize=32)
plt.xlabel('Timestamp (s)', fontsize=28)
plt.ylabel('Error [rpm/s]', fontsize=28)
plt.xlim(0, 10)
plt.ylim(-10, 10)
plt.grid(True)
plt.tick_params(axis='both', which='major', labelsize=24)

# Plot jerk data
plt.subplot(4, 2, 7)
plt.plot(timestamp_jerk, reference_jerk, label='Reference Jerk', linestyle='-', color='orange')
plt.plot(timestamp_jerk, actual_jerk, label='Actual Jerk', linestyle='--', color='maroon')
plt.title('Jerk Data', fontsize=32)
plt.xlabel('Timestamp (s)', fontsize=28)
plt.ylabel('Jerk [rpm/s²]', fontsize=28)
plt.xlim(0, 10)
plt.ylim(-500, 300)
plt.legend(fontsize=20, loc='upper right')  # Legend font size
plt.grid(True)
plt.tick_params(axis='both', which='major', labelsize=24)

# Plot jerk error
plt.subplot(4, 2, 8)
plt.plot(timestamp_jerk, jerk_error, label='Jerk Error', linestyle='-', color='red')
plt.title('Jerk Error', fontsize=32)
plt.xlabel('Timestamp (s)', fontsize=28)
plt.ylabel('Error [rpm/s²]', fontsize=28)
plt.xlim(0, 10)
plt.ylim(-300, 300)
plt.grid(True)
plt.tick_params(axis='both', which='major', labelsize=24)

# Adjust layout and show the plots
plt.tight_layout()
plt.show()
