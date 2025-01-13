import matplotlib.pyplot as plt
import numpy as np

# Load position and velocity data
# position_data = np.loadtxt('csp_zero_vel/csp_zero_vel_pos03.txt')
# velocity_data = np.loadtxt('csp_zero_vel/csp_zero_vel_vel03.txt')
# accel_data = np.loadtxt('csp_zero_vel/csp_zero_vel_acc03.txt')
# jerk_data = np.loadtxt('csp_zero_vel/csp_zero_vel_jerk03.txt')

# position_data = np.loadtxt('csv_prev_vel/csv_prev_vel_pos02.txt')
# velocity_data = np.loadtxt('csv_prev_vel/csv_prev_vel_vel02.txt')
# accel_data = np.loadtxt('csv_prev_vel/csv_prev_vel_acc02.txt')
# jerk_data = np.loadtxt('csv_prev_vel/csv_prev_vel_jerk02.txt')

position_data = np.loadtxt('csp_predict/csp_predict_pos02.txt')
velocity_data = np.loadtxt('csp_predict/csp_predict_vel02.txt')
accel_data = np.loadtxt('csp_predict/csp_predict_acc02.txt')
jerk_data = np.loadtxt('csp_predict/csp_predict_jerk02.txt')


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

# Extract jerk data
timestamp_jerk = jerk_data[:, 0]
reference_jerk = jerk_data[:, 1]

# Create a figure with two subplots
plt.figure(figsize=(12, 8))

# Plot position data
plt.subplot(4, 1, 1)
plt.plot(timestamp_pos, reference_position, label='Reference Position', linestyle='-', color='blue')
plt.plot(timestamp_pos, actual_position, label='Actual Position', linestyle='--', color='red')
plt.title('Position Data')
plt.xlabel('Timestamp (s)')
plt.ylabel('Position')
plt.xlim(0, 100)
plt.ylim(-10, 30)
plt.legend()
plt.grid(True)

# Plot velocity data
plt.subplot(4, 1, 2)
plt.plot(timestamp_vel, reference_velocity, label='Reference Velocity', linestyle='-', color='green')
plt.plot(timestamp_vel, actual_velocity, label='Actual Velocity', linestyle='--', color='orange')
plt.title('Velocity Data')
plt.xlabel('Timestamp (s)')
plt.ylabel('Velocity')
plt.xlim(0, 100)
plt.ylim(-2, 2)
plt.legend()
plt.grid(True)

# Plot accel data
plt.subplot(4, 1, 3)
plt.plot(timestamp_accel, reference_accel, label='Reference Acceleration', linestyle='-', color='purple')
plt.title('Acceleration Data')
plt.xlabel('Timestamp (s)')
plt.ylabel('Acceleration [rpm/s]')
plt.xlim(0, 100)
plt.ylim(-6, 6)
plt.legend(loc='upper right')  # Legend font size
plt.grid(True)
plt.tick_params(axis='both', which='major')

# Plot jerk data
plt.subplot(4, 1, 4)
plt.plot(timestamp_jerk, reference_jerk, label='Reference Jerk', linestyle='-', color='orange')
plt.title('Jerk Data')
plt.xlabel('Timestamp (s)')
plt.ylabel('Jerk [rpm/s²]')
plt.xlim(-60, 60)
# plt.ylim(-500, 300)
plt.legend(loc='upper right')  # Legend font size
plt.grid(True)
plt.tick_params(axis='both', which='major')

# Adjust layout and show the plots
plt.tight_layout()
plt.show()