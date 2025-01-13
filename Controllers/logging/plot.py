import matplotlib.pyplot as plt
import numpy as np

# Load position and velocity data
position_data = np.loadtxt('csp_zero_vel/csp_zero_vel_pos03.txt')
velocity_data = np.loadtxt('csp_zero_vel/csp_zero_vel_vel03.txt')
accel_data = np.loadtxt('csp_zero_vel/csp_zero_vel_acc03.txt')
jerk_data = np.loadtxt('csp_zero_vel/csp_zero_vel_jerk03.txt')

# position_data = np.loadtxt('csv_prev_vel/csv_prev_vel_pos02.txt')
# velocity_data = np.loadtxt('csv_prev_vel/csv_prev_vel_vel02.txt')
# accel_data = np.loadtxt('csv_prev_vel/csv_prev_vel_acc02.txt')
# jerk_data = np.loadtxt('csv_prev_vel/csv_prev_vel_jerk02.txt')

# position_data = np.loadtxt('csp_predict/csp_predict_pos02.txt')
# velocity_data = np.loadtxt('csp_predict/csp_predict_vel02.txt')
# accel_data = np.loadtxt('csp_predict/csp_predict_acc02.txt')
# jerk_data = np.loadtxt('csp_predict/csp_predict_jerk02.txt')

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

# position data statistics
position_rmse = np.sqrt(np.mean(position_error**2))
position_max_error = position_error[np.argmax(np.abs(position_error))]

# jerk data statistics
reference_jerk_rms = np.sqrt(np.mean(reference_jerk**2))
reference_jerk_max = reference_jerk[np.argmax(np.abs(reference_jerk))]

# Print calculated results
print("Position error data: RMSE =", position_rmse, ", Max =", position_max_error)
print("Reference Jerk data: RMS =", reference_jerk_rms, ", Max =", reference_jerk_max)

# Create a figure with a (4, 2) layout
plt.figure(figsize=(16, 12))

# title
plt.suptitle('csp_zero_vel', fontsize=36, fontweight='bold')
# plt.suptitle('csv_prev_vel', fontsize=36, fontweight='bold')
# plt.suptitle('csv_predict', fontsize=50, fontweight='bold')

# Plot position data
plt.subplot(2, 2, 1)
plt.plot(timestamp_pos, reference_position, label='Reference Position', linestyle='-', color='blue')
plt.plot(timestamp_pos, actual_position, label='Actual Position', linestyle='--', color='cyan')
plt.title('Position Data', fontsize=50)  # Title font size
plt.xlabel('Timestamp (s)', fontsize=42)  # X-axis label font size
plt.ylabel('Position [deg]', fontsize=42)  # Y-axis label font size
plt.xlim(0, 28)
plt.ylim(-10, 30)
plt.legend(fontsize=30, loc='upper right')  # Legend font size
plt.grid(True)
plt.tick_params(axis='both', which='major', labelsize=38)  # Tick label font size

# Plot position error
plt.subplot(2, 2, 2)
plt.plot(timestamp_pos, position_error, label='Position Error', linestyle='-', color='red')
plt.title('Position Error', fontsize=50)
plt.xlabel('Timestamp (s)', fontsize=42)
plt.ylabel('Error [deg]', fontsize=42)
plt.xlim(0, 28)
plt.ylim(-0.3, 0.3)
plt.grid(True)
plt.tick_params(axis='both', which='major', labelsize=38)

# Plot jerk data
plt.subplot(2, 2, 3)
plt.plot(timestamp_jerk, reference_jerk, label='Reference Jerk', linestyle='-', color='orange')
plt.plot(timestamp_jerk, actual_jerk, label='Actual Jerk', linestyle='--', color='maroon')
plt.title('Jerk Data', fontsize=50)
plt.xlabel('Timestamp (s)', fontsize=42)
plt.ylabel('Jerk [rpm/s²]', fontsize=42)
plt.xlim(0, 28)
plt.ylim(-50, 50)
plt.legend(fontsize=30, loc='upper right')  # Legend font size
plt.grid(True)
plt.tick_params(axis='both', which='major', labelsize=38)

# Plot jerk error
plt.subplot(2, 2, 4)
plt.plot(timestamp_jerk, jerk_error, label='Jerk Error', linestyle='-', color='red')
plt.title('Jerk Error', fontsize=50)
plt.xlabel('Timestamp (s)', fontsize=42)
plt.ylabel('Error [rpm/s²]', fontsize=42)
plt.xlim(0, 28)
plt.ylim(-50, 50)
plt.grid(True)
plt.tick_params(axis='both', which='major', labelsize=38)

# # Plot position data
# plt.subplot(4, 2, 1)
# plt.plot(timestamp_pos, reference_position, label='Reference Position', linestyle='-', color='blue')
# plt.plot(timestamp_pos, actual_position, label='Actual Position', linestyle='--', color='cyan')
# plt.title('Position Data', fontsize=32)  # Title font size
# plt.xlabel('Timestamp (s)', fontsize=28)  # X-axis label font size
# plt.ylabel('Position [deg]', fontsize=28)  # Y-axis label font size
# plt.xlim(0, 28)
# plt.ylim(-10, 30)
# plt.legend(fontsize=20, loc='upper right')  # Legend font size
# plt.grid(True)
# plt.tick_params(axis='both', which='major', labelsize=24)  # Tick label font size

# # Plot position error
# plt.subplot(4, 2, 2)
# plt.plot(timestamp_pos, position_error, label='Position Error', linestyle='-', color='red')
# plt.title('Position Error', fontsize=32)
# plt.xlabel('Timestamp (s)', fontsize=28)
# plt.ylabel('Error [deg]', fontsize=28)
# plt.xlim(0, 28)
# #plt.ylim(-0.5, 0.5)
# plt.grid(True)
# plt.tick_params(axis='both', which='major', labelsize=24)

# # # Plot velocity data
# # plt.subplot(4, 2, 3)
# # plt.plot(timestamp_vel, reference_velocity, label='Reference Velocity', linestyle='-', color='green')
# # plt.plot(timestamp_vel, actual_velocity, label='Actual Velocity', linestyle='--', color='lime')
# # plt.title('Velocity Data', fontsize=32)
# # plt.xlabel('Timestamp (s)', fontsize=28)
# # plt.ylabel('Velocity [rpm]', fontsize=28)
# # plt.xlim(0, 20)
# # plt.ylim(-15, 20)
# # plt.legend(fontsize=20, loc='upper right')  # Legend font size
# # plt.grid(True)
# # plt.tick_params(axis='both', which='major', labelsize=24)

# # # Plot velocity error
# # plt.subplot(4, 2, 4)
# # plt.plot(timestamp_vel, velocity_error, label='Velocity Error', linestyle='-', color='red')
# # plt.title('Velocity Error', fontsize=32)
# # plt.xlabel('Timestamp (s)', fontsize=28)
# # plt.ylabel('Error [rpm]', fontsize=28)
# # plt.xlim(0, 20)
# # plt.ylim(-3, 3)
# # plt.grid(True)
# # plt.tick_params(axis='both', which='major', labelsize=24)

# # # Plot accel data
# # plt.subplot(4, 2, 5)
# # plt.plot(timestamp_accel, reference_accel, label='Reference Acceleration', linestyle='-', color='purple')
# # plt.plot(timestamp_accel, actual_accel, label='Actual Acceleration', linestyle='--', color='plum')
# # plt.title('Acceleration Data', fontsize=32)
# # plt.xlabel('Timestamp (s)', fontsize=28)
# # plt.ylabel('Acceleration [rpm/s]', fontsize=28)
# # plt.xlim(0, 20)
# # plt.ylim(-50, 50)
# # plt.legend(fontsize=20, loc='upper right')  # Legend font size
# # plt.grid(True)
# # plt.tick_params(axis='both', which='major', labelsize=24)

# # # Plot acceleration error
# # plt.subplot(4, 2, 6)
# # plt.plot(timestamp_accel, accel_error, label='Acceleration Error', linestyle='-', color='red')
# # plt.title('Acceleration Error', fontsize=32)
# # plt.xlabel('Timestamp (s)', fontsize=28)
# # plt.ylabel('Error [rpm/s]', fontsize=28)
# # plt.xlim(0, 20)
# # plt.ylim(-10, 10)
# # plt.grid(True)
# # plt.tick_params(axis='both', which='major', labelsize=24)

# # Plot jerk data
# plt.subplot(4, 2, 7)
# plt.plot(timestamp_jerk, reference_jerk, label='Reference Jerk', linestyle='-', color='orange')
# plt.plot(timestamp_jerk, actual_jerk, label='Actual Jerk', linestyle='--', color='maroon')
# plt.title('Jerk Data', fontsize=32)
# plt.xlabel('Timestamp (s)', fontsize=28)
# plt.ylabel('Jerk [rpm/s²]', fontsize=28)
# plt.xlim(0, 28)
# #plt.ylim(-500, 300)
# plt.legend(fontsize=20, loc='upper right')  # Legend font size
# plt.grid(True)
# plt.tick_params(axis='both', which='major', labelsize=24)

# # Plot jerk error
# plt.subplot(4, 2, 8)
# plt.plot(timestamp_jerk, jerk_error, label='Jerk Error', linestyle='-', color='red')
# plt.title('Jerk Error', fontsize=32)
# plt.xlabel('Timestamp (s)', fontsize=28)
# plt.ylabel('Error [rpm/s²]', fontsize=28)
# plt.xlim(0, 28)
# #plt.ylim(-300, 300)
# plt.grid(True)
# plt.tick_params(axis='both', which='major', labelsize=24)

# Adjust layout and show the plots
plt.tight_layout()
plt.show()
