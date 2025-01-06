import matplotlib.pyplot as plt
import numpy as np

# Load position and velocity data
position_data = np.loadtxt('csp_zero_vel_pos01.txt')
velocity_data = np.loadtxt('csp_zero_vel_vel01.txt')

# Extract columns from position data
timestamp_pos = position_data[:, 0]
reference_position = position_data[:, 1]
actual_position = position_data[:, 2]

# Extract columns from velocity data
timestamp_vel = velocity_data[:, 0]
reference_velocity = velocity_data[:, 1]
actual_velocity = velocity_data[:, 2]

# Create a figure with two subplots
plt.figure(figsize=(12, 8))

# Plot position data
plt.subplot(2, 1, 1)
plt.plot(timestamp_pos, reference_position, label='Reference Position', linestyle='-', color='blue')
plt.plot(timestamp_pos, actual_position, label='Actual Position', linestyle='--', color='red')
plt.title('Position Data')
plt.xlabel('Timestamp (s)')
plt.ylabel('Position')
plt.legend()
plt.grid(True)

# Plot velocity data
plt.subplot(2, 1, 2)
plt.plot(timestamp_vel, reference_velocity, label='Reference Velocity', linestyle='-', color='green')
plt.plot(timestamp_vel, actual_velocity, label='Actual Velocity', linestyle='--', color='orange')
plt.title('Velocity Data')
plt.xlabel('Timestamp (s)')
plt.ylabel('Velocity')
plt.legend()
plt.grid(True)

# Adjust layout and show the plots
plt.tight_layout()
plt.show()
