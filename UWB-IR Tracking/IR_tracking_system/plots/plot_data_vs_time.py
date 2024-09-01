import matplotlib.pyplot as plt
import pandas as pd

# Read the data from the CSV file
csv_filename = 'x position vs time.csv'
data = pd.read_csv(csv_filename)

# Create subplots for x and y positions vs. time
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# Plot x positions vs. time
ax1.plot(data['Time'], data['x_world_smoothed'], 'b', label='IR X Position', alpha=0.6)
# ax1.plot(data['Time'], data['LPF_X'], 'm-', label='Low-Pass Filtered X Position', alpha=0.6)
# ax1.plot(data['Time'], data['EKF_X'], 'b-', label='EKF Filtered X Position', alpha=0.6)
ax1.set_ylabel('X Position')
# ax1.legend()
ax1.grid(True)

# Plot y positions vs. time
ax2.plot(data['Time'], data['y_world_smoothed'] + 100, 'b', label='R Y Position', alpha=0.6)
# ax2.plot(data['Time'], data['LPF_Y'], 'm-', label='Low-Pass Filtered Y Position', alpha=0.6)
# ax2.plot(data['Time'], data['EKF_Y'], 'b-', label='EKF Filtered Y Position', alpha=0.6)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Y Position')
# ax2.legend()
ax2.grid(True)

# Plot z positions vs. time
ax3.plot(data['Time'], data['z_world_smoothed'], 'b', label='IR Z Position', alpha=0.6)
# ax2.plot(data['Time'], data['LPF_Y'], 'm-', label='Low-Pass Filtered Y Position', alpha=0.6)
# ax2.plot(data['Time'], data['EKF_Y'], 'b-', label='EKF Filtered Y Position', alpha=0.6)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Z Position')
# ax3.legend()
ax3.grid(True)
# Set the main title
plt.suptitle('X and Y Positions vs. Time')

# Show the plot
plt.show()
