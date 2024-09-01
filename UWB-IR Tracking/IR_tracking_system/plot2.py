import csv
import matplotlib.pyplot as plt
import numpy as np

# Read data from CSV file
filename = 'recorded_data.csv'
time_data = []
x_world_data = []
y_world_data = []
z_world_data = []
x_world_smoothed = []
y_world_smoothed = []
z_world_smoothed = []

with open(filename, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)  # Skip header row
    for row in csvreader:
        time_data.append(float(row[0]))
        x_world_data.append(float(row[1]))
        y_world_data.append(float(row[2]))
        z_world_data.append(float(row[3]))
        x_world_smoothed.append(float(row[4]))
        y_world_smoothed.append(float(row[5]))
        z_world_smoothed.append(float(row[6]))

# Skip the first 150 points
points=0
time_data = time_data[points:]
x_world_data = np.array(x_world_data[points:]) - 2.5
y_world_data = np.array(y_world_data[points:]) - 2
z_world_data = np.array(z_world_data[points:])
x_world_smoothed = np.array(x_world_smoothed[points:])- 2.5
y_world_smoothed = np.array(y_world_smoothed[points:]) - 2 
z_world_smoothed = np.array(z_world_smoothed[points:])

# Create a figure with three subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(15, 7), sharex=True)

# Plot for X Position
ax1.plot(time_data, x_world_data, linestyle='-', color='b', label='Raw X Data')
ax1.plot(time_data, x_world_smoothed, linestyle='-', color='r', label='Smoothed X Data')
ax1.axhline(y=100, color='black', linestyle='-', linewidth=2, label='True X distance')
ax1.set_ylabel('X Position (cm)')
ax1.set_ylim(95, 110)
ax1.set_title('3D Position vs. Time')
ax1.legend()
# ax1.grid()

# Plot for Y Position
ax2.plot(time_data, y_world_data, linestyle='-', color='b')
ax2.plot(time_data, y_world_smoothed, linestyle='-', color='r')
ax2.axhline(y=20, color='black', linestyle='-', linewidth=2)
ax2.set_ylabel('Y Position (cm)')
ax2.set_ylim(18, 22)
# ax2.set_title('Y Position vs. Time')
# ax2.legend()
# ax2.grid()

# Plot for Z Position
ax3.plot(time_data, z_world_data, linestyle='-', color='b')
ax3.plot(time_data, z_world_smoothed, linestyle='-', color='r')
ax3.axhline(y=5.5, color='black', linestyle='-', linewidth=2)

ax3.set_ylabel('Z Position (cm)')
ax3.set_xlabel('Time (s)')
ax3.set_ylim(5, 6)
ax3.set_xlim(0, 250)

# ax3.set_title('Z Position vs. Time')
# ax3.legend()
# ax3.grid()

plt.tight_layout()
plt.show()