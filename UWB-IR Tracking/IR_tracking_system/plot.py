import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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
points = 200
time_data = time_data[points:]
x_world_data = np.array(x_world_data[points:]) -2.5
y_world_data = np.array(y_world_data[points:])-2.5
z_world_data = z_world_data[points:]
x_world_smoothed = np.array(x_world_smoothed[points:])-2.5
y_world_smoothed = np.array(y_world_smoothed[points:]) -2.5
z_world_smoothed = z_world_smoothed[points:]

# Create a figure for 3D plot
fig1 = plt.figure(figsize=(7, 7))

# Create a 3D plot for raw and smoothed data
ax1 = fig1.add_subplot(111, projection='3d')

# Plot the raw data
ax1.plot(x_world_data, y_world_data, z_world_data, linestyle='-', color='b', label='Raw Data')

# Plot the smoothed data
ax1.plot(x_world_smoothed, y_world_smoothed, z_world_smoothed, linestyle='-', color='r', label='Smoothed Data')

# Add a small cube at (0, 0, 0)
cube_size = 10
cube_x = [0, 0, 0 + cube_size, 0 + cube_size, 0]
cube_y = [0, 0 + cube_size, 0 + cube_size, 0, 0]
cube_z = [0, 0, 0, 0, 0]

for i in range(4):
    ax1.plot([cube_x[i], cube_x[i+1]], [cube_y[i], cube_y[i+1]], [cube_z[i], cube_z[i+1]], color='r')

for i in range(4):
    ax1.plot([cube_x[i], cube_x[i]], [cube_y[i], cube_y[i]], [cube_z[i], cube_z[i] + cube_size], color='r')

for i in range(4):
    ax1.plot([cube_x[i], cube_x[i+1]], [cube_y[i], cube_y[i+1]], [cube_z[i] + cube_size, cube_z[i] + cube_size], color='r')

# Add labels and title for 3D plot
ax1.set_xlabel('X World')
ax1.set_ylabel('Y World')
ax1.set_zlabel('Z World')
ax1.set_title('3D Plot of World Coordinates')
ax1.legend()

# Set plot limits for 3D plot
ax1.set_xlim(0, 250)
ax1.set_ylim(0, 250)
ax1.set_zlim(0, 50)

# Show the 3D plot
plt.show()

############################ Vs Time #####################################
# Create a figure for 2D plot
fig2 = plt.figure(figsize=(7, 7))

# Create a 2D plot for raw and smoothed data
ax2 = fig2.add_subplot(111)

# Plot the raw data
ax2.plot(time_data, x_world_data, linestyle='-', color='b', label='Raw Data')

ax2.plot(time_data, y_world_data, linestyle='-', color='b')
# Plot the smoothed data
ax2.plot(time_data, x_world_smoothed, linestyle='-', color='r', label='Smoothed Data')
ax2.plot(time_data, y_world_smoothed, linestyle='-', color='r')

# Add a horizontal line at 100
ax2.axhline(y=100, color='black', linestyle='-', linewidth=2, label='True distance')
ax2.axhline(y=20, color='black', linestyle='-', linewidth=2, label='True distance')
# Add labels and title for 2D plot
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('X Position (cm)')
ax2.set_title('X Position vs. Time')
ax2.legend()

# Set plot limits for 2D plot
ax2.set_ylim(0, 110)
plt.grid()
# Show the 2D plot
plt.show()

############################## 2D PLot ###################################
# # Create a figure for 2D plot
# fig3 = plt.figure(figsize=(7, 7))

# # Create a 2D plot for raw and smoothed data
# ax2 = fig3.add_subplot(111)

# # Plot the raw data
# ax2.plot(x_world_data, y_world_data, linestyle='-', color='b', label='Raw Data')

# # Plot the smoothed data
# ax2.plot(x_world_smoothed, y_world_smoothed, linestyle='-', color='r', label='Smoothed Data')


# # Add labels and title for 2D plot
# ax2.set_xlabel('Y Position  (s)')
# ax2.set_ylabel('X Position (cm)')
# ax2.set_title('2D Position Mapping')
# ax2.legend()

# # Set plot limits for 2D plot
# ax2.set_xlim(0, 150)
# ax2.set_ylim(0, 150)
# plt.grid()

# # Show the 2D plot
# plt.show()
