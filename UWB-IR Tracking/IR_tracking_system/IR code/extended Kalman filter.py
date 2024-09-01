import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import csv
import time
import numpy as np

# Function to read data from CSV file
def read_data_from_csv(filename):
    with open(filename, 'r') as csvfile:
        csv_reader = csv.reader(csvfile)
        next(csv_reader)  # Skip header
        for row in csv_reader:
            yield float(row[0]), float(row[1])

# Moving average filter class
class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data_window_x = []
        self.data_window_y = []

    def filter(self, new_value_x, new_value_y):
        if len(self.data_window_x) >= self.window_size:
            self.data_window_x.pop(0)
            self.data_window_y.pop(0)
        self.data_window_x.append(new_value_x)
        self.data_window_y.append(new_value_y)
        avg_x = np.mean(self.data_window_x)
        avg_y = np.mean(self.data_window_y)
        return avg_x, avg_y

# Nonlinear state transition function
def f(x, dt):
    F = np.array([[1, 0, dt, 0],
                  [0, 1, 0, dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    return F @ x

# Jacobian of the state transition function
def F_jacobian(x, dt):
    return np.array([[1, 0, dt, 0],
                     [0, 1, 0, dt],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

# Measurement function
def h(x):
    return np.array([x[0], x[1]])

# Jacobian of the measurement function
def H_jacobian(x):
    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0]])

# Initialize plot
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.3, top=0.9)  # Adjust layout to make room for sliders and title

anchors = [
    [0.0, 0.0],  # Anchor 1
    [2.0, 0.0],  # Anchor 2
    [1.0, 2.0],  # Anchor 3
]

# Plot anchors
for anchor in anchors:
    ax.scatter(anchor[0], anchor[1], c='r', marker='o', s=100)
    ax.text(anchor[0], anchor[1], f"Anchor ({anchor[0]}, {anchor[1]})", color='red')

# Set plot labels
ax.set_xlabel('X')
ax.set_ylabel('Y')

# Set plot title
ax.set_title('2D Position of Tag and Anchors')

# Initial plot of the tag position
tag_scatter = ax.scatter([], [], c='b', marker='^', s=100, label='Filtered Position (EKF)')
raw_scatter = ax.scatter([], [], c='c', marker='x', s=100, label='Raw Position')
avg_scatter = ax.scatter([], [], c='g', marker='o', s=100, label='Averaged Position')

tag_text = ax.text(0, 0, "", color='blue')
time_text = ax.text(1.5, 2.2, "", color='black', fontsize=12, bbox=dict(facecolor='white', alpha=0.5))

# Lines and text for distances
lines = [ax.plot([], [], 'g-')[0] for _ in anchors]
distance_texts = [ax.text(0, 0, '', color='green') for _ in anchors]

# EKF initialization
dt = 1.0  # Time step

# State vector [x, y, vx, vy]
x = np.zeros(4)

# Initial covariance matrix
P = np.eye(4) * 100

# Initial process noise covariance matrix
Q = np.eye(4) * 0.001

# Initial measurement noise covariance matrix
R = np.array([[0.5, 0],
              [0, 0.5]])

# Function to update the plot
def update_plot(measured_x, measured_y, avg_x, avg_y, filtered_x, filtered_y, elapsed_time):
    # Update tag position (filtered)
    tag_scatter.set_offsets([filtered_x, filtered_y])
    tag_text.set_position((filtered_x, filtered_y))
    tag_text.set_text(f"Tag ({filtered_x:.2f}, {filtered_y:.2f})")
    
    # Update raw position
    raw_scatter.set_offsets([measured_x, measured_y])
    
    # Update averaged position
    avg_scatter.set_offsets([avg_x, avg_y])

    # Update lines and distances
    for i, anchor in enumerate(anchors):
        # Update line
        lines[i].set_data([anchor[0], filtered_x], [anchor[1], filtered_y])
        
        # Calculate and display distance
        distance = ((anchor[0] - filtered_x)**2 + (anchor[1] - filtered_y)**2)**0.5
        distance_texts[i].set_position(((anchor[0] + filtered_x) / 2, (anchor[1] + filtered_y) / 2))
        distance_texts[i].set_text(f'{distance:.2f} m')
    
    # Update time text
    time_text.set_text(f'Time: {elapsed_time:.2f}s')

    plt.draw()
    plt.pause(0.01)

# Path to the CSV file
filename = 'tag_positions.csv'

# Sliders
axcolor = 'lightgoldenrodyellow'
ax_q = plt.axes([0.1, 0.2, 0.65, 0.03], facecolor=axcolor)
ax_r = plt.axes([0.1, 0.15, 0.65, 0.03], facecolor=axcolor)
ax_p = plt.axes([0.1, 0.1, 0.65, 0.03], facecolor=axcolor)

s_q = Slider(ax_q, 'Process Noise Q', 0.0001, 0.1, valinit=0.001, valstep=0.0001)
s_r = Slider(ax_r, 'Measurement Noise R', 0.1, 5.0, valinit=0.5, valstep=0.1)
s_p = Slider(ax_p, 'Initial Uncertainty P', 1, 500, valinit=100, valstep=1)

def update(val):
    global Q, R, P
    Q = np.eye(4) * s_q.val
    R = np.array([[s_r.val, 0],
                  [0, s_r.val]])
    P = np.eye(4) * s_p.val

s_q.on_changed(update)
s_r.on_changed(update)
s_p.on_changed(update)

# Read data from CSV file and update plot
start_time = time.time()

# Initialize moving average filter with window size 5
window_size = 10    # 5
ma_filter = MovingAverageFilter(window_size)

try:
    for measured_x, measured_y in read_data_from_csv(filename):
        # Apply moving average filter
        avg_x, avg_y = ma_filter.filter(measured_x, measured_y)

        # EKF predict step
        x = f(x, dt)
        F = F_jacobian(x, dt)
        P = F @ P @ F.T + Q

        # EKF update step
        z = np.array([avg_x, avg_y])  # Measurement
        H = H_jacobian(x)
        y = z - h(x)  # Measurement residual
        S = H @ P @ H.T + R  # Residual covariance
        K = P @ H.T @ np.linalg.inv(S)  # Kalman gain
        x = x + K @ y
        P = (np.eye(len(P)) - K @ H) @ P

        # Use the filtered position for plotting
        filtered_x, filtered_y = x[0], x[1]
        elapsed_time = time.time() - start_time
        update_plot(measured_x, measured_y, avg_x, avg_y, filtered_x, filtered_y, elapsed_time)
        time.sleep(0.1)  # Simulate delay between data points
except KeyboardInterrupt:
    print("Terminated by user")
finally:
    plt.legend(loc='upper left')
    plt.show()
