import serial
import math
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import csv
from collections import deque
from UWB_EKF_LPF import start_uwb_tracking
import time

# Set up serial communication for the IR system
ir_port = 'COM3'  # Change this to your serial port for IR system
ir_baudrate = 19200
ir_ser = serial.Serial(ir_port, ir_baudrate, timeout=1)

# Predefined offsets for IR
tilt_angle_cam1 = math.radians(3.68)
tilt_angle_cam2 = math.radians(3.94)
y_offset = 154

# Servo control parameters for IR
servo_center_angle_x = 105
servo_center_angle_y = 75
servo_range_x = 80
servo_range_y = 29
window_width, window_height = 900, 600
led_center_x = window_width // 2
led_center_y = window_height // 2
current_servo_angle_x = servo_center_angle_x
current_servo_angle_y = servo_center_angle_y
max_servo_step = 1
dead_zone = 45
proportional_gain_x = 150  # 70
proportional_gain_y = 200 # 160
derivative_gain_x = 1
derivative_gain_y = 1

previous_deviation_x = 0
previous_deviation_y = 0

baseline_cm = 7
camera_focal_length = 1331.43
camera_position = [100.0, 0.0]

# Moving average parameters
window_size = 20
x_world_window = deque(maxlen=window_size)
y_world_window = deque(maxlen=window_size)
z_world_window = deque(maxlen=window_size)  # Added for Z axis

# Variables to hold positions
ir_position = None
ir_raw_position = None
ir_mv_position = None
uwb_position = [0, 0, 0]  # Updated to include Z axis
lpf_position = [0, 0, 0]  # Updated to include Z axis
raw_position = [0, 0, 0]  # Updated to include Z axis
kalman_position = [0, 0, 0]  # Updated to include Z axis

# List to store Kalman Filtered positions over time
kf_position_history = []
uwb_position_history = []
ir_mv_position_history = []

# Anchors position
anchors = [
    [0.0, 0.0],  # Anchor 1
    [2.0, 0.0],  # Anchor 2
    [1.0, 1.8],  # Anchor 3
]

# Kalman filter initialization
class KalmanFilterXY:
    def __init__(self, dt, Q, R):
        self.dt = dt
        self.Q = Q
        self.R = R
        self.P = np.eye(4)  # Updated to 4x4 for 2D (x, y, vx, vy)
        self.x = np.zeros(4)  # Initial state [x, y, vx, vy]

    def predict(self):
        F = np.array([[1, 0, self.dt, 0],
                      [0, 1, 0, self.dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

# Initialize filters
dt = 0.1
Q = np.eye(4) * 0.1
R = np.eye(2)  # Updated for 2D (x, y)
kalman_filter_xy = KalmanFilterXY(dt, Q, R)

# Initialize CSV file
csv_filename = 'record_3d.csv'
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time (s)', 'IR_Raw_X', 'IR_Raw_Y', 'IR_Raw_Z', 'IR_MV_X', 'IR_MV_Y', 'IR_MV_Z',
                     'UWB_Raw_X', 'UWB_Raw_Y', 'UWB_Raw_Z', 'UWB_LPF_X', 'UWB_LPF_Y', 'UWB_LPF_Z',
                     'UWB_EKF_X', 'UWB_EKF_Y', 'UWB_EKF_Z', 'Kalman_X', 'Kalman_Y', 'Kalman_Z'])

# Record the start time
start_time = time.time()

# Counter to track the number of data points processed
data_point_counter = 0

# Function to read serial data for IR
def read_serial_data():
    try:
        if ir_ser.in_waiting > 0:
            line = ir_ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                return list(map(int, line.split(',')))
    except (serial.SerialException, ValueError) as e:
        print(f"Error reading serial data: {e}")
    return None

# Function to apply tilt correction
def apply_tilt_correction(ix, iy, tilt_angle):
    new_x = ix
    new_y = iy * math.cos(tilt_angle) - ix * math.sin(tilt_angle)
    return int(new_x), int(new_y)

# Function to send servo angles
def send_servo_angles(angle_x, angle_y):
    try:
        ir_ser.write(f"{angle_x},{angle_y}\n".encode('utf-8'))
    except Exception as e:
        print(f"Failed to send angles to Arduino: {e}")

def calculate_depth(x1, x2):
    r = abs(x1 - x2)
    if r == 0:
        print("Warning: No distance between LED positions.")
        return None
    try:
        depth_cm = (camera_focal_length * baseline_cm) / r
        return depth_cm
    except ZeroDivisionError:
        print("ZeroDivisionError: Distance is zero.")
        return None

def ir_tracking():
    global current_servo_angle_x, current_servo_angle_y
    global previous_deviation_x, previous_deviation_y
    global ir_position, ir_raw_position, ir_mv_position

    last_time = time.time()

    while True:
        current_time = time.time()
        time_delta = current_time - last_time
        last_time = current_time

        if time_delta == 0:
            time_delta = 1e-6

        data = read_serial_data()
        if data and len(data) == 16:
            ix1, iy1, ix2, iy2, ix3, iy3, ix4, iy4 = data[:8]
            ix5, iy5, ix6, iy6, ix7, iy7, ix8, iy8 = data[8:]

            iy1 = window_height - iy1
            iy2 = window_height - iy2
            iy3 = window_height - iy3
            iy4 = window_height - iy4
            iy5 = window_height - iy5
            iy6 = window_height - iy6
            iy7 = window_height - iy7
            iy8 = window_height - iy8

            ix1, iy1 = apply_tilt_correction(ix1, iy1, tilt_angle_cam1)
            ix2, iy2 = apply_tilt_correction(ix2, iy2, tilt_angle_cam1)
            ix3, iy3 = apply_tilt_correction(ix3, iy3, tilt_angle_cam1)
            ix4, iy4 = apply_tilt_correction(ix4, iy4, tilt_angle_cam1)

            ix5, iy5 = apply_tilt_correction(ix5, iy5, tilt_angle_cam2)
            ix6, iy6 = apply_tilt_correction(ix6, iy6, tilt_angle_cam2)
            ix7, iy7 = apply_tilt_correction(ix7, iy7, tilt_angle_cam2)
            ix8, iy8 = apply_tilt_correction(ix8, iy8, tilt_angle_cam2)

            iy5 += y_offset
            iy6 += y_offset
            iy7 += y_offset
            iy8 += y_offset

            coords = [
                (ix1, iy1), (ix2, iy2), (ix3, iy3), (ix4, iy4),
                (ix5, iy5), (ix6, iy6), (ix7, iy7), (ix8, iy8)
            ]

            valid_coords = [(ix, iy) for ix, iy in coords if 0 <= ix < window_width and 0 <= iy < window_height]

            if valid_coords:
                avg_ix = sum(ix for ix, iy in valid_coords) // len(valid_coords)
                avg_iy = sum(iy for ix, iy in valid_coords) // len(valid_coords)

                deviation_x = avg_ix - led_center_x
                deviation_y = avg_iy - led_center_y

                if abs(deviation_x) > dead_zone:
                    proportional_x = proportional_gain_x * deviation_x
                    derivative_x = derivative_gain_x * (deviation_x - previous_deviation_x) / time_delta
                    previous_deviation_x = deviation_x
                    adjustment_x = proportional_x + derivative_x
                    target_servo_angle_x = servo_center_angle_x + (adjustment_x / (window_width / 2)) * servo_range_x
                    target_servo_angle_x = max(servo_center_angle_x - servo_range_x, min(servo_center_angle_x + servo_range_x, target_servo_angle_x))

                    if abs(target_servo_angle_x - current_servo_angle_x) > max_servo_step:
                        if target_servo_angle_x > current_servo_angle_x:
                            current_servo_angle_x += max_servo_step
                        else:
                            current_servo_angle_x -= max_servo_step
                    else:
                        current_servo_angle_x = target_servo_angle_x

                if abs(deviation_y) > dead_zone:
                    proportional_y = -proportional_gain_y * deviation_y
                    derivative_y = derivative_gain_y * (deviation_y - previous_deviation_y) / time_delta
                    previous_deviation_y = deviation_y
                    adjustment_y = proportional_y + derivative_y
                    target_servo_angle_y = servo_center_angle_y + (adjustment_y / (window_height / 2)) * servo_range_y
                    target_servo_angle_y = max(servo_center_angle_y - servo_range_y, min(servo_center_angle_y + servo_range_y, target_servo_angle_y))

                    if abs(target_servo_angle_y - current_servo_angle_y) > max_servo_step:
                        if target_servo_angle_y > current_servo_angle_y:
                            current_servo_angle_y += max_servo_step
                        else:
                            current_servo_angle_y -= max_servo_step
                    else:
                        current_servo_angle_y = target_servo_angle_y

                send_servo_angles(int(current_servo_angle_x), int(current_servo_angle_y))
                depth_cm = calculate_depth(ix1, ix5)
                
                servo_angle_x_rad = math.radians(current_servo_angle_x - servo_center_angle_x)
                servo_angle_y_rad = math.radians(current_servo_angle_y - servo_center_angle_y)

                y_world = depth_cm * math.sin(math.radians(90) - servo_angle_y_rad) * math.cos(servo_angle_x_rad)
                x_world = depth_cm * math.sin(math.radians(90) - servo_angle_y_rad) * math.sin(servo_angle_x_rad)
                z_world = depth_cm * math.cos(math.radians(90) - servo_angle_y_rad)

                ir_position = [-x_world + camera_position[0], y_world + camera_position[1], z_world]

                x_world_window.append(ir_position[0])
                y_world_window.append(ir_position[1])
                z_world_window.append(ir_position[2])

                ir_mv_position = [np.mean(x_world_window), np.mean(y_world_window), np.mean(z_world_window)]
            else:
                ir_position = None
                ir_mv_position = None

# Initialize 3D Matplotlib plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')  # Create 3D axes
ax.set_xlim(-10, 210)
ax.set_ylim(-10, 210)
ax.set_zlim(-10, 100)  # Set limits for Z axis
ax.set_xlabel("X Position (cm)")
ax.set_ylabel("Y Position (cm)")
ax.set_zlabel("Z Position (cm)")  # Label Z axis
ax.set_title("3D Position Tracking")

# Plot anchors and camera position
for i, anchor in enumerate(anchors):
    ax.scatter(anchor[0] * 100, anchor[1] * 100, 0, c='r', marker='o', s=100)
    ax.text(anchor[0] * 100 + 2, anchor[1] * 100 + 2, 0, f"Anchor {i+1}", color='red')

ax.scatter(camera_position[0], camera_position[1], 0, c='k', marker='s', s=100)
ax.text(camera_position[0] + 2, camera_position[1] + 2, 0, "Camera", color='black')

# Ground truth in 3D
# ground_truth_x = [60, 60, 120, 120]
# ground_truth_y = [100, 100, 100, 100]
# ground_truth_z = [0, 50, 50, 0]
# Ground truth in 3D
ground_truth_x = [60, 60, 120, 120, 120, 120]
ground_truth_y = [100, 100, 100, 100, 60, 60]
ground_truth_z = [0, 45, 45, 30, 30, 0]
ax.plot(ground_truth_x, ground_truth_y, ground_truth_z, c='black', linestyle='-', label='Ground Truth')

# Initialize plot points
ir_line, = ax.plot([], [], [], 'm-', label="IR MV Filtered Path")  # Updated to 3D line
uwb_line, = ax.plot([], [], [], 'b-', label="UWB EKF Path")  # Updated to 3D line
kf_line, = ax.plot([], [], [], 'g-', label="Kalman Filtered Path")  # Updated to 3D line

def update_plot(frame):
    global kalman_position, kf_position_history, data_point_counter
    global ir_mv_position_history, uwb_position_history

    data_point_counter += 1

    if data_point_counter <= 70:
        return []

    kalman_filter_xy.predict()

    if ir_mv_position is not None:
        measurement_xy = np.array(ir_mv_position[:2]) * 0.70 + np.array(uwb_position[:2]) * 0.30
        kalman_filter_xy.update(measurement_xy)
        kalman_position[:2] = kalman_filter_xy.x[:2]
        kalman_position[2] = ir_mv_position[2]  # Directly take Z from IR MV position
    else:
        measurement_xy = np.array(uwb_position[:2])
        kalman_filter_xy.update(measurement_xy)
        kalman_position[:2] = kalman_filter_xy.x[:2]
        kalman_position[2] = uwb_position[2]

    kf_position_history.append(kalman_position.copy())

    if ir_mv_position is not None:
        ir_mv_position_history.append(ir_mv_position.copy())
        ir_line.set_data(*zip(*[(p[0], p[1]) for p in ir_mv_position_history]))
        ir_line.set_3d_properties([p[2] for p in ir_mv_position_history])
    else:
        ir_line.set_data([], [])
        ir_line.set_3d_properties([])

    uwb_position_history.append(uwb_position.copy())
    uwb_line.set_data(*zip(*[(p[0], p[1]) for p in uwb_position_history]))
    uwb_line.set_3d_properties([p[2] for p in uwb_position_history])

    kf_line.set_data(*zip(*[(p[0], p[1]) for p in kf_position_history]))
    kf_line.set_3d_properties([p[2] for p in kf_position_history])

    elapsed_time = time.time() - start_time
    with open(csv_filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([
            elapsed_time,
            ir_position[0] if ir_position is not None else None,
            ir_position[1] if ir_position is not None else None,
            ir_position[2] if ir_position is not None else None,
            ir_mv_position[0] if ir_mv_position is not None else None,
            ir_mv_position[1] if ir_mv_position is not None else None,
            ir_mv_position[2] if ir_mv_position is not None else None,
            raw_position[0], raw_position[1], raw_position[2],  # Updated to ensure Z axis access
            lpf_position[0], lpf_position[1], lpf_position[2],  # Updated to ensure Z axis access
            uwb_position[0], uwb_position[1], uwb_position[2],
            kalman_position[0], kalman_position[1], kalman_position[2]
        ])

    return ir_line, uwb_line, kf_line

# Start data acquisition threads
ir_thread = threading.Thread(target=ir_tracking)
ir_thread.start()

uwb_serial_port = serial.Serial('COM4', 115200, timeout=1)
uwb_thread = start_uwb_tracking(uwb_serial_port, uwb_position, lpf_position, raw_position)

ani = FuncAnimation(fig, update_plot, interval=50, blit=True, cache_frame_data=False)
ax.legend()
plt.show()

ir_thread.join()
uwb_thread.join()
