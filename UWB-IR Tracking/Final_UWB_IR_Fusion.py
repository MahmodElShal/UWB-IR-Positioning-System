import serial
import math
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from UWB_EKF_LPF import start_uwb_tracking  # Import the UWB tracking function
import time

# Set up serial communication for the IR system
ir_port = 'COM3'  # Change this to your serial port for IR system
ir_baudrate = 19200
ir_ser = serial.Serial(ir_port, ir_baudrate, timeout=1)

# Predefined offsets for IR
tilt_angle_cam1 = math.radians(3.68)  # Convert degrees to radians
tilt_angle_cam2 = math.radians(3.94)  # Convert degrees to radians
y_offset = 154  # Vertical offset for Camera 2

# Servo control parameters for IR
servo_center_angle_x = 105  # Center angle for X-axis servo
servo_center_angle_y = 75  # Center angle for Y-axis servo
servo_range_x = 80  # Range of motion for X-axis servo (±80 degrees)
servo_range_y = 29  # Range of motion for Y-axis servo (±30 degrees)
window_width, window_height = 900, 600  # Window dimensions for reference
led_center_x = window_width // 2  # Assume LED should be centered horizontally
led_center_y = window_height // 2  # Assume LED should be centered vertically
current_servo_angle_x = servo_center_angle_x  # Start at center position for X-axis
current_servo_angle_y = servo_center_angle_y  # Start at center position for Y-axis
max_servo_step = 1  # Maximum change in servo angle per frame for smoother movement
dead_zone = 45  # Dead zone for ignoring minor deviations
proportional_gain_x = 70  # Proportional gain for X-axis
proportional_gain_y = 160  # Proportional gain for Y-axis
derivative_gain_x = 1  # Derivative gain for X-axis
derivative_gain_y = 1  # Derivative gain for Y-axis

previous_deviation_x = 0  # To store the previous deviation for derivative calculation in X-axis
previous_deviation_y = 0  # To store the previous deviation for derivative calculation in Y-axis

baseline_cm = 7  # Distance between cameras in cm
camera_focal_length = 1331.43  # Camera focal length
camera_position = [100.0, 0.0]  # New origin for the camera

# Variables to hold positions
ir_position = None  # None initially to indicate no detection
ir_raw_position = None  # Raw IR data
ir_mv_position = None  # Moving Average filtered IR position
uwb_position = [0, 0]  # EKF UWB position
lpf_position = [0, 0]  # LPF UWB position
raw_position = [0, 0]  # Raw UWB data
kalman_position = [0, 0]  # Kalman filtered position

# Anchors position
anchors = [
    [0.0, 0.0],  # Anchor 1
    [2.0, 0.0],  # Anchor 2
    [1.0, 2.0],  # Anchor 3
]

# Moving Average filter
class MovingAverage:
    def __init__(self, n=5):
        self.n = n
        self.data = []

    def filter(self, value):
        self.data.append(value)
        if len(self.data) > self.n:
            self.data.pop(0)
        return np.mean(self.data, axis=0)

# Kalman filter initialization
class KalmanFilter:
    def __init__(self, dt, Q, R):
        self.dt = dt
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance
        self.P = np.eye(4)  # Initial covariance matrix
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
Q = np.eye(4) * 0.1  # Process noise covariance
R = np.array([[0.5, 0], [0, 1]])  # Measurement noise covariance (trust IR more)
kalman_filter = KalmanFilter(dt, Q, R)
ir_mv_filter = MovingAverage(n=5)

# Function to read serial data for IR
def read_serial_data():
    try:
        if ir_ser.in_waiting > 0:  # Check if there's data waiting to be read
            line = ir_ser.readline().decode('utf-8', errors='ignore').strip()  # Ignore decoding errors
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

        # Ensure time_delta is not zero
        if time_delta == 0:
            time_delta = 1e-6

        data = read_serial_data()
        if data and len(data) == 16:  # Ensure the data contains exactly 16 values
            # Unpack the data for both cameras
            ix1, iy1, ix2, iy2, ix3, iy3, ix4, iy4 = data[:8]
            ix5, iy5, ix6, iy6, ix7, iy7, ix8, iy8 = data[8:]

            # Flip y-coordinates (assuming origin is bottom-left in serial data)
            iy1 = window_height - iy1
            iy2 = window_height - iy2
            iy3 = window_height - iy3
            iy4 = window_height - iy4
            iy5 = window_height - iy5
            iy6 = window_height - iy6
            iy7 = window_height - iy7
            iy8 = window_height - iy8

            # Store raw IR position (midpoint between the two cameras' data)
            ir_raw_position = [(ix1 + ix5) // 2, (iy1 + iy5) // 2]

            # Apply tilt correction for camera 1
            ix1, iy1 = apply_tilt_correction(ix1, iy1, tilt_angle_cam1)
            ix2, iy2 = apply_tilt_correction(ix2, iy2, tilt_angle_cam1)
            ix3, iy3 = apply_tilt_correction(ix3, iy3, tilt_angle_cam1)
            ix4, iy4 = apply_tilt_correction(ix4, iy4, tilt_angle_cam1)

            # Apply tilt correction for camera 2
            ix5, iy5 = apply_tilt_correction(ix5, iy5, tilt_angle_cam2)
            ix6, iy6 = apply_tilt_correction(ix6, iy6, tilt_angle_cam2)
            ix7, iy7 = apply_tilt_correction(ix7, iy7, tilt_angle_cam2)
            ix8, iy8 = apply_tilt_correction(ix8, iy8, tilt_angle_cam2)

            # Apply y_offset to Camera 2 data
            iy5 += y_offset
            iy6 += y_offset
            iy7 += y_offset
            iy8 += y_offset

            # Ensure coordinates are within window bounds
            coords = [
                (ix1, iy1), (ix2, iy2), (ix3, iy3), (ix4, iy4),
                (ix5, iy5), (ix6, iy6), (ix7, iy7), (ix8, iy8)
            ]

            valid_coords = [(ix, iy) for ix, iy in coords if 0 <= ix < window_width and 0 <= iy < window_height]

            if valid_coords:
                # Calculate the average position for the shared feed
                avg_ix = sum(ix for ix, iy in valid_coords) // len(valid_coords)
                avg_iy = sum(iy for ix, iy in valid_coords) // len(valid_coords)

                # Calculate the deviation from the center in x-axis and y-axis
                deviation_x = avg_ix - led_center_x
                deviation_y = avg_iy - led_center_y

                # Check if the deviation is outside the dead zone for X-axis
                if abs(deviation_x) > dead_zone:
                    # Proportional term for X-axis
                    proportional_x = proportional_gain_x * deviation_x

                    # Derivative term for X-axis (rate of change of error)
                    derivative_x = derivative_gain_x * (deviation_x - previous_deviation_x) / time_delta

                    # Update previous deviation for the next frame
                    previous_deviation_x = deviation_x

                    # Total adjustment for X-axis
                    adjustment_x = proportional_x + derivative_x

                    # Map the adjustment to the servo range
                    target_servo_angle_x = servo_center_angle_x + (adjustment_x / (window_width / 2)) * servo_range_x
                    target_servo_angle_x = max(servo_center_angle_x - servo_range_x, min(servo_center_angle_x + servo_range_x, target_servo_angle_x))

                    # Update the current servo angle for X-axis
                    if abs(target_servo_angle_x - current_servo_angle_x) > max_servo_step:
                        if target_servo_angle_x > current_servo_angle_x:
                            current_servo_angle_x += max_servo_step
                        else:
                            current_servo_angle_x -= max_servo_step
                    else:
                        current_servo_angle_x = target_servo_angle_x

                # Check if the deviation is outside the dead zone for Y-axis
                if abs(deviation_y) > dead_zone:
                    # Proportional term for Y-axis
                    proportional_y = -proportional_gain_y * deviation_y

                    # Derivative term for Y-axis (rate of change of error)
                    derivative_y = derivative_gain_y * (deviation_y - previous_deviation_y) / time_delta

                    # Update previous deviation for the next frame
                    previous_deviation_y = deviation_y

                    # Total adjustment for Y-axis
                    adjustment_y = proportional_y + derivative_y

                    # Map the adjustment to the servo range
                    target_servo_angle_y = servo_center_angle_y + (adjustment_y / (window_height / 2)) * servo_range_y
                    target_servo_angle_y = max(servo_center_angle_y - servo_range_y, min(servo_center_angle_y + servo_range_y, target_servo_angle_y))

                    # Update the current servo angle for Y-axis
                    if abs(target_servo_angle_y - current_servo_angle_y) > max_servo_step:
                        if target_servo_angle_y > current_servo_angle_y:
                            current_servo_angle_y += max_servo_step
                        else:
                            current_servo_angle_y -= max_servo_step
                    else:
                        current_servo_angle_y = target_servo_angle_y

                # Send the servo angles to Arduino
                send_servo_angles(int(current_servo_angle_x), int(current_servo_angle_y))
                depth_cm = calculate_depth(ix1, ix5)
                
                # Convert angles from degrees to radians
                servo_angle_x_rad = math.radians(current_servo_angle_x - servo_center_angle_x)
                servo_angle_y_rad = math.radians(current_servo_angle_y - servo_center_angle_y)

                y_world = depth_cm * math.sin(math.radians(90) - servo_angle_y_rad) * math.cos(servo_angle_x_rad)
                x_world = depth_cm * math.sin(math.radians(90) - servo_angle_y_rad) * math.sin(servo_angle_x_rad)

                # Adjust the IR position to account for the camera's new origin at [1, 0]
                ir_position = [-x_world + camera_position[0], y_world + camera_position[1]]

                # Apply Moving Average filter to IR position
                ir_mv_position = ir_mv_filter.filter(ir_position)

                # Print the IR position
                print(f"IR Position: X = {ir_position[0]:.2f} cm, Y = {ir_position[1]:.2f} cm")
            else:
                ir_position = None  # No valid IR data
                ir_mv_position = None  # No valid IR data

# Initialize Matplotlib plot
fig, ax = plt.subplots()
ax.set_xlim(-10, 210)  # Set X axis limits
ax.set_ylim(-10, 210)  # Set Y axis limits
ax.set_xlabel("X Position (cm)")
ax.set_ylabel("Y Position (cm)")
ax.set_title("2D Position Tracking")

# Plot anchors as red dots and label them
for i, anchor in enumerate(anchors):
    ax.plot(anchor[0] * 100, anchor[1] * 100, 'ro', markersize=10)
    ax.text(anchor[0] * 100 + 2, anchor[1] * 100 + 2, f"Anchor {i+1}", color='red')

# Plot camera position as a black square and label it
ax.plot(camera_position[0], camera_position[1], 'ks', markersize=10)
ax.text(camera_position[0] + 2, camera_position[1] + 2, "Camera", color='black')

# Initialize plot points
ir_point, = ax.plot([], [], 'mo', markersize=8, label="IR MV Filtered")  # Purple dot for MV IR
# ir_raw_point, = ax.plot([], [], 'yo', markersize=6, label="IR Raw Data")  # Yellow dot for raw IR
# uwb_raw_point, = ax.plot([], [], 'ro', markersize=6, label="UWB Raw Data")  # Red dot for raw UWB
# lpf_point, = ax.plot([], [], 'go', markersize=6, label="UWB LPF")  # Green dot for LPF UWB
uwb_point, = ax.plot([], [], 'b^', markersize=10, label="UWB EKF")  # Blue triangle for EKF UWB
kf_point, = ax.plot([], [], 'gx', markersize=10, label="Kalman Filtered Position")  # Green cross for Kalman filtered

def update_plot(frame):
    global kalman_position

    # Combine IR (MV) and UWB (EKF) positions using Kalman Filter
    kalman_filter.predict()
    if ir_mv_position is not None:
        measurement = np.array(ir_mv_position) * 0.65 + np.array(uwb_position) * 0.35  # Weighted measurement
    else:
        measurement = np.array(uwb_position)  # Use only UWB if IR is not available
    kalman_filter.update(measurement)
    kalman_position = kalman_filter.x[:2]

    # Update plot points with current positions
    if ir_mv_position is not None:
        ir_point.set_data([ir_mv_position[0]], [ir_mv_position[1]])  # MV IR
    else:
        ir_point.set_data([], [])  # Clear the IR point if not detected

    # ir_raw_point.set_data([ir_position[0]], [ir_position[1]])  # IR Position
    # uwb_raw_point.set_data([raw_position[0]], [raw_position[1]])  # Raw UWB
    # lpf_point.set_data([lpf_position[0]], [lpf_position[1]])  # LPF UWB
    uwb_point.set_data([uwb_position[0]], [uwb_position[1]])  # EKF UWB
    kf_point.set_data([kalman_position[0]], [kalman_position[1]])  # Kalman Filtered

    return ir_point, uwb_point, kf_point  #, ir_raw_point, uwb_raw_point, lpf_point

# Start data acquisition threads
ir_thread = threading.Thread(target=ir_tracking)
ir_thread.start()

# Start the UWB tracking by importing the function from the UWB script
uwb_serial_port = serial.Serial('COM4', 115200, timeout=1)  # Replace with your UWB serial port
uwb_thread = start_uwb_tracking(uwb_serial_port, uwb_position, lpf_position, raw_position)

# Start the animation
ani = FuncAnimation(fig, update_plot, interval=50, blit=True, cache_frame_data=False)
ax.legend()
plt.show()

# Ensure the threads are properly joined when the script ends
ir_thread.join()
uwb_thread.join()
