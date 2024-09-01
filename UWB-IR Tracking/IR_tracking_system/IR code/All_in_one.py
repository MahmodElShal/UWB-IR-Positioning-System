import serial
import math
import sys
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Set up serial communication
port = 'COM3'  # Change this to your serial port
baudrate = 19200
ser = serial.Serial(port, baudrate, timeout=1)

# Predefined offsets
tilt_angle_cam1 = math.radians(3.68)  # Convert degrees to radians
tilt_angle_cam2 = math.radians(3.94)  # Convert degrees to radians
y_offset = 154  # Vertical offset for Camera 2

# Servo control parameters
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

# Variable to store the last valid depth value
last_valid_depth_cm = None

# Initialize matplotlib figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.ion()
fig.show()
fig.canvas.draw()

# Function to read serial data
def read_serial_data():
    try:
        if ser.in_waiting > 0:  # Check if there's data waiting to be read
            line = ser.readline().decode('utf-8').strip()
            if line:
                return list(map(int, line.split(',')))
    except (serial.SerialException, UnicodeDecodeError, ValueError) as e:
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
        ser.write(f"{angle_x},{angle_y}\n".encode('utf-8'))
        # print(f"Servo angles: {angle_x}, {angle_y}")
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
        print("ZeroDivisionError: Distance is zero. Using last valid depth.")
        return last_valid_depth_cm

def main():
    global current_servo_angle_x, current_servo_angle_y
    global previous_deviation_x, previous_deviation_y

    last_time = time.time()
    running = True

    while running:
        current_time = time.time()
        time_delta = current_time - last_time
        last_time = current_time

        # Ensure time_delta is not zero
        if time_delta == 0:
            time_delta = 1e-6

        data = read_serial_data()
        if data and len(data) >= 16:
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
                # print(depth_cm)

                # Convert angles from degrees to radians
                servo_angle_x_rad = math.radians(current_servo_angle_x - servo_center_angle_x)
                servo_angle_y_rad = math.radians(current_servo_angle_y - servo_center_angle_y)

                x_world = depth_cm * math.sin(math.radians(90) - servo_angle_y_rad) * math.cos(servo_angle_x_rad)
                y_world = depth_cm * math.sin(math.radians(90) - servo_angle_y_rad) * math.sin(servo_angle_x_rad)
                z_world = depth_cm * math.cos(math.radians(90) - servo_angle_y_rad)
                print(x_world, y_world, z_world)

                # Update the plot with the new coordinates
                ax.clear()
                ax.scatter(x_world, y_world, z_world, c='r', marker='o')
                ax.set_xlabel('X World')
                ax.set_ylabel('Y World')
                ax.set_zlabel('Z World')
                plt.draw()
                plt.pause(0.01)

    ser.close()
    sys.exit()

if __name__ == "__main__":
    main()
