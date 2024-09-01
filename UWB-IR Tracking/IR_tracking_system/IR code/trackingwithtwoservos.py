import serial
import pygame
import sys
import pygame_gui
import math

# Initialize Pygame
pygame.init()

# Set up the display for the overlap area only
window_width, window_height = 600, 400  # Adjusted window size
window = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption("Shared Camera Feed - LED Tracking")

# Set up Pygame GUI manager
manager = pygame_gui.UIManager((window_width, window_height))

# Set up serial communication
port = 'COM3'  # Change this to your serial port
baudrate = 19200
ser = serial.Serial(port, baudrate, timeout=1)

# Predefined offsets
tilt_angle_cam1 = math.radians(3.68)  # Convert degrees to radians
tilt_angle_cam2 = math.radians(3.94)  # Convert degrees to radians
y_offset = 154  # Vertical offset for Camera 2

# Servo control parameters
servo_center_angle_x = 100  # Center angle for X-axis servo
servo_center_angle_y = 80  # Center angle for Y-axis servo
servo_range_x = 80  # Range of motion for X-axis servo (±80 degrees)
servo_range_y = 25  # Range of motion for Y-axis servo (±30 degrees)
led_center_x = window_width // 2  # Assume LED should be centered horizontally
led_center_y = window_height // 2  # Assume LED should be centered vertically
current_servo_angle_x = servo_center_angle_x  # Start at center position for X-axis
current_servo_angle_y = servo_center_angle_y  # Start at center position for Y-axis
max_servo_step = 1  # Maximum change in servo angle per frame for smoother movement
dead_zone = 45  # Dead zone for ignoring minor deviations
proportional_gain_x = 70  # Proportional gain for X-axis
proportional_gain_y = -70  # Proportional gain for Y-axis
derivative_gain_x = 1  # Derivative gain for X-axis
derivative_gain_y = 1  # Derivative gain for Y-axis

previous_deviation_x = 0  # To store the previous deviation for derivative calculation in X-axis
previous_deviation_y = 0  # To store the previous deviation for derivative calculation in Y-axis

# Function to draw the grid
def draw_grid(surface, grid_color=(200, 200, 200), cell_size=25):
    for x in range(0, surface.get_width(), cell_size):
        pygame.draw.line(surface, grid_color, (x, 0), (x, surface.get_height()))
    for y in range(0, surface.get_height(), cell_size):
        pygame.draw.line(surface, grid_color, (0, y), (surface.get_width(), y))

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
    except Exception as e:
        print(f"Failed to send angles to Arduino: {e}")

def main():
    global current_servo_angle_x, current_servo_angle_y
    global previous_deviation_x, previous_deviation_y

    clock = pygame.time.Clock()
    running = True
    font = pygame.font.Font(None, 24)  # Font for displaying coordinates

    while running:
        time_delta = clock.tick(30) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            manager.process_events(event)

        manager.update(time_delta)
        window.fill((77, 77, 77))  # Fill the background
        
        draw_grid(window)
        
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

                # Draw the ellipse representing the target LED at the center of the shared view
                pygame.draw.circle(window, (255, 255, 0), (avg_ix, avg_iy), 5)

                # Print the coordinates of the LED
                window.blit(font.render(f"({avg_ix}, {avg_iy})", True, (255, 255, 0)), (avg_ix + 15, avg_iy))

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
                    proportional_y = proportional_gain_y * deviation_y

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

                # Display the current angles
                window.blit(font.render(f"X Servo Angle: {current_servo_angle_x:.2f}", True, (255, 255, 255)), (10, 10))
                window.blit(font.render(f"Y Servo Angle: {current_servo_angle_y:.2f}", True, (255, 255, 255)), (10, 30))

        manager.draw_ui(window)
        pygame.display.flip()

    ser.close()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
