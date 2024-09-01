import serial
import pygame
import sys
import pygame_gui
import math
import matplotlib.pyplot as plt
import numpy as np

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
servo_center_angle = 90  # Default center angle for the servo
servo_range = 90  # Range of motion for the servo (±45 degrees from center)
led_center_x = window_width // 2  # Assume LED should be centered
current_servo_angle = servo_center_angle  # Start at center position
max_servo_step = 1  # Maximum change in servo angle per frame for smoother movement
dead_zone = 5  # Dead zone for ignoring minor deviations
proportional_gain = 0.1  # Proportional gain for adjusting the servo angle
derivative_gain = 0.05  # Derivative gain to reduce oscillations

previous_deviation_x = 0  # To store the previous deviation for derivative calculation

# Data storage for plotting
time_stamps = []
deviations = []
proportional_adjustments = []
derivative_adjustments = []
servo_angles = []

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

# Function to send servo angle
def send_servo_angle(angle):
    try:
        ser.write(f"{angle}\n".encode('utf-8'))
    except Exception as e:
        print(f"Failed to send angle to Arduino: {e}")

# Function to plot data
def plot_data():
    plt.figure(figsize=(10, 8))

    # Plot deviation (error) over time
    plt.subplot(4, 1, 1)
    plt.plot(time_stamps, deviations, label='Deviation')
    plt.ylabel('Deviation (pixels)')
    plt.legend()

    # Plot proportional adjustments over time
    plt.subplot(4, 1, 2)
    plt.plot(time_stamps, proportional_adjustments, label='Proportional Adjustment')
    plt.ylabel('Proportional Adjustment')
    plt.legend()

    # Plot derivative adjustments over time
    plt.subplot(4, 1, 3)
    plt.plot(time_stamps, derivative_adjustments, label='Derivative Adjustment')
    plt.ylabel('Derivative Adjustment')
    plt.legend()

    # Plot servo angles over time
    plt.subplot(4, 1, 4)
    plt.plot(time_stamps, servo_angles, label='Servo Angle')
    plt.ylabel('Servo Angle (degrees)')
    plt.xlabel('Time (s)')
    plt.legend()

    plt.tight_layout()
    plt.show()

def main():
    global current_servo_angle, previous_deviation_x
    clock = pygame.time.Clock()
    running = True
    font = pygame.font.Font(None, 24)  # Font for displaying coordinates

    start_time = pygame.time.get_ticks() / 1000  # Initial time in seconds

    while running:
        time_delta = clock.tick(30) / 1000.0
        current_time = pygame.time.get_ticks() / 1000 - start_time

        for event in pygame.event.get():
            if event.type == pygame.QUIT or current_time >= 30:
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

                # Calculate the deviation from the center in x-axis
                deviation_x = avg_ix - led_center_x

                # Check if the deviation is outside the dead zone
                if abs(deviation_x) > dead_zone:
                    # Proportional term
                    proportional = proportional_gain * deviation_x

                    # Derivative term (rate of change of error)
                    derivative = derivative_gain * (deviation_x - previous_deviation_x) / time_delta

                    # Update previous deviation for the next frame
                    previous_deviation_x = deviation_x

                    # Total adjustment
                    adjustment = proportional + derivative

                    # Smoothly transition to the target angle
                    target_servo_angle = current_servo_angle + adjustment
                    target_servo_angle = max(0, min(180, target_servo_angle))

                    # Update the current servo angle
                    if abs(target_servo_angle - current_servo_angle) > max_servo_step:
                        if target_servo_angle > current_servo_angle:
                            current_servo_angle += max_servo_step
                        else:
                            current_servo_angle -= max_servo_step
                    else:
                        current_servo_angle = target_servo_angle

                    # Send the servo angle
                    send_servo_angle(int(current_servo_angle))

                    # Log data for plotting
                    time_stamps.append(current_time)
                    deviations.append(deviation_x)
                    proportional_adjustments.append(proportional)
                    derivative_adjustments.append(derivative)
                    servo_angles.append(current_servo_angle)

                # Display the current angle value
                window.blit(font.render(f"Servo Angle: {current_servo_angle:.2f}", True, (255, 255, 255)), (10, 10))

        manager.draw_ui(window)
        pygame.display.flip()

    ser.close()
    pygame.quit()
    
    # Plot the collected data after quitting
    plot_data()

if __name__ == "__main__":
    main()
