import serial
import pygame
import pygame_gui
import math

# Initialize Pygame
pygame.init()

# Set up the display with a smaller size
width, height = 1400, 700
window = pygame.display.set_mode((width, height))
pygame.display.set_caption("Camera and LED Visualization")

# Set up Pygame GUI manager
manager = pygame_gui.UIManager((width, height))

# Set up serial communication
port = 'COM3'  # Change this to your serial port
baudrate = 19200
ser = serial.Serial(port, baudrate, timeout=1)

# Predefined offsets
tilt_angle_cam1 = math.radians(3.68)  # Convert degrees to radians
tilt_angle_cam2 = math.radians(3.94)  # Convert degrees to radians
y_offset = 154

# Constants
baseline_cm = 7  # Distance between cameras in mm
camera_focal_length = 1331.43 # the focal length of the ir camera
fov_horizontal_deg = 33  # Horizontal field of view in degrees
fov_vertical_deg = 23  # Vertical field of view in degrees
resolution_width = 128  # Width of image in pixels
resolution_height = 96  # Height of image in pixels
distance_between_sources_mm = 200  # Distance between the two light sources in mm

# Calculate average field of view in radians
fov_horizontal_rad = math.radians(fov_horizontal_deg)
fov_vertical_rad = math.radians(fov_vertical_deg)
theta_fov = ((fov_horizontal_rad / resolution_width) + (fov_vertical_rad / resolution_height)) / 2

# Variable to store the last valid depth value
last_valid_depth_cm = None

def rotate_point(x, y, angle):
    """ Rotate a point around the origin (0, 0) by a given angle. """
    angle_rad = math.radians(angle)
    cos_angle = math.cos(angle_rad)
    sin_angle = math.sin(angle_rad)
    new_x = x * cos_angle - y * sin_angle
    new_y = x * sin_angle + y * cos_angle
    return new_x, new_y

def draw_distance_line(surface, start_x, start_y, end_x, end_y, angle):
    """ Draw a distance line and tilt it by the specified angle. """
    # Rotate the end point by the tilt angle
    rotated_end_x, rotated_end_y = rotate_point(end_x - start_x, end_y - start_y, angle)
    rotated_end_x += start_x
    rotated_end_y += start_y

    # Draw the line
    pygame.draw.line(surface, (255, 255, 0), (start_x, start_y), (rotated_end_x, rotated_end_y), 2)

    # Draw a small circle at the end of the line (where the LED is)
    pygame.draw.circle(surface, (255, 0, 0), (int(rotated_end_x), int(rotated_end_y)), 10)

    # Draw the angle label next to the line
    font = pygame.font.Font(None, 24)
    angle_text_surface = font.render(f"Angle: {angle}°", True, (255, 255, 255))
    surface.blit(angle_text_surface, (rotated_end_x + 10, rotated_end_y - 10))

def draw_camera(surface, x, y, camera_color, label):
    # Draw camera as a triangle
    points = [(x, y - 20), (x - 15, y + 10), (x + 15, y + 10)]
    pygame.draw.polygon(surface, camera_color, points)
    font = pygame.font.Font(None, 24)
    text_surface = font.render(label, True, camera_color)
    surface.blit(text_surface, (x - 15, y + 20))

def draw_grid(surface):
    grid_color = (200, 200, 200)
    scale = 50  # Grid scale in pixels

    # Draw vertical lines
    for x in range(0, width, scale):
        pygame.draw.line(surface, grid_color, (x, 0), (x, height))

    # Draw horizontal lines
    for y in range(0, height, scale):
        pygame.draw.line(surface, grid_color, (0, y), (width, y))

    # Draw X and Z axis labels
    font = pygame.font.Font(None, 36)
    x_label_surface = font.render("X Axis", True, (255, 255, 255))
    z_label_surface = font.render("Z Axis", True, (255, 255, 255))
    surface.blit(x_label_surface, (width // 2 + 10, height - 30))
    surface.blit(z_label_surface, (10, height // 2 - 20))

def read_serial_data():
    try:
        # Read the line from serial
        line = ser.readline().decode('utf-8').strip()
        
        # Check if line is not empty and split it by commas
        if line:
            # Split the data into a list
            data_list = line.split(',')
            
            # Check if the number of data points is as expected
            if len(data_list) == 16:
                # Convert the data points to integers
                return list(map(int, data_list))
            else:
                print("Warning: Unexpected number of data points.")
                return None
        else:
            print("Warning: No data received.")
            return None
    except ValueError as e:
        # Handle any conversion errors
        print(f"ValueError: {e}")
        return None
    except Exception as e:
        # Handle any other exceptions
        print(f"Exception: {e}")
        return None

def apply_tilt_correction(ix, iy, tilt_angle):
    new_x = ix
    new_y = iy * math.cos(tilt_angle) - ix * math.sin(tilt_angle)
    return int(new_x), int(new_y)

def calculate_depth(x1, x2):
    # Calculate distance between LED positions in pixels
    r = abs(x1 - x2)

    if r == 0:
        print("Warning: No distance between LED positions.")
        return None

    # # Calculate the angle alpha
    # alpha = (r * theta_fov) / 2

    # if alpha == 0:
    #     print("Warning: Alpha angle is zero.")
    #     return None

    # Calculate depth in mm
    try:
        # depth_mm = distance_between_sources_mm / math.tan(alpha)
        depth_cm = (camera_focal_length* baseline_cm)/r
        return depth_cm
    except ZeroDivisionError:
        print("ZeroDivisionError: Alpha angle is zero. Using last valid depth.")
        return last_valid_depth_cm

def send_servo_angle(angle):
    try:
        ser.write(f"{angle}\n".encode('utf-8'))
    except Exception as e:
        print(f"Failed to send angle to Arduino: {e}")

def main():
    global last_valid_depth_cm
    clock = pygame.time.Clock()
    running = True

    # Create a slider for the servo angle
    angle_slider = pygame_gui.elements.UIHorizontalSlider(
        relative_rect=pygame.Rect((10, 50, 200, 30)),
        start_value=100.0,  # Default value
        value_range=(0.0, 180.0),  # Min and max values
        manager=manager
    )

    # Initialize the angle variable
    angle = angle_slider.get_current_value()

    while running:
        time_delta = clock.tick(30) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            manager.process_events(event)

        manager.update(time_delta)
        window.fill((77, 77, 77))  # Fill the background

        # Draw grid and axis labels
        draw_grid(window)

        # Center Y position for the cameras
        center_y = height // 2

        # Draw Camera 1 and Camera 2
        cam1_x = width // 2 - baseline_cm*10 // 2
        cam2_x = width // 2 + baseline_cm*10 // 2
        draw_camera(window, cam1_x, center_y, (0, 0, 255), "Camera 1")
        draw_camera(window, cam2_x, center_y, (0, 255, 0), "Camera 2")

        # Read and process serial data
        data = read_serial_data()
        if data and len(data) >= 16:
            # Unpack the data for both cameras
            ix1, iy1, ix2, iy2, ix3, iy3, ix4, iy4 = data[:8]
            ix5, iy5, ix6, iy6, ix7, iy7, ix8, iy8 = data[8:]

            # Apply tilt correction for both cameras
            ix1, iy1 = apply_tilt_correction(ix1, iy1, tilt_angle_cam1)
            ix5, iy5 = apply_tilt_correction(ix5, iy5, tilt_angle_cam2)

            # Calculate depth
            depth_cm = calculate_depth(ix1, ix5)
            if depth_cm is not None:
                # Update the last valid depth if a new valid depth is calculated
                if depth_cm != last_valid_depth_cm:
                    last_valid_depth_cm = depth_cm

                # Display depth
                font = pygame.font.Font(None, 36)
                text_surface = font.render(f"Depth: {depth_cm:.2f} cm", True, (255, 255, 255))
                window.blit(text_surface, (width - 200, height - 50))
            else:
                # Display the last valid depth if the current calculation fails
                if last_valid_depth_cm is not None:
                    font = pygame.font.Font(None, 36)
                    text_surface = font.render(f"Depth: {last_valid_depth_cm:.2f} cm", True, (255, 255, 255))
                    window.blit(text_surface, (width - 200, height - 50))

            # Update the angle from the slider
            angle = int(angle_slider.get_current_value())
            send_servo_angle(angle)

            # Draw the tilted distance line and LED
            center_x = (cam1_x + cam2_x) // 2
            draw_distance_line(window, center_x, center_y, center_x, iy1, angle - 100)

        # Display the current angle value
        font = pygame.font.Font(None, 24)
        angle_text_surface = font.render(f"Angle: {angle}", True, (255, 255, 255))
        window.blit(angle_text_surface, (10, 90))

        manager.draw_ui(window)
        pygame.display.flip()

    ser.close()
    pygame.quit()

if __name__ == "__main__":
    main()
