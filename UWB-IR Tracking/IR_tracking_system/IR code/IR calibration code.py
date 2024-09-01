import serial
import pygame
import sys
import pygame_gui
import math

# Initialize Pygame
pygame.init()

# Set up the display
width, height = 1600, 800  # Increase width to fit both camera feeds
window = pygame.display.set_mode((width, height))
pygame.display.set_caption("IR Camera Feed with Grid and Offset Adjustment")

# Set up Pygame GUI manager
manager = pygame_gui.UIManager((width, height))

# Set up serial communication
port = 'COM3'  # Change this to your serial port
baudrate = 19200
ser = serial.Serial(port, baudrate, timeout=1)

# Function to draw the grid
def draw_grid(surface, grid_color=(200, 200, 200), cell_size=25):
    for x in range(0, surface.get_width(), cell_size):
        pygame.draw.line(surface, grid_color, (x, 0), (x, surface.get_height()))
    for y in range(0, surface.get_height(), cell_size):
        pygame.draw.line(surface, grid_color, (0, y), (surface.get_width(), y))

def read_serial_data():
    line = ser.readline().decode('utf-8').strip()
    if line:
        return list(map(int, line.split(',')))
    return None

def calculate_tilt_correction(ix1, iy1, ix2, iy2):
    if ix1 == ix2:
        return 0  # Prevent division by zero
    return math.atan((iy2 - iy1) / (ix2 - ix1))

def apply_tilt_correction(ix, iy, tilt_angle):
    new_x = ix
    new_y = iy * math.cos(tilt_angle) - ix * math.sin(tilt_angle)
    return int(new_x), int(new_y)

def calculate_y_offset(iy1, iy2):
    return (iy1 + iy2) / 2

def main():
    clock = pygame.time.Clock()
    running = True
    y_offset = 0

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

            # Flip y-coordinates
            iy1 = height - iy1
            iy2 = height - iy2
            iy3 = height - iy3
            iy4 = height - iy4
            iy5 = height - iy5
            iy6 = height - iy6
            iy7 = height - iy7
            iy8 = height - iy8

            # Calculate tilt corrections for each camera
            tilt_angle_cam1 = calculate_tilt_correction(ix1, iy1, ix2, iy2)
            tilt_angle_cam2 = calculate_tilt_correction(ix5, iy5, ix6, iy6)

            # Apply tilt correction
            ix1, iy1 = apply_tilt_correction(ix1, iy1, tilt_angle_cam1)
            ix2, iy2 = apply_tilt_correction(ix2, iy2, tilt_angle_cam1)
            ix3, iy3 = apply_tilt_correction(ix3, iy3, tilt_angle_cam1)
            ix4, iy4 = apply_tilt_correction(ix4, iy4, tilt_angle_cam1)

            ix5, iy5 = apply_tilt_correction(ix5, iy5, tilt_angle_cam2)
            ix6, iy6 = apply_tilt_correction(ix6, iy6, tilt_angle_cam2)
            ix7, iy7 = apply_tilt_correction(ix7, iy7, tilt_angle_cam2)
            ix8, iy8 = apply_tilt_correction(ix8, iy8, tilt_angle_cam2)

            # Calculate y offset to align both feeds
            y_offset_cam1 = calculate_y_offset(iy1, iy2)
            y_offset_cam2 = calculate_y_offset(iy5, iy6)
            y_offset = y_offset_cam1 - y_offset_cam2

            # Draw the ellipses for camera 1
            pygame.draw.circle(window, (255, 0, 0), (ix1, iy1), 10)
            pygame.draw.circle(window, (0, 255, 0), (ix2, iy2), 10)
            pygame.draw.circle(window, (0, 0, 255), (ix3, iy3), 10)
            pygame.draw.circle(window, (255, 255, 255), (ix4, iy4), 10)
            
            # Draw the ellipses for camera 2 with an offset
            offset_x = width // 2
            pygame.draw.circle(window, (255, 0, 0), (ix5 + offset_x, iy5 + y_offset), 10)
            pygame.draw.circle(window, (0, 255, 0), (ix6 + offset_x, iy6 + y_offset), 10)
            pygame.draw.circle(window, (0, 0, 255), (ix7 + offset_x, iy7 + y_offset), 10)
            pygame.draw.circle(window, (255, 255, 255), (ix8 + offset_x, iy8 + y_offset), 10)

            # Print the y offset value on the screen
            font = pygame.font.Font(None, 36)
            text_surface = font.render(f"Y Offset: {y_offset:.0f}", True, (255, 255, 255))
            window.blit(text_surface, (width - 200, height - 90))

            # Print the tilt angles on the screen
            tilt_cam1_surface = font.render(f"Tilt Cam1: {math.degrees(tilt_angle_cam1):.2f}°", True, (255, 255, 255))
            tilt_cam2_surface = font.render(f"Tilt Cam2: {math.degrees(tilt_angle_cam2):.2f}°", True, (255, 255, 255))
            window.blit(tilt_cam1_surface, (width - 200, height - 130))
            window.blit(tilt_cam2_surface, (width - 200, height - 170))

        manager.draw_ui(window)
        pygame.display.flip()

    ser.close()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
