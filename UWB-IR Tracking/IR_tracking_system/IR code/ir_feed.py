import serial
import pygame
import sys

# Initialize Pygame
pygame.init()

# Set up the display
width, height = 800, 800
window = pygame.display.set_mode((width, height))

# Set up serial communication
port = 'COM3'  # Change this to your serial port
baudrate = 19200
ser = serial.Serial(port, baudrate, timeout=1)

def read_serial_data():
    line = ser.readline().decode('utf-8').strip()
    if line:
        return list(map(int, line.split(',')))
    return None

def main():
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        window.fill((77, 77, 77))  # Fill the background
        
        data = read_serial_data()
        if data and len(data) >= 8:
            # Unpack the data
            xx, yy, ww, zz, xxx, yyy, www, zzz = data
            
            # Draw the ellipses
            pygame.draw.circle(window, (255, 0, 0), (xx, yy), 20)
            pygame.draw.circle(window, (0, 255, 0), (ww, zz), 20)
            pygame.draw.circle(window, (0, 0, 255), (xxx, yyy), 20)
            pygame.draw.circle(window, (255, 255, 255), (www, zzz), 20)
        
        pygame.display.flip()
    
    ser.close()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
